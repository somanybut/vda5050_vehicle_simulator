use std::error::Error;
use std::time::Duration;

use paho_mqtt as mqtt;
use tokio::sync::watch;
use tokio::time::{self, Instant, Interval, MissedTickBehavior};

use crate::config::Config;
use crate::mqtt_utils;
use crate::protocol::vda_2_0_0::vda5050_2_0_0_connection::{Connection, ConnectionState};
use crate::protocol::vda_2_0_0::vda5050_2_0_0_instant_actions::InstantActions;
use crate::protocol::vda_2_0_0::vda5050_2_0_0_order::Order;
use crate::utils::get_timestamp;
use crate::vehicle_simulator::VehicleSimulator;

type SessionResult = Result<(), Box<dyn Error + Send + Sync>>;
const IO_TIMEOUT: Duration = Duration::from_secs(5);

struct Topics {
    order: String,
    instant_actions: String,
    connection: String,
    state: String,
    visualization: String,
}

impl Topics {
    fn new(config: &Config) -> Self {
        let vehicle = &config.vehicle;
        let base = mqtt_utils::generate_vda_mqtt_base_topic(
            &config.mqtt_broker.vda_interface,
            &vehicle.vda_version,
            &vehicle.manufacturer,
            &vehicle.serial_number,
        );
        Self {
            order: format!("{base}/order"),
            instant_actions: format!("{base}/instantActions"),
            connection: format!("{base}/connection"),
            state: format!("{base}/state"),
            visualization: format!("{base}/visualization"),
        }
    }
}

pub async fn run_vehicle(config: Config, mut shutdown: watch::Receiver<bool>) {
    let topics = Topics::new(&config);
    let mut simulator = VehicleSimulator::new(config.clone());
    let mut ticks = time::interval(Duration::from_millis(50));
    ticks.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let mut connection_id = 0;

    while !*shutdown.borrow() {
        let result = run_session(
            &config,
            &topics,
            &mut simulator,
            &mut ticks,
            &mut shutdown,
            connection_id,
        )
        .await;
        match result {
            Ok(()) => return,
            Err(error) => eprintln!(
                "{} MQTT session ended: {error}",
                config.vehicle.serial_number
            ),
        }
        // Each session reserves the next connection header ID for its Will.
        connection_id += 2;
        let retry = time::sleep(Duration::from_secs(1));
        tokio::pin!(retry);
        loop {
            tokio::select! {
                _ = &mut retry => break,
                _ = shutdown.changed() => return,
                _ = ticks.tick() => { simulator.update_state(); }
            }
        }
    }
}

async fn run_session(
    config: &Config,
    topics: &Topics,
    simulator: &mut VehicleSimulator,
    ticks: &mut Interval,
    shutdown: &mut watch::Receiver<bool>,
    online_id: u64,
) -> SessionResult {
    let mut client = mqtt::AsyncClient::new(mqtt_utils::mqtt_create_opts(&config.mqtt_broker))?;
    // Avoid silently dropping QoS 1 commands or the disconnect notification
    // when a burst fills Paho's stream buffer.
    let incoming = client.get_stream(None);
    let will = connection_message(
        config,
        topics,
        online_id + 1,
        ConnectionState::ConnectionBroken,
    )?;
    let connect = {
        let mut options = mqtt::ConnectOptionsBuilder::with_mqtt_version(mqtt::MQTT_VERSION_5);
        options
            .clean_start(true)
            .keep_alive_interval(Duration::from_secs(5))
            .connect_timeout(IO_TIMEOUT)
            .will_message(will);
        if let (Some(username), Some(password)) =
            (&config.mqtt_broker.username, &config.mqtt_broker.password)
        {
            options.user_name(username).password(password);
        }
        client.connect(options.finalize())
    };
    let establish = async {
        time::timeout(IO_TIMEOUT, connect).await??;
        let subscribed = time::timeout(
            IO_TIMEOUT,
            client.subscribe_many_same_qos(&[&topics.order, &topics.instant_actions], mqtt::QOS_1),
        )
        .await??;
        if !subscribed
            .subscribe_many_response()
            .is_some_and(|codes| codes.len() == 2 && codes.iter().all(|code| *code < 128))
        {
            return Err("Broker rejected a vehicle subscription".into());
        }
        send(
            &client,
            connection_message(config, topics, online_id, ConnectionState::Online)?,
        )
        .await
    };
    tokio::pin!(establish);
    loop {
        tokio::select! {
            result = &mut establish => { result?; break; }
            _ = shutdown.changed() => {
                if client.is_connected() {
                    publish_offline(&client, config, topics, online_id + 1).await?;
                }
                return Ok(());
            }
            _ = ticks.tick() => { simulator.update_state(); }
        }
    }

    publish_state(&client, topics, simulator).await?;
    let state_period = Duration::from_secs_f64(1.0 / config.settings.state_frequency.max(1) as f64);
    let viz_period =
        Duration::from_secs_f64(1.0 / config.settings.visualization_frequency.max(1) as f64);
    let mut next_state = Instant::now() + state_period;
    let mut next_viz = Instant::now() + viz_period;
    loop {
        tokio::select! {
            _ = shutdown.changed() => {
                return publish_offline(&client, config, topics, online_id + 1).await;
            }
            message = incoming.recv() => {
                let Some(message) = message? else {
                    return Err("MQTT connection lost".into());
                };
                if handle_message(&message, topics, simulator) {
                    // A retransmit can recover the original terminal state immediately.
                    publish_state(&client, topics, simulator).await?;
                    next_state = Instant::now() + state_period;
                }
            }
            _ = ticks.tick() => {
                let changed = simulator.update_state();
                if changed || Instant::now() >= next_state {
                    publish_state(&client, topics, simulator).await?;
                    next_state = Instant::now() + state_period;
                }
                if Instant::now() >= next_viz {
                    simulator.visualization.header_id += 1;
                    simulator.visualization.timestamp = get_timestamp();
                    let payload = serde_json::to_vec(&simulator.visualization)?;
                    send(&client, mqtt::Message::new(&topics.visualization, payload, mqtt::QOS_1)).await?;
                    next_viz = Instant::now() + viz_period;
                }
            }
        }
    }
}

fn handle_message(
    message: &mqtt::Message,
    topics: &Topics,
    simulator: &mut VehicleSimulator,
) -> bool {
    if message.retained() {
        return false;
    }
    let result = if message.topic() == topics.order {
        serde_json::from_slice::<Order>(message.payload())
            .map(|order| simulator.process_order(order))
    } else if message.topic() == topics.instant_actions {
        serde_json::from_slice::<InstantActions>(message.payload())
            .map(|actions| simulator.accept_instant_actions(actions))
    } else {
        return false;
    };
    if let Err(error) = result {
        eprintln!("Invalid {} message: {error}", message.topic());
        return false;
    }
    true
}

fn connection_message(
    config: &Config,
    topics: &Topics,
    header_id: u64,
    connection_state: ConnectionState,
) -> serde_json::Result<mqtt::Message> {
    let connection = Connection {
        header_id,
        timestamp: get_timestamp(),
        version: config.vehicle.vda_full_version.clone(),
        manufacturer: config.vehicle.manufacturer.clone(),
        serial_number: config.vehicle.serial_number.clone(),
        connection_state,
    };
    Ok(mqtt::Message::new_retained(
        &topics.connection,
        serde_json::to_vec(&connection)?,
        mqtt::QOS_1,
    ))
}

async fn publish_offline(
    client: &mqtt::AsyncClient,
    config: &Config,
    topics: &Topics,
    id: u64,
) -> SessionResult {
    send(
        client,
        connection_message(config, topics, id, ConnectionState::Offline)?,
    )
    .await?;
    time::timeout(IO_TIMEOUT, client.disconnect(None)).await??;
    Ok(())
}

async fn publish_state(
    client: &mqtt::AsyncClient,
    topics: &Topics,
    simulator: &mut VehicleSimulator,
) -> SessionResult {
    simulator.state.header_id += 1;
    simulator.state.timestamp = get_timestamp();
    let payload = serde_json::to_vec(&simulator.state)?;
    send(
        client,
        mqtt::Message::new(&topics.state, payload, mqtt::QOS_1),
    )
    .await
}

async fn send(client: &mqtt::AsyncClient, message: mqtt::Message) -> SessionResult {
    time::timeout(IO_TIMEOUT, client.publish(message)).await??;
    Ok(())
}
