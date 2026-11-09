mod common;

use std::time::Duration;

use common::*;
use paho_mqtt as mqtt;
use serde_json::Value;
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::watch;
use tokio::time::timeout;
use uuid::Uuid;
use vda5050_vehicle_simulator::mqtt_handler::run_vehicle;
use vda5050_vehicle_simulator::protocol::vda_2_0_0::vda5050_2_0_0_instant_actions::InstantActions;

const DEADLINE: Duration = Duration::from_secs(12);

async fn client(
    broker: &str,
) -> (
    mqtt::AsyncClient,
    mqtt::AsyncReceiver<Option<mqtt::Message>>,
) {
    let options = mqtt::CreateOptionsBuilder::new()
        .server_uri(format!("tcp://{broker}"))
        .client_id(Uuid::new_v4().to_string())
        .finalize();
    let mut client = mqtt::AsyncClient::new(options).unwrap();
    let inbox = client.get_stream(None);
    timeout(
        DEADLINE,
        client.connect(
            mqtt::ConnectOptionsBuilder::with_mqtt_version(mqtt::MQTT_VERSION_5).finalize(),
        ),
    )
    .await
    .unwrap()
    .unwrap();
    (client, inbox)
}

async fn receive(
    inbox: &mqtt::AsyncReceiver<Option<mqtt::Message>>,
    topic: &str,
    matches: impl Fn(&Value) -> bool,
) -> (mqtt::Message, Value) {
    timeout(DEADLINE, async {
        loop {
            let message = inbox
                .recv()
                .await
                .unwrap()
                .expect("test client disconnected");
            if message.topic() == topic {
                let payload: Value = serde_json::from_slice(message.payload()).unwrap();
                if matches(&payload) {
                    return (message, payload);
                }
            }
        }
    })
    .await
    .unwrap_or_else(|_| panic!("timed out waiting for {topic}"))
}

async fn publish(client: &mqtt::AsyncClient, topic: &str, command: &InstantActions) {
    timeout(
        DEADLINE,
        client.publish(mqtt::Message::new(
            topic,
            serde_json::to_vec(command).unwrap(),
            mqtt::QOS_1,
        )),
    )
    .await
    .unwrap()
    .unwrap();
}

fn finished(state: &Value, id: &str) -> bool {
    state["actionStates"]
        .as_array()
        .unwrap()
        .iter()
        .any(|action| action["actionId"] == id && action["actionStatus"] == "FINISHED")
}

fn verify_pick(state: &Value) {
    let loads = state["loads"].as_array().unwrap();
    assert_eq!(loads.len(), 1);
    assert_eq!(loads[0]["loadId"], "box-1");
    let results: Vec<_> = state["actionStates"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|a| a["actionId"] == "pick-1")
        .collect();
    assert_eq!(results.len(), 1);
    assert_eq!(results[0]["actionStatus"], "FINISHED");
    assert_eq!(results[0]["resultDescription"], "Picked load box-1");
}

#[tokio::test]
#[ignore = "requires an MQTT 5 broker; set VDA5050_TEST_MQTT_ADDR to host:port"]
async fn lost_pick_result_is_recovered_after_mqtt_reconnect_without_second_pick() {
    let broker = std::env::var("VDA5050_TEST_MQTT_ADDR").expect("set VDA5050_TEST_MQTT_ADDR");
    let mut config = make_config();
    config.vehicle.serial_number = format!("recovery-{}", Uuid::new_v4());
    let base = format!(
        "uagv/v2/{}/{}",
        config.vehicle.manufacturer, config.vehicle.serial_number
    );
    let connection = format!("{base}/connection");
    let state = format!("{base}/state");
    let commands = format!("{base}/instantActions");

    // Only break the vehicle's TCP link. The broker and both test clients
    // stay up so the broker must generate the vehicle's retained Will.
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    config.mqtt_broker.port = listener.local_addr().unwrap().port().to_string();
    let (link, mut gate) = watch::channel(true);
    let broker_target = broker.clone();
    let proxy = tokio::spawn(async move {
        loop {
            let (mut downstream, _) = listener.accept().await.unwrap();
            if !*gate.borrow_and_update() {
                continue;
            }
            let mut upstream = TcpStream::connect(&broker_target).await.unwrap();
            tokio::select! {
                _ = tokio::io::copy_bidirectional(&mut downstream, &mut upstream) => {}
                _ = gate.changed() => {}
            }
        }
    });

    let (observer, observer_inbox) = client(&broker).await;
    observer
        .subscribe_many_same_qos(&[&connection, &state], mqtt::QOS_1)
        .await
        .unwrap();
    let (controller, controller_inbox) = client(&broker).await;
    let (shutdown, stop) = watch::channel(false);
    let vehicle = tokio::spawn(run_vehicle(config, stop));
    let (_, first_online) = receive(&observer_inbox, &connection, |c| {
        c["connectionState"] == "ONLINE"
    })
    .await;
    let mut pick = make_instant_actions(vec![make_pick_action("pick-1", "box-1")]);
    pick.serial_number = base.rsplit('/').next().unwrap().into();

    // The controller cannot see the first result. A separate observer proves
    // the physical effect was already applied before the link is cut.
    publish(&controller, &commands, &pick).await;
    let (_, first_result) = receive(&observer_inbox, &state, |s| finished(s, "pick-1")).await;
    verify_pick(&first_result);
    assert!(controller_inbox.try_recv().is_err());

    link.send(false).unwrap();
    let (_, broken) = receive(&observer_inbox, &connection, |c| {
        c["connectionState"] == "CONNECTIONBROKEN"
    })
    .await;
    assert!(broken["headerId"].as_u64() > first_online["headerId"].as_u64());
    controller
        .subscribe(&connection, mqtt::QOS_1)
        .await
        .unwrap();
    let (retained, _) = receive(&controller_inbox, &connection, |c| {
        c["connectionState"] == "CONNECTIONBROKEN"
    })
    .await;
    assert!(retained.retained());

    link.send(true).unwrap();
    let (_, reconnected) = receive(&controller_inbox, &connection, |c| {
        c["connectionState"] == "ONLINE"
    })
    .await;
    assert!(reconnected["headerId"].as_u64() > broken["headerId"].as_u64());
    receive(&observer_inbox, &connection, |c| {
        c["headerId"] == reconnected["headerId"]
    })
    .await;
    let (_, restored) = receive(&observer_inbox, &state, |s| {
        s["headerId"].as_u64() > first_result["headerId"].as_u64() && finished(s, "pick-1")
    })
    .await;
    verify_pick(&restored);

    controller.subscribe(&state, mqtt::QOS_1).await.unwrap();
    pick.header_id += 1;
    publish(&controller, &commands, &pick).await;
    let (_, retry_result) = receive(&controller_inbox, &state, |s| finished(s, "pick-1")).await;
    verify_pick(&retry_result);

    // A stateRequest received after the retransmit is a barrier: the
    // simulator has processed pending actions, not just echoed a snapshot.
    let mut query = pick;
    query.header_id += 1;
    query.actions = vec![make_action("stateRequest", "verify-after-retry")];
    publish(&controller, &commands, &query).await;
    let (_, checked) = receive(&controller_inbox, &state, |s| {
        finished(s, "verify-after-retry")
    })
    .await;
    verify_pick(&checked);

    shutdown.send(true).unwrap();
    let (_, offline) = receive(&controller_inbox, &connection, |c| {
        c["connectionState"] == "OFFLINE"
    })
    .await;
    assert!(offline["headerId"].as_u64() > reconnected["headerId"].as_u64());
    timeout(DEADLINE, vehicle).await.unwrap().unwrap();
    proxy.abort();
}
