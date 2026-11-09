use tokio::sync::watch;
use vda5050_vehicle_simulator::{config, mqtt_handler};

#[tokio::main]
async fn main() {
    let config = config::get_config();
    let (shutdown, stop) = watch::channel(false);
    let mut vehicles = Vec::new();

    for index in 0..config.settings.robot_count {
        let mut vehicle = config.clone();
        if config.settings.robot_count > 1 {
            vehicle.vehicle.serial_number =
                format!("{}{}", config.vehicle.serial_number, index + 1);
        }
        vehicles.push(tokio::spawn(mqtt_handler::run_vehicle(
            vehicle,
            stop.clone(),
        )));
    }

    if let Err(error) = tokio::signal::ctrl_c().await {
        eprintln!("Could not wait for Ctrl-C: {error}");
    }
    let _ = shutdown.send(true);
    for vehicle in vehicles {
        if let Err(error) = vehicle.await {
            eprintln!("Vehicle task failed: {error}");
        }
    }
}
