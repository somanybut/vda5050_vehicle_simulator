use paho_mqtt as mqtt;
use uuid::Uuid;

use crate::config::MqttBrokerConfig;

pub fn mqtt_create_opts(config: &MqttBrokerConfig) -> mqtt::CreateOptions {
    mqtt::CreateOptionsBuilder::new()
        .server_uri(format!("tcp://{}:{}", config.host, config.port))
        .client_id(Uuid::new_v4().to_string())
        .finalize()
}

pub fn generate_vda_mqtt_base_topic(
    vda_interface: &str,
    vda_version: &str,
    manufacturer: &str,
    serial_number: &str,
) -> String {
    format!("{vda_interface}/{vda_version}/{manufacturer}/{serial_number}")
}
