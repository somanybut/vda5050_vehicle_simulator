# VDA5050 Robot Simulator

This project is a VDA5050-compliant robot simulator written in Rust. It simulates the behavior of automated guided vehicles (AGVs) following the VDA5050 standard, using an MQTT broker to communicate. The simulator is configurable via a TOML file, and it supports basic customization like vehicle configuration, state update frequency, and more. Also simulator supoorts create multiple simulator at the same time.

## Features

- Simulates AGVs using the VDA5050 standard.
- Communicates with a broker via MQTT.
- Configurable vehicle, map, and simulator settings.
- Supports visualization updates and actions.
- Supports trajectory

## Configuration

You can configure the simulator using a `config.toml` file. Below is an example configuration:

```toml
[mqtt_broker]
host = "localhost"                  # MQTT broker address
port = "1883"                        # MQTT broker port
vda_interface = "uagv"               # VDA interface to use
username = ""                   # MQTT broker username (optional)
password = ""                   # MQTT broker password (optional)

[vehicle]
serial_number = "s1"                 # Serial number of the AGV
manufacturer = "rikeb"               # Manufacturer name
vda_version = "v2"                   # VDA standard version
vda_full_version = "2.0.0"           # Full VDA version

[settings]
map_id = "webots"                    # Map identifier
state_frequency = 1                  # Frequency for state updates (in Hz)
visualization_frequency = 1          # Frequency for visualization updates (in Hz)
action_time = 1.0                    # Action execution time (in seconds)
robot_count = 1                      # Number of robots to simulate
speed = 0.05                         # Robot speed in meters per second
```

### MQTT Broker Section
- **host**: The address of the MQTT broker (default: localhost).
- **port**: The port of the MQTT broker (default: 1883).
- **vda_interface**: The type of VDA interface used.
- **username**: The username for authenticating with the MQTT broker. Leave empty or omit if the broker requires no authentication.
- **password**: The password for authenticating with the MQTT broker. Leave empty or omit if the broker requires no authentication.

### Vehicle Section

- **serial_number**: The serial number of the simulated robot.
- **manufacturer**: The name of the robot manufacturer.
- **vda_version**: The version of the VDA standard being used.
- **vda_full_version**: The full version number of the VDA standard.

If more than one robot is specified (`robot_count` below), an integer will be appended to each robot's `serial_number`, starting with 1. This will *not* occur if `robot_count = 1`.

The following, for instance, will produce robots with the serial numbers "my_robot1", "my_robot2" and "my_robot3":

```toml
[vehicle]
serial_number = "my_robot"

[settings]
robot_count = 3
```

### Settings Section

- **map_id**: Identifier for the map used in the simulation (e.g., "webots").
- **state_frequency**: Frequency of state updates in Hertz (Hz). Determines how often the robot sends its current state to the broker.
- **visualization_frequency**: Frequency of visualization updates in Hertz (Hz). Controls how often the simulator will send data for visualization purposes.
- **action_time**: The time it takes to complete an action (in seconds). This controls how long each task or action will take for the robot to execute.
- **robot_count**: The number of robots being simulated. This allows you to simulate multiple robots within the same environment.
- **speed**: The speed of the robot in meters per second, which dictates how fast the robot will move in the simulation.

## Docker 

The repository ships a `Dockerfile` and two Compose files that let you build and run the simulator without installing Rust locally.

| File | Purpose |
|---|---|
| `docker-compose.yml` | Base file. Uses a pre-built simulator image and includes a local Mosquitto MQTT broker. |
| `docker-compose.override.yml` | Development override. Adds a build context so the image is compiled from source. Applied automatically by Docker Compose when both files are present. |

Run `docker compose up --build` to start the broker and build and start the simulator. The simulator connects to the broker through the Compose service name `mqtt`. The provided `mosquitto.conf` allows anonymous access and is intended only for isolated local testing.

## Requirements

- **Rust**: Ensure that Rust is installed. You can follow the installation instructions [here](https://www.rust-lang.org/tools/install) if you don’t have it installed.
  
- **MQTT Broker**: Native runs require an MQTT broker such as [Mosquitto](https://mosquitto.org/). The default Compose setup starts one automatically.

## Getting Started

To set up and run the VDA5050 robot simulator, follow these steps:

1. Build the project using Cargo, Rust’s package manager:

    ```bash
    cargo build --release
    ```

2. Configure the simulator by modifying the `config.toml` file. You can adjust parameters such as the MQTT broker address, vehicle details, and simulation settings.

3. Run the simulator:

    ```bash
    cargo run --release
    ```

## Usage

Once the simulator is running, it will start sending messages to the MQTT broker according to the configuration in the `config.toml` file. You can monitor the robot's state, actions, and other telemetry by subscribing to the relevant MQTT topics using a client or tool such as [MQTT Explorer](https://mqtt-explorer.com/).

To visualize the robot's status and actions, you can adjust the `visualization_frequency` setting in `config.toml`.

## Command retries and MQTT recovery

- Retry an instant command with the same `actionId` and action contents. The simulator reports the original terminal result and does not repeat the load change. A changed MQTT header or timestamp does not create a new command. `stateRequest` reports the current state.
- Retry the current order with the same `orderId`, `orderUpdateId` and route. Extend an order using a higher update ID and the last released node as the first node of the update. The simulator keeps the released route and its action results, and replaces the remaining horizon.
- `cancelOrder` stops this model immediately and fails unfinished actions belonging to that order. A new cancellation with no active order reports `FAILED` and `noOrderToCancel`.
- The broker retains `ONLINE`, the `CONNECTIONBROKEN` Last Will, and the `OFFLINE` report sent on Ctrl-C. Reconnect restores the subscriptions and publishes the current vehicle state. Command deduplication and results are held in memory across MQTT reconnects; a process restart resets them. The wire state reports order results only for the current order.

Run the broker regression against a local MQTT 5 broker (the test cuts only the simulator's TCP connection and leaves the broker running):

```sh
VDA5050_TEST_MQTT_ADDR=127.0.0.1:1883 cargo test --locked --test mqtt_recovery_tests -- --ignored
```

## License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for more details.


## Libraries Used

This simulator was built using the [`vda5050-types-rs`](https://github.com/kKdH/vda5050-types-rs) library for handling VDA5050 standard data types and message structures.
