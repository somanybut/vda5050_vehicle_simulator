mod action_handler;
mod instant_actions_manager;
mod navigator;
mod order_manager;

pub use action_handler::ActionHandler;
pub use instant_actions_manager::InstantActionsManager;
pub use navigator::Navigator;
pub use order_manager::OrderManager;

use crate::config;
use crate::protocol::vda5050_common::AgvPosition;
use crate::protocol::vda_2_0_0::vda5050_2_0_0_instant_actions::InstantActions;
use crate::protocol::vda_2_0_0::vda5050_2_0_0_order::Order;
use crate::protocol::vda_2_0_0::vda5050_2_0_0_state::{
    ActionStatus, BatteryState, EStop, Error, ErrorLevel, ErrorReference, OperatingMode,
    SafetyState, State,
};
use crate::protocol::vda_2_0_0::vda5050_2_0_0_visualization::Visualization;
use crate::utils;

/// Top-level vehicle simulator. Acts as a thin facade that owns the protocol
/// payloads and delegates behaviour to focused collaborators:
///
/// - [`ActionHandler`] runs `initPosition` / `pick` / `drop` actions.
/// - [`OrderManager`] tracks the current order.
/// - [`InstantActionsManager`] tracks actions received on the
///   `instantActions` topic.
/// - [`Navigator`] advances the AGV one tick at a time.
pub struct VehicleSimulator {
    pub state: State,
    pub visualization: Visualization,

    action_handler: ActionHandler,
    order_manager: OrderManager,
    instant_actions_manager: InstantActionsManager,
    navigator: Navigator,
}

impl VehicleSimulator {
    pub fn new(config: config::Config) -> Self {
        let (state, agv_position) = build_initial_state(&config);
        let visualization = build_initial_visualization(&config, &agv_position);

        Self {
            state,
            visualization,
            action_handler: ActionHandler::new(config.settings.action_time),
            order_manager: OrderManager::new(),
            instant_actions_manager: InstantActionsManager::new(),
            navigator: Navigator::new(config.settings.speed),
        }
    }

    // -- Order / instant action intake -------------------------------------

    pub fn process_order(&mut self, order_request: Order) {
        let instant_actions = &self.instant_actions_manager;
        self.order_manager
            .process_order_with(order_request, &mut self.state, |id| {
                instant_actions.contains_id(id)
            });
    }

    pub fn accept_instant_actions(&mut self, request: InstantActions) {
        self.instant_actions_manager
            .accept(request, &mut self.state);
    }

    pub fn is_vehicle_ready_for_new_order(&self) -> bool {
        OrderManager::is_vehicle_ready_for_new_order(&self.state)
    }

    // -- Simulation tick ---------------------------------------------------

    /// Advance one tick. Returns whether an action, movement transition or
    /// node arrival warrants an immediate state report.
    pub fn update_state(&mut self) -> bool {
        let mut changed = self.run_pending_instant_actions();
        if self.action_handler.is_busy() || !self.order_manager.has_order() {
            return changed;
        }

        if let Some(action) = self
            .order_manager
            .next_node_action(&self.state)
            .or_else(|| self.order_manager.next_edge_action(&self.state))
        {
            self.action_handler.mark_started();
            self.action_handler
                .execute(&action, &mut self.state, &mut self.visualization);
            return true;
        }

        let previous = (
            self.state.last_node_sequence_id,
            self.state.node_states.len(),
            self.state.driving,
        );
        self.navigator
            .step(&mut self.state, &mut self.visualization);
        self.order_manager.update_base_request(&mut self.state);
        changed |= previous
            != (
                self.state.last_node_sequence_id,
                self.state.node_states.len(),
                self.state.driving,
            );
        changed
    }

    pub fn process_instant_actions(&mut self) {
        self.run_pending_instant_actions();
    }

    fn run_pending_instant_actions(&mut self) -> bool {
        let mut changed = false;
        for action in self.instant_actions_manager.pending(&self.state) {
            match action.action_type.as_str() {
                "cancelOrder" => self.cancel_order(&action.action_id),
                "stateRequest" => self.finish_instant_action(&action.action_id, Ok(())),
                _ if self.action_handler.is_busy() => continue,
                _ => self
                    .action_handler
                    .execute(&action, &mut self.state, &mut self.visualization),
            }
            self.instant_actions_manager
                .record_result(&action.action_id, &self.state);
            changed = true;
        }
        changed
    }

    fn cancel_order(&mut self, action_id: &str) {
        if self.order_manager.cancel(&mut self.state) {
            self.action_handler.clear_busy();
            self.visualization.velocity = None;
            self.finish_instant_action(action_id, Ok(()));
        } else {
            let reason = "No active order to cancel";
            self.state
                .errors
                .retain(|e| e.error_type != "noOrderToCancel");
            self.state.errors.push(Error {
                error_type: "noOrderToCancel".into(),
                error_level: ErrorLevel::Warning,
                error_description: Some(reason.into()),
                error_references: vec![ErrorReference {
                    reference_key: "actionId".into(),
                    reference_value: action_id.into(),
                }],
            });
            self.finish_instant_action(action_id, Err(reason));
        }
    }

    fn finish_instant_action(&mut self, id: &str, result: Result<(), &str>) {
        if let Some(action) = self
            .state
            .action_states
            .iter_mut()
            .find(|a| a.action_id == id && a.action_status == ActionStatus::Waiting)
        {
            match result {
                Ok(()) => action.action_status = ActionStatus::Finished,
                Err(reason) => {
                    action.action_status = ActionStatus::Failed;
                    action.result_description = Some(reason.into());
                }
            }
        }
    }
}

fn build_initial_state(config: &config::Config) -> (State, AgvPosition) {
    // Spawn each robot inside a small random patch around the origin so a
    // multi-robot run does not stack every AGV on the same point.
    let random_x = rand::random::<f32>() * 5.0 - 2.5;
    let random_y = rand::random::<f32>() * 5.0 - 2.5;

    let agv_position = AgvPosition {
        x: random_x,
        y: random_y,
        position_initialized: false,
        theta: 0.0,
        map_id: config.settings.map_id.clone(),
        deviation_range: None,
        map_description: None,
        localization_score: None,
    };

    let state = State {
        header_id: 0,
        timestamp: utils::get_timestamp(),
        version: config.vehicle.vda_full_version.clone(),
        manufacturer: config.vehicle.manufacturer.clone(),
        serial_number: config.vehicle.serial_number.clone(),
        driving: false,
        distance_since_last_node: None,
        operating_mode: OperatingMode::Automatic,
        node_states: vec![],
        edge_states: vec![],
        last_node_id: String::new(),
        order_id: String::new(),
        order_update_id: 0,
        last_node_sequence_id: 0,
        action_states: vec![],
        information: vec![],
        loads: vec![],
        errors: vec![],
        battery_state: BatteryState {
            battery_charge: 100.0,
            battery_voltage: None,
            battery_health: None,
            charging: false,
            reach: None,
        },
        safety_state: SafetyState {
            e_stop: EStop::None,
            field_violation: false,
        },
        paused: None,
        new_base_request: None,
        agv_position: Some(agv_position.clone()),
        velocity: None,
        zone_set_id: None,
    };

    (state, agv_position)
}

fn build_initial_visualization(
    config: &config::Config,
    agv_position: &AgvPosition,
) -> Visualization {
    Visualization {
        header_id: 0,
        timestamp: utils::get_timestamp(),
        version: config.vehicle.vda_full_version.clone(),
        manufacturer: config.vehicle.manufacturer.clone(),
        serial_number: config.vehicle.serial_number.clone(),
        agv_position: Some(agv_position.clone()),
        velocity: None,
    }
}
