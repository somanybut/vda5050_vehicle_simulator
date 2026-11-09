mod common;

use common::*;
use vda5050_vehicle_simulator::protocol::vda_2_0_0::vda5050_2_0_0_state::{
    ActionState, ActionStatus,
};
use vda5050_vehicle_simulator::vehicle_simulator::VehicleSimulator;

fn localized() -> VehicleSimulator {
    let mut sim = VehicleSimulator::new(make_config());
    sim.accept_instant_actions(make_instant_actions(vec![make_init_position_action(
        "init", 0.0, 0.0, 0.0, "A",
    )]));
    sim.update_state();
    sim
}

fn action<'a>(sim: &'a VehicleSimulator, id: &str) -> &'a ActionState {
    let mut matches = sim.state.action_states.iter().filter(|s| s.action_id == id);
    let state = matches.next().expect("expected a state for this action");
    assert!(matches.next().is_none(), "duplicate action state: {id}");
    state
}

fn tick_until(sim: &mut VehicleSimulator, predicate: impl Fn(&VehicleSimulator) -> bool) {
    for _ in 0..100 {
        if predicate(sim) {
            return;
        }
        sim.update_state();
    }
    panic!("vehicle did not reach the expected state");
}

#[test]
fn lost_pick_result_retries_return_original_result_without_picking_again() {
    let mut sim = localized();
    let pick = make_pick_action("take-1", "box-1");
    let mut retry = make_instant_actions(vec![pick.clone()]);
    sim.accept_instant_actions(retry.clone());
    retry.header_id += 1;
    sim.accept_instant_actions(retry.clone());
    sim.update_state();
    assert_eq!(action(&sim, "take-1").action_status, ActionStatus::Finished);
    let original_result = action(&sim, "take-1").result_description.clone();
    assert_eq!(sim.state.loads.len(), 1);

    // Accepting a new order removes completed instant actions from visible state.
    sim.process_order(make_order(
        "order-1",
        0,
        vec![make_node("A", 0, 0.0, 0.0, true, vec![])],
        vec![],
    ));
    assert!(!sim
        .state
        .action_states
        .iter()
        .any(|s| s.action_id == "take-1"));

    retry.header_id += 1;
    sim.accept_instant_actions(retry);
    sim.update_state();
    assert_eq!(sim.state.loads.len(), 1);
    assert_eq!(sim.state.loads[0].load_id.as_deref(), Some("box-1"));
    assert_eq!(action(&sim, "take-1").action_status, ActionStatus::Finished);
    assert_eq!(action(&sim, "take-1").result_description, original_result);

    sim.accept_instant_actions(make_instant_actions(vec![make_pick_action(
        "take-1",
        "different-box",
    )]));
    sim.update_state();
    assert_eq!(sim.state.loads.len(), 1);
    assert!(sim
        .state
        .errors
        .iter()
        .any(|e| e.error_type == "instantActionError"));
}

#[test]
fn lost_order_pick_result_retries_preserve_result_and_retired_order_cannot_reexecute() {
    let mut sim = localized();
    let mut order = make_order(
        "order-1",
        0,
        vec![make_node(
            "A",
            0,
            0.0,
            0.0,
            true,
            vec![make_pick_action("take-1", "box-1")],
        )],
        vec![],
    );
    sim.process_order(order.clone());
    tick_until(&mut sim, |s| s.state.node_states.is_empty());

    order.header_id += 1;
    sim.process_order(order.clone());
    sim.update_state();
    assert_eq!(sim.state.loads.len(), 1);
    assert_eq!(action(&sim, "take-1").action_status, ActionStatus::Finished);

    sim.process_order(make_order(
        "order-2",
        0,
        vec![make_node("A", 0, 0.0, 0.0, true, vec![])],
        vec![],
    ));
    sim.update_state();
    sim.process_order(order);
    sim.update_state();
    assert_eq!(sim.state.order_id, "order-2");
    assert_eq!(sim.state.loads.len(), 1);
}

#[test]
fn continuation_during_motion_keeps_base_and_replaces_horizon() {
    let mut sim = localized();
    let stitch = make_node(
        "B",
        2,
        3.0,
        0.0,
        true,
        vec![make_pick_action("take-1", "box-1")],
    );
    sim.process_order(make_order(
        "order",
        0,
        vec![
            make_node("A", 0, 0.0, 0.0, true, vec![]),
            stitch.clone(),
            make_node(
                "old-horizon",
                4,
                8.0,
                0.0,
                false,
                vec![make_pick_action("old-action", "old-box")],
            ),
        ],
        vec![
            make_edge("AB", 1, "A", "B", true, vec![]),
            make_edge("old", 3, "B", "old-horizon", false, vec![]),
        ],
    ));
    sim.update_state();
    let moving_x = sim.state.agv_position.as_ref().unwrap().x;
    assert!(sim.state.driving && moving_x > 0.0 && moving_x < 3.0);

    let mut wrong_anchor = stitch.clone();
    wrong_anchor.node_id = "wrong-anchor".into();
    sim.process_order(make_order("order", 1, vec![wrong_anchor], vec![]));
    assert_eq!(sim.state.order_update_id, 0);
    assert_eq!(sim.state.edge_states[0].edge_id, "AB");

    sim.process_order(make_order(
        "order",
        1,
        vec![stitch, make_node("C", 4, 6.0, 0.0, true, vec![])],
        vec![make_edge("BC", 3, "B", "C", true, vec![])],
    ));
    assert_eq!(sim.state.order_update_id, 1);
    assert_eq!(sim.state.agv_position.as_ref().unwrap().x, moving_x);
    assert_eq!(sim.state.edge_states[0].edge_id, "AB");
    assert!(!sim
        .state
        .action_states
        .iter()
        .any(|s| s.action_id == "old-action"));

    tick_until(&mut sim, |s| s.state.node_states.is_empty());
    assert_eq!(sim.state.last_node_id, "C");
    assert_eq!(sim.state.loads.len(), 1);
    assert_eq!(action(&sim, "take-1").action_status, ActionStatus::Finished);
}

#[test]
fn releasing_horizon_after_stop_does_not_replay_completed_stitch_action() {
    let mut sim = localized();
    let stitch = make_node(
        "A",
        0,
        0.0,
        0.0,
        true,
        vec![make_pick_action("take-1", "box-1")],
    );
    sim.process_order(make_order(
        "order",
        0,
        vec![stitch.clone(), make_node("B", 2, 2.0, 0.0, false, vec![])],
        vec![make_edge("AB", 1, "A", "B", false, vec![])],
    ));
    for _ in 0..5 {
        sim.update_state();
    }
    assert_eq!(sim.state.agv_position.as_ref().unwrap().x, 0.0);
    assert_eq!(sim.state.new_base_request, Some(true));
    assert_eq!(sim.state.loads.len(), 1);

    sim.process_order(make_order(
        "order",
        1,
        vec![stitch, make_node("B", 2, 2.0, 0.0, true, vec![])],
        vec![make_edge("AB", 1, "A", "B", true, vec![])],
    ));
    assert_eq!(sim.state.new_base_request, Some(false));
    tick_until(&mut sim, |s| s.state.node_states.is_empty());
    assert_eq!(sim.state.last_node_id, "B");
    assert_eq!(sim.state.loads.len(), 1);
    assert_eq!(action(&sim, "take-1").action_status, ActionStatus::Finished);
}

#[test]
fn cancel_stops_motion_fails_unfinished_actions_and_preserves_completed_pick() {
    let mut sim = localized();
    let order = make_order(
        "order",
        0,
        vec![
            make_node(
                "A",
                0,
                0.0,
                0.0,
                true,
                vec![make_pick_action("take-1", "box-1")],
            ),
            make_node(
                "B",
                2,
                4.0,
                0.0,
                true,
                vec![make_drop_action("drop-1", Some("box-1"))],
            ),
        ],
        vec![make_edge("AB", 1, "A", "B", true, vec![])],
    );
    sim.process_order(order.clone());
    tick_until(&mut sim, |s| s.state.driving);
    let x = sim.state.agv_position.as_ref().unwrap().x;
    let cancel = make_instant_actions(vec![make_action("cancelOrder", "cancel-1")]);
    sim.accept_instant_actions(cancel.clone());
    sim.update_state();
    assert!(!sim.state.driving);
    assert!(sim.state.node_states.is_empty() && sim.state.edge_states.is_empty());
    assert_eq!(sim.state.order_id, "order");
    assert_eq!(action(&sim, "drop-1").action_status, ActionStatus::Failed);
    assert_eq!(action(&sim, "take-1").action_status, ActionStatus::Finished);
    assert_eq!(
        action(&sim, "cancel-1").action_status,
        ActionStatus::Finished
    );

    let mut late_update = order;
    late_update.order_update_id = 1;
    sim.process_order(late_update);
    sim.accept_instant_actions(cancel);
    sim.accept_instant_actions(make_instant_actions(vec![make_action(
        "cancelOrder",
        "cancel-2",
    )]));
    for _ in 0..10 {
        sim.update_state();
    }
    assert_eq!(sim.state.order_update_id, 0);
    assert_eq!(sim.state.agv_position.as_ref().unwrap().x, x);
    assert_eq!(sim.state.loads.len(), 1);
    assert_eq!(
        action(&sim, "cancel-1").action_status,
        ActionStatus::Finished
    );
    assert_eq!(action(&sim, "cancel-2").action_status, ActionStatus::Failed);
    assert!(sim
        .state
        .errors
        .iter()
        .any(|e| e.error_type == "noOrderToCancel"
            && e.error_references
                .iter()
                .any(|r| r.reference_key == "actionId" && r.reference_value == "cancel-2")));
}
