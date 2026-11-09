use std::collections::HashSet;

use crate::protocol::vda_2_0_0::vda5050_2_0_0_action::Action;
use crate::protocol::vda_2_0_0::vda5050_2_0_0_order::{Edge, Node, Order};
use crate::protocol::vda_2_0_0::vda5050_2_0_0_state::{
    ActionState, ActionStatus, EdgeState, Error, ErrorLevel, ErrorReference, NodeState, State,
};
use crate::utils;

#[derive(Default)]
pub struct OrderManager {
    last_request: Option<Order>,
    nodes: Vec<Node>,
    edges: Vec<Edge>,
    accepted_ids: HashSet<String>,
    canceled: bool,
}

impl OrderManager {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn has_order(&self) -> bool {
        self.last_request.is_some() && !self.canceled
    }

    pub fn is_vehicle_ready_for_new_order(state: &State) -> bool {
        state.node_states.is_empty()
            && state.edge_states.is_empty()
            && state
                .agv_position
                .as_ref()
                .is_some_and(|p| p.position_initialized)
    }

    pub fn next_node_action(&self, state: &State) -> Option<Action> {
        let node = self
            .nodes
            .iter()
            .find(|n| n.released && n.sequence_id == state.last_node_sequence_id)?;
        waiting_action(&node.actions, state)
    }

    pub(super) fn next_edge_action(&self, state: &State) -> Option<Action> {
        let next_node = state.node_states.get(1).filter(|node| node.released)?;
        let edge = self.edges.iter().find(|edge| {
            edge.released && edge.sequence_id.checked_add(1) == Some(next_node.sequence_id)
        })?;
        waiting_action(&edge.actions, state)
    }

    pub fn process_order(&mut self, request: Order, state: &mut State) {
        self.process_order_with(request, state, |_| false);
    }

    pub(super) fn process_order_with(
        &mut self,
        request: Order,
        state: &mut State,
        instant_id: impl Fn(&str) -> bool,
    ) {
        if let Some(last) = self
            .last_request
            .as_ref()
            .filter(|last| last.order_id == request.order_id)
        {
            if request.order_update_id == last.order_update_id {
                // The transport may give a retry a new header and timestamp.
                if last.nodes != request.nodes
                    || last.edges != request.edges
                    || last.zone_set_id != request.zone_set_id
                {
                    reject(
                        state,
                        &request,
                        "orderUpdateError",
                        "Same order update ID with different content",
                    );
                }
                return;
            }
            if request.order_update_id < last.order_update_id || self.canceled {
                reject(
                    state,
                    &request,
                    "orderUpdateError",
                    "Order update is stale or order was canceled",
                );
                return;
            }
            self.extend_order(request, state, instant_id);
        } else {
            self.start_order(request, state, instant_id);
        }
    }

    fn start_order(
        &mut self,
        request: Order,
        state: &mut State,
        instant_id: impl Fn(&str) -> bool,
    ) {
        if self.accepted_ids.contains(&request.order_id) {
            reject(
                state,
                &request,
                "orderUpdateError",
                "Order ID was already used",
            );
            return;
        }
        if request.order_id.is_empty()
            || request.order_update_id != 0
            || !valid_route(&request)
            || request.nodes[0].sequence_id != 0
            || !unique_action_ids(order_actions(&request.nodes, &request.edges), &instant_id)
        {
            reject(
                state,
                &request,
                "validationError",
                "Invalid initial route or action IDs",
            );
            return;
        }
        if !Self::is_vehicle_ready_for_new_order(state) || self.has_unfinished_actions(state) {
            reject(
                state,
                &request,
                "orderError",
                "Vehicle still has an active order or is not localized",
            );
            return;
        }
        let first = &request.nodes[0];
        let at_start = match (&first.node_position, &state.agv_position) {
            (Some(node), Some(position)) => {
                node.map_id == position.map_id
                    && utils::get_distance(node.x, node.y, position.x, position.y)
                        <= node.allowed_deviation_xy.unwrap_or(0.1)
            }
            _ => state.last_node_id == first.node_id,
        };
        if !at_start {
            reject(
                state,
                &request,
                "orderError",
                "First node is not reachable from the current position",
            );
            return;
        }

        state
            .action_states
            .retain(|s| instant_id(&s.action_id) && !s.action_status.is_terminal());
        state
            .action_states
            .extend(order_actions(&request.nodes, &request.edges).map(action_state_for));
        state.node_states = request.nodes.iter().map(node_state_for).collect();
        state.edge_states = request.edges.iter().map(edge_state_for).collect();
        state.last_node_id = first.node_id.clone();
        state.last_node_sequence_id = first.sequence_id;
        self.nodes = request.nodes.clone();
        self.edges = request.edges.clone();
        self.accepted_ids.insert(request.order_id.clone());
        self.canceled = false;
        self.accept_request(request, state);
    }

    fn extend_order(
        &mut self,
        request: Order,
        state: &mut State,
        instant_id: impl Fn(&str) -> bool,
    ) {
        let stitch = self
            .nodes
            .iter()
            .rfind(|node| node.released)
            .expect("accepted route has a base");
        if !valid_route(&request)
            || request.nodes[0].node_id != stitch.node_id
            || request.nodes[0].sequence_id != stitch.sequence_id
            || request.zone_set_id != state.zone_set_id
        {
            reject(
                state,
                &request,
                "orderUpdateError",
                "Update must start at the last released node of the current order",
            );
            return;
        }
        let stitch_seq = stitch.sequence_id;
        // The old base, including its copy of the stitching node, is authoritative.
        let base_nodes = self.nodes.partition_point(|n| n.sequence_id <= stitch_seq);
        let base_edges = self.edges.partition_point(|e| e.sequence_id < stitch_seq);
        let old_base_ids: HashSet<_> =
            order_actions(&self.nodes[..base_nodes], &self.edges[..base_edges])
                .map(|a| a.action_id.as_str())
                .collect();
        if !unique_action_ids(order_actions(&request.nodes[1..], &request.edges), |id| {
            old_base_ids.contains(id) || instant_id(id)
        }) {
            reject(
                state,
                &request,
                "orderUpdateError",
                "Update reuses an action ID in the released route",
            );
            return;
        }
        let obsolete_ids: HashSet<_> =
            order_actions(&self.nodes[base_nodes..], &self.edges[base_edges..])
                .map(|a| a.action_id.as_str())
                .collect();

        state
            .action_states
            .retain(|s| !obsolete_ids.contains(s.action_id.as_str()));
        state
            .action_states
            .extend(order_actions(&request.nodes[1..], &request.edges).map(action_state_for));
        state.node_states.retain(|n| n.sequence_id <= stitch_seq);
        // The navigator may have consumed the final base node before the update arrived.
        if state.node_states.is_empty() {
            state.node_states.push(node_state_for(stitch));
        }
        state
            .node_states
            .extend(request.nodes[1..].iter().map(node_state_for));
        state.edge_states.retain(|e| e.sequence_id < stitch_seq);
        state
            .edge_states
            .extend(request.edges.iter().map(edge_state_for));

        self.nodes.truncate(base_nodes);
        self.nodes.extend_from_slice(&request.nodes[1..]);
        self.edges.truncate(base_edges);
        self.edges.extend_from_slice(&request.edges);
        self.accept_request(request, state);
    }

    fn accept_request(&mut self, request: Order, state: &mut State) {
        state.order_id = request.order_id.clone();
        state.order_update_id = request.order_update_id;
        state.zone_set_id = request.zone_set_id.clone();
        state.errors.retain(|e| {
            !matches!(
                e.error_type.as_str(),
                "validationError" | "orderError" | "orderUpdateError"
            )
        });
        self.last_request = Some(request);
        self.update_base_request(state);
    }

    pub(super) fn update_base_request(&self, state: &mut State) {
        state.new_base_request = Some(
            self.has_order()
                && self.nodes.last().is_some_and(|node| !node.released)
                && self
                    .nodes
                    .iter()
                    .rfind(|node| node.released)
                    .is_some_and(|node| node.sequence_id == state.last_node_sequence_id),
        );
    }

    pub(super) fn has_unfinished_actions(&self, state: &State) -> bool {
        self.has_order()
            && order_actions(&self.nodes, &self.edges).any(|action| {
                state
                    .action_states
                    .iter()
                    .any(|s| s.action_id == action.action_id && !s.action_status.is_terminal())
            })
    }

    /// This kinematic model can stop immediately. Only unfinished order
    /// actions are canceled; completed effects and instant actions are kept.
    pub(super) fn cancel(&mut self, state: &mut State) -> bool {
        if !self.has_order()
            || (state.node_states.is_empty()
                && state.edge_states.is_empty()
                && !self.has_unfinished_actions(state))
        {
            return false;
        }

        let ids: HashSet<_> = order_actions(&self.nodes, &self.edges)
            .map(|a| a.action_id.as_str())
            .collect();
        for action in &mut state.action_states {
            if ids.contains(action.action_id.as_str()) && !action.action_status.is_terminal() {
                action.action_status = ActionStatus::Failed;
                action.result_description = Some("Order canceled".into());
            }
        }
        self.canceled = true;
        state.node_states.clear();
        state.edge_states.clear();
        state.driving = false;
        state.velocity = None;
        self.update_base_request(state);
        true
    }
}

fn valid_route(request: &Order) -> bool {
    let Some(first) = request.nodes.first() else {
        return false;
    };
    first.released
        && request.edges.len() == request.nodes.len() - 1
        && request
            .nodes
            .windows(2)
            .zip(&request.edges)
            .all(|(pair, edge)| {
                let [start, end] = [&pair[0], &pair[1]];
                edge.start_node_id == start.node_id
                    && edge.end_node_id == end.node_id
                    && start.sequence_id.checked_add(1) == Some(edge.sequence_id)
                    && edge.sequence_id.checked_add(1) == Some(end.sequence_id)
                    && (start.released || !end.released)
                    && edge.released == end.released
            })
}

fn order_actions<'a>(nodes: &'a [Node], edges: &'a [Edge]) -> impl Iterator<Item = &'a Action> {
    nodes
        .iter()
        .flat_map(|n| &n.actions)
        .chain(edges.iter().flat_map(|e| &e.actions))
}

fn unique_action_ids<'a>(
    actions: impl Iterator<Item = &'a Action>,
    reserved: impl Fn(&str) -> bool,
) -> bool {
    let mut ids = HashSet::new();
    actions.into_iter().all(|a| {
        !a.action_id.is_empty() && !reserved(&a.action_id) && ids.insert(a.action_id.as_str())
    })
}

fn waiting_action(actions: &[Action], state: &State) -> Option<Action> {
    actions
        .iter()
        .find(|a| {
            state
                .action_states
                .iter()
                .any(|s| s.action_id == a.action_id && s.action_status == ActionStatus::Waiting)
        })
        .cloned()
}

fn reject(state: &mut State, order: &Order, kind: &str, reason: &str) {
    state.errors.retain(|error| error.error_type != kind);
    state.errors.push(Error {
        error_type: kind.into(),
        error_description: Some(reason.into()),
        error_level: ErrorLevel::Warning,
        error_references: vec![
            ErrorReference {
                reference_key: "orderId".into(),
                reference_value: order.order_id.clone(),
            },
            ErrorReference {
                reference_key: "orderUpdateId".into(),
                reference_value: order.order_update_id.to_string(),
            },
        ],
    });
}

fn node_state_for(node: &Node) -> NodeState {
    NodeState {
        node_id: node.node_id.clone(),
        sequence_id: node.sequence_id,
        released: node.released,
        node_description: node.node_description.clone(),
        node_position: node.node_position.clone(),
    }
}

fn edge_state_for(edge: &Edge) -> EdgeState {
    EdgeState {
        edge_id: edge.edge_id.clone(),
        sequence_id: edge.sequence_id,
        released: edge.released,
        edge_description: edge.edge_description.clone(),
        trajectory: edge.trajectory.clone(),
    }
}

fn action_state_for(action: &Action) -> ActionState {
    ActionState {
        action_id: action.action_id.clone(),
        action_type: Some(action.action_type.clone()),
        action_description: action.action_description.clone(),
        action_status: ActionStatus::Waiting,
        result_description: None,
    }
}
