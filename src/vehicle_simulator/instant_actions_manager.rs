use std::collections::{HashMap, VecDeque};

use crate::protocol::vda_2_0_0::vda5050_2_0_0_action::Action;
use crate::protocol::vda_2_0_0::vda5050_2_0_0_instant_actions::InstantActions;
use crate::protocol::vda_2_0_0::vda5050_2_0_0_state::{
    ActionState, ActionStatus, Error, ErrorLevel, ErrorReference, State,
};

struct Command {
    action: Action,
    result: Option<ActionState>,
}

/// An instant command executes at most once per simulator process. On a
/// retry after an order change, the original result can be reattached to state.
#[derive(Default)]
pub struct InstantActionsManager {
    queue: VecDeque<String>,
    commands: HashMap<String, Command>,
}

impl InstantActionsManager {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn accept(&mut self, request: InstantActions, state: &mut State) {
        for action in request.actions {
            let id = &action.action_id;
            let visible = state.action_states.iter().any(|s| s.action_id == *id);
            if let Some(previous) = self.commands.get(id) {
                if previous.action != action {
                    reject(
                        state,
                        id,
                        "Action ID already identifies a different command",
                    );
                } else if !visible {
                    if let Some(result) = &previous.result {
                        state.action_states.push(result.clone());
                    }
                }
                continue;
            }
            if visible || id.is_empty() {
                reject(
                    state,
                    id,
                    "Empty action ID or action ID already used by the current order",
                );
                continue;
            }

            state.action_states.push(ActionState {
                action_id: id.clone(),
                action_status: ActionStatus::Waiting,
                action_type: Some(action.action_type.clone()),
                result_description: None,
                action_description: action.action_description.clone(),
            });
            self.queue.push_back(id.clone());
            self.commands.insert(
                id.clone(),
                Command {
                    action,
                    result: None,
                },
            );
        }
    }

    pub fn pending(&self, state: &State) -> Vec<Action> {
        self.queue
            .iter()
            .filter(|id| {
                state
                    .action_states
                    .iter()
                    .any(|s| s.action_id == **id && s.action_status == ActionStatus::Waiting)
            })
            .map(|id| self.commands[id].action.clone())
            .collect()
    }

    pub(super) fn record_result(&mut self, action_id: &str, state: &State) {
        if let Some(result) = state
            .action_states
            .iter()
            .find(|s| s.action_id == action_id && s.action_status.is_terminal())
        {
            if let Some(command) = self.commands.get_mut(action_id) {
                command.result = Some(result.clone());
                self.queue.retain(|id| id != action_id);
            }
        }
    }

    pub(super) fn contains_id(&self, id: &str) -> bool {
        self.commands.contains_key(id)
    }
}

fn reject(state: &mut State, id: &str, reason: &str) {
    state
        .errors
        .retain(|e| e.error_type != "instantActionError");
    state.errors.push(Error {
        error_type: "instantActionError".into(),
        error_description: Some(reason.into()),
        error_level: ErrorLevel::Warning,
        error_references: vec![ErrorReference {
            reference_key: "actionId".into(),
            reference_value: id.into(),
        }],
    });
}
