## I'm barrier state's script

def get_state_id_and_outcome(state_info):
    state_id = state_info.split(":")[0].lstrip().split("_")[0]
    state_outcome_id = state_info.split(":")[1].lstrip().split("_")[0]
    return state_id, int(state_outcome_id)


def execute(self, inputs, outputs, gvm):
    outcome_transitions = self.properties_data["outcome_transitions"]
    
    final_outcome_id_dict = {}
    states_info = list(outcome_transitions.keys())[0].split("--and--")
    for state_info in states_info:
        state_id, state_outcome_id = get_state_id_and_outcome(state_info)
        final_outcome_id_dict[state_id] = self.get_outcome_for_state_id(state_id).outcome_id
        if final_outcome_id_dict[state_id] == -1:
            self.logger.error("state_id: {}'s error: {}".format(state_id, self.get_errors_for_state_id(state_id)))
            raise self.get_errors_for_state_id(state_id)
        elif final_outcome_id_dict[state_id] == -2:
            return -2
            
    final_outcome = None
    for transition in list(outcome_transitions.keys()):
        states_info = transition.split("--and--")
        find_transition = True
        for state_info in states_info:
            state_id, state_outcome_id = get_state_id_and_outcome(state_info)
            if final_outcome_id_dict[state_id] != state_outcome_id:
                find_transition = False
                break
        if find_transition:
            final_outcome = outcome_transitions[transition]
            break

    if final_outcome is not None:
        return final_outcome
    else:
        raise Exception("Do not find the source")






