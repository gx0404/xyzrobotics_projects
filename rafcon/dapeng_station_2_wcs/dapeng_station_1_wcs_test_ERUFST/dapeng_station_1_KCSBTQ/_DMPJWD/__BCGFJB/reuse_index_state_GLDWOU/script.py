
def execute(self, inputs, outputs, gvm):
    """ 
    Reuse a state's output only when the state can be reused(
    Developer ensure the semantics of reuse, its properties_data has the key "reuse_tag". 
    The developer also need to realize the semantics of reuse in the code).

    For example, the state generate_grasp_plan can be reused, and its code is as blow.

    object_plan_pairs = {"current_index" : 1,
                         "grasp_plans": grasp_plans,
                         "object_poses": object_poses}
    gvm.set_variable(
        "generate_grasp_plan&{}".format(self.smart_data["reuse_tag"]),
        object_plan_pairs,
        per_reference=True)

    Now, you can understand we use gvm to realize the function. You can create a new reuse state. 
    
    Args:
        Inputs Data:
            None

        Outputs Data:
            Will copy its output to reused state.

        Gvm: 
            properties_data["target_state_name"] + "&" + properties_data["reuse_tag"] (dict) {Get}:
                Get target state's reuse tag from gvm.
            

    Properties Data:
        comment, (unicode): Default value (u"check_array_finished").
            The comment of this state. This will show in state's GUI block.

        reuse_tag, (unicode): Default value (u"test_tag").
            The tag used in our reuse feature.

        target_state_name, (unicode): Default value (u"name").
            The state we want to reused.

    Raises:
        Exception: Cannot find target state's reuse tag.

    Outcomes:
        1: exhausted
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    
    target_state_name = self.smart_data["target_state_name"]
    reuse_tag = self.smart_data["reuse_tag"]

    final_tag = str(target_state_name) + "&" + str(reuse_tag) 
    
    if not gvm.variable_exist(final_tag):
        raise Exception("tag: {} not defined".format(final_tag))
    
    index_dict = gvm.get_variable(final_tag, per_reference = True)
    current_index = index_dict["current_index"]
    outputs_keys = [key for key in index_dict.keys() if key != "current_index"]

    for key in outputs_keys:
        if current_index >= len(index_dict[key]):
            return "exhausted"
        outputs[key] = index_dict[key][current_index]
    
    index_dict["current_index"] = index_dict["current_index"] + 1
    gvm.set_variable(final_tag, index_dict, per_reference = True)

    return "success"
