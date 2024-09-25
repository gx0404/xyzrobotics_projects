from xyz_env_manager.client import book_pick_items, book_place_items
from xyz_env_manager.msg import BookedPickItems, BookedPlaceItems

def execute(self, inputs, outputs, gvm):
    self.logger.info("Running {}({})".format(self.name, self.state_id))
    self.logger.info(self.smart_data["comment"])
    execution_payload = inputs["execution_payload"]

    if execution_payload["grasp_plan"] and execution_payload["solution"]:
        book_pick = BookedPickItems()
        book_pick.workspace_id = execution_payload["grasp_plan"].from_workspace_id
        book_pick.pick_item_names = [obj.name for obj in execution_payload["grasp_plan"].objects]
        book_pick_items(book_pick)

        all_items = execution_payload["solution"].solved_env.get_container_items(execution_payload["grasp_plan"].to_workspace_id)
        placed_items = [item for item in all_items if item.name in book_pick.pick_item_names]

        book_place = BookedPlaceItems()
        book_place.workspace_id = execution_payload["grasp_plan"].to_workspace_id
        book_place.planned_item_ids = execution_payload["grasp_plan"].planned_items_ids
        book_place.placed_items = placed_items
        book_place_items(book_place)

    return "success"