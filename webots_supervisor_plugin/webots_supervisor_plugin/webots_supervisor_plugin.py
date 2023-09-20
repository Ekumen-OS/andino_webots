import rclpy


class WebotsSupervisorPlugin:
    def __init__(self):
        self.robot_name = None
        self.unwanted_devices = []

    def init(self, webots_node, properties):
        rclpy.init(args=None)
        self.__node = rclpy.create_node("Webots_supervisor_node")
        self.__node.get_logger().info("  - properties: " + str(properties))
        self.robot_name = str(properties["robotName"])

        self.__robot = webots_node.robot
        self.unwanted_devices = [device for device in properties["devices"].split(" ")]

    def get_robot(self, robot_name):
        root_node = self.__robot.getRoot()
        children = root_node.getField("children")
        # Look for 'robot_name' among all robots
        for idx in range(children.getCount()):
            child = children.getMFNode(idx)
            if (
                child.getTypeName() == "Robot"
                and child.getField("name").getSFString() == robot_name
            ):
                return child

    def remove_node(self, node):
        if node is not None:
            if (
                node.getField("name")
                and node.getField("name").getSFString() in self.unwanted_devices
            ):
                self.__node.get_logger().info(
                    f"Removing {node.getField('name').getSFString()}"
                )
                node.remove()
                self.unwanted_devices.remove(node.getField("name").getSFString())
                return
            children = node.getField("children")
            if children is not None:
                for child_node in [
                    children.getMFNode(idx) for idx in range(children.getCount())
                ]:
                    self.remove_node(child_node)
            devices = node.getField("device")
            if devices is not None:
                for device in [
                    devices.getMFNode(idx) for idx in range(devices.getCount())
                ]:
                    self.remove_node(device)
                self.remove_node(node.getField("endPoint").getSFNode())

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        if self.unwanted_devices:
            try:
                robot = self.get_robot(self.robot_name)
                self.remove_node(robot)
            except Exception as ex:
                self.__node.get_logger().info(f"Exception {ex}!")
