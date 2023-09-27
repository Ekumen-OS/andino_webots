# BSD 3-Clause License

# Copyright (c) 2023, Ekumen Inc.
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


class NodeRemoverPlugin:
    def __init__(self):
        self.robot_name = None
        self.unwanted_nodes = []

    def init(self, webots_node, properties):
        self.robot_name = str(properties["robotName"])

        self.__robot = webots_node.robot
        self.unwanted_nodes = [node for node in properties["nodes"].split(" ")]

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
                and node.getField("name").getSFString() in self.unwanted_nodes
            ):
                print(f"Removing {node.getField('name').getSFString()}")
                node.remove()
                self.unwanted_nodes.remove(node.getField("name").getSFString())
                return
            children = node.getField("children")
            if children is not None:
                for child_node in [
                    children.getMFNode(idx) for idx in range(children.getCount())
                ]:
                    self.remove_node(child_node)
            devices = node.getField("device")
            if devices is not None:
                for device_node in [
                    devices.getMFNode(idx) for idx in range(devices.getCount())
                ]:
                    self.remove_node(device_node)
                self.remove_node(node.getField("endPoint").getSFNode())

    def step(self):
        if self.unwanted_nodes:
            try:
                robot = self.get_robot(self.robot_name)
                self.remove_node(robot)
            except Exception as ex:
                print(f"Exception {ex}!")
