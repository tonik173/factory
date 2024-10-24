import random
import time
from mqtt_spb_wrapper import *

_DEBUG = True 

class SparkplugNode:

    def __init__(self, host, port, group_id, edge_node_id):

        self.node = MqttSpbEntity(group_id, edge_node_id, None, _DEBUG)
        self.node.on_command = self.callback_command

        self.set_position(origin = "shoulder_joint", source = "position", unit = "revolute", reference = 0.0, actual=0.0)
        self.set_position(origin = "upper_arm_joint", source = "position", unit = "revolute", reference = 0.0, actual=0.0)
        self.set_position(origin = "middle_arm_joint", source = "position", unit = "revolute", reference = 0.0, actual=0.0)
        self.set_position(origin = "fore_arm_joint", source = "position", unit = "revolute", reference = 0.0, actual=0.0)

        # --- Attributes   ----------------------------------------------------------------------------------------------------
        # Only sent during BIRTH message, with the prefix "ATTR/" over all the metric names
        # You can change the attributes birth prefix using the attribute: device.attributes.birth_prefix = "ATTR"
        self.node.attributes.set_value("description", "Robot arm node")
        self.node.attributes.set_value("type", "robot-arm-node")
        self.node.attributes.set_value("version", "0.1")

        # --- Commands  --------------------------------------------------------------------------------------------------------
        # During BIRTH message, the prefix "CMD/" is added to all the metric names
        # You can change the attributes birth prefix using the attribute: device.commands.birth_prefix = "CMD"
        self.node.commands.set_value(name="rebirth", value=False)

        # Connect to the spB MQTT broker ---------------------------------------------------------------------------------------
        connected = False
        while not connected:
            print("Trying to connect to broker %s:%d ..." % (host, port))

            connected = self.node.connect(host=host, port=port, user="", password="", )

            if not connected:
                print("  Error, could not connect. Trying again in a few seconds ...")
                time.sleep(3)

        # Send device birth message
        self.node.publish_birth()

        print("SPARKPLUG connected")

    def callback_command(cmd):
        print("NODE received CMD: %s" % str(cmd))

    def set_position(self, origin: str, source: str, unit: str, reference: float, actual: float):
        name = (origin + '/' + source + '/' + unit).lower()   

        self.node.data.set_value(
            name=name,
            value={
                "Reference": [reference],
                "Actual": [actual],
            }
        )   

    def send(self):
        self.node.publish_data()

    def disconnect(self):

        print("Disconnecting node")
        self.node.disconnect()

if __name__ == '__main__':

    host = "172.16.1.10"
    edgeNode = SparkplugNode(host, 1885, "factory_test", "robot_arm_test")
    edgeNode.set_position("shoulder_joint", "position", "revolute", random.uniform(-40.0, 40.0), random.uniform(-20.0, 20.0))
    edgeNode.send()
    edgeNode.disconnect()