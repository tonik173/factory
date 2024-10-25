import random
import time
from mqtt_spb_wrapper import *

_DEBUG = True 

class SparkplugNode:

    def __init__(self, host, port, group_id, edge_node_id):

        self.node = MqttSpbEntity(group_id, edge_node_id, None, _DEBUG)
        self.node.on_command = self.callback_command
        self.node.commands.set_value(name="run", value=False, callback_on_change=self.callback_cmd_run )

        zero = [0.0, 0.0, 0.0, 0.0]
        self.set_trajectory_move(origin = "shoulder_joint", unit = "revolute", p = zero, v = zero, a = zero, f = zero)
        self.set_trajectory_move(origin = "upper_arm_joint", unit = "revolute", p = zero, v = zero, a = zero, f = zero)
        self.set_trajectory_move(origin = "middle_arm_joint", unit = "revolute", p = zero, v = zero, a = zero, f = zero)
        self.set_trajectory_move(origin = "fore_arm_joint", unit = "revolute", p = zero, v = zero, a = zero, f = zero)

        # --- Attributes   ----------------------------------------------------------------------------------------------------
        # Only sent during BIRTH message, with the prefix "ATTR/" over all the metric names
        # Change the attributes birth prefix using the attribute: node.attributes.birth_prefix = "ATTR"
        self.node.attributes.set_value("description", "Robot arm node")
        self.node.attributes.set_value("type", "robot-arm-node")
        self.node.attributes.set_value("version", "0.1")

        # --- Commands  --------------------------------------------------------------------------------------------------------
        # During BIRTH message, the prefix "CMD/" is added to all the metric names
        # Change the attributes birth prefix using the attribute: node.commands.birth_prefix = "CMD"
        self.node.commands.set_value(name="rebirth", value=False)

        # Connect to the spB MQTT broker ---------------------------------------------------------------------------------------
        connected = False
        while not connected:
            print("Trying to connect to broker %s:%d ..." % (host, port))

            connected = self.node.connect(host=host, port=port, user="", password="", )

            if not connected:
                print("  Error, could not connect. Trying again in a few seconds ...")
                time.sleep(3)

        self.node.publish_birth()
        print("SPARKPLUG connected")

    def callback_command(cmd):
        print("NODE received CMD: %s" % str(cmd))

    def callback_cmd_run(data_value):
        print("   CMD run received - value: " + str(data_value))

    def set_trajectory_move(self, origin: str, unit: str, p: [float], v: [float], a: [float], f: [float]):
        name = (origin + '/' + '/' + unit).lower()   

        self.node.data.set_value(
            name=name,
            value={
                "p": p,
                "v": v,
                "a": a,
                "f": f,
                "src" : ["ref", "fb", "err", "out"]
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
    edgeNode.set_trajectory_move("shoulder_joint", "revolute", [1,2,3,4], [5,6,7,8], [9,10,11,12], [13,14,15,16])
    edgeNode.send()
    edgeNode.disconnect()