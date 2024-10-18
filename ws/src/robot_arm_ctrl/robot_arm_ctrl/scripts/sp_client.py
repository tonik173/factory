import random
import sys
import pysparkplug as psp

class SparkplugEdgeNode:

    def __init__(self, host, port, group_id, edge_node_id):

        self.group_id = group_id
        self.edge_node_id = edge_node_id

        metrics = (
            self.compose_metrics(origin = "shoulder_joint", source = "position", unit = "revolute", value = 0.0) 
            + self.compose_metrics(origin = "upper_arm_joint", source = "position", unit = "revolute", value = 0.0)
            + self.compose_metrics(origin = "middle_arm_joint", source = "position", unit = "revolute", value = 0.0)
            + self.compose_metrics(origin = "fore_arm_joint", source = "position", unit = "revolute", value = 0.0)
        )
        
        self.edge_node = psp.EdgeNode(self.group_id, self.edge_node_id, metrics)
        self.edge_node.connect(host, port=port)
        print("SPARKPLUG connected")

    def compose_metrics(self, origin: str, source: str, unit: str, value: float) -> psp.Metric:
        name = (origin + '/' + source + '/' + unit).lower()
        metrics = psp.Metric(timestamp=psp.get_current_timestamp(), name=name, datatype=psp.DataType.FLOAT, value=value),          
        return metrics

    def send(self, origin: str, source: str, unit: str, value: float):
        try:
            metrics = self.compose_metrics(origin, source, unit, value)
            self.edge_node.update(metrics)
        except:
            err = " failed ({e})".format(e=sys.exc_info()[0])
            print(message + err)

    def disconnect(self):

        self.edge_node.disconnect()

if __name__ == '__main__':

    host = "172.16.1.10"
    edgeNode = SparkplugEdgeNode(host, 1885, "factory_test", "robot_arm_test")
    edgeNode.send("shoulder_joint", "position", "revolute", random.uniform(-40.0, 40.0))
    edgeNode.disconnect()