
import sys
import rclpy
from owlready2 import *
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_prefix

class PublisherNode(Node):
    def __init__(self):
        super().__init__('owl_reasoner')
        package_shared_directory = get_package_prefix('nav_ontologies')
        self.onto = get_ontology(package_shared_directory +'/share/nav_ontologies/owl_knowledge/usv.owl').load()
        
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.msg_ = String()
        self.counter_ = 0
        self.timer_ = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info("ROS2 class node first test")
    
    def timer_callback(self):
            # Print classes
        print("Classes:")
        for cls in self.onto.classes():
            self.msg_.data = self.msg_.data + "  " + cls.name
            print(cls.name)

        self.publisher_.publish(self.msg_)

        # # Print individuals
        # print("\nIndividuals:")
        # for individual in self.onto.individuals():
        #     print(individual.name)

        # # Print object properties
        # print("\nObject Properties:")
        # for prop in self.onto.object_properties():
        #     print(prop.name)

        # # Print data properties
        # print("\nData Properties:")
        # for prop in self.onto.data_properties():
        #     print(prop.name)

def main(args=None):
    # Load the ontology
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
