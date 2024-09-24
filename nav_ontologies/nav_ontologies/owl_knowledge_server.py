
import sys
import rclpy
from owlready2 import *
from rclpy.node import Node
from std_msgs.msg import String
from owl_knowledge_interface.srv import GetOWLKnowledge

from ament_index_python.packages import get_package_prefix

class KnowledgeServer(Node):

    def __init__(self):
        super().__init__('knowledge_server')
       
        self.init_knowledge_server()
        self.srv = self.create_service(GetOWLKnowledge, 'get_owl_knowledge', self.get_owl_classes_callback)
    
    def init_knowledge_server(self):
        package_shared_directory = get_package_prefix('nav_ontologies')
        self.onto = get_ontology(package_shared_directory +'/share/nav_ontologies/owl_knowledge/usv.owl').load()
          

    def get_owl_classes_callback(self, request, response):
        msg = ''
        if request.owl == 'USV':
            for cls in self.onto.classes():
                msg = msg + "  " + cls.name
                print(cls.name)
            response.owl_classes = msg
            self.get_logger().info('CLASS LIST IS : %s' % (response.owl_classes))
        else:
            response.owl_classes = 'null'

        return response


def main(args=None):
    rclpy.init(args=args)

    knowledge_server = KnowledgeServer()

    rclpy.spin(knowledge_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()