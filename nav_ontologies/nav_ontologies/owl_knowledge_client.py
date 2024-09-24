import sys

from owl_knowledge_interface.srv import GetOWLKnowledge
import rclpy
from rclpy.node import Node


class KnowledgeClient(Node):

    def __init__(self):
        super().__init__('knowledge_client')
        self.cli = self.create_client(GetOWLKnowledge, 'get_owl_knowledge')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetOWLKnowledge.Request()

    def send_request(self, owl):
        self.req.owl = owl
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
def main(args=None):
    rclpy.init(args=args)

    knowledge_client = KnowledgeClient()
    owl = 'USV'

    knowledge_client.send_request(owl)

    while rclpy.ok():
        rclpy.spin_once(knowledge_client)
        if knowledge_client.future.done():
            try:
                response = knowledge_client.future.result()
            except Exception as e:
                knowledge_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                knowledge_client.get_logger().info(
                    'Result of add_two_ints: for %s' %
                    (response.owl_classes))
            break

    knowledge_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()