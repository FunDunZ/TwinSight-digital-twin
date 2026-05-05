import rclpy
from rclpy.node import Node
from launch  

from std_srvs.srv import SetBool
import os

class LineFollowService(Node):
    def __init__(self):
        super().__init__('line_follow_service')

        self.srv = self.create_service(
                SetBool,
                'start_line_following',
                self.line_follow_callback
            )
        self.get_logger().info('Line-following service is ready.')

    def line_follow_callback(self, request, response):
        
        # Activate the line following logic

        if request.data:
            self.get_logger().info('Starting line-following...')
            # Call your line-following function here

            
        else:
            self.get_logger().info('Stopping line-following...')
            # Call stop function here
        response.success = True
        response.message = 'Line-following started.' if request.data else 'Line-following stopped.'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowService()
    rclpy.spin(line_follow_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

