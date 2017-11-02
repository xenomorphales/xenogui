from time import sleep

import rclpy

from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('xenogui')

    while rclpy.ok():
        for name in node.get_node_names():
            print(name)
        sleep(1)
        
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
