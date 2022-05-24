# swarm_sync_ndoe
# Author: Martin Schoerner
# Last Change: 2019-08-9
#
from time import sleep
import rclpy
import sys
import os

from std_msgs.msg import UInt8

node = None

def get_mavid():
    global node
    ret = 0
    home = os.getenv("HOME")
    try:
        with open("%s/mavid.txt"%home, "r") as f:
            ret = int(f.readline())
        print("Mavid=%d"%ret)
    except:
        print("No mavid.txt file found in home directory. Using default (0)")

    return ret

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('swarm_sync_client')

    publisher = node.create_publisher(UInt8, '/sync/heartbeat', 10)

    msg = UInt8()

    mavid = get_mavid()

    while rclpy.ok():
        msg.data = mavid

#        node.get_logger().info('Publishing: "%d"' % msg.data)
        publisher.publish(msg)
        sleep(1.0)  # seconds

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
