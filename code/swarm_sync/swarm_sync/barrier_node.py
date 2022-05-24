# swarm_sync_ndoe
# Author: Martin Schoerner
# Last Change: 2019-08-9
#
# Waits at barrier until all other participants have arrived

from time import sleep
import rclpy
from std_msgs.msg import UInt8MultiArray, UInt8
import sys
import os
node = None
participants = list()
arrived = set()
reached_barrier = False
def is_barrier_reached():
    for p in participants:
        if p not in arrived:
            return False
    return True




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

def info_cb(msg):
    global participants, node
    participants_tmp = list()
    for element in msg.data:
        participants_tmp.append(element)

    participants = participants_tmp


def barrier_cb(msg):
    global reached_barrier
    arrived.add(msg.data)
    if(is_barrier_reached()):
        reached_barrier = True
        node.get_logger().info("We think we're done")

def barrier_reached_cb(msg):
    global reached_barrier
    if(msg.data == 1):
        reached_barrier = True
    node.get_logger().info("Other participant says we're done")

# Args: barrier_id
def main(args=None):
    global node
    rclpy.init(args=args)

    barrier_name = sys.argv[1]

    mav_id = get_mavid()
    node_name = 'barrier_%s' % barrier_name
    # We get an array of swarm participants here
    info_topic = '/sync/info'
    # Everybody at the barrier publishes here
    barrier_topic = '/barrier/%s' % barrier_name
    # If everybody reached the barrier, publish here
    barrier_reached_topic = '/barrier/%s/reached' % barrier_name
    rate = 20

    # Start node
    node = rclpy.create_node(node_name)
    node.get_logger().info("%s started" % node_name)
    info_sub = node.create_subscription(UInt8MultiArray, info_topic, info_cb)
    info_sub  # prevent unused variable warning

    # This will prevent barriers from saying they're reached
    # when they are started before the control node is launched
    while len(participants) < 1:
        rclpy.spin_once(node)

    node.get_logger().info("got %s participants" % len(participants))
    barrier_sub = node.create_subscription(UInt8, barrier_topic , barrier_cb)
    barrier_sub  # prevent unused variable warning

    barrier_reached_sub = node.create_subscription(UInt8, barrier_reached_topic , barrier_reached_cb)
    barrier_reached_sub  # prevent unused variable warning

    barrier_pub = node.create_publisher(UInt8, barrier_topic)
    barrier_reached_pub = node.create_publisher(UInt8, barrier_reached_topic)


    id_msg = UInt8()
    id_msg.data = mav_id
    barrier_pub.publish(id_msg)
    while rclpy.ok() and reached_barrier == False:
        barrier_pub.publish(id_msg)
        for i in range(0,10):
            rclpy.spin_once(node)
        #rclpy.spin_once(node)
        sleep(1 / rate)  # seconds
    node.get_logger().info("Reached Barrier %s!" % barrier_name)

    reached_msg = UInt8()
    reached_msg.data = 1
    barrier_reached_pub.publish(reached_msg)
    # Get stuck messages out
    for i in range(0,10):
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
