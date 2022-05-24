# swarm_sync_ndoe
# Author: Martin Schoerner
# Last Change: 2019-08-9
#
from time import sleep
import rclpy
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import UInt8 
from std_srvs.srv import Empty
node = None
init_complete = False
heartbeatlist = []

def heartbeat_callback(msg):
    global node, heartbeatlist
    mavid = msg.data
    if mavid not in heartbeatlist:
        node.get_logger().info('I heard heartbeat from: "%d"' % msg.data)
        heartbeatlist.append(mavid)


def srv_callback(request, response):
    global node, init_complete

    node.get_logger().info("Swarm Sync Init complete triggered")
    init_complete = True

    return response

def main(args=None):
    global node, thread_active
    rclpy.init(args=args)

    # Start node
    node = rclpy.create_node('swarm_sync_node')

    # Read parameters
    rate = 2 
    info_topic = '/sync/info'
    heartbeat_topic = '/sync/heartbeat'
    initservicename = 'sync/init_complete'
    
    # Create subscriber
    subscriber = node.create_subscription(UInt8, heartbeat_topic, heartbeat_callback, 10)
    subscriber  # prevent unused variable warning

    # Cerate publisher
    publisher = node.create_publisher(UInt8MultiArray, info_topic)

    # Create service
    srv = node.create_service(Empty, initservicename, srv_callback)


    while (not init_complete):
        rclpy.spin_once(node)


    # Create  message for the sensor values
    msg = UInt8MultiArray()

    # use collected heartbeats
    msg.data = heartbeatlist



    node.get_logger().info("Swarm Sync Node Online!")
    membersstr = "Swarm members are: "
    for member in heartbeatlist:
        membersstr += ", "
        membersstr += str(member)
    node.get_logger().info(membersstr)

    # open serial port
    while rclpy.ok():
        #  read line from serial port
        rclpy.spin_once(node)
        publisher.publish(msg)

        sleep(1 / rate)  # seconds

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    node.destroy_service(srv)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
