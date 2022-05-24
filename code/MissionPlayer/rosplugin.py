import shlex
from subprocess import Popen
import subprocess
import time

scriptdir = './rosplugin/'


def block_call_service_ros2(args):
    """ Calls a ros2 service

        Parameters:
        args (dict): Parsed json arguments for block as dict
            Possible keys:

            next: Next block in case of success
            address: Address of the service
            svctype: Type of the service
            data: Data for service call
            [commandline_args]: Additional commandline args for ros2 service
                                command as list of strings
        Returns:
        string: Id of next block to be executed
   """
    print("Calling ROS2 Service %s" % (args['address']))
    if 'data' in args.keys():
        popenargs = scriptdir+'call_service_ros2.sh %s %s "%s"' % (
                 args['address'],
                 args['svctype'],
                 args['data'])

    else:
        popenargs = scriptdir+'call_service_ros2.sh %s %s' % (
                 args['address'],
                 args['svctype'])

    if 'commandline_args' in args.keys():
        for cmdlinearg in args["commandline_args"]:
            popenargs += " "
            popenargs += cmdlinearg
#    Popen(shlex.split(popenargs))

    subprocess.call(shlex.split(popenargs))
    return args['next']


def block_call_service_ros1(args):
    """ Calls a ros1 service

        Parameters:
        args (dict): Parsed json arguments for block as dict
            Possible keys:

            next: Next block in case of success
            name: Address of the service
            data: Data for service call
            [commandline_args]: Additional commandline args for ros2 service
                                command as list of strings
            [timeoutms]: Timeout for servicecall in ms
            [next_timeout]: Next block in case of timeout
        Returns:
        string: Id of next block to be executed
   """
    print("Calling ROS1 Service %s" % (args['name']))
    if 'data' in args.keys():
        popenargs = scriptdir+'call_service_ros1.sh %s %s' % (
                     args['name'],
                     args['data'])
    else:
        popenargs = scriptdir+'call_service_ros1.sh %s' % (
                     args['name'])

    if 'commandline_args' in args.keys():
        for cmdlinearg in args["commandline_args"]:
            popenargs += " "
            popenargs += cmdlinearg

    if 'timeout' in args.keys():
        try:
            subprocess.call(shlex.split(popenargs),
                            timeout=int(args['timeoutms'])/1000.0)
        except TimeoutExpired:
            if 'next_timeout' in args.keys():
                return args['next_timeout']
            else:
                return args['next']
    else:
        subprocess.call(shlex.split(popenargs))
    return args['next']


def block_start_roscore(args):
    """ Starts ROS1 roscore

        Parameters:
        args (dict): Parsed json arguments for block as dict
            Possible keys:
            next: Next block in case of success
            [commandline_args]: Additional commandline args for ros2 service command as list of strings

        Returns:
        string: Id of next block to be executed
   """
    print("Starting roscore ")
    popenargs = scriptdir+'roscore.sh'
    if 'commandline_args' in args.keys():
        for cmdlinearg in args["commandline_args"]:
            popenargs += " "
            popenargs += cmdlinearg

    Popen(shlex.split(popenargs))
    return args['next']



def block_wait_for_active_topic_ros1(args):
    """ Waits for messages in ros1 topic

        Parameters:
        args (dict): Parsed json arguments for block as dict
            Possible keys:

            next: Next block in case of success
            name: name of the topic
            [timeoutms]: Timeout for servicecall in ms
            [next_timeout]: Next block in case of timeout
        Returns:
        string: Id of next block to be executed
   """
    print("Waiting for ROS1 Topic %s" % (args['name']))

    popenargs = scriptdir+'wait_for_active_topic_ros1.sh %s' % (
                 args['name'])

    if 'commandline_args' in args.keys():
        for cmdlinearg in args["commandline_args"]:
            popenargs += " "
            popenargs += cmdlinearg

    if 'timeout' in args.keys():
        try:
            subprocess.call(shlex.split(popenargs), timeout=int(args['timeoutms'])/1000.0)
        except TimeoutExpired:
            if 'next_timeout' in args.keys():
                return args['next_timeout']
            else:
                return args['next']
    else:
        subprocess.call(shlex.split(popenargs))
    return args['next']


def block_wait_for_active_topic_ros2(args):
        """ Waits for messages in ros2 topic

            Parameters:
            args (dict): Parsed json arguments for block as dict
                Possible keys:

                next: Next block in case of success
                name: name of the topic
                [timeoutms]: Timeout in ms
                [next_timeout]: Next block in case of timeout
            Returns:
            string: Id of next block to be executed
       """
        print("Waiting for ROS2 Topic %s" % (args['name']))

        popenargs = scriptdir+'wait_for_active_topic_ros2.sh %s' % (
                     args['name'].replace("/", "\\\\/"))

        if 'commandline_args' in args.keys():
            for cmdlinearg in args["commandline_args"]:
                popenargs += " "
                popenargs += cmdlinearg

        if 'timeout' in args.keys():
            try:
                subprocess.call(shlex.split(popenargs),
                                timeout=int(args['timeoutms'])/1000.0)
            except TimeoutExpired:
                if 'next_timeout' in args.keys():
                    return args['next_timeout']
                else:
                    return args['next']
        else:
            subprocess.call(shlex.split(popenargs))
        return args['next']


def block_start_node_ros2(args):
    """ Starts the ros2 node


        Parameters:
        args (dict): Parsed json arguments for block as dict
            Possible keys:

            next: Next block in case of success
            package: Package of ROS2 node to be started
            node: Name of the node
            nodename: Name of the instance of the node
            [commandline_args]: Additional commandline args for
                              ros2 run command as list of strings
        Returns:
        string: Id of next block to be executed
    """
    print("Starting ROS2 Node %s %s %s" % (
          args['package'],
          args['node'],
          args['nodename']))

    popenargs = scriptdir+"start_node_ros2.sh %s %s %s" % (
                args['package'],
                args['node'],
                args['nodename'])

    if 'commandline_args' in args.keys():
        for cmdlinearg in args["commandline_args"]:
            popenargs += " "
            popenargs += cmdlinearg
    Popen(shlex.split(popenargs))

    return args['next']


_logger_handle = None


def block_start_logging(args):
    """ Starts the ros2 bag record process for logging sensor values.
        Only one Logger can be started at a time

        Parameters:
        args (dict): Parsed json arguments for block as dict
             Possible keys:

             next: Next block in case of success
             [next_already_active]: Next block if no logger was active
             [logger_args]: args for the ros2 logger

        Returns:
        string: Id of next block to be executed
   """
    # ros2 bag record --all
    global _logger_handle
    if _logger_handle is not None:
        print('Logger already active, starting another one won\'t work...')
        if 'next_already_active' in args.keys():
            return args['next_already_active']
        else:
            return args['next']
    popenargs = shlex.split(scriptdir+'start_logging.sh')
    if 'logger_args' in args.keys():
        popenargs.append(shlex.split(args['logger_args']))
    else:
        popenargs.append("--all")
    _logger_handle = Popen(popenargs)
    return args['next']


def block_stop_logging(args):
    """ Kills the currently running logger module

        Parameters:
        args (dict): Parsed json arguments for block as dict
            Possible keys:

                next: Next block in case of success
                [next_not_active]: Next block if no logger was active
                [next_timeout]: Next block if termination times out
                [timeout]: Timeout in ms (Default 10000)

        Returns:
        string: Id of next block to be executed
    """
    if _logger_handle is None:
        print('Tried to end inactive logger, that won\'t work...')
        if 'next_not_active' in args.keys():
            return args['next_not_active']
        else:
            return args['next']
    _logger_handle.terminate()

    # Wait for logger to die
    time_of_termination = time.time()

    timeout = 10.000
    if 'timeout' in args.keys():
        timeout = float(args['timeout']) / 1000.0

    while _logger_handle.poll() is not None:
        if time.time() - time_of_termination > timeout:
            print('Process took to long to die')
            if 'next_timeout' in args.keys():
                return args['next_timeout']
            else:
                return args['next']
    return args['next']


def block_launch_ros1(args):
    """ Launches ROS1 Launch file


        Parameters:
        args (dict): Parsed json arguments for block as dict
            Possible keys:

            next: Next block in case of success
            package: Package of ROS2 node to be started
            launchfile: Name of the node

            [commandline_args]: Additional commandline args for
                              ros2 run command as list of strings
        Returns:
        string: Id of next block to be executed
    """
    print("Launching ROS1 Launch File %s %s" % (
          args['package'],
          args['launchfile']))

    popenargs = scriptdir+"launch_ros1.sh %s %s" % (
                args['package'],
                args['launchfile'])

    if 'commandline_args' in args.keys():
        for cmdlinearg in args["commandline_args"]:
            popenargs += " "
            popenargs += cmdlinearg
    Popen(shlex.split(popenargs))

    return args['next']

def block_start_ros1_bridge(args):
    """ Starts ROS1 bridge

        Parameters:
        args (dict): Parsed json arguments for block as dict
            Possible keys:
            next: Next block in case of success
            [commandline_args]: Additional commandline args for command as list of strings

        Returns:
        string: Id of next block to be executed
   """
    print("Starting roscore ")
    popenargs = scriptdir+'start_ros1_bridge.sh'
    if 'commandline_args' in args.keys():
        for cmdlinearg in args["commandline_args"]:
            popenargs += " "
            popenargs += cmdlinearg

    Popen(shlex.split(popenargs))
    return args['next']

    
blocks = {
    "CallServiceROS2": block_call_service_ros2,
    "CallServiceROS1": block_call_service_ros1,
    "StartRoscore": block_start_roscore,
    "WaitForActiveTopicROS1": block_wait_for_active_topic_ros1,
    "WaitForActiveTopicROS2": block_wait_for_active_topic_ros2,
    "StartNodeROS2": block_start_node_ros2,
    "LaunchROS1": block_launch_ros1,
    "StartROS1Bridge": block_start_ros1_bridge,
    "StartLogging": block_start_logging,
    "StopLogging": block_stop_logging,
}
