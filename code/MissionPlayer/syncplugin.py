from subprocess import Popen
import subprocess
import shlex
from tkinter import *
from tkinter import messagebox


scriptdir = './syncplugin/'


def block_sync_init(args):
    """ Starts the Swarm Synchronisation node

        Parameters:
        args (dict): Parsed json arguments for block as dict
            Possible keys:

            next: Next block in case of success
            [params_file]: Path to params.yaml file
        Returns:
        string: Id of next block to be executed
   """
    print("Starting Sync")
    popenargs = scriptdir+"sync_init.sh"

    if 'params_file' in args.keys():
            popenargs += ' __params:='
            popenargs += args['params_file']
    Popen(shlex.split(popenargs))

    return args['next']


def block_barrier(args):
    """ Starts the ros2 node

        Parameters:
        args (dict): Parsed json arguments for block as dict
            Possible keys:

            next: Next block in case of success
            id: ID of Barrier to be waited for
            [timeout]: opitonal timeout in ms
            [next_timeout]: Next block in case of timeout
        Returns:
        string: Id of next block to be executed
    """
    print("Waiting on SwarmBarrier %s" % (args['id']))

    popenargs = scriptdir+"swarm_barrier.sh %s" % args['id']

    if 'timeout' in args.keys():
        try:
            subprocess.call(shlex.split(popenargs),
                            timeout=float(args['timeout'])/1000.0)
        except TimeoutExpired:
            if 'next_timeout' in args.keys():
                return args['next_timeout']
            else:
                return args['next']
    else:
        subprocess.call(shlex.split(popenargs))
    return args['next']


def block_user_breakpoint(args):
    """ Starts ui Message Box and blocks until it is closed

        Parameters:
        args (dict): Parsed json arguments for block as dict
            Possible keys:

            [message]: Message to be displayed in the MessageBox
            [title]: Title of the MessageBox window
            next: Next block in case of success
            id: ID of Barrier to be waited for

        Returns:
        string: Id of next block to be executed
    """
    # Optional title
    title = "Program Paused"
    if 'title' in args.keys():
        title = args['title']

    # Optional Message
    message = "Program Paused"
    if 'message' in args.keys():
        title = args['message']

    messagebox.showinfo(title, message)

    return args['next']


blocks = {
    "SyncInit": block_sync_init,
    "Barrier": block_barrier,
    "UserBreakpoint": block_user_breakpoint
}
