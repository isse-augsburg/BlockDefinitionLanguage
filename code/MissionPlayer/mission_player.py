#!/usr/bin/env python3
import json
import time
import random
import threading
import ctypes
import sys
import os
class MissionThread(threading.Thread):
    def __init__(self, mission, start, previous):
        threading.Thread.__init__(self)
        self.start_block = start
        self.mission = mission
        self.previous = previous

    def run(self):
        # target function of the thread class
        try:
            current = self.start_block
            previous = self.previous
            while True:
                print("Current: %s (%s)" % (current, self.mission[current]['type']))
                tmp = current
                current = exec_block(self.mission, current, previous)
                previous = tmp
                if current is None:
                    return
        finally:
            print('branch %s ended' % self.start_block)

    def get_id(self):

        # returns id of the respective thread
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for thread_id, thread in threading._active.items():
            if thread is self:
                return thread_id

    def kill(self):
        thread_id = self.get_id()
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id,
                                                         ctypes.py_object(SystemExit))
        if res > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0)
            print('Exception raise failure')


def block_decision_random(args):
    if random.random() < float(args['chanceForTrue']):
        return args['nextIfTrue']
    else:
        return args['nextIfFalse']

def block_start(args):
    #print("Tried to execute START Block. This should not have happened")
    print("If you're seeing this, the code is in what I thougt was an unreachable state.")
    print("I could give you advice for what to do.")
    print("But honestly, why should you trust me?")
    print("I clearly screwed this up.")
    print("I'm writing a message that should never appear,")
    print("yet I know it will pobably appear someday.")
    print("On a deep level, I know I'm not up to this task.")
    print("I'm sorry.")
    return args['next']

def block_fork(args):
    print("Tried to execute FORK Block. This should not have happened")
    return args['next']

def block_join(args):
    print("Tried to execute JOIN Block. This should not have happened")
    return args['next']


def block_end(args):
    print("Tried to execute END Block. This should not have happened")
    return args['next']


def block_wait(args):
    time.sleep(float(args['ms'])/1000.0)
    return args['next']

switcher = {
    "Wait": block_wait,
    "Start": block_start,
    "Fork": block_fork,
    "Join": block_join,
    "End": block_end,
    "DecisionRandom": block_decision_random,
    }


joinsReached = dict()
joinsNeeded = dict()
threads = list()
ended = False

def get_mavid():
    ret = 0
    home = os.getenv("HOME")
    try:
        with open("%s/mavid.txt"%home, "r") as f:
            ret = int(f.readline())
        print("Mavid=%d"%ret)
    except:
        print("No mavid.txt file found in home directory. Using default (0)")

    return ret


id = get_mavid()

variables = {
    "$mavid": str(id),
    "$time": str(int(time.time()))
}

def parse_variables(args):
    for k, v in args.items():
        if isinstance(v,dict):
            parse_variables(args[k])
        elif isinstance(v, list):
            for variable, value in variables.items():
                args[k] = [i.replace(variable, value) for i in args[k]]

        else:
            for variable, value in variables.items():
                args[k] = args[k].replace(variable, value)





def add_block(block_type, function):
    global switcher
    switcher[block_type] = function


def get_func(block_type):
    func = switcher.get(block_type)
    if func is None:
        print("Error: requested Block not found: " + block_type)
    return func


def get_start_node(blocks):
    for block in blocks:
        if blocks[block]['type'] == "Start":
            return blocks[block]
    print("Error: No Start found")
    return None


def do_join_initialization(mission):
    global joinsNeeded, joinsReached

    # Search all joins
    joins = list()
    for block in mission:
        if mission[block]["type"] == "Join":
            joins.append(block)

    # Initialize joins needed and joins reached
    for join in joins:
        joinsNeeded[join] = set()
        joinsReached[join] = set()
    # Search all references to all joins
    for join in joins:
        for block in mission:
            if mission[block]['type'] != 'End' and mission[block]['next'] == join:
                # print("%s is predecessor of join %s" % (block, join))
                joinsNeeded[join].add(block)
    print("Joins Needed: %s" % joinsNeeded)


def join_completely_reached(join):
    # compare list contents of joinsNeeded[join] and joinsReached[join],
    return joinsNeeded[join].difference(joinsReached[join]) == set()


def play_mission(filename):
    global ended
    # Read Mission File
    with open(filename) as json_file:
        data = json.load(json_file)
        mission = data['mission']

    parse_variables(mission)

    # Initialize all the Things!
    do_join_initialization(mission)
    start = get_start_node(mission)
    ended = False
    random.seed(time.time()) # This is not secure, change for any security related random decisions


    # Start at the Start Block
    current = start['next']
    mt0 = MissionThread(mission, current, None)
    threads.append(mt0)
    mt0.start()
    while not ended:
        time.sleep(0.2) # TODO maybe do something smarter than polling here
    print("Mission Complete!")


def exec_block(mission, current, previous):
    global threads, ended
    # print(current_data)
    current_data = mission[current]
    if current_data['type'] == 'Fork':

        for nxt in current_data['next']:
            thread = MissionThread(mission, nxt, current)
            thread.start()
            threads.append(thread)

        return None
    # If we reached a join, update joinsReached
    elif current_data['type'] == 'Join':
        joinsReached[current].add(previous)

        # If we can continue, continue, else die
        if join_completely_reached(current):
            print('join  %s completely reached!' % current)
            return current_data['next'] # Is always a join block so we
                                        # can manually access next
        else:
            # Kill the Thread
            return None
    elif current_data['type'] == 'End':
        # The end is near, kill everything and everyone!
        for thread in threads:
            thread.kill()
        ended = True
        return None
    else:
        # Execute Blocks that actually do something
        func = get_func(current_data['type'])
        ret = func(current_data)
        return ret


def add_plugin(blocks):
    for type, function in blocks.items():
        add_block(type, function)

def main():

    # Play Mission
    if len(sys.argv) == 2:
        play_mission(sys.argv[1])
    else:
        play_mission("./join_test_mission.json")


if __name__ == "__main__":
    main()
