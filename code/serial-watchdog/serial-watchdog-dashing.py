#!/usr/bin/env python


from subprocess import Popen
from os import walk
from time import sleep
import serial
import os

dds_binary_path = "/usr/local/bin/MicroXRCEAgent"

baudrate = "115200"



def get_mavid():
    ret = "0"
    home = os.getenv("HOME")
    try:
        with open("%s/mavid.txt"%home, "r") as f:
            ret = f.readline().replace("\n", "")
        print("Mavid=%s"%ret)
    except:
        print("No mavid.txt file found in home directory. Using default (0)")

    return ret

def main():
    # Read mavid
    mavid =get_mavid()

    serial_ports = dict()
    old_ports = None

    while True:
        rate = 2   # Hz
        # Search for disconnected serial ports
        # Search for all serial ports
        f = []
        for (dirpath, dirnames, filenames) in walk('/dev/'):
            f.extend(filenames)
            break
        available_ports = list(
            #filter(lambda x: 'ttyUSB' in x or 'ttyACM' in x, f))
            filter(lambda x: 'ttyUSB' in x, f))
        if old_ports == None:
                removed_ports = list()
        else:
            removed_ports = old_ports - available_ports

        # kill all agents with no serial port
        for portstr in removed_ports:
            print("Port " + portstr + "no longer available, killing!")
            serial_ports[portstr].terminate()

        # Clear all serial ports with dead Agents
        newdict = dict(serial_ports)
        for portstr, handle in serial_ports.iteritems():
            if(handle.poll() is not None):    # Process finished
                print(portstr+" died")
                del newdict[portstr]
        serial_ports = newdict

        # Start ros node for ports that are not in the list and add them
        for portstr in available_ports:
            if portstr not in serial_ports:
                # tell the sensor our mavid first
                ser = serial.Serial('/dev/'+portstr, 115200)  # open serial port
                ser.write(mavid)
                ser.close()

                # start rosserial node
#                print("%s %s %s %s %s"%(dds_binary_path, '--serial', '/dev/'+portstr, '--baudrate', baudrate))    # tell the world

                serial_ports[portstr] = Popen([dds_binary_path, 'serial','--dev', '/dev/'+portstr, '-b', baudrate])
                print(portstr+" connected!")    # tell the world

        sleep(1.0/rate)
    # kill all serial processes
    for name, process in serial_ports:
        process.terminate()
        print(name+" terminated")


if __name__ == '__main__':
    main()
