#!/usr/bin/env python

import os
import sys
import time
import argparse
from time import sleep

import numpy as np

import lcm
from scout_lcm_msgs import *

class ScoutCommander(object):
    def __init__(self, lcm_h):
        self.lcm_h = lcm_h

    def publish_scout_command(self):
        print "publish scout command"
        cmd_msg = ScoutCommand_t()

        cmd_msg.linear = 1.0
        cmd_msg.angular = 0

        self.lcm_h.publish("SCOUT_LCM_CMD", cmd_msg.encode())

        print "command published"

    # def map_request_handler(self, channel, data):
        # msg = MapRequest_t.decode(data)        
        # print("Received message on channel \"%s\"" % channel)
        
        # self.set_env_size(msg.map_size_x, msg.map_size_y, msg.map_size_z)
        
        # self.generate_space()
        # self.publish_map()

def main():
    print("started scout commander")

    # create a LCM instance
    lc = lcm.LCM()    
    car_cmd = ScoutCommander(lc)
    car_cmd.publish_scout_command()

    # subscription = lc.subscribe("envsim/map_request", gen.map_request_handler)
    
    # try:
    #     while True:
    #         lc.handle()
    # except KeyboardInterrupt:
    #     pass

    # lc.unsubscribe(subscription)


# Standard boilerplate to call the main() function to begin
# the program.
if __name__ == '__main__':
    main()
