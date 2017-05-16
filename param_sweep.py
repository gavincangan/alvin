#!/usr/bin/env python

import sys, os, shutil, math
from math import pi
from colorama import init, Fore, Style
from configsingleton import ConfigSingleton

def execute(cmd):
    # Print the command to execute in green
    print(Fore.GREEN + cmd)
    print(Style.RESET_ALL)
    os.system(cmd)

config = ConfigSingleton.get_instance('default.cfg')

#linear_speeds = [x / 2.0 for x in range(2, 10 + 1)]
#print linear_speeds

#angular_speeds = [x / 2.0 for x in range(1, 10 + 1)]
#print angular_speeds

#slow_factors = [x / 4.0 for x in range(1, 4 + 1)]
#print slow_factors

#front_angle_thresholds = [(pi/4)*(i/10.0) for i in range(0, 11)]
#print front_angle_thresholds

#number_robots = [i for i in range(2, 6)]
number_robots = [5]
print number_robots

output_dir = '/tmp/param_sweep'
shutil.rmtree(output_dir, ignore_errors=True)
os.mkdir(output_dir)

number_trials = 3

for nr in number_robots: 
    #config.set("GauciController", "linear_speed", l)
    #config.set("GauciController", "angular_speed", a)
    #config.set("GauciController", "front_angle_threshold", fat)
    config.set("AlvinSim", "number_robots", nr)

    filename_base = "{}/{}".format(output_dir, nr)
    filename = filename_base.replace('.', 'p') + '.cfg'

    config.write(open(filename, 'w'))

    for trial in range(number_trials):
        execute("alvin.py {} {}".format(filename, trial))

