#!/usr/bin/env python

# import os
# import sys

# import argparse

# python

file_table = ["H6_HL_Copter_4_2_complete",
              "H6_RichenPower_Copter_4_2_complete",
              "H6_Lowheiser_Copter_4_2_complete",
              "H6_Hydrone_Copter_4_2_complete",
              "HX8_Electric_Copter_4_2_complete",
              ]
for filename in file_table:
    '''set defaults to contents of a file'''
    print("Setting defaults from %s" % filename)
    f = open(filename + '.parm', 'r')
    # contents = f.read()
    # f.close()

    contents_new = []
    for line in f:
        if line.startswith("#"):
            continue
        elif line.startswith("\n"):
            continue
        # contents_new.append(line)

        f_output = open(filename + '_stripped' + '.parm', "a")
        f_output.write(line)

    f_output.write("\n")
    f_output.close()

# python Tools/scripts/apj_tool.py --set-file ../partner/Harris_Aerial/parameter_lists/complete_lists/H6_Lowheiser_Copter_4_2_complete_stripped.parm build/CubeOrange/bin/arducopter.apj

# python ~/ardupilot/Tools/scripts/apj_tool.py --set-file H6_RichenPower_Copter_4_2_complete_stripped.parm ~/ardupilot/build/sitl/bin/arducopter
# python ~/ardupilot/Tools/scripts/apj_tool.py --set-file H6_RichenPower_Copter_4_2_complete_stripped.parm ~/ardupilot/build/CubeOrange/bin/arducopter.apj

# Call the APJ Tools

# Program structure

# 1  generate complete parameter lists from split list

# all_vehicle parms
#  frame type H6, H6_HL, HX8
# energy type hydrone, lowheiser, fully_electric, richen_power
# combine all together to generate complete parameter lists with comments

# 2 Strip out comments from complete parameter lists


# 3 Gather firmware files or build firmware files as needed

# 4 Insert Default parameter files into the firmware files
#   rename with appropriate parameter list name

# 5 zip lists and firmwares up

# apj tools

# get firmware files
