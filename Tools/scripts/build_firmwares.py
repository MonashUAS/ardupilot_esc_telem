#!/usr/bin/env python3

'''
AP_FLAKE8_CLEAN

WIP
Script to generate all Harris Aerial Firmwares

Assumes: Branches exist already for now locally


TO DO: We should not use waf except once on each branch
    In the for loop for each model we should just use the apj_tool and make the apj tool accept
    inheritted default.parm files in the hwdef directories

How to use?
Starting in the ardupilot directory.
~/ardupilot $ python Tools/scripts/build_firmwares.py --outdir="Harris/firmware"

Options:
        --outdir   : Specify the directory where each built firmware is placed
        --no-clean : Do not remove output directory folder and do not call "waf distclean" between builds
        --build-rid: Build each hwdef with an additional -RID string
                     Example: Build CubeOrange-Harris  and additonally build CubeOrange-Harris-RID

Output is placed into ./[OUT_DIRECTORY}/[BRANCH_NAME]/[FIRMWARE_VEHICLE_NAME].apj
'''

import optparse
import os
import shutil
import string
import subprocess
import sys
import time

if sys.version_info[0] < 3:
    raise Exception("Must use Python3. Time for an upgrade!!")
    running_python3 = False
else:
    running_python3 = True


# Branch Names

# [Branch_Name, RID_Compatible]
# could just check firmware version string somewhere??? is 4.2.3 or  greater?
branch_prefix = 'Harris_Aerial'
branches = {('all_vehicles-4.2.0', False),
            ('all_vehicles-4.2.3', True),
            # ('pr_add_RID_hwdefs', True)
            # ('pr-icm20602_fast_reset_backport_copter_4_2_0', false),
            # (`pr_icm20602_fast_reset_backport_copter_4_2_3`, true),
            }

# branches = {'all_vehicles-4.2.0',
#             'all_vehicles-4.2.3',
#             # ['pr-icm20602_fast_reset_backport_copter_4_2_0', false],
#             # [`pr_icm20602_fast_reset_backport_copter_4_2_3`, true],
#             }


hwdef_prefix = 'CubeOrange-Harris-'
hwdef_models = {'H6-HL',
                'H6-Hydrone',
                'H6-Loweheiser',
                'HX8-Electric',
                }


# branches = {'all_vehicles-4.2.0',
#             'all_vehicles-4.2.3',
#             # ['pr-icm20602_fast_reset_backport_copter_4_2_0', false],
#             # [`pr_icm20602_fast_reset_backport_copter_4_2_3`, true],
#             }


# vast amounts of stuff copied into here from build_binaries.py & size_compare_branches.py
class Build_Firmwares(object):
    def __init__(self, outdir=None, build_rid=True, no_clean=False):
        self.outdir = outdir
        self.build_rid = build_rid
        self.no_clean = no_clean

        self.branch_prefix = branch_prefix
        self.branches = branches
        self.hwdef_prefix = hwdef_prefix
        self.hwdef_models = hwdef_models

        if outdir is None:
            self.outdir = self.branch_prefix

        self.bin_dir = self.find_bin_dir()

    def find_bin_dir(self):
        '''attempt to find where the arm-none-eabi tools are'''
        binary = shutil.which("arm-none-eabi-g++")
        if binary is None:
            raise Exception("No arm-none-eabi-g++?")
        return os.path.dirname(binary)

    def run_program(self, prefix, cmd_list, show_output=True, env=None):
        if show_output:
            self.progress("Running (%s)" % " ".join(cmd_list))
        p = subprocess.Popen(cmd_list, bufsize=1, stdin=None,
                             stdout=subprocess.PIPE, close_fds=True,
                             stderr=subprocess.STDOUT, env=env)
        output = ""
        while True:
            x = p.stdout.readline()
            if len(x) == 0:
                returncode = os.waitpid(p.pid, 0)
                if returncode:
                    break
                    # select not available on Windows... probably...
                time.sleep(0.1)
                continue
            if running_python3:
                x = bytearray(x)
                x = filter(lambda x : chr(x) in string.printable, x)
                x = "".join([chr(c) for c in x])
            output += x
            x = x.rstrip()
            if show_output:
                print("%s: %s" % (prefix, x))
        (_, status) = returncode
        if status != 0 and show_output:
            self.progress("Process failed (%s)" %
                          str(returncode))
            raise subprocess.CalledProcessError(
                returncode, cmd_list)
        return output

    def run_git(self, args):
        '''run git with args git_args; returns git's output'''
        cmd_list = ["git"]
        cmd_list.extend(args)
        return self.run_program("SCB-GIT", cmd_list)

    def run_waf(self, args, compiler=None):
        if os.path.exists("waf"):
            waf = "./waf"
        else:
            waf = os.path.join(".", "modules", "waf", "waf-light")
        cmd_list = [waf]
        cmd_list.extend(args)
        env = None
        if compiler is not None:
            # default to $HOME/arm-gcc, but allow for any path with AP_GCC_HOME environment variable
            gcc_home = os.environ.get("AP_GCC_HOME", os.path.join(os.environ["HOME"], "arm-gcc"))
            gcc_path = os.path.join(gcc_home, compiler, "bin")
            if os.path.exists(gcc_path):
                # setup PATH to point at the right compiler, and setup to use ccache
                env = os.environ.copy()
                env["PATH"] = gcc_path + ":" + env["PATH"]
                env["CC"] = "ccache arm-none-eabi-gcc"
                env["CXX"] = "ccache arm-none-eabi-g++"
            else:
                raise Exception("BB-WAF: Missing compiler %s" % gcc_path)
        self.run_program("SCB-WAF", cmd_list, env=env)

    def progress(self, string):
        '''pretty-print progress'''
        print("SCB: %s" % string)

    def build_branch_into_dir(self, board, branch, vehicle, outdir):
        self.run_git(["checkout", branch])
        self.run_git(["submodule", "update", "--recursive"])

        shutil.rmtree("build", ignore_errors=True)
        self.run_waf(["configure", "--board", board])
        self.run_waf([vehicle])
        shutil.rmtree(outdir, ignore_errors=True)
        shutil.copytree("build", outdir)

    def build_hwdef_models(self, hwdef, outdir_branch_name):
        full_hwdef = self.hwdef_prefix + hwdef
        # try:
        self.run_waf(["configure", "--board", full_hwdef])
        # except:
        #     self.progress("failed to configure waf for" + full_hwdef)

        # build for the given vehicle type
        self.run_waf(["copter"])

        # Move the built firmwares to the branch output directory and rename
        firmware_output_name = full_hwdef + "_Copter" + ".apj"

        # gets the current working dir
        src_dir = os.getcwd()

        # moving
        src_vehicle_apj_location = src_dir + "/build/" + full_hwdef + "/bin/" + "arducopter.apj"
        dest_vehicle_apj_location = self.outdir + "/" + outdir_branch_name + "/" + firmware_output_name
        shutil.move(src_vehicle_apj_location, dest_vehicle_apj_location)

    def run(self):
        self.progress('Starting firmware generation')

        # Print List of Branches Here

        # Print List of Hwdefs models here

        # Remove previous firmware output folder
        if self.no_clean is False:
            self.progress("Cleaning previous firmware folder")
            shutil.rmtree(self.outdir, ignore_errors=True)

        # Create the top-level output directory
        if not os.path.exists(self.outdir):
            os.mkdir(self.outdir)

        # First Step Build Bootloaders if asked

        for (branch, rid_compatible) in branches:

            # Probably should always clean between branches!
            if self.no_clean is False:
                # Clean the build folder when switching branches
                self.run_waf(["distclean"])

            full_branch_name = self.branch_prefix + "/" + branch
            # try:
            self.run_git(["checkout", full_branch_name])
            self.run_git(["submodule", "update", "--recursive"])

            # FIXME should check that git status returns 0 changes, especially submodules or just run
            # ./Tools/gittools/submodule_update.sh

            # except Exception as e:
            #     self.progress(e)
            #     self.progress("GIT Checkout of" + full_branch_name + "failed")

            # Create the branch output directory where each vehicle's firmware will be placed
            # FIXME Poor method of cleaning branch names.....
            outdir_branch_name = branch.replace(".", "_")
            if not os.path.exists(self.outdir + "/" + outdir_branch_name):
                os.mkdir(self.outdir + "/" + outdir_branch_name)

            # Build firmwares for each hwdef
            # FIXME this should call the apjtool.py after the first build of a branch!!! If not a RID firmware
            for hwdef in self.hwdef_models:
                self.build_hwdef_models(hwdef, outdir_branch_name)

                # Remote RID models
                if (self.build_rid is True) and (rid_compatible is True):
                    self.build_hwdef_models(hwdef + '-' + 'RID', outdir_branch_name)


if __name__ == '__main__':

    parser = optparse.OptionParser("build_firmwares.py")
    parser.add_option("",
                      "--outdir",
                      type="string",
                      default=None,
                      help="Top level output directory")
    parser.add_option("",
                      "--build-rid",
                      action='store_true',
                      default=False,
                      help="Build Remote ID Hwdefs",
                      dest="build_rid",
                      )
    parser.add_option("",
                      "--no-clean",
                      default=False,
                      action='store_true',
                      help="do not clean before building",
                      dest="no_clean",
                      )

    cmd_opts, cmd_args = parser.parse_args()

    x = Build_Firmwares(
        outdir=cmd_opts.outdir,
        build_rid=cmd_opts.build_rid,
        no_clean=cmd_opts.no_clean,
    )

    x.run()
