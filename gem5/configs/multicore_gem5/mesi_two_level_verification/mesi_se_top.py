# Copyright (c) 2023 Songzhu Zhang
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
******************************************************************************
*                                                                            *
* SE Test config for MESI Directory Chip Multi-Processor (CMP).              *
*                                                                            *
******************************************************************************

Author: Songzhu Zhang
Date: 2023/04/03

Description:
CMP refers to a type of multi-core processor design where multiple processing
units (cores) share a single physical chip, as opposed to multiple physical
chips connected via an interconnect. In this design, each core has access to
its own L1 cache, but multiple cores may share a higher-level cache or memory.

NOTE: To run the multi-threaded C++ program using this config, one must ensure
that the `network_class` option is `SimplePt2Pt`. Other network classes are
not supported by the gem5 SE mode.

Usage:
build/X86_mesi/gem5.opt configs/multicore_gem5/mesi_two_level_verification/mesi_se_top.py [options]=[arguments]

Options:
-h, --help     Show help message and exit
-v, --verbose  Enable verbose mode

Examples:
build/X86_debug/gem5.debug configs/multicore_gem5/mesi_two_level_verification/mesi_se_top.py --max_mem_reqs=-1
build/X86_debug/gem5.debug configs/multicore_gem5/mesi_two_level_verification/mesi_se_top.py --max_mem_reqs=-1 --network_class=GarnetPt2Pt
build/X86_debug/gem5.debug --debug-flags=RubySlicc configs/multicore_gem5/mesi_two_level_verification/mesi_se_top.py
build/X86_debug/gem5.debug --debug-flags=ProtocolTrace configs/multicore_gem5/mesi_two_level_verification/mesi_se_top.py
"""

import m5
from m5.objects import *
from m5.defines import buildEnv
from m5.util import addToPath
import os, argparse, sys, time
from m5.stats.gem5stats import get_simstat

from pprint import pprint
from cache_system_se import CacheSystemSE
#import pdb; pdb.set_trace()

# Needed for running C++ threads
m5.util.addToPath("../../")
from common.FileSystemConfig import config_filesystem

if buildEnv["PROTOCOL"] != "MESI_Two_Level":
    fatal("This system assumes MESI_Two_Level!")

# Sets up a command-line argument parser using the argparse module
parser = argparse.ArgumentParser()

parser.add_argument(
    "--sys_clock",
    action="store",
    type=str,
    default="4GHz",
    help="Top-level clock for blocks running at system speed"
)

parser.add_argument(
    "--network_class",
    action="store",
    type=str,
    default="SimplePt2Pt",
    help="Garnet topology for on-chip interconnection network",
    choices=["GarnetPt2Pt", "SimplePt2Pt", "GarnetMesh_XY"]
)

parser.add_argument(
    "--sys_voltage",
    action="store",
    type=str,
    default="1.0V",
    help="Top-level voltage for blocks running at system power supply",
)

parser.add_argument("-n", "--num_cpus", type=int, default=4)

parser.add_argument("--l1i_size", type=str, default="32kB")
parser.add_argument("--l1i_assoc", type=int, default=8)

parser.add_argument("--l1d_size", type=str, default="32kB")
parser.add_argument("--l1d_assoc", type=int, default=8)

parser.add_argument("--l2_size", type=str, default="256kB")
parser.add_argument("--l2_assoc", type=int, default=16)
parser.add_argument("--num_l2Caches", type=int, default=1)

"""
Parse the arguments and store the results in the `args` variable. This allows
the values of the command-line arguments to be easily accessed and used later
in the configuration script.
"""
args = parser.parse_args()

'''
Note that when ruby prefetcher is used, the following parameters must be
set to appropriate values (e.g., cannot be too small). Or, the error message
`packet.hh:1214: T* gem5::Packet::getPtr() [with T = unsigned char]:
Assertion `flags.isSet(STATIC_DATA|DYNAMIC_DATA)' failed` will be encountered.
'''
args.l1d_size = "16kB"
args.l1d_assoc = 2

args.l1i_size = "16kB"
args.l1i_assoc = 2

args.l2_size = "128kB"
args.l2_assoc = 4

print("num_cpus: ", args.num_cpus)
print("num_l2Caches: ", args.num_l2Caches)
print("l1d_size: ", args.l1d_size)
print("l1d_assoc: ", args.l1d_assoc)
print("l1i_size: ", args.l1i_size)
print("l1i_assoc: ", args.l1i_assoc)
print("l2_size: ", args.l2_size)
print("l2_assoc: ", args.l2_assoc)

# Create the system we are going to simulate
system = System()

# Create a top-level voltage domain and clock domain
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = args.sys_clock
system.clk_domain.voltage_domain = VoltageDomain(voltage=args.sys_voltage)

# Any memory requests made by the simulated system will be directed to the
# memory range specified by `args.mem_size`.
system.mem_mode = "timing"  # Use timing accesses
system.mem_ranges = [AddrRange("2GB")]  # Create an address range

# Create the CPU objects
# NOTE: DerivO3CPU is not supported by the SE mode.
system.cpu = [X86TimingSimpleCPU() for i in range(args.num_cpus)]

# Create a DDR3 memory controller and connect it to the membus
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]

print(f"Cache line size of the system: {system.cache_line_size}")

# Must instantiate interrupt controllers before instantiating the Ruby cache
# system for the X86 gem5 simulator to work.
# The interrupt controllers receive interrupt responses from its corresponding
# Ruby sequencer and send interrupt requests to the Ruby sequencer.
# These connections are made automatically.
for cpu in system.cpu:
    cpu.createInterruptController()

# Create the Ruby System which must be done after the CPU interrupt
# controllers have been created.
system.caches = CacheSystemSE()
system.caches.setup(
    options=args,
    system=system,
    network_class=args.network_class
)

# os.path.dirname returns the parent directory of the specified path.
# config_path = the absolute path of the directory containing the config file
config_path = os.path.dirname(os.path.abspath(__file__))
print("Dreictory path of the config file: ", config_path)

# config_root = the absolute path of the parent directory of `config_path`
config_root = os.path.dirname(config_path)
print("Config root: ", config_root)

m5_root = os.path.dirname(config_root)
print("m5 root: ", m5_root)

binary = os.path.join(
    config_root,
    "../../",
    "tests/test-progs/threads/bin/x86/linux/threads"
)
print(f"Path of the binary: {binary}")

# -----------------------
#   Run Simulation
# -----------------------
# Create a process for a simple "multi-threaded" application
process = Process()

# cmd is a list which begins with the executable (like argv)
process.cmd = [binary]

# Set the cpu to use the process as its workload and create thread contexts
for cpu in system.cpu:
    cpu.workload = process
    cpu.createThreads()

system.workload = SEWorkload.init_compatible(binary)

# Set up the pseudo file system for the threads function above
config_filesystem(system)

# Set up the root SimObject and start the simulation
root = Root(full_system=False, system=system)

# Instantiate all of the objects we've created above
m5.instantiate()

globalStart = time.time()

print("Start running simulation ...")
exit_event = m5.simulate()

if exit_event.getCause() == "exiting with last active thread context" or \
    exit_event.getCause() == "user interrupt received":

    print("Dump stats at the end of the ROI!")
    m5.stats.dump()
else:
    print("Unexpected termination of simulation while ROI was being executed!")
    print(
        "Exiting @ tick {} because {}.".format(
            m5.curTick(),
            exit_event.getCause()
        )
    )
    exit(-1)

gem5stats = get_simstat(root)

# Simulation is over at this point. We acknowledge that all the simulation
# events were successful.
print("All simulation events were successful.")

print("Done with the simulation")
print()
print("Performance statistics:")

print("Ran a total of", m5.curTick()/1e12, "simulated seconds")
print("Total wallclock time: %.2fs, %.2f min" % \
            (time.time()-globalStart, (time.time()-globalStart)/60))