


from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.memory.single_channel import SingleChannelDDR3_1600
from gem5.components.memory.multi_channel import DualChannelDDR3_1600
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.cpu_types import (
    CPUTypes,
    get_cpu_type_from_str,
    get_cpu_types_str_set,
)
from gem5.resources.resource import Resource
from gem5.resources.resource import FileResource
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires
from gem5.isas import ISA

import argparse

from m5.util import addToPath
import os, argparse, sys, time

from pprint import pprint

# Needed for debugging with PDB
import pdb


from gem5.components.cachehierarchies.classic.private_l1_private_l2_shared_l3_cache_hierarchy import (
    PrivateL1PrivateL2SharedL3CacheHierarchy,
)

requires(isa_required=ISA.X86)

cache_hierarchy = PrivateL1PrivateL2SharedL3CacheHierarchy(
    l1d_size="32KiB",
    l1i_size="32KiB",
    l2_size="256KiB",
    l3_size="2MiB"
)

valid_cpu = {
    "TimingSimpleCPU": CPUTypes.TIMING,
    "DerivO3CPU": CPUTypes.O3,
    "AtomicSimpleCPU": CPUTypes.ATOMIC,
    "MinorCPU": CPUTypes.MINOR
}

parser = argparse.ArgumentParser(
    description="A gem5 script for running simple binaries in SE mode."
)

# CPU can be atomic, o3, timing, or minor.
parser.add_argument(
    "--cpu",
    action="store",
    type=str,
    #default="DerivO3CPU",
    default="TimingSimpleCPU",
    required=False,
    choices=valid_cpu.keys(),
    help="The CPU type used."
)

parser.add_argument(
    "-n",
    "--num-cores",
    type=int,
    default=2,
    required=False,
    help="The number of CPU cores to run.",
)

args = parser.parse_args()

# Declare the rest of the components.
#memory = SingleChannelDDR3_1600("8GiB")
memory = DualChannelDDR3_1600("8GiB")

processor = SimpleProcessor(cpu_type=valid_cpu[args.cpu], isa=ISA.X86, num_cores=args.num_cores)

# Add them to the motherboard.
motherboard = SimpleBoard(
    clk_freq="3GHz", processor=processor, memory=memory, cache_hierarchy=cache_hierarchy
)

# os.path.dirname returns the parent directory of the specified path.
# config_path = the absolute path of the directory containing the config file
config_path = os.path.dirname(os.path.abspath(__file__))
print("Directory path of the config file: ", config_path)

# config_root = the absolute path of the parent directory of `config_path`
config_root = os.path.dirname(config_path)
print("Config root: ", config_root)

# Set the workload.
binary_path = os.path.join(
    config_root,
    "../../",
    "tests/test-progs/threads/bin/x86/linux/threads"
)

# More `Abstract Resource` subclasses can be found in src/python/gem5/resources/resource.py
binary = FileResource(binary_path)

motherboard.set_se_binary_workload(binary)

# Setup the Simulator and run the simulation.
simulator = Simulator(board=motherboard)

print("Start running simulation ...")
globalStart = time.time()
simulator.run()

print(
    "Exiting @ tick {} because {}.".format(
        simulator.get_current_tick(), simulator.get_last_exit_event_cause()
    )
)

print(
    "Ran a total of", simulator.get_current_tick() / 1e12, "simulated seconds"
)

print("Total wallclock time: %.2fs, %.2f min" % \
            (time.time()-globalStart, (time.time()-globalStart)/60))