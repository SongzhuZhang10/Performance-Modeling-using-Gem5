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
***********************************************
*                                             *
*          My Python Program's Name           *
*                                             *
***********************************************

Author: Your Name
Date: Today's date

Description:
This program does something useful.

Usage:
python my_program.py [options] [arguments]

Options:
-h, --help     Show help message and exit
-v, --verbose  Enable verbose mode

Examples:
python my_program.py -v
"""

from m5.defines import buildEnv
from m5.util import fatal, addToPath
from m5.objects import *

from pprint import pprint

import math

from typing import List, Optional, Sequence, Tuple

from gem5.coherence_protocol import CoherenceProtocol

from gem5.utils.requires import requires

from l1_cache import L1Cache
from l2_cache import L2Cache
from directory import Directory
from simple_pt2pt import SimplePt2Pt
from garnet_mesh_XY import GarnetMesh_XY
from garnet_pt2pt import GarnetPt2Pt

class CacheSystemSE(RubySystem):
    def __init__(self):
        super().__init__()

        if buildEnv["PROTOCOL"] != "MESI_Two_Level" and buildEnv["PROTOCOL"] != "MESI_Two_Level_Pythia":
            fatal("The system assumes MESI_Two_Level with or without Pythia prefetcher(s)!")

    def setup(self, options, system, network_class):
        """
        Set up the Ruby cache subsystem. Note: This can't be done in the
        constructor because many of these items require a pointer to the
        ruby system (self). This causes infinite recursion in initialize()
        if we do this in the __init__.
        Setting up for running the RubyRandomTester is a little different
        than when we're using CPUs.
        """

        #requires(coherence_protocol_required=CoherenceProtocol.MESI_Two_Level_Pythia)

        print(f"The network class for the Ruby system is {network_class}.")

        # Ruby's global network.
        if network_class == 'SimplePt2Pt':
            self.network = SimplePt2Pt(self)
        elif network_class == 'GarnetPt2Pt':
            self.network = GarnetPt2Pt(self)
        elif network_class == 'GarnetMesh_XY':
            self.network = GarnetMesh_XY(self)
        else:
            raise ValueError(f"network {network_class} is not implemented.")

        self.number_of_virtual_networks = 5
        self.network.number_of_virtual_networks = 5

        #######################################################
        #                                                     #
        #                 Set Up Clock Domains                #
        #                                                     #
        #######################################################
        self.clk_domain = system.clk_domain

        """
        Set the clk domain of the memory controller in the Ruby memory system.
        This clock domain is used to specify the timing relationship between
        the memory controller and the rest of the system, such as the L1 and
        L2 caches. 
        """
        self.memctrl_clk_domain = DerivedClockDomain(
            clk_domain=self.clk_domain, clk_divider=4
        )

        l1_cache_clk_domain = DerivedClockDomain(
            clk_domain=self.clk_domain, clk_divider=1
        )

        l2_cache_clk_domain = DerivedClockDomain(
            clk_domain=self.clk_domain, clk_divider=1.5
        )

        dir_clk_domain = DerivedClockDomain(
            clk_domain=self.clk_domain, clk_divider=3
        )

        #######################################################
        #                                                     #
        #                 Set Up L1 Cache                     #
        #                                                     #
        #######################################################
        l1_ctrls = []

        for i in range(options.num_cpus):
            l1_cache = L1Cache(
                l1i_size=options.l1i_size,
                l1i_assoc=options.l1i_assoc,
                l1d_size=options.l1d_size,
                l1d_assoc=options.l1d_assoc,
                num_l2Caches=options.num_l2Caches,
                cache_line_size=system.cache_line_size,
                ruby_system=self,
                clk_domain=l1_cache_clk_domain,
                enable_l1_prefetch=options.enable_l1_prefetch,
            )
            l1_ctrls.append(l1_cache)

        """
        Although the class `RubySystem` does not have the `sequencers`
        attribute, it is recommended to add this new attribute to this class
        because the generated dot file will look neater in the sense that
        the sequencers will be in the block of this class.
        If we don't do this, the sequencers will show up in their own L1 cache
        block in the dot file.
        """
        self.sequencers = [
            RubySequencer(
                version=i,
                dcache=l1_cache.L1Dcache,
                clk_domain=l1_cache_clk_domain,
                ruby_system=self,
            )
            for i in range(options.num_cpus)
        ]

        for i, c in enumerate(l1_ctrls):
            c.sequencer = self.sequencers[i]

        # Must configure the attribute of the profiler
        self.num_of_sequencers = len(self.sequencers)
        print("num_of_sequencers: ", self.num_of_sequencers)

        #######################################################
        #                                                     #
        #                 Set Up L2 Cache                     #
        #                                                     #
        #######################################################
        l2_ctrls = []
        for i in range(options.num_l2Caches):
            l2_cache = L2Cache(
                l2_size=options.l2_size,
                l2_assoc=options.l2_assoc,
                num_l2Caches=options.num_l2Caches,
                ruby_system=self,
                clk_domain=l2_cache_clk_domain,
                cache_line_size=system.cache_line_size,
            )
            l2_ctrls.append(l2_cache)
        """
        We don't need to create any sequencer for the L2 cache; this is 
        because, in its SLICC implementation, there are no Ruby sequencers
        being used.
        """

        #######################################################
        #                                                     #
        #                 Set Up Directory                    #
        #                                                     #
        #######################################################
        dir_ctrls = [
            Directory(
                ruby_system=self,
                cache_line_size=system.cache_line_size,
                mem_range=system.mem_ranges,
                port=system.mem_ctrl.port,
                clk_domain=dir_clk_domain,
            )
        ]

        #######################################################
        #                                                     #
        #       Create Network to Connect Controllers.        #
        #                                                     #
        #######################################################
        """
        To make the network object a seperate module that is connected to the
        controllers in the `CacheSystemSE` level, We should add the new
        attribute `all_ctrls` to the `RubySystem` object `self`. Both the
        network and controllers should be the attribute of the `RubySystem`
        object.
        If we don't do this, then all the controllers will be instantiated
        inside the instance of the SimplePt2Pt object, which can be reflected
        in the generated `config.dot` file.
        """
        self.all_ctrls = l1_ctrls \
            + l2_ctrls \
            + dir_ctrls

        # Create the network and connect the controllers.
        # NOTE: This is quite different if using Garnet!
        if (network_class == 'GarnetMesh_XY'):
            self.network.connectControllers(
                self.all_ctrls,
                len(l1_ctrls)
            )
        elif (network_class == 'GarnetPt2Pt'):
            self.network.connectControllers(
                self.all_ctrls
            )
        elif (network_class == 'SimplePt2Pt'):
            self.network.connectControllers(self.all_ctrls)
            self.network.setup_buffers()

        print(f"Cache line size of the Ruby memory: {system.cache_line_size}")

        # Set up a proxy port for the system_port. Used for load binaries and
        # other functional-only things.
        self.sys_port_proxy = RubyPortProxy()
        system.system_port = self.sys_port_proxy.in_ports

        # Connect cpu memory request output ports to the corresponding Ruby
        # sequencer input ports.
        for i, cpu in enumerate(system.cpu):
            self.sequencers[i].connectCpuPorts(cpu)