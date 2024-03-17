# Copyright (c) 2017 Jason Power
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

""" This file creates a set of Ruby caches, the Ruby network, and a simple
point-to-point topology.
See Part 3 in the Learning gem5 book:
http://gem5.org/documentation/learning_gem5/part3/MSIintro

IMPORTANT: If you modify this file, it's likely that the Learning gem5 book
           also needs to be updated. For now, email Jason <jason@lowepower.com>

"""

import math

from m5.defines import buildEnv
from m5.util import fatal, panic

from m5.objects import *
from pprint import pprint
from m5.util import (
    fatal,
    panic,
)


class MyCacheSystem(RubySystem):
    def __init__(self):
        # Check the SCons variable PROTOCOL to ensure that
        # the right configuration file for the protocol is compiled.
        if buildEnv["PROTOCOL"] != "MSI":
            fatal("This system assumes MSI from learning gem5!")
        """We cannot create the controllers in the constructor because it
        will result in a circular dependence in the SimObject hierarchy,
        causing infinite recursion when being instantiated.        
        """
        super(MyCacheSystem, self).__init__()

    def setup(self, system, cpus, mem_ctrls):
        """Set up the Ruby cache subsystem. Note: This can't be done in the
        constructor because many of these items require a pointer to the
        ruby system (i.e., self). This causes infinite recursion in initialize()
        if we do this in the __init__.
        """
        # Ruby's global network.
        self.network = MyNetwork(self)

        # MSI uses 3 virtual networks. One for requests (lowest priority), one
        # for responses (highest priority), and one for "forwards" or
        # cache-to-cache requests. See *.sm files for details.
        self.number_of_virtual_networks = 3
        self.network.number_of_virtual_networks = 3

        # There is a single global list of all of the controllers.
        # It is used to make connecting everything to the global network easier.
        # This can be customized depending on the topology/network requirements.
        # Create one controller for each L1 cache (and the cache mem obj.)
        # Create a single directory controller (i.e., the memory controller) for the system.
        self.controllers = [L1Cache(system, self, cpu) for cpu in cpus] + [
            DirController(self, system.mem_ranges, mem_ctrls)
        ]

        # Create one sequencer per CPU. In many systems, this is more
        # complicated since you have to create sequencers for DMA controllers
        # and other controllers, too.
        """Each sequencer needs a pointer to the instruction and data cache to simulate
        the correct latency when initially accessing the cache.
        """
        self.sequencers = [
            RubySequencer(
                version=i,
                # I/D cache is combined and grab from ctrl
                dcache=self.controllers[i].cacheMemory,
                clk_domain=self.controllers[i].clk_domain,
            )
            for i in range(len(cpus))
        ]

        # After creating the sequencers, we set the sequencer variable on each L1 cache controller.
        # We put the controllers in an order such that the first
        # N (N=#CPUs) of them are the L1 caches which need a sequencer pointer.
        for i, c in enumerate(self.controllers[0 : len(self.sequencers)]):
            c.sequencer = self.sequencers[i]

        self.num_of_sequencers = len(self.sequencers)

        # Create the network and connect the controllers.
        # NOTE: This is quite different if using Garnet!
        # Connect all of the controllers to the network.
        self.network.connectControllers(self.controllers)
        
        # Call the setup_buffers function on the network.
        self.network.setup_buffers()

        # Set up a proxy port for the system_port.
        # The proxy port is used to load binaries and
        # other functional-only things.
        self.sys_port_proxy = RubyPortProxy()
        system.system_port = self.sys_port_proxy.in_ports

        # Connect the cpu's cache, interrupt, and TLB ports to Ruby
        """We assume that there are only CPU sequencers. So, the first CPU is
        connected to the first sequencer, and so on. We also have to connect
        the TLBs and interrupt ports (if we are using x86).
        """
        for i, cpu in enumerate(cpus):
            self.sequencers[i].connectCpuPorts(cpu)


class L1Cache(L1Cache_Controller):

    # variable to track the version number
    _version = 0

    # Decorator to create a class method
    @classmethod
    def versionCount(cls):
        # You have to number each SLICC state machine in ascending order from 0.
        # Each machine of the same type should have a unique version number.
        cls._version += 1  # Use count for this particular type
        return cls._version - 1

    def __init__(self, system, ruby_system, cpu):
        """Here, we set all of the parameters that we named in
        the state machine file (e.g., cacheMemory).
        """
        super().__init__()

        self.version = self.versionCount()
        
        # This is the cache memory object that stores the cache data and tags.
        # Hardcode the block size and associativity of the cache.
        self.cacheMemory = RubyCache(
            size="16kB", assoc=8, start_index_bit=self.getBlockSizeBits(system)
        )
        
        self.clk_domain = cpu.clk_domain

        self.send_evictions = self.sendEvicts(cpu)
        """
        Must assign `ruby_system` to the `self` instance of the class. Or, an error message
        "Error in unproxying param 'ruby_system' of system.caches.controllers0.cacheMemory"
        will be printed.
        """
        self.ruby_system = ruby_system
        self.connectQueues(ruby_system)
        

    # This function returns the number of bits in byte offset field.
    def getBlockSizeBits(self, system):
        bits = int(math.log(system.cache_line_size, 2))
        if 2**bits != system.cache_line_size.value:
            panic("Cache line size not a power of 2!")
        return bits

    def sendEvicts(self, cpu):
        """ Here, we decide whether to send eviction notices to the CPU.
        True if the CPU model or ISA requires sending evictions from caches
        to the CPU. Two scenarios warrant forwarding evictions to the CPU:
        1.  Use the O3 CPU. The O3 model must keep the LSQ coherent with the caches.
        2.  Use x86 ISA. The x86 mwait instruction is built on top of coherence.    
        3.  Use ARM ISA. The local exclusive monitor in ARM systems.

        As this is an X86 simulation, we return True.
        """
        return True

    def connectQueues(self, ruby_system):
        """Connect all of the message buffers (i.e., queues) to the Ruby network
        for this L1 cache controller.
        """
        # mandatoryQueue is a special variable. It is used by the sequencer to
        # send RubyRequests originated from the CPU (or other processor)
        # to cache controllers. It isn't
        # explicitly connected to anything.
        # Create a message buffer for the mandatory queue.
        self.mandatoryQueue = MessageBuffer()

        # All message buffers must be created and connected to the
        # general Ruby network. In this case, "in_port/out_port" don't
        # mean the same thing as normal gem5 ports. If a MessageBuffer
        # is a "to" buffer (i.e., out), then you use the "out_port";
        # otherwise, you use the in_port.
        # Instantiate a message buffer for each buffer in the controller.
        # For all of the “to” buffers, the network is the master.
        self.requestToDir = MessageBuffer(ordered=True)
        self.requestToDir.out_port = ruby_system.network.in_port
        self.responseToDirOrSibling = MessageBuffer(ordered=True)
        self.responseToDirOrSibling.out_port = ruby_system.network.in_port
        
        # For all of the “from” buffers, the network is the slave.
        self.forwardFromDir = MessageBuffer(ordered=True)
        self.forwardFromDir.in_port = ruby_system.network.out_port
        self.responseFromDirOrSibling = MessageBuffer(ordered=True)
        self.responseFromDirOrSibling.in_port = ruby_system.network.out_port


class DirController(Directory_Controller):
    _version = 0

    @classmethod
    def versionCount(cls):
        cls._version += 1  # Use count for this particular type
        return cls._version - 1

    def __init__(self, ruby_system, ranges, mem_ctrls):
        """Ranges are the memory ranges assigned to this controller."""
        if len(mem_ctrls) > 1:
            panic("This cache system can only be connected to one mem ctrl")
        super().__init__()
        self.version = self.versionCount()
        self.addr_ranges = ranges
        self.ruby_system = ruby_system
        self.directory = RubyDirectoryMemory()
        # Connect this directory to the memory side.
        self.memory = mem_ctrls[0].port
        self.connectQueues(ruby_system)

    def connectQueues(self, ruby_system):
        self.requestFromCache = MessageBuffer(ordered=True)
        self.requestFromCache.in_port = ruby_system.network.out_port
        self.responseFromCache = MessageBuffer(ordered=True)
        self.responseFromCache.in_port = ruby_system.network.out_port

        self.responseToCache = MessageBuffer(ordered=True)
        self.responseToCache.out_port = ruby_system.network.in_port
        self.forwardToCache = MessageBuffer(ordered=True)
        self.forwardToCache.out_port = ruby_system.network.in_port

        # These are other special message buffers. They are used to send
        # requests to memory and responses from memory back to the controller.
        # Any messages sent or received on the memory port (see self.memory
        # above) will be directed through these message buffers.
        self.requestToMemory = MessageBuffer()
        self.responseFromMemory = MessageBuffer()


class MyNetwork(SimpleNetwork):
    """A simple point-to-point network. This doesn't not use garnet."""

    def __init__(self, ruby_system):
        super().__init__()
        self.netifs = []
        self.ruby_system = ruby_system

    def connectControllers(self, controllers):
        """Connect all of the controllers to routers and connect the routers
        together in a point-to-point network.
        This implements a very simple, *unrealistic* point-to-point network.
        In other words, every controller has a direct link to every other controller.
        
        This function
            1. creates a router for each controller.
            2. creates an external link from that router to the controller.
            3. Adds all of the “internal” links. Each router is connected to all
               other routers to make the point-to-point network.
        """
        # Create one router/switch per controller in the system
        self.routers = [Switch(router_id=i) for i in range(len(controllers))]

        # Make an external link from each controller to the router.
        self.ext_links = [
            SimpleExtLink(link_id=i, ext_node=c, int_node=self.routers[i])
            for i, c in enumerate(controllers)
        ]

        # Make an "internal" link (internal to the network) between every pair
        # of routers.
        link_count = 0
        int_links = []
        for ri in self.routers:
            for rj in self.routers:
                if ri == rj:
                    continue  # Don't connect a router to itself!
                link_count += 1
                int_links.append(
                    SimpleIntLink(link_id=link_count, src_node=ri, dst_node=rj)
                )
        self.int_links = int_links
