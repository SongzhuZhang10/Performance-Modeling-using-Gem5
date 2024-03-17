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
*          Config file for L1 Cache           *
*                                             *
***********************************************

Author: Songzhu Zhang
Date: 2023/04/03

Description:
This config file configures the L1Cache_Controller class defined in
`build/X86_debug/mem/ruby/protocol/L1Cache_Controller.py`.

It is customized to support Ruby ramdon test.

To use a prefetcher SimObject in the L1 cache, one must properly declare the
prefetcher pointer and use it in the cache coherence protocol in the SLICC file
located in `src/mem/ruby/protocol/MESI_Two_Level-L1cache.sm`.
"""

import math

from m5.util import fatal, panic

from m5.objects import *

class L1Cache(L1Cache_Controller):

    # variable to track the version number
    _version = 0

    # Decorator to create a class method
    @classmethod
    def versionCount(cls):
        # Must number each SLICC state machine in ascending order from 0.
        # Each machine of the same type should have a unique version number.
        cls._version += 1
        return cls._version - 1

    def __init__(
        self,
        l1i_size,
        l1i_assoc,
        l1d_size,
        l1d_assoc,
        num_l2Caches,
        cache_line_size,
        ruby_system,
        clk_domain: ClockDomain,
        enable_l1_prefetch=False
    ):
        """
        Create an L1 cache controller that consists of both instruction
        and data cache.
        """
        super().__init__()

        self.version = self.versionCount()

        self._cache_line_size = cache_line_size

        """
        We set all of the parameters that we named in the *.sm files.
        To know which paramters that need to be set in the config file,
        use `build/{ISA}/mem/ruby/protocol/L1Cache_Controller.py` as reference.
        """
        # This is the cache memory object that stores the cache data and tags
        self.L1Icache = RubyCache(
            size=l1i_size,
            assoc=l1i_assoc,
            start_index_bit=self.getBlockSizeBits(),
            is_icache=True,
            # default value for replacement_policy
            replacement_policy = TreePLRURP()
        )

        self.L1Dcache = RubyCache(
            size=l1d_size,
            assoc=l1d_assoc,
            start_index_bit=self.getBlockSizeBits(),
            is_icache=False,
            replacement_policy = TreePLRURP()
        )

        self.l2_select_num_bits = int(math.log(num_l2Caches, 2))
        self.clk_domain = clk_domain

        self.enable_l1_prefetch = enable_l1_prefetch

        # prefetcher is defined in the SLICC file and must be initialized in the config file
        self.prefetcher = PythiaPrefetcher()

        self.send_evictions = True
        self.transitions_per_cycle = 4

        self.ruby_system = ruby_system # Must do this!
        self.connectQueues(ruby_system.network)

    # This function returns the number of bits in byte offset field.
    def getBlockSizeBits(self):
        bits = int(math.log(self._cache_line_size, 2))
        if 2**bits != self._cache_line_size.value:
            raise Exception("Cache line size not a power of 2!")
        return bits


    # All the message buffers involved here will be used in the L1$'s SLICC file.
    def connectQueues(self, network):
        self.mandatoryQueue = MessageBuffer()
        
        self.requestFromL1Cache = MessageBuffer()
        self.requestFromL1Cache.out_port = network.in_port
        self.responseFromL1Cache = MessageBuffer()
        self.responseFromL1Cache.out_port = network.in_port
        self.unblockFromL1Cache = MessageBuffer()
        self.unblockFromL1Cache.out_port = network.in_port

        self.optionalQueue = MessageBuffer(ordered=True)

        self.requestToL1Cache = MessageBuffer()
        self.requestToL1Cache.in_port = network.out_port
        self.responseToL1Cache = MessageBuffer()
        self.responseToL1Cache.in_port = network.out_port
