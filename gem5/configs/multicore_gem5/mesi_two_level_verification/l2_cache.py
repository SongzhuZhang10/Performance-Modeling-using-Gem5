# Copyright (c) 2021 The Regents of the University of California
# All Rights Reserved.
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
*          Config file for L2 Cache           *
*                                             *
***********************************************

Author: Songzhu Zhang
Date: 2023/04/03

Description:
This config file configures the L2Cache_Controller class defined in
`build/X86_debug/mem/ruby/protocol/L2Cache_Controller.py`.

It is customized to support Ruby ramdon test.
"""

import math

from m5.objects import *

class L2Cache(L2Cache_Controller):

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
        l2_size,
        l2_assoc,
        num_l2Caches,
        ruby_system,
        clk_domain: ClockDomain,
        cache_line_size,
    ):
        super().__init__()

        self.version = self.versionCount()
        self._cache_line_size = cache_line_size

        """
        This is the cache memory object that stores the cache data and tags.
        NOTE: You cannot use things like `self.cacheMemory = RubyCache(...)`
        here because the name self.L2cache is reserved in
        `build/X86_debug/mem/ruby/protocol/L2Cache_Controller.py`.
        If you use any other name (e.g., cacheMemory), when calling 
        `m5.instantiate()` in the top config file, an error message like the
        following will show up:
        `fatal: system.caches.network.ext_links2.ext_node.L2cache without
        default or user set value`.
        This is because `L2Cache_Controller` does not have an attribute that
        is named as `cacheMemory`, leaving the attribute `L2cache` undefined.
        """
        self.L2cache = RubyCache(
            size=l2_size,
            assoc=l2_assoc,
            start_index_bit=self.getIndexBit(num_l2Caches),
        )

        self.ruby_system = ruby_system # Don't forget this!
        self.clk_domain = clk_domain
        self.transitions_per_cycle = 4
        self.connectQueues(ruby_system.network)

    def getIndexBit(self, num_l2caches):
        l2_bits = int(math.log(num_l2caches, 2))
        bits = int(math.log(self._cache_line_size, 2)) + l2_bits
        return bits

    def connectQueues(self, network):
        self.DirRequestFromL2Cache = MessageBuffer()
        self.DirRequestFromL2Cache.out_port = network.in_port
        self.L1RequestFromL2Cache = MessageBuffer()
        self.L1RequestFromL2Cache.out_port = network.in_port
        # When using DDR3_1600_8x8 as the main memory and all other buffers
        # in the system are set to `ordered`, this buffer must be set
        # to unordered. Or, a panic message that says "FIFO ordering violated"
        # will be printed.
        self.responseFromL2Cache = MessageBuffer()
        self.responseFromL2Cache.out_port = network.in_port
        self.unblockToL2Cache = MessageBuffer()
        self.unblockToL2Cache.in_port = network.out_port
        self.L1RequestToL2Cache = MessageBuffer()
        self.L1RequestToL2Cache.in_port = network.out_port
        self.responseToL2Cache = MessageBuffer()
        self.responseToL2Cache.in_port = network.out_port
