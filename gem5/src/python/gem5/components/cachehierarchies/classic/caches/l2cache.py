# Copyright (c) 2021 The Regents of the University of California
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

from typing import Type, Optional

from m5.objects import (
    BasePrefetcher,
    Cache,
    Clusivity,
    StridePrefetcher,
    ScoobyPrefetcher,
    TaggedPrefetcher,
    DCPTPrefetcher,
    IndirectMemoryPrefetcher,
    SignaturePathPrefetcher,
    AMPMPrefetcher,
    BOPPrefetcher
)

from .....utils.override import *

"""
A simple L2 Cache with default values.

Assume the best case round-trip latency in L2$ is 14 cycles.

Tag Latency:
The time taken to access the tag array of a cache. The tag array stores
information about the data blocks held in the cache, such as their addresses.
Accessing the tag array is necessary to determine whether the requested data is
in the cache.

Data Latency:
If the data is found in the cache (a cache hit), this is the time taken to
access the actual data block within the cache memory.

Response Latency:
The time taken to send the data found in the cache back to the processor or to
signal a cache miss if the data is not found.

Best-case Round-Trip Latency = Tag Latency + Data Latency + Response Latency
In best-case, the hit rate of a cache is 100%.

Note that the following prefetchers currently do not work well in L2$ level:
    IrregularStreamBufferPrefetcher,
    SignaturePathPrefetcherV2,
"""

class L2Cache(Cache):

    def __init__(
        self,
        size: str,
        assoc: int = 16,
        tag_latency: int = 3,
        data_latency: int = 3,
        response_latency: int = 8,
        mshrs: int = 32,
        tgts_per_mshr: int = 12,
        writeback_clean: bool = False,
        clusivity: Clusivity = "mostly_incl",
        #clusivity: Clusivity = "mostly_excl",
        #PrefetcherCls: Type[BasePrefetcher] = SignaturePathPrefetcher,
        PrefetcherCls: Optional[Type[BasePrefetcher]] = None,
    ):
        super().__init__()
        self.size = size
        self.assoc = assoc
        self.tag_latency = tag_latency
        self.data_latency = data_latency
        self.response_latency = response_latency
        self.mshrs = mshrs
        self.tgts_per_mshr = tgts_per_mshr
        self.writeback_clean = writeback_clean
        self.clusivity = clusivity
        self.is_read_only = False
        if PrefetcherCls is not None:
            self.prefetcher = PrefetcherCls()
