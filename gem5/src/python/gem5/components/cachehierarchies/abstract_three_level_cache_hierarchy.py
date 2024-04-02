# Copyright (c) 2022 The Regents of the University of California
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

from ...utils.override import *

class AbstractThreeLevelCacheHierarchy:
    """
    An abstract three-level hierarchy with configurable size and associativity
    for each of L1, L2, and L3 caches.
    """

    def __init__(
        self,
        l1i_size: str,
        l1i_assoc: int,
        l1i_mshrs: int,
        l1i_tgts_per_mshr: int,

        l1d_size: str,
        l1d_assoc: int,
        l1d_mshrs: int,
        l1d_tgts_per_mshr: int,

        l2_size: str,
        l2_assoc: int,
        l2_mshrs: int,
        l2_tgts_per_mshr: int,

        l3_size: str,
        l3_assoc: int,
        l3_mshrs: int,
        l3_tgts_per_mshr: int,

        l1i_PrefetcherCls: Optional[Type[BasePrefetcher]] = None,
        l1d_PrefetcherCls: Optional[Type[BasePrefetcher]] = None,
        l2_PrefetcherCls: Optional[Type[BasePrefetcher]] = None,
        l3_PrefetcherCls: Optional[Type[BasePrefetcher]] = None,
    ):
        self._l1i_size = l1i_size
        self._l1i_assoc = l1i_assoc
        self._l1i_mshrs = l1i_mshrs
        self._l1i_tgts_per_mshr = l1i_tgts_per_mshr
        self._l1i_PrefetcherCls = l1i_PrefetcherCls

        self._l1d_size = l1d_size
        self._l1d_assoc = l1d_assoc
        self._l1d_mshrs = l1d_mshrs
        self._l1d_tgts_per_mshr = l1d_tgts_per_mshr
        self._l1d_PrefetcherCls = l1d_PrefetcherCls

        self._l2_size = l2_size
        self._l2_assoc = l2_assoc
        self._l2_mshrs = l2_mshrs
        self._l2_tgts_per_mshr = l2_tgts_per_mshr
        self._l2_PrefetcherCls = l2_PrefetcherCls

        self._l3_size = l3_size
        self._l3_assoc = l3_assoc
        self._l3_mshrs = l3_mshrs
        self._l3_tgts_per_mshr = l3_tgts_per_mshr
        self._l3_PrefetcherCls = l3_PrefetcherCls
