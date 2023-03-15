# -*- coding: utf-8 -*-
# Copyright (c) 2017 Jason Lowe-Power
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

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject


class SimpleCache(ClockedObject):
    type = "SimpleCache"
    cxx_header = "learning_gem5/part2/simple_cache.hh"
    cxx_class = "gem5::SimpleCache"

    # Both the instruction and data ports connect to this port which is
    # automatically split out into two ports.
    # Vector ports allow this object to connect to multiple peers.
    # |PortID idx| is used to differentiate between the different ports.
    # By using a vector port, this cache can be connected into the system flexibly.
    cpu_side = VectorResponsePort("CPU side port")

    mem_side = RequestPort("Memory side port")

    latency = Param.Cycles(1, "Cycles taken on a hit or to resolve a miss")

    size = Param.MemorySize("16kB", "The size of the cache")

    # This is needed so that we can get the cache block size from the system object
    # when we're initializing the cache.
    # To reference the system object this cache is connected to, a special |proxy|
    # parameter -- |Parent.any| was used.
    # When a |SimpleCache| is instantiated, this proxy parameter searches through
    # all of the parents of the |SimpleCache| instance to find a SimObject that
    # matches the |System| type.
    # Since we often use a System as the root SimObject, you will often see a system
    # parameter resolved with this proxy parameter.
    system = Param.System(Parent.any, "The main system this cache is connected to")
