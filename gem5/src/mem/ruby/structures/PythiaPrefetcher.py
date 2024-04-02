# Copyright (c) 2023 Songzhu Zhang
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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

from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *

from m5.objects.System import System

class PythiaPrefetcher(SimObject) :
    type = "PythiaPrefetcher"
    cxx_class = "gem5::ruby::PythiaPrefetcher"
    cxx_header = "mem/ruby/structures/PythiaPrefetcher.hh"

    """
    The standard value of EQ is 256. This value cannot be too large. The larger
    it is, the longer it takes to start training the QVStore.
    """
    eval_que_capacity = Param.UInt32(256, "Capacity of the Evaluation Queue (EQ)")

    delta_que_capacity = Param.UInt32(4, "Capacity of the Delta Queue")

    # The learning rate parameter that controls the convergence rate of Q-values.
    alpha = Param.Float(0.0065, "Learning rate")

    """
    The discount factor, which is used to assign more weight to the immediate
    reward received by the agent at any given timestep than to the delayed
    future rewards.
    A gamma value closer to 1 gives a “far-sighted" planning capability to the
    agent, i.e., the agent can trade off a low immediate reward to gain higher
    rewards in the future. This is particularly useful in creating an
    autonomous agent that can anticipate the long-term effects of taking an
    action to optimize its policy that gets closer to optimal over time.
    """
    gamma = Param.Float(0.556, "Discount factor")

    """
    Epsilon is the exploration rate used to strike a balance between
    exploration and exploitation. It enables the epsilon-greedy agent
    stochastically takes a random action with a low probability of epsilon as
    opposed to the action that provides the highest Q-value.
    """
    epsilon = Param.Float(0.002, "Exploration rate")

    """
    For a 64-bit virtual address space with an 4 KB page size,
    #bits needed for page offset = log2(4 KB) = 12 bits
    #bits needed for page number = 64 - 12 = 52 bits
    If the page size is 4 KB, then 12 bits are required to represent an offset
    within a 4 KB page.
    """
    page_offset_bits = Param.UInt32(12, "Number of bits needed for the page offset")