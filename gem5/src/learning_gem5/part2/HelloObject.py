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
from m5.SimObject import SimObject


class HelloObject(SimObject):
    # |type| is the C++ class you're wrapping with this Python SimObject.
    type = "HelloObject"
    
    # |cxx_header| is the file that contains the declaration of the |type| class.
    cxx_header = "learning_gem5/part2/hello_object.hh"
    
    # |cxx_class| is an attribute specifying the newly created SimObject is declared within the gem5 namespace.
    cxx_class = "gem5::HelloObject"

    
    # Param.<TypeName> declares a parameter of type TypeName. Common types are Int for integers, Float for floats, etc.
    # These types act like regular Python classes.
    # Each parameter declaration takes one or two parameters.
    # If you only specify a single parameter to the parameter declaration, it is the description
    time_to_wait = Param.Latency("Time before firing the event") # parameter declaration
    
    # parameter declaration
    # First parameter: default value of |number_of_fires|; it's used when your Python config file
    # does not specify any value for |number_of_fires|.
    # Second parameter: a short description of the parameter; it must be a Python string.
    number_of_fires = Param.Int(
        1, "Number of times to fire the event before " "goodbye"
    )

    # Add a GoodbyeObject as a parameter to the HelloObject.
    # To do this, you simply specify the SimObject class name (i.e., |GoodbyeObject|) as the |TypeName| of the Param.
    # type_name = Param.TypeName
    goodbye_object = Param.GoodbyeObject("A goodbye object")


class GoodbyeObject(SimObject):
    type = "GoodbyeObject"
    cxx_header = "learning_gem5/part2/goodbye_object.hh"
    cxx_class = "gem5::GoodbyeObject"

    # buffer_size and write_bandwidth are parameters of the object.
    # They both have a default value.
    # |buffer_size| is a |MemorySize| parameter.
    buffer_size = Param.MemorySize(
        "1kB", "Size of buffer to fill with goodbye"
    )
    
    # |write_bandwidth| specifies the speed to fill the buffer.
    # Once the buffer is full, the simulation will exit.
    write_bandwidth = Param.MemoryBandwidth(
        "100MB/s", "Bandwidth to fill " "the buffer"
    )
