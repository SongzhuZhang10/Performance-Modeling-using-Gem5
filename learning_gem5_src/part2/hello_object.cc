/*
 * Copyright (c) 2017 Jason Lowe-Power
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "learning_gem5/part2/hello_object.hh"

#include "base/logging.hh"
#include "base/trace.hh"

// We need to include the automatically generated header file in any
// files where we plan to use the debug flag.
#include "debug/HelloExample.hh"

namespace gem5
{

// Constructor
HelloObject::HelloObject(const HelloObjectParams &params) :
    SimObject(params),
    // When the event is triggered, it will call the processEvent() function.
    // This is an instance of the event function wrapper.
    // Its first parameter is a lambda. We capture |this| in the lambda |[this]|
    // so we can call member functions of the instance of the class.
    event([this]{ processEvent(); }, name() + ".event"),

    goodbye(params.goodbye_object),
    
    // Each params instantiation has a name which comes from the Python config file
    // when it is instantiated.
    // Note: This is not needed as you can *always* reference this->name() because
    // for all SimObjects, there is a |name()| function that always returns the name.
    myName(params.name),

    // Add default values for the |latency|.
    // Copy the value of |time_to_wait| defined in the SimObject Python file
    // to the C++ class in its CONSTRUCTOR!
    // Note that |time_to_wait| value must be specified in the Python 
    // config file |run_hello.py| because no default value is specified in the |HelloObject.py| file.
    latency(params.time_to_wait),


    // Copy the value of |timesLeft| from the Python file to the C++ class in its constructor.
    // Add default values for the |timesLeft|.
    timesLeft(params.number_of_fires)
{
    // The first parameter is a debug flag that has been declared in a SConscript file.
    // We can use the flag |HelloExample| since we declared it in the 
    // |src/learning_gem5/SConscript| file.
    // Three things are printed to stdout.
    // 1. The current tick when the DPRINTF is executed.
    // 2. The name of the SimObject that called DPRINTF.
    //    This name is usually the Python variable name from the Python config file.
    // 3. You see whatever format string you passed to the DPRINTF function.
    DPRINTF(HelloExample, "Created the hello object\n");

    /*
     * Add a check in the constructor to make sure the goodbye pointer is valid.
     * It is possible to pass a null pointer as a SimObject via the parameters by
     * using the NULL special Python SimObject. We should panic when this happens
     * since it is not a case this object has been coded to accept.
     */
    panic_if(!goodbye, "HelloObject must have a non-null GoodbyeObject");
}

/* 
 * Initially schedule the event in the startup() function in the object we created.
 * It does not get executed until the simulation begins for the first time (i.e., 
 * the simulate() function is called from a Python config file).
 */
void
HelloObject::startup()
{
    // Schedule the event for the event to be processed.
    // Before simulation starts, we need to schedule the event
    // Schedule the event to execute at tick specified be |latency|.
    schedule(event, latency);
}

// Definition of the function event which must take no parameters and return nothing.
void
HelloObject::processEvent()
{
    timesLeft--;
    DPRINTF(HelloExample, "Hello world! Processing the event! %d left\n",
                          timesLeft);

    if (timesLeft <= 0) {
        DPRINTF(HelloExample, "Done firing!\n");
        
        // Once we have processed the number of events specified by the |timesLeft| parameter,
        // we should call the |sayGoodbye| function in the |GoodbyeObject|.
        goodbye->sayGoodbye(myName);
    } else {
        schedule(event, curTick() + latency);
    }
}

} // namespace gem5
