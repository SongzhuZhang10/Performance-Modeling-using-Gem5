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

#include "learning_gem5/part2/goodbye_object.hh"

#include "base/trace.hh"
#include "debug/HelloExample.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

GoodbyeObject::GoodbyeObject(const GoodbyeObjectParams &params) :
    SimObject(params),
    event([this]{ processEvent(); }, name() + ".event"),
    bandwidth(params.write_bandwidth),
    bufferSize(params.buffer_size),
    buffer(nullptr),
    bufferUsed(0)
{
    buffer = new char[bufferSize]();
    DPRINTF(HelloExample, "Created the goodbye object\n");
}

GoodbyeObject::~GoodbyeObject()
{
    // destructor frees the resource
    delete[] buffer;
}

void
GoodbyeObject::processEvent()
{
    DPRINTF(HelloExample, "Processing the event!\n");

    // Actually do the "work" of the event
    fillBuffer();
}

// This is the interface to the GoodbyeObject. It takes a string as a parameter.
void
GoodbyeObject::sayGoodbye(std::string other_name)
{
    DPRINTF(HelloExample, "Saying goodbye to %s\n", other_name);

    // The simulator builds the message and saves it in a member variable.
    message = "Goodbye " + other_name + "!! ";

    // Then, we begin filling the buffer.
    // Kick off the the first buffer fill. If it can't fill the whole buffer
    // because of a limited bandwidth, then this function will schedule another
    // event to finish the fill
    fillBuffer();
}

void
GoodbyeObject::fillBuffer()
{
    // There better be a message
    assert(message.length() > 0);

    // Copy from the message to the buffer per byte.
    int bytes_copied = 0;
    for (auto it = message.begin();
         it < message.end() && bufferUsed < bufferSize - 1;
         it++, bufferUsed++, bytes_copied++) {
        // Copy the character into the buffer
        buffer[bufferUsed] = *it;
    }

    if (bufferUsed < bufferSize - 1) {
        /* 
         * To model the limited bandwidth, each time we write the message to the buffer,
         * we pause for the latency it takes to write the message.
         * We use a simple event to model this pause.
         */
        // Wait for the next copy for as long as it would have taken
        DPRINTF(HelloExample, "Scheduling another fillBuffer in %d ticks\n",
                bandwidth * bytes_copied);
        
        // The |bandwidth| variable is automatically converted into ticks per byte.
        // So, latency =  bandwidth X bytes to be written into the buffer
        // The second parameter of |schedule| function is time point at which the next 
        // event is scheduled to happen.
        schedule(event, curTick() + bandwidth * bytes_copied);
    } else {
        DPRINTF(HelloExample, "Goodbye done copying!\n");
        // When the buffer is full, we call the function exitSimLoop,
        // which will exit the simulation.
        // The first parameter is the message to return to the Python config script (exit_event.getCause()).
        // The second parameter is the exit code.
        // The third parameter specifies when to exit.
        // Be sure to take into account the time for the last bytes
        exitSimLoop(buffer, 0, curTick() + bandwidth * bytes_copied);
    }
}

} // namespace gem5
