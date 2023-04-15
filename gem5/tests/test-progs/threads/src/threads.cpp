/*
* Copyright (c) 2023 Songzhu Zhang
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

#include <iostream>

// Use the standard C++ libraries for multi-threading
#include <thread>

#include <mutex>
#include <vector>


using namespace std;

mutex shared_var_mutex;
mutex shared_print_mutex;
unsigned int shared_var = 0;

const unsigned num_iterations = 10;

void busy_wait(int count, int thread_id) {
    // NOTE: When a thread prints something out, the print statement must be
    // guarded by mutexes.
    shared_print_mutex.lock();
    printf("Thread %d is busy for %d ticks.\n", thread_id, count);
    shared_print_mutex.unlock();

    // Perform a large number of iterations to keep the program busy
    for (int i = 0; i < count; ++i) {
        // Do some simple computation
        int j = i % 10;
        j *= 2;
    }

    shared_print_mutex.lock();
    printf("Thread %d is free.\n", thread_id);
    shared_print_mutex.unlock();
}

// Thread function
void thread_func(int thread_id, int num_iterations)
{
    for (int i = 0; i < num_iterations; ++i)
    {
        // Lock the mutex to access the shared variable
        shared_var_mutex.lock();
        shared_var++;
        // Unlock the mutex before sleeping to allow other threads to access it
        shared_var_mutex.unlock();

        shared_print_mutex.lock();
        printf("Thread %d accessed the common resource!\n", thread_id);
        shared_print_mutex.unlock();
        
        int sleep_duration = rand() % (10 * (thread_id + 1) + num_iterations) + num_iterations;
        busy_wait(sleep_duration, thread_id);
    }
}

// This is a simple multi-threaded application with
// false sharing to stress the Ruby protocol.
int main()
{

    unsigned num_cores = thread::hardware_concurrency();
    //unsigned num_cores = 2;

    cout << "This test program is running on " << num_cores << " physical cores ";
    cout << "with " << num_iterations << " iterations." << endl;
    cout << "Note that only SimplePt2Pt network class is supported by gem5 SE mode." << endl;

    vector<thread> threads;

    // NOTE: -1 is required for this test program to work in SE mode.
    for (int i = 0; i < num_cores - 1; i++) {
        threads.emplace_back(thread_func, i, num_iterations);
    }

    // Execute the last thread with this thread context to avoid runtime errors
    // that exist only in SE mode.
    thread_func(num_cores - 1, num_iterations);

    // Wait for all threads to finish.
    for (auto& thread : threads) {
        thread.join();
    }

    // Check if the result is correct
    int expected_result = num_cores * num_iterations;
    if (shared_var == expected_result)
    {
        cout << "Result is correct: " << shared_var << endl;
    }
    else
    {
        cout << "Result is incorrect: expected " << expected_result << ", actual " << shared_var << endl;
    }
    return 0;
}
