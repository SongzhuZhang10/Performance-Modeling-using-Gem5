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
#include <cstdlib>  // For rand() and srand()
#include <mutex>
#include <vector>
#include <cmath>
#include <stdio.h>

using namespace std;

mutex shared_var_mutex;
mutex shared_print_mutex;
int shared_var = 0;
const bool enable_print = true;
const unsigned num_iterations = 50;

unsigned factorial(int n) {
    if (n == 0) {
        return 1;
    }
    else {
        return n * factorial(n - 1);
    }
}

void iCacheIntensiveTask(int n, int thread_id) {
    for (int i = 0; i < n; ++i) {
        // Lock the mutex to access the shared variable
        shared_var_mutex.lock();
        shared_var++;
        // Unlock the mutex before sleeping to allow other threads to access it
        shared_var_mutex.unlock();
    }

    volatile double result = 0;
    // 1. Nested loops with complex calculations:
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < (n * 1.4); ++j) {
            result = n * (i * j) + sqrt(i * j) + pow(i + j, 3.14);
            // Perform some operations with result to prevent compiler optimizations
            if ((rand() % n) > (n / 2)) {
                result *= 2 * (rand() % n + 2);
            }
            else {
                result /= 4;
            }
        }
    }

    // Memory-bound operations with unpredictable access patterns:
    vector<unsigned> largeVector(10000, 0);
    for (unsigned i = 0; i < largeVector.size(); ++i) {
        largeVector[i] = i * n + (n % 10);
    }
    for (unsigned i = 0; i < largeVector.size(); ++i) {
        int index = n + (largeVector[i] % 100);
        largeVector[index] = largeVector[i] * largeVector[index];
    }
    // Combine elements using XOR
    volatile unsigned combinedValue = 0;
    for (unsigned i = 0; i < largeVector.size(); ++i) {
        combinedValue ^= largeVector[i];
    }

    // Function calls with varying n sizes:
    volatile unsigned factorial_result = 0;
    for (int i = 0; i < (rand() % 10); ++i) {
        shared_var_mutex.lock();
        factorial_result = factorial(rand() % (shared_var + i + 10));  // Assuming a factorial function is defined elsewhere
        shared_var_mutex.unlock();
    }

    if (enable_print) {
        // NOTE: When a thread prints something out, the print statement must be
        // guarded by mutexes.
        shared_print_mutex.lock();
        printf("Thread %d: reault        = %.3f\n", thread_id, result);
        printf("Thread %d: combinedValue = %u\n", thread_id, combinedValue);
        printf("Thread %d: combinedValue = %u\n", thread_id, factorial_result);
        shared_print_mutex.unlock();
    }
}

void busy_wait(int count, int thread_id) {
    if (enable_print) {
        // NOTE: When a thread prints something out, the print statement must be
        // guarded by mutexes.
        shared_print_mutex.lock();
        printf("Thread %d is busy for %d ticks.\n", thread_id, count);
        shared_print_mutex.unlock();
    }

    volatile int j = 0;
    // Perform a large number of iterations to keep the program busy
    for (int i = 0; i < count; ++i) {
        // Do some simple computation
        j = i % 10;
        j *= 2;
    }

    if (enable_print) {
        shared_print_mutex.lock();
        printf("Thread %d is free with j = %d.\n", thread_id, j);
        shared_print_mutex.unlock();
    }
}

// Thread function
void dCacheIntensiveTask(int n, int thread_id)
{
    for (int i = 0; i < n; ++i) {
        // Lock the mutex to access the shared variable
        shared_var_mutex.lock();
        shared_var++;
        // Unlock the mutex before sleeping to allow other threads to access it
        shared_var_mutex.unlock();
    }

    if (enable_print) {
        shared_print_mutex.lock();
        printf("Thread %d accessed the common resource!\n", thread_id);
        shared_print_mutex.unlock();
    }

    // Allocate a large vector to stress the cache
    std::vector<int> data(n);

    // Sequential access pattern (cache-friendly)
    for (int i = 0; i < n; i++) {
        data[i] = i * 2;
    }

    // Random access pattern (cache-unfriendly)
    for (int i = 0; i < n; i++) {
        int randomIndex = rand() % n;
        data[randomIndex] += i * (rand() % n);
    }

    // Computation on the data (to ensure usage)
    volatile int sum = 0;
    for (int i = 0; i < n; i++) {
        sum += data[i];
    }

    if (enable_print) {
        shared_print_mutex.lock();
        printf("Thread %d: result = %d\n", thread_id, sum);
        shared_print_mutex.unlock();
    }

    // Prevent compiler optimization from removing unused results
    volatile int result = sum % n;

    int sleep_duration = rand() % (10 * (thread_id + 1) + result);
    busy_wait(sleep_duration, thread_id);
}

// This is a simple multi-threaded application with
// false sharing to stress the Ruby protocol.
int main()
{
    // For debugging, use a fixed seed:
    srand(1);
    unsigned num_cores = thread::hardware_concurrency();

    cout << "This test program is running on " << num_cores << " physical cores ";
    cout << "with " << num_iterations << " iterations." << endl;
    //cout << "Note that only SimplePt2Pt network class is supported by gem5 SE mode." << endl;

    vector<thread> threads;

    // NOTE: -1 is required for this test program to work in SE mode.
    for (unsigned i = 0; i < num_cores - 1; i++) {
        threads.emplace_back(iCacheIntensiveTask, num_iterations, i);
    }

    // Execute the last thread with this thread context to avoid runtime errors
    // that exist only in SE mode.
    iCacheIntensiveTask(num_cores - 1, num_iterations);

    // Wait for all threads to finish.
    for (auto& thread : threads) {
        thread.join();
    }

    // Check if the result is correct
    int expected_result = (num_cores + 1) * num_iterations;
    if (shared_var == expected_result)
    {
        cout << "---------> The shared variable has the correct result! It is " << shared_var << endl;
    }
    else
    {
        cout << "Warning: The shared variable has incorrect result: expected " << expected_result << ", actual " << shared_var << endl;
    }
    return 0;
}
