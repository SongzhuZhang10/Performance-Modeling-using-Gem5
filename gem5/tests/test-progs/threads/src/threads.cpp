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
#include <unordered_map>
#include <functional>

using namespace std;

mutex shared_var_mutex;
mutex shared_print_mutex;
/**
 * Aligning shared_var to a 64-byte boundary is an attempt to prevent false
 * sharing. False sharing occurs when multiple variables that are not
 * logically related share the same cache line, leading to unnecessary cache
 * invalidations and performance degradation in a multi-threaded environment.
 * The alignas(64) directive is telling the compiler to ensure that the memory
 * for shared_var starts at an address that is a multiple of 64 bytes. This
 * aligns the variable to a cache line boundary, which can be beneficial for
 * performance in some situations.
 * However, the effectiveness of alignment optimizations is not guaranteed.
 */
alignas(64) int shared_var = 0; // Align to a 64-byte boundary (common cache line size)

const unsigned num_iterations = 9999;

vector<function<void(int&)>> operations;

void iCacheIntensiveTask(int n, int thread_id);
void initializeOperations();
void unpredictablyModifyVector(vector<int>& vec, int modifier);
void dCacheIntensiveTask(int n, int thread_id);

void initializeOperations()
{
    operations.push_back([](int& x){ x += rand() % 100; });
    operations.push_back([](int& x){ x -= rand() % 50; });
    operations.push_back([](int& x){ x ^= rand() % 25; });
}

void unpredictablyModifyVector(vector<int>& vec, int modifier)
{
    for (size_t i = 0; i < vec.size(); ++i) {
        int index = rand() % vec.size();
        operations[rand() % operations.size()](vec[index]);
        vec[index] = (vec[index] + modifier) % 10000;
    }
}

void iCacheIntensiveTask(int n, int thread_id)
{
    printf("I-Cache Intensive Task with ID %d starts running.\n", thread_id);
    vector<int> dynamicVector(rand() % 20000 + 10000);
    for (auto& elem : dynamicVector) {
        elem = rand();
    }
    unpredictablyModifyVector(dynamicVector, n);
    int result = 0;
    for (size_t i = 0; i < dynamicVector.size(); i += rand() % 100 + 1) {
        shared_var_mutex.lock();
        result ^= dynamicVector[i] + shared_var;
        shared_var_mutex.unlock();
    }

    // NOTE: When a thread prints something out, the print statement must
    // be guarded by mutexes.
    shared_print_mutex.lock();
    printf("I-Cache Intensive Task Thread %d: result = %d\n", thread_id, result);
    shared_print_mutex.unlock();
}


void dCacheIntensiveTask(int n, int thread_id)
{
    printf("D-Cache Intensive Task with ID %d starts running.\n", thread_id);
    unordered_map<int, int> dataMap;
    for (int i = 0; i < n; ++i) {
        dataMap[rand() % n] = rand();
    }
    for (auto& pair : dataMap) {
        // Lock the mutex to access the shared variable
        shared_var_mutex.lock();
        pair.second += shared_var;
        // Unlock the mutex before sleeping to allow other threads to access it
        shared_var_mutex.unlock();
    }
    int sum = 0;
    for (auto& pair : dataMap) {
        sum += pair.second;
    }

    shared_print_mutex.lock();
    printf("D-Cache Intensive Task Thread %d: sum = %d\n", thread_id, sum);
    shared_print_mutex.unlock();
}

// This is a simple multi-threaded application with
// extensive random memory access pattern to stress the L2 cache prefetcher.
int main()
{
    srand(7); // For debugging, use a fixed seed:
    unsigned num_cores = thread::hardware_concurrency();

    cout << "This test program is running on " << num_cores << " physical cores ";
    cout << "with " << num_iterations << " iterations." << endl;
    
    initializeOperations();
    
    if (num_cores > 1) {
        vector<thread> threads;

        /**
         * Attention: The total number of threads (including the thread running
         * the main() funciton) cannot exceed the number of physical cores in SE
         * mode. Thus,-1 is required for this test program to work.
         */
        for (unsigned i = 0; i < num_cores - 1; i++) {
            if (i % 2 == 0) {
                threads.emplace_back(iCacheIntensiveTask, num_iterations, i);
            } else {
                threads.emplace_back(dCacheIntensiveTask, num_iterations, i);
            }
        }
        dCacheIntensiveTask(num_iterations, 0);
        // Execute the last thread with this thread context to avoid runtime errors
        // that exist only in SE mode.
        // Wait for all threads to complete their execution
        for (auto& thread : threads) {
            thread.join();
        }
    } else {
        // If there's only one core, just run one instance of each task sequentially
        iCacheIntensiveTask(num_iterations, 0);
        dCacheIntensiveTask(num_iterations, 0);
    }

    cout << "---------> End <---------" << endl;
    return 0;
}