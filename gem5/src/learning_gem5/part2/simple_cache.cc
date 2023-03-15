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

/*
 * Cache Miss on Read Request
 * 1. The CPU checks if the data is in the cache. Then a cache miss occurs.
 * 2. The required data is brought into the cache from the DRAM.
 * 3. In the next single clock cycle, the CPU reads the cache again for
 *    the data that was not successfully fetched from the cache in step 1.
 *    (Though the CPU can read the requested data from the cache this time,
 *     the cache hit counter remains unchanged.)
 * 
 * Cache Miss on Write Request (no cache block eviction happens)
 * 1. The CPU checks if the data is in the cache. Then a cache miss occurs.
 * 2. The required data is brought into the cache from the DRAM.
 * 3. In the next single clock cycle, the CPU writes the cache again for
 *    the data that was not successfully written into the cache in step 1.
 *    (Though the CPU can write the requested data into the cache this time,
 *     the cache hit counter remains unchanged.)
 * 
 * TODO: Cache Miss on Write Request (cache block eviction happens)
 * 
 */
#include "learning_gem5/part2/simple_cache.hh"

#include "base/compiler.hh"
#include "base/random.hh"

// We need to include the automatically generated header file in any
// files where we plan to use the debug flag.
#include "debug/SimpleCache.hh"
#include "base/trace.hh"

#include "sim/system.hh"

namespace gem5
{

SimpleCache::SimpleCache(const SimpleCacheParams &params) :
    // -> initialization list start
    ClockedObject(params), // passed from the Python class
    latency(params.latency), // passed from the Python class

    // cache block size = cache line size = number of words fetched by one normal cache fetch operation
    // Use the cacheLineSize from the system parameters to set the blockSize for this cache
    blockSize(params.system->cacheLineSize()), // cache block size = 64 bytes

    // Initialize the capacity based on the block size and |params.size|
    // Capacity here is defined as the number of blocks in the cache.
    // size is the total storage size of the cache in the unit of kB.
    // 
    capacity(params.size / blockSize), // # blocks in the cache = 16
    
    // memPort is the instance name of the class |MemSidePort|.
    memPort(params.name + ".mem_side", this),

    blocked(false), originalPacket(nullptr), waitingPortId(-1), stats(this)
    // -> initialization list end
{
    DPRINTF(SimpleCache, "Call the constructor of SimpleCache.\n");

    // Cache block size is 64 bytes, and it is constant throughout the simulation.
    DPRINTF(SimpleCache, "Cache block size: %u bytes\n", params.system->cacheLineSize());
    DPRINTF(SimpleCache, "Number of blocks in the cache: %u\n", params.size / blockSize);

    // Since the CPU side ports are a vector of ports, we need to create 
    // an instance of the CPUSidePort for each connection. This member of 
    // params is automatically created depending on the name of the vector 
    // port and holds the number of connections to this port name.
    // Since the |cpu_side| port was declared as a |VectorSlavePort| in the 
    // SimObject Python file, the parameter automatically has a variable
    // |port_cpu_side_connection_count|. 
    for (int i = 0; i < params.port_cpu_side_connection_count; ++i) {
        DPRINTF(SimpleCache, "Make CPU side connection %d.\n", i);
        /**
         * Create a number of CPUSidePorts based on the number of connections to this object.
         * For each of these connections, we add a new |CPUSidePort| to a |cpuPorts| vector 
         * declared in the SimpleCache class.
         * emplace_back: Adds an element constructed in place to the end of the vector.
         */
        cpuPorts.emplace_back(name() + csprintf(".cpu_side[%d]", i), i, this);
    }
}

Port &
SimpleCache::getPort(const std::string &if_name, PortID idx)
{
    /* Return the port based on the id requested */
    DPRINTF(SimpleCache, "Function: getPort\n");
    DPRINTF(SimpleCache, "Name: %s\n", if_name);
    DPRINTF(SimpleCache, "Port ID: %d\n", idx);

    // This is the name from the Python SimObject declaration in SimpleCache.py
    if (if_name == "mem_side") {
        panic_if(idx != InvalidPortID,
                 "Mem side of simple cache is not a vector port!");
        // Memory side ports are request ports.
        return memPort;
    } else if (if_name == "cpu_side" && idx < cpuPorts.size()) {
        // Because |cpuPorts| is of vector type, the |size| function will return
        // the number of elements in the vector.
        // We should have already created all of the ports in the constructor
        // CPU side ports are response ports.
        return cpuPorts[idx];
    } else {
        // pass it along to our super class
        return ClockedObject::getPort(if_name, idx);
    }
}

void
SimpleCache::CPUSidePort::sendPacket(PacketPtr pkt)
{
    DPRINTF(SimpleCache, "Function: sendPacket\n");

    // Note: This flow control is very simple since the cache is blocking.

    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    // If we can't send the packet across the port, store it for later.
    DPRINTF(SimpleCache, "Sending packet (%s) to CPU\n", pkt->print());

    DPRINTF(SimpleCache, "Function: sendTimingResp\n");
    if (!sendTimingResp(pkt)) {
        DPRINTF(SimpleCache, "Failed to send timing response to CPU!\n");
        DPRINTF(SimpleCache, "The packet is blocked!\n");
        blockedPacket = pkt;
    }
}

AddrRangeList
SimpleCache::CPUSidePort::getAddrRanges() const
{
    DPRINTF(SimpleCache, "Function: getAddrRanges\n");
    return owner->getAddrRanges();
}

void
SimpleCache::CPUSidePort::trySendRetry()
{
    DPRINTF(SimpleCache, "Function: trySendRetry\n");
    if (needRetry && blockedPacket == nullptr) {
        // blockedPacket == nullptr means that no outstanding packet blocking the CPU side port.
        DPRINTF(SimpleCache, "No outstanding packet blocking the CPU side port.\n");
        // Only send a retry if the port is now completely free
        needRetry = false;
        DPRINTF(SimpleCache, "Deassert needRetry. Sending retry request.\n");
        sendRetryReq();
    }
}

void
SimpleCache::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    DPRINTF(SimpleCache, "Function: recvFunctional\n");
    // Just forward to the cache.
    return owner->handleFunctional(pkt);
}

bool
SimpleCache::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(SimpleCache, "Function: recvTimingReq\n");

    DPRINTF(SimpleCache, "Got request (%s)\n", pkt->print());

    if (blockedPacket || needRetry) {
        // The cache may not be able to send a reply if this is blocked
        DPRINTF(SimpleCache, "Request blocked\n");
        needRetry = true;
        return false;
    }
    // Just forward to the cache.
    if (!owner->handleRequest(pkt, id)) {
        DPRINTF(SimpleCache, "Request failed\n");
        // stalling
        needRetry = true;
        DPRINTF(SimpleCache, "Assert needRetry flag\n");
        return false;
    } else {
        DPRINTF(SimpleCache, "Request succeeded\n");
        return true;
    }
}

void
SimpleCache::CPUSidePort::recvRespRetry()
{
    DPRINTF(SimpleCache, "Function: recvRespRetry\n");

    // We should have a blocked packet if this function is called.
    assert(blockedPacket != nullptr);

    // Grab the blocked packet.
    PacketPtr pkt = blockedPacket;
    // Reset the blocked packet.
    blockedPacket = nullptr;

    DPRINTF(SimpleCache, "Retrying the response packet (%s)\n", pkt->print());

    // Try to resend it. It's possible that it fails again.
    sendPacket(pkt);

    // We may now be able to accept new packets
    trySendRetry();
}

void
SimpleCache::MemSidePort::sendPacket(PacketPtr pkt)
{
    DPRINTF(SimpleCache, "Function: sendPacket\n");
    
    // Note: This flow control is very simple since the cache is blocking.

    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    // If we can't send the packet across the port, store it for later.
    DPRINTF(SimpleCache, "Function: sendTimingReq\n");
    if (!sendTimingReq(pkt)) {
        DPRINTF(SimpleCache, "Failed to send the packet. Store it for later.");
        blockedPacket = pkt;
    }
}

bool
SimpleCache::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(SimpleCache, "Function: recvTimingResp\n");
    // Just forward to the cache.
    return owner->handleResponse(pkt);
}

void
SimpleCache::MemSidePort::recvReqRetry()
{
    DPRINTF(SimpleCache, "Function: recvReqRetry\n");

    // We should have a blocked packet if this function is called.
    assert(blockedPacket != nullptr);

    // Grab the blocked packet.
    DPRINTF(SimpleCache, "Grab the blocked packet and then reset blockedPacket.\n");
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    // Try to resend it. It's possible that it fails again.
    sendPacket(pkt);
}

void
SimpleCache::MemSidePort::recvRangeChange()
{
    DPRINTF(SimpleCache, "Function: recvRangeChange\n");
    owner->sendRangeChange();
}

bool
SimpleCache::handleRequest(PacketPtr pkt, int port_id)
{
    DPRINTF(SimpleCache, "Function: handleRequest\n");
    DPRINTF(SimpleCache, "Packet Info: %s\n", pkt->print());
    DPRINTF(SimpleCache, "Port ID: %d\n", port_id);

    if (blocked) {
        // There is currently an outstanding request so we can't respond. Stall
        DPRINTF(SimpleCache, "There is currently an outstanding request. Need to retry.\n");
        return false;
    }

    DPRINTF(SimpleCache, "Got request for addr %#x\n", pkt->getAddr());

    // This cache is now blocked waiting for the response to this packet.
    DPRINTF(SimpleCache, "The cache is now blocked, waiting for the response to this packet.\n");
    blocked = true;

    assert(waitingPortId == -1);
    
    // port_id: the id of the port which the request originated
    // Without port_id, we would not be able to forward the response to the correct port.
    // Store the port id of the request
    // Since the SimpleCache is blocking (only allows a single request outstanding at a time),
    // we only need to save a single port id.
    DPRINTF(SimpleCache, "Store the port id of the request.\n");
    waitingPortId = port_id;

    // Take into account the latency to access the cache tags and the cache data for a request
    // by using an event to stall the request for the needed amount of time. We schedule a new
    // event for |latency| cycles in the future.
    DPRINTF(SimpleCache, "Schedule the event of accessing the cache for a request.\n");
    schedule(new EventFunctionWrapper([this, pkt]{ accessTiming(pkt); },
                                      name() + ".accessEvent", true),
             // clockEdge function returns the tick that the nth cycle in the future occurs on.
             clockEdge(latency));

    return true;
}

bool
SimpleCache::handleResponse(PacketPtr pkt)
{
    DPRINTF(SimpleCache, "Function: handleResponse\n");

    // On a response from memory, we know that this was caused by a cache miss.
    assert(blocked);
    DPRINTF(SimpleCache, "Got response for addr %#x\n", pkt->getAddr());

    // For now, assume that inserts are off of the critical path, and don't count
    // for any added latency.
    // Insert the responding packet into the cache.
    // Using %#x will prefix the value with |0x|.
    // Using %x will just show the value.
    DPRINTF(SimpleCache, "Insert the responding packet from DRAM address %#x into the cache\n", pkt->getAddr());
    insert(pkt);

    stats.missLatency.sample(curTick() - missTime);

    // If we had to upgrade the request packet to a full cache line, now we
    // can use that packet to construct the response.
    if (originalPacket != nullptr) {
        /**
         * Entering this branch means that the packet we are receiving as a response
         * was an upgrade packet which was produced when the original request was
         * smaller than a cache line.
         */
        DPRINTF(SimpleCache, "The packet sent from the CPU needed to be upgraded.\n");

        DPRINTF(SimpleCache, "Functionally access the cache and then update the origional packet.\n");
        // Write data from the packet into the cache block for a write request.
        // Read the data out of the cache block into the packet for a read request.
        // The data in |originalPacket| is modified here!!!
        [[maybe_unused]] bool hit = accessFunctional(originalPacket);

        panic_if(!hit, "Should always hit after inserting");

        originalPacket->makeResponse();

        // Delete the new packet that we made in the miss handling logic
        delete pkt;

        pkt = originalPacket;
        originalPacket = nullptr;
    } // else, pkt contains the data it needs

    sendResponse(pkt);

    return true;
}

void SimpleCache::sendResponse(PacketPtr pkt)
{
    DPRINTF(SimpleCache, "Function: sendResponse\n");

    assert(blocked);
    DPRINTF(SimpleCache, "Sending response for addr %#x\n", pkt->getAddr());

    // Use the |waitingPortId| to send the packet to the right port
    int port = waitingPortId;

    /* 
     * The packet is now done. We're about to put it in the port, no need for
     * this object to continue to stall.
     * We need to free the resource before sending the packet in case the CPU
     * tries to send another request immediately (e.g., in the same callchain).
     * Mark the SimpleCache unblocked before calling |sendPacket| in case
     * the peer on the CPU side immediately calls |sendTimingReq|.
     */
    DPRINTF(SimpleCache, "Unblock the cache.\n");
    blocked = false;
    

    // Reset waitingPortId since it has been used already.
    waitingPortId = -1;

    // Simply forward to the memory port
    cpuPorts[port].sendPacket(pkt);

    // For each of the cpu ports, if it needs to send a retry to the CPU side ports, 
    // it should do it now since this memory object may be unblocked now.
    DPRINTF(SimpleCache,"Check each of the cpu ports to see if there is an outstanding packet waiting to be sent to the CPU side ports.\n");
    for (auto& port : cpuPorts) {
        port.trySendRetry();
    }
}

void
SimpleCache::handleFunctional(PacketPtr pkt)
{
    DPRINTF(SimpleCache, "Function: handleFunctional\n");

    if (accessFunctional(pkt)) {
        DPRINTF(SimpleCache, "Make a response to the request.\n");
        pkt->makeResponse();
    } else {
        DPRINTF(SimpleCache, "Send a functional request packet to update the memory system.\n");
        DPRINTF(SimpleCache, "Function: sendFunctional\n");
        memPort.sendFunctional(pkt);
    }
}

void
SimpleCache::accessTiming(PacketPtr pkt)
{
    DPRINTF(SimpleCache, "Function: accessTiming\n");

    // Perform the functional access of the cache.
    // It either reads or writes the cache on a hit
    // based on the result (hit or miss) of that access.
    bool hit = accessFunctional(pkt);

    DPRINTF(SimpleCache, "%s for packet: %s\n", hit ? "Hit" : "Miss",
            pkt->print());

    /**
     * The size of the request, which is part of the packet, is controlled by the CPU model
     * based on the address range in the request in the packet.
     * For example, |WriteReq [80508:8050b]| means the packet size is 4 (8050b-80508+1).
     * When the CPU performs instruction fetch (labeled as "IF"), the size of the request is always
     * 8 bytes. 
     * When the CPU performs operations other than instruction fetch (labeled with ""), the size
     * of the requst can be 1 or 4 bytes.
     */
    if (hit) {
        DPRINTF(SimpleCache, "Cache hit occurred! Increment cache hit counter.\n");
        //DPRINTF(SimpleCache, "The size of the request %s from the CPU is %u bytes.\n", pkt->print(), pkt->getSize());

        stats.hits++; // update stats

        // Convert the packet from a request packet to a response packet
        // For instance, if the memory command in the packet was a ReadReq,
        // this gets converted into a ReadResp. Then, we can send the 
        // response back to the CPU.
        pkt->makeResponse();

        sendResponse(pkt);
    } else {
        DPRINTF(SimpleCache, "Cache miss occurred! Increment cache miss counter.\n");
        //DPRINTF(SimpleCache, "The size of the request %s from the CPU is %u bytes.\n", pkt->print(), pkt->getSize());

        stats.misses++; // update stats

        missTime = curTick();

        // Forward to the memory side.
        // We can't directly forward the packet unless it is exactly the size
        // of the cache line, and aligned. Check for that here.
        Addr addr = pkt->getAddr();
        Addr block_addr = pkt->getBlockAddr(blockSize);
        unsigned size = pkt->getSize();

        DPRINTF(SimpleCache, "packet addr: %#x      block addr: %#x\n", addr, block_addr);

        if (addr == block_addr && size == blockSize) {
            /**
             * The packet is aligned and the size of the request is the
             * same as that of a cache block. So, we can forward the
             * request to memory.
             */
            // This branch is never taken in this simulation!
            DPRINTF(SimpleCache, "Addr and size are aligned. Forwarding the packet...\n");
            memPort.sendPacket(pkt);
        } else {
            // Need to create a new packet to read the entire cache block from memory.
            DPRINTF(SimpleCache, "Addr and size are unaligned.\n");

            panic_if(addr - block_addr + size > blockSize,
                     "Cannot handle accesses that span multiple cache lines");

            // Unaligned access to one cache block
            assert(pkt->needsResponse());

            MemCmd cmd;

            if (pkt->isWrite() || pkt->isRead()) {
                // Read the data from memory to write into the block.
                // We'll write the data in the cache (i.e., a writeback cache)
                /* 
                 * Here, whether the packet is a read or a write request, we send a
                 * read request to memory to load the data into the cache block.
                 * In the case of a write, it will occur in the cache after we have loaded
                 * the data from memory (i.e., writeback cache).
                 */
                cmd = MemCmd::ReadReq;
            } else {
                panic("Unknown packet type in upgrade size");
            }

            // Create a new packet whose size is specified by |blockSize|
            // Use the original request object in the packet |pkt->req|
            // to let the memory-side objects know the original requestor
            // and the original request type for statistics.
            PacketPtr new_pkt = new Packet(pkt->req, cmd, blockSize);

            // Call the allocate function to allocate memory in the |Packet| object
            // for the data that we will read from memory.
            // Memory in the |Packet| object is freed when we free the object.
            new_pkt->allocate();

            // Address should now be block aligned
            assert(new_pkt->getAddr() == new_pkt->getBlockAddr(blockSize));

            // Save the original packet pointer (pkt) in a member variable so that
            // we can recover it when the SimpleCache receives a response.
            DPRINTF(SimpleCache, "Save the origional packet pointer to |originalPacket|\n");
            originalPacket = pkt;

            DPRINTF(SimpleCache, "forwarding packet\n");

            // Send the new packet across the memory side port.
            DPRINTF(SimpleCache, "Upgrade packet to the size of cache block, and then send the request packet to the DRAM for data.\n");
            memPort.sendPacket(new_pkt);
        }
    }
}

bool
SimpleCache::accessFunctional(PacketPtr pkt)
{
    DPRINTF(SimpleCache, "Function: accessFunctional\n");

    //Check to see if there is an entry in the map which matches the addresses in the packet.

    Addr block_addr = pkt->getBlockAddr(blockSize);

    // Use the |getBlockAddr| function of the |Packet| type to get the block-aligned address.
    DPRINTF(SimpleCache, "Access block address: %#x\n", block_addr); // display unsigned int as hexadecimal

    // Search for that block address in the map.
    // If we do not find the address, the function returns false, meaning that data is not in 
    // the cache (cache miss).
    auto it = cacheStore.find(block_addr);

    if (it != cacheStore.end()) {
        // Entering this branch means a cache hit.
        DPRINTF(SimpleCache, "The requested data exists in cache.\n");

        if (pkt->isWrite()) { // if the packet is a write request, then ...
            DPRINTF(SimpleCache, "Cache write occurred at address %#x! Writing data into the cache block...\n",
                    it->first);

            /**
             * The function |writeDataToBlock| can modifty a cache block pointed by the data pointer |it->second|;
             * this can be verified in the case of a cache miss on a cache read request where you can see the cache
             * cotents are indeed changed after calling |writeDataToBlock|.
             * However, in the case of a cache hit on a cache write request, the cache contents remain unchanged
             * after calling |writeDataToBlock|. The reason is that the data in the hit cache block is overwritten
             * with its CURRENT data. One may think that calling |writeDataToBlock| again in this case is redundant.
             * It turns out that it is not redundant at all. If we comment out |pkt->writeDataToBlock(it->second, blockSize);|,
             * the simulation won't run properply, leading to no statistic data printed in |m5out/stats.txt|.
             * My guess for this is that the |writeDataToBlock| function not only modifies cache contents but also changes the
             * state of some flags. This makes calling this function a necessary step for the simulation to run properply.
             */
            //DPRINTF(SimpleCache, "Before calling writeDataToBlock\n");
            //DPRINTF(SimpleCache, "Print out the whole cache:\n");
            //printMap(cacheStore);
            //DPRINTF(SimpleCache, "Print out the element to be overwritten:\n");
            //DDUMP(SimpleCache, it->second + pkt->getOffset(blockSize), blockSize);

            pkt->writeDataToBlock(it->second, blockSize); // Write the data from the packet into the cache block

            //DPRINTF(SimpleCache, "After calling writeDataToBlock\n");
            //DPRINTF(SimpleCache, "Print out the whole cache:\n");
            //printMap(cacheStore);
            //DPRINTF(SimpleCache, "Print out the element that has been overwritten:\n");
            //DDUMP(SimpleCache, it->second + pkt->getOffset(blockSize), blockSize);
        } else if (pkt->isRead()) {
            DPRINTF(SimpleCache, "Cache read occurred at address %#x! Reading data from the cache block...\n",
                    it->first);
            // Read the data out of the cache block into the packet
            pkt->setDataFromBlock(it->second, blockSize);
        } else {
            panic("Unknown packet type!");
        }
        return true;
    }

    DPRINTF(SimpleCache, "The requested data does not exist in cache.\n");
    return false;
}

void
SimpleCache::printMap(std::unordered_map<Addr, uint8_t*> myMap)
{
    /**
     * Cache blocks (i.e., <addr, data> pairs) are assigned to buckets
     * based on the hash function. This means that
     *  1. Some buckets may be empty.
     *  2. Some buckets may contain multiple cache blocks.
     *  3. The bucket order number (i.e., ID) has no physical interpretation.
     *  4. The bucket size (i.e., number of elements in a single bucket)
     *     has no physical interpretation.
     *  5. The bucket count of the unordered map and the bucket size of each bucket
     *     both depend on the specific compiler you use and is indeterminate
     *     before compilation. Thus, total bucket count may be greater than the
     *     total number of elements (i.e., <addr, data> pairs) in the entire map.
     *  6. Which bucket will have zero element or multiple elements
     *     is indeterminate before compilation.
     * 
     * Thus, bucket count and bucket size have no physical interpretation.
     * 
     */
    unsigned num_buckets = myMap.bucket_count();

    // Some of the buckets are empty by the nature of an unordered map.
    DPRINTF(SimpleCache, "Number of buckets in the unordered map: %u\n", num_buckets);

    for (unsigned i = 0; i < num_buckets; i++) {
        if (myMap.bucket_size(i) != 0)
            DPRINTF(SimpleCache, "Iterator #%u contains %u elements\n", i, myMap.bucket_size(i));
        
        for (auto it = myMap.begin(i); it != myMap.end(i); it++) {
            // Print out all elements in the bucket
            DPRINTF(SimpleCache, "Bucket ID: %u\n", myMap.bucket(it->first));
            DPRINTF(SimpleCache, "addr: %#x\n", it->first);
            DPRINTF(SimpleCache, "data:\n");
            // Given |uint8_t *data = new uint8_t[blockSize];|,
            // we know the size of the data/array in the map <addr, data>
            // is |blockSize|.
            DDUMP(SimpleCache, it->second, blockSize);

        }
    }

        //DPRINTF(SimpleCache, "bucket size = %u\n", myMap.bucket_size(myMap.bucket(bucket.first)));
        //DPRINTF(SimpleCache, "addr: %#x\n", bucket.first);
        //DPRINTF(SimpleCache, "data:\n");

        //DDUMP(SimpleCache, bucket.second, myMap.bucket_size(myMap.bucket(bucket.first)));
        //DDUMP(SimpleCache, myMap[bucket.first], myMap.bucket_size(myMap.bucket(bucket.first)));
}

/**
 * We are modelling the cache block data insertion and eviction rather than functionally
 * implementing such behaviour. Thus, the cache block eviction operation will not
 * actually move the data of the evicted block back to the DRAM; it just sends a mem command
 * through a packet to dictate the memory port how long it should wait before sending
 * a response to a timing reqest.
 */
void
SimpleCache::insert(PacketPtr pkt)
{
    DPRINTF(SimpleCache, "Function: insert\n");
    DPRINTF(SimpleCache, "Insert the packet's data into the cache block.\n");

    // This function is called every time the memory side port responds to a request.

    // The packet should be aligned at this point.
    assert(pkt->getAddr() ==  pkt->getBlockAddr(blockSize));
    // The address should not be in the cache.
    assert(cacheStore.find(pkt->getAddr()) == cacheStore.end());
    // The pkt should be a response.
    assert(pkt->isResponse());

    /**
     * This is a fully associative cache with random block eviction policy.
     * Check if the cache is currently full.
     * If the cache has more entries (blocks) than the capacity
     * of the cache as set by the SimObject parameter,
     * then we need to evict one cache block.
     */
    // Capacity represents the total number of blocks in the cache.
    if (cacheStore.size() >= capacity) {
        DPRINTF(SimpleCache, "Cache is full. Evict a cache block randomly.\n");

        /* Select a random block to evict. */

        int bucket, bucket_size;

        do {
            bucket = random_mt.random(0, (int)cacheStore.bucket_count() - 1);
            /**
             * The index value returned from this loop corresponds to the bucket that is non-empty.
             * Note that it's possible that the index value |bucket| corresponds to a bucket that
             * is undefined (i.e., size is 0), which is why we need to check if it is
             * defined (i.e., non-empty) using the condition in the while loop.
             * It all the buckets were defined, the loop should only be run onece. However,
             * the loop is tipically run multiple times. meaning not all buckets are defined
             * even when the cache starage is full.
             */
            
            //DPRINTF(SimpleCache, "Exp: bucket ID: %d\n", bucket);
            //DPRINTF(SimpleCache, "Exp: bucket size: %d\n", cacheStore.bucket_size(bucket));

            // |bucket_size| returns the number of elements (i.e., cache blocks) in bucket n.
            // When |bucket_size| is 2, it means that there are 2 cache blocks in the same backet.
            // Valid range of buckets is from 0 to bucket_count - 1.
        } while ( (bucket_size = cacheStore.bucket_size(bucket)) == 0 );

        //DPRINTF(SimpleCache, "Exp: final bucket size: %d\n", bucket_size);
        //DPRINTF(SimpleCache, "Exp: final bucket ID: %d\n", bucket);
        /**
         * cacheStore.begin(bucket): Returns the element of the bucket numbered as |bucket|.
         * 
         * random_mt.random(0, bucket_size - 1) returns an integer that is less than bucket_size.
         */

        //DPRINTF(SimpleCache, "Exp: Print out cacheStore BEFORE calling erase function.\n");
        //printMap(cacheStore);

        // The |next| function returns the exact block that
        // is going to be evicted from |cacheStore|. When there are multiple
        // elements (or cache blocks) in a single bucket, a random one will be selected
        // to be evicted later. The random number returned by |random_mt.random|
        // is used to randomly select an element in a single bucket.
        int rand_num = random_mt.random(0, bucket_size - 1);
        //DPRINTF(SimpleCache, "Exp: Random number used to select the element in a single bucket is %d.\n", rand_num);
        auto block = std::next(cacheStore.begin(bucket), rand_num);

        DPRINTF(SimpleCache, "Removing the cache block with starting address %#x\n", block->first);


        // On an eviction, we need to write the data back to the memory in case it has been updated.

        // Create a new request-packet pair in two steps.

        // Step 1: Create a |Request| object that is owned by |req| pointer.
        // Parameter 1: Physical address of the request
        // Parameter 2: Size of the request
        // Parameter 3: Value of |flags|. Don't know what it does.
        // Parameter 4: Requestor ID
        RequestPtr req = std::make_shared<Request>(block->first, blockSize, 0, 0);

        // Step 2: Use the |req| pointer to create a Packet referenced by the pointer |new_pkt|.
        // The memory command variable |WritebackDirty| is used to model the memory access latency
        // for writing dirty data in the cache block back to the DRAM. So, it is part of the packet
        // that is to be sent to the DRAM.
        PacketPtr new_pkt = new Packet(req, MemCmd::WritebackDirty, blockSize);

        new_pkt->dataDynamic(block->second); // This will be deleted later

        DPRINTF(SimpleCache, "Writing packet (%s) back into the DRAM.\n", new_pkt->print());

        // Send the packet to be written across the memory side port (memPort)
        memPort.sendPacket(new_pkt);

        DPRINTF(SimpleCache, "Erase the entry in the cache storage map.\n");
        // Removes from the unordered_map container a single element.
        cacheStore.erase(block->first);

        //DPRINTF(SimpleCache, "Exp: Print out cacheStore AFTER calling erase function.\n");
        //printMap(cacheStore);
    }

    DPRINTF(SimpleCache, "Inserting %s\n", pkt->print());
    
    DPRINTF(SimpleCache, "Dump the packet's data fetched from the DRAM.\n");
    DDUMP(SimpleCache, pkt->getConstPtr<uint8_t>(), blockSize);

    // Allocate space for the cache block data
    // This means that the size of the data in <addr, data> pair
    // is of |blockSize| bytes!
    uint8_t *data = new uint8_t[blockSize];

    // Insert the pointer of data-address pair into the cache store
    cacheStore[pkt->getAddr()] = data;

    DPRINTF(SimpleCache, "WDTB: Before calling writeDataToBlock\n");
    printMap(cacheStore);
    // Fill the cache block with the data fetched from DRAM via the pointer |data|.
    pkt->writeDataToBlock(data, blockSize);
    DPRINTF(SimpleCache, "WDTB: After calling writeDataToBlock\n");
    printMap(cacheStore);

    DPRINTF(SimpleCache, "Data written into the cache block:\n");
    DDUMP(SimpleCache, data, blockSize);
}

AddrRangeList
SimpleCache::getAddrRanges() const
{
    DPRINTF(SimpleCache, "Function: getAddrRanges\n");
    
    DPRINTF(SimpleCache, "Sending new ranges\n");
    // Just use the same ranges as whatever is on the memory side.
    return memPort.getAddrRanges();
}

void
SimpleCache::sendRangeChange() const
{
    DPRINTF(SimpleCache, "Function: sendRangeChange\n");
    
    for (auto& port : cpuPorts) {
        port.sendRangeChange();
    }
}

SimpleCache::SimpleCacheStats::SimpleCacheStats(statistics::Group *parent)
      : statistics::Group(parent),
      ADD_STAT(hits, statistics::units::Count::get(), "Number of hits"),
      ADD_STAT(misses, statistics::units::Count::get(), "Number of misses"),
      ADD_STAT(missLatency, statistics::units::Tick::get(),
               "Ticks for misses to the cache"),
      ADD_STAT(hitRatio, statistics::units::Ratio::get(),
               "The ratio of hits to the total accesses to the cache",
               hits / (hits + misses))
{
    missLatency.init(16); // number of buckets
}

} // namespace gem5
