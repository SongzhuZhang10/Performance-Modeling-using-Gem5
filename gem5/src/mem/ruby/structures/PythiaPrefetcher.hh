/*
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) Songzhu Zhang
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

#ifndef __MEM_RUBY_STRUCTURES_PYTHIA_PREFETCHER_HH__
#define __MEM_RUBY_STRUCTURES_PYTHIA_PREFETCHER_HH__

#include "base/circular_queue.hh"
#include "base/statistics.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/slicc_interface/RubyRequest.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "params/PythiaPrefetcher.hh"
#include "sim/sim_object.hh"

#include <deque>
#include <tuple>
#include <random>
#include <vector>
#include <unordered_map>
#include <utility>
#include <functional>
#include <bitset>
#include <algorithm>
#include <limits>
#include <cassert>
#include <cstdlib>
#include <memory>
/**
 * The cache line size must be exactly 64 bytes for this prefetcher to work!
 * A no-prefetch action is taken when an action that prefetches beyond the
 * current physical page.
 * 
 */
/**
 * State-vector is defined as a combination of several features.
 * The two constituent features of the state-vector of Pythia prefetcher's are
 * PC+Delta and Sequence of last-4 deltas.
 */
namespace gem5
{

namespace ruby
{

typedef enum
{
    no_prefetch = -2,
    inaccurate = -14,
    loss_of_coverage = -12,
    accurate_late = 12,
    accurate_timely = 20,
    none = 0
} Reward;

std::string RewardType_to_string(const Reward& reward);

// uint32_t FeatureKnowledge::process_PC_delta
template <typename T, typename U>
struct PairHasher {
public:
    Addr operator()(const std::pair<T, U>& p) const {
        // Combine hashes using XOR and bit shifts for better distribution
        Addr hash_value = std::hash<T>()(p.first) ^ (std::hash<U>()(p.second) << 1) ^ (std::hash<U>()(p.second) >> 1);
        // Multiply by a prime number for better distribution
        const Addr prime_number = 0x9E3779B9;  // Large prime number for spreading
        hash_value *= prime_number;

        return hash_value;
    }
};

template <typename T>
struct DequeHasher {
    Addr operator()(const std::deque<T>& deque) const {
        // Combine hash values of elements with appropriate prime multipliers
        size_t hash = 0;
        size_t prime = 31; // A well-chosen prime number for dispersion
        for (const T& element : deque) {
            hash = hash * prime + std::hash<T>()(element);
        }

        // Final mixing to enhance distribution
        hash ^= hash >> 16;
        hash *= 0x85ebca6b;
        hash ^= hash >> 13;
        hash *= 0xc2b2ae35;
        hash ^= hash >> 16;

        return static_cast<Addr>(hash);
    }
};

using StateType = Addr;
using StatePair = std::pair<StateType, StateType>;
using StateActionPair = std::pair<StateType, int>;
using Vault = std::unordered_map<StateActionPair, float, PairHasher<StateType, int>>;

class QVStore
{
public:
    QVStore(unsigned size, float alpha, float gamma, float epsilon);

    void updateQ(Reward r, StatePair s1, int a1, StatePair s2, int a2);

    std::pair<float, int> seachMaxQAndActionInVault(int idx, Addr state, const std::vector<int>& action_list);

    int getActionWithMaxQ(const StatePair& state_pair,
        const std::vector<int>& action_list);

    void printQVStore() const;

    unsigned getNumUpdates(int idx) const;

private:
    std::vector<Vault> vaultVector;
    const int m_num_vaults;
    const float m_alpha;
    const float m_gamma;
    const float m_epsilon;
    const float m_default_q;
    

    unsigned numVault0Updates; // Number of vault 0 updates
    unsigned numVault1Updates; // Number of vault 1 updates

    float getQFromVault(int idx, StateActionPair key, int action);
    std::tuple<int, Addr, float> searchMaxQTuple(StatePair state_pair, int action);

};

class EQEntry
{
public:
    EQEntry(Addr addr, Addr _pc, StatePair state_pair, int _action, Reward reward);

    inline bool hasReward() { return rewardType != Reward::none; }
    inline void reward(Reward reward) { rewardType = reward; }
    inline Reward getReward() const { return rewardType; }
    inline Addr getPrefAddr() const { return prefAddr; }
    inline int getAction() const { return action; }
    inline bool isFilled() const { return filled; }
    inline void setFill() { filled = true; }
    inline void unsetFill() { filled = false; }
    inline StatePair getStatePair() const { return statePair; }
    inline void storeStatePair(StatePair state_pair) { statePair = state_pair; }
    void printEQEntry() const;

private:
    Addr prefAddr; // prefetch address generated for the corresponding action
    Addr pc;
    StatePair statePair;
    int action;         // the taken action
    Reward rewardType;  // for debugging
    bool filled; // true when the prefetch request has been filled into the cache
};

class PythiaPrefetcher : public SimObject
{
public:
    typedef PythiaPrefetcherParams Params;
    PythiaPrefetcher(const Params& p);
    ~PythiaPrefetcher() = default;

    /* Called for every demand request */
    void trainAndPredict(Addr demand_address, Addr pc,
        const RubyRequestType& type);

    /**
     * For every prefetch fill, search the address in EQ and mark the
     * corresponding EQ entry as filled
     */
    void prefetch_fill(Addr addr); // Invoked in the SLICC file.
    
    /**
     * This function will be called in the generated L1Cache_Controller.cc file.
     * The C++ code generated by the SLICC file will call this function;
     * however, it does not use smart pointers. Thus, we have to use a raw
     * pointer here.
     */
    void setController(AbstractController *_ctrl) { m_controller = _ctrl; }

    void observeHit(Addr demand_address, Addr pc, const RubyRequestType& type);
    void observePfMiss(Addr demand_address, Addr pc, const RubyRequestType& type);
    void observeMiss(Addr demand_address, Addr pc, const RubyRequestType& type);

private:
    std::deque<std::shared_ptr<EQEntry>> eval_que;
    // Assume last-n deltas are for both load and non-load instructions.
    std::deque<int> deltas; // used to store sequence of last-4 deltas

    const unsigned evalQueCapacity;
    const unsigned deltaQueCapacity;
    const unsigned prefOffsetQueCapacity;

    const Addr invalidAddr;
    Addr demandAddr;
    Addr lastPrefetchedAddr;
    
    const float m_alpha;
    const float m_gamma;
    const float m_epsilon;
    Addr m_pc_delta; // Hash value of PC+delta
    Addr m_delta_sig; // Signature of the last 4 deltas
    const bool m_prefetch_cross_pages;
    const unsigned m_delta_bits;

    std::deque<int> m_pythia_offsets;
    int action;
    
    /**
     * 4KB (i.e., 2^12 bytes) page size ->
     * pageOffsetBits (i.e., bit width of page offset)= 12
     */
    const unsigned pageOffsetBits;
    // Elements represent prefetch offset index by `action`.
    const std::vector<int> actionList;

    int qMax;
    std::default_random_engine generator;
    // Returns true with a probability of epsilon and false with a probability of (1 - epsilon).
    std::bernoulli_distribution explore;
    // Generate a random integer between 0 and (numActions - 1) (inclusive).
    std::uniform_int_distribution<unsigned> actionGen;
    unsigned num_vaults; // Number of vaults in the QVStore
    QVStore qVStore;

    // When this is flase, L1 instruction prefetch is enabled.
    bool l1_data_prefetch;
    bool l2_prefetch;
    Addr debug_cycle;
    RubyRequestType requestType;
    AbstractController *m_controller;
    std::shared_ptr<EQEntry> last_evicted_eq_entry;

    void insertDelta(const int delta);

    StatePair getCurrStatePair() const;

    void issueNextPrefetch(Addr addr, Addr pc, const RubyRequestType& type);

    /* Insert the entry. Get the evicted EQ entry. */
    void insertEvalQue(const std::shared_ptr<EQEntry>& new_entry);

    /// determine the page aligned address
    Addr getPageAlignedAddr(Addr addr) const;
    unsigned getPageNum(Addr addr) const;
    unsigned getPageOffset(Addr addr) const;
    int getStrideOffset(Addr addr) const;
    int getStride(Addr addr, int action) const;

    void updateEnvState(Addr pc, int pref_offset, RubyRequestType type);
    int chooseAction(const StatePair& state_pair);

    struct Stats : public statistics::Group
    {
        Stats(statistics::Group *parent);

        statistics::Scalar numPrefetchRequested;
        statistics::Scalar numAccurateAndTimely;
        statistics::Scalar numAccurateButLate;
        statistics::Scalar numInaccurate;
        statistics::Scalar numUnrewardedEntries;

        statistics::Scalar numObservedHits;
        statistics::Scalar numMissObserved;
        statistics::Scalar numPartialHits;
        statistics::Scalar numPageCross;
        statistics::Scalar numEvalQueHits;
        statistics::Scalar numNoPref;
        statistics::Scalar numPrefFill;
        statistics::Scalar numDemands;
        statistics::Scalar numEQInsertions;
        statistics::Scalar numEQEvictions;

    } stats;

    void printEvalQue() const;
    void printStateVector(Addr line_addr, Addr pc) const;
    void printQVStore() const;
    void printDebugCycle(Addr demand_address, Addr pc) const;
    bool printEnable() const;
};

} // namespace ruby
} // namespace gem5
#endif // __MEM_RUBY_STRUCTURES_PYTHIA_PREFETCHER_HH__
