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

/**
 * Implementation of the Pythia Prefetcher (aka Scooby Prefetcher)
 * 
 * References:
 *  R. Bera, K. Kanellopoulos, A. Nori, Taha Shahroodi, Sreenivas Subramoney,
 *  and O. Mutlu, “Pythia: A Customizable Hardware Prefetching Framework Using
 *  Online Reinforcement Learning,” arXiv (Cornell University), Oct. 2021, doi:
 *  https://doi.org/10.1145/3466752.3480114.
 * 
 * Much of the code was derived from my interpretation of the referenced paper.
 * For prefetcher design details that are either omitted or ambiguous in the
 * paper, I had to consult the original code used by the authors for
 * architectural exploration. This process facilitated the identification of
 * all necessary design details not explicitly mentioned in the paper.
 */

#ifndef __MEM_CACHE_PREFETCH_SCOOBY_HH__
#define __MEM_CACHE_PREFETCH_SCOOBY_HH__

#include <deque>
#include <random>
#include <vector>
#include <cstdlib>
#include <memory>
#include "params/ScoobyPrefetcher.hh"
#include "mem/cache/prefetch/queued.hh"

namespace gem5
{

struct ScoobyPrefetcherParams;

namespace prefetch
{

enum class RewardValue
{
    R_np_h  = -2,
    R_np_l  = -4,
    R_in_h  = -14,
    R_in_l  = -8,
    R_cl    = -12,
    R_al    = 12,
    R_at    = 20,
    none    = 0
};

enum class RewardType
{
    no_prefetch,
    inaccurate,
    loss_of_coverage,
    accurate_late ,
    accurate_timely,
    none
};

typedef enum
{
    F_PC_Delta = 0, // 0
    F_Delta_Path,   // 1

    NumFeatureTypes
} FeatureType;

std::string RewardType_to_string(const RewardType& reward_type);

class State
{
private:
    Addr pc;
    Addr addr; // line addr pf phi
    Addr page; // page number
    uint32_t offset;
    int32_t delta;
    uint32_t delta_path; // signature of the last four deltas

public:
    State(Addr _pc, Addr _addr, Addr _page, uint32_t _offset, int32_t _delta,
        uint32_t _delta_path) : pc(_pc), addr(_addr), page(_page),
        offset(_offset), delta(_delta), delta_path(_delta_path)
    {
        assert(page == addr >> 12);
    }

    uint32_t getDeltaPath() const { return delta_path; }
    Addr getPC() const { return pc; }
    Addr getPage() const { return page; }
    int32_t getDelta() const { return delta; }
    uint32_t getOffset() const { return offset; }
};

class Vault
{
public:

    Vault(uint32_t num_tilings, uint32_t num_tiles, uint32_t num_actions,
        FeatureType feature_type, float alpha, float gamma,
            bool use_tiling_offset, float default_q);

    void updateQ(int32_t r0, const std::shared_ptr<State>& s0, int32_t a0,
        const std::shared_ptr<State>& s1, int32_t a1);

    float retrieveQ(const std::shared_ptr<State>& state, uint32_t action) const;

private:

    const uint32_t m_num_tilings; // always 3
    const uint32_t m_num_tiles; // always 128 
    const uint32_t m_num_actions;

    const FeatureType m_feature_type;

    const float m_alpha;
    const float m_gamma;

    const bool m_use_tiling_offset;
    std::vector<std::vector<std::vector<float>>> m_qtable;
    std::vector<uint32_t> tiling_offset;

    uint32_t process_pc_delta(uint32_t tiling, uint64_t pc, int32_t delta) const;
    uint32_t process_delta_path(uint32_t tiling, uint32_t delta_path) const;
    uint32_t folded_xor(uint64_t value, uint32_t num_folds) const;
    uint32_t getHash(uint32_t key) const;
    float getQ(uint32_t tiling, uint32_t tile_index, uint32_t action) const;
    void setQ(uint32_t tiling, uint32_t tile_index, uint32_t action, float value);
    
    uint32_t getTileIndex(const std::shared_ptr<State>& state, uint32_t tiling) const;
};

class QVStore
{
public:

    QVStore(float alpha, float gamma, float epsilon, uint32_t num_tilings,
        uint32_t num_tiles, uint32_t num_actions, bool use_tiling_offset);

    uint32_t chooseAction(const std::shared_ptr<State>& state);

    void train(int32_t r0, const std::shared_ptr<State>& s0, int32_t a0, const std::shared_ptr<State>& s1, int32_t a1);

private:
    
    const float m_alpha;
    const float m_gamma;
    // Returns true with a probability of epsilon and false with a probability of (1 - epsilon).
    std::bernoulli_distribution explore;

    const uint32_t m_num_tilings;
    const uint32_t m_num_tiles;
    const uint32_t m_num_actions;
    const bool m_use_tiling_offset;
    const float m_default_q;
    std::vector<std::shared_ptr<Vault>> vaults;

    std::default_random_engine generator;
    // Generate a random integer between 0 and (numActions - 1) (inclusive).
    std::uniform_int_distribution<int32_t> actionGen;

    uint32_t getMaxAction(const std::shared_ptr<State>& state) const;
    float consultQ(const std::shared_ptr<State>& state, uint32_t action) const;
};

class SigTableEntry
{
private:
    class ActionTracker
    {
    public:
        int32_t action_offset;
        int32_t conf;
        ActionTracker(int32_t _action_offset, int32_t _conf) : action_offset(_action_offset), conf(_conf) {}
    };

    Addr page;
    std::deque<Addr> pcs;
    std::deque<uint32_t> offsets;
    std::deque<int32_t> deltas; // used to store sequence of last-4 deltas
    std::bitset<64> bmp;
    const uint32_t numActionTrackers;
    std::deque<std::shared_ptr<ActionTracker>> actionTrackers;


public:
    SigTableEntry(Addr _page, Addr pc, uint32_t offset)
    : page(_page), numActionTrackers(2) {
        pcs.clear();
        offsets.clear();
        deltas.clear();

        pcs.push_back(pc);
        offsets.push_back(offset);
    }

    int32_t getLatestDelta() const;
    uint32_t computeDeltaPath() const;
    void update(Addr _page, Addr pc, uint32_t offset);
    Addr getPageNum() const { return page; }
    void printDeltas() const;
    void printOffsets() const;
    void trackPrefetch(int32_t predicted_offset, int32_t action_offset);
    void insertAction(int32_t action); // same as `insert_action_tracker`
    bool searchActionIndex(int32_t action_offset, int32_t& conf);
    void printActionTrackers() const;
};

class EqEntry
{
private:
    Addr addr; // prefetch address generated for the corresponding action
    const std::shared_ptr<State> state; // The addr in `state` is the packet addr rather than pref addr.
    int32_t action;     // the taken action
    RewardType rewardType;
    int32_t rewardValue;
    bool fill; // true when the prefetch request has been filled into the cache

public:
    EqEntry(Addr _addr, const std::shared_ptr<State> _state, int32_t _action)
    : addr(_addr), state(_state), action(_action), rewardType(RewardType::none),
    rewardValue(static_cast<int32_t>(RewardValue::none)), fill(false) {}

    bool hasReward() const;
    void setRewardType(RewardType reward_type) { rewardType = reward_type; }
    void setRewardValue(int32_t reward_value) { rewardValue = reward_value; }
    RewardType getRewardType() const { return rewardType; }
    int32_t getRewardValue() const { return rewardValue; }
    Addr getPrefAddr() const { return addr; }
    int32_t getAction() const { return action; }

    bool isFilled() const { return fill; }
    void setFill() { fill = true; }
    void unsetFill() { fill = false; }
    void print() const;
    const std::shared_ptr<State> getState() const { return state; }
};

struct EpochStats
{
    uint64_t cacheMisses;
    uint64_t totalPfs;
    uint64_t usefulPfs;

    EpochStats() : cacheMisses(0), totalPfs(0), usefulPfs(0) {}
    void reset() {
        cacheMisses = 0;
        totalPfs = 0;
        usefulPfs = 0;
    }
};

class Scooby : public Queued
{
public:
    Scooby(const ScoobyPrefetcherParams& p);
    ~Scooby() = default;

    void startup() override;
    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses,
                           const CacheAccessor &cache) override;


private:
    std::deque<std::shared_ptr<EqEntry>> eval_que;
    std::deque<std::shared_ptr<SigTableEntry>> sig_table; // signature table

    const bool m_secure_mode;
    const bool m_enable_dyn_degree;
    const uint32_t evalQueCapacity;
    const uint32_t sigTableCapacity;

    const Addr invalidAddr;
    
    /**
     * 4KB (i.e., 2^12 bytes) page size ->
     * pageOffsetBits (i.e., bit width of page offset)= 12
     */
    const uint32_t pageOffsetBits;
    
    // Elements represent prefetch offset index by `action`.
    const std::vector<int32_t> actionList;

    const std::vector<int32_t> m_conf_thresholds;
    const std::vector<int32_t> m_deg_normal;
    const bool enable_reward_all;
    std::shared_ptr<EqEntry> last_evicted_eq_entry;

    EventFunctionWrapper epochEvent;
    EpochStats epochStats;

    /** Off chip memory latency to use for the epoch bandwidth calculation */
    const Tick offChipMemoryLatency;

    /** Cycles in an epoch period */
    const Cycles epochCycles;

    float m_bandwidth;
    float maxBandwidth;
    const float m_high_bw_threshold;

    uint64_t debug_cycle;

    struct Stats : public statistics::Group
    {
        Stats(statistics::Group *parent);
        statistics::Scalar demands;
        statistics::Scalar pfIssued;
        statistics::Scalar multiDeg;
        statistics::Scalar accurateAndTimely;
        statistics::Scalar accurateButLate;
        statistics::Scalar inaccurate;
        statistics::Scalar pageCross;
        statistics::Scalar evalQueHits;
        statistics::Scalar evalQueMisses;
        statistics::Scalar noPref;
        statistics::Scalar prefFill;
        statistics::Scalar reads;
        statistics::Scalar writes;
        statistics::Scalar inp_h_bw;
        statistics::Scalar inp_l_bw;
        statistics::Scalar np_h_bw;
        statistics::Scalar np_l_bw;
    } stats;

    std::unique_ptr<QVStore> qVStore;

    /* Called for every demand request */
    void rewardEqEntry(Addr pkt_addr);
    void predict(Addr pkt_addr, const std::shared_ptr<State>& state, std::vector<AddrPriority>& addresses);

    Addr getPageNum(Addr addr) const;
    uint32_t getScoobyOffset(Addr addr) const;
    int32_t getPredictedOffset(Addr pkt_addr, int32_t action_offset) const;
    Addr computePrefAddr(Addr addr, int32_t offset) const;
    Addr getDynPrefDegree(Addr page, int32_t action_offset) const;
    std::shared_ptr<SigTableEntry> updateEnvState(Addr pc, Addr page_num, int32_t scooby_offset);
    std::vector<std::shared_ptr<EqEntry>> searchEq(Addr addr, bool search_all) const;

    void registerFill(const CacheAccessor &cache);
    void reward(const std::shared_ptr<EqEntry>& entry);
    bool track(Addr addr, const std::shared_ptr<State>& state, uint32_t action, std::shared_ptr<EqEntry>& eq_entry);
    void assignReward(const std::shared_ptr<EqEntry>& entry, RewardType reward_type);
    void genMultiDegreePref(Addr page, uint32_t scooby_offset, int32_t action, uint32_t pref_degree, std::vector<AddrPriority>& addresses);
    void trackAction(Addr page, int32_t predicted_offset, int32_t action_offset); // same as `track_in_st`

    bool isHighBW() const;
    int32_t computeReward(RewardType reward_type);
    void processEpochEvent();

    void printEvalQue() const;
    void printDebugInfo(Addr page_num, Addr pkt_addr, Addr line_addr, Addr pc,
        uint32_t offset, const std::shared_ptr<SigTableEntry>& entry) const;
    void printActTackersInSigTable() const;
};

} // namespace ruby
} // namespace gem5
#endif // __MEM_CACHE_PREFETCH_SCOOBY_HH__
