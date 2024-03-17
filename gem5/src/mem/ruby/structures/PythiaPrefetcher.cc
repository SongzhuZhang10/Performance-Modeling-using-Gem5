/*
 *
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

#include "base/bitfield.hh"
#include "debug/PythiaPrefetcher.hh"
#include "mem/ruby/slicc_interface/RubySlicc_ComponentMapping.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "mem/ruby/structures/PythiaPrefetcher.hh"
#include <cmath>

namespace gem5
{

namespace ruby
{

std::string
RewardType_to_string(const Reward& reward)
{
    switch(reward) {
        case no_prefetch:
            return "No Prefetch";
        case inaccurate:
            return "Inaccurate";
        case loss_of_coverage:
            return "Loss of Coverage";
        case accurate_late:
            return "Accurate Late";
        case accurate_timely:
            return "Accurate Timely";
        case none:
            return "None";
        default:
            panic("Invalid reward!");
    }
    return "";
}

// ****************************************************************************
// QVStore Member Function definitions
// ****************************************************************************
QVStore::QVStore(unsigned size, float alpha, float gamma, float epsilon)
    : vaultVector(size),
    m_num_vaults(size),
    m_alpha(alpha),
    m_gamma(gamma),
    m_epsilon(epsilon),
    m_default_q(1.0f / (1.0f - gamma)),
    numVault0Updates(0),
    numVault1Updates(0)
{
};

/**
 * The Q-value of the evicted state-action pair is updated via the SARSA
 * algorithm (lines 26-29), using
 *      - the reward stored in the evicted EQ entry and
 *      - the Q-value of the state-action pair in the head of the EQ-entry
 */
void
QVStore::updateQ(Reward r, StatePair s1, int a1, StatePair s2, int a2)
{
    assert(r != Reward::none);

    auto q1_tuple = searchMaxQTuple(s1, a1);
    float q1 = std::get<2>(q1_tuple);
    int q1_idx = std::get<0>(q1_tuple);

    auto q2_tuple = searchMaxQTuple(s2, a2);
    float q2 = std::get<2>(q2_tuple);
    
    /* Perform the SARSA update */
    q1 = q1 + m_alpha * (static_cast<float>(r) + m_gamma * q2 - q1);

    if (q1_idx == 0) {
        auto key0 = std::make_pair(s1.first, a1);
        assert(vaultVector[q1_idx].find(key0) != vaultVector[q1_idx].end());
        vaultVector[q1_idx].at(key0) = q1;
        numVault0Updates++;
        //DPRINTF(PythiaPrefetcher, "Vault[%d] at <0x%6x, %d> was updated to %f.\n", q1_idx, key0.first, key0.second, q1);
    } else {
        assert(q1_idx == 1);
        auto key1 = std::make_pair(s1.second, a1);
        assert(vaultVector[q1_idx].find(key1) != vaultVector[q1_idx].end());
        vaultVector[q1_idx].at(key1) = q1;
        numVault1Updates++;
        //DPRINTF(PythiaPrefetcher, "Vault[%d] at <0x%6x, %d> was updated to %f.\n", q1_idx, key1.first, key1.second, q1);
    }
}

std::tuple<int, Addr, float>
QVStore::searchMaxQTuple(StatePair state_pair, int action)
{
    auto key0 = std::make_pair(state_pair.first, action);
    auto key1 = std::make_pair(state_pair.second, action);

    float q_s0 = getQFromVault(0, key0, action);
    float q_s1 = getQFromVault(1, key1, action);

    if (q_s0 > q_s1) {
        return std::make_tuple(0, state_pair.first, q_s0);
    } else {
        return std::make_tuple(1, state_pair.second, q_s1);
    }
}

float
QVStore::getQFromVault(int idx, StateActionPair key, int action)
{
    assert(key.first != 0);
    float q;
    if (vaultVector[idx].find(key) != vaultVector[idx].end()) {
        q = vaultVector[idx].at(key);
    } else {
        q = m_default_q;
        vaultVector[idx].insert({key, q});
    }
    return q;
}

std::pair<float, int>
QVStore::seachMaxQAndActionInVault(int idx, Addr state, const std::vector<int>& action_list)
{
    // uint32_t LearningEngineBasic::getMaxAction(uint32_t state)
    float q_max = -1000.0f;
    int action = -1; // default action

    // Because the default value of action is 0, init value of idx can be 1.
    for (int i = 0; i < action_list.size(); ++i) {
        std::pair<Addr, int> key = std::make_pair(state, i);
        float q_tmp = getQFromVault(idx, key, i);
        if (q_tmp > q_max) {
            q_max = q_tmp;
            action = i;
        }
    }
    assert(action > -1);
    return std::make_pair(q_max, action);
}

int
QVStore::getActionWithMaxQ(const StatePair& state_pair,
    const std::vector<int>& action_list)
{
    auto [q0, a0] = seachMaxQAndActionInVault(0, state_pair.first, action_list);
    auto [q1, a1] = seachMaxQAndActionInVault(1, state_pair.second, action_list);

    if (q0 > q1)
        return a0;
    else
        return a1;
}

unsigned
QVStore::getNumUpdates(int idx) const
{
    if (idx == 0)
        return numVault0Updates;
    else if (idx == 1)
        return numVault1Updates;
    else {
        panic("Invalid vault index\n");
        return 0;
    }
}
// ****************************************************************************
// EQEntry Member Function definitions
// ****************************************************************************
EQEntry::EQEntry(Addr addr, Addr _pc, StatePair state_pair, int _action, Reward reward)
    : prefAddr(addr), pc(_pc), statePair(state_pair), action(_action), rewardType(reward), filled(false)
{
}

void
EQEntry::printEQEntry() const
{
    DPRINTF(PythiaPrefetcher, "Pref Addr    = 0x%6x,                 PC           = 0x%6x\n", prefAddr, pc);
    DPRINTF(PythiaPrefetcher, "Action       = %d,                        Reward Type  = %s\n", action, RewardType_to_string(rewardType));
    DPRINTF(PythiaPrefetcher, "State        = <0x%6x, 0x%6x>,   Filled       = %d\n", statePair.first, statePair.second, filled);
}
// ****************************************************************************
// PythiaPrefetcher Member Function definitions
// ****************************************************************************
PythiaPrefetcher::PythiaPrefetcher(const Params& p)
    : SimObject(p),
    evalQueCapacity(p.eval_que_capacity),
    deltaQueCapacity(p.delta_que_capacity),
    prefOffsetQueCapacity(1),
    invalidAddr(0xFFFFFFFFFFFFFFFF),
    demandAddr(0),
    lastPrefetchedAddr(0),
    m_alpha(p.alpha),
    m_gamma(p.gamma),
    m_epsilon(p.epsilon),
    m_pc_delta(0),
    m_delta_sig(0),
    m_prefetch_cross_pages(false),
    m_delta_bits(p.page_offset_bits - RubySystem::getBlockSizeBits()),
    action(0),
    pageOffsetBits(p.page_offset_bits),
    actionList({-6, -3, -1, 0, 1, 3, 4, 5, 10, 11, 12, 16, 22, 23, 30, 32}),   
    qMax(0),
    //generator(std::random_device{}()),
    explore(p.epsilon),
    num_vaults(2),
    qVStore(2, p.alpha, p.gamma, p.epsilon),
    debug_cycle(0),
    stats(this)
{
    generator.seed(3);
    actionGen = std::uniform_int_distribution<unsigned>(0, actionList.size() - 1);
    assert(evalQueCapacity == 256);
    assert(deltaQueCapacity == 4);
    assert(pageOffsetBits > 0);
    assert(m_alpha > 0);
    assert(m_gamma > 0);
    assert(m_epsilon > 0);
    assert(m_delta_bits > 0);
}

int
PythiaPrefetcher::chooseAction(const StatePair& state_pair)
{
    int action = -1;
    if (explore(generator)) {
        action = actionGen(generator); // take random action
    } else {
        action = qVStore.getActionWithMaxQ(state_pair, actionList);
    }
    return action;
}

PythiaPrefetcher::
Stats::Stats(statistics::Group *parent)
    : statistics::Group(parent, "Pythia"),
      ADD_STAT(numPrefetchRequested, "Number of prefetch requests made"),
      ADD_STAT(numAccurateAndTimely, "Number of accurate and timely rewards"),
      ADD_STAT(numAccurateButLate, "Number of accurate but late rewards"),
      ADD_STAT(numInaccurate, "Number of inaccurate rewards"),
      ADD_STAT(numUnrewardedEntries, "Number of unrewarded entries currently in the evaluation que"),

      ADD_STAT(numMissObserved, "Number of misses observed"),
      ADD_STAT(numPartialHits, "Number of misses observed for a block being prefetched"),
      ADD_STAT(numPageCross, "Number of prefetches across pages"),
      ADD_STAT(numEvalQueHits, "Number of hits in evaluation queue"),
      ADD_STAT(numNoPref, "Number of no prefetch rewards"),
      ADD_STAT(numPrefFill, "Number of effective prefetch fills"),
      ADD_STAT(numDemands, "Number of demands"),
      ADD_STAT(numEQInsertions, "Number of entries inserted into the evaluation que"),
      ADD_STAT(numEQEvictions, "Number of entries evicted from the evaluation que")
{
}

void
PythiaPrefetcher::observeMiss(Addr demand_address, Addr pc, const RubyRequestType& type)
{
    // Cache miss observed
    stats.numMissObserved++;
    //DPRINTF(PythiaPrefetcher, "Observed miss for 0x%6x\n", printAddress(demand_address));
    trainAndPredict(demand_address, pc, type);
}

void
PythiaPrefetcher::recordPartialHits(Addr demand_address, Addr pc)
{
    stats.numPartialHits++;
    //DPRINTF(PythiaPrefetcher, "Observed partial hit for 0x%6x\n", printAddress(demand_address));
}

StatePair
PythiaPrefetcher::getCurrStatePair() const
{
    return std::make_pair(m_pc_delta, m_delta_sig);
}

void
PythiaPrefetcher::updateEnvState(Addr pc, int pythia_offset, RubyRequestType type)
{
//if (type == RubyRequestType_LD /*|| type == RubyRequestType_IFETCH*/ || type == RubyRequestType_ST || type == RubyRequestType_ATOMIC) {
    if (type == RubyRequestType_LD) {
        if (!m_pythia_offsets.empty()) {
            // delta is defined as the difference between the current and the last cache aligned addresses.
            // It can be calculated by the difference between the current and the last prefetch offsets.
            // Note that delta can be negative values.
            int delta = (pythia_offset > m_pythia_offsets.back()) ? (pythia_offset - m_pythia_offsets.back()) : (-1)*(m_pythia_offsets.back() - pythia_offset);
            insertDelta(delta);

            // It's guaranteed that delta can be represented by 6 bits given 4KB page size.
            m_pc_delta = PairHasher<Addr, int>()(std::make_pair(pc, delta));
        }
    }
    // Insert prefetch offset
    if (m_pythia_offsets.size() == prefOffsetQueCapacity) {
        m_pythia_offsets.pop_front();
    }
    m_pythia_offsets.push_back(pythia_offset);
}

Addr
PythiaPrefetcher::getPageAlignedAddr(Addr addr) const
{
    /**
     * @param pageOffsetBits Number of bits from low to high to mask to get the
     * page aligned address which is a 64-bit value with all the bits whose bit
     * positions are lower than the one specified by `pageOffsetBits` being zero.
     * pageOffsetBits represents the bitwidth of page offset.
     */

    assert(
        ((addr >> pageOffsetBits) << pageOffsetBits) == mbits<Addr>(addr, 63, pageOffsetBits)
    );
    return mbits<Addr>(addr, 63, pageOffsetBits);
}

unsigned
PythiaPrefetcher::getPageNum(Addr addr) const
{
    assert(bits<Addr>(addr, 63, pageOffsetBits) == addr >> pageOffsetBits);
    return bits<Addr>(addr, 63, pageOffsetBits);
}

unsigned
PythiaPrefetcher::getPageOffset(Addr addr) const
{
    return mbits<Addr>(addr, pageOffsetBits, 0);
}

void
PythiaPrefetcher::insertDelta(const int delta)
{    
    if (deltas.size() == deltaQueCapacity) {
        deltas.pop_front();
    }
    deltas.push_back(delta);
    assert(deltas.size() <= deltaQueCapacity);
    
    if (deltas.size() == deltaQueCapacity) // If last 4 deltas are ready
        m_delta_sig = DequeHasher<int>()(deltas);
}

/**
 * Pythia offset is defined as the binary value extracted from the starting
 * index bit of the address to the last bit of page offset field.
 * For computer system whose cache block size is 64 bytes has 4 KB page size,
 * the starting index bit is the 6th bit and the last bit of page offset field
 * is 11th bit. The Pythia offset is the 6-bit value extracted from the 6th bit
 * to the 11th bit.
 */
int
PythiaPrefetcher::getStrideOffset(Addr addr) const
{
    int offset = bits<Addr>(addr, pageOffsetBits - 1, RubySystem::getBlockSizeBits());
    assert(offset == ((addr >> RubySystem::getBlockSizeBits()) & ((1ull << (pageOffsetBits - RubySystem::getBlockSizeBits())) - 1)));
    return offset;
}

/**
 * prefetch offset is the same thing as prefetch stride.
 */
int
PythiaPrefetcher::getStride(Addr addr, int action) const
{
    return (getStrideOffset(addr) + actionList[action]);
}

void
PythiaPrefetcher::issueNextPrefetch(Addr addr, Addr pc)
{
    if (m_pc_delta == 0 || m_delta_sig == 0)
        return;

    int action = chooseAction(getCurrStatePair());
    assert(action >= 0);

    int stride = 0;
    Addr pref_line_addr = invalidAddr;
    Addr page_aligned_addr = getPageAlignedAddr(addr);

    Reward reward_type = Reward::none;

    if (actionList[action] != 0) {
        stride = getStride(addr, action);
        if (stride >= 0 && stride < 64) { // if no page-crossing
            pref_line_addr = makeNextStrideAddress(page_aligned_addr, stride);
            
            assert(page_aligned_addr == getPageAlignedAddr(pref_line_addr));
            assert(pref_line_addr == page_aligned_addr + (stride << RubySystem::getBlockSizeBits()));

            stats.numPrefetchRequested++;
            stats.numUnrewardedEntries++;
            //DPRINTF(PythiaPrefetcher, "DEBUG (%llu): Requesting prefetch for addr 0x%6x\n", debug_cycle, pref_line_addr);
            m_controller->enqueuePrefetch(pref_line_addr, RubyRequestType_LD); // issue the next prefetch request
        } else {
            // out-of-page prefetch -> immediately assign a reward to the EQ entry
            stats.numPageCross++;
            reward_type = Reward::loss_of_coverage;
        }
    } else {
        // not to prefetch -> immediately assign a reward to the EQ entry
        stats.numNoPref++;
        reward_type = Reward::no_prefetch;
    }
    /**
     * Create a new EQ entry with the current state-vector, the selected
     * action, and its corresponding prefetched address (line 18)
     * Immediate reward assignment during EQ insertion.
     * Note that, a no-prefetch action or an action that prefetches an address
     * beyond the current physical page is also inserted into EQ. The reward
     * for such an action is instantaneously assigned to the EQ entry.
     * This means that the pref_line_addr of an EQ entry can be invalidAddr!
     */
    insertEvalQue(std::make_shared<EQEntry>(pref_line_addr, pc, getCurrStatePair(), action, reward_type));
    printEvalQue();
}

void
PythiaPrefetcher::insertEvalQue(const std::shared_ptr<EQEntry>& new_entry)
{
    assert(eval_que.size() <= evalQueCapacity);
    
    if (eval_que.size() == evalQueCapacity) {
        std::shared_ptr<EQEntry> dq_entry = eval_que.front();
        // The EQ dq_entry is then inserted, which evicts an dq_entry from EQ.
        eval_que.pop_front();
        stats.numEQEvictions++;
        // Reward assignment during EQ eviction
        if (dq_entry->getReward() == Reward::none) {
            /**
             * If the evicted EQ entry does not already have a reward assigned
             * (indicating that the corresponding prefetch address is not demanded by
             * the processor so far), assigns the reward R_IN.
             */
            stats.numUnrewardedEntries--;
            dq_entry->reward(Reward::inaccurate);
            stats.numInaccurate++;
        }

        Reward r = dq_entry->getReward();
        StatePair s1 = dq_entry->getStatePair();
        int a1 = dq_entry->getAction();

        StatePair s2 = eval_que.front()->getStatePair(); // EQ.head.state
        int a2 = eval_que.front()->getAction(); // EQ.head.action

        // Only the last evicted eq entry is used to update the QVStore.
        qVStore.updateQ(r, s1, a1, s2, a2);
#if 0
        if (!printEnable())
            return;
        else
            qVStore.printQVStore();
#endif
    }

    eval_que.push_back(new_entry);
    stats.numEQInsertions++;
    assert(eval_que.size() <= evalQueCapacity);
    assert(stats.numEQInsertions.value() >= stats.numPrefetchRequested.value());
}

void
PythiaPrefetcher::trainAndPredict(Addr demand_address, Addr pc,
    const RubyRequestType& type)
{
    stats.numDemands++;
    debug_cycle++;

    // Convert demand addr to cache aligned addr
    Addr line_addr = makeLineAddress(demand_address);

    assert(line_addr != invalidAddr);

    /**
     * Assumption: If there are multiple matching prefetch requests that are
     * associated with the same cache line addr, the oldest one will be rewarded.
     */
    auto it = std::find_if(eval_que.begin(), eval_que.end(), [line_addr](const auto& entry){
        return (entry->getPrefAddr() == line_addr && entry->getReward() == Reward::none);
    });

    if (it != eval_que.end()) {
        stats.numEvalQueHits++;
        /**
         * Pythia has issued a prefetch request for this address in the past,
         * it signifies that the prefetch action corresponding to the EQ entry
         * has generated a useful prefetch request.
         */

        assert((*it)->getReward() == Reward::none && (*it)->getPrefAddr() != invalidAddr);

        // Reward assignment during EQ residency,
        if ((*it)->isFilled()) {
            (*it)->reward(Reward::accurate_timely);
            stats.numAccurateAndTimely++;
        }
        else {
            (*it)->reward(Reward::accurate_late);
            stats.numAccurateButLate++;
        }
        stats.numUnrewardedEntries--;
    }

    updateEnvState(pc, getStrideOffset(demand_address), type);
    printDebugCycle(demand_address, pc);
    printStateVector(makeLineAddress(demand_address), pc);
    issueNextPrefetch(demand_address, pc);
}

void
PythiaPrefetcher::prefetch_fill(Addr addr)
{
    addr = makeLineAddress(addr);
    assert(addr != invalidAddr);
    /**
     * If an entry has been rewarded, its prefetch fill bit will never be
     * asserted because we don't want to record ineffective prefetch fill stats.
     */

    /**
     * Assumption: If there are multiple matching prefetch requests that are
     * associated with the same cache line addr, the fill bit of the oldest one
     * will be set.
     */
    auto it = std::find_if(eval_que.begin(), eval_que.end(), [addr](const auto& entry){
        return entry->getPrefAddr() == addr && entry->getReward() == Reward::none;
    });
    
    if (it != eval_que.end()) {
        if (!(*it)->isFilled()) {
            (*it)->setFill();
            stats.numPrefFill++;
        }
    }
}

void
PythiaPrefetcher::printStateVector(Addr line_addr, Addr pc) const
{
    if (!printEnable())
        return;

#if 0
    DPRINTF(PythiaPrefetcher, "Line Addr: 0x%6x,        PC: 0x%6x\n", line_addr, pc);
    DPRINTF(PythiaPrefetcher, "PC+Delta:  0x%6x,        Last %u deltas: 0x%6x\n", m_pc_delta, deltaQueCapacity, m_delta_sig);
    DPRINTF(PythiaPrefetcher, "numPrefetchRequested = %u\n", stats.numPrefetchRequested.value());
    DPRINTF(PythiaPrefetcher, "numAccurateAndTimely = %u\n", stats.numAccurateAndTimely.value());
    DPRINTF(PythiaPrefetcher, "numAccurateButLate   = %u\n", stats.numAccurateButLate.value());
    DPRINTF(PythiaPrefetcher, "numUnrewardedEntries = %u\n", stats.numUnrewardedEntries.value());
    DPRINTF(PythiaPrefetcher, "numInaccurate        = %u\n", stats.numInaccurate.value());
    DPRINTF(PythiaPrefetcher, "numEQEvictions       = %u\n", stats.numEQEvictions.value());
    //DPRINTF(PythiaPrefetcher, "numNoPref            = %u\n", stats.numNoPref.value());

    if (deltas.size() > 0) {
        DPRINTF(PythiaPrefetcher, "Sequence of deltas:\n");
        for (const auto& delta : deltas) {
            DPRINTF(PythiaPrefetcher, "| %d |\n", delta);
        }
    }
#endif
}

void
PythiaPrefetcher::printEvalQue() const
{
    if (!printEnable())
        return;

    if (eval_que.size() > 0) {
        DPRINTF(PythiaPrefetcher, "Num of Eval Que Evictions: %u\n", stats.numEQEvictions.value());
        DPRINTF(PythiaPrefetcher, "Eval Que: \n");
        for (const auto& entry : eval_que) {
            entry->printEQEntry();
            DPRINTF(PythiaPrefetcher, "\n");
        }
    }
}

void
QVStore::printQVStore() const
{
    for (int i = 0; i < m_num_vaults; ++i) {
        if (vaultVector[i].size() > 0) {
            DPRINTF(PythiaPrefetcher, "===================================================================\n");
#if 0
            for (const auto& pair : vaultVector[i]) {
                /**
                 * State[0] = pc+delta, State[1] = sequence of last 4 deltas
                 */
                DPRINTF(PythiaPrefetcher, "<State[%d], Action>: <0x%6x, %d>, Value: %f\n", i, pair.first.first, pair.first.second, pair.second);
            }
#endif
            DPRINTF(PythiaPrefetcher, "Num of Vault[%d] updates: %u\n", i, getNumUpdates(i));
            DPRINTF(PythiaPrefetcher, "===================================================================\n");
        }
    }
}

void
PythiaPrefetcher::printDebugCycle(Addr demand_address, Addr pc) const
{
    if (!printEnable())
        return;

    DPRINTF(PythiaPrefetcher, "================================    Pythia    =================================\n");
    DPRINTF(PythiaPrefetcher, "Debug Cycle: %llu\n", debug_cycle);

}

bool
PythiaPrefetcher::printEnable() const
{
    return false;
    //return (debug_cycle > 0 && debug_cycle < 150) ? true : false;
    //return true;
}

} // namespace ruby
} // namespace gem5