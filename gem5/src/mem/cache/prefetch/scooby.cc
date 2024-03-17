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

#include <cassert>
#include "mem/cache/prefetch/scooby.hh"
#include "debug/HWPrefetchScooby.hh"

namespace gem5
{

namespace prefetch
{

static const Addr INVALID_ADDR = 0xFFFFFFFFFFFFFFFF;
static const uint32_t MAX_DEBUG_CYCLE = 0;

std::string
RewardType_to_string(const RewardType& reward_type)
{
    switch(reward_type) {
        case RewardType::no_prefetch:
            return "No Prefetch";
        case RewardType::inaccurate:
            return "Inaccurate";
        case RewardType::loss_of_coverage:
            return "Loss of Coverage";
        case RewardType::accurate_late:
            return "Accurate Late";
        case RewardType::accurate_timely:
            return "Accurate Timely";
        case RewardType::none:
            return "None";
        default:
            panic("Invalid reward value!");
    }
    return "";
}

// ****************************************************************************
// Vault Member Function definitions
// ****************************************************************************
Vault::Vault(uint32_t num_tilings, uint32_t num_tiles, uint32_t num_actions,
    FeatureType feature_type, float alpha, float gamma,
    bool use_tiling_offset, float default_q)
    : m_num_tilings(num_tilings),
    m_num_tiles(num_tiles),
    m_num_actions(num_actions),
    m_feature_type(feature_type),
    m_alpha(alpha),
    m_gamma(gamma),
    m_use_tiling_offset(use_tiling_offset),
    m_qtable(num_tilings, std::vector<std::vector<float>>(num_tiles, std::vector<float>(num_actions, default_q))),
    tiling_offset({
        0xaca081b9, 0x666a1c67, 0xc11d6a53, 0x8e5d97c1, 0x0d1cad54, 0x874f71cb, 0x20d2fa13, 0x73f7c4a7,
        0x0b701f6c, 0x8388d86d, 0xf72ac9f2, 0xbab16d82, 0x524ac258, 0xb5900302, 0xb48ccc72, 0x632f05bf,
        0xe7111073, 0xeb602af4, 0xf3f29ebb, 0x2a6184f2, 0x461da5da, 0x6693471d, 0x62fd0138, 0xc484efb3,
        0x81c9eeeb, 0x860f3766, 0x334faf86, 0x5e81e881, 0x14bc2195, 0xf47671a8, 0x75414279, 0x357bc5e0
    })
{
    assert(m_num_tilings == 3);
    assert(m_num_tiles == 128);
}

uint32_t
Vault::getHash(uint32_t key) const
{
    // Robert Jenkins' 32 bit mix function
    key += (key << 12);
    key ^= (key >> 22);
    key += (key << 4);
    key ^= (key >> 9);
    key += (key << 10);
    key ^= (key >> 2);
    key += (key << 7);
    key ^= (key >> 12);
    return key;
}

uint32_t
Vault::folded_xor(uint64_t value, uint32_t num_folds) const
{
    assert(num_folds > 1);
    assert((num_folds & (num_folds-1)) == 0); /* has to be power of 2 */
    uint32_t mask = 0;
    uint32_t bits_in_fold = 64/num_folds;
    if(num_folds == 2)
    {
        mask = 0xffffffff;
    }
    else
    {
        mask = (1ul << bits_in_fold) - 1;
    }
    uint32_t folded_value = 0;
    for(uint32_t fold = 0; fold < num_folds; ++fold)
    {
        folded_value = folded_value ^ ((value >> (fold * bits_in_fold)) & mask);
    }
    return folded_value;
}

uint32_t
Vault::process_pc_delta(uint32_t tiling, uint64_t pc, int32_t delta) const
{
    uint32_t _delta = (delta < 0) ? (((-1) * delta) + (1 << (7 - 1))) : delta;
    Addr tmp = pc << 7;
    tmp += _delta;
    uint32_t raw_index = folded_xor(tmp, 2);
    if (m_use_tiling_offset) raw_index = raw_index ^ tiling_offset[tiling];
    uint32_t hashed_index = getHash(raw_index);
    return (hashed_index % m_num_tiles);
}

uint32_t
Vault::process_delta_path(uint32_t tiling, uint32_t delta_path) const
{
    if (m_use_tiling_offset) delta_path = delta_path ^ tiling_offset[tiling];
    uint32_t hashed_index = getHash(delta_path);
    return (hashed_index % m_num_tiles);
}

void
Vault::setQ(uint32_t tiling, uint32_t tile_index, uint32_t action, float value)
{
    assert(tiling < static_cast<int>(m_qtable.size()) &&
        tile_index < static_cast<int>(m_qtable[tiling].size()) &&
        action < static_cast<int>(m_qtable[tiling][tile_index].size()));

    assert(tiling < m_num_tilings);
    assert(tile_index < m_num_tiles);
    assert(action < m_num_actions);
    m_qtable[tiling][tile_index][action] = value;
}

float
Vault::getQ(uint32_t tiling, uint32_t tile_index, uint32_t action) const
{
    assert(tiling < static_cast<int>(m_qtable.size()) &&
        tile_index < static_cast<int>(m_qtable[tiling].size()) &&
        action < static_cast<int>(m_qtable[tiling][tile_index].size()));

    assert(tiling < m_num_tilings);
    assert(tile_index < m_num_tiles);
    assert(action < m_num_actions);

    return m_qtable[tiling][tile_index][action];
}

uint32_t
Vault::getTileIndex(const std::shared_ptr<State>& state, uint32_t tiling) const
{
    Addr pc = state->getPC();
    int32_t delta = state->getDelta();
    uint32_t delta_path = state->getDeltaPath();
    
    switch(m_feature_type) {
        case F_PC_Delta:        return process_pc_delta(tiling, pc, delta);
        case F_Delta_Path:      return process_delta_path(tiling, delta_path);
        default:                assert(false); return 0;
    }
}

float
Vault::retrieveQ(const std::shared_ptr<State>& state, uint32_t action) const
{
    uint32_t tile_index = 0;
    float q = 0.0f;

    for (uint32_t tiling = 0; tiling < m_num_tilings; ++tiling) {
        tile_index = getTileIndex(state, tiling);
        q += getQ(tiling, tile_index, action);
    }

    return q;
}

void
Vault::updateQ(int32_t r0, const std::shared_ptr<State>& s0, int32_t a0, const std::shared_ptr<State>& s1, int32_t a1)
{
    uint32_t tile_idx_0 = 0, tile_idx_1 = 0;
    float Qsa_0, Qsa_1;

    //float Qsa_0_old_overall = retrieveQ(s0, a0);
    //float Qsa_1_old_overall = retrieveQ(s1, a1);
#if 0
    DPRINTF(HWPrefetchScooby, "State 0 @ <0x%x, 0x%x>           State 1 @ <0x%x, 0x%x>:\n",
        s0->getPC(), s0->getDeltaPath(), s1->getPC(), s1->getDeltaPath());
#endif
    for(uint32_t tiling = 0; tiling < m_num_tilings; ++tiling)
    {
        tile_idx_0 = getTileIndex(s0, tiling);
        tile_idx_1 = getTileIndex(s1, tiling);
        Qsa_0 = getQ(tiling, tile_idx_0, a0);
        Qsa_1 = getQ(tiling, tile_idx_1, a1);
        float Qsa_0_new = Qsa_0 + m_alpha * (static_cast<float>(r0) + m_gamma * Qsa_1 - Qsa_0); /* SARSA */
        setQ(tiling, tile_idx_0, a0, Qsa_0_new);
        //DPRINTF(HWPrefetchScooby, "Tile Q @ <%u, %u, %d>: %f -> %f\n", tiling, tile_idx_0, a0, Qsa_0, Qsa_0);
    }

    //DPRINTF(HWPrefetchScooby, "Q0: %f -> %f\n", Qsa_0_old_overall, retrieveQ(s0, a0));
}

// ****************************************************************************
// QVStore Member Function definitions
// ****************************************************************************
QVStore::QVStore(float alpha, float gamma, float epsilon,
    uint32_t num_tilings, uint32_t num_tiles, uint32_t num_actions,
        bool use_tiling_offset)
    : m_alpha(alpha),
    m_gamma(gamma),
    explore(epsilon),
    m_num_tilings(num_tilings),
    m_num_tiles(num_tiles),
    m_num_actions(num_actions),
    m_use_tiling_offset(use_tiling_offset),
    m_default_q(1.0f / (1.0f - gamma)),
    vaults(NumFeatureTypes)
{
    generator.seed(3);
    actionGen = std::uniform_int_distribution<int32_t>(0, m_num_actions - 1);

    for (uint32_t i = 0; i < NumFeatureTypes; ++i) {
        vaults[i] = std::make_shared<Vault>(
            m_num_tilings,
            m_num_tiles,
            m_num_actions,
            static_cast<FeatureType>(i),
            m_alpha,
            m_gamma,
            m_use_tiling_offset,
            m_default_q
        );
    }
}

uint32_t
QVStore::chooseAction(const std::shared_ptr<State>& state)
{
    int32_t action = -1; // Note that -1 represents invalid value.

    if (explore(generator)) {
        action = actionGen(generator); // take random action
    } else {
        action = getMaxAction(state);
    }

    return action;
}

float
QVStore::consultQ(const std::shared_ptr<State>& state, uint32_t action) const
{
    assert(action < m_num_actions);
    float q_value = 0.0;
    float max = -1000000000.0f;

    /* pool Q-value accross all feature tables */
    for (uint32_t index = 0; index < NumFeatureTypes; ++index) {
        if (vaults[index]) {
            float tmp = vaults[index]->retrieveQ(state, action);
            if (tmp >= max) {
                max = tmp;
                q_value = tmp;
            }
        }
    }

    return q_value;
}

void
QVStore::train(int32_t r0, const std::shared_ptr<State>& s0, int32_t a0, const std::shared_ptr<State>& s1, int32_t a1)
{
    assert(r0 != static_cast<int32_t>(RewardValue::none));

    for (uint32_t i = 0; i < NumFeatureTypes; ++i) {
        vaults[i]->updateQ(r0, s0, a0, s1, a1);
    }
}

uint32_t
QVStore::getMaxAction(const std::shared_ptr<State>& state) const
{
    float max_q = 0.0f;
    float q = 0.0f;
    uint32_t selected_action = 0;

    for (uint32_t action = 0; action < m_num_actions; ++action) {
        q = consultQ(state, action);
        if (q > max_q) {
            max_q = q;
            selected_action = action;
        }
    }
    return selected_action;
}

// ****************************************************************************
// EqEntry Member Function definitions
// ****************************************************************************
void
EqEntry::print() const
{
    if (addr == INVALID_ADDR)
        DPRINTF(HWPrefetchScooby, "Addr         = %s\n", "N/A");
    else
        DPRINTF(HWPrefetchScooby, "Addr         = 0x%x\n", addr);

    DPRINTF(HWPrefetchScooby, "PC           = 0x%x\n", state->getPC());
    DPRINTF(HWPrefetchScooby, "Action       = %d\n", action);
    DPRINTF(HWPrefetchScooby, "RewardType       = %s\n", RewardType_to_string(rewardType));
    DPRINTF(HWPrefetchScooby, "PC           = 0x%x\n", state->getPC());
    DPRINTF(HWPrefetchScooby, "Delta        = 0x%x\n", state->getDelta());
    DPRINTF(HWPrefetchScooby, "Delta Path   = 0x%x\n", state->getDeltaPath());
    DPRINTF(HWPrefetchScooby, "Fill         = %d\n", fill);
}

bool
EqEntry::hasReward() const
{
    if (rewardType != RewardType::none)
        assert(rewardValue != static_cast<int32_t>(RewardValue::none));

    return rewardType != RewardType::none;
}
// ****************************************************************************
// SigTableEntry Member Function definitions
// ****************************************************************************
void
SigTableEntry::update(Addr _page, Addr pc, uint32_t offset) {
    assert(this->page == _page);

    /* insert PC */
    if (this->pcs.size() == 1) {
        this->pcs.pop_front();
    }
    this->pcs.push_back(pc);
    
    /* insert deltas */
    if (!this->offsets.empty()) {
        int32_t delta = (offset > this->offsets.back()) ? (offset - this->offsets.back()) : (-1)*(this->offsets.back() - offset);
        if (this->deltas.size() >= 5) {
            this->deltas.pop_front();
        }
        this->deltas.push_back(delta);
    }

    /* insert offset */
    if (this->offsets.size() == 5) {
        this->offsets.pop_front();
    }
    this->offsets.push_back(offset);
}

void
SigTableEntry::trackPrefetch(int32_t predicted_offset, int32_t action_offset)
{
    assert(predicted_offset >= 0 && predicted_offset < 64);
    assert(action_offset >= -6 && action_offset <= 32);

    if (bmp[predicted_offset])
        return;

    bmp[predicted_offset] = 1;
    insertAction(action_offset);
}

void
SigTableEntry::printActionTrackers() const
{
    DPRINTF(HWPrefetchScooby, "Action Trackers with page number %u:\n", page);
    for (const auto& tracker : actionTrackers) {
        DPRINTF(HWPrefetchScooby, "action_offset: %d        conf: %d\n", tracker->action_offset, tracker->conf);
    }
}

void
SigTableEntry::insertAction(int32_t action_offset)
{
    auto it = std::find_if(actionTrackers.begin(), actionTrackers.end(), [action_offset](const auto& entry){
        return entry->action_offset == action_offset;
    });

    if (it != actionTrackers.end()) {
        (*it)->conf++;
        /* Save a copy of the shared_ptr before erasing the iterator */
        auto tmp = (*it);
        /* Maintain the recency order */
        actionTrackers.erase(it);
        actionTrackers.push_back(tmp);
    } else {
        if (actionTrackers.size() == numActionTrackers) {
            std::shared_ptr<ActionTracker> victim = actionTrackers.front();
            actionTrackers.pop_front();
        }
        auto action_tracker = std::make_shared<ActionTracker>(action_offset, 0);
        actionTrackers.push_back(action_tracker);
        assert(actionTrackers.size() <= numActionTrackers);
    }
}

int32_t
SigTableEntry::getLatestDelta() const
{
    if (deltas.empty())
        return 0;
    else
        return deltas.back();
}

uint32_t
SigTableEntry::computeDeltaPath() const // same as `get_delta_sig2()`
{
    uint32_t sig = 0; // delta signature

    /* Compute signature only using last 4 deltas */
    uint32_t ptr = (deltas.size() >= 4) ? (deltas.size() - 4) : 0;
    for (uint32_t i = ptr; i < deltas.size(); ++i) {
        int32_t temp = (deltas[i] < 0) ? (((-1) * deltas[i]) + (1 << (7 - 1))) : deltas[i];
        sig = static_cast<uint32_t>(((sig << 3) ^ temp) & ((1 << 12) - 1));
    }

    return sig;
}

void
SigTableEntry::printDeltas() const
{
    DPRINTF(HWPrefetchScooby, "Queue of Deltas:\n");
    for (const auto& delta : deltas) {
        DPRINTF(HWPrefetchScooby, "%d   ", delta);
    }
    DPRINTF(HWPrefetchScooby, "\n--------------------------------\n");
}

void
SigTableEntry::printOffsets() const
{
    DPRINTF(HWPrefetchScooby, "Queue of Offsets:\n");
    for (const auto& offset : offsets) {
        DPRINTF(HWPrefetchScooby, "%u   ", offset);
    }
    DPRINTF(HWPrefetchScooby, "\n--------------------------------\n");
}

// ****************************************************************************
// Scooby Member Function definitions
// ****************************************************************************
Scooby::
Stats::Stats(statistics::Group *parent)
    : statistics::Group(parent, "Scooby"),
      ADD_STAT(demands, statistics::units::Count::get(),
        "Number of demands"),
      ADD_STAT(pfIssued, statistics::units::Count::get(),
        "Number of pf issued (excluding multi-degree pf)"),
      ADD_STAT(multiDeg, statistics::units::Count::get(),
        "Number of pf issued by multi degree pf"),
      ADD_STAT(accurateAndTimely, statistics::units::Count::get(),
        "Number of accurate and timely rewards"),
      ADD_STAT(accurateButLate, statistics::units::Count::get(),
        "Number of accurate but late rewards"),
      ADD_STAT(inaccurate, statistics::units::Count::get(),
        "Number of inaccurate rewards"),
      ADD_STAT(pageCross, statistics::units::Count::get(),
        "Number of prefetches that across pages"),
      ADD_STAT(evalQueHits, statistics::units::Count::get(),
        "Number of hits in evaluation queue"),
      ADD_STAT(evalQueMisses, statistics::units::Count::get(),
        "Number of misses in evaluation queue"),
      ADD_STAT(noPref, statistics::units::Count::get(),
        "Number of no prefetch rewards"),
      ADD_STAT(prefFill, statistics::units::Count::get(),
        "Number of effective prefetch fills"),
      ADD_STAT(reads, statistics::units::Count::get(),
        "Number of read requests"),
      ADD_STAT(writes, statistics::units::Count::get(),
        "Number of write requests")
{
    using namespace statistics;
    reads.flags(nozero);
    writes.flags(nozero);
}

void
Scooby::startup()
{
    schedule(epochEvent, clockEdge(epochCycles));
}

void
Scooby::processEpochEvent()
{
    schedule(epochEvent, clockEdge(epochCycles));

    float num_requests = epochStats.cacheMisses + epochStats.totalPfs;

    m_bandwidth = num_requests * offChipMemoryLatency / epochCycles;

    if (m_bandwidth > maxBandwidth) {
        maxBandwidth = m_bandwidth;
        warn("Max Bandwidth = %f", maxBandwidth);
    }

    if (m_bandwidth != 0) DPRINTF(HWPrefetchScooby, "bandwidth = %f\n", m_bandwidth);

    epochStats.reset();
}

bool
Scooby::isHighBW() const
{
    return m_bandwidth >= m_high_bw_threshold ? true : false;
}

Scooby::Scooby(const ScoobyPrefetcherParams& p)
    : Queued(p),
    m_secure_mode(false),
    m_enable_dyn_degree(p.enableDynDegree),
    evalQueCapacity(p.eval_que_capacity),
    sigTableCapacity(64),
    invalidAddr(INVALID_ADDR),
    pageOffsetBits(floorLog2(pageBytes)), // pageByte is the page size in bytes (e.g., 4096 bytes)
    actionList({-6, -3, -1, 0, 1, 3, 4, 5, 10, 11, 12, 16, 22, 23, 30, 32}),
    m_conf_thresholds({1, 3, 8}),
    m_deg_normal({1, 2, 4, 6}),
    enable_reward_all(false),
    last_evicted_eq_entry(nullptr),
    epochEvent([this]{ processEpochEvent(); }, name()),
    epochStats(),
    offChipMemoryLatency(p.offchip_memory_latency),
    epochCycles(p.epoch_cycles),
    m_bandwidth(0),
    maxBandwidth(0),
    m_high_bw_threshold(114),
    debug_cycle(0),
    stats(this)
{
    float alpha = 0.0065;
    float gamma = 0.556;
    float epsilon = 0.002;

    assert(evalQueCapacity == 256);
    assert(pageOffsetBits == 12);
    assert(actionList.size() == 16);
    assert(lBlkSize == 6); // number bits for byte offset
    assert(p.useTilingOffset);

    qVStore = std::make_unique<QVStore>(
        alpha,
        gamma,
        epsilon,
        p.numTilings,
        p.numTiles,
        static_cast<uint32_t>(actionList.size()),
        p.useTilingOffset
    );
}

int32_t
Scooby::computeReward(RewardType reward_type) const
{
    bool high_bw = isHighBW();
    int32_t reward = 0;

    switch (reward_type) {
        case RewardType::accurate_timely:
            reward = static_cast<int32_t>(RewardValue::R_at);
            break;
        case RewardType::accurate_late:
            reward = static_cast<int32_t>(RewardValue::R_al);
            break;
        case RewardType::loss_of_coverage:
            reward = static_cast<int32_t>(RewardValue::R_cl);
            break;
        case RewardType::inaccurate:
            reward = high_bw ? static_cast<int32_t>(RewardValue::R_in_h)
                : static_cast<int32_t>(RewardValue::R_in_l);
            break;
        case RewardType::no_prefetch:
            reward = high_bw ? static_cast<int32_t>(RewardValue::R_np_h)
                : static_cast<int32_t>(RewardValue::R_np_l);
            break;
        default:
            assert(false);
    }

    return reward;
}

void
Scooby::assignReward(const std::shared_ptr<EqEntry>& entry, RewardType reward_type)
{
    entry->setRewardType(reward_type);

    int32_t reward = computeReward(reward_type);
    entry->setRewardValue(reward);

    if (reward_type == RewardType::accurate_timely)
        epochStats.usefulPfs++;

    switch (reward_type) {
        case RewardType::no_prefetch:       stats.noPref++; break;
        case RewardType::inaccurate:        stats.inaccurate++; break;
        case RewardType::loss_of_coverage:  stats.pageCross++; break;
        case RewardType::accurate_late:     stats.accurateButLate++; break;
        case RewardType::accurate_timely:   stats.accurateAndTimely++; break;
        default:                            break;
    }
}

Addr
Scooby::getPageNum(Addr addr) const
{
    assert(bits<Addr>(addr, 63, pageOffsetBits) == addr >> pageOffsetBits);
    return bits<Addr>(addr, 63, pageOffsetBits);
}

std::vector<std::shared_ptr<EqEntry>>
Scooby::searchEq(Addr addr, bool search_all) const
{
    std::vector<std::shared_ptr<EqEntry>> entries;
    for (int32_t i = 0; i < eval_que.size(); ++i) {
        if (eval_que[i]->getPrefAddr() == addr) {
            entries.push_back(eval_que[i]);
            if (!search_all) break;
        }
    }
    return entries;
}

std::shared_ptr<SigTableEntry>
Scooby::updateEnvState(Addr pc, Addr page_num, int32_t scooby_offset)
{
    stats.demands++;
    std::shared_ptr<SigTableEntry> entry = nullptr;
    auto it = find_if(sig_table.begin(), sig_table.end(), [page_num](const auto& sig_entry)
        {
            return sig_entry->getPageNum() == page_num;
        }
    );

    if (it != sig_table.end()) {
        // sig_table_hit++;
        entry = (*it);
        entry->update(page_num, pc, scooby_offset);
        sig_table.erase(it);
        sig_table.push_back(entry);
        return entry;
    } else {
        if (sig_table.size() == sigTableCapacity) {
            // sig_table_evict++;
            sig_table.pop_front();
        }
        // sig_table_insert++;
        auto entry = std::make_shared<SigTableEntry>(page_num, pc, scooby_offset);
        sig_table.push_back(entry);
        return entry;
    }
}

/**
 * Scooby offset is defined as the binary value extracted from the starting
 * index bit of the address to the last bit of page offset field.
 * For computer system whose cache block size is 64 bytes has 4 KB page size,
 * the starting index bit is the 6th bit and the last bit of page offset field
 * is 11th bit. The Scooby offset is the 6-bit value extracted from the 6th bit
 * to the 11th bit.
 */
uint32_t
Scooby::getScoobyOffset(Addr addr) const
{
    uint32_t offset = bits<Addr>(addr, pageOffsetBits - 1, lBlkSize);
    assert(offset == ((addr >> lBlkSize) & ((1ull << (pageOffsetBits - lBlkSize)) - 1)));
    return offset;
}

int32_t
Scooby::getPredictedOffset(Addr pkt_addr, int32_t action_offset) const
{
    return (getScoobyOffset(pkt_addr) + action_offset);
}

// returns the next stride address based on line address
Addr
Scooby::computePrefAddr(Addr addr, int32_t offset) const
{
    assert(pageAddress(addr) == ((addr >> 12) << 12));
    // blkSize represents the number of bytes in a cache block.
    return (pageAddress(addr) + static_cast<Addr>(offset << lBlkSize));
}

void
Scooby::reward(const std::shared_ptr<EqEntry>& entry)
{
    if (entry->hasReward())
        return;

    if (entry->getPrefAddr() == invalidAddr) {
        assignReward(entry, RewardType::no_prefetch);
    } else {
        assignReward(entry, RewardType::inaccurate);
    }
}

bool
Scooby::track(Addr addr, const std::shared_ptr<State> state, uint32_t action, std::shared_ptr<EqEntry>& eq_entry)
{    
    std::vector<std::shared_ptr<EqEntry>> eq_entries_hit = searchEq(addr, false);
    bool new_addr = eq_entries_hit.empty() ? true : false;
    if (!new_addr && addr != invalidAddr) {
        stats.evalQueHits++;
        eq_entry = nullptr;
        return new_addr;
    }

    /* new prefetched address that hasn't been seen before */
    stats.evalQueMisses++;
    if (eval_que.size() == evalQueCapacity) {
        std::shared_ptr<EqEntry> dq_entry = eval_que.front();
        // The EQ dq_entry is then inserted, which evicts an dq_entry from EQ.
        eval_que.pop_front();
        if (last_evicted_eq_entry) {
            assert(last_evicted_eq_entry->hasReward());
            int32_t r0 = last_evicted_eq_entry->getRewardValue();
            const std::shared_ptr<State> s0 = last_evicted_eq_entry->getState();
            int32_t a0 = last_evicted_eq_entry->getAction();

            const std::shared_ptr<State> s1 = dq_entry->getState();
            int32_t a1 = dq_entry->getAction();

            // Only the last evicted eq entry is used to update the QVStore.
            qVStore->train(r0, s0, a0, s1, a1);
        }
        last_evicted_eq_entry = dq_entry; // TODO: Swap this line with the next line affect results?
        reward(last_evicted_eq_entry);
    }
    auto new_entry = std::make_shared<EqEntry>(addr, state, action);
    eval_que.push_back(new_entry);

    eq_entry = new_entry;

    return new_addr;
}

void
Scooby::genMultiDegreePref(Addr page, uint32_t scooby_offset, int32_t action_offset, uint32_t pref_degree, std::vector<AddrPriority>& addresses)
{
    Addr line_addr = invalidAddr;
    int32_t predicted_offset = 0;
    if (action_offset != 0) {
        // One pref addr was issued in the caller. So, exclude that one.
        for (int32_t degree = 2; degree <= pref_degree; ++degree) {
            // It's possible that the predicted offset is negative.
            predicted_offset = static_cast<int32_t>(scooby_offset) + degree * action_offset;
            if (predicted_offset >= 0 && predicted_offset < 64) {
                line_addr = (page << pageOffsetBits) + (predicted_offset << lBlkSize);
                addresses.push_back(AddrPriority(line_addr, 0));
                stats.multiDeg++;
                epochStats.totalPfs++;
            }
        }
    }
}

void
Scooby::predict(Addr pkt_addr, const std::shared_ptr<State> state, std::vector<AddrPriority>& addresses)
{
    Addr page = state->getPage();
    uint32_t pref_degree = 1;

    uint32_t action = qVStore->chooseAction(state);

    int32_t action_offset = actionList[action];

    if (m_enable_dyn_degree)
        pref_degree = getDynPrefDegree(page, action_offset);

    int32_t predicted_offset = 0;
    Addr pref_line_addr = invalidAddr;
    std::shared_ptr<EqEntry> eq_entry = nullptr;

    /**
     * When the EQ entry has been rewarded prior to being filled, it is
     * guaranteed that pref_line_addr is assigned an invalid pkt_addr value.
     */
    if (action_offset != 0) {
        predicted_offset = getPredictedOffset(pkt_addr, action_offset);
        if (predicted_offset >= 0 && predicted_offset < 64) { // if no page-crossing
            pref_line_addr = computePrefAddr(pkt_addr, predicted_offset);
            
            assert(samePage(pref_line_addr, pkt_addr));

            bool new_addr = track(pref_line_addr, state, action, eq_entry);
            if (new_addr) {
                addresses.push_back(AddrPriority(pref_line_addr, 0));
                stats.pfIssued++;
                epochStats.totalPfs++;
                trackAction(page, predicted_offset, action_offset);
                if (pref_degree > 1)
                    genMultiDegreePref(page, state->getOffset(), action_offset, pref_degree, addresses);
            }
            //DPRINTF(HWPrefetchScooby, "Predicted Offset: %d\n", predicted_offset);
        } else {
            /**
             * The cache line size must be exactly 64 bytes for this prefetcher to work!
             * A no-prefetch action is taken when an action that prefetches beyond the
             * current physical page.
             */
            assert(pref_line_addr == invalidAddr && eq_entry == nullptr);
            track(pref_line_addr, state, action, eq_entry);
            assert(eq_entry);
            // out-of-page prefetch -> immediately assign a reward value to the EQ entry
            assignReward(eq_entry, RewardType::loss_of_coverage);
        }
    } else {
        assert(pref_line_addr == invalidAddr);
        track(pref_line_addr, state, action, eq_entry);
    }
}

void
Scooby::trackAction(Addr page, int32_t predicted_offset, int32_t action_offset)
{
    auto it = find_if(sig_table.begin(), sig_table.end(), [page](const auto& entry) {
        return entry->getPageNum() == page;
    });

    if (it != sig_table.end()) {
        (*it)->trackPrefetch(predicted_offset, action_offset);
    }
}

bool
SigTableEntry::searchActionIndex(int32_t action_offset, int32_t& conf)
{
    conf = 0;
    auto it = std::find_if(actionTrackers.begin(), actionTrackers.end(), [action_offset](const auto& entry){
        return entry->action_offset == action_offset;
    });
    if (it != actionTrackers.end()) {
        conf = (*it)->conf;
        return true;
    } else {
        return false;
    }
}

Addr
Scooby::getDynPrefDegree(Addr page, int32_t action_offset) const
{
    bool counted = false;
    uint32_t degree = 1;

    auto it = std::find_if(sig_table.begin(), sig_table.end(), [page](const auto& entry) {
        return entry->getPageNum() == page;
    });

    if (it != sig_table.end()) {
        int32_t conf = -1;
        bool found = (*it)->searchActionIndex(action_offset, conf);
        assert(conf >= 0);
        if (found) {
            for (uint32_t i = 0; i < m_conf_thresholds.size(); ++i) {
                if (conf <= m_conf_thresholds[i]) {
                    degree = m_deg_normal[i];
                    counted = true;
                    break;
                }
            }

            if (!counted)
                degree = m_deg_normal.back();

        } else {
            degree = 1;
        }
    }

    return degree;
}

void
Scooby::calculatePrefetch(const PrefetchInfo &pfi,
                          std::vector<AddrPriority> &addresses,
                          const CacheAccessor &cache)
{
    if (!pfi.hasPC())
        return;

    registerFill(cache); // Should be called on each cache event.

    Addr pkt_addr = pfi.getAddr();
    Addr pc = pfi.getPC();
    Addr page_num = getPageNum(pkt_addr);

    if (pfi.isWrite())
        stats.writes++;
    else
        stats.reads++;

    assert(pfi.isSecure() == m_secure_mode);

    rewardEqEntry(pkt_addr);

    if (!pfi.isCacheMiss())
        return;

    epochStats.cacheMisses++;
    assert((pkt_addr >> pageOffsetBits) == page_num);

    /* per page state tracking and updating */
    std::shared_ptr<SigTableEntry> entry = updateEnvState(pc, page_num, getScoobyOffset(pkt_addr));

    const std::shared_ptr<State> state = std::make_shared<State>(
        pc,
        blockAddress(pkt_addr),
        page_num,
        getScoobyOffset(pkt_addr),
        entry->getLatestDelta(),
        entry->computeDeltaPath()
    );

    predict(pkt_addr, state, addresses);
    debug_cycle++;
    printDebugInfo(page_num, pkt_addr, blockAddress(pkt_addr), pc,
        getScoobyOffset(pkt_addr), entry);
    
}

/**
 * Assumption: If there are multiple matching prefetch requests that are
 * associated with the same cache line addr, the oldest one will be rewarded.
 */
void
Scooby::rewardEqEntry(Addr pkt_addr)
{
    // Convert demand addr to cache line addr
    Addr line_addr = blockAddress(pkt_addr);

    // verify the correctness of blockAddress
    assert(line_addr == mbits<Addr>(pkt_addr, 63, lBlkSize));

    std::vector<std::shared_ptr<EqEntry>> hit_entries = searchEq(line_addr, enable_reward_all);

    /**
     * Scooby has issued a prefetch request for this address in the past.
     * This means that the prefetch action in the EQ entry has generated a
     * useful prefetch request.
     */
    for (int32_t i = 0; i < hit_entries.size(); ++i) {
        std::shared_ptr<EqEntry> entry = hit_entries[i];
        if (entry->hasReward()) {
            return;
        }
        // RewardType assignment during EQ residency,
        if (entry->isFilled()) {
            assignReward(entry, RewardType::accurate_timely);
        } else {
            assignReward(entry, RewardType::accurate_late);
        }
    }
}

/**
 * If an entry has been rewarded, its prefetch fill bit will never be
 * asserted because we don't want to record ineffective prefetch fill stats.
 * 
 * Assumption: If there are multiple matching prefetch requests that are
 * associated with the same cache line addr, the fill bit of the oldest one
 * will be set.
 */
void
Scooby::registerFill(const CacheAccessor &cache)
{
    for (const auto& entry : eval_que) {
        if (entry->getRewardType() == RewardType::none && !entry->isFilled()) {
            Addr pref_addr = entry->getPrefAddr();
            if (cache.hasBeenPrefetched(pref_addr, m_secure_mode)) {
                entry->setFill();
                stats.prefFill++;
                break; // This increases accuracy and coverage a little bit.
            }
        }
    }
}

void
Scooby::printEvalQue() const
{
    if (eval_que.size() > 0) {
        DPRINTF(HWPrefetchScooby, "Eval Que: \n");
        for (const auto& entry : eval_que) {
            entry->print();
            DPRINTF(HWPrefetchScooby, "\n");
        }
    }
}

void
Scooby::printDebugInfo(Addr page_num, Addr pkt_addr, Addr line_addr, Addr pc,
    uint32_t offset, const std::shared_ptr<SigTableEntry>& entry) const
{
    if (debug_cycle > MAX_DEBUG_CYCLE)
        return;

    DPRINTF(HWPrefetchScooby, "================================    Debug Cycle: %llu    =================================\n", debug_cycle);
    DPRINTF(HWPrefetchScooby, "pkt addr: 0x%x          line addr: 0x%x\n", pkt_addr, line_addr);
    DPRINTF(HWPrefetchScooby, "PC:       0x%x   Page Num:  %u\n", pc, page_num);
    DPRINTF(HWPrefetchScooby, "Offset:   %d                Delta:     %u\n", offset, entry->getLatestDelta());

    if (last_evicted_eq_entry) {
        DPRINTF(HWPrefetchScooby, "Last Evicted EQ Entry: \n");
        last_evicted_eq_entry->print();
    }

    printEvalQue();
    entry->printOffsets();
    entry->printDeltas();
    if (m_enable_dyn_degree)
        printActTackersInSigTable();
}

void
Scooby::printActTackersInSigTable() const
{
    DPRINTF(HWPrefetchScooby, "Sig Table: \n");
    for (const auto& entry : sig_table) {
        entry->printActionTrackers();
        DPRINTF(HWPrefetchScooby, "----------------------\n");
    }
}

} // namespace prefetch
} // namespace gem5