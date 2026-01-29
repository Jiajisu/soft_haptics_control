#include "UserStudyManager.hpp"
#include <algorithm>
#include <random>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <map>

namespace {
int polarityToInt(SurfacePolarity p) {
    return (p == SurfacePolarity::POSITIVE) ? 1 : -1;
}

SurfacePolarity polarityFromInt(int v) {
    return (v >= 0) ? SurfacePolarity::POSITIVE : SurfacePolarity::NEGATIVE;
}

std::string polarityToString(SurfacePolarity p) {
    return (p == SurfacePolarity::POSITIVE) ? "positive" : "negative";
}

const std::array<Exp2Plane, 3> kExp2AxisPlanes = {
    Exp2Plane::PLANE_X,
    Exp2Plane::PLANE_Y,
    Exp2Plane::PLANE_Z
};
// Axis-plane angles (B)
const std::vector<double> kExp2AnglesDeg = { 0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 270.0, 300.0, 330.0 };
constexpr int kExp2Repetitions = 5;

// Combined-angle pairs [A,B] where A = plane rotation about Z, B = in-plane angle
using Exp2AnglePair = std::array<double, 2>;
const std::vector<Exp2AnglePair> kExp2CombinedAnglesDeg = {
    {30.0, 30.0}, {30.0, 60.0}, {30.0, 120.0}, {30.0, 150.0},
    {60.0, 30.0}, {60.0, 60.0}, {60.0, 120.0}, {60.0, 150.0},
    {120.0, 30.0}, {120.0, 60.0}, {120.0, 120.0}, {120.0, 150.0},
    {150.0, 30.0}, {150.0, 60.0}, {150.0, 120.0}, {150.0, 150.0},
};
constexpr int kExp2CombinedRepetitions = 5;

inline size_t getExp2TrialsPerAxisGroup() {
    return kExp2AnglesDeg.size() * kExp2Repetitions;
}

inline size_t getExp2CombinedGroupSize() {
    return kExp2CombinedAnglesDeg.size() * kExp2CombinedRepetitions;
}

inline size_t getExp2ExpectedTrialCountInternal() {
    return getExp2TrialsPerAxisGroup() * kExp2AxisPlanes.size() + getExp2CombinedGroupSize();
}

std::vector<int> getDefaultExp2GroupSizes() {
    std::vector<int> sizes;
    const int perAxisGroup = static_cast<int>(getExp2TrialsPerAxisGroup());
    for (size_t i = 0; i < kExp2AxisPlanes.size(); ++i) {
        sizes.push_back(perAxisGroup);
    }
    const int combinedSize = static_cast<int>(getExp2CombinedGroupSize());
    if (combinedSize > 0) {
        sizes.push_back(combinedSize);
    }
    return sizes;
}
}

UserStudyManager::UserStudyManager()
    : m_trialActive(false)
    , m_currentMode(InteractionMode::FORCE_TO_POSITION)
    , m_currentDirection(FeedbackDirection::LATERAL_X)
    , m_currentTrialGroup(0)
    , m_currentTrialInGroup(0)
    , m_referenceStiffness(100.0)
    , m_sequenceLoaded(false)
    , m_experimentState(ExperimentState::NOT_STARTED)
    , m_csvFileReady(false)
    , m_currentPolarity(SurfacePolarity::POSITIVE)
    , m_exp2State(ExperimentState::NOT_STARTED)
    , m_exp2SequenceLoaded(false)
    , m_exp2CurrentTrialIndex(0)
    , m_exp2CurrentGroup(0)
    , m_exp2TrialActive(false)
    , m_exp2NeedBreak(false)
{
    generateComparisonStiffnesses();
    m_exp2GroupSizes = getDefaultExp2GroupSizes();
}

UserStudyManager::~UserStudyManager()
{
    if (!m_results.empty()) {
        saveResults("user_" + m_userId + "_results.csv");
        saveProgress(); // ??????
    }
}

void UserStudyManager::generateComparisonStiffnesses()
{
    m_comparisonStiffnesses.clear();
    const std::vector<double> ratios = { 0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.4,1.5 };
    for (double r : ratios)
        m_comparisonStiffnesses.push_back(m_referenceStiffness * r);
}

// ??:?????????
// ?? loadOrCreateUserSession ??
// ? UserStudyManager.cpp ?,?? loadOrCreateUserSession ??:

bool UserStudyManager::loadOrCreateUserSession(const std::string& userId)
{
    m_userId = userId;

    std::cout << "[UserStudy] Setting user ID: " << userId << std::endl;

    bool seq1Loaded = loadSequenceFromFile(getSequenceFilename());
    if (!seq1Loaded) {
        std::cout << "[UserStudy] No Experiment 1 sequence found. Generating new." << std::endl;
        generateRandomSequence();
        saveSequenceToFile(getSequenceFilename());
    }
    bool seq2Loaded = loadExp2Sequence(getExp2SequenceFilename());
    if (!seq2Loaded) {
        std::cout << "[UserStudy] No Experiment 2 sequence found. Generating new." << std::endl;
        generateExp2Sequence();
        saveExp2Sequence(getExp2SequenceFilename());
    }
    ensureExp2SequenceGrouped();
    if (m_exp2Sequence.size() != getExp2ExpectedTrialCountInternal()) {
        std::cout << "[UserStudy][Exp2] Sequence size mismatch detected. Regenerating for 4 groups." << std::endl;
        generateExp2Sequence();
        saveExp2Sequence(getExp2SequenceFilename());
        m_exp2CurrentTrialIndex = 0;
        m_exp2CurrentGroup = 0;
        m_exp2State = ExperimentState::READY;
        m_exp2GroupProgress.clear();
    }
    ensureExp1GroupProgressSize();
    ensureExp2GroupProgressSize();

    if (loadProgress(getProgressFilename())) {
        std::cout << "[UserStudy] Progress loaded: Exp1 Group "
            << (m_currentTrialGroup + 1) << "/6 Trial "
            << (m_currentTrialInGroup + 1) << "/110" << std::endl;
    }
    else {
        std::cout << "[UserStudy] No progress file found. Starting fresh." << std::endl;
        m_currentTrialGroup = 0;
        m_currentTrialInGroup = 0;
        m_experimentState = ExperimentState::READY;
        m_exp2CurrentTrialIndex = 0;
        m_exp2CurrentGroup = 0;
        m_exp2State = ExperimentState::READY;
        saveProgress();
    }

    if (m_currentTrialGroup >= 6) {
        m_experimentState = ExperimentState::EXPERIMENT_COMPLETE;
    }
    else if (m_currentTrialInGroup >= 110) {
        m_experimentState = ExperimentState::GROUP_COMPLETE;
    }
    else if (m_experimentState != ExperimentState::IN_PROGRESS) {
        m_experimentState = ExperimentState::READY;
    }

    if (m_exp2CurrentTrialIndex >= static_cast<int>(m_exp2Sequence.size())) {
        m_exp2State = ExperimentState::EXPERIMENT_COMPLETE;
    }
    else if (m_exp2State != ExperimentState::IN_PROGRESS) {
        m_exp2State = ExperimentState::READY;
    }
    ensureExp2GroupProgressSize();
    int nextGroup = findNextExp2GroupWithRemaining(m_exp2CurrentGroup);
    if (nextGroup >= 0) {
        m_exp2CurrentGroup = nextGroup;
    }

    std::cout << "[UserStudy] Initializing CSV files..." << std::endl;
    createOrLoadUserCSV();
    createOrLoadUserCSVExp2();

    return true;
}


// ????????
void UserStudyManager::generateRandomSequence()
{
    m_experimentSequence.clear();

    // ??6????? (2 modes � 3 directions)
    std::vector<std::pair<InteractionMode, FeedbackDirection>> groupConfigs = {
        {InteractionMode::FORCE_TO_POSITION, FeedbackDirection::LATERAL_X},
        {InteractionMode::FORCE_TO_POSITION, FeedbackDirection::LATERAL_Y},
        {InteractionMode::FORCE_TO_POSITION, FeedbackDirection::VERTICAL_Z},
        {InteractionMode::FORCE_TO_FORCE, FeedbackDirection::LATERAL_X},
        {InteractionMode::FORCE_TO_FORCE, FeedbackDirection::LATERAL_Y},
        {InteractionMode::FORCE_TO_FORCE, FeedbackDirection::VERTICAL_Z}
    };

    // ???????
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(groupConfigs.begin(), groupConfigs.end(), g);

    // ????????????
    for (const auto& config : groupConfigs) {
        GroupConfig group;
        group.mode = config.first;
        group.direction = config.second;
        group.trials = generateRandomizedTrials();
        m_experimentSequence.push_back(group);
    }

    m_sequenceLoaded = true;
}

// ?????????????
std::vector<TrialConfig> UserStudyManager::generateRandomizedTrials()
{
    std::vector<TrialConfig> trials;

    for (int stiffnessIdx = 0; stiffnessIdx < 11; ++stiffnessIdx) {
        for (int rep = 0; rep < 10; ++rep) {
            SurfacePolarity pol = (rep < 5) ? SurfacePolarity::POSITIVE : SurfacePolarity::NEGATIVE;
            trials.push_back({ stiffnessIdx, rep, pol });
        }
    }

    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(trials.begin(), trials.end(), g);

    return trials;
}

void UserStudyManager::generateExp2Sequence()
{
    m_exp2Sequence.clear();
    m_exp2GroupSizes.clear();

    std::vector<std::vector<AngleTrialConfig>> buckets;
    // Axis-aligned planes
    for (size_t planeIdx = 0; planeIdx < kExp2AxisPlanes.size(); ++planeIdx) {
        buckets.emplace_back();
        auto& bucket = buckets.back();
        for (int rep = 0; rep < kExp2Repetitions; ++rep) {
            for (double ang : kExp2AnglesDeg) {
                bucket.push_back({ kExp2AxisPlanes[planeIdx], 0.0, ang });
            }
        }
    }

    // Combined 3D angles (fourth group)
    if (!kExp2CombinedAnglesDeg.empty()) {
        buckets.emplace_back();
        auto& comboBucket = buckets.back();
        for (int rep = 0; rep < kExp2CombinedRepetitions; ++rep) {
            for (const auto& pair : kExp2CombinedAnglesDeg) {
                comboBucket.push_back({ Exp2Plane::PLANE_COMBINED, pair[0], pair[1] });
            }
        }
    }

    std::random_device rd;
    std::mt19937 g(rd());
    for (auto& bucket : buckets) {
        std::shuffle(bucket.begin(), bucket.end(), g);
        m_exp2GroupSizes.push_back(static_cast<int>(bucket.size()));
        m_exp2Sequence.insert(m_exp2Sequence.end(), bucket.begin(), bucket.end());
    }

    m_exp2SequenceLoaded = true;
    rebuildExp2GroupSizesFromSequence();
    ensureExp2GroupProgressSize();
}

// ???????
void UserStudyManager::saveSequenceToFile(const std::string& filename)
{
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "[UserStudy] Cannot create sequence file: " << filename << std::endl;
        return;
    }

    // ?????
    file << "# Experiment Sequence for User: " << m_userId << "\n";
    file << "# Format: GroupIndex Mode Direction\n";

    for (size_t i = 0; i < m_experimentSequence.size(); ++i) {
        const auto& group = m_experimentSequence[i];
        file << i << " " << (int)group.mode << " " << (int)group.direction << "\n";
    }

    file << "\n# Trial Sequence (StiffnessIndex Repetition Polarity)\n";

    // ??????????
    for (size_t groupIdx = 0; groupIdx < m_experimentSequence.size(); ++groupIdx) {
        file << "# Group " << groupIdx << " (110 trials)\n";
        const auto& trials = m_experimentSequence[groupIdx].trials;

        // ?????110???
        for (size_t i = 0; i < trials.size(); ++i) {
            file << trials[i].stiffnessIndex << " "
                << trials[i].repetition << " "
                << polarityToInt(trials[i].polarity);

            // ??10???,????
            if ((i + 1) % 10 == 0) {
                file << "\n";
            }
            else if (i < trials.size() - 1) {
                file << " ";
            }
        }

        // ????????
        if (groupIdx < m_experimentSequence.size() - 1) {
            file << "\n\n";
        }
    }

    file.close();
    std::cout << "[UserStudy] Sequence saved to: " << filename << std::endl;
}

void UserStudyManager::saveExp2Sequence(const std::string& filename)
{
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "[UserStudy][Exp2] Cannot save sequence: " << filename << std::endl;
        return;
    }
    file << "# Experiment2 angle sequence (plane planeSpinDeg inPlaneDeg)\n";
    for (const auto& t : m_exp2Sequence) {
        file << static_cast<int>(t.plane) << " "
            << t.planeRotationDeg << " "
            << t.inPlaneAngleDeg << "\n";
    }
    file.close();
}

// ???????
bool UserStudyManager::loadSequenceFromFile(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "[UserStudy] Cannot open sequence file: " << filename << std::endl;
        return false;
    }

    m_experimentSequence.clear();
    std::string line;
    bool readingGroups = true;
    int currentGroupIdx = -1;

    std::cout << "[UserStudy] Loading sequence from: " << filename << std::endl;

    while (std::getline(file, line)) {
        // ???????
        if (line.empty() || line[0] == '#') {
            // ????????????
            if (line.find("Trial Sequence") != std::string::npos) {
                readingGroups = false;
                std::cout << "[UserStudy] Starting to read trial sequences..." << std::endl;
            }
            continue;
        }

        std::istringstream iss(line);

        if (readingGroups && m_experimentSequence.size() < 6) {
            // ?????
            int groupIdx, mode, dir;
            if (iss >> groupIdx >> mode >> dir) {
                GroupConfig group;
                group.mode = static_cast<InteractionMode>(mode);
                group.direction = static_cast<FeedbackDirection>(dir);
                group.trials.clear(); // ??trials?????
                m_experimentSequence.push_back(group);

                std::cout << "[UserStudy] Loaded group " << groupIdx
                    << " Mode:" << mode << " Dir:" << dir << std::endl;
            }
        }
        else if (!readingGroups) {
            // ??????????????
            if (currentGroupIdx < 0 || currentGroupIdx >= m_experimentSequence.size()) {
                // ???????????
                for (size_t i = 0; i < m_experimentSequence.size(); ++i) {
                    if (m_experimentSequence[i].trials.size() < 110) {
                        currentGroupIdx = i;
                        break;
                    }
                }
            }

            if (currentGroupIdx >= 0 && currentGroupIdx < m_experimentSequence.size()) {
                std::vector<int> tokens;
                int value;
                while (iss >> value) {
                    tokens.push_back(value);
                }

                size_t stride = 0;
                if (!tokens.empty()) {
                    stride = (tokens.size() % 3 == 0) ? 3 : 2;
                }

                for (size_t idx = 0; stride != 0 && idx + stride <= tokens.size(); idx += stride) {
                    int stiffIdx = tokens[idx];
                    int rep = tokens[idx + 1];
                    SurfacePolarity pol = SurfacePolarity::POSITIVE;
                    if (stride == 3) {
                        pol = polarityFromInt(tokens[idx + 2]);
                    }
                    if (m_experimentSequence[currentGroupIdx].trials.size() < 110) {
                        m_experimentSequence[currentGroupIdx].trials.push_back({ stiffIdx, rep, pol });
                    }
                }

                // ???????,?????
                if (m_experimentSequence[currentGroupIdx].trials.size() >= 110) {
                    std::cout << "[UserStudy] Group " << currentGroupIdx
                        << " loaded with " << m_experimentSequence[currentGroupIdx].trials.size()
                        << " trials" << std::endl;
                    currentGroupIdx++;
                }
            }
        }
    }

    file.close();

    // ???????
    std::cout << "[UserStudy] Sequence loading complete. Validating..." << std::endl;
    std::cout << "[UserStudy] Total groups loaded: " << m_experimentSequence.size() << std::endl;

    bool valid = true;
    if (m_experimentSequence.size() != 6) {
        std::cerr << "[UserStudy] ERROR: Expected 6 groups, got "
            << m_experimentSequence.size() << std::endl;
        valid = false;
    }

    for (size_t i = 0; i < m_experimentSequence.size(); ++i) {
        size_t trialCount = m_experimentSequence[i].trials.size();
        std::cout << "[UserStudy] Group " << i << " has " << trialCount << " trials" << std::endl;

        if (trialCount != 110) {
            std::cerr << "[UserStudy] ERROR: Group " << i
                << " should have 110 trials, has " << trialCount << std::endl;
            valid = false;
        }
    }

    m_sequenceLoaded = valid;

    if (!valid) {
        std::cerr << "[UserStudy] Sequence validation failed! Clearing data." << std::endl;
        m_experimentSequence.clear();
    }

    return m_sequenceLoaded;
}

bool UserStudyManager::loadExp2Sequence(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file) {
        return false;
    }
    m_exp2Sequence.clear();
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        int planeInt = 0;
        double planeSpin = 0.0;
        double inPlane = 0.0;
        if (!(iss >> planeInt)) continue;
        // Old format: only one angle value
        if (!(iss >> planeSpin)) continue;
        if (!(iss >> inPlane)) {
            // two-column legacy format: plane angle
            inPlane = planeSpin;
            planeSpin = 0.0;
        }

        if (planeInt < 0 || planeInt > static_cast<int>(Exp2Plane::PLANE_COMBINED)) {
            continue;
        }
        m_exp2Sequence.push_back({ static_cast<Exp2Plane>(planeInt), planeSpin, inPlane });
    }
    m_exp2SequenceLoaded = !m_exp2Sequence.empty();
    rebuildExp2GroupSizesFromSequence();
    ensureExp2GroupProgressSize();
    return m_exp2SequenceLoaded;
}

int UserStudyManager::expectedExp2GroupSizeForPlane(Exp2Plane plane) const
{
    switch (plane) {
    case Exp2Plane::PLANE_X:
    case Exp2Plane::PLANE_Y:
    case Exp2Plane::PLANE_Z:
        return static_cast<int>(getExp2TrialsPerAxisGroup());
    case Exp2Plane::PLANE_COMBINED:
        return static_cast<int>(getExp2CombinedGroupSize());
    default:
        return 0;
    }
}

void UserStudyManager::rebuildExp2GroupSizesFromSequence()
{
    m_exp2GroupSizes.clear();
    if (m_exp2Sequence.empty()) {
        m_exp2GroupSizes = getDefaultExp2GroupSizes();
        return;
    }

    Exp2Plane currentPlane = m_exp2Sequence.front().plane;
    int count = 0;
    for (const auto& cfg : m_exp2Sequence) {
        if (cfg.plane != currentPlane) {
            m_exp2GroupSizes.push_back(count);
            currentPlane = cfg.plane;
            count = 0;
        }
        ++count;
    }
    if (count > 0) {
        m_exp2GroupSizes.push_back(count);
    }
    if (m_exp2GroupSizes.empty()) {
        m_exp2GroupSizes = getDefaultExp2GroupSizes();
    }
}

bool UserStudyManager::isExp2SequenceGroupedByPlane() const
{
    if (!m_exp2SequenceLoaded ||
        m_exp2Sequence.size() != getExp2ExpectedTrialCountInternal()) {
        return false;
    }

    if (m_exp2Sequence.empty()) return false;

    std::vector<Exp2Plane> expectedOrder;
    for (auto p : kExp2AxisPlanes) expectedOrder.push_back(p);
    if (getExp2CombinedGroupSize() > 0) expectedOrder.push_back(Exp2Plane::PLANE_COMBINED);

    std::vector<int> expectedSizes = getDefaultExp2GroupSizes();

    std::vector<int> actualSizes;
    std::vector<Exp2Plane> actualOrder;
    Exp2Plane currentPlane = m_exp2Sequence.front().plane;
    int count = 0;
    for (const auto& cfg : m_exp2Sequence) {
        if (cfg.plane != currentPlane) {
            actualOrder.push_back(currentPlane);
            actualSizes.push_back(count);
            currentPlane = cfg.plane;
            count = 0;
        }
        ++count;
    }
    if (count > 0) {
        actualOrder.push_back(currentPlane);
        actualSizes.push_back(count);
    }

    if (actualOrder.size() != expectedOrder.size() ||
        actualSizes.size() != expectedSizes.size()) {
        return false;
    }

    for (size_t i = 0; i < expectedOrder.size(); ++i) {
        if (actualOrder[i] != expectedOrder[i]) return false;
        if (actualSizes[i] != expectedSizes[i]) return false;
    }

    return true;
}

void UserStudyManager::regroupExp2SequenceByPlane(bool shuffleWithinGroups)
{
    if (m_exp2Sequence.empty()) return;

    std::vector<Exp2Plane> order;
    for (auto p : kExp2AxisPlanes) order.push_back(p);
    if (getExp2CombinedGroupSize() > 0) order.push_back(Exp2Plane::PLANE_COMBINED);

    std::vector<std::vector<AngleTrialConfig>> buckets(order.size());
    std::map<Exp2Plane, size_t> idxMap;
    for (size_t i = 0; i < order.size(); ++i) {
        idxMap[order[i]] = i;
    }

    for (const auto& cfg : m_exp2Sequence) {
        auto it = idxMap.find(cfg.plane);
        if (it == idxMap.end()) continue;
        buckets[it->second].push_back(cfg);
    }

    for (size_t i = 0; i < buckets.size(); ++i) {
        int expectedSize = expectedExp2GroupSizeForPlane(order[i]);
        if (expectedSize > 0 && buckets[i].size() != static_cast<size_t>(expectedSize)) {
            std::cout << "[UserStudy][Exp2] Regroup skipped: unexpected trial count for plane "
                << static_cast<int>(order[i]) << " (" << buckets[i].size()
                << " vs expected " << expectedSize << ")" << std::endl;
            return;
        }
    }

    if (shuffleWithinGroups) {
        std::random_device rd;
        std::mt19937 g(rd());
        for (auto& bucket : buckets) {
            std::shuffle(bucket.begin(), bucket.end(), g);
        }
    }

    m_exp2GroupSizes.clear();
    m_exp2Sequence.clear();
    for (const auto& bucket : buckets) {
        m_exp2GroupSizes.push_back(static_cast<int>(bucket.size()));
        m_exp2Sequence.insert(m_exp2Sequence.end(), bucket.begin(), bucket.end());
    }
    m_exp2SequenceLoaded = (m_exp2Sequence.size() == getExp2ExpectedTrialCountInternal());
}

void UserStudyManager::ensureExp2SequenceGrouped()
{
    if (!m_exp2SequenceLoaded) {
        return;
    }

    rebuildExp2GroupSizesFromSequence();
    if (isExp2SequenceGroupedByPlane()) {
        return;
    }

    std::cout << "[UserStudy][Exp2] Regrouping sequence into plane-specific blocks (X/Y/Z + combined angles)." << std::endl;
    regroupExp2SequenceByPlane(true);
    if (isExp2SequenceGroupedByPlane()) {
        saveExp2Sequence(getExp2SequenceFilename());
    }
}

int UserStudyManager::findExp2GroupStartIndex(Exp2Plane plane) const
{
    if (!m_exp2SequenceLoaded || m_exp2Sequence.empty()) {
        return -1;
    }

    int offset = 0;
    int groupCount = getExp2GroupCount();
    for (int g = 0; g < groupCount; ++g) {
        int size = getExp2GroupSize(g);
        if (size <= 0) break;
        if (offset < static_cast<int>(m_exp2Sequence.size()) &&
            m_exp2Sequence[offset].plane == plane) {
            return offset;
        }
        offset += size;
    }

    for (size_t i = 0; i < m_exp2Sequence.size(); ++i) {
        if (m_exp2Sequence[i].plane == plane) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void UserStudyManager::setManualExp2Group(Exp2Plane plane)
{
    if (!m_exp2SequenceLoaded || m_exp2Sequence.empty()) {
        std::cout << "[UserStudy][Exp2] No sequence loaded. Unable to change group." << std::endl;
        return;
    }

    int startIdx = findExp2GroupStartIndex(plane);
    if (startIdx < 0) {
        std::cout << "[UserStudy][Exp2] Plane group not found in current sequence." << std::endl;
        return;
    }

    int groupIdx = getExp2GroupIndexForTrial(startIdx);
    ensureExp2GroupProgressSize();
    int trialOffset = (groupIdx >= 0 && groupIdx < static_cast<int>(m_exp2GroupProgress.size()))
        ? m_exp2GroupProgress[groupIdx]
        : 0;
    int groupSize = getExp2GroupSize(groupIdx);
    if (trialOffset > groupSize) trialOffset = groupSize;
    m_exp2CurrentGroup = groupIdx;
    int groupStart = getExp2GroupStartIndex(groupIdx);
    m_exp2CurrentTrialIndex = groupStart + trialOffset;
    if (m_exp2CurrentTrialIndex >= static_cast<int>(m_exp2Sequence.size())) {
        if (groupSize > 0) {
            m_exp2CurrentTrialIndex = groupStart + groupSize - 1;
        }
        else {
            m_exp2CurrentTrialIndex = static_cast<int>(m_exp2Sequence.size()) - 1;
        }
        if (m_exp2CurrentTrialIndex < 0) m_exp2CurrentTrialIndex = 0;
    }
    m_exp2TrialActive = false;
    m_exp2NeedBreak = false;
    m_exp2CurrentTrial.plane = plane;
    m_exp2CurrentTrial.targetPlaneRotationDeg = 0.0;
    m_exp2CurrentTrial.targetInPlaneAngleDeg = 0.0;
    m_exp2CurrentTrial.userPlaneRotationDeg = 0.0;
    m_exp2CurrentTrial.userInPlaneAngleDeg = 0.0;
    bool remaining = hasNextTrialExp2();
    m_exp2State = remaining ? ExperimentState::READY : ExperimentState::EXPERIMENT_COMPLETE;

    saveProgress();

    std::cout << "[UserStudy][Exp2] Manual jump to plane "
        << static_cast<int>(plane)
        << " starting at trial #" << (m_exp2CurrentTrialIndex + 1)
        << " of " << m_exp2Sequence.size() << std::endl;
}

// ????
void UserStudyManager::saveProgress()
{
    std::ofstream file(getProgressFilename());
    if (!file) return;

    file << "# Progress for User: " << m_userId << "\n";

    ensureExp1GroupProgressSize();
    ensureExp2GroupProgressSize();

    int exp1TotalDone = getExp1CompletedTrials();
    int exp2TotalDone = getExp2CompletedTrials();

    file << "Exp1CurrentTrial " << (m_currentTrialGroup * 110 + m_currentTrialInGroup) << "\n";
    file << "Exp1CurrentGroup " << m_currentTrialGroup << "\n";
    file << "Exp1State " << static_cast<int>(m_experimentState) << "\n";
    file << "Exp1TotalTrialsCompleted " << exp1TotalDone << "\n";
    file << "Exp1GroupProgress";
    for (int v : m_groupProgress) file << " " << v;
    file << "\n";

    file << "Exp2CurrentTrial " << m_exp2CurrentTrialIndex << "\n";
    file << "Exp2CurrentGroup " << m_exp2CurrentGroup << "\n";
    file << "Exp2State " << static_cast<int>(m_exp2State) << "\n";
    file << "Exp2TotalTrialsCompleted " << exp2TotalDone << "\n";
    file << "Exp2GroupProgress";
    for (int v : m_exp2GroupProgress) file << " " << v;
    file << "\n";

    file.close();
}

// ????
// ?? loadProgress ??,??????
bool UserStudyManager::loadProgress(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file) return false;

    // defaults in case keys missing (backward compatibility)
    m_exp2CurrentTrialIndex = 0;
    m_exp2State = ExperimentState::READY;
    m_exp2NeedBreak = false;
    m_exp2TrialActive = false;
    m_groupProgress.clear();
    m_exp2GroupProgress.clear();
    m_currentTrialGroup = 0;
    m_currentTrialInGroup = 0;
    m_experimentState = ExperimentState::READY;
    int exp1CurrentTrialFlat = -1;
    int exp2CurrentTrialFlat = -1;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::string key;
        if (iss >> key) {
            // New-format keys
            if (key == "Exp1CurrentTrial") {
                iss >> exp1CurrentTrialFlat;
            }
            else if (key == "Exp1CurrentGroup") {
                iss >> m_currentTrialGroup;
            }
            else if (key == "Exp1State") {
                int stateInt = 0;
                iss >> stateInt;
                if (stateInt < 0 || stateInt > static_cast<int>(ExperimentState::EXPERIMENT_COMPLETE)) stateInt = 0;
                m_experimentState = static_cast<ExperimentState>(stateInt);
            }
            else if (key == "Exp1TotalTrialsCompleted") {
                int tmp; iss >> tmp;
            }
            else if (key == "Exp1GroupProgress") {
                int v;
                while (iss >> v) {
                    m_groupProgress.push_back(v);
                }
            }
            else if (key == "Exp2CurrentTrial") {
                iss >> exp2CurrentTrialFlat;
                if (exp2CurrentTrialFlat < 0) exp2CurrentTrialFlat = 0;
            }
            else if (key == "Exp2CurrentGroup") {
                iss >> m_exp2CurrentGroup;
                if (m_exp2CurrentGroup < 0) m_exp2CurrentGroup = 0;
            }
            else if (key == "Exp2State") {
                int stateInt = 0;
                iss >> stateInt;
                if (stateInt < 0 || stateInt > static_cast<int>(ExperimentState::EXPERIMENT_COMPLETE)) stateInt = 0;
                m_exp2State = static_cast<ExperimentState>(stateInt);
            }
            else if (key == "Exp2GroupProgress") {
                int v;
                while (iss >> v) {
                    m_exp2GroupProgress.push_back(v);
                }
            }
            else if (key == "Exp2TotalTrialsCompleted") {
                int tmp; iss >> tmp;
            }
            // Legacy keys (backward compatibility)
            else if (key == "CurrentGroup") {
                iss >> m_currentTrialGroup;
            }
            else if (key == "CurrentTrialInGroup") {
                iss >> m_currentTrialInGroup;
            }
            else if (key == "TotalTrialsCompleted") {
                int tmp; iss >> tmp;
            }
            else if (key == "Exp2CurrentTrialLegacy") {
                // no-op placeholder
            }
        }
    }

    file.close();
    ensureExp1GroupProgressSize();
    ensureExp2GroupProgressSize();

    // Derive Exp1 indices
    if (exp1CurrentTrialFlat >= 0) {
        m_currentTrialGroup = exp1CurrentTrialFlat / 110;
        m_currentTrialInGroup = exp1CurrentTrialFlat % 110;
    }
    // Backward compatibility: if group progress missing, seed from current indices
    if (m_groupProgress.empty() && !m_experimentSequence.empty()) {
        m_groupProgress.resize(m_experimentSequence.size(), 0);
    }
    if (m_currentTrialGroup >= 0 && m_currentTrialGroup < static_cast<int>(m_groupProgress.size())) {
        if (m_groupProgress[m_currentTrialGroup] < m_currentTrialInGroup) {
            m_groupProgress[m_currentTrialGroup] = m_currentTrialInGroup;
        }
        else {
            m_currentTrialInGroup = m_groupProgress[m_currentTrialGroup];
        }
    }

    int exp2GroupCount = getExp2GroupCount();
    if (exp2CurrentTrialFlat >= 0) {
        m_exp2CurrentTrialIndex = exp2CurrentTrialFlat;
    }
    if (m_exp2GroupProgress.empty() && exp2GroupCount > 0) {
        m_exp2GroupProgress.resize(exp2GroupCount, 0);
        int remaining = m_exp2CurrentTrialIndex;
        for (int g = 0; g < exp2GroupCount; ++g) {
            int size = getExp2GroupSize(g);
            if (remaining >= size) {
                m_exp2GroupProgress[g] = size;
                remaining -= size;
            }
            else {
                m_exp2GroupProgress[g] = std::max(0, remaining);
                remaining = 0;
            }
        }
    }
    if (m_exp2CurrentGroup < 0 || m_exp2CurrentGroup >= exp2GroupCount) {
        m_exp2CurrentGroup = getExp2GroupIndexForTrial(m_exp2CurrentTrialIndex);
        if (m_exp2CurrentGroup < 0 || m_exp2CurrentGroup >= exp2GroupCount) {
            m_exp2CurrentGroup = 0;
        }
    }
    if (m_exp2CurrentGroup < static_cast<int>(m_exp2GroupProgress.size())) {
        int grpProg = m_exp2GroupProgress[m_exp2CurrentGroup];
        int grpSize = getExp2GroupSize(m_exp2CurrentGroup);
        if (grpProg > grpSize) grpProg = grpSize;
        int computedIndex = getExp2GroupStartIndex(m_exp2CurrentGroup) + grpProg;
        if (computedIndex < static_cast<int>(m_exp2Sequence.size())) {
            m_exp2CurrentTrialIndex = computedIndex;
        }
    }
    sanitizeProgressIndices();
    return true;
}

// ?? startNextTrial ??
void UserStudyManager::startNextTrial()
{
    // ??????
    if (m_experimentState == ExperimentState::EXPERIMENT_COMPLETE) {
        std::cout << "[UserStudy] Experiment already complete!" << std::endl;
        return;
    }

    if (m_experimentState == ExperimentState::IN_PROGRESS) {
        std::cout << "[UserStudy] Trial already in progress! Make a choice first." << std::endl;
        return;
    }

    if (!m_sequenceLoaded) {
        std::cout << "[UserStudy] No sequence loaded!" << std::endl;
        return;
    }

    if (m_experimentSequence.empty()) {
        std::cout << "[UserStudy] ERROR: Experiment sequence is empty!" << std::endl;
        return;
    }

    sanitizeProgressIndices();
    ensureExp1GroupProgressSize();
    if (m_currentTrialGroup >= 0 && m_currentTrialGroup < static_cast<int>(m_groupProgress.size())) {
        m_currentTrialInGroup = m_groupProgress[m_currentTrialGroup];
    }

    // ????
    if (m_currentTrialGroup >= m_experimentSequence.size()) {
        std::cout << "[UserStudy] ERROR: Invalid group index " << m_currentTrialGroup
            << " (max: " << m_experimentSequence.size() - 1 << ")" << std::endl;
        m_experimentState = ExperimentState::EXPERIMENT_COMPLETE;
        return;
    }

    // ???????????
    if (m_currentTrialInGroup >= 110) {
        if (m_currentTrialGroup >= 0 && m_currentTrialGroup < static_cast<int>(m_groupProgress.size())) {
            m_groupProgress[m_currentTrialGroup] = 110;
        }
        m_currentTrialGroup++;
        if (m_currentTrialGroup >= 6) {
            m_experimentState = ExperimentState::EXPERIMENT_COMPLETE;
            std::cout << "[UserStudy] Experiment Complete! Thank you for your participation." << std::endl;
            saveResults("user_" + m_userId + "_final_results.csv");
            return;
        }
        ensureExp1GroupProgressSize();
        if (m_currentTrialGroup < static_cast<int>(m_groupProgress.size())) {
            m_currentTrialInGroup = m_groupProgress[m_currentTrialGroup];
        }
        else {
            m_currentTrialInGroup = 0;
        }
    }

    // ???????
    if (m_currentTrialGroup >= m_experimentSequence.size()) {
        std::cout << "[UserStudy] ERROR: Group index out of bounds!" << std::endl;
        return;
    }

    // ???????
    const auto& currentGroup = m_experimentSequence[m_currentTrialGroup];

    // ??????
    if (m_currentTrialInGroup >= currentGroup.trials.size()) {
        std::cout << "[UserStudy] ERROR: Trial index " << m_currentTrialInGroup
            << " out of bounds (max: " << currentGroup.trials.size() - 1 << ")" << std::endl;
        return;
    }

    m_currentMode = currentGroup.mode;
    m_currentDirection = currentGroup.direction;

    // ????????
    const auto& trialConfig = currentGroup.trials[m_currentTrialInGroup];

    // ??????
    if (trialConfig.stiffnessIndex >= m_comparisonStiffnesses.size()) {
        std::cout << "[UserStudy] ERROR: Stiffness index " << trialConfig.stiffnessIndex
            << " out of bounds (max: " << m_comparisonStiffnesses.size() - 1 << ")" << std::endl;
        return;
    }

    // ?????
    m_currentTrial = TrialResult{};
    m_currentTrial.trialNumber = m_currentTrialGroup * 110 + m_currentTrialInGroup;
    m_currentTrial.referenceStiffness = m_referenceStiffness;
    m_currentTrial.comparisonStiffness = m_comparisonStiffnesses[trialConfig.stiffnessIndex];
    m_currentPolarity = trialConfig.polarity;
    m_currentTrial.polarity = m_currentPolarity;
    // ? ??:???T????????????
    m_currentTrial.trialStartTime = std::chrono::steady_clock::now();
    m_trialActive = true;
    m_experimentState = ExperimentState::IN_PROGRESS;

    std::cout << "\n[UserStudy] === NEW TRIAL ===" << std::endl;
    std::cout << "Trial " << (m_currentTrialInGroup + 1) << "/110"
        << " | Group " << (m_currentTrialGroup + 1) << "/6"
        << " | Mode: " << ((int)m_currentMode == 0 ? "F-P" : "F-F")
        << " | Direction: ";

    switch (m_currentDirection) {
    case FeedbackDirection::LATERAL_X: std::cout << "X"; break;
    case FeedbackDirection::LATERAL_Y: std::cout << "Y"; break;
    case FeedbackDirection::VERTICAL_Z: std::cout << "Z"; break;
    }

    std::cout << " | Polarity: "
        << ((m_currentPolarity == SurfacePolarity::POSITIVE) ? "+ (Red)" : "- (Blue)");

    std::cout << "\nRef: " << m_currentTrial.referenceStiffness
        << " | Comp: " << m_currentTrial.comparisonStiffness
        << " (Ratio: " << std::fixed << std::setprecision(1)
        << (trialConfig.stiffnessIndex * 0.1 + 0.5) << ")"
        << std::endl;
}

bool UserStudyManager::peekNextTrialSurface(FeedbackDirection& direction, SurfacePolarity& polarity) const
{
	if (!m_sequenceLoaded || m_experimentSequence.empty()) {
		return false;
	}

	int groupIndex = m_currentTrialGroup;
	int trialIndex = m_currentTrialInGroup;

	if (trialIndex >= 110) {
		trialIndex = 0;
		groupIndex++;
	}

	if (groupIndex < 0 || groupIndex >= static_cast<int>(m_experimentSequence.size())) {
		return false;
	}

	const auto& group = m_experimentSequence[groupIndex];
	if (group.trials.empty()) {
		return false;
	}

	if (trialIndex < 0 || trialIndex >= static_cast<int>(group.trials.size())) {
		return false;
	}

	direction = group.direction;
	polarity = group.trials[trialIndex].polarity;
	return true;
}

// ?? recordUserChoice ??
void UserStudyManager::recordUserChoice(int choice)
{
    if (!m_trialActive || m_experimentState != ExperimentState::IN_PROGRESS) {
        std::cout << "[UserStudy] No active trial to record choice!" << std::endl;
        return;
    }

    m_currentTrial.choiceTime = std::chrono::steady_clock::now();
    m_currentTrial.userChoice = choice;

    m_currentTrial.reactionTime =
        std::chrono::duration<double>(m_currentTrial.choiceTime - m_currentTrial.trialStartTime).count();

    m_results.push_back(m_currentTrial);
    m_trialActive = false;

    // ????CSV??
    if (m_csvFileReady) {
        writeTrialToCSV(m_currentTrial);
    }

    std::cout << "[UserStudy] Choice recorded: " << (choice == 0 ? "Left" : "Right")
        << " | Reaction Time: " << std::fixed << std::setprecision(3)
        << m_currentTrial.reactionTime << "s" << std::endl;

    // ????
    m_currentTrialInGroup++;
    ensureExp1GroupProgressSize();
    if (m_currentTrialGroup >= 0 && m_currentTrialGroup < static_cast<int>(m_groupProgress.size())) {
        if (m_currentTrialInGroup > m_groupProgress[m_currentTrialGroup]) {
            m_groupProgress[m_currentTrialGroup] = m_currentTrialInGroup;
        }
    }

    // ????
    if (m_currentTrialInGroup >= 110) {
        if (m_currentTrialGroup >= 0 && m_currentTrialGroup < static_cast<int>(m_groupProgress.size())) {
            m_groupProgress[m_currentTrialGroup] = 110;
        }
        m_experimentState = ExperimentState::GROUP_COMPLETE;
        std::cout << "\n[UserStudy] >>> GROUP " << (m_currentTrialGroup + 1)
            << " COMPLETE! <<<" << std::endl;

        if (m_currentTrialGroup + 1 < 6) {
            std::cout << "Take a 5-minute break before continuing." << std::endl;
            std::cout << "Press 'T' when ready to start next group." << std::endl;
        }
        else {
            m_experimentState = ExperimentState::EXPERIMENT_COMPLETE;
            std::cout << "\n*** EXPERIMENT COMPLETE! ***" << std::endl;
            std::cout << "Thank you for your participation!" << std::endl;
            saveResults("user_" + m_userId + "_final_results.csv");
        }
    }
    else {
        m_experimentState = ExperimentState::TRIAL_COMPLETE;

        // ?55???????
        if (m_currentTrialInGroup == 55) {
            std::cout << "\n*** 2-MINUTE BREAK TIME ***" << std::endl;
            std::cout << "You've completed 55 trials. Please rest for 2 minutes." << std::endl;
        }

        std::cout << "Press 'T' for next trial." << std::endl;
    }

    // ????
    saveProgress();
}

// ????????...
double UserStudyManager::getCurrentReferenceStiffness() const
{
    return m_currentTrial.referenceStiffness;
}

double UserStudyManager::getCurrentComparisonStiffness() const
{
    return m_currentTrial.comparisonStiffness;
}

void UserStudyManager::saveResults(const std::string& fn)
{
    std::ofstream file(fn);
    if (!file) {
        std::cerr << "[UserStudy] Cannot open " << fn << "\n";
        return;
    }

    file << "TrialNumber,Mode,Direction,ReferenceStiffness,ComparisonStiffness,"
        << "UserChoice,ReactionTime,Polarity\n";

    for (const auto& r : m_results) {
        // ??????????
        int groupIdx = r.trialNumber / 110;
        if (groupIdx < m_experimentSequence.size()) {
            const auto& group = m_experimentSequence[groupIdx];
            file << r.trialNumber << ','
                << (int)group.mode << ','
                << (int)group.direction << ','
                << r.referenceStiffness << ','
                << r.comparisonStiffness << ','
                << r.userChoice << ','
                << r.reactionTime << ','
                << polarityToString(r.polarity) << '\n';
        }
    }

    std::cout << "[UserStudy] Saved " << m_results.size() << " trials -> " << fn << std::endl;
}

void UserStudyManager::setManualTrialGroup(InteractionMode m, FeedbackDirection d)
{
    // ?????,??????
    for (size_t i = 0; i < m_experimentSequence.size(); ++i) {
        if (m_experimentSequence[i].mode == m &&
            m_experimentSequence[i].direction == d) {
            m_currentTrialGroup = i;
            ensureExp1GroupProgressSize();
            m_currentTrialInGroup = (i < m_groupProgress.size()) ? m_groupProgress[i] : 0;
            m_currentMode = m;
            m_currentDirection = d;
            std::cout << "[UserStudy] Manual jump to Group " << i
                << " Mode:" << (int)m << " Dir:" << (int)d << std::endl;
            saveProgress();
            return;
        }
    }
    std::cout << "[UserStudy] Warning: Group not found in sequence!\n";
}

void UserStudyManager::resetCurrentGroup()
{
    m_currentTrialInGroup = 0;
    ensureExp1GroupProgressSize();
    if (m_currentTrialGroup >= 0 && m_currentTrialGroup < static_cast<int>(m_groupProgress.size())) {
        m_groupProgress[m_currentTrialGroup] = 0;
    }
    saveProgress();
    std::cout << "[UserStudy] Group reset.\n";
}

void UserStudyManager::sanitizeProgressIndices()
{
    ensureExp1GroupProgressSize();
    if (!m_sequenceLoaded || m_experimentSequence.empty()) {
        m_currentTrialGroup = 0;
        m_currentTrialInGroup = 0;
        return;
    }

    const int groupCount = static_cast<int>(m_experimentSequence.size());
    if (m_currentTrialGroup < 0 || m_currentTrialGroup >= groupCount) {
        std::cout << "[UserStudy] WARNING: Invalid group index " << m_currentTrialGroup
            << ". Resetting to 0." << std::endl;
        m_currentTrialGroup = 0;
        m_currentTrialInGroup = 0;
    }

    const auto& group = m_experimentSequence[m_currentTrialGroup];
    const int trialsInGroup = static_cast<int>(group.trials.size());
    if (trialsInGroup == 0) {
        std::cout << "[UserStudy] WARNING: Group " << m_currentTrialGroup
            << " has no trials configured. Resetting trial index." << std::endl;
        m_currentTrialInGroup = 0;
        return;
    }

    if (m_currentTrialInGroup < 0 || m_currentTrialInGroup >= trialsInGroup) {
        std::cout << "[UserStudy] WARNING: Invalid trial index " << m_currentTrialInGroup
            << " for group " << m_currentTrialGroup << ". Resetting to 0." << std::endl;
        m_currentTrialInGroup = 0;
    }
}

void UserStudyManager::ensureExp1GroupProgressSize()
{
    size_t groupCount = m_experimentSequence.size();
    if (groupCount == 0) return;
    if (m_groupProgress.size() < groupCount) {
        m_groupProgress.resize(groupCount, 0);
    }
    // clamp oversized entries
    for (size_t i = 0; i < groupCount; ++i) {
        int maxTrials = (i < m_experimentSequence.size()) ? static_cast<int>(m_experimentSequence[i].trials.size()) : 110;
        if (maxTrials < 0) maxTrials = 0;
        if (m_groupProgress[i] < 0) m_groupProgress[i] = 0;
        if (m_groupProgress[i] > maxTrials) m_groupProgress[i] = maxTrials;
    }
}

void UserStudyManager::ensureExp2GroupProgressSize()
{
    int groupCount = getExp2GroupCount();
    if (groupCount < 0) groupCount = 0;
    if (m_exp2GroupProgress.size() < static_cast<size_t>(groupCount)) {
        m_exp2GroupProgress.resize(groupCount, 0);
    }
    for (int i = 0; i < groupCount; ++i) {
        int maxTrials = getExp2GroupSize(i);
        if (maxTrials < 0) maxTrials = 0;
        if (m_exp2GroupProgress[i] < 0) m_exp2GroupProgress[i] = 0;
        if (m_exp2GroupProgress[i] > maxTrials) m_exp2GroupProgress[i] = maxTrials;
    }
}

int UserStudyManager::findNextExp2GroupWithRemaining(int startGroup) const
{
    int groupCount = getExp2GroupCount();
    if (groupCount <= 0) return -1;
    for (int offset = 0; offset < groupCount; ++offset) {
        int idx = (startGroup + offset) % groupCount;
        int size = getExp2GroupSize(idx);
        int prog = (idx < static_cast<int>(m_exp2GroupProgress.size())) ? m_exp2GroupProgress[idx] : 0;
        if (prog < size) return idx;
    }
    return -1;
}


void UserStudyManager::initializeTrials()
{
    m_results.clear();
    std::cout << "[UserStudy] Trials initialized\n";
}

// ?? hasNextTrial ??
bool UserStudyManager::hasNextTrial()
{
    if (!m_sequenceLoaded ||
        m_experimentState == ExperimentState::EXPERIMENT_COMPLETE) {
        return false;
    }

    if (m_experimentSequence.empty()) {
        return false;
    }

    sanitizeProgressIndices();

    if (m_currentTrialGroup < 0 ||
        m_currentTrialGroup >= static_cast<int>(m_experimentSequence.size())) {
        return false;
    }

    return true;
}

bool UserStudyManager::hasNextTrialExp2() const
{
    if (!m_exp2SequenceLoaded ||
        m_exp2State == ExperimentState::EXPERIMENT_COMPLETE) {
        return false;
    }
    int groupCount = getExp2GroupCount();
    for (int i = 0; i < groupCount; ++i) {
        int size = getExp2GroupSize(i);
        int prog = (i < static_cast<int>(m_exp2GroupProgress.size())) ? m_exp2GroupProgress[i] : 0;
        if (prog < size) return true;
    }
    return false;
}

void UserStudyManager::startNextTrialExp2()
{
    if (!hasNextTrialExp2()) {
        m_exp2CurrentTrialIndex = static_cast<int>(m_exp2Sequence.size());
        m_exp2State = ExperimentState::EXPERIMENT_COMPLETE;
        return;
    }

    m_exp2NeedBreak = false;
    ensureExp2GroupProgressSize();
    int groupCount = getExp2GroupCount();
    if (groupCount <= 0) {
        m_exp2State = ExperimentState::EXPERIMENT_COMPLETE;
        return;
    }

    if (m_exp2CurrentGroup < 0 || m_exp2CurrentGroup >= groupCount) {
        m_exp2CurrentGroup = 0;
    }

    int targetGroup = m_exp2CurrentGroup;
    int grpSize = getExp2GroupSize(targetGroup);
    int grpProg = (targetGroup < static_cast<int>(m_exp2GroupProgress.size())) ? m_exp2GroupProgress[targetGroup] : 0;
    if (grpProg >= grpSize) {
        int nextGroup = findNextExp2GroupWithRemaining(targetGroup + 1);
        if (nextGroup < 0) {
            m_exp2State = ExperimentState::EXPERIMENT_COMPLETE;
            m_exp2CurrentTrialIndex = static_cast<int>(m_exp2Sequence.size());
            return;
        }
        targetGroup = nextGroup;
        grpSize = getExp2GroupSize(targetGroup);
        grpProg = (targetGroup < static_cast<int>(m_exp2GroupProgress.size())) ? m_exp2GroupProgress[targetGroup] : 0;
    }

    m_exp2CurrentGroup = targetGroup;
    int groupStart = getExp2GroupStartIndex(targetGroup);
    int trialIdx = groupStart + grpProg;
    if (trialIdx < 0 || trialIdx >= static_cast<int>(m_exp2Sequence.size())) {
        m_exp2State = ExperimentState::EXPERIMENT_COMPLETE;
        m_exp2CurrentTrialIndex = static_cast<int>(m_exp2Sequence.size());
        return;
    }

    m_exp2CurrentTrialIndex = trialIdx;
    const auto& cfg = m_exp2Sequence[m_exp2CurrentTrialIndex];
    m_exp2CurrentTrial.trialNumber = m_exp2CurrentTrialIndex;
    m_exp2CurrentTrial.plane = cfg.plane;
    m_exp2CurrentTrial.targetPlaneRotationDeg = cfg.planeRotationDeg;
    m_exp2CurrentTrial.targetInPlaneAngleDeg = cfg.inPlaneAngleDeg;
    m_exp2CurrentTrial.userPlaneRotationDeg = 0.0;
    m_exp2CurrentTrial.userInPlaneAngleDeg = 0.0;
    m_exp2CurrentTrial.trialStartTime = std::chrono::steady_clock::now();
    m_exp2TrialStart = m_exp2CurrentTrial.trialStartTime;
    m_exp2TrialActive = true;
    m_exp2State = ExperimentState::IN_PROGRESS;
    m_exp2NeedBreak = false;
}

void UserStudyManager::recordExp2UserPose(double planeSpinDeg, double inPlaneAngleDeg, double durationSeconds)
{
    if (!m_exp2TrialActive) {
        std::cout << "[UserStudy][Exp2] No active trial to record.\n";
        return;
    }

    m_exp2CurrentTrial.confirmTime = std::chrono::steady_clock::now();
    if (durationSeconds <= 0.0) {
        durationSeconds = std::chrono::duration<double>(m_exp2CurrentTrial.confirmTime - m_exp2TrialStart).count();
    }
    m_exp2CurrentTrial.userPlaneRotationDeg = planeSpinDeg;
    m_exp2CurrentTrial.userInPlaneAngleDeg = inPlaneAngleDeg;
    m_exp2CurrentTrial.durationSeconds = durationSeconds;
    m_exp2Results.push_back(m_exp2CurrentTrial);
    if (m_exp2CsvFileReady) {
        writeExp2TrialToCSV(m_exp2CurrentTrial);
    }

    m_exp2TrialActive = false;

    int groupIdx = getExp2GroupIndexForTrial(m_exp2CurrentTrial.trialNumber);
    int trialInGroup = getExp2TrialOffsetInGroup(m_exp2CurrentTrial.trialNumber);
    int groupSize = getExp2GroupSize(groupIdx);
    int nextInGroup = trialInGroup + 1;

    ensureExp2GroupProgressSize();
    if (groupIdx >= 0 && groupIdx < static_cast<int>(m_exp2GroupProgress.size())) {
        if (nextInGroup > m_exp2GroupProgress[groupIdx]) {
            m_exp2GroupProgress[groupIdx] = nextInGroup;
        }
    }

    // Update current group pointer
    m_exp2CurrentGroup = groupIdx;

    if (!hasNextTrialExp2()) {
        m_exp2State = ExperimentState::EXPERIMENT_COMPLETE;
        m_exp2CurrentTrialIndex = static_cast<int>(m_exp2Sequence.size());
    }
    else {
        m_exp2State = ExperimentState::TRIAL_COMPLETE;
        m_exp2NeedBreak = (nextInGroup >= groupSize && groupSize > 0);

        int nextGroup = m_exp2CurrentGroup;
        if (nextInGroup >= groupSize) {
            nextGroup = findNextExp2GroupWithRemaining(m_exp2CurrentGroup + 1);
        }
        if (nextGroup < 0) {
            m_exp2State = ExperimentState::EXPERIMENT_COMPLETE;
            m_exp2CurrentTrialIndex = static_cast<int>(m_exp2Sequence.size());
        }
        else {
            m_exp2CurrentGroup = nextGroup;
            int nextStart = getExp2GroupStartIndex(m_exp2CurrentGroup);
            int nextOffset = (m_exp2CurrentGroup < static_cast<int>(m_exp2GroupProgress.size()))
                ? m_exp2GroupProgress[m_exp2CurrentGroup]
                : 0;
            m_exp2CurrentTrialIndex = nextStart + nextOffset;
        }
    }
    saveProgress();
}

void UserStudyManager::recordExp2UserAngle(double userAngleDeg, double durationSeconds)
{
    recordExp2UserPose(0.0, userAngleDeg, durationSeconds);
}

// Legacy shim to keep existing Soft_gripper.cpp building while Exp2 UI is refactored.
void UserStudyManager::recordExp2UserAngles(double userAngleX, double userAngleZ, double durationSeconds)
{
    recordExp2UserPose(0.0, userAngleX, durationSeconds);
}

double UserStudyManager::getExp2TargetPlaneSpinDeg() const
{
    return m_exp2CurrentTrial.targetPlaneRotationDeg;
}

double UserStudyManager::getExp2TargetAngleDeg() const
{
    return m_exp2CurrentTrial.targetInPlaneAngleDeg;
}

// Legacy shims
double UserStudyManager::getExp2TargetAngleX() const
{
    return m_exp2CurrentTrial.targetInPlaneAngleDeg;
}

double UserStudyManager::getExp2TargetAngleZ() const
{
    return 0.0;
}

int UserStudyManager::getExp2ExpectedTrialCount() const
{
    return static_cast<int>(getExp2ExpectedTrialCountInternal());
}

int UserStudyManager::getExp2TrialsPerGroup(int groupIdx) const
{
    return getExp2GroupSize(groupIdx);
}

int UserStudyManager::getExp2GroupCount() const
{
    if (m_exp2GroupSizes.empty()) {
        return static_cast<int>(getDefaultExp2GroupSizes().size());
    }
    return static_cast<int>(m_exp2GroupSizes.size());
}

int UserStudyManager::getExp2GroupStartIndex(int groupIdx) const
{
    if (groupIdx <= 0) return 0;
    int start = 0;
    for (int i = 0; i < groupIdx && i < getExp2GroupCount(); ++i) {
        start += getExp2GroupSize(i);
    }
    return start;
}

int UserStudyManager::getExp2GroupIndexForTrial(int flatTrialIndex) const
{
    if (flatTrialIndex < 0) return 0;
    int start = 0;
    for (int i = 0; i < getExp2GroupCount(); ++i) {
        int size = getExp2GroupSize(i);
        if (flatTrialIndex < start + size) {
            return i;
        }
        start += size;
    }
    return std::max(0, getExp2GroupCount() - 1);
}

int UserStudyManager::getExp2TrialOffsetInGroup(int flatTrialIndex) const
{
    if (flatTrialIndex < 0) return 0;
    int start = 0;
    for (int i = 0; i < getExp2GroupCount(); ++i) {
        int size = getExp2GroupSize(i);
        if (flatTrialIndex < start + size) {
            return flatTrialIndex - start;
        }
        start += size;
    }
    return 0;
}

int UserStudyManager::getExp2GroupSize(int groupIdx) const
{
    if (groupIdx < 0) return 0;
    if (groupIdx < static_cast<int>(m_exp2GroupSizes.size())) {
        return m_exp2GroupSizes[groupIdx];
    }

    auto defaults = getDefaultExp2GroupSizes();
    if (groupIdx < static_cast<int>(defaults.size())) {
        return defaults[groupIdx];
    }
    return 0;
}

int UserStudyManager::getExp2CompletedTrials() const
{
    int total = 0;
    int groupCount = getExp2GroupCount();
    for (int i = 0; i < groupCount; ++i) {
        total += std::min(getExp2GroupSize(i), (i < static_cast<int>(m_exp2GroupProgress.size())) ? m_exp2GroupProgress[i] : 0);
    }
    return total;
}

int UserStudyManager::getExp1CompletedTrials() const
{
    int total = 0;
    size_t groupCount = m_groupProgress.size();
    for (size_t i = 0; i < groupCount; ++i) {
        int maxTrials = (i < m_experimentSequence.size()) ? static_cast<int>(m_experimentSequence[i].trials.size()) : 110;
        total += std::min(maxTrials, m_groupProgress[i]);
    }
    return total;
}
void UserStudyManager::createOrLoadUserCSV()
{
    std::string csvFilename = getUserCSVFilename();

    std::cout << "[UserStudy] CSV filename: " << csvFilename << std::endl;

    // ????????????
    std::ifstream checkFile(csvFilename);
    bool fileExists = false;
    if (checkFile.is_open()) {
        checkFile.seekg(0, std::ios::end);
        fileExists = (checkFile.tellg() > 0);
        checkFile.close();
        std::cout << "[UserStudy] File exists and has content: " << (fileExists ? "YES" : "NO") << std::endl;
    }
    else {
        std::cout << "[UserStudy] File does not exist." << std::endl;
    }

    if (fileExists) {
        std::cout << "[UserStudy] Found existing CSV file with content: " << csvFilename << std::endl;
        std::cout << "[UserStudy] Will continue appending data to existing file." << std::endl;
    }
    else {
        std::cout << "[UserStudy] Creating new CSV file: " << csvFilename << std::endl;

        // ?????????????
        std::ofstream newFile(csvFilename, std::ios::out);
        if (newFile.is_open()) {
            // ? ??:?????
            std::string header = "TrialNumber,Mode,Direction,ReferenceStiffness,ComparisonStiffness,UserChoice,ReactionTime,Polarity";
            newFile << header << std::endl;
            newFile.flush();

            std::cout << "[UserStudy] Header written to file." << std::endl;

            newFile.close();

            // ????
            std::ifstream verifyFile(csvFilename);
            if (verifyFile.is_open()) {
                std::string firstLine;
                std::getline(verifyFile, firstLine);
                verifyFile.close();

                if (firstLine.empty()) {
                    std::cerr << "[UserStudy] ERROR: Header was not written!" << std::endl;
                    m_csvFileReady = false;
                    return;
                }
                else {
                    std::cout << "[UserStudy] Verification successful. First line: " << firstLine << std::endl;
                }
            }

            std::cout << "[UserStudy] CSV file created successfully." << std::endl;
        }
        else {
            std::cerr << "[UserStudy] ERROR: Could not create CSV file!" << std::endl;
            m_csvFileReady = false;
            return;
        }
    }

    m_csvFileReady = true;
}

void UserStudyManager::writeTrialToCSV(const TrialResult& trial)
{
    std::string csvFilename = getUserCSVFilename();
    std::ofstream csvFile(csvFilename, std::ios::app);  // ????

    if (!csvFile.is_open()) {
        std::cerr << "[UserStudy] ERROR: Could not open CSV file for writing!" << std::endl;
        return;
    }

    // ????????????
    int groupIdx = trial.trialNumber / 110;
    if (groupIdx < m_experimentSequence.size()) {
        const auto& group = m_experimentSequence[groupIdx];

        // ? ??:???CSV?,??UserID?GroupNumber?TrialInGroup?Timestamp
        csvFile << trial.trialNumber << ","
            << (int)group.mode << ","
            << (int)group.direction << ","
            << std::fixed << std::setprecision(2) << trial.referenceStiffness << ","
            << std::fixed << std::setprecision(2) << trial.comparisonStiffness << ","
            << trial.userChoice << ","
            << std::fixed << std::setprecision(4) << trial.reactionTime << ","
            << polarityToString(trial.polarity) << std::endl;

        csvFile.flush();
        std::cout << "[UserStudy] Trial data written to CSV file." << std::endl;
    }

    csvFile.close();
}

void UserStudyManager::createOrLoadUserCSVExp2()
{
    std::string csvFilename = getUserExp2CSVFilename();
    std::ifstream check(csvFilename);
    bool exists = false;
    if (check.is_open()) {
        check.seekg(0, std::ios::end);
        exists = (check.tellg() > 0);
        check.close();
    }

    if (!exists) {
        std::ofstream file(csvFilename, std::ios::out);
        if (!file.is_open()) {
            std::cerr << "[UserStudy][Exp2] ERROR: Cannot create CSV file!\n";
            m_exp2CsvFileReady = false;
            return;
        }
        file << "TrialNumber,GroupIndex,TrialInGroup,Plane,TargetPlaneSpinDeg,TargetAngleDeg,UserPlaneSpinDeg,UserAngleDeg,DurationSeconds\n";
        file.close();
    }

    m_exp2CsvFileReady = true;
}

void UserStudyManager::writeExp2TrialToCSV(const AngleTrialResult& trial)
{
    if (!m_exp2CsvFileReady) return;
    std::ofstream file(getUserExp2CSVFilename(), std::ios::app);
    if (!file.is_open()) {
        std::cerr << "[UserStudy][Exp2] ERROR: Cannot open CSV for writing\n";
        return;
    }
    int groupIdx = getExp2GroupIndexForTrial(trial.trialNumber);
    int trialInGroup = getExp2TrialOffsetInGroup(trial.trialNumber);
    file << (trial.trialNumber + 1) << ","
        << (groupIdx + 1) << ","
        << (trialInGroup + 1) << ","
        << static_cast<int>(trial.plane) << ","
        << std::fixed << std::setprecision(2) << trial.targetPlaneRotationDeg << ","
        << trial.targetInPlaneAngleDeg << ","
        << trial.userPlaneRotationDeg << ","
        << trial.userInPlaneAngleDeg << ","
        << std::fixed << std::setprecision(3) << trial.durationSeconds << "\n";
    file.close();
}
