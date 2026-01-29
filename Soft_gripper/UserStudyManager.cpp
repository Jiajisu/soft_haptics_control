#include "UserStudyManager.hpp"
#include <algorithm>
#include <random>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <map>
#include <cctype>

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

const char* kExp1MethodTag = "STAIRCASE_V2";
constexpr double kDefaultStartKComp = 160.0;
constexpr double kDefaultMinKComp = 100.0;
constexpr double kDefaultMaxKComp = 200.0;
constexpr double kDefaultStartStep = 10.0;
constexpr double kDefaultFineStep = 5.0;
constexpr int kDefaultSwitchAfterReversals = 2;
constexpr int kDefaultTargetReversals = 10;
}

AdaptiveStaircase::AdaptiveStaircase(
    double kRef,
    double startKComp,
    double minKComp,
    double maxKComp,
    double startStep,
    double fineStep,
    int switchAfterReversals,
    int targetReversals)
    : m_kRef(kRef)
    , m_minKComp(minKComp)
    , m_maxKComp(maxKComp)
    , m_startStep(startStep)
    , m_fineStep(fineStep)
    , m_switchAfterReversals(switchAfterReversals)
    , m_targetReversals(targetReversals)
{
    m_state.currentKComp = clampK(startKComp);
    m_state.currentStep = startStep;
    m_state.reversalCount = 0;
    m_state.trialCount = 0;
    m_state.correctStreak = 0;
    m_state.lastMoveDir = 0;
    refreshStep();
}

double AdaptiveStaircase::clampK(double k) const
{
    if (k < m_minKComp) return m_minKComp;
    if (k > m_maxKComp) return m_maxKComp;
    return k;
}

void AdaptiveStaircase::refreshStep()
{
    m_state.currentStep = (m_state.reversalCount >= m_switchAfterReversals) ? m_fineStep : m_startStep;
}

double AdaptiveStaircase::getNextKComp() const
{
    return m_state.currentKComp;
}

double AdaptiveStaircase::getDeltaK() const
{
    return m_state.currentKComp - m_kRef;
}

double AdaptiveStaircase::getStepSize() const
{
    return (m_state.reversalCount >= m_switchAfterReversals) ? m_fineStep : m_startStep;
}

void AdaptiveStaircase::setState(const StaircaseState& state)
{
    m_state = state;
    if (m_state.trialCount < 0) m_state.trialCount = 0;
    if (m_state.reversalCount < 0) m_state.reversalCount = 0;
    m_state.currentKComp = clampK(m_state.currentKComp);
    refreshStep();
}

void AdaptiveStaircase::update(bool isCorrect)
{
    refreshStep();
    const double step = m_state.currentStep;
    const double prevK = m_state.currentKComp;
    int moveDir = 0;

    if (!isCorrect) {
        m_state.correctStreak = 0;
        double nextK = clampK(prevK + step);
        if (nextK != prevK) {
            moveDir = 1;
        }
        m_state.currentKComp = nextK;
    }
    else {
        m_state.correctStreak++;
        if (m_state.correctStreak >= 2) {
            double nextK = clampK(prevK - step);
            if (nextK != prevK) {
                moveDir = -1;
            }
            m_state.currentKComp = nextK;
            m_state.correctStreak = 0;
        }
    }

    if (moveDir != 0) {
        if (m_state.lastMoveDir != 0 && m_state.lastMoveDir != moveDir) {
            ++m_state.reversalCount;
        }
        m_state.lastMoveDir = moveDir;
    }

    ++m_state.trialCount;
    refreshStep();
}

bool AdaptiveStaircase::shouldStop() const
{
    return m_state.reversalCount >= m_targetReversals;
}

UserStudyManager::UserStudyManager()
    : m_trialActive(false)
    , m_userId("")
    , m_currentMode(InteractionMode::FORCE_TO_POSITION)
    , m_currentDirection(FeedbackDirection::LATERAL_X)
    , m_currentTrialGroup(0)
    , m_currentTrialInGroup(0)
    , m_currentPolarity(SurfacePolarity::POSITIVE)
    , m_groupProgress()
    , m_referenceStiffness(100.0)
    , m_comparisonStiffnesses()
    , m_results()
    , m_currentTrial()
    , m_experimentSequence()
    , m_sequenceLoaded(false)
    , m_staircases()
    , m_currentStaircaseId(0)
    , m_currentReferenceOnLeft(true)
    , m_pendingReferenceOnLeft(true)
    , m_pendingPolarity(SurfacePolarity::POSITIVE)
    , m_hasPendingTrialConfig(false)
    , m_pendingGroupIdx(-1)
    , m_startKComp(kDefaultStartKComp)
    , m_minKComp(kDefaultMinKComp)
    , m_maxKComp(kDefaultMaxKComp)
    , m_startStep(kDefaultStartStep)
    , m_fineStep(kDefaultFineStep)
    , m_switchAfterReversals(kDefaultSwitchAfterReversals)
    , m_targetReversals(kDefaultTargetReversals)
    , m_rng(std::random_device{}())
    , m_csvFileReady(false)
    , m_exp2CsvFileReady(false)
    , m_experimentState(ExperimentState::NOT_STARTED)
    , m_exp2State(ExperimentState::NOT_STARTED)
    , m_exp2Sequence()
    , m_exp2SequenceLoaded(false)
    , m_exp2GroupSizes()
    , m_exp2CurrentTrialIndex(0)
    , m_exp2CurrentGroup(0)
    , m_exp2TrialActive(false)
    , m_exp2NeedBreak(false)
    , m_exp2GroupProgress()
    , m_exp2CurrentTrial()
    , m_exp2Results()
    , m_exp2TrialStart()
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
    generateComparisonStiffnesses();
    initializeStaircases();

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
    syncGroupProgressFromStaircases();

    if (loadProgress(getProgressFilename())) {
        StaircaseState st = getStaircaseStateForGroup(m_currentTrialGroup);
        std::cout << "[UserStudy] Progress loaded: Exp1 Group "
            << (m_currentTrialGroup + 1) << "/6 Trial "
            << (m_currentTrialInGroup + 1)
            << " (reversals " << st.reversalCount << "/" << m_targetReversals << ")" << std::endl;
    }
    else {
        std::cout << "[UserStudy] No progress file found. Starting fresh." << std::endl;
        m_currentTrialGroup = 0;
        m_currentTrialInGroup = 0;
        m_experimentState = ExperimentState::READY;
        m_exp2CurrentTrialIndex = 0;
        m_exp2CurrentGroup = 0;
        m_exp2State = ExperimentState::READY;
        syncGroupProgressFromStaircases();
        saveProgress();
    }

    int nextExp1Group = findNextExp1GroupWithRemaining(m_currentTrialGroup);
    if (nextExp1Group < 0) {
        m_experimentState = ExperimentState::EXPERIMENT_COMPLETE;
    }
    else {
        m_currentTrialGroup = nextExp1Group;
        if (m_currentTrialGroup < static_cast<int>(m_groupProgress.size())) {
            m_currentTrialInGroup = m_groupProgress[m_currentTrialGroup];
        }
        m_currentMode = m_experimentSequence[m_currentTrialGroup].mode;
        m_currentDirection = m_experimentSequence[m_currentTrialGroup].direction;
        if (m_experimentState != ExperimentState::IN_PROGRESS) {
            m_experimentState = ExperimentState::READY;
        }
    }
    sanitizeProgressIndices();
    m_trialActive = false;

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
        group.trials.clear();
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

    file << "# Experiment Sequence for User: " << m_userId << "\n";
    file << "Exp1Method " << kExp1MethodTag << "\n";
    file << "Exp1StaircaseParams "
        << "kRef=" << m_referenceStiffness << " "
        << "startKComp=" << m_startKComp << " "
        << "minKComp=" << m_minKComp << " "
        << "maxKComp=" << m_maxKComp << " "
        << "step" << m_startStep << "_to" << m_fineStep << "_atRev=" << m_switchAfterReversals << " "
        << "targetRev=" << m_targetReversals << "\n";
    file << "# GroupIndex Mode Direction\n";

    for (size_t i = 0; i < m_experimentSequence.size(); ++i) {
        const auto& group = m_experimentSequence[i];
        file << "Group " << i << " " << static_cast<int>(group.mode) << " " << static_cast<int>(group.direction) << "\n";
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
    bool methodOk = false;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream iss(line);
        std::string key;
        iss >> key;

        if (key == "Exp1Method") {
            std::string method;
            iss >> method;
            if (method == kExp1MethodTag) {
                methodOk = true;
            }
            else {
                std::cout << "[UserStudy] Unsupported Exp1Method in sequence: " << method << std::endl;
                return false;
            }
        }
        else if (key == "Exp1StaircaseParams") {
            std::string token;
            while (iss >> token) {
                auto pos = token.find('=');
                if (pos == std::string::npos) continue;
                std::string name = token.substr(0, pos);
                std::string value = token.substr(pos + 1);
                try {
                    if (name == "kRef") {
                        m_referenceStiffness = std::stod(value);
                    }
                    else if (name == "startKComp") {
                        m_startKComp = std::stod(value);
                    }
                    else if (name == "minKComp") {
                        m_minKComp = std::stod(value);
                    }
                    else if (name == "maxKComp") {
                        m_maxKComp = std::stod(value);
                    }
                    else if (name.rfind("step", 0) == 0) {
                        m_switchAfterReversals = std::stoi(value);
                        auto toPos = name.find("_to");
                        auto atPos = name.find("_atRev");
                        if (toPos != std::string::npos) {
                            std::string startStr = name.substr(4, toPos - 4);
                            double startVal = std::stod(startStr);
                            if (atPos != std::string::npos && atPos > toPos + 3) {
                                std::string fineStr = name.substr(toPos + 3, atPos - (toPos + 3));
                                double fineVal = std::stod(fineStr);
                                m_startStep = startVal;
                                m_fineStep = fineVal;
                            }
                            else {
                                m_startStep = startVal;
                            }
                        }
                    }
                    else if (name == "targetRev") {
                        m_targetReversals = std::stoi(value);
                    }
                }
                catch (const std::exception&) {
                    std::cout << "[UserStudy] Warning: could not parse staircase param token '" << token << "'" << std::endl;
                }
            }
        }
        else if (key == "Group") {
            int idx = 0;
            int mode = 0;
            int dir = 0;
            if (iss >> idx >> mode >> dir) {
                GroupConfig group;
                group.mode = static_cast<InteractionMode>(mode);
                group.direction = static_cast<FeedbackDirection>(dir);
                group.trials.clear();
                m_experimentSequence.push_back(group);
            }
        }
    }

    file.close();

    if (!methodOk) {
        std::cerr << "[UserStudy] Sequence file missing Exp1Method tag or not STAIRCASE_V2." << std::endl;
        return false;
    }

    if (m_experimentSequence.size() != 6) {
        std::cerr << "[UserStudy] ERROR: Expected 6 groups, got " << m_experimentSequence.size() << std::endl;
        m_experimentSequence.clear();
        return false;
    }

    m_sequenceLoaded = true;
    return true;
}

void UserStudyManager::initializeStaircases()
{
    m_staircases.clear();
    if (m_experimentSequence.empty()) return;
    double minK = std::max(m_minKComp, m_referenceStiffness);
    double maxK = std::max(minK, m_maxKComp);
    double startK = std::min(std::max(m_startKComp, minK), maxK);

    for (size_t i = 0; i < m_experimentSequence.size(); ++i) {
        m_staircases.emplace_back(
            m_referenceStiffness,
            startK,
            minK,
            maxK,
            m_startStep,
            m_fineStep,
            m_switchAfterReversals,
            m_targetReversals);
    }
    syncGroupProgressFromStaircases();
    m_hasPendingTrialConfig = false;
    m_pendingGroupIdx = -1;
}

int UserStudyManager::selectStaircaseId(int /*groupIdx*/) const
{
    return 0;
}

StaircaseState UserStudyManager::getStaircaseStateForGroup(int groupIdx) const
{
    if (groupIdx >= 0 && groupIdx < static_cast<int>(m_staircases.size())) {
        return m_staircases[groupIdx].getState();
    }
    return StaircaseState{};
}

void UserStudyManager::setStaircaseStateForGroup(int groupIdx, const StaircaseState& state)
{
    if (groupIdx >= 0 && groupIdx < static_cast<int>(m_staircases.size())) {
        m_staircases[groupIdx].setState(state);
    }
}

void UserStudyManager::syncGroupProgressFromStaircases()
{
    size_t groupCount = m_experimentSequence.size();
    if (m_groupProgress.size() < groupCount) {
        m_groupProgress.resize(groupCount, 0);
    }
    for (size_t i = 0; i < groupCount; ++i) {
        if (i < m_staircases.size()) {
            m_groupProgress[i] = m_staircases[i].getState().trialCount;
        }
    }
}

void UserStudyManager::randomizeNextTrialConfig(int groupIdx, SurfacePolarity& polarityOut, bool& referenceOnLeftOut)
{
    std::uniform_int_distribution<int> coin(0, 1);
    referenceOnLeftOut = (coin(m_rng) == 1);
    polarityOut = (coin(m_rng) == 1) ? SurfacePolarity::POSITIVE : SurfacePolarity::NEGATIVE;
    m_pendingGroupIdx = groupIdx;
    m_pendingReferenceOnLeft = referenceOnLeftOut;
    m_pendingPolarity = polarityOut;
    m_hasPendingTrialConfig = true;
}

int UserStudyManager::findNextExp1GroupWithRemaining(int startGroup) const
{
    int groupCount = static_cast<int>(m_experimentSequence.size());
    if (groupCount <= 0 || m_staircases.size() < m_experimentSequence.size()) {
        return -1;
    }
    if (startGroup < 0) startGroup = 0;
    for (int offset = 0; offset < groupCount; ++offset) {
        int idx = (startGroup + offset) % groupCount;
        if (idx < 0 || idx >= groupCount) continue;
        if (!m_staircases[idx].shouldStop()) {
            return idx;
        }
    }
    return -1;
}

bool UserStudyManager::isGroupIndexComplete(int groupIdx) const
{
    if (groupIdx < 0 || groupIdx >= static_cast<int>(m_staircases.size())) {
        return false;
    }
    return m_staircases[groupIdx].shouldStop();
}

StaircaseState UserStudyManager::getCurrentStaircaseState() const
{
    return getStaircaseStateForGroup(m_currentTrialGroup);
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
    syncGroupProgressFromStaircases();

    file << "Exp1Method " << kExp1MethodTag << "\n";
    file << "Exp1CurrentGroup " << m_currentTrialGroup << "\n";
    file << "Exp1CurrentTrialInGroup " << m_currentTrialInGroup << "\n";
    file << "Exp1State " << static_cast<int>(m_experimentState) << "\n";
    file << "Exp1GroupProgress";
    for (int v : m_groupProgress) file << " " << v;
    file << "\n";
    file << "Exp1StaircaseState\n";
    for (size_t i = 0; i < m_staircases.size(); ++i) {
        const auto state = m_staircases[i].getState();
        file << "g" << i << " "
            << state.currentKComp << " "
            << state.currentStep << " "
            << state.reversalCount << " "
            << state.trialCount << " "
            << state.correctStreak << " "
            << state.lastMoveDir << "\n";
    }

    file << "Exp2CurrentTrial " << m_exp2CurrentTrialIndex << "\n";
    file << "Exp2CurrentGroup " << m_exp2CurrentGroup << "\n";
    file << "Exp2State " << static_cast<int>(m_exp2State) << "\n";
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

    if (m_staircases.size() < m_experimentSequence.size()) {
        initializeStaircases();
    }

    // defaults
    m_exp2CurrentTrialIndex = 0;
    m_exp2State = ExperimentState::READY;
    m_exp2NeedBreak = false;
    m_exp2TrialActive = false;
    m_groupProgress.clear();
    m_exp2GroupProgress.clear();
    m_currentTrialGroup = 0;
    m_currentTrialInGroup = 0;
    m_experimentState = ExperimentState::READY;
    m_trialActive = false;
    int exp2CurrentTrialFlat = -1;
    bool exp1MethodOk = false;

    std::vector<StaircaseState> loadedStates(m_staircases.size());
    for (size_t i = 0; i < loadedStates.size(); ++i) {
        loadedStates[i] = m_staircases[i].getState();
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::string key;
        if (iss >> key) {
            if (key == "Exp1Method") {
                std::string method;
                iss >> method;
                if (method == kExp1MethodTag) {
                    exp1MethodOk = true;
                }
            }
            else if (key == "Exp1CurrentGroup") {
                iss >> m_currentTrialGroup;
            }
            else if (key == "Exp1CurrentTrialInGroup") {
                iss >> m_currentTrialInGroup;
            }
            else if (key == "Exp1State") {
                int stateInt = 0;
                iss >> stateInt;
                if (stateInt < 0 || stateInt > static_cast<int>(ExperimentState::EXPERIMENT_COMPLETE)) stateInt = 0;
                m_experimentState = static_cast<ExperimentState>(stateInt);
            }
            else if (key == "Exp1GroupProgress") {
                int v;
                while (iss >> v) {
                    m_groupProgress.push_back(v);
                }
            }
            else if (key == "Exp1StaircaseState") {
                // header line; actual states parsed below
            }
            else if (!key.empty() && key[0] == 'g' && key.size() > 1 && std::isdigit(static_cast<unsigned char>(key[1]))) {
                int idx = std::stoi(key.substr(1));
                StaircaseState state = {};
                iss >> state.currentKComp >> state.currentStep >> state.reversalCount
                    >> state.trialCount >> state.correctStreak >> state.lastMoveDir;
                if (idx >= 0 && idx < static_cast<int>(loadedStates.size())) {
                    loadedStates[idx] = state;
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
        }
    }

    file.close();
    if (!exp1MethodOk) {
        std::cout << "[UserStudy] Progress file missing Exp1Method or incompatible version." << std::endl;
        return false;
    }

    if (m_groupProgress.empty() && !m_experimentSequence.empty()) {
        m_groupProgress.resize(m_experimentSequence.size(), 0);
    }
    ensureExp1GroupProgressSize();
    for (size_t i = 0; i < loadedStates.size(); ++i) {
        setStaircaseStateForGroup(static_cast<int>(i), loadedStates[i]);
    }
    syncGroupProgressFromStaircases();

    if (m_currentTrialGroup >= 0 && m_currentTrialGroup < static_cast<int>(m_groupProgress.size())) {
        if (m_currentTrialInGroup > m_groupProgress[m_currentTrialGroup]) {
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
    ensureExp2GroupProgressSize();
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
    m_currentStaircaseId = selectStaircaseId(m_currentTrialGroup);
    sanitizeProgressIndices();
    return true;
}

// ?? startNextTrial ??
void UserStudyManager::startNextTrial()
{
    if (m_experimentState == ExperimentState::EXPERIMENT_COMPLETE) {
        std::cout << "[UserStudy] Experiment already complete!" << std::endl;
        return;
    }

    if (m_experimentState == ExperimentState::IN_PROGRESS) {
        std::cout << "[UserStudy] Trial already in progress! Make a choice first." << std::endl;
        return;
    }

    if (!m_sequenceLoaded || m_experimentSequence.empty() || m_staircases.size() < m_experimentSequence.size()) {
        std::cout << "[UserStudy] No sequence or staircase loaded!" << std::endl;
        return;
    }

    sanitizeProgressIndices();
    syncGroupProgressFromStaircases();

    int targetGroup = findNextExp1GroupWithRemaining(m_currentTrialGroup);
    if (targetGroup < 0) {
        m_experimentState = ExperimentState::EXPERIMENT_COMPLETE;
        std::cout << "[UserStudy] Experiment Complete! Thank you for your participation." << std::endl;
        saveResults("user_" + m_userId + "_final_results.csv");
        return;
    }

    m_currentTrialGroup = targetGroup;
    m_currentMode = m_experimentSequence[m_currentTrialGroup].mode;
    m_currentDirection = m_experimentSequence[m_currentTrialGroup].direction;
    m_currentStaircaseId = selectStaircaseId(m_currentTrialGroup);

    AdaptiveStaircase& staircase = m_staircases[m_currentTrialGroup];
    StaircaseState stairState = staircase.getState();
    m_currentTrialInGroup = stairState.trialCount;

    if (!m_hasPendingTrialConfig || m_pendingGroupIdx != m_currentTrialGroup) {
        randomizeNextTrialConfig(m_currentTrialGroup, m_pendingPolarity, m_pendingReferenceOnLeft);
    }
    m_currentReferenceOnLeft = m_pendingReferenceOnLeft;
    m_currentPolarity = m_pendingPolarity;
    m_hasPendingTrialConfig = false;
    m_pendingGroupIdx = -1;

    double comparisonK = staircase.getNextKComp();

    m_currentTrial = TrialResult{};
    m_currentTrial.trialNumber = getExp1CompletedTrials();
    m_currentTrial.groupIndex = m_currentTrialGroup;
    m_currentTrial.trialInGroup = m_currentTrialInGroup;
    m_currentTrial.referenceStiffness = m_referenceStiffness;
    m_currentTrial.comparisonStiffness = comparisonK;
    m_currentTrial.userChoice = -1;
    m_currentTrial.correctChoice = m_currentReferenceOnLeft ? 1 : 0;
    m_currentTrial.isCorrect = false;
    m_currentTrial.reactionTime = 0.0;
    m_currentTrial.polarity = m_currentPolarity;
    m_currentTrial.referenceOnLeft = m_currentReferenceOnLeft;
    m_currentTrial.staircaseDeltaK = comparisonK - m_referenceStiffness;
    m_currentTrial.staircaseStep = staircase.getStepSize();
    m_currentTrial.staircaseReversals = stairState.reversalCount;
    m_currentTrial.staircaseTrialInGroup = stairState.trialCount + 1;
    m_currentTrial.staircaseId = m_currentStaircaseId;
    m_currentTrial.trialStartTime = std::chrono::steady_clock::now();

    m_trialActive = true;
    m_experimentState = ExperimentState::IN_PROGRESS;

    std::cout << "\n[UserStudy] === NEW TRIAL ===" << std::endl;
    std::cout << "Group " << (m_currentTrialGroup + 1) << "/6"
        << " | Trial " << (m_currentTrialInGroup + 1)
        << " | Mode: " << ((int)m_currentMode == 0 ? "F-P" : "F-F")
        << " | Direction: ";

    switch (m_currentDirection) {
    case FeedbackDirection::LATERAL_X: std::cout << "X"; break;
    case FeedbackDirection::LATERAL_Y: std::cout << "Y"; break;
    case FeedbackDirection::VERTICAL_Z: std::cout << "Z"; break;
    }

    std::cout << " | Polarity: "
        << ((m_currentPolarity == SurfacePolarity::POSITIVE) ? "+ (Red)" : "- (Blue)") << std::endl;

    std::cout << "RefOnLeft: " << (m_currentReferenceOnLeft ? "Yes" : "No")
        << " | Ref: " << m_currentTrial.referenceStiffness
        << " | Comp: " << m_currentTrial.comparisonStiffness
        << " | DeltaK: " << std::fixed << std::setprecision(2) << m_currentTrial.staircaseDeltaK
        << " | Step: " << m_currentTrial.staircaseStep
        << " | Reversals: " << stairState.reversalCount << "/" << m_targetReversals
        << " | CorrectAnswer: " << (m_currentTrial.correctChoice == 0 ? 1 : 2)
        << " (1=Left, 2=Right)" << std::endl;
}

bool UserStudyManager::peekNextTrialSurface(FeedbackDirection& direction, SurfacePolarity& polarity)
{
    if (!hasNextTrial()) {
        return false;
    }

    int targetGroup = findNextExp1GroupWithRemaining(m_currentTrialGroup);
    if (targetGroup < 0 || targetGroup >= static_cast<int>(m_experimentSequence.size())) {
        return false;
    }

    direction = m_experimentSequence[targetGroup].direction;

    if (!m_hasPendingTrialConfig || m_pendingGroupIdx != targetGroup) {
        randomizeNextTrialConfig(targetGroup, m_pendingPolarity, m_pendingReferenceOnLeft);
    }
    polarity = m_pendingPolarity;
    return true;
}

// ?? recordUserChoice ??
void UserStudyManager::recordUserChoice(int choice)
{
    if (!m_trialActive || m_experimentState != ExperimentState::IN_PROGRESS) {
        std::cout << "[UserStudy] No active trial to record choice!" << std::endl;
        return;
    }

    StaircaseState postState = getCurrentStaircaseState();

    m_currentTrial.choiceTime = std::chrono::steady_clock::now();
    m_currentTrial.userChoice = choice;

    m_currentTrial.reactionTime =
        std::chrono::duration<double>(m_currentTrial.choiceTime - m_currentTrial.trialStartTime).count();

    m_currentTrial.correctChoice = m_currentReferenceOnLeft ? 1 : 0;
    m_currentTrial.isCorrect = (choice == m_currentTrial.correctChoice);

    ensureExp1GroupProgressSize();
    if (m_currentTrialGroup >= 0 && m_currentTrialGroup < static_cast<int>(m_staircases.size())) {
        AdaptiveStaircase& staircase = m_staircases[m_currentTrialGroup];
        staircase.update(m_currentTrial.isCorrect);
        postState = staircase.getState();
        if (m_currentTrialGroup < static_cast<int>(m_groupProgress.size())) {
            m_groupProgress[m_currentTrialGroup] = postState.trialCount;
        }
        m_currentTrialInGroup = postState.trialCount;
    }

    m_results.push_back(m_currentTrial);
    m_trialActive = false;

    if (m_csvFileReady) {
        writeTrialToCSV(m_currentTrial);
    }

    std::cout << "[UserStudy] Choice recorded: " << (choice == 0 ? "Left" : "Right")
        << " | Correct choice: " << (m_currentTrial.correctChoice == 0 ? "Left" : "Right")
        << " | IsCorrect: " << (m_currentTrial.isCorrect ? "Yes" : "No")
        << " | Reaction Time: " << std::fixed << std::setprecision(3)
        << m_currentTrial.reactionTime << "s" << std::endl;

    if (isGroupIndexComplete(m_currentTrialGroup)) {
        m_experimentState = ExperimentState::GROUP_COMPLETE;
        std::cout << "\n[UserStudy] >>> GROUP " << (m_currentTrialGroup + 1)
            << " COMPLETE (reversals " << postState.reversalCount
            << "/" << m_targetReversals << ") <<<" << std::endl;
        int nextGroup = findNextExp1GroupWithRemaining(m_currentTrialGroup + 1);
        if (nextGroup < 0) {
            m_experimentState = ExperimentState::EXPERIMENT_COMPLETE;
            std::cout << "\n*** EXPERIMENT COMPLETE! ***" << std::endl;
            std::cout << "Thank you for your participation!" << std::endl;
            saveResults("user_" + m_userId + "_final_results.csv");
        }
        else {
            m_currentTrialGroup = nextGroup;
            m_currentMode = m_experimentSequence[m_currentTrialGroup].mode;
            m_currentDirection = m_experimentSequence[m_currentTrialGroup].direction;
            m_currentStaircaseId = selectStaircaseId(m_currentTrialGroup);
            m_currentPolarity = SurfacePolarity::POSITIVE;
            if (m_currentTrialGroup < static_cast<int>(m_groupProgress.size())) {
                m_currentTrialInGroup = m_groupProgress[m_currentTrialGroup];
            }
            else {
                m_currentTrialInGroup = getStaircaseStateForGroup(m_currentTrialGroup).trialCount;
            }
            m_hasPendingTrialConfig = false;
            m_pendingGroupIdx = -1;
            std::cout << "Press 'T' when ready to continue to the next group." << std::endl;
        }
    }
    else {
        m_experimentState = ExperimentState::TRIAL_COMPLETE;
        std::cout << "Press 'T' for next trial." << std::endl;
    }

    // ????
    saveProgress();
}

// ????????...
double UserStudyManager::getCurrentReferenceStiffness() const
{
    return m_referenceStiffness;
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

    file << "TrialNumber,GroupIndex,TrialInGroup,Mode,Direction,ReferenceStiffness,ComparisonStiffness,"
        << "ReferenceOnLeft,CorrectChoice,IsCorrect,UserChoice,ReactionTime,Polarity,"
        << "StaircaseKComp,StaircaseDeltaK,StaircaseStep,StaircaseReversals,StaircaseTrialInGroup,StaircaseId\n";

    for (const auto& r : m_results) {
        int groupIdx = r.groupIndex;
        int modeVal = 0;
        int dirVal = 0;
        if (groupIdx >= 0 && groupIdx < static_cast<int>(m_experimentSequence.size())) {
            modeVal = static_cast<int>(m_experimentSequence[groupIdx].mode);
            dirVal = static_cast<int>(m_experimentSequence[groupIdx].direction);
        }
        file << (r.trialNumber + 1) << ','
            << (groupIdx + 1) << ','
            << r.staircaseTrialInGroup << ','
            << modeVal << ','
            << dirVal << ','
            << r.referenceStiffness << ','
            << r.comparisonStiffness << ','
            << (r.referenceOnLeft ? 1 : 0) << ','
            << r.correctChoice << ','
            << (r.isCorrect ? 1 : 0) << ','
            << r.userChoice << ','
            << r.reactionTime << ','
            << polarityToString(r.polarity) << ','
            << r.comparisonStiffness << ','
            << r.staircaseDeltaK << ','
            << r.staircaseStep << ','
            << r.staircaseReversals << ','
            << r.staircaseTrialInGroup << ','
            << r.staircaseId << '\n';
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
            StaircaseState st = getStaircaseStateForGroup(static_cast<int>(i));
            ensureExp1GroupProgressSize();
            if (i < m_groupProgress.size()) {
                m_groupProgress[i] = st.trialCount;
            }
            m_currentTrialInGroup = st.trialCount;
            m_currentMode = m;
            m_currentDirection = d;
            m_currentStaircaseId = selectStaircaseId(static_cast<int>(i));
            m_experimentState = isGroupIndexComplete(static_cast<int>(i)) ? ExperimentState::GROUP_COMPLETE : ExperimentState::READY;
            m_hasPendingTrialConfig = false;
            m_pendingGroupIdx = -1;
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
    double minK = std::max(m_minKComp, m_referenceStiffness);
    double maxK = std::max(minK, m_maxKComp);
    double startK = std::min(std::max(m_startKComp, minK), maxK);
    if (m_currentTrialGroup >= 0 && m_currentTrialGroup < static_cast<int>(m_staircases.size())) {
        m_staircases[m_currentTrialGroup] = AdaptiveStaircase(
            m_referenceStiffness,
            startK,
            minK,
            maxK,
            m_startStep,
            m_fineStep,
            m_switchAfterReversals,
            m_targetReversals);
    }
    m_hasPendingTrialConfig = false;
    m_pendingGroupIdx = -1;
    m_experimentState = ExperimentState::READY;
    syncGroupProgressFromStaircases();
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

    int trialsInGroup = (m_currentTrialGroup < static_cast<int>(m_groupProgress.size()))
        ? m_groupProgress[m_currentTrialGroup]
        : 0;
    if (trialsInGroup < 0) trialsInGroup = 0;

    if (m_currentTrialInGroup < 0 || m_currentTrialInGroup > trialsInGroup) {
        if (m_currentTrialInGroup < 0) {
            std::cout << "[UserStudy] WARNING: Invalid trial index " << m_currentTrialInGroup
                << " for group " << m_currentTrialGroup << ". Resetting to 0." << std::endl;
            m_currentTrialInGroup = 0;
        }
        else {
            m_currentTrialInGroup = trialsInGroup;
        }
    }
}

bool UserStudyManager::isGroupComplete() const
{
    return isGroupIndexComplete(m_currentTrialGroup);
}

void UserStudyManager::ensureExp1GroupProgressSize()
{
    size_t groupCount = m_experimentSequence.size();
    if (groupCount == 0) return;
    if (m_groupProgress.size() < groupCount) {
        m_groupProgress.resize(groupCount, 0);
    }
    for (size_t i = 0; i < groupCount; ++i) {
        if (m_groupProgress[i] < 0) m_groupProgress[i] = 0;
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

    if (m_experimentSequence.empty() || m_staircases.size() < m_experimentSequence.size()) {
        return false;
    }

    sanitizeProgressIndices();

    int nextGroup = findNextExp1GroupWithRemaining(m_currentTrialGroup);
    return nextGroup >= 0;
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
    m_exp2CurrentTrial.forceCueCount = 0;
    m_exp2CurrentTrial.trialStartTime = std::chrono::steady_clock::now();
    m_exp2TrialStart = m_exp2CurrentTrial.trialStartTime;
    m_exp2TrialActive = true;
    m_exp2State = ExperimentState::IN_PROGRESS;
    m_exp2NeedBreak = false;
}

void UserStudyManager::incrementExp2ForceCueCount()
{
    if (!m_exp2TrialActive) return;
    ++m_exp2CurrentTrial.forceCueCount;
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
    for (const auto& stair : m_staircases) {
        total += stair.getState().trialCount;
    }
    if (m_staircases.size() < m_groupProgress.size()) {
        for (size_t i = m_staircases.size(); i < m_groupProgress.size(); ++i) {
            if (m_groupProgress[i] > 0) {
                total += m_groupProgress[i];
            }
        }
    }
    return total;
}
void UserStudyManager::createOrLoadUserCSV()
{
    std::string csvFilename = getUserCSVFilename();
    const std::string header = "TrialNumber,GroupIndex,TrialInGroup,Mode,Direction,ReferenceStiffness,ComparisonStiffness,ReferenceOnLeft,CorrectChoice,IsCorrect,UserChoice,ReactionTime,Polarity,StaircaseKComp,StaircaseDeltaK,StaircaseStep,StaircaseReversals,StaircaseTrialInGroup,StaircaseId";

    bool hasCorrectHeader = false;
    std::ifstream checkFile(csvFilename);
    if (checkFile.is_open()) {
        std::string firstLine;
        std::getline(checkFile, firstLine);
        if (firstLine == header) {
            hasCorrectHeader = true;
        }
    }

    if (!hasCorrectHeader) {
        std::ofstream newFile(csvFilename, std::ios::out | std::ios::trunc);
        if (!newFile.is_open()) {
            std::cerr << "[UserStudy] ERROR: Could not create CSV file!" << std::endl;
            m_csvFileReady = false;
            return;
        }
        newFile << header << std::endl;
        newFile.close();
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

    int groupIdx = trial.groupIndex;
    int modeVal = 0;
    int dirVal = 0;
    if (groupIdx >= 0 && groupIdx < static_cast<int>(m_experimentSequence.size())) {
        modeVal = static_cast<int>(m_experimentSequence[groupIdx].mode);
        dirVal = static_cast<int>(m_experimentSequence[groupIdx].direction);
    }

    csvFile << (trial.trialNumber + 1) << ","
        << (groupIdx + 1) << ","
        << trial.staircaseTrialInGroup << ","
        << modeVal << ","
        << dirVal << ","
        << std::fixed << std::setprecision(2) << trial.referenceStiffness << ","
        << std::setprecision(2) << trial.comparisonStiffness << ","
        << (trial.referenceOnLeft ? 1 : 0) << ","
        << trial.correctChoice << ","
        << (trial.isCorrect ? 1 : 0) << ","
        << trial.userChoice << ","
        << std::setprecision(4) << trial.reactionTime << ","
        << polarityToString(trial.polarity) << ","
        << std::setprecision(2) << trial.comparisonStiffness << ","
        << std::setprecision(2) << trial.staircaseDeltaK << ","
        << std::setprecision(2) << trial.staircaseStep << ","
        << trial.staircaseReversals << ","
        << trial.staircaseTrialInGroup << ","
        << trial.staircaseId << std::endl;

    csvFile.flush();
    csvFile.close();
    std::cout << "[UserStudy] Trial data written to CSV file." << std::endl;
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
        file << "TrialNumber,GroupIndex,TrialInGroup,Plane,TargetPlaneSpinDeg,TargetAngleDeg,UserPlaneSpinDeg,UserAngleDeg,DurationSeconds,ForceCueCount\n";
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
        << std::fixed << std::setprecision(3) << trial.durationSeconds << ","
        << trial.forceCueCount << "\n";
    file.close();
}
