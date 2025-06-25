#include "UserStudyManager.hpp"
#include <algorithm>
#include <random>
#include <iostream>
#include <sstream>
#include <iomanip>

UserStudyManager::UserStudyManager()
    : m_trialActive(false)
    , m_currentMode(InteractionMode::FORCE_TO_POSITION)
    , m_currentDirection(FeedbackDirection::LATERAL_X)
    , m_currentTrialGroup(0)
    , m_currentTrialInGroup(0)
    , m_referenceStiffness(1000.0)
    , m_sequenceLoaded(false)
    , m_experimentState(ExperimentState::NOT_STARTED)
    , m_csvFileReady(false)  // ★ 确认这行存在
{
    generateComparisonStiffnesses();
}

UserStudyManager::~UserStudyManager()
{
    if (!m_results.empty()) {
        saveResults("user_" + m_userId + "_results.csv");
        saveProgress(); // 保存最终进度
    }
}

void UserStudyManager::generateComparisonStiffnesses()
{
    m_comparisonStiffnesses.clear();
    const std::vector<double> ratios = { 0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.4,1.5 };
    for (double r : ratios)
        m_comparisonStiffnesses.push_back(m_referenceStiffness * r);
}

// 新增：加载或创建用户会话
// 修改 loadOrCreateUserSession 函数
// 在 UserStudyManager.cpp 中，修改 loadOrCreateUserSession 函数：

bool UserStudyManager::loadOrCreateUserSession(const std::string& userId)
{
    m_userId = userId;

    std::cout << "[UserStudy] Setting user ID: " << userId << std::endl;  // 调试输出

    // 尝试加载已有的序列文件
    if (loadSequenceFromFile(getSequenceFilename())) {
        std::cout << "[UserStudy] Loaded existing sequence for user: " << userId << std::endl;

        // 尝试加载进度
        if (loadProgress(getProgressFilename())) {
            std::cout << "[UserStudy] Resumed from Group " << m_currentTrialGroup + 1
                << "/6, Trial " << m_currentTrialInGroup + 1 << "/110" << std::endl;

            // 检查实验状态
            if (m_currentTrialGroup >= 6) {
                m_experimentState = ExperimentState::EXPERIMENT_COMPLETE;
                std::cout << "[UserStudy] This user has already completed the experiment!" << std::endl;
            }
            else if (m_currentTrialInGroup >= 110) {
                m_experimentState = ExperimentState::GROUP_COMPLETE;
                std::cout << "[UserStudy] Current group is complete. Press 'T' to start next group." << std::endl;
            }
            else {
                m_experimentState = ExperimentState::READY;
                std::cout << "[UserStudy] Ready to continue. Press 'T' to start next trial." << std::endl;
            }
        }
        else {
            // 有序列但没有进度，从头开始
            m_experimentState = ExperimentState::READY;
            std::cout << "[UserStudy] Starting from beginning. Press 'T' to start first trial." << std::endl;
        }
    }
    else {
        // 创建新的随机序列
        std::cout << "[UserStudy] Creating new sequence for user: " << userId << std::endl;
        generateRandomSequence();
        saveSequenceToFile(getSequenceFilename());

        // 初始化进度
        m_currentTrialGroup = 0;
        m_currentTrialInGroup = 0;
        m_experimentState = ExperimentState::READY;
        saveProgress();

        std::cout << "[UserStudy] New experiment ready. Press 'T' to start first trial." << std::endl;
    }

    // ★ 重要：在这里添加CSV文件创建调用
    std::cout << "[UserStudy] Initializing CSV file..." << std::endl;
    createOrLoadUserCSV();
    std::cout << "[UserStudy] CSV initialization complete. Ready: " << (m_csvFileReady ? "YES" : "NO") << std::endl;

    return true;
}


// 生成随机实验序列
void UserStudyManager::generateRandomSequence()
{
    m_experimentSequence.clear();

    // 创建6个组的配置 (2 modes × 3 directions)
    std::vector<std::pair<InteractionMode, FeedbackDirection>> groupConfigs = {
        {InteractionMode::FORCE_TO_POSITION, FeedbackDirection::LATERAL_X},
        {InteractionMode::FORCE_TO_POSITION, FeedbackDirection::LATERAL_Y},
        {InteractionMode::FORCE_TO_POSITION, FeedbackDirection::VERTICAL_Z},
        {InteractionMode::FORCE_TO_FORCE, FeedbackDirection::LATERAL_X},
        {InteractionMode::FORCE_TO_FORCE, FeedbackDirection::LATERAL_Y},
        {InteractionMode::FORCE_TO_FORCE, FeedbackDirection::VERTICAL_Z}
    };

    // 随机打乱组顺序
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(groupConfigs.begin(), groupConfigs.end(), g);

    // 为每个组生成随机试次序列
    for (const auto& config : groupConfigs) {
        GroupConfig group;
        group.mode = config.first;
        group.direction = config.second;
        group.trials = generateRandomizedTrials();
        m_experimentSequence.push_back(group);
    }

    m_sequenceLoaded = true;
}

// 生成一个组内的随机试次序列
std::vector<TrialConfig> UserStudyManager::generateRandomizedTrials()
{
    std::vector<TrialConfig> trials;

    // 创建110个试次：11个刚度级别，每个重复10次
    for (int stiffnessIdx = 0; stiffnessIdx < 11; ++stiffnessIdx) {
        for (int rep = 0; rep < 10; ++rep) {
            trials.push_back({ stiffnessIdx, rep });
        }
    }

    // 随机打乱
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(trials.begin(), trials.end(), g);

    return trials;
}

// 保存序列到文件
void UserStudyManager::saveSequenceToFile(const std::string& filename)
{
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "[UserStudy] Cannot create sequence file: " << filename << std::endl;
        return;
    }

    // 写入组顺序
    file << "# Experiment Sequence for User: " << m_userId << "\n";
    file << "# Format: GroupIndex Mode Direction\n";

    for (size_t i = 0; i < m_experimentSequence.size(); ++i) {
        const auto& group = m_experimentSequence[i];
        file << i << " " << (int)group.mode << " " << (int)group.direction << "\n";
    }

    file << "\n# Trial Sequence (StiffnessIndex Repetition)\n";

    // 写入每个组的试次序列
    for (size_t groupIdx = 0; groupIdx < m_experimentSequence.size(); ++groupIdx) {
        file << "# Group " << groupIdx << " (110 trials)\n";
        const auto& trials = m_experimentSequence[groupIdx].trials;

        // 确保每组有110个试次
        for (size_t i = 0; i < trials.size(); ++i) {
            file << trials[i].stiffnessIndex << " " << trials[i].repetition;

            // 每行10对数据，便于阅读
            if ((i + 1) % 10 == 0) {
                file << "\n";
            }
            else if (i < trials.size() - 1) {
                file << " ";
            }
        }

        // 确保组之间有空行
        if (groupIdx < m_experimentSequence.size() - 1) {
            file << "\n\n";
        }
    }

    file.close();
    std::cout << "[UserStudy] Sequence saved to: " << filename << std::endl;
}

// 从文件加载序列
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
        // 跳过空行和注释
        if (line.empty() || line[0] == '#') {
            // 检查是否开始读取试次序列
            if (line.find("Trial Sequence") != std::string::npos) {
                readingGroups = false;
                std::cout << "[UserStudy] Starting to read trial sequences..." << std::endl;
            }
            continue;
        }

        std::istringstream iss(line);

        if (readingGroups && m_experimentSequence.size() < 6) {
            // 读取组配置
            int groupIdx, mode, dir;
            if (iss >> groupIdx >> mode >> dir) {
                GroupConfig group;
                group.mode = static_cast<InteractionMode>(mode);
                group.direction = static_cast<FeedbackDirection>(dir);
                group.trials.clear(); // 确保trials向量是空的
                m_experimentSequence.push_back(group);

                std::cout << "[UserStudy] Loaded group " << groupIdx
                    << " Mode:" << mode << " Dir:" << dir << std::endl;
            }
        }
        else if (!readingGroups) {
            // 确定当前正在读取哪个组的试次
            if (currentGroupIdx < 0 || currentGroupIdx >= m_experimentSequence.size()) {
                // 找到下一个有效的组索引
                for (size_t i = 0; i < m_experimentSequence.size(); ++i) {
                    if (m_experimentSequence[i].trials.size() < 110) {
                        currentGroupIdx = i;
                        break;
                    }
                }
            }

            if (currentGroupIdx >= 0 && currentGroupIdx < m_experimentSequence.size()) {
                // 读取试次数据
                int stiffIdx, rep;
                while (iss >> stiffIdx >> rep) {
                    if (m_experimentSequence[currentGroupIdx].trials.size() < 110) {
                        m_experimentSequence[currentGroupIdx].trials.push_back({ stiffIdx, rep });
                    }
                }

                // 如果当前组已满，移到下一组
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

    // 验证加载的数据
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

// 保存进度
void UserStudyManager::saveProgress()
{
    std::ofstream file(getProgressFilename());
    if (!file) return;

    file << "# Progress for User: " << m_userId << "\n";
    file << "CurrentGroup " << m_currentTrialGroup << "\n";
    file << "CurrentTrialInGroup " << m_currentTrialInGroup << "\n";
    file << "TotalTrialsCompleted " << m_results.size() << "\n";

    file.close();
}

// 加载进度
// 修改 loadProgress 函数，修复读取问题
bool UserStudyManager::loadProgress(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file) return false;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::string key;
        if (iss >> key) {
            if (key == "CurrentGroup") {
                iss >> m_currentTrialGroup;
            }
            else if (key == "CurrentTrialInGroup") {
                iss >> m_currentTrialInGroup;
            }
            else if (key == "TotalTrialsCompleted") {
                int totalCompleted;
                iss >> totalCompleted;
                // 可以用来验证数据一致性
            }
        }
    }

    file.close();
    return true;
}

// 修改 startNextTrial 函数
void UserStudyManager::startNextTrial()
{
    // 检查实验状态
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

    // 边界检查
    if (m_currentTrialGroup >= m_experimentSequence.size()) {
        std::cout << "[UserStudy] ERROR: Invalid group index " << m_currentTrialGroup
            << " (max: " << m_experimentSequence.size() - 1 << ")" << std::endl;
        m_experimentState = ExperimentState::EXPERIMENT_COMPLETE;
        return;
    }

    // 检查是否需要移到下一组
    if (m_currentTrialInGroup >= 110) {
        m_currentTrialInGroup = 0;
        m_currentTrialGroup++;
        if (m_currentTrialGroup >= 6) {
            m_experimentState = ExperimentState::EXPERIMENT_COMPLETE;
            std::cout << "[UserStudy] Experiment Complete! Thank you for your participation." << std::endl;
            saveResults("user_" + m_userId + "_final_results.csv");
            return;
        }
    }

    // 再次检查组索引
    if (m_currentTrialGroup >= m_experimentSequence.size()) {
        std::cout << "[UserStudy] ERROR: Group index out of bounds!" << std::endl;
        return;
    }

    // 获取当前组配置
    const auto& currentGroup = m_experimentSequence[m_currentTrialGroup];

    // 检查试次索引
    if (m_currentTrialInGroup >= currentGroup.trials.size()) {
        std::cout << "[UserStudy] ERROR: Trial index " << m_currentTrialInGroup
            << " out of bounds (max: " << currentGroup.trials.size() - 1 << ")" << std::endl;
        return;
    }

    m_currentMode = currentGroup.mode;
    m_currentDirection = currentGroup.direction;

    // 获取当前试次配置
    const auto& trialConfig = currentGroup.trials[m_currentTrialInGroup];

    // 检查刚度索引
    if (trialConfig.stiffnessIndex >= m_comparisonStiffnesses.size()) {
        std::cout << "[UserStudy] ERROR: Stiffness index " << trialConfig.stiffnessIndex
            << " out of bounds (max: " << m_comparisonStiffnesses.size() - 1 << ")" << std::endl;
        return;
    }

    // 初始化试次
    m_currentTrial = TrialResult{};
    m_currentTrial.trialNumber = m_currentTrialGroup * 110 + m_currentTrialInGroup;
    m_currentTrial.referenceStiffness = m_referenceStiffness;
    m_currentTrial.comparisonStiffness = m_comparisonStiffnesses[trialConfig.stiffnessIndex];
    m_currentTrial.touchCountLeft = 0;
    m_currentTrial.touchCountRight = 0;
    // ★ 修改：记录按T时的时间作为反应时间起点
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

    std::cout << "\nRef: " << m_currentTrial.referenceStiffness
        << " | Comp: " << m_currentTrial.comparisonStiffness
        << " (Ratio: " << std::fixed << std::setprecision(1)
        << (trialConfig.stiffnessIndex * 0.1 + 0.5) << ")"
        << std::endl;
}

// 修改 recordUserChoice 函数
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

    // 自动写入CSV文件
    if (m_csvFileReady) {
        writeTrialToCSV(m_currentTrial);
    }

    std::cout << "[UserStudy] Choice recorded: " << (choice == 0 ? "Left" : "Right")
        << " | Reaction Time: " << std::fixed << std::setprecision(3)
        << m_currentTrial.reactionTime << "s" << std::endl;

    // 下一试次
    m_currentTrialInGroup++;

    // 检查状态
    if (m_currentTrialInGroup >= 110) {
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

        // 每55个试次提醒休息
        if (m_currentTrialInGroup == 55) {
            std::cout << "\n*** 2-MINUTE BREAK TIME ***" << std::endl;
            std::cout << "You've completed 55 trials. Please rest for 2 minutes." << std::endl;
        }

        std::cout << "Press 'T' for next trial." << std::endl;
    }

    // 保存进度
    saveProgress();
}

// 其余函数保持不变...
void UserStudyManager::recordTouch(bool left)
{
    if (!m_trialActive) return;
    if (left) ++m_currentTrial.touchCountLeft;
    else ++m_currentTrial.touchCountRight;
}

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
        << "UserChoice,ReactionTime,TouchCountLeft,TouchCountRight\n";

    for (const auto& r : m_results) {
        // 获取试次对应的组信息
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
                << r.touchCountLeft << ','
                << r.touchCountRight << '\n';
        }
    }

    std::cout << "[UserStudy] Saved " << m_results.size() << " trials -> " << fn << std::endl;
}

void UserStudyManager::setManualTrialGroup(InteractionMode m, FeedbackDirection d)
{
    // 手动设置时，查找匹配的组
    for (size_t i = 0; i < m_experimentSequence.size(); ++i) {
        if (m_experimentSequence[i].mode == m &&
            m_experimentSequence[i].direction == d) {
            m_currentTrialGroup = i;
            m_currentTrialInGroup = 0;
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
    saveProgress();
    std::cout << "[UserStudy] Group reset.\n";
}


void UserStudyManager::initializeTrials()
{
    m_results.clear();
    std::cout << "[UserStudy] Trials initialized\n";
}

// 修改 hasNextTrial 函数
bool UserStudyManager::hasNextTrial()
{
    return m_sequenceLoaded &&
        m_currentTrialGroup < 6 &&
        m_experimentState != ExperimentState::EXPERIMENT_COMPLETE;
}

void UserStudyManager::createOrLoadUserCSV()
{
    std::string csvFilename = getUserCSVFilename();

    std::cout << "[UserStudy] CSV filename: " << csvFilename << std::endl;

    // 检查文件是否存在且有内容
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

        // 创建新文件并写入简化的表头
        std::ofstream newFile(csvFilename, std::ios::out);
        if (newFile.is_open()) {
            // ★ 修改：简化的表头
            std::string header = "TrialNumber,Mode,Direction,ReferenceStiffness,ComparisonStiffness,UserChoice,ReactionTime,TouchCountLeft,TouchCountRight";
            newFile << header << std::endl;
            newFile.flush();

            std::cout << "[UserStudy] Header written to file." << std::endl;

            newFile.close();

            // 验证写入
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
    std::ofstream csvFile(csvFilename, std::ios::app);  // 追加模式

    if (!csvFile.is_open()) {
        std::cerr << "[UserStudy] ERROR: Could not open CSV file for writing!" << std::endl;
        return;
    }

    // 获取当前试次对应的组信息
    int groupIdx = trial.trialNumber / 110;
    if (groupIdx < m_experimentSequence.size()) {
        const auto& group = m_experimentSequence[groupIdx];

        // ★ 修改：简化的CSV行，去掉UserID、GroupNumber、TrialInGroup、Timestamp
        csvFile << trial.trialNumber << ","
            << (int)group.mode << ","
            << (int)group.direction << ","
            << std::fixed << std::setprecision(2) << trial.referenceStiffness << ","
            << std::fixed << std::setprecision(2) << trial.comparisonStiffness << ","
            << trial.userChoice << ","
            << std::fixed << std::setprecision(4) << trial.reactionTime << ","
            << trial.touchCountLeft << ","
            << trial.touchCountRight << std::endl;

        csvFile.flush();
        std::cout << "[UserStudy] Trial data written to CSV file." << std::endl;
    }

    csvFile.close();
}