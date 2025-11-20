#ifndef USER_STUDY_MANAGER_HPP
#define USER_STUDY_MANAGER_HPP

#include "chai3d.h"
#include <vector>
#include <string>
#include <fstream>
#include <chrono>
#include <array>
#include <algorithm>
#include <random>

enum class InteractionMode { FORCE_TO_POSITION, FORCE_TO_FORCE };
enum class FeedbackDirection { LATERAL_X, LATERAL_Y, VERTICAL_Z };

enum class ExperimentState {
    NOT_STARTED,      // 未开始
    READY,           // 准备就绪
    IN_PROGRESS,     // 进行中
    TRIAL_COMPLETE,  // 试次完成，等待下一个
    GROUP_COMPLETE,  // 组完成
    EXPERIMENT_COMPLETE  // 实验完成
};

enum class SurfacePolarity { POSITIVE = 1, NEGATIVE = -1 };

struct TrialResult {
    int trialNumber;
    double referenceStiffness;
    double comparisonStiffness;
    int userChoice;
    double reactionTime;
    SurfacePolarity polarity;
    std::chrono::steady_clock::time_point trialStartTime;  // 按T时的时间
    std::chrono::steady_clock::time_point choiceTime;      // 按1或2时的时间
};

// 新增：试次配置结构体
struct TrialConfig {
    int stiffnessIndex;  // 0-10 (对应11个刚度比例)
    int repetition;      // 0-9 (每个刚度重复10次)
    SurfacePolarity polarity;
};

// 新增：组配置结构体
struct GroupConfig {
    InteractionMode mode;
    FeedbackDirection direction;
    std::vector<TrialConfig> trials; // 110个试次
};

class UserStudyManager {
public:
    UserStudyManager();
    ~UserStudyManager();

    /* --- 新增：流程管理 ------------------------------------------------- */
    bool loadOrCreateUserSession(const std::string& userId);
    void saveProgress();
    bool loadProgress(const std::string& filename);

    /* --- 试次/组控制 --------------------------------------------------- */
    void setManualTrialGroup(InteractionMode mode, FeedbackDirection direction);
    void resetCurrentGroup();
    void sanitizeProgressIndices();
    bool isGroupComplete() const { return m_currentTrialInGroup >= 110; }
    int getCurrentTrialGroup() const { return m_currentTrialGroup; }
    int getCurrentTrialInGroup() const { return m_currentTrialInGroup; }
    int getCurrentTrialNumber() const { return m_currentTrialGroup * 110 + m_currentTrialInGroup; }

    /* --- 状态查询 ------------------------------------------------------- */
    bool isTrialActive() const { return m_trialActive; }
    InteractionMode getCurrentMode() const { return m_currentMode; }
    FeedbackDirection getCurrentDirection() const { return m_currentDirection; }

    /* --- 实验控制 ------------------------------------------------------- */
    void initializeTrials();
    bool hasNextTrial();
    void startNextTrial();
    void recordUserChoice(int choice);
    SurfacePolarity getCurrentPolarity() const { return m_currentPolarity; }

    /* --- 刚度 ----------------------------------------------------------- */
    double getCurrentReferenceStiffness() const;
    double getCurrentComparisonStiffness() const;

    /* --- 数据 ----------------------------------------------------------- */
    void saveResults(const std::string& filename);
    // 新增：获取实验状态
    ExperimentState getExperimentState() const { return m_experimentState; }
    void setExperimentReady() { m_experimentState = ExperimentState::READY; }


private:
    /* --- 实验状态 ------------------------------------------------------- */
    bool m_trialActive;
    std::string m_userId;  // 新增：用户ID

    /* --- 设计参数 ------------------------------------------------------- */
    InteractionMode m_currentMode;
    FeedbackDirection m_currentDirection;
    int m_currentTrialGroup;    // 0-5
    int m_currentTrialInGroup;  // 0-109
    SurfacePolarity m_currentPolarity;

    /* --- 刚度 ----------------------------------------------------------- */
    double m_referenceStiffness;
    std::vector<double> m_comparisonStiffnesses;

    /* --- 试次数据 ------------------------------------------------------- */
    std::vector<TrialResult> m_results;
    TrialResult m_currentTrial;

    /* --- 新增：实验流程配置 --------------------------------------------- */
    std::vector<GroupConfig> m_experimentSequence; // 6个组的完整配置
    bool m_sequenceLoaded;


    /* --- 私有工具 ------------------------------------------------------- */
    void generateComparisonStiffnesses();
    void generateRandomSequence();
    std::vector<TrialConfig> generateRandomizedTrials();
    void saveSequenceToFile(const std::string& filename);
    bool loadSequenceFromFile(const std::string& filename);
    std::string getProgressFilename() const { return "user_" + m_userId + "_progress.txt"; }
    std::string getSequenceFilename() const { return "user_" + m_userId + "_sequence.txt"; }

    void createOrLoadUserCSV();
    std::string getUserCSVFilename() const { return "user_" + m_userId + "_experiment_data.csv"; }

    void writeTrialToCSV(const TrialResult& trial);
    bool m_csvFileReady = false;

    ExperimentState m_experimentState;

};

#endif // USER_STUDY_MANAGER_HPP
