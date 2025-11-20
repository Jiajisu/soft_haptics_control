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
enum class ExperimentType { NONE, EXPERIMENT1, EXPERIMENT2 };

enum class ExperimentState {
    NOT_STARTED,      // 未开始
    READY,           // 准备就绪
    IN_PROGRESS,     // 进行中
    TRIAL_COMPLETE,  // 试次完成，等待下一个
    GROUP_COMPLETE,  // 组完成
    EXPERIMENT_COMPLETE  // 实验完成
};

enum class SurfacePolarity { POSITIVE = 1, NEGATIVE = -1 };

struct AngleTrialConfig {
    double angleX;
    double angleZ;
};

struct AngleTrialResult {
    int trialNumber;
    double targetAngleX;
    double targetAngleZ;
    double userAngleX;
    double userAngleZ;
    double durationSeconds;
    std::chrono::steady_clock::time_point trialStartTime;
    std::chrono::steady_clock::time_point confirmTime;
};

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
    // Experiment 2 (angular discrimination)
    bool hasNextTrialExp2() const;
    void startNextTrialExp2();
    void recordExp2UserAngles(double userAngleX, double userAngleZ, double durationSeconds);
    double getExp2TargetAngleX() const;
    double getExp2TargetAngleZ() const;
    int getExp2CurrentTrialIndex() const { return m_exp2CurrentTrialIndex; }
    ExperimentState getExp2State() const { return m_exp2State; }
    bool isExp2TrialActive() const { return m_exp2TrialActive; }
    bool isExp2BreakPending() const { return m_exp2NeedBreak; }
    void clearExp2BreakFlag() { m_exp2NeedBreak = false; }

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
    std::string getExp2SequenceFilename() const { return "user_" + m_userId + "_sequence_exp2.txt"; }

    void createOrLoadUserCSV();
    std::string getUserCSVFilename() const { return "user_" + m_userId + "_experiment_data.csv"; }
    void createOrLoadUserCSVExp2();
    std::string getUserExp2CSVFilename() const { return "user_" + m_userId + "_experiment2_data.csv"; }

    void writeTrialToCSV(const TrialResult& trial);
    void writeExp2TrialToCSV(const AngleTrialResult& trial);
    bool m_csvFileReady = false;
    bool m_exp2CsvFileReady = false;

    ExperimentState m_experimentState;
    ExperimentState m_exp2State;
    // Experiment 2 data
    std::vector<AngleTrialConfig> m_exp2Sequence;
    bool m_exp2SequenceLoaded = false;
    int m_exp2CurrentTrialIndex = 0;
    bool m_exp2TrialActive = false;
    bool m_exp2NeedBreak = false;
    AngleTrialResult m_exp2CurrentTrial;
    std::vector<AngleTrialResult> m_exp2Results;
    std::chrono::steady_clock::time_point m_exp2TrialStart;
    void generateExp2Sequence();
    bool loadExp2Sequence(const std::string& filename);
    void saveExp2Sequence(const std::string& filename);

};

#endif // USER_STUDY_MANAGER_HPP
