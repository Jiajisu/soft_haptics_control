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
enum class Exp2Plane { PLANE_X, PLANE_Y, PLANE_Z, PLANE_COMBINED };

enum class ExperimentState {
    NOT_STARTED,      // ???
    READY,           // ????
    IN_PROGRESS,     // ???
    TRIAL_COMPLETE,  // ????,?????
    GROUP_COMPLETE,  // ???
    EXPERIMENT_COMPLETE  // ????
};

enum class SurfacePolarity { POSITIVE = 1, NEGATIVE = -1 };

struct StaircaseState {
    double currentKComp = 0.0;
    double currentStep = 0.0;
    int reversalCount = 0;
    int trialCount = 0;
    int correctStreak = 0;
    int lastMoveDir = 0;
};

class AdaptiveStaircase {
public:
    AdaptiveStaircase(double kRef,
        double startKComp,
        double minKComp,
        double maxKComp,
        double startStep,
        double fineStep,
        int switchAfterReversals = 2,
        int targetReversals = 10);

    double getNextKComp() const;
    double getDeltaK() const;
    double getStepSize() const;
    void update(bool isCorrect);
    bool shouldStop() const;
    StaircaseState getState() const { return m_state; }
    void setState(const StaircaseState& state);

private:
    double clampK(double k) const;
    void refreshStep();

    double m_kRef;
    double m_minKComp;
    double m_maxKComp;
    double m_startStep;
    double m_fineStep;
    int m_switchAfterReversals;
    int m_targetReversals;
    StaircaseState m_state;
};

struct AngleTrialConfig {
    Exp2Plane plane;
    double planeRotationDeg; // rotation of the base plane about Z (A)
    double inPlaneAngleDeg;  // rotation of the line within the plane (B)
};

struct AngleTrialResult {
    int trialNumber;
    Exp2Plane plane;
    double targetPlaneRotationDeg;
    double targetInPlaneAngleDeg;
    double userPlaneRotationDeg;
    double userInPlaneAngleDeg;
    double durationSeconds;
    int forceCueCount = 0;
    std::chrono::steady_clock::time_point trialStartTime;
    std::chrono::steady_clock::time_point confirmTime;
};

struct TrialResult {
    int trialNumber;
    int groupIndex;
    int trialInGroup;
    double referenceStiffness;
    double comparisonStiffness;
    int userChoice;
    int correctChoice;
    bool isCorrect;
    double reactionTime;
    SurfacePolarity polarity;
    bool referenceOnLeft;
    double staircaseDeltaK;
    double staircaseStep;
    int staircaseReversals;
    int staircaseTrialInGroup;
    int staircaseId;
    std::chrono::steady_clock::time_point trialStartTime;  // ?T????
    std::chrono::steady_clock::time_point choiceTime;      // ?1?2????
};

// ??:???????
struct TrialConfig {
    int stiffnessIndex;  // 0-10 (??11?????)
    int repetition;      // 0-9 (??????10?)
    SurfacePolarity polarity;
};

// ??:??????
struct GroupConfig {
    InteractionMode mode;
    FeedbackDirection direction;
    std::vector<TrialConfig> trials; // placeholder for legacy constant-stimuli configs
};

class UserStudyManager {
public:
    UserStudyManager();
    ~UserStudyManager();

    /* --- ??:???? ------------------------------------------------- */
    bool loadOrCreateUserSession(const std::string& userId);
    void saveProgress();
    bool loadProgress(const std::string& filename);

    /* --- ??/??? --------------------------------------------------- */
    void setManualTrialGroup(InteractionMode mode, FeedbackDirection direction);
    void resetCurrentGroup();
    void sanitizeProgressIndices();
    bool isGroupComplete() const;
    int getCurrentTrialGroup() const { return m_currentTrialGroup; }
    int getCurrentTrialInGroup() const { return m_currentTrialInGroup; }
    int getCurrentTrialNumber() const { return m_currentTrial.trialNumber; }

    /* --- ???? ------------------------------------------------------- */
    bool isTrialActive() const { return m_trialActive; }
    InteractionMode getCurrentMode() const { return m_currentMode; }
    FeedbackDirection getCurrentDirection() const { return m_currentDirection; }
    bool peekNextTrialSurface(FeedbackDirection& direction, SurfacePolarity& polarity);
    bool getReferenceOnLeft() const { return m_currentReferenceOnLeft; }
    StaircaseState getCurrentStaircaseState() const;
    int getActiveStaircaseId() const { return m_currentStaircaseId; }
    int getTargetReversals() const { return m_targetReversals; }

    /* --- ???? ------------------------------------------------------- */
    void initializeTrials();
    bool hasNextTrial();
    void startNextTrial();
    void recordUserChoice(int choice);
    SurfacePolarity getCurrentPolarity() const { return m_currentPolarity; }
    // Experiment 2 (angular discrimination)
    bool hasNextTrialExp2() const;
    void startNextTrialExp2();
    void recordExp2UserPose(double planeSpinDeg, double inPlaneAngleDeg, double durationSeconds);
    void recordExp2UserAngle(double userAngleDeg, double durationSeconds); // legacy shim, uses current plane spin
    void recordExp2UserAngles(double userAngleX, double userAngleZ, double durationSeconds); // legacy shim
    void incrementExp2ForceCueCount();
    double getExp2TargetPlaneSpinDeg() const;
    double getExp2TargetAngleDeg() const;
    double getExp2TargetAngleX() const; // legacy shim
    double getExp2TargetAngleZ() const; // legacy shim
    Exp2Plane getExp2Plane() const { return m_exp2CurrentTrial.plane; }
    void setManualExp2Group(Exp2Plane plane);
    int getExp2TrialsPerGroup(int groupIdx = 0) const;
    int getExp2GroupCount() const;
    int getExp2GroupSize(int groupIdx) const;
    int getExp2GroupStartIndex(int groupIdx) const;
    int getExp2GroupIndexForTrial(int flatTrialIndex) const;
    int getExp2TrialOffsetInGroup(int flatTrialIndex) const;
    int getExp2CompletedTrials() const;
    double getReferenceStiffness() const { return m_referenceStiffness; }
    int getExp2CurrentTrialIndex() const { return m_exp2CurrentTrialIndex; }
    int getExp2TotalTrials() const { return static_cast<int>(m_exp2Sequence.size()); }
    int getExp2ExpectedTrialCount() const;
    ExperimentState getExp2State() const { return m_exp2State; }
    bool isExp2TrialActive() const { return m_exp2TrialActive; }
    bool isExp2BreakPending() const { return m_exp2NeedBreak; }
    void clearExp2BreakFlag() { m_exp2NeedBreak = false; }
    int getExp1CompletedTrials() const;

    /* --- ?? ----------------------------------------------------------- */
    double getCurrentReferenceStiffness() const;
    double getCurrentComparisonStiffness() const;

    /* --- ?? ----------------------------------------------------------- */
    void saveResults(const std::string& filename);
    // ??:??????
    ExperimentState getExperimentState() const { return m_experimentState; }
    void setExperimentReady() { m_experimentState = ExperimentState::READY; }


private:
    /* --- ???? ------------------------------------------------------- */
    bool m_trialActive;
    std::string m_userId;  // ??:??ID

    /* --- ???? ------------------------------------------------------- */
    InteractionMode m_currentMode;
    FeedbackDirection m_currentDirection;
    int m_currentTrialGroup;    // 0-5
    int m_currentTrialInGroup;  // 0-based trial count within the current group
    SurfacePolarity m_currentPolarity;
    std::vector<int> m_groupProgress; // per-group completed trial count (staircase trialCount)

    /* --- ?? ----------------------------------------------------------- */
    double m_referenceStiffness;
    std::vector<double> m_comparisonStiffnesses;

    /* --- ???? ------------------------------------------------------- */
    std::vector<TrialResult> m_results;
    TrialResult m_currentTrial;

    /* --- ??:?????? --------------------------------------------- */
    std::vector<GroupConfig> m_experimentSequence; // 6???????
    bool m_sequenceLoaded;
    std::vector<AdaptiveStaircase> m_staircases;
    int m_currentStaircaseId;
    bool m_currentReferenceOnLeft;
    bool m_pendingReferenceOnLeft;
    SurfacePolarity m_pendingPolarity;
    bool m_hasPendingTrialConfig;
    int m_pendingGroupIdx;
    double m_startKComp;
    double m_minKComp;
    double m_maxKComp;
    double m_startStep;
    double m_fineStep;
    int m_switchAfterReversals;
    int m_targetReversals;
    std::mt19937 m_rng;


    /* --- ???? ------------------------------------------------------- */
    void generateComparisonStiffnesses();
    void generateRandomSequence();
    std::vector<TrialConfig> generateRandomizedTrials();
    void saveSequenceToFile(const std::string& filename);
    bool loadSequenceFromFile(const std::string& filename);
    void initializeStaircases();
    int findNextExp1GroupWithRemaining(int startGroup) const;
    int selectStaircaseId(int groupIdx) const;
    StaircaseState getStaircaseStateForGroup(int groupIdx) const;
    void setStaircaseStateForGroup(int groupIdx, const StaircaseState& state);
    void syncGroupProgressFromStaircases();
    void randomizeNextTrialConfig(int groupIdx, SurfacePolarity& polarityOut, bool& referenceOnLeftOut);
    bool isGroupIndexComplete(int groupIdx) const;
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
    std::vector<int> m_exp2GroupSizes;
    int m_exp2CurrentTrialIndex = 0;
    int m_exp2CurrentGroup = 0;
    bool m_exp2TrialActive = false;
    bool m_exp2NeedBreak = false;
    std::vector<int> m_exp2GroupProgress; // per-group next trial index
    AngleTrialResult m_exp2CurrentTrial;
    std::vector<AngleTrialResult> m_exp2Results;
    std::chrono::steady_clock::time_point m_exp2TrialStart;
    void generateExp2Sequence();
    bool loadExp2Sequence(const std::string& filename);
    void saveExp2Sequence(const std::string& filename);
    int findExp2GroupStartIndex(Exp2Plane plane) const;
    bool isExp2SequenceGroupedByPlane() const;
    void regroupExp2SequenceByPlane(bool shuffleWithinGroups);
    void ensureExp2SequenceGrouped();
    void rebuildExp2GroupSizesFromSequence();
    int expectedExp2GroupSizeForPlane(Exp2Plane plane) const;
    void ensureExp1GroupProgressSize();
    void ensureExp2GroupProgressSize();
    int findNextExp2GroupWithRemaining(int startGroup) const;

};

#endif // USER_STUDY_MANAGER_HPP
