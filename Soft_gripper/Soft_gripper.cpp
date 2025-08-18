
//------------------------------------------------------------------------------
// Arduino
//------------------------------------------------------------------------------
#include "SerialPort.hpp"
#include <iomanip>
//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>


//------------------------------------------------------------------------------
// MTW
//------------------------------------------------------------------------------
#include "MicronTrackerWrapper.hpp"
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
// Self Define functions 
//------------------------------------------------------------------------------

#include "ControlPCC.hpp"
#include "TrajectoryGenerator.hpp"
#include "TimeUtil.hpp"
#include "UserStudyManager.hpp"

// 在现有全局变量声明后添加
//------------------------------------------------------------------------------
// 表征实验对象
//------------------------------------------------------------------------------
cShapeLine* axisX = nullptr;
cShapeLine* axisY = nullptr;
cShapeLine* axisZ = nullptr;
cShapeSphere* sphereTarget = nullptr;
cShapeCylinder* cylinderTop = nullptr;
cShapeCylinder* cylinderBot = nullptr;
cShapeCylinder* cylinderTopRel = nullptr;
//------------------------------------------------------------------------------
// 用户实验管理
//------------------------------------------------------------------------------

UserStudyManager* userStudy = nullptr;
bool experimentMode = false;  // 切换实验模式的标志

// 虚拟立方体用于刚度测试
cMesh* leftCube = nullptr;
cMesh* rightCube = nullptr;
bool isInteractingWithLeftCube = false;
bool isInteractingWithRightCube = false;
// 添加一个全局变量存储用户ID
std::string currentUserId = "";
//------------------------------------------------------------------------------
// 场景管理变量
//------------------------------------------------------------------------------
// 保存原始对象的显示状态
bool originalObjectsVisible = true;
// 实验提示标签
cLabel* labelExperimentInstructions = nullptr;
cLabel* labelTrialInfo = nullptr;
// 场景管理函数
void showOriginalObjects(bool show);
void showExperimentObjects(bool show);
void updateExperimentLabels();
void updateSurfaceOrientation(FeedbackDirection direction);
//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
	C_STEREO_DISABLED:            Stereo is disabled
	C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
	C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
	C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;

//------------------------------------------------------------------------------
// DECLARED VARIABLES JJS
//------------------------------------------------------------------------------


// write to csv
// a file to store force
std::ofstream csvFile;
bool recordSensorDataToCSV = false;  // toggle if record sensor value
static bool hasInitPress = false;
cVector3d presCurr;
chai3d::cVector3d PressuretoArduino;
int circleIndex = 0;
int pointIndex = 10;
double duration = 14.0;         // total movement time
static bool g_trajectoryStarted = false;
static double g_trajectoryStartTime = 0.0;
// This accumulator persists across multiple calls to updateHaptics
//static cVector3d pressureAccum(0.0, 0.0, 0.0);
bool g_enableControl = false;
cVector3d ZeroPressure(0, 0, 0);
static mtw::MicronTracker MT;
PNODATA makePNO_mm_colRM(const double pos_mm[3], const double Rcol[9]);
void MTLoop();
//std::thread mtThread;   // 全局，默认构造为空线程
cThread* mtThread = nullptr;
static inline void mulRT(const double R[9], const double v[3], double o[3]);
static inline void colToRow(const double C[9], double R[9]);
//――――――― MicronTracker 线程共享数据 ―――――――
struct MTPoseSnapshot {
	bool   haveFr = false;
	bool   haveAct = false;
	bool   haveRel = false;
	mtw::Pose fr, act;
	double  relPos[3]{};
	double  relR[9]{};
	double  ts = 0.0;
};
MTPoseSnapshot       g_mtSnap;
std::mutex           g_mtMtx;
std::atomic<bool>    g_mtRunning{ true };

/// mm → m，并把 3×3 行主序矩阵转成 cMatrix3d
inline void mtPose2Chai(const double pos_mm[3],
	const double Rrow[9],
	cVector3d& p_out,
	cMatrix3d& R_out)
{
	constexpr double s = 0.001;           // mm → m
	p_out.set(pos_mm[0] * s, pos_mm[1] * s, pos_mm[2] * s);

	R_out.set(Rrow[0], Rrow[1], Rrow[2],
		Rrow[3], Rrow[4], Rrow[5],
		Rrow[6], Rrow[7], Rrow[8]);
}
static bool g_mtAvailable = false;           // ★ 新增



//------------------------------------------------------------------------------
// Calibration相关变量
//------------------------------------------------------------------------------

// 显示球体
cShapeSphere* sphereFinger = nullptr;    // 绿色 - finger marker
cShapeSphere* sphereActuator = nullptr;  // 红色 - actuator marker  
cShapeSphere* sphereBot = nullptr;       // 蓝色 - 计算得到的bot
cShapeSphere* sphereTop = nullptr;       // 黄色 - 计算得到的top

// 显示控制标志
bool g_showCalibrationSpheres = false;
//------------------------------------------------------------------------------
// 坐标变换函数声明
//------------------------------------------------------------------------------
cTransform transformCameraToBase(const double camPos[3], const double camRot[9]);
bool getMTMarkerTransform(const std::string& markerName, cTransform& outTransform);

//--------------------------------------------------------------
//  Base Marker 参考系
//--------------------------------------------------------------
//struct BaseReferenceFrame
//{
//	// base marker在相机坐标系下的位姿
//	double basePos[3]{};     // 位置 (mm)
//	double baseR[9]{};       // 旋转矩阵 (col-major)
//
//	// base到虚拟环境的变换（用户配置）
//	double ve_offset[3]{ 0.0, 0.0, 0.0 };  // 平移偏移
//	double ve_scale = 1.0;               // 缩放因子
//
//	    // base的逆变换（用于快速计算）
//    double baseR_inv[9]{};   // R_base^T (用于转换到base坐标系)
//
//	bool valid{ false };
//} g_camera_T_base;

cTransform g_camera_T_base;
cTransform g_base_offset(
	cVector3d(0.0, 0.0, 0.0),
	cMatrix3d(
		0.0, 1.0, 0.0,
		1.0, 0.0, 0.0,
		0.0, 0.0, -1.0
	)
);

void matrixTranspose(const double M[9], double MT[9]) {
	for (int r = 0; r < 3; ++r) {
		for (int c = 0; c < 3; ++c) {
			MT[c * 3 + r] = M[r * 3 + c];
		}
	}
}

// 初始化base参考系
bool initBaseReference(mtw::MicronTracker& MT) {
	std::cout << "[MT] ========================================" << std::endl;
	std::cout << "[MT] Initializing BASE reference frame..." << std::endl;
	std::cout << "[MT] Make sure 'base' marker is visible!" << std::endl;
	std::cout << "[MT] ========================================" << std::endl;

	if (!MT.update()) {
		std::cerr << "[MT] Update failed - cannot init base\n";
		return false;
	}

	mtw::Pose base;
	if (!MT.getPoseAndRotMat("base", base)) {
		std::cerr << "[MT] ERROR: Cannot find 'base' marker!\n";
		return false;
	}

	// 保存base位姿
	cVector3d pos(
		base.pos[0] * 0.001,
		base.pos[1] * 0.001,
		base.pos[2] * 0.001);

	cMatrix3d rot(
		base.rot[0], base.rot[1], base.rot[2],
		base.rot[3], base.rot[4], base.rot[5],
		base.rot[6], base.rot[7], base.rot[8]);

	g_camera_T_base.set(pos, rot);
	g_camera_T_base = g_camera_T_base * g_base_offset;

	std::cout << "[MT] Base reference initialized successfully!";
	std::cout << "[MT] Position: " << (g_camera_T_base.getLocalPos() * 1000).str(3) << "mm" << std::endl;

	return true;
}


//------------------------------------------------------------------------------
// Calibration相关变量
//------------------------------------------------------------------------------
// Calibration模式控制
bool g_calibrationMode = false;
enum CalibrationStep {
	CALIB_NONE,
	CALIB_WAITING_FINGER,
	CALIB_WAITING_BOT,
	CALIB_WAITING_ACTUATOR,
	CALIB_WAITING_TOP,
	CALIB_COMPLETE
};
CalibrationStep g_calibStep = CALIB_NONE;

// 保存calibration时记录的位置和旋转
cVector3d g_calibFingerPos, g_calibBotPos, g_calibActuatorPos, g_calibTopPos;
cMatrix3d g_calibFingerRot, g_calibBotRot, g_calibActuatorRot, g_calibTopRot;

// 计算得到的变换矩阵
cMatrix3d g_T_finger_to_bot_rot;
cVector3d g_T_finger_to_bot_pos;
cMatrix3d g_T_actuator_to_top_rot;
cVector3d g_T_actuator_to_top_pos;
bool g_calibrationValid = false;

// 显示calibration提示的标签
cLabel* labelCalibrationStatus = nullptr;



//--------------------------------------------------------------
//  保存调零时抓到的基准
//--------------------------------------------------------------

cVector3d g_calibTopInBotRef;

//auto now{ std::chrono::system_clock::now() };
//// Convert the time point to duration in microseconds
//auto duration{ now.time_since_epoch() };
//// Convert to microseconds and then to double seconds
//string timestamp;
// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight* light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a few objects that are placed in the scene
cMesh* base;
cMesh* box;
cMesh* sphere;
cMesh* cone;
cMultiSegment* segments;

// a colored background
cBackground* background;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;
size_t counter = 0;

//------------------------------------------------------------------------------
// DECLARED Arduino Connection
//------------------------------------------------------------------------------
SerialPort* myDevice;
//bool sendPosToArduino{ false };
bool writeForceToCSV{ false };
const char* portNumber = "\\\\.\\COM3";
void arduinoWriteData(unsigned int delay_time, const std::string& send_str);
cVector3d proxyPos;
double posMagnitude;
//------------------------------------------------------------------------------
// DECLARED Control functions 
//------------------------------------------------------------------------------
ControlPCC ResolvedRateControl;
//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

void openCSV(const std::string& filename);
void writeToCSV(const std::vector<string>& row);
void closeCSV();

void printTransform(const cTransform& T, const std::string& name);

std::string currentCsvType = "characterization"; // "characterization" 或 "experiment"


//==============================================================================
/*
	Soft haptic gripper 2025
	Jiaji Su 
	Zonghe Chua
*/
//==============================================================================


int main(int argc, char* argv[])
{
	//--------------------------------------------------------------------------
	// INITIALIZATION
	//--------------------------------------------------------------------------

	cout << endl;
	cout << "-----------------------------------" << endl;
	cout << "CHAI3D" << endl;
	cout << "Jiaji Su Soft Haptic " << endl;
	cout << "Copyright 2003-2016" << endl;
	cout << "-----------------------------------" << endl << endl << endl;
	cout << "Keyboard Options:" << endl << endl;
	cout << "[q] - Exit application" << endl;
	cout << "[b] - Boresight sensor1 (0,0,0)+(0,0,0,1)" << endl;
	cout << "[r] - Clear boresight for sensor1 " << endl;
	cout << "[o] - Open CSV file for writing" << endl;      
	cout << "[w] - Writing sensor1 sensor2 rel_sensor2 data to CSV" << endl; 
	cout << "[k] - Writing scaled force generated by CHAI3D" << endl;
	cout << "[L] - Sart to move the target and record the time" << endl;
	cout << "[E] - Toggle experiment mode" << endl;
	cout << "[T] - Start next trial (experiment mode)" << endl;
	cout << "[1] - Left cube stiffer (experiment mode)" << endl;
	cout << "[2] - Right cube stiffer (experiment mode)" << endl;

	//cout << "[I] - Start communication with Arduino" << endl;
	

	cout << endl << endl;

	//--------------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

	// initialize GLFW library
	if (!glfwInit())
	{
		cout << "failed initialization" << endl;
		cSleepMs(1000);
		return 1;
	}

	// set error callback
	glfwSetErrorCallback(errorCallback);

	// compute desired size of window
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	int w = 0.8 * mode->height;
	int h = 0.5 * mode->height;
	int x = 0.5 * (mode->width - w);
	int y = 0.5 * (mode->height - h);

	// set OpenGL version
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	// set active stereo mode
	if (stereoMode == C_STEREO_ACTIVE)
	{
		glfwWindowHint(GLFW_STEREO, GL_TRUE);
	}
	else
	{
		glfwWindowHint(GLFW_STEREO, GL_FALSE);
	}

	// create display context
	window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
	if (!window)
	{
		cout << "failed to create window" << endl;
		cSleepMs(1000);
		glfwTerminate();
		return 1;
	}

	// get width and height of window
	glfwGetWindowSize(window, &width, &height);

	// set position of window
	glfwSetWindowPos(window, x, y);

	// set key callback
	glfwSetKeyCallback(window, keyCallback);

	// set resize callback
	glfwSetWindowSizeCallback(window, windowSizeCallback);

	// set current display context
	glfwMakeContextCurrent(window);

	// sets the swap interval for the current display context
	glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
	// initialize GLEW library
	if (glewInit() != GLEW_OK)
	{
		cout << "failed to initialize GLEW library" << endl;
		glfwTerminate();
		return 1;
	}
#endif


	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	world->m_backgroundColor.setWhite();

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// position and orient the camera
	camera->set(cVector3d(0.2, 0.05, 0.15),    // camera position (eye)
		cVector3d(0.0, 0.0, 0.015),    // lookat position (target)
		cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

	// set the near and far clipping planes of the camera
	// anything in front or behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 10.0);

	// set stereo mode
	camera->setStereoMode(stereoMode);

	// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoEyeSeparation(0.03);
	camera->setStereoFocalLength(1.8);

	// set vertical mirrored display mode
	camera->setMirrorVertical(mirroredDisplay);

	// create a light source
	light = new cSpotLight(world);

	// attach light to camera
	world->addChild(light);

	// enable light source
	light->setEnabled(true);

	// position the light source
	light->setLocalPos(2, 1, 1.5);

	// define the direction of the light beam
	light->setDir(0, 0, 0);

	// enable this light source to generate shadows
	light->setShadowMapEnabled(true);

	// set the resolution of the shadow map
	//light->m_shadowMap->setQualityLow();
	light->m_shadowMap->setQualityVeryHigh();

	// set light cone half angle
	light->setCutOffAngleDeg(60);


	//--------------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//--------------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get access to the first available haptic device
	handler->getDevice(hapticDevice, 0);

	// retrieve information about the current haptic device
	cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

	// if the haptic devices carries a gripper, enable it to behave like a user switch
	hapticDevice->setEnableGripperUserSwitch(true);

	// create a tool (cursor) and insert into the world
	tool = new cToolCursor(world);
	world->addChild(tool);

	// connect the haptic device to the tool
	tool->setHapticDevice(hapticDevice);

	// map the physical workspace of the haptic device to a larger virtual workspace.
	tool->setWorkspaceRadius(1.0);

	// define the radius of the tool (sphere)
	double toolRadius = 0.002;

	// define a radius for the tool
	tool->setRadius(toolRadius);

	// hide the device sphere. only show proxy.
	tool->setShowContactPoints(true, false);

	//tool->setShowFrame(true);        // 显示坐标轴
	//tool->setFrameSize(0.02);        // 设置坐标轴长度为20mm


	// enable if objects in the scene are going to rotate of translate
	// or possibly collide against the tool. If the environment
	// is entirely static, you can set this parameter to "false"
	tool->enableDynamicObjects(true);

	// haptic forces are enabled only if small forces are first sent to the device;
	// this mode avoids the force spike that occurs when the application starts when 
	// the tool is located inside an object for instance. 
	tool->setWaitForSmallForce(true);

	// start the haptic tool
	tool->start();

	//--------------------------------------------------------------------------
	// ARDUINO CODE
	//--------------------------------------------------------------------------

	cout << "Connecting arduino boards" << endl;

	myDevice = new SerialPort(portNumber);

	if (myDevice->isConnected())
	{
		cout << endl << "Connection Established" << endl;
	}


	//--------------------------------------------------------------------------
	// CREATE OBJECTS
	//--------------------------------------------------------------------------

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

	// stiffness properties
	double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;


	/////////////////////////////////////////////////////////////////////////
	// BASE
	/////////////////////////////////////////////////////////////////////////

	// create a mesh
	cMesh* base = new cMesh();

	// add object to world
	//world->addChild(base);

	// build mesh using a cylinder primitive
	cCreateCylinder(base,
		0.001,
		0.09,
		36,
		1,
		10,
		true,
		true,
		cVector3d(0.0, 0.0, -0.001),
		cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(0), C_EULER_ORDER_XYZ)
	);

	// set material properties
	base->m_material->setBluePaleTurquoise();
	base->m_material->setStiffness(0.5 * maxStiffness);
	// build collision detection tree
	base->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	base->setUseDisplayList(true);


	/////////////////////////////////////////////////////////////////////////
	// BOX
	/////////////////////////////////////////////////////////////////////////

	// create a mesh
	cMesh* box = new cMesh();

	// add object to world
	//world->addChild(box);

	// build mesh using a cylinder primitive
	cCreateBox(box, 0.03, 0.03, 0.03,
		cVector3d(-0.070, -0.020, 0.20), cIdentity3d(), cColorf(0, 1, 0));

	// position object
	box->setLocalPos(0.0, 0.0, 0);

	// set material properties
	box->m_material->setRed();
	box->m_material->setStiffness(0.65 * maxStiffness);

	// build collision detection tree
	box->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	box->setUseDisplayList(true);


	/////////////////////////////////////////////////////////////////////////
	// SPHERE
	/////////////////////////////////////////////////////////////////////////

	// create a mesh
	cMesh* sphere = new cMesh();

	// add object to world
	//world->addChild(sphere);

	// build mesh using a cylinder primitive
	cCreateSphere(sphere,
		0.08, 32U, 32U, cVector3d(0, 0, 0));

	sphere->setLocalPos(0.1, -0.2, 0.1);
	// set material properties
	sphere->m_material->setRedDark();
	sphere->m_material->setStiffness(0.5 * maxStiffness);

	// build collision detection tree
	sphere->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	sphere->setUseDisplayList(true);


	/////////////////////////////////////////////////////////////////////////
	// CONE
	/////////////////////////////////////////////////////////////////////////

	// create a mesh
	cMesh* cone = new cMesh();

	// add object to world
	//base->addChild(cone);

	// build mesh using a cylinder primitive
	cCreateCone(cone,
		0.2,
		0.25,
		0.15,
		32,
		1,
		1,
		true,
		true,
		cVector3d(0, 0, -0.05),
		cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(0), C_EULER_ORDER_XYZ)
	);
	/////////////////////////////////////////////////////////////////////////
    // Local Coordinate
    /////////////////////////////////////////////////////////////////////////
	// set material properties
	cone->m_material->setBluePaleTurquoise();
	cone->m_material->setStiffness(0.95 * maxStiffness);

	// build collision detection tree
	cone->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	cone->setUseDisplayList(true);

	// 改为：
	axisX = new cShapeLine(
		cVector3d(0, 0, 0),
		cVector3d(10, 0, 0)
	);
	axisX->m_material->setRedCrimson();

	axisY = new cShapeLine(
		cVector3d(0, 0, 0),
		cVector3d(0, 0, 0)
	);
	axisY->m_material->setGreenForest();

	axisZ = new cShapeLine(
		cVector3d(0, 0, 0),
		cVector3d(0, 0, 10)
	);
	axisZ->m_material->setBlueRoyal();

	world->addChild(axisX);
	world->addChild(axisY);
	world->addChild(axisZ);

	//leftCube->setLocalPos( 0.02, -0.03, 0.04);
	// 在现有对象创建代码后添加
	// 
/////////////////////////////////////////////////////////////////////////
// CALIBRATION SPHERES
/////////////////////////////////////////////////////////////////////////

// 创建4个球体用于显示calibration
	double calibSphereRadius = 0.003; // 3mm半径

	// Finger marker - 绿色
	sphereFinger = new cShapeSphere(calibSphereRadius);
	sphereFinger->m_material->setGreenForest();
	sphereFinger->setEnabled(false); // 初始隐藏
	sphereFinger->setShowFrame(true);
	sphereFinger->setFrameSize(0.015);  // 15mm长度
	world->addChild(sphereFinger);

	// Actuator marker - 红色
	sphereActuator = new cShapeSphere(calibSphereRadius);
	sphereActuator->m_material->setRedCrimson();
	sphereActuator->setEnabled(false);
	sphereActuator->setShowFrame(true);
	sphereActuator->setFrameSize(0.015);
	world->addChild(sphereActuator);

	// Bot (计算得到) - 蓝色
	sphereBot = new cShapeSphere(calibSphereRadius);
	sphereBot->m_material->setBlueRoyal();
	sphereBot->setEnabled(false);
	sphereActuator->setShowFrame(true);
	sphereActuator->setFrameSize(0.015);
	world->addChild(sphereBot);

	// Top (计算得到) - 黄色
	sphereTop = new cShapeSphere(calibSphereRadius);
	sphereTop->m_material->setYellow();
	sphereTop->setEnabled(false);
	sphereActuator->setShowFrame(true);
	sphereActuator->setFrameSize(0.015);
	world->addChild(sphereTop);


/////////////////////////////////////////////////////////////////////////
// 实验用立方体 (左右两个)
/////////////////////////////////////////////////////////////////////////

// 左立方体 - 改为薄片
	leftCube = new cMesh();
	world->addChild(leftCube);
	// 根据方向创建不同朝向的薄片，初始为Z方向（水平面）
	cCreateBox(leftCube, 0.03, 0.03, 0.001);  // 厚度只有1mm
	leftCube->setLocalPos( 0.04, -0.02, 0.05);
	leftCube->m_material->setGrayLevel(0.7);
	leftCube->m_material->setStiffness(1000);
	leftCube->createAABBCollisionDetector(toolRadius);
	leftCube->setUseDisplayList(true);
	leftCube->setEnabled(false);

	// 右立方体 - 改为薄片
	rightCube = new cMesh();
	world->addChild(rightCube);
	cCreateBox(rightCube, 0.03, 0.03, 0.001);  // 厚度只有1mm
	rightCube->setLocalPos( 0.04, 0.04, 0.05);
	rightCube->m_material->setGrayLevel(0.7);
	rightCube->m_material->setStiffness(1200);
	rightCube->createAABBCollisionDetector(toolRadius);
	rightCube->setUseDisplayList(true);
	rightCube->setEnabled(false);


	//--------------------------------------------------------------------------
	// CREATE SHADERS
	//--------------------------------------------------------------------------

	// create program shader
	cShaderProgramPtr shaderProgram = cShaderProgram::create(C_SHADER_FONG_VERT, C_SHADER_FONG_FRAG);

	// set uniforms
	shaderProgram->setUniformi("uShadowMap", C_TU_SHADOWMAP);

	// assign shader to mesh objects in the world
	tool->setShaderProgram(shaderProgram);
	base->setShaderProgram(shaderProgram);
	box->setShaderProgram(shaderProgram);
	sphere->setShaderProgram(shaderProgram);
	cone->setShaderProgram(shaderProgram);


	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	// create a font
	font = NEW_CFONTCALIBRI40();

	// create a label to display the haptic and graphic rate of the simulation
	labelRates = new cLabel(font);
	labelRates->m_fontColor.setBlack();
	camera->m_frontLayer->addChild(labelRates);

	// create experimental instruction labels
	labelExperimentInstructions = new cLabel(font);
	labelExperimentInstructions->m_fontColor.setBlack();
	labelExperimentInstructions->setLocalPos(20, 50);
	camera->m_frontLayer->addChild(labelExperimentInstructions);
	labelExperimentInstructions->setEnabled(false); // 默认隐藏

	labelTrialInfo = new cLabel(font);
	labelTrialInfo->m_fontColor.setBlue();
	labelTrialInfo->setLocalPos(20, 150);
	camera->m_frontLayer->addChild(labelTrialInfo);
	labelTrialInfo->setEnabled(false); // 默认隐藏

	// 创建calibration状态标签
	labelCalibrationStatus = new cLabel(font);
	labelCalibrationStatus->m_fontColor.setRed();
	labelCalibrationStatus->setLocalPos(20, height - 100);
	camera->m_frontLayer->addChild(labelCalibrationStatus);
	labelCalibrationStatus->setEnabled(false);

	// create a background
	background = new cBackground();
	camera->m_backLayer->addChild(background);

	// set background properties
	background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
		cColorf(1.0f, 1.0f, 1.0f),
		cColorf(0.8f, 0.8f, 0.8f),
		cColorf(0.8f, 0.8f, 0.8f));


	//--------------------------------------------------------------------------
	// START SIMULATION
	//--------------------------------------------------------------------------
	hapticsThread = new cThread();
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
	std::cout << "[Haptics] thread started.\n";
	//-------------------------------------------
	// 1) Turn on micron tracker
	//-------------------------------------------
	try {
		MT.init();                       // 保留原调用
		g_mtAvailable = true;            // ★ 成功则置真

		// 使用CHAI3D线程框架，设置为中等优先级（低于haptic但高于普通线程）
		mtThread = new cThread();
		mtThread->start(MTLoop, CTHREAD_PRIORITY_GRAPHICS); // 或使用 CTHREAD_PRIORITY_HAPTICS-1

		std::cout << "[MT] connected.\n";
	}
	catch (const std::exception& e) {
		std::cerr << "[MT] not detected → run without MT (" << e.what() << ")\n";
		g_mtRunning = false;             // 不启动采集线程
	}
	//MT.zero(28);
	// setup callback when application exits
	atexit(close);

	//--------------------------------------------------------------------------
	// MAIN GRAPHIC LOOP
	//--------------------------------------------------------------------------

	// call window size callback at initialization
	windowSizeCallback(window, width, height);

	// main graphic loop
	while (!glfwWindowShouldClose(window))
	{
		// get width and height of window
		glfwGetWindowSize(window, &width, &height);

		// render graphics
		updateGraphics();

		// swap buffers
		glfwSwapBuffers(window);

		// process events
		glfwPollEvents();

		// signal frequency counter
		freqCounterGraphics.signal(1);
	}

	// close window
	glfwDestroyWindow(window);

	// terminate GLFW library
	glfwTerminate();
	// exit
	return (0);
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
	// update window size
	width = a_width;
	height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
	cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
	// filter calls that only include a key press
	if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
	{
		return;
	}

	// option - exit
	else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
	{
		glfwSetWindowShouldClose(a_window, GLFW_TRUE);
	}

	// option - save shadow map to file
	else if (a_key == GLFW_KEY_S)
	{
		cImagePtr image = cImage::create();
		light->m_shadowMap->copyDepthBuffer(image);
		image->saveToFile("shadowmapshot.png");
		cout << "> Saved screenshot of shadowmap to file.       \r";
	}

	// option - toggle fullscreen
	else if (a_key == GLFW_KEY_F)
	{
		// toggle state variable
		fullscreen = !fullscreen;

		// get handle to monitor
		GLFWmonitor* monitor = glfwGetPrimaryMonitor();

		// get information about monitor
		const GLFWvidmode* mode = glfwGetVideoMode(monitor);

		// set fullscreen or window mode
		if (fullscreen)
		{
			glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
		else
		{
			int w = 0.8 * mode->height;
			int h = 0.5 * mode->height;
			int x = 0.5 * (mode->width - w);
			int y = 0.5 * (mode->height - h);
			glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
	}

	// option - toggle vertical mirroring
	else if (a_key == GLFW_KEY_M)
	{
		mirroredDisplay = !mirroredDisplay;
		camera->setMirrorVertical(mirroredDisplay);
	}
	else if (a_key == GLFW_KEY_O)
	{
		if (experimentMode) {
			// 实验模式下不允许手动创建CSV，只显示状态信息
			std::cout << "=== EXPERIMENT MODE - AUTO CSV MANAGEMENT ===" << std::endl;
			if (userStudy) {
				std::string csvFile = "user_" + currentUserId + "_experiment_data.csv";
				std::cout << "Auto CSV file: " << csvFile << std::endl;
				std::cout << "Data is automatically saved during trials." << std::endl;
				std::cout << "No manual CSV operation needed in experiment mode." << std::endl;
			}
			else {
				std::cout << "No user study session active." << std::endl;
			}
		}
		else {
			// 表征模式下保留原有的手动创建功能
			std::string csvFilename = "Cir_"
				+ std::to_string(circleIndex)
				+ "_"
				+ std::to_string(pointIndex)
				+ ".csv";

			openCSV(csvFilename);
			currentCsvType = "characterization";

			std::cout << "Characterization CSV file ready. Press 'W' to start recording sensor data." << std::endl;
		}
	}
	else if (a_key == GLFW_KEY_K)
	{
		writeForceToCSV = !writeForceToCSV;
		cout << "Writing Status: " << writeForceToCSV << endl;
	}
	else if (a_key == GLFW_KEY_W)
	{
		if (experimentMode) {
			// 实验模式下不允许手动控制记录，只显示状态
			std::cout << "=== EXPERIMENT MODE - AUTO RECORDING ===" << std::endl;
			std::cout << "Recording is automatically managed during trials." << std::endl;
			std::cout << "No manual recording control needed." << std::endl;
			if (userStudy && userStudy->isTrialActive()) {
				std::cout << "Current trial is active - data will be saved on choice." << std::endl;
			}
			else {
				std::cout << "No active trial. Press 'T' to start a trial." << std::endl;
			}
		}
		else {
			// 表征模式下保留原有的手动记录功能
			recordSensorDataToCSV = !recordSensorDataToCSV;
			std::cout << "Record Sensor Data to CSV: " << recordSensorDataToCSV << std::endl;
		}
		}
	else if (a_key == GLFW_KEY_L)
	{
		// Mark the trajectory as started
		g_trajectoryStarted = true;
		// Record the current time as the start reference
		g_trajectoryStartTime = getCurrentTime();
		std::cout << "[INFO] Trajectory started at time=" << g_trajectoryStartTime << std::endl;
	}
	else if (a_key == GLFW_KEY_C)
	{
		g_enableControl = true;
		double g_controlStartTime = getCurrentTime();
		std::cout << "[INFO] Control started at time="
			<< g_controlStartTime << std::endl;
		}
	else if (a_key == GLFW_KEY_I)  // I for Initialize base reference
	{
		if (g_mtAvailable) {
			std::cout << "\n[Main] Initializing base reference frame..." << std::endl;
			if (initBaseReference(MT)) {
				std::cout << "[SUCCESS] Base reference initialized!" << std::endl;
				std::cout << "All coordinates will now be relative to base marker" << std::endl;
				std::cout << "You can now:" << std::endl;
				std::cout << "  - Press 'B' to zero calibrate (relative to base)" << std::endl;
				std::cout << "  - Press 'Z' for marker calibration (relative to base)" << std::endl;
			}
			else {
				std::cerr << "[ERROR] Failed to initialize base reference" << std::endl;
				std::cerr << "Make sure 'base' marker is visible!" << std::endl;
			}
		}
		else {
			std::cout << "[Main] MT not available" << std::endl;
		}
}
		// 在现有按键处理后添加
	else if (a_key == GLFW_KEY_E)
	{
		// 切换实验模式
		experimentMode = !experimentMode;

		if (experimentMode) {
			// 进入实验模式
			if (!userStudy) {
				hasInitPress = false;
				presCurr = ResolvedRateControl.m_initPressure;  // 立即初始化
				// 提示输入用户ID
				std::cout << "\n=== ENTER USER ID ===\n";
				std::cout << "Please enter user ID: ";
				std::cin >> currentUserId;

				userStudy = new UserStudyManager();
				userStudy->loadOrCreateUserSession(currentUserId);
				userStudy->initializeTrials();
			}

			// 隐藏原始对象，显示实验对象
			showOriginalObjects(false);
			showExperimentObjects(true);

			// 设置初始surface朝向（默认Z方向）
			updateSurfaceOrientation(FeedbackDirection::VERTICAL_Z);

			updateExperimentLabels();

			std::cout << "====== EXPERIMENT MODE ON ======" << std::endl;
			std::cout << "Press 'T' to start next trial" << std::endl;
			std::cout << "Press '1' for left surface stiffer, '2' for right surface stiffer" << std::endl;
			std::cout << "Press 'E' again to exit experiment mode" << std::endl;
		}
		else {
			// 退出实验模式
			showExperimentObjects(false);
			showOriginalObjects(true);

			std::cout << "====== EXPERIMENT MODE OFF ======" << std::endl;
			std::cout << "Back to characterization mode" << std::endl;
		}
		}
	else if (a_key == GLFW_KEY_T && experimentMode)
	{
		if (userStudy) {
			// 检查实验状态
			auto state = userStudy->getExperimentState();

			if (state == ExperimentState::EXPERIMENT_COMPLETE) {
				std::cout << "Experiment already complete!" << std::endl;
				return;
			}

			if (state == ExperimentState::IN_PROGRESS) {
				std::cout << "Please make a choice first (press '1' or '2')" << std::endl;
				return;
			}

			// 开始新试次
			userStudy->startNextTrial();

			// 更新surface朝向
			updateSurfaceOrientation(userStudy->getCurrentDirection());

			// 更新显示
			updateExperimentLabels();
		}
		}
	else if (a_key == GLFW_KEY_1 && experimentMode)
	{
		if (userStudy && userStudy->isTrialActive()) {
			userStudy->recordUserChoice(0);  // 这里面已经打印了反应时间
			updateExperimentLabels();

			std::cout << "=== CHOICE RECORDED ===" << std::endl;
			std::cout << "User choice: Left surface stiffer" << std::endl;
			// 删除重复的反应时间打印，因为recordUserChoice已经打印了

			if (userStudy->hasNextTrial()) {
				std::cout << "Press 'T' for next trial" << std::endl;
			}
			else {
				std::cout << "Experiment completed! Press 'E' to exit" << std::endl;
			}
		}
		else {
			std::cout << "No active trial! Press 'T' to start a trial first." << std::endl;
		}
		}

	else if (a_key == GLFW_KEY_2 && experimentMode)
	{
		if (userStudy && userStudy->isTrialActive()) {
			userStudy->recordUserChoice(1);  // 这里面已经打印了反应时间
			updateExperimentLabels();

			std::cout << "=== CHOICE RECORDED ===" << std::endl;
			std::cout << "User choice: Right surface stiffer" << std::endl;
			// 删除重复的反应时间打印，因为recordUserChoice已经打印了

			if (userStudy->hasNextTrial()) {
				std::cout << "Press 'T' for next trial" << std::endl;
			}
			else {
				std::cout << "Experiment completed! Press 'E' to exit" << std::endl;
			}
		}
		else {
			std::cout << "No active trial! Press 'T' to start a trial first." << std::endl;
		}
		}
	// 在keyCallback函数中，现有的实验按键处理后添加：

// 手动设置实验组合的按键 (5-9, 0)
	else if (a_key == GLFW_KEY_5 && experimentMode)
	{
		// Mode 1 (F-P), Direction 1 (X)
		if (userStudy) {
			userStudy->setManualTrialGroup(InteractionMode::FORCE_TO_POSITION, FeedbackDirection::LATERAL_X);
			userStudy->resetCurrentGroup();
			updateSurfaceOrientation(FeedbackDirection::LATERAL_X);
			updateExperimentLabels();
			std::cout << "=== MANUAL GROUP SET ===" << std::endl;
			std::cout << "Mode: F-P, Direction: X (Lateral Left/Right)" << std::endl;
			std::cout << "Press 'T' to start trials" << std::endl;
		}
}
	else if (a_key == GLFW_KEY_6 && experimentMode)
	{
		// Mode 1 (F-P), Direction 2 (Y)
		if (userStudy) {
			userStudy->setManualTrialGroup(InteractionMode::FORCE_TO_POSITION, FeedbackDirection::LATERAL_Y);
			userStudy->resetCurrentGroup();
			updateSurfaceOrientation(FeedbackDirection::LATERAL_Y);
			updateExperimentLabels();
			std::cout << "=== MANUAL GROUP SET ===" << std::endl;
			std::cout << "Mode: F-P, Direction: Y (Lateral Forward/Backward)" << std::endl;
			std::cout << "Press 'T' to start trials" << std::endl;
		}
		}
	else if (a_key == GLFW_KEY_7 && experimentMode)
	{
		// Mode 1 (F-P), Direction 3 (Z)
		if (userStudy) {
			userStudy->setManualTrialGroup(InteractionMode::FORCE_TO_POSITION, FeedbackDirection::VERTICAL_Z);
			userStudy->resetCurrentGroup();
			updateSurfaceOrientation(FeedbackDirection::VERTICAL_Z);
			updateExperimentLabels();
			std::cout << "=== MANUAL GROUP SET ===" << std::endl;
			std::cout << "Mode: F-P, Direction: Z (Vertical Up/Down)" << std::endl;
			std::cout << "Press 'T' to start trials" << std::endl;
		}
		}
	else if (a_key == GLFW_KEY_8 && experimentMode)
	{
		// Mode 2 (F-F), Direction 1 (X)
		if (userStudy) {
			userStudy->setManualTrialGroup(InteractionMode::FORCE_TO_FORCE, FeedbackDirection::LATERAL_X);
			userStudy->resetCurrentGroup();
			updateSurfaceOrientation(FeedbackDirection::LATERAL_X);
			updateExperimentLabels();
			std::cout << "=== MANUAL GROUP SET ===" << std::endl;
			std::cout << "Mode: F-F, Direction: X (Lateral Left/Right)" << std::endl;
			std::cout << "Press 'T' to start trials" << std::endl;
		}
		}
	else if (a_key == GLFW_KEY_9 && experimentMode)
	{
		// Mode 2 (F-F), Direction 2 (Y)
		if (userStudy) {
			userStudy->setManualTrialGroup(InteractionMode::FORCE_TO_FORCE, FeedbackDirection::LATERAL_Y);
			userStudy->resetCurrentGroup();
			updateSurfaceOrientation(FeedbackDirection::LATERAL_Y);
			updateExperimentLabels();
			std::cout << "=== MANUAL GROUP SET ===" << std::endl;
			std::cout << "Mode: F-F, Direction: Y (Lateral Forward/Backward)" << std::endl;
			std::cout << "Press 'T' to start trials" << std::endl;
		}
		}
	else if (a_key == GLFW_KEY_0 && experimentMode)
	{
		// Mode 2 (F-F), Direction 3 (Z)
		if (userStudy) {
			userStudy->setManualTrialGroup(InteractionMode::FORCE_TO_FORCE, FeedbackDirection::VERTICAL_Z);
			userStudy->resetCurrentGroup();
			updateSurfaceOrientation(FeedbackDirection::VERTICAL_Z);
			updateExperimentLabels();
			std::cout << "=== MANUAL GROUP SET ===" << std::endl;
			std::cout << "Mode: F-F, Direction: Z (Vertical Up/Down)" << std::endl;
			std::cout << "Press 'T' to start trials" << std::endl;
		}
		}
	else if (a_key == GLFW_KEY_V)  // V for Verify calibration
	{
		g_showCalibrationSpheres = !g_showCalibrationSpheres;

		if (g_showCalibrationSpheres) {
			std::cout << "[INFO] Calibration verification ON - showing all markers" << std::endl;
			std::cout << "  Green = Finger, Red = Actuator" << std::endl;
			std::cout << "  Blue = Bot (calculated), Yellow = Top (calculated)" << std::endl;
		}
		else {
			std::cout << "[INFO] Calibration verification OFF" << std::endl;
		}

		// 切换显示状态
		if (sphereFinger) sphereFinger->setEnabled(g_showCalibrationSpheres);
		if (sphereActuator) sphereActuator->setEnabled(g_showCalibrationSpheres);
		if (sphereBot) sphereBot->setEnabled(g_showCalibrationSpheres);
		if (sphereTop) sphereTop->setEnabled(g_showCalibrationSpheres);
		}
	else if (a_key == GLFW_KEY_Z)  // Z for Calibration mode
	{
		if (!g_calibrationMode) {
			// 进入calibration模式
			g_calibrationMode = true;
			g_calibStep = CALIB_WAITING_FINGER;
			g_calibrationValid = false;

			std::cout << "\n=== ENTERING CALIBRATION MODE ===" << std::endl;
			std::cout << "Please position markers and press SPACE to record:" << std::endl;
			std::cout << "1. Position FINGER marker and press SPACE" << std::endl;

			// 显示所有球体以便观察
			if (sphereFinger) sphereFinger->setEnabled(true);
			if (sphereActuator) sphereActuator->setEnabled(true);
			if (sphereBot) sphereBot->setEnabled(true);
			if (sphereTop) sphereTop->setEnabled(true);

			labelCalibrationStatus->setEnabled(true);
			labelCalibrationStatus->setText("Calibration: Position FINGER marker and press SPACE");
		}
		else {
			// 退出calibration模式
			g_calibrationMode = false;
			g_calibStep = CALIB_NONE;
			labelCalibrationStatus->setEnabled(false);

			std::cout << "\n=== EXITING CALIBRATION MODE ===" << std::endl;
			if (g_calibrationValid) {
				std::cout << "Calibration saved and will be used for tracking" << std::endl;
			}
			else {
				std::cout << "Calibration incomplete - tracking disabled" << std::endl;
			}
		}
		}
	else if (a_key == GLFW_KEY_SPACE && g_calibrationMode)  // SPACE to record position
	{
		// 在calibration模式下记录当前位置
		if (g_calibStep == CALIB_WAITING_FINGER) {
			// 从MT读取finger marker的变换
			cTransform T_finger;
			if (getMTMarkerTransform("finger", T_finger)) {
				g_calibFingerPos = T_finger.getLocalPos();
				g_calibFingerRot = T_finger.getLocalRot();
				g_calibStep = CALIB_WAITING_BOT;
				std::cout << "Finger position recorded in BASE frame: " << (g_calibFingerPos * 1000).str(3) << " mm" << std::endl;
				std::cout << "2. Place BOT marker at calibration position and press SPACE" << std::endl;
				labelCalibrationStatus->setText("Calibration: Place BOT marker and press SPACE");

				// 更新球体显示
				if (sphereFinger) {
					sphereFinger->setLocalPos(g_calibFingerPos);
					sphereFinger->setLocalRot(g_calibFingerRot);
				}
			}
			else {
				std::cout << "[ERROR] Cannot read finger marker position from MT!" << std::endl;
				std::cout << "Make sure base reference is initialized (press 'I' first)" << std::endl;
			}
		}
		else if (g_calibStep == CALIB_WAITING_BOT) {
			// 从MT读取bot marker的变换
			cTransform T_bot;
			if (getMTMarkerTransform("bot", T_bot)) {
				g_calibBotPos = T_bot.getLocalPos();
				g_calibBotRot = T_bot.getLocalRot();
				g_calibStep = CALIB_WAITING_ACTUATOR;
				std::cout << "Bot position recorded in BASE frame: " << (g_calibBotPos * 1000).str(3) << " mm" << std::endl;
				std::cout << "3. Position ACTUATOR marker and press SPACE" << std::endl;
				labelCalibrationStatus->setText("Calibration: Position ACTUATOR marker and press SPACE");

				// 更新球体显示
				if (sphereBot) {
					sphereBot->setLocalPos(g_calibBotPos);
					sphereBot->setLocalRot(g_calibBotRot);
					sphereBot->setEnabled(true);
					sphereBot->setShowFrame(true);
					sphereBot->setFrameSize(0.015);
				}
			}
			else {
				std::cout << "[ERROR] Cannot read bot marker position from MT!" << std::endl;
				std::cout << "Make sure 'bot' marker is visible and base reference is valid" << std::endl;
			}
		}
		else if (g_calibStep == CALIB_WAITING_ACTUATOR) {
			// 从MT读取actuator marker的变换
			cTransform T_actuator;
			if (getMTMarkerTransform("actuator", T_actuator)) {
				g_calibActuatorPos = T_actuator.getLocalPos();
				g_calibActuatorRot = T_actuator.getLocalRot();
				g_calibStep = CALIB_WAITING_TOP;
				std::cout << "Actuator position recorded in BASE frame: " << (g_calibActuatorPos * 1000).str(3) << " mm" << std::endl;
				std::cout << "4. Place TOP marker at calibration position and press SPACE" << std::endl;
				labelCalibrationStatus->setText("Calibration: Place TOP marker and press SPACE");

				// 更新球体显示
				if (sphereActuator) {
					sphereActuator->setLocalPos(g_calibActuatorPos);
					sphereActuator->setLocalRot(g_calibActuatorRot);
				}
			}
			else {
				std::cout << "[ERROR] Cannot read actuator marker position from MT!" << std::endl;
				std::cout << "Make sure 'actuator' marker is visible and base reference is valid" << std::endl;
			}
		}
		else if (g_calibStep == CALIB_WAITING_TOP) {
			std::cout << "[DEBUG] Attempting to read top marker in BASE frame..." << std::endl;

			// 先检查sphereTop是否存在
			if (!sphereTop) {
				std::cerr << "[ERROR] sphereTop is null!" << std::endl;
				return;
			}

			// 尝试读取top marker的变换
			cTransform T_top;
			if (getMTMarkerTransform("top", T_top)) {
				g_calibTopPos = T_top.getLocalPos();
				g_calibTopRot = T_top.getLocalRot();

				std::cout << "Top position recorded in BASE frame: " << (g_calibTopPos * 1000).str(3) << " mm" << std::endl;

				// 更新球体显示
				sphereTop->setLocalPos(g_calibTopPos);
				sphereTop->setLocalRot(g_calibTopRot);
				sphereTop->setEnabled(true);
				sphereTop->setShowFrame(true);
				sphereTop->setFrameSize(0.015);

				// 计算变换矩阵（全部在VE坐标系下）
				try {
					// 使用cTransform计算相对变换
					// finger_T_bot: bot在finger坐标系下的位姿
					cTransform world_T_finger(
						g_calibFingerPos, g_calibFingerRot
					);
					cTransform world_T_bot(
						g_calibBotPos, g_calibBotRot);

					// 计算finger的逆变换
					cTransform finger_T_world(world_T_finger);
					finger_T_world.invert();

					// finger_T_bot = T_finger^-1 * T_bot
					cTransform finger_T_bot = finger_T_world * world_T_bot;
					g_T_finger_to_bot_pos = finger_T_bot.getLocalPos();
					g_T_finger_to_bot_rot = finger_T_bot.getLocalRot();

					// actuator_T_top: top在actuator坐标系下的位姿
					cTransform world_T_actuator(
						g_calibActuatorPos, g_calibActuatorRot);
					cTransform world_T_top(
						g_calibTopPos, g_calibTopRot);

					// 计算actuator的逆变换
					cTransform actuator_T_world(world_T_actuator);
					actuator_T_world.invert();

					// actuator_T_top = T_actuator^-1 * T_top
					cTransform actuator_T_top = actuator_T_world * world_T_top;
					g_T_actuator_to_top_pos = actuator_T_top.getLocalPos();
					g_T_actuator_to_top_rot = actuator_T_top.getLocalRot();

					// 验证刚体约束
					cTransform T_bot_verify = world_T_finger * finger_T_bot;
					cTransform T_top_verify = world_T_actuator * actuator_T_top;

					cVector3d botPosTest = T_bot_verify.getLocalPos();
					cVector3d topPosTest = T_top_verify.getLocalPos();

					double botError = (botPosTest - g_calibBotPos).length() * 1000;
					double topError = (topPosTest - g_calibTopPos).length() * 1000;

					std::cout << "[VALIDATION] Bot reconstruction error: " << botError << " mm (should be ~0)" << std::endl;
					std::cout << "[VALIDATION] Top reconstruction error: " << topError << " mm (should be ~0)" << std::endl;

					// 计算参考值 - 使用cTransform
					cTransform T_bot_inv = world_T_bot;
					T_bot_inv.invert();
					cTransform T_top_in_bot = T_bot_inv * world_T_top;
					g_calibTopInBotRef = T_top_in_bot.getLocalPos();

					g_calibrationValid = true;
					g_calibStep = CALIB_COMPLETE;

					std::cout << "[CALIBRATION] Complete. Reference Top in Bot: " << (g_calibTopInBotRef * 1000).str(3) << " mm" << std::endl;
					labelCalibrationStatus->setText("Calibration COMPLETE! Press 'Z' to exit, 'V' to verify");
				}
				catch (const std::exception& e) {
					std::cerr << "[CALIB-ERROR] " << e.what() << std::endl;
					g_calibrationValid = false;
					labelCalibrationStatus->setText("Calibration FAILED!");
				}
			}
			else {
				std::cout << "[ERROR] Cannot read top marker position from MT!" << std::endl;
				std::cout << "Possible reasons:" << std::endl;
				std::cout << "1. Base reference not initialized (press 'I' first)" << std::endl;
				std::cout << "2. The marker 'top' is not visible to the camera" << std::endl;
				std::cout << "3. The marker template 'top' is not loaded" << std::endl;
				std::cout << "4. The marker name might be case-sensitive (try 'Top' or 'TOP')" << std::endl;
			}
		}
	}



}

//------------------------------------------------------------------------------

void close(void)
{
	// stop the simulationg_mtAvailable
	simulationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!simulationFinished) { cSleepMs(100); }

	// close haptic device
	tool->stop();
	myDevice->closeSerial();
	// delete resources
	delete hapticsThread;
	delete world;
	delete handler;
	closeCSV();
	g_mtRunning = false;
	if (g_mtAvailable && mtThread != nullptr)
	{
		mtThread->stop();      // 使用CHAI3D线程的停止方法
		delete mtThread;       // 释放线程对象
		mtThread = nullptr;
	}
	if (axisX) { delete axisX; axisX = nullptr; }
	if (axisY) { delete axisY; axisY = nullptr; }
	if (axisZ) { delete axisZ; axisZ = nullptr; }
	if (sphereTarget) { delete sphereTarget; sphereTarget = nullptr; }
	if (cylinderTop) { delete cylinderTop; cylinderTop = nullptr; }
	if (cylinderBot) { delete cylinderBot; cylinderBot = nullptr; }
	if (cylinderTopRel) { delete cylinderTopRel; cylinderTopRel = nullptr; }

	// 在现有清理代码后添加
	if (userStudy) {
		delete userStudy;
		userStudy = nullptr;
	}
	// 清理实验标签
	if (labelExperimentInstructions) {
		delete labelExperimentInstructions;
		labelExperimentInstructions = nullptr;
	}
	if (labelTrialInfo) {
		delete labelTrialInfo;
		labelTrialInfo = nullptr;
	}
	// 清理calibration球体
	if (sphereFinger) { delete sphereFinger; sphereFinger = nullptr; }
	if (sphereActuator) { delete sphereActuator; sphereActuator = nullptr; }
	if (sphereBot) { delete sphereBot; sphereBot = nullptr; }
	if (sphereTop) { delete sphereTop; sphereTop = nullptr; }
	closeCSV();
	//closeExperimentCSV();
	if (labelCalibrationStatus) {
		delete labelCalibrationStatus;
		labelCalibrationStatus = nullptr;
	}
}

//------------------------------------------------------------------------------
void MTLoop()
{
	while (g_mtRunning)
	{	
		if (!g_mtAvailable) return;
		if (!g_calibrationValid) continue;
		if (!MT.update()) continue;           // 阻塞在 SDK FPS

		MTPoseSnapshot snap;                  // 本地暂存
		snap.haveFr = MT.getPoseAndRotMat("finger", snap.fr);
		snap.haveAct = MT.getPoseAndRotMat("actuator", snap.act);
		snap.haveRel = (snap.haveFr && snap.haveAct &&
			MT.getRelPos("actuator", "finger", snap.relPos) &&
			MT.getRelRotMat("actuator", "finger", snap.relR));
		snap.ts = MT.getLastTimestamp();

		// 复制到共享区
		{
			std::lock_guard<std::mutex> lk(g_mtMtx);
			g_mtSnap = snap;
		}
	}
}



void updateGraphics(void)
{
	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////

	// update haptic and graphic rate data
	labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
		cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

	// update position of label
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	// render world
	camera->renderView(width, height);

	// wait until all GL commands are completed
	glFinish();

	// check for any OpenGL errors
	GLenum err = glGetError();
	if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

enum cMode
{
	IDLE,
	SELECTION
};

void updateHaptics(void)
{
	cMode state = IDLE;
	cGenericObject* object = NULL;
	cTransform tool_T_object;
	// ★ 添加这三行声明
	PNODATA pno_1{};           // 用于存储finger marker数据
	PNODATA pno_2{};           // 用于存储actuator marker数据  
	PNODATA sensor2CrctPNO{};  // 用于存储相对位姿数据

	// ========== 初始化部分（只执行一次）==========
	static bool hapticsInitialized = false;
	if (!hapticsInitialized) {
		// add top and bot cap
		double radius = 0.005; // 5mm
		double height = 0.001; // 1mm

		// 创建 cylinderTop
		if (!cylinderTop) {
			cylinderTop = new cShapeCylinder(radius, radius, height);
			cylinderTop->m_material->setGreen();
			// 注意：cylinderTop 不添加到 world（根据你的原代码）
		}

		// 创建 cylinderBot
		if (!cylinderBot) {
			cylinderBot = new cShapeCylinder(radius, radius, height);
			cylinderBot->m_material->setRed();
			world->addChild(cylinderBot);
		}

		// 创建 cylinderTopRel
		double relRadius = 0.005;
		double relHeight = 0.001;
		if (!cylinderTopRel) {
			cylinderTopRel = new cShapeCylinder(relRadius, relRadius, relHeight);
			cylinderTopRel->m_material->setGreenForest();
			cylinderTopRel->setLocalPos(0, 0, 0.0);
			world->addChild(cylinderTopRel);
		}

		// 创建 sphereTarget
		double sphereRadius = 0.0025; // 小球半径
		if (!sphereTarget) {
			sphereTarget = new cShapeSphere(sphereRadius);
			sphereTarget->m_material->setBlueCornflower();
			sphereTarget->m_material->setTransparencyLevel(0.6);
			sphereTarget->setUseTransparency(true);
			world->addChild(sphereTarget);
		}

		hapticsInitialized = true;
	}

	// ========== 以下是需要每次循环都执行的代码 ==========

	// generate targets on circles
	std::vector<TrajectoryGenerator::CircleDefinition> circles = {
		{0.0035, chai3d::cVector3d(0.0, 0.0, ResolvedRateControl.h_0 * 0.001 + 0.003)},
		{0.0045, chai3d::cVector3d(0.0, 0.0, ResolvedRateControl.h_0 * 0.001 - 0.006)},
		{0.0055, chai3d::cVector3d(0.0, 0.0, ResolvedRateControl.h_0 * 0.001 - 0.0012)}
	};

	int points = 12;
	std::vector<std::vector<chai3d::cVector3d>> allCircles =
		TrajectoryGenerator::generateMultipleCirclesPoints(circles, points);

	cVector3d startPos(0.0, 0.0, ResolvedRateControl.h_0 * 0.001);

	// 我们定义一个"sensor2PosOffset"，用于让 sensor2 在无驱动时变成 (0,0,h_0)
	static cVector3d sensor2PosOffset(0.0, 0.0, 0.0);

	chai3d::cVector3d PointOnCir = allCircles[circleIndex][pointIndex];

	static const chai3d::cVector3d center(0.0, 0.0, ResolvedRateControl.h_0 * 0.001);   // 起始点
	static const chai3d::cVector3d amp(0.002, 0.000, 0.00); // 幅值
	double freq = 1;                      // 1 Hz 带宽测试

	// simulation in now running
	simulationRunning = true;
	simulationFinished = false;

	// main haptic simulation loop
	while (simulationRunning)
	{
		/////////////////////////////////////////////////////////////////////////
		// HAPTIC RENDERING
		/////////////////////////////////////////////////////////////////////////

		// signal frequency counter
		freqCounterHaptics.signal(1);

		// compute global reference frames for each object
		world->computeGlobalPositions(true);

		// update position and orientation of tool



		// 	/////////////////////////////////////////////////////////////////////////
		// Define Parameters for a fixed trajectory 
		/////////////////////////////////////////////////////////////////////////
		// -----------------------------Get Time --------------------------------------
		double absoluteTime = getCurrentTime();

		 double elapsedTime = 0.0;
		 if (g_trajectoryStarted)
		 {
			 // if started, compute time since we pressed 'R'
			 elapsedTime = absoluteTime - g_trajectoryStartTime;
			 // clamp at >= 0
			 if (elapsedTime < 0.0) elapsedTime = 0.0;
		 }

		//-----------------------------Generate a circular traj---------------------------

	    //double radius = 0.5;
		//chai3d::cVector3d center(0.0, 0.0, 0);
		//double angularSpeed = 1; 
		////std::cout << "Current time since program start: " << currentTime << " s" << std::endl;
		//// call tajectory generator to get a circal 
		//chai3d::cVector3d TrajTarget =
		//	TrajectoryGenerator::getCircularTrajectory(radius, center, angularSpeed, currentTime);
		//sphereTarget->setLocalPos(TrajTarget);
		//-----------------------------Gnerate a straight traj---------------------------
		// 
		//chai3d::cVector3d endPos(0, 0.00001, 0.033);
		//chai3d::cVector3d TrajTarget = TrajectoryGenerator::getStraightLineTrajectory(
		//	startPos,
		//	endPos,
		//	elapsedTime,
		//	duration
		//);
		//-----------------------------Gnerate a Sinewave traj---------------------------
		 chai3d::cVector3d TrajTarget;          // ① 提前声明
		 if (g_trajectoryStarted)               // ② 只有开始后才计算正弦
		 {
			 TrajTarget = TrajectoryGenerator::getSineWaveTrajectory(
				 center, amp, freq, elapsedTime);  // 默认 11 周期
		 }
		 else
		 {
			 TrajTarget = center;               // 未开始时保持在中心
		 }

		//-----------------------------Gnerate On Circle traj---------------------------
		//chai3d::cVector3d TrajTarget = TrajectoryGenerator::getTransitionPosition(
		//	startPos,       // start point
		//	PointOnCir,      // target point on the circle
		//	elapsedTime,
		//	duration
		//);




		sphereTarget->setLocalPos(TrajTarget);


		/////////////////////////////////////////////////////////////////////////
		/*
		Control Algrithm Start
		Control Soft robot
		*/
		/////////////////////////////////////////////////////////////////////////

		proxyPos = tool->getDeviceLocalForce();
		try
		{
			posMagnitude = proxyPos.length();
			// Remap function for Jiaji
			posMagnitude = (posMagnitude - 0) * (3 - 0) / (20 - 0) + 0;
			proxyPos.normalize();
		}
		catch (const std::exception&)
		{
			cout << "Touch some thing" << endl;
		}


		//-----------------------------Open loop Control---------------------------

		//-----------------------------Internal iteration-------------------------
		//// Spring Kp = 4  
		////ResolvedRateControl.reachTarget(proxyPos * posMagnitude * 4 + ResolvedRateControl.m_initCoord);
		//ResolvedRateControl.reachTarget(TrajTarget * 1000);
		////  reachTarget 
		//chai3d::cVector3d Current_pressure = ResolvedRateControl.getDevicePressure();
		//std::cout << "Current Pressure Combination: "
		//	<< Current_pressure.str()  // 
		//	<< std::endl;
		//-----------------------------External iteration-------------------------

		if (!hasInitPress)
		{
			presCurr = ResolvedRateControl.m_initPressure;  // (20, 20, 20)
			hasInitPress = true;       //
		}
		cVector3d targetPos = TrajTarget * 1000;
		auto result = ResolvedRateControl.updateMotion(presCurr, targetPos);
		chai3d::cVector3d newPos = result[0];
		chai3d::cVector3d newPressure = result[1];
		double error = (newPos - targetPos).length();
		if (error < 0.1)
		{
			cVector3d PressuretoArduino = presCurr;
			//cVector3d PressuretoArduino(20,20,20);
			std::string sendStr = PressuretoArduino.str();
			arduinoWriteData(50, sendStr);
			//std::cout << "sendStr: "
			//	<< sendStr << std::endl;
		}
		else
		{
			presCurr = newPressure; 
		}

		////------------------------------------------------------------
		//// 1) 读取 fr / Actuator 位姿，先检查返回值
		////------------------------------------------------------------
		mtw::Pose fr, act;
		//――――― 读取最新快照（非阻塞，仅拷贝一次）―――――
		MTPoseSnapshot snap;
		if (g_mtAvailable) {
			std::lock_guard<std::mutex> lk(g_mtMtx);
			snap = g_mtSnap;
		}
		else {
			snap.haveFr = false;   // ★ 无 MT 时后面直接跳过
		}
		// 若无新数据，直接进入下个循环
		if (!snap.haveFr) {
			cSleepMs(0);   // 让线程保持 1 kHz 以上
			goto FINISH_HAPTIC_CYCLE;
		}

		// 把原本 okFr/okAct/relPos/relRot 的逻辑改成走 snap
		fr = snap.fr;
		act = snap.act;
		double relPos[3];  double relRot[9];
		if (snap.haveRel) {
			std::memcpy(relPos, snap.relPos, sizeof(relPos));
			std::memcpy(relRot, snap.relR, sizeof(relRot));
		}
		bool okFr = snap.haveFr;
		bool okAct = snap.haveAct;
		bool okRel = snap.haveRel;

		// 保存原始相机坐标（用于base转换）
		double frCamPos[3] = { fr.pos[0], fr.pos[1], fr.pos[2] };
		double frCamRot[9];
		std::memcpy(frCamRot, fr.rot, 9 * sizeof(double));

		double actCamPos[3] = { act.pos[0], act.pos[1], act.pos[2] };
		double actCamRot[9];
		std::memcpy(actCamRot, act.rot, 9 * sizeof(double));

		////------------------------------------------------------------
		//// 4) 处理位置和旋转（使用新的变换函数）
		////------------------------------------------------------------

		// 获取finger的变换
		cTransform T_finger_ve;
		T_finger_ve = transformCameraToBase(frCamPos, frCamRot);

		// 获取actuator的变换
		cTransform T_actuator_ve;
		T_actuator_ve = transformCameraToBase(actCamPos, actCamRot);

		// 处理相对位姿
		//cTransform T_rel_ve;
		//if (okRel) {
		//	if (g_camera_T_base.valid) {
		//		// 相对位姿也需要进行坐标系变换
		//		cTransform T_rel_base;
		//		cVector3d posRelBase(relPos[0] * 0.001, relPos[1] * 0.001, relPos[2] * 0.001);
		//		cMatrix3d rotRelBase;
		//		rotRelBase.set(relRot[0], relRot[3], relRot[6],
		//			relRot[1], relRot[4], relRot[7],
		//			relRot[2], relRot[5], relRot[8]);
		//		T_rel_base.setLocalPos(posRelBase);
		//		T_rel_base.setLocalRot(rotRelBase);

		//		// 应用base到VE的变换
		//		T_rel_ve = transformBaseToVE(T_rel_base);
		//	}
		//	else {
		//		cVector3d pos(relPos[0] * 0.001, relPos[1] * 0.001, relPos[2] * 0.001);
		//		cMatrix3d rot;
		//		rot.set(relRot[0], relRot[3], relRot[6],
		//			relRot[1], relRot[4], relRot[7],
		//			relRot[2], relRot[5], relRot[8]);
		//		T_rel_ve.setLocalPos(pos);
		//		T_rel_ve.setLocalRot(rot);
		//	}
		//}

		////------------------------------------------------------------
		//// 6) 渲染
		////------------------------------------------------------------

		// 获取finger的位置和旋转用于渲染
		cVector3d posFrM = T_finger_ve.getLocalPos();
		cMatrix3d rotFrM = T_finger_ve.getLocalRot();

		// 更新显示对象
		cylinderBot->setLocalPos(0, 0.107, 0.07);
		cylinderBot->setLocalRot(rotFrM);

		// 更新tool位姿
		tool->setDeviceLocalPos(posFrM);
		tool->setDeviceLocalRot(rotFrM);
		//tool->setLocalPos(posFrM);     // ← 添加此行
		//tool->setLocalRot(rotFrM);      // ← 添加此行
		//tool->setShowFrame(true);       // ← 添加此行，每帧都设置
		//tool->setFrameSize(0.03);       // ← 添加此行，确保大小合适

		tool->updateToolImagePosition();
		tool->computeInteractionForces();

		// -- rel渲染 --
		//if (okRel) {
		//	cVector3d posRelM = T_rel_ve.getLocalPos();
		//	cMatrix3d rotRelM = T_rel_ve.getLocalRot();

		//	// 添加偏移
		//	posRelM += cVector3d(0, 0.1, 0.07);

		//	cylinderTopRel->setEnabled(true);
		//	cylinderTopRel->setLocalPos(posRelM);
		//	cylinderTopRel->setLocalRot(rotRelM);

		//	// 7) 构造 PNODATA（需要原始mm单位的数据）
		//	// 为了构造PNODATA，我们需要获取base坐标系下的原始数据（mm单位）
		//	if (okRel) {
		//		// pno_1 ← fr
		//		if (g_camera_T_base.valid) {
		//			// 获取base坐标系下的数据
		//			cTransform T_finger_base = transformCameraToBase(frCamPos, frCamRot);
		//			cVector3d pos_mm = T_finger_base.getLocalPos() * 1000;  // m转mm
		//			cMatrix3d rot = T_finger_base.getLocalRot();

		//			double posFr[3] = { pos_mm.x(), pos_mm.y(), pos_mm.z() };
		//			double RFr[9];
		//			for (int r = 0; r < 3; r++) {
		//				for (int c = 0; c < 3; c++) {
		//					RFr[c * 3 + r] = rot(r, c);  // 转为列主序
		//				}
		//			}
		//			pno_1 = makePNO_mm_colRM(posFr, RFr);
		//		}
		//		else {
		//			double posFr[3] = { fr.pos[0], fr.pos[1], fr.pos[2] };
		//			double RFr[9];
		//			for (int i = 0; i < 9; ++i) RFr[i] = fr.rot[i];
		//			pno_1 = makePNO_mm_colRM(posFr, RFr);
		//		}

		//		// pno_2 ← act（若 okAct）
		//		if (okAct) {
		//			if (g_camera_T_base.valid) {
		//				cTransform T_actuator_base = transformCameraToBase(actCamPos, actCamRot);
		//				cVector3d pos_mm = T_actuator_base.getLocalPos() * 1000;
		//				cMatrix3d rot = T_actuator_base.getLocalRot();

		//				double posAct[3] = { pos_mm.x(), pos_mm.y(), pos_mm.z() };
		//				double RAct[9];
		//				for (int r = 0; r < 3; r++) {
		//					for (int c = 0; c < 3; c++) {
		//						RAct[c * 3 + r] = rot(r, c);
		//					}
		//				}
		//				pno_2 = makePNO_mm_colRM(posAct, RAct);
		//			}
		//			else {
		//				double posAct[3] = { act.pos[0], act.pos[1], act.pos[2] };
		//				double RAct[9];
		//				for (int i = 0; i < 9; ++i) RAct[i] = act.rot[i];
		//				pno_2 = makePNO_mm_colRM(posAct, RAct);
		//			}
		//		}

		//		// sensor2CrctPNO ← rel
		//		double posRel[3] = { relPos[0], relPos[1], relPos[2] };
		//		double RRel[9];
		//		for (int i = 0; i < 9; ++i) RRel[i] = relRot[i];
		//		sensor2CrctPNO = makePNO_mm_colRM(posRel, RRel);
		//	}
		//

		//	//------------------------------P to L model---------------------------------

		//	//cVector3d targetPos = TrajTarget * 1000;  // 目标位置 (mm)
		//	//if (g_enableControl)
		//	//{
		//	//	//std::cout << std::fixed << std::setprecision(2)
		//	//	//	<< "[sensor2CrctPNOArc] (" << sensor2CrctPNOArc.pos[0] << ", "
		//	//	//	<< sensor2CrctPNOArc.pos[1] << ", "
		//	//	//	<< sensor2CrctPNOArc.pos[2] << ") mm    "
		//	//	//	<< "[targetPos] ( "
		//	//	//	<< targetPos << " ) mm\r";
		//	//	auto result = ResolvedRateControl.updateMotionCloseLoop(targetPos, sensor2CrctPNO);
		//	//	if (!std::isfinite(frZeroPos[0])) {
		//	//		std::cerr << " Invalid data detected, skip frame\n";
		//	//		continue;
		//	//	}
		//	//	//std::cout << "   targetPos: " << targetPos << "   RELPos: " << sensor2CrctPNO.pos[0] << ", " << sensor2CrctPNO.pos[1] << ", " << sensor2CrctPNO.pos[2] << std::endl;
		//	//	cVector3d newPos = result[0];       // 计算的新位置
		//	//	cVector3d newPressure = result[1];  // 计算的新压力
		//	//	double error = (newPos - targetPos).length();
		//	//	//std::cout << "Error: " << error << " = " << newPos << " - " << targetPos << std::endl;
		//	//	//std::cout << "newpos " << newPos << std::endl;
		//	//	cVector3d PressuretoArduino = presCurr;
		//	//	//cVector3d PressuretoArduino(20,20,20);
		//	//	std::string sendStr = PressuretoArduino.str();
		//	//	arduinoWriteData(50, sendStr);
		//	//	//std::cout << "sendStr: "
		//	//	//	<< sendStr << "\r";
		//	//	presCurr = newPressure; // 
		//	//}




		//	if (recordSensorDataToCSV)
		//	{
		//		// ① 取 MicronTracker 时间戳（秒）
		//		double ts = MT.getLastTimestamp();   // 秒
		//		// ② 拼装 CSV 行
		//		std::vector<std::string> row;
		//		row.emplace_back(std::to_string(ts));                // 时间戳

		//		// -- fr --
		//		row.emplace_back(std::to_string(pno_1.pos[0]));
		//		row.emplace_back(std::to_string(pno_1.pos[1]));
		//		row.emplace_back(std::to_string(pno_1.pos[2]));
		//		row.emplace_back(std::to_string(pno_1.ori[0]));
		//		row.emplace_back(std::to_string(pno_1.ori[1]));
		//		row.emplace_back(std::to_string(pno_1.ori[2]));
		//		row.emplace_back(std::to_string(pno_1.ori[3]));

		//		// -- Actuator --
		//		row.emplace_back(std::to_string(pno_2.pos[0]));
		//		row.emplace_back(std::to_string(pno_2.pos[1]));
		//		row.emplace_back(std::to_string(pno_2.pos[2]));
		//		row.emplace_back(std::to_string(pno_2.ori[0]));
		//		row.emplace_back(std::to_string(pno_2.ori[1]));
		//		row.emplace_back(std::to_string(pno_2.ori[2]));
		//		row.emplace_back(std::to_string(pno_2.ori[3]));

		//		// -- Rel (Act → fr) --
		//		row.emplace_back(std::to_string(sensor2CrctPNO.pos[0]));
		//		row.emplace_back(std::to_string(sensor2CrctPNO.pos[1]));
		//		row.emplace_back(std::to_string(sensor2CrctPNO.pos[2]));
		//		row.emplace_back(std::to_string(sensor2CrctPNO.ori[0]));
		//		row.emplace_back(std::to_string(sensor2CrctPNO.ori[1]));
		//		row.emplace_back(std::to_string(sensor2CrctPNO.ori[2]));
		//		row.emplace_back(std::to_string(sensor2CrctPNO.ori[3]));

		//		row.emplace_back(std::to_string(TrajTarget.x()));   // ★
		//		row.emplace_back(std::to_string(TrajTarget.y()));   // ★
		//		row.emplace_back(std::to_string(TrajTarget.z()));   // ★
		//		// ③ 写入
		//		writeToCSV(row);
		//	}




		//}
		//else
		//{
		//	cylinderTopRel->setEnabled(false);      // 隐藏，避免用垃圾值渲染
		//}


		// ★ 将calibration验证代码放在这里 ★
		// 在获取到snap数据之后，添加calibration验证代码
		if (g_showCalibrationSpheres && snap.haveFr && snap.haveAct) {
			// 获取finger和actuator的原始位姿
			mtw::Pose fingerPose = snap.fr;      // fr对应finger
			mtw::Pose actuatorPose = snap.act;   // act对应actuator

			// ★ 使用新的变换函数
			{
				// 获取finger和actuator的完整变换（从相机到VE）
				cTransform T_finger_ve = transformCameraToBase(fingerPose.pos, fingerPose.rot);
				cTransform T_actuator_ve = transformCameraToBase(actuatorPose.pos, actuatorPose.rot);

				// 更新直接追踪到的marker位置（绿色和红色球）
				if (sphereFinger) {
					sphereFinger->setLocalPos(T_finger_ve.getLocalPos());
					sphereFinger->setLocalRot(T_finger_ve.getLocalRot());
				}
				if (sphereActuator) {
					sphereActuator->setLocalPos(T_actuator_ve.getLocalPos());
					sphereActuator->setLocalRot(T_actuator_ve.getLocalRot());
				}

				// ★ 使用calibration数据计算bot和top位置
				if (g_calibrationValid) {
					// 构建finger到bot的变换
					cTransform T_finger_to_bot;
					T_finger_to_bot.setLocalPos(g_T_finger_to_bot_pos);
					T_finger_to_bot.setLocalRot(g_T_finger_to_bot_rot);

					// 构建actuator到top的变换
					cTransform T_actuator_to_top;
					T_actuator_to_top.setLocalPos(g_T_actuator_to_top_pos);
					T_actuator_to_top.setLocalRot(g_T_actuator_to_top_rot);

					// 计算bot和top的世界位置
					cTransform world_T_bot = T_finger_ve * T_finger_to_bot;
					cTransform world_T_top = T_actuator_ve * T_actuator_to_top;

					if (sphereBot) {
						sphereBot->setLocalPos(world_T_bot.getLocalPos());
						sphereBot->setLocalRot(world_T_bot.getLocalRot());
					}

					if (sphereTop) {
						sphereTop->setLocalPos(world_T_top.getLocalPos());
						sphereTop->setLocalRot(world_T_top.getLocalRot());
					}

					//cTransform bot_T_world(world_T_bot);
					//bot_T_world.invert();

					//cTransform bot_T_top = bot_T_world * world_T_top;
					//printTransform(bot_T_top, "bot_T_top");
				}
			}
		}

		/////////////////////////////////////////////////////////////////////////
		// 实验模式处理
		/////////////////////////////////////////////////////////////////////////
if (experimentMode && userStudy && userStudy->isTrialActive()) {
	// 显示立方体
	leftCube->setEnabled(true);
	rightCube->setEnabled(true);

	// ★ 每次进入实验模式都检查并修复 NaN
	if (!std::isfinite(presCurr.x()) || !std::isfinite(presCurr.y()) || !std::isfinite(presCurr.z())) {
		presCurr = ResolvedRateControl.m_initPressure;
		std::cout << "[EXP] Detected NaN in pressure, reset to: " << presCurr.str() << std::endl;
	}

	// 如果还没初始化，也要初始化
	if (!hasInitPress) {
		presCurr = ResolvedRateControl.m_initPressure;
		hasInitPress = true;
		std::cout << "[EXP] Initial pressure set to: " << presCurr.str() << std::endl;
	}

	// 根据当前试次设置刚度
	double refStiffness = userStudy->getCurrentReferenceStiffness();
	double compStiffness = userStudy->getCurrentComparisonStiffness();

	// 随机分配左右位置
	static bool leftIsReference = (rand() % 2 == 0);
	if (leftIsReference) {
		leftCube->m_material->setStiffness(refStiffness);
		rightCube->m_material->setStiffness(compStiffness);
	}
	else {
		leftCube->m_material->setStiffness(compStiffness);
		rightCube->m_material->setStiffness(refStiffness);
	}

	// 检测立方体交互
	bool currentlyTouchingLeft = false;
	bool currentlyTouchingRight = false;

	if (tool->m_hapticPoint->getNumCollisionEvents() > 0) {
		cCollisionEvent* event = tool->m_hapticPoint->getCollisionEvent(0);
		if (event->m_object == leftCube) {
			currentlyTouchingLeft = true;
			if (!isInteractingWithLeftCube) {
				userStudy->recordTouch(true);
				isInteractingWithLeftCube = true;
			}
		}
		else if (event->m_object == rightCube) {
			currentlyTouchingRight = true;
			if (!isInteractingWithRightCube) {
				userStudy->recordTouch(false);
				isInteractingWithRightCube = true;
			}
		}
	}

	// 更新交互状态
	if (!currentlyTouchingLeft) isInteractingWithLeftCube = false;
	if (!currentlyTouchingRight) isInteractingWithRightCube = false;

	/////////////////////////////////////////////////////////////////////////
	// 用户实验模式下的力-位移控制
	/////////////////////////////////////////////////////////////////////////
	// 获取触觉设备的交互力（全局坐标系）
	cVector3d interactionForce = tool->getDeviceGlobalForce();

	// 力到位移映射参数
	static const double forceToDisplacementScale = 0.0001; // 1N -> 1mm位移
	static const double maxDisplacement = 0.05; // 最大位移10mm
	static const double forceDeadZone = 0.1; // 力的死区 0.1N

	// 应用死区处理
	double forceMagnitude = interactionForce.length();
	if (forceMagnitude < forceDeadZone) {
		interactionForce.zero();
	}

	// 力到位移的线性映射
	cVector3d tipDisplacement = interactionForce * forceToDisplacementScale;

	// 限制最大位移
	double displacementMagnitude = tipDisplacement.length();
	if (displacementMagnitude > maxDisplacement) {
		tipDisplacement.normalize();
		tipDisplacement *= maxDisplacement;
	}

	// 获取基准位置（软体机器人的自然零位）
	static cVector3d baseTipPosition(0.0, 0.0, ResolvedRateControl.h_0 * 0.001);

	// 计算目标tip位置
	cVector3d targetTipPosition = baseTipPosition + tipDisplacement;


	// 使用运动学模型计算压力
	cVector3d targetPosMM = targetTipPosition * 1000; // 转换为mm

	// 使用运动学模型计算压力（迭代方式）
	const double errorThreshold = 0.1;  // 误差阈值 (mm)
	const int maxIterations = 100;      // 最大迭代次数
	int iterations = 0;

	// 迭代计算直到误差足够小
	while (iterations < maxIterations) {
		std::cout << "[DEBUG] Iteration " << iterations
			<< " - presCurr: " << presCurr.str()
			<< ", targetPosMM: " << targetPosMM.str() << std::endl;

		auto result = ResolvedRateControl.updateMotion(presCurr, targetPosMM);
		cVector3d modeledPosition = result[0];
		cVector3d newPressure = result[1];

		// 计算误差
		double error = (modeledPosition - targetPosMM).length();

		std::cout << "[DEBUG] Iteration " << iterations
			<< " - error: " << error << "mm"
			<< ", newPressure: " << newPressure.str() << std::endl;

		// 检查是否收敛
		if (error < errorThreshold) {
			std::cout << "[DEBUG] Converged after " << iterations << " iterations" << std::endl;
			break;
		}

		// 验证新压力值
		if (!std::isfinite(newPressure.x()) ||
			!std::isfinite(newPressure.y()) ||
			!std::isfinite(newPressure.z())) {
			std::cout << "[EXP] ERROR: Invalid pressure calculated, stopping iteration" << std::endl;
			break;
		}

		// 更新压力值，继续迭代
		presCurr = newPressure;
		iterations++;
	}

	// 迭代完成后发送压力到Arduino
	std::string sendStr = presCurr.str();
	arduinoWriteData(50, sendStr);

	// 单行实时调试输出
	std::cout << std::fixed << std::setprecision(2)
		<< "[EXP] Force:" << interactionForce.str(2) << "N "
		<< "Disp:" << (tipDisplacement * 1000).str(1) << "mm "
		<< "Target:" << targetTipPosition.str(3) << "m "
		<< "Press:" << presCurr.str(1) << "kPa "
		<< "Iter:" << iterations << " "
		<< "Touch:L=" << (currentlyTouchingLeft ? "Y" : "N")
		<< ",R=" << (currentlyTouchingRight ? "Y" : "N") << "    \r";
}
else if (!experimentMode) {
	// 非实验模式时隐藏立方体
	if (leftCube) leftCube->setEnabled(false);
	if (rightCube) rightCube->setEnabled(false);

	/////////////////////////////////////////////////////////////////////////
	// 表征模式：保持原有的轨迹跟踪控制
	/////////////////////////////////////////////////////////////////////////
	if (!hasInitPress) {
		presCurr = ResolvedRateControl.m_initPressure;
		hasInitPress = true;
	}

	cVector3d targetPos = TrajTarget * 1000; // 目标轨迹位置
	auto result = ResolvedRateControl.updateMotion(presCurr, targetPos);
	chai3d::cVector3d newPos = result[0];
	chai3d::cVector3d newPressure = result[1];
	double error = (newPos - targetPos).length();

	if (error < 0.1) {
		cVector3d PressuretoArduino = presCurr;
		std::string sendStr = PressuretoArduino.str();
		arduinoWriteData(50, sendStr);
	}
	else {
		presCurr = newPressure;
	}

}
else {
	// 实验模式但无活跃试次时隐藏立方体
	if (leftCube) leftCube->setEnabled(false);
	if (rightCube) rightCube->setEnabled(false);
}

		/////////////////////////////////////////////////////////////////////////
		// HAPTIC MANIPULATION
		/////////////////////////////////////////////////////////////////////////

		// compute transformation from world to tool (haptic device)
		cTransform world_T_tool = tool->getDeviceGlobalTransform();

		// get status of user switch
		bool button = tool->getUserSwitch(0);

		//----------------------------------------------------------------------
		// STATE 1:
		// Idle mode - user presses the user switch
		//----------------------------------------------------------------------
		if ((state == IDLE) && (button == true))
		{
			// check if at least one contact has occurred
			if (tool->m_hapticPoint->getNumCollisionEvents() > 0)
			{
				// get contact event
				cCollisionEvent* collisionEvent = tool->m_hapticPoint->getCollisionEvent(0);

				// get object from contact event
				object = collisionEvent->m_object;

				// get transformation from object
				cTransform world_T_object = object->getGlobalTransform();

				// compute inverse transformation from contact point to object 
				cTransform tool_T_world = world_T_tool;
				tool_T_world.invert();

				// store current transformation tool
				tool_T_object = tool_T_world * world_T_object;

				// update state
				state = SELECTION;
			}
		}

		//----------------------------------------------------------------------
		// STATE 2:
		// Selection mode - operator maintains user switch enabled and moves object
		//----------------------------------------------------------------------
		else if ((state == SELECTION) && (button == true))
		{
			// compute new tranformation of object in global coordinates
			cTransform world_T_object = world_T_tool * tool_T_object;

			// compute new tranformation of object in local coordinates
			cTransform parent_T_world = object->getParent()->getLocalTransform();
			parent_T_world.invert();
			cTransform parent_T_object = parent_T_world * world_T_object;

			// assign new local transformation to object
			object->setLocalTransform(parent_T_object);

			// set zero forces when manipulating objects
			tool->setDeviceGlobalForce(0.0, 0.0, 0.0);
		}
		//----------------------------------------------------------------------
		// STATE 3:
		// Finalize Selection mode - operator releases user switch.
        //----------------------------------------------------------------------
		else
		{
			state = IDLE;
		}
		/////////////////////////////////////////////////////////////////////////
		// FINALIZE
		/////////////////////////////////////////////////////////////////////////
	FINISH_HAPTIC_CYCLE:
		;   // 空语句占位
		// send forces to haptic device
		// tool->applyToDevice();  
	}

	// exit haptics thread
	simulationFinished = true;
}


//------------------------------------------------------------------------------
void arduinoWriteData(unsigned int delay_time, const std::string& send_str)
{
	// A static counter that persists across calls
	static size_t send_count = 0;

	// Always increment the call count
	send_count++;

	// Only send data every `delay_time` calls
	if (send_count % delay_time == 0)
	{
		// --------------------------------------------------------------------
		// 1) Parse the input string (e.g. "20,20,20") into floats
		// --------------------------------------------------------------------
		// We'll split by comma, handle up to e.g. 3 channels (extend if needed)
		std::vector<std::string> tokens;
		{
			// Simple split by comma
			std::stringstream ss(send_str);
			std::string item;
			while (std::getline(ss, item, ','))
			{
				tokens.push_back(item);
			}
		}

		// Now clamp or fix each channel
		std::vector<double> values;
		for (auto& t : tokens)
		{
			// Trim potential whitespace
			// (optional) or just do t.erase(...)

			// Convert to double
			double val = 0.0;
			try
			{
				val = std::stod(t); // parse string => double
			}
			catch (...)
			{
				// If parse fails => treat as NaN => forcibly 0
				val = 0.0;
			}

			// Check if val is NaN => forcibly 0
			if (std::isnan(val))
			{
				val = 0.0;
			}
			// Then clamp to [-35, 35]
			if (val < -35.0) val = -35.0;
			if (val > 40.0) val = 35.0;

			values.push_back(val);
		}

		// --------------------------------------------------------------------
		// 2) Reassemble into a corrected string
		// --------------------------------------------------------------------
		// e.g. "10.0, -35.0, 35.0"
		std::ostringstream oss;
		for (size_t i = 0; i < values.size(); ++i)
		{
			oss << values[i];
			if (i + 1 < values.size()) oss << ",";
		}
		std::string correctedStr = oss.str();

		// Append a newline for Arduino to read line-by-line
		correctedStr += "\n";

		// --------------------------------------------------------------------
		// 3) Write to serial
		// --------------------------------------------------------------------
		myDevice->writeSerialPort(correctedStr.c_str(), correctedStr.length());
		//std::cout << "Send PRESSURE: " << correctedStr << "\r";

		// Reset the counter
		send_count = 0;
	}
}


void openCSV(const std::string& filename)
{
	csvFile.open(filename, std::ios::app);
	if (!csvFile.is_open()) {
		std::cerr << "Unable to open file\n";
		return;
	}
	// 只在文件为空时写表头
	if (csvFile.tellp() == 0) {
		csvFile
			<< "TimeStamp,"                             // ★修改：统一叫 TimeStamp
			<< "fr_posX,fr_posY,fr_posZ,fr_oriW,fr_oriX,fr_oriY,fr_oriZ,"
			<< "act_posX,act_posY,act_posZ,act_oriW,act_oriX,act_oriY,act_oriZ,"
			<< "rel_posX,rel_posY,rel_posZ,rel_oriW,rel_oriX,rel_oriY,rel_oriZ,"
			<< "cmd_X,cmd_Y,cmd_Z"                      // ★新增
			<< '\n';
		csvFile.flush();
	}
}


// Function to write data to the open CSV file
void writeToCSV(const std::vector<string>& row) {
	if (csvFile.is_open()) {
		for (size_t i = 0; i < row.size(); ++i) {
			csvFile << row[i];  // Write each double value
			if (i < row.size() - 1) {
				csvFile << ",";  // Add comma if it's not the last element
			}
		}
		csvFile << "\n";  // Newline after each row
		csvFile.flush();  // Flush to ensure data is written immediately
	}
	else {
		std::cerr << "File not open for writing" << std::endl;
	}
}

// Function to close the CSV file when done
void closeCSV() {
	if (csvFile.is_open()) {
		csvFile.close();
	}
}



// 将列主序( OpenGL-style )矩阵转成本函数需要的 row-major
static inline void colToRow(const double C[9], double R[9])
{
	// C[i*3 + j]  =  行 j, 列 i
	R[0] = C[0];  R[1] = C[3];  R[2] = C[6];
	R[3] = C[1];  R[4] = C[4];  R[5] = C[7];
	R[6] = C[2];  R[7] = C[5];  R[8] = C[8];
}

// --- 行向量乘 R^T -------------------------------------------------------------
static inline void mulRT(const double R[9], const double v[3], double o[3])
{
	o[0] = R[0] * v[0] + R[3] * v[1] + R[6] * v[2];
	o[1] = R[1] * v[0] + R[4] * v[1] + R[7] * v[2];
	o[2] = R[2] * v[0] + R[5] * v[1] + R[8] * v[2];
}

PNODATA makePNO_mm_colRM(const double pos_mm[3],
	const double Rcol[9])
{
	PNODATA out{};   // 全字段零初始化

	///* -------- 1) 位置：mm → cm（SDK 默认 POS_CM） -------- */
	//constexpr double kMM2CM = 1;
	out.pos[0] = pos_mm[0];
	out.pos[1] = pos_mm[1];
	out.pos[2] = pos_mm[2];

	//std::cout << std::fixed << std::setprecision(2)
	//	<< "[out.pos] (" << out.pos[0] << ", "
	//	<< out.pos[1] << ", "
	//	<< out.pos[2] << ") mm   \r ";


	/* -------- 2) 旋转矩阵 → 四元数 (w,x,y,z) --------
	   将列主序转为行主序元素便于公式书写                         */
	const double m00 = Rcol[0], m01 = Rcol[3], m02 = Rcol[6];
	const double m10 = Rcol[1], m11 = Rcol[4], m12 = Rcol[7];
	const double m20 = Rcol[2], m21 = Rcol[5], m22 = Rcol[8];

	double t = m00 + m11 + m22;          // matrix trace
	double qw, qx, qy, qz;

	if (t > 0.0) {
		double s = std::sqrt(t + 1.0) * 2.0;   // s = 4*qw
		qw = 0.25 * s;
		qx = (m21 - m12) / s;
		qy = (m02 - m20) / s;
		qz = (m10 - m01) / s;
	}
	else if (m00 > m11 && m00 > m22) {
		double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;  // s = 4*qx
		qw = (m21 - m12) / s;
		qx = 0.25 * s;
		qy = (m01 + m10) / s;
		qz = (m02 + m20) / s;
	}
	else if (m11 > m22) {
		double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;  // s = 4*qy
		qw = (m02 - m20) / s;
		qx = (m01 + m10) / s;
		qy = 0.25 * s;
		qz = (m12 + m21) / s;
	}
	else {
		double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;  // s = 4*qz
		qw = (m10 - m01) / s;
		qx = (m02 + m20) / s;
		qy = (m12 + m21) / s;
		qz = 0.25 * s;
	}

	/* 归一化（数值安全） */
	double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
	qw /= norm;  qx /= norm;  qy /= norm;  qz /= norm;

	out.ori[0] = static_cast<float>(qw);
	out.ori[1] = static_cast<float>(qx);
	out.ori[2] = static_cast<float>(qy);
	out.ori[3] = static_cast<float>(qz);


	return out;
}

//------------------------------------------------------------------------------
// 场景管理函数
//------------------------------------------------------------------------------

void showOriginalObjects(bool show)
{
	// 隐藏/显示原始对象
	if (base) base->setEnabled(show);
	if (box) box->setEnabled(show);
	if (sphere) sphere->setEnabled(show);
	if (cone) cone->setEnabled(show);

	// 隐藏/显示表征实验专用对象
	if (axisX) axisX->setEnabled(show);
	if (axisY) axisY->setEnabled(show);
	if (axisZ) axisZ->setEnabled(show);
	if (sphereTarget) sphereTarget->setEnabled(show);
	if (cylinderBot) cylinderBot->setEnabled(show);
	if (cylinderTopRel) cylinderTopRel->setEnabled(show);
	// cylinderTop 没有添加到world，所以不需要处理
}

void showExperimentObjects(bool show)
{
	// 显示/隐藏实验立方体
	if (leftCube) leftCube->setEnabled(show);
	if (rightCube) rightCube->setEnabled(show);

	// 显示/隐藏实验标签
	if (labelExperimentInstructions) labelExperimentInstructions->setEnabled(show);
	if (labelTrialInfo) labelTrialInfo->setEnabled(show);
}

void updateExperimentLabels()
{
	if (!userStudy || !experimentMode) return;

	// 更新指令标签
	if (labelExperimentInstructions) {
		std::string instructions = "STIFFNESS DISCRIMINATION EXPERIMENT\n";
		instructions += "Data automatically saved to: user_" + currentUserId + "_experiment_data.csv\n\n";

		// 根据实验状态显示不同信息
		switch (userStudy->getExperimentState()) {
		case ExperimentState::NOT_STARTED:
			instructions += "Loading experiment...\n";
			break;

		case ExperimentState::READY:
			instructions += "Experiment ready!\n";
			instructions += "Press 'T' to start the first trial\n";
			break;

		case ExperimentState::IN_PROGRESS:
			instructions += "Trial in progress\n";
			instructions += "Touch both surfaces and decide which is stiffer\n";
			instructions += "Press '1' for LEFT surface stiffer, '2' for RIGHT surface stiffer\n";
			break;

		case ExperimentState::TRIAL_COMPLETE:
			instructions += "Choice recorded!\n";
			instructions += "Press 'T' to start next trial\n";
			break;

		case ExperimentState::GROUP_COMPLETE:
			instructions += "Group complete! Take a 5-minute break.\n";
			instructions += "Press 'T' when ready to continue\n";
			break;

		case ExperimentState::EXPERIMENT_COMPLETE:
			instructions += "EXPERIMENT COMPLETE!\n";
			instructions += "Thank you for your participation!\n";
			instructions += "Press 'E' to exit experiment mode\n";
			break;
		}

		instructions += "\nManual Mode: Press 5-9,0 to jump to specific groups";
		instructions += "\nPress 'E' to exit experiment mode";

		labelExperimentInstructions->setText(instructions);

		// 居中显示
		int labelWidth = labelExperimentInstructions->getWidth();
		int centerX = (width - labelWidth) / 2;
		labelExperimentInstructions->setLocalPos(centerX, height - 200);
	}

	// 更新试次信息标签
	if (labelTrialInfo && userStudy->hasNextTrial()) {
		int trialInGroup = userStudy->getCurrentTrialInGroup();
		int groupNum = userStudy->getCurrentTrialGroup() + 1;

		std::string trialInfo = "Group: " + std::to_string(groupNum) + "/6 | ";
		trialInfo += "Trial: " + std::to_string(trialInGroup) + "/110\n";

		std::string modeStr = (userStudy->getCurrentMode() == InteractionMode::FORCE_TO_POSITION) ? "F-P" : "F-F";
		std::string dirStr;
		switch (userStudy->getCurrentDirection()) {
		case FeedbackDirection::LATERAL_X: dirStr = "X (Left/Right)"; break;
		case FeedbackDirection::LATERAL_Y: dirStr = "Y (Forward/Backward)"; break;
		case FeedbackDirection::VERTICAL_Z: dirStr = "Z (Up/Down)"; break;
		}

		trialInfo += "Mode: " + modeStr + " | Direction: " + dirStr;

		labelTrialInfo->setText(trialInfo);
		labelTrialInfo->m_fontColor.setBlack();

		int labelWidth = labelTrialInfo->getWidth();
		int centerX = (width - labelWidth) / 2;
		labelTrialInfo->setLocalPos(centerX, 80);
	}
}
// 在文件末尾实现这个函数
void updateSurfaceOrientation(FeedbackDirection direction)
{
	if (!leftCube || !rightCube) return;

	// 清除现有的几何体
	leftCube->clear();
	rightCube->clear();

	// 根据方向创建不同朝向的薄片
	switch (direction) {
	case FeedbackDirection::LATERAL_X:
		// X方向 - 垂直面，法线沿X轴
		cCreateBox(leftCube, 0.001, 0.03, 0.03);
		cCreateBox(rightCube, 0.001, 0.03, 0.03);
		break;

	case FeedbackDirection::LATERAL_Y:
		// Y方向 - 垂直面，法线沿Y轴
		cCreateBox(leftCube, 0.03, 0.001, 0.03);
		cCreateBox(rightCube, 0.03, 0.001, 0.03);
		break;

	case FeedbackDirection::VERTICAL_Z:
		// Z方向 - 水平面，法线沿Z轴
		cCreateBox(leftCube, 0.03, 0.03, 0.001);
		cCreateBox(rightCube, 0.03, 0.03, 0.001);
		break;
	}

	// 重新创建碰撞检测
	leftCube->createAABBCollisionDetector(0.002);
	rightCube->createAABBCollisionDetector(0.002);
}

//------------------------------------------------------------------------------
// 坐标变换函数实现 - 使用 cTransform 进行整体运算
//------------------------------------------------------------------------------

cTransform transformCameraToBase(const double camPos[3], const double camRot[9]) {

	cTransform g_baseRef_inv(g_camera_T_base);
	g_baseRef_inv.invert();

	// 构建相机坐标系下的变换
	cVector3d pos(
		camPos[0] * 0.001, 
		camPos[1] * 0.001, 
		camPos[2] * 0.001);
	cMatrix3d rot(
		camRot[0], camRot[1], camRot[2],
		camRot[3], camRot[4], camRot[5],
		camRot[6], camRot[7], camRot[8]);

	cTransform T_cam(pos, rot);

	// 计算: base_T_marker = T_base_inv * T_cam
	cTransform base_T_marker;
	base_T_marker = g_baseRef_inv * T_cam;

	return base_T_marker;
}

bool getMTMarkerTransform(const std::string& markerName, cTransform& outTransform) {
	if (!g_mtAvailable || !MT.update()) {
		return false;
	}

	mtw::Pose pose;
	if (!MT.getPoseAndRotMat(markerName, pose)) {
		return false;
	}

	// 获取完整的变换（从相机坐标系到VE坐标系）
	outTransform = transformCameraToBase(pose.pos, pose.rot);
	return true;
}

void printTransform(const cTransform& T, const std::string& name) {
	cVector3d p = T.getLocalPos();
	cMatrix3d R = T.getLocalRot();

	std::cout << name << " translation: "
		<< p.x() << ", " << p.y() << ", " << p.z() << std::endl;

	std::cout << name << " rotation matrix:\n";
	for (int r = 0; r < 3; ++r) {
		std::cout << R(0, r) << "  "
			<< R(1, r) << "  "
			<< R(2, r) << std::endl;
	}
}
