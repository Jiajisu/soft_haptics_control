
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
// Pohelmus
//------------------------------------------------------------------------------
#include <cstdio>
#include <cstdlib>
#include "VPcmdIF.h"
#include "VPtrace.h"
#include <string>
#include <iostream>
#include <iomanip>  // for std::setw
#define POLHEMUS_VID 0x0f44
#define VIPER_PID	0xBF01

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

#include "frameTrans.hpp"
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
static bool g_doBoresight = false;
static bool g_trajectoryStarted = false;
static double g_trajectoryStartTime = 0.0;
// This accumulator persists across multiple calls to updateHaptics
//static cVector3d pressureAccum(0.0, 0.0, 0.0);
bool g_enableControl = false;
std::atomic<bool> g_doMTZero{ false };   // 之前是 bool
cVector3d ZeroPressure(0, 0, 0);
static mtw::MicronTracker MT;
bool mtZeroMarkers(mtw::MicronTracker& MT, double h0_mm);
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

//--------------------------------------------------------------
//  保存调零时抓到的基准
//--------------------------------------------------------------
struct MTZeroFrame
{
	// --- fr 在相机坐标系下的零点姿态 ---
	double frPos[3]{};
	double frR[9]{};          // col-major

	// --- Actuator 在相机坐标系下的零点姿态（可选，调试方便） ---
	double actPos[3]{};
	double actR[9]{};         // col-major

	// --- Actuator 在 fr 坐标系下的“应扣除”量 ---
	double relPosOff[3]{};    // (Δp₀ - (0,0,h₀))  ，单位 mm
	double relROff[9]{};      // 相对旋转零点 (col-major)

	bool   valid{ false };

} g_mtZero;


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


// 在现有全局变量声明后添加
//std::ofstream experimentCsvFile;  // 新增：实验模式专用CSV文件
//
//bool recordExperimentDataToCSV = false;  // 新增：实验数据记录开关
std::string currentCsvType = "characterization"; // "characterization" 或 "experiment"

//// 在现有函数声明后添加
//void openExperimentCSV(const std::string& filename);
//void writeExperimentDataToCSV(int userChoice, double refStiffness, double compStiffness, double reactionTime);
//void closeExperimentCSV();

//================================Polhemus======================================
#include <cstdio>
#include <cstdlib>
#include "VPcmdIF.h"
#include "VPtrace.h"
#include <string>
#include <iostream>
#include <iomanip>  // for std::setw
#define POLHEMUS_VID 0x0f44
#define VIPER_PID	0xBF01
enum {
	VIPER_TRACKER
	//, LIBERTY_TRACKER
	//, FASTRAK3_TRACKER

	, TRACKER_MAX

}eTracker;
vpcmd_context g_ctx;
vpdev_hnd     g_hnd;
uint16_t g_vid = POLHEMUS_VID;
uint16_t g_pid = VIPER_PID;
int32_t g_TrackerType = VIPER_TRACKER;
int		   g_TraceLevel = VPCMD_TRACE_3_NORMALIO;
int		   g_bTraceMode = true;
UNITS_CONFIG config;
typedef int (*usbcnxfunc)(vpcmd_context, vpdev_hnd&, uint16_t);
struct _cnx_fxn_info
{
	usbcnxfunc func;
	const char* szfunc;
};
_cnx_fxn_info usbcnx_fxn_info[TRACKER_MAX] =
{
	vpctx_connectusb, "vpctx_connectusb",
};

typedef struct _MYPNO {
	SEUPNO seupno;
	SENFRAMEDATA sarr[SENSORS_PER_SEU];
}MYPNOTYPE;

vpFrameInfo f;	///frame structs
CFrameRateCfg g_cfrate;
// 用于记录“在按下 boresightSensor1() 时，传感器1的当前位置”
// 后续循环把该值作为 offset，令后续 pos - g_posOffset => 位置归零
cVector3d g_posOffset(0.0, 0.0, 0.0);

/****************************************************************************
Function Identifier
 ****************************************************************************/
bool Connect();
void Disconnect();
void DisplaySEUs();
void DisplayError(int32_t err, const char* szMsg);
bool StartCont();
bool StopCont();
bool boresightSensor1();
bool boresightSensor2();
bool clearBoresightSensor1();
PNODATA GrabFramePNO(MYPNOTYPE* pv, uint32_t s_index);
double getCurrentTime();

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
	world->addChild(base);

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
		cVector3d(-10, 0, 0),
		cVector3d(10, 0, 0)
	);
	axisX->m_material->setRedCrimson();

	axisY = new cShapeLine(
		cVector3d(0, -10, 0),
		cVector3d(0, 10, 0)
	);
	axisY->m_material->setGreenForest();

	axisZ = new cShapeLine(
		cVector3d(0, 0, -10),
		cVector3d(0, 0, 10)
	);
	axisZ->m_material->setBlueRoyal();

	world->addChild(axisX);
	world->addChild(axisY);
	world->addChild(axisZ);

	//leftCube->setLocalPos( 0.02, -0.03, 0.04);
	// 在现有对象创建代码后添加
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
	//
	Connect();
	// 先设置帧率到30Hz
	CFrameRateCfg cfrate;
	int rr = vpcmd_dev_framerate(g_ctx, g_hnd, CMD_ACTION_GET, cfrate);
	if (rr != 0) {
		DisplayError(rr, "vpcmd_dev_framerate GET");
	}
	else {
		cfrate.frame_rate = FR_60;  // FR_30即枚举0,表示 30Hz
		rr = vpcmd_dev_framerate(g_ctx, g_hnd, CMD_ACTION_SET, cfrate);
		if (rr != 0) {
			DisplayError(rr, "vpcmd_dev_framerate SET to 60Hz");
		}
		else {
			std::cout << "Frame rate set to 60Hz.\n";
		}
	}
	config.pos_units = POS_CM;
	config.ori_units = ORI_QUATERNION;
	vpcmd_dev_units(g_ctx, g_hnd, CMD_ACTION_SET, &config);
	StartCont();
	// create a thread which starts the main haptics rendering loop
	hapticsThread = new cThread();
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);


	// 1) 让 Polhemus 内部输出连续 PNO 数据
	//int r = vpcmd_dev_contpno(g_ctx, g_hnd, CMD_ACTION_SET);
	//if (r != 0) {
	//	DisplayError(r, "vpcmd_dev_contpno SET");
	//}

	// 3) 如果想要 Polhemus 给每帧打上时间戳，则启用 time-stamp
	//    viper 提供 vpdev_tsenable(...) 
	bool wantViperTimestamp = true;
	vpdev_tsenable(g_ctx, g_hnd, wantViperTimestamp);


	int sensorAll = VP_ID_ALL;  // = -1 => 对所有传感器生效

	// a) 位置(Position)滤波 => Light
	{
		CFilterCfg filterLightPos;
		filterLightPos.Fill(FILTER_LVL_LIGHT);
		vpcmd_sns_filter(g_ctx, g_hnd, sensorAll, CMD_ACTION_SET, &filterLightPos, FILTER_TRGT_POS);

	}

	// b) 姿态(Orientation)滤波 => Light
	{
		CFilterCfg filterLightOri;
		filterLightOri.Fill(FILTER_LVL_LIGHT);
		vpcmd_sns_filter(g_ctx, g_hnd, sensorAll, CMD_ACTION_SET, &filterLightOri, FILTER_TRGT_ORI);

	}

	//-------------------------------------------
	// 2) 打开预测滤波，并设参数为默认值(20ms 等)
	//-------------------------------------------
	{
		PF_CONFIG predCfg;
		memset(&predCfg, 0, sizeof(predCfg));
		predCfg.qFil_on = 1;     // 开启姿态预测
		predCfg.rFil_on = 1;     // 开启位置预测
		predCfg.predTimeS = 0.02f; // 20毫秒
		vpcmd_sns_predfilter(g_ctx, g_hnd, sensorAll, CMD_ACTION_SET, &predCfg);

	}
	//-------------------------------------------
	// 2) Turn on micron tracker
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
	StopCont();
	Disconnect();
	Disconnect();
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
	//Press R for turn off boresight (just read raw data) 
	else if (a_key == GLFW_KEY_R)
	{
		clearBoresightSensor1();

	}
	//Press B for boresight the sensor 1 to (0,0,0) (0,0,0,1)
	else if (a_key == GLFW_KEY_B)
	{
		// call boresight sensor1
		g_doBoresight = true;
		g_doMTZero = true;
		std::cout << "[INFO] boresight requested.\n";


	}
	//else if (a_key == GLFW_KEY_I)
	//{
	//	sendPosToArduino = !sendPosToArduino;
	//	cout << "Sending Position Info to Arduino: " << sendPosToArduino << endl;
	//}
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
		// 在现有按键处理后添加
	else if (a_key == GLFW_KEY_E)
	{
		// 切换实验模式
		experimentMode = !experimentMode;

		if (experimentMode) {
			// 进入实验模式
			if (!userStudy) {
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


}

//------------------------------------------------------------------------------

void close(void)
{
	// stop the simulation
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
	closeCSV();
	//closeExperimentCSV();
}

//------------------------------------------------------------------------------
void MTLoop()
{
	while (g_mtRunning)
	{	
		if (!g_mtAvailable) return;
		if (!MT.update()) continue;           // 阻塞在 SDK FPS

		MTPoseSnapshot snap;                  // 本地暂存
		snap.haveFr = MT.getPoseAndRotMat("fr", snap.fr);
		snap.haveAct = MT.getPoseAndRotMat("Actuator", snap.act);
		snap.haveRel = (snap.haveFr && snap.haveAct &&
			MT.getRelPos("Actuator", "fr", snap.relPos) &&
			MT.getRelRotMat("Actuator", "fr", snap.relR));
		snap.ts = MT.getLastTimestamp();

		// 复制到共享区
		{
			std::lock_guard<std::mutex> lk(g_mtMtx);
			g_mtSnap = snap;
		}
		if (g_doMTZero.exchange(false))      // 把标志原子置回 false
		{
			const double h0_mm = ResolvedRateControl.h_0;      // 你的机械零点高度
			if (mtZeroMarkers(MT, h0_mm))
				std::cout << "[MT] zero done (from MTLoop)\n";
			else
				std::cerr << "[MT] zero failed\n";

			// 写入 g_mtZero 已在 mtZeroMarkers() 内完成
			// 如担心多线程读写，可再加 mutex：
			// { std::lock_guard<std::mutex> lk(g_mtMtx); ... }
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
	//add top and bot cap
	double radius = 0.005; // 12mm
	double height = 0.001; // 3mm
	// 改为：
	if (!cylinderTop) {
		cylinderTop = new cShapeCylinder(radius, radius, height);
		cylinderTop->m_material->setRed();
	}
	if (!cylinderBot) {
		cylinderBot = new cShapeCylinder(radius, radius, height);
		world->addChild(cylinderBot);
	}

	//world->addChild(cylinderTop);
	world->addChild(cylinderBot);

	cylinderTop->m_material->setRed();

	//double relRadius = 0.012;
	//double relHeight = 0.0035;
	double relRadius = 0.005;
	double relHeight = 0.001;
	// 同样修改 cylinderTopRel
	if (!cylinderTopRel) {
		cylinderTopRel = new cShapeCylinder(relRadius, relRadius, relHeight);
		world->addChild(cylinderTopRel);
		cylinderTopRel->m_material->setGreenForest();
		cylinderTopRel->setLocalPos(0, 0, 0.0);
	}
	world->addChild(cylinderTopRel);


	cylinderTopRel->m_material->setGreenForest();
	cylinderTopRel->setLocalPos(0, 0, 0.0); // 先往 X 方向左移

	double sphereRadius = 0.0025; // 小球半径 (可根据需要调整)
	// 修改 sphereTarget
	if (!sphereTarget) {
		sphereTarget = new cShapeSphere(sphereRadius);
		sphereTarget->m_material->setBlueCornflower();
		sphereTarget->m_material->setTransparencyLevel(0.6);
		sphereTarget->setUseTransparency(true);
		world->addChild(sphereTarget);
	}

	// 设置材质或颜色
	sphereTarget->m_material->setBlueCornflower(); // 示例颜色
	sphereTarget->m_material->setTransparencyLevel(0.6);  // 半透明 (0.0 ~ 1.0)
	sphereTarget->setUseTransparency(true); // 启用透明效果

	// 将小球添加到场景
	world->addChild(sphereTarget);

	// generate targets on circles
	//  1) r=3.5, center=(0,0,19.5)
	//  2) r=5.0, center=(0,0,21.5)
	//  3) r=3.0, center=(0,0,24.5)
	std::vector<TrajectoryGenerator::CircleDefinition> circles = {
		{0.0035, chai3d::cVector3d(0.0, 0.0, ResolvedRateControl.h_0 * 0.001+ 0.003)},
		{0.0045, chai3d::cVector3d(0.0, 0.0, ResolvedRateControl.h_0 * 0.001 - 0.006)},
		{0.0055, chai3d::cVector3d(0.0, 0.0, ResolvedRateControl.h_0 *0.001 -0.0012)}
	};

	// number of traget points on the circle
	int points = 12;


	// 调用函数，得到一个二维容器
	// outer.size()=3, each inner.size()=12
	std::vector<std::vector<chai3d::cVector3d>> allCircles =
		TrajectoryGenerator::generateMultipleCirclesPoints(circles, points);
	cVector3d startPos(0.0, 0.0, ResolvedRateControl.h_0 * 0.001);
	
	// 我们定义一个“sensor2PosOffset”，用于让 sensor2 在无驱动时变成 (0,0,h_0)
	static cVector3d sensor2PosOffset(0.0, 0.0, 0.0);

	chai3d::cVector3d PointOnCir = allCircles[circleIndex][pointIndex];


	static const chai3d::cVector3d center(0.0, 0.0, ResolvedRateControl.h_0 * 0.001);   // 起始点
	//static const chai3d::cVector3d amp    (0.00, 0.00001, 0.003); // 幅值: ±3 mm, ±2 mm, ±1 mm
	static const chai3d::cVector3d amp    (0.002, 0.000, 0.00); // 幅值: ±3 mm, ±2 mm, ±1 mm
	double freq = 1;                      // 5 Hz 带宽测试

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

		PNODATA pno_1;
		PNODATA pno_2;
		PNODATA pno2_in_sensor1_frame;
		// 1) 读取 Polhemus FIFO 的下一帧
		vpFrameInfo f;
		//  vpcmd_context g_ctx;   // 你的全局/外部变量
		//  vpdev_hnd     g_hnd;   // 同上
		//vpctx_dev_fifopnof(g_ctx, g_hnd, f);
		//READ LAST PNO DATA
		 vpctx_dev_lastpnof(g_ctx, g_hnd, f);




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
		//------------------------------------------------------------
		// 0) 采集新帧
		//------------------------------------------------------------
		//if (!MT.update()) {
		//	std::cerr << "[MT] update failed, skip\n";
		//	continue;
		//}

		////------------------------------------------------------------
		//// 1) 读取 fr / Actuator 位姿，先检查返回值
		////------------------------------------------------------------
		mtw::Pose fr, act;
		//bool okFr = MT.getPoseAndRotMat("fr", fr);
		//bool okAct = MT.getPoseAndRotMat("Actuator", act);

		//if (!okFr) {                                    // 没有 fr → 本帧无效
		//	std::cerr << " fr lost, skip frame\n";
		//	continue;
		//}

		////------------------------------------------------------------
		//// 2) 首次调零（只做一次）
		////------------------------------------------------------------
		//if (g_doMTZero) {
		//	const double h0_mm = 29.45;
		//	if (mtZeroMarkers(MT, h0_mm))
		//		std::cout << "[MT] zero done\n";
		//	g_doMTZero = false;
		//}

		////------------------------------------------------------------
		//// 3) 相对位姿（只有 okAct 时才计算）
		////------------------------------------------------------------
		//double relPos[3] = { 0 }, relRot[9];        // 初始化
		//bool   okRel = false;

		//if (okAct &&
		//	MT.getRelPos("Actuator", "fr", relPos) &&
		//	MT.getRelRotMat("Actuator", "fr", relRot))
		//{
		//	okRel = true;
		//}
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

		//------------------------------------------------------------
		// 4) 调零：位置（严格刚体变换） / 旋转
		//------------------------------------------------------------
		double frZeroPos[3];
		double frRotZero[9];
		double relZeroPos[3];
		double relRotCorr[9];

		// ----- fr的调零（严格刚体变换）-----
		if (g_mtZero.valid) {
			for (int r = 0; r < 3; ++r) {
				frZeroPos[r] =
					g_mtZero.frR[0 * 3 + r] * (fr.pos[0] - g_mtZero.frPos[0]) +
					g_mtZero.frR[1 * 3 + r] * (fr.pos[1] - g_mtZero.frPos[1]) +
					g_mtZero.frR[2 * 3 + r] * (fr.pos[2] - g_mtZero.frPos[2]);
			}

			for (int r = 0; r < 3; ++r)
				for (int c = 0; c < 3; ++c)
					frRotZero[c * 3 + r] =
					g_mtZero.frR[0 * 3 + r] * fr.rot[0 * 3 + c] +
					g_mtZero.frR[1 * 3 + r] * fr.rot[1 * 3 + c] +
					g_mtZero.frR[2 * 3 + r] * fr.rot[2 * 3 + c];
		}
		else {
			// 没有调零时保留原始值
			for (int i = 0; i < 3; ++i) frZeroPos[i] = fr.pos[i];
			for (int i = 0; i < 9; ++i) frRotZero[i] = fr.rot[i];
		}

		// ----- rel的调零（严格刚体变换）-----
		if (okRel && g_mtZero.valid) {
			// 平移差 d = Δp₀ - (0,0,h₀)
			double d[3] = { relPos[0] - g_mtZero.relPosOff[0],
							relPos[1] - g_mtZero.relPosOff[1],
							relPos[2] - g_mtZero.relPosOff[2] };

			// 行向量乘 Rᵀ ：mulRT 已按 “R 转置” 实现
			double Rrow[9];
			colToRow(g_mtZero.relROff, Rrow);   // <<--- 只需这一句
			mulRT(Rrow, d, relZeroPos);         // 用 Rrow 而不是原来的 relROff

			for (int r = 0; r < 3; ++r)
				for (int c = 0; c < 3; ++c)
					relRotCorr[c * 3 + r] =
					g_mtZero.relROff[0 * 3 + r] * relRot[0 * 3 + c] +
					g_mtZero.relROff[1 * 3 + r] * relRot[1 * 3 + c] +
					g_mtZero.relROff[2 * 3 + r] * relRot[2 * 3 + c];
		}
		else {
			for (int i = 0; i < 3; ++i) relZeroPos[i] = relPos[i];
			for (int i = 0; i < 9; ++i) relRotCorr[i] = relRot[i];
		}



		//------------------------------------------------------------
		// 6) 渲染  (mm→m ×0.001)
		//------------------------------------------------------------
		// -- fr --
		cVector3d posFrM(frZeroPos[0] * 0.001, frZeroPos[1] * 0.001, frZeroPos[2] * 0.001);
		cMatrix3d rotFrM;
		rotFrM.set(
			frRotZero[0], frRotZero[3], frRotZero[6],
			frRotZero[1], frRotZero[4], frRotZero[7],
			frRotZero[2], frRotZero[5], frRotZero[8]);

		//cylinderBot->setLocalPos(posFrM);

		cylinderBot->setLocalPos(0, 0.107,  0.07);

		cylinderBot->setLocalRot(rotFrM);
		tool->setDeviceLocalPos(posFrM);
		// -- rel (仅当 okRel && g_mtZero.valid) --
		if (okRel && g_mtZero.valid)
		{
			cVector3d posRelM(relZeroPos[0] * 0.001,
				relZeroPos[1] * 0.001,
				relZeroPos[2] * 0.001);
			posRelM += cVector3d(0, 0.1, 0.07);   // +3 cm 偏移

			cMatrix3d rotRelM;
			rotRelM.set(
				relRotCorr[0], relRotCorr[3], relRotCorr[6],
				relRotCorr[1], relRotCorr[4], relRotCorr[7],
				relRotCorr[2], relRotCorr[5], relRotCorr[8]);

			cylinderTopRel->setEnabled(true);
			cylinderTopRel->setLocalPos(posRelM);
			cylinderTopRel->setLocalRot(rotRelM);

			PNODATA sensor2CrctPNO{};
			if (okRel && g_mtZero.valid)
			{
				//double posRel[3] = { relPos[0], relPos[1], relPos[2] };   // mm
				//double RRel[9] = { relRot[0], relRot[1], relRot[2],
				//					 relRot[3], relRot[4], relRot[5],
				//					 relRot[6], relRot[7], relRot[8] };




		//--------------------------------------------------------------------
		// 7)  构造 PNODATA
		//--------------------------------------------------------------------

				 //---------- (1) pno_1  ← fr（原始，未调零） ----------
				{
					double posFr[3] = { fr.pos[0], fr.pos[1], fr.pos[2] };       // mm
					double RFr[9] = { fr.rot[0], fr.rot[1], fr.rot[2],
										fr.rot[3], fr.rot[4], fr.rot[5],
										fr.rot[6], fr.rot[7], fr.rot[8] };        // 列主序
					pno_1 = makePNO_mm_colRM(posFr, RFr);
				}
				// ---------- (2) pno_2  ← act（若 okAct） ----------
				if (okAct)
				{
					double posAct[3] = { act.pos[0], act.pos[1], act.pos[2] };   // mm
					double RAct[9] = { act.rot[0], act.rot[1], act.rot[2],
										 act.rot[3], act.rot[4], act.rot[5],
										 act.rot[6], act.rot[7], act.rot[8] };
					pno_2 = makePNO_mm_colRM(posAct, RAct);
				}

				// ---------- (3) sensor2CrctPNOArc  ← 调零后的 rel ----------
				double posRel[3] = { relZeroPos[0], relZeroPos[1], relZeroPos[2] };   // mm
				double RRel[9] = { relRotCorr[0], relRotCorr[1], relRotCorr[2],
									 relRotCorr[3], relRotCorr[4], relRotCorr[5],
									 relRotCorr[6], relRotCorr[7], relRotCorr[8] };


				sensor2CrctPNO = makePNO_mm_colRM(posRel, RRel);
			}


			//std::cout << std::fixed << std::setprecision(2)
			//	<< "[relZeroPos] (" << relZeroPos[0] << ", "
			//	<< relZeroPos[1] << ", "
			//	<< relZeroPos[2] << ") mm    "
			//	<< "MicronTracker FPS = "
			//	<< std::setprecision(1) << MT.getFPS() << " Hz\r";

			//------------------------------P to L model---------------------------------

			//cVector3d targetPos = TrajTarget * 1000;  // 目标位置 (mm)
			//if (g_enableControl)
			//{
			//	//std::cout << std::fixed << std::setprecision(2)
			//	//	<< "[sensor2CrctPNOArc] (" << sensor2CrctPNOArc.pos[0] << ", "
			//	//	<< sensor2CrctPNOArc.pos[1] << ", "
			//	//	<< sensor2CrctPNOArc.pos[2] << ") mm    "
			//	//	<< "[targetPos] ( "
			//	//	<< targetPos << " ) mm\r";
			//	auto result = ResolvedRateControl.updateMotionCloseLoop(targetPos, sensor2CrctPNO);
			//	if (!std::isfinite(frZeroPos[0])) {
			//		std::cerr << " Invalid data detected, skip frame\n";
			//		continue;
			//	}
			//	//std::cout << "   targetPos: " << targetPos << "   RELPos: " << sensor2CrctPNO.pos[0] << ", " << sensor2CrctPNO.pos[1] << ", " << sensor2CrctPNO.pos[2] << std::endl;
			//	cVector3d newPos = result[0];       // 计算的新位置
			//	cVector3d newPressure = result[1];  // 计算的新压力
			//	double error = (newPos - targetPos).length();
			//	//std::cout << "Error: " << error << " = " << newPos << " - " << targetPos << std::endl;
			//	//std::cout << "newpos " << newPos << std::endl;
			//	cVector3d PressuretoArduino = presCurr;
			//	//cVector3d PressuretoArduino(20,20,20);
			//	std::string sendStr = PressuretoArduino.str();
			//	arduinoWriteData(50, sendStr);
			//	//std::cout << "sendStr: "
			//	//	<< sendStr << "\r";
			//	presCurr = newPressure; // 
			//}




			if (recordSensorDataToCSV)
			{
				// ① 取 MicronTracker 时间戳（秒）
				double ts = MT.getLastTimestamp();   // 秒
				// ② 拼装 CSV 行
				std::vector<std::string> row;
				row.emplace_back(std::to_string(ts));                // 时间戳

				// -- fr --
				row.emplace_back(std::to_string(pno_1.pos[0]));
				row.emplace_back(std::to_string(pno_1.pos[1]));
				row.emplace_back(std::to_string(pno_1.pos[2]));
				row.emplace_back(std::to_string(pno_1.ori[0]));
				row.emplace_back(std::to_string(pno_1.ori[1]));
				row.emplace_back(std::to_string(pno_1.ori[2]));
				row.emplace_back(std::to_string(pno_1.ori[3]));

				// -- Actuator --
				row.emplace_back(std::to_string(pno_2.pos[0]));
				row.emplace_back(std::to_string(pno_2.pos[1]));
				row.emplace_back(std::to_string(pno_2.pos[2]));
				row.emplace_back(std::to_string(pno_2.ori[0]));
				row.emplace_back(std::to_string(pno_2.ori[1]));
				row.emplace_back(std::to_string(pno_2.ori[2]));
				row.emplace_back(std::to_string(pno_2.ori[3]));

				// -- Rel (Act → fr) --
				row.emplace_back(std::to_string(sensor2CrctPNO.pos[0]));
				row.emplace_back(std::to_string(sensor2CrctPNO.pos[1]));
				row.emplace_back(std::to_string(sensor2CrctPNO.pos[2]));
				row.emplace_back(std::to_string(sensor2CrctPNO.ori[0]));
				row.emplace_back(std::to_string(sensor2CrctPNO.ori[1]));
				row.emplace_back(std::to_string(sensor2CrctPNO.ori[2]));
				row.emplace_back(std::to_string(sensor2CrctPNO.ori[3]));

				row.emplace_back(std::to_string(TrajTarget.x()));   // ★
				row.emplace_back(std::to_string(TrajTarget.y()));   // ★
				row.emplace_back(std::to_string(TrajTarget.z()));   // ★
				// ③ 写入
				writeToCSV(row);
			}




		}
		else
		{
			cylinderTopRel->setEnabled(false);      // 隐藏，避免用垃圾值渲染
		}







		//------------------------------------------------------------
		// 5) 打印（可选）——确认调零后数值是否有限
		//------------------------------------------------------------
		//if (!std::isfinite(frZeroPos[0])) {
		//	std::cerr << " Invalid data detected, skip frame\n";
		//	continue;
		//}


		//std::cout << std::fixed << std::setprecision(2)
		//	<< "[sensor2CrctPNOArc] (" << sensor2CrctPNO.pos[0] << ", "
		//	<< sensor2CrctPNO.pos[1] << ", "
		//	<< sensor2CrctPNO.pos[2] << ") mm   \r ";






		// 
			// 2) 检查 f.uiSize 是否有数据
			/*
			if (f.uiSize > 0 && f.pF != nullptr)
			{
				// f.ts 就是时间戳(微秒级或根据 Polhemus 版本)
				// 只有 wantTimestamp = true 时才有意义
				uint64_t ViperTimestamp = f.ts;

				//// 3) 解析传感器数据
				//if ((MYPNOTYPE*)&(f.pF)[0])
				//{
					// a) Sensor1 (cylinderBot)
					pno_1 = GrabFramePNO((MYPNOTYPE*)&(f.pF)[0], 0);
					double C3Dscale = 0.01; // cm->m
					double Arcscale = 10; // cm->mm
					cVector3d C3DPos1(
						pno_1.pos[0] * C3Dscale,
						pno_1.pos[1] * C3Dscale,
						pno_1.pos[2] * C3Dscale
					);
					cVector3d relPos1 = C3DPos1 - g_posOffset;
					cQuaternion q1(pno_1.ori[0], pno_1.ori[1], pno_1.ori[2], pno_1.ori[3]);
					cMatrix3d r1 = quaternionToMatrix(q1);

					cylinderBot->setLocalPos(relPos1);
					cylinderBot->setLocalRot(r1);

					// b) Sensor2 (cylinderTop)
					pno_2 = GrabFramePNO((MYPNOTYPE*)&(f.pF)[0], 1);
					cVector3d C3DPos2(
						pno_2.pos[0] * C3Dscale,
						pno_2.pos[1] * C3Dscale,
						pno_2.pos[2] * C3Dscale
					);
					cVector3d relPos2 = C3DPos2 - g_posOffset;
					cQuaternion q2(pno_2.ori[0], pno_2.ori[1], pno_2.ori[2], pno_2.ori[3]);
					cMatrix3d r2 = quaternionToMatrix(q2);
					cVector3d sensor2FinalPos = relPos2 - sensor2PosOffset;
					cylinderTop->setLocalPos(sensor2FinalPos);
					cylinderTop->setLocalRot(r2);

					PNODATA sensor1finalPNO = pno_1;          // 复制
					sensor1finalPNO.pos[0] = relPos1.x() / C3Dscale; // 若保持 cm
					sensor1finalPNO.pos[1] = relPos1.y() / C3Dscale;
					sensor1finalPNO.pos[2] = relPos1.z() / C3Dscale;
					PNODATA sensor2finalPNO = pno_2;          // 复制
					sensor2finalPNO.pos[0] = sensor2FinalPos.x() / C3Dscale; // 若保持 cm
					sensor2finalPNO.pos[1] = sensor2FinalPos.y() / C3Dscale;
					sensor2finalPNO.pos[2] = sensor2FinalPos.z() / C3Dscale;



					//std::cout << "relPos1 " << sensor1finalPNO.pos[0] << "   sensor2FinalPos: " << sensor1finalPNO.pos[0] << std::endl;



					// c) 计算 sensor2 相对于 sensor1 的位置和姿态
					PNODATA pno2_in_sensor1_frame = convertToSensor1Frame(sensor1finalPNO, sensor2finalPNO);


					cVector3d sensor2RelPosC3D(
						pno2_in_sensor1_frame.pos[0] * C3Dscale,
						pno2_in_sensor1_frame.pos[1] * C3Dscale,
						pno2_in_sensor1_frame.pos[2] * C3Dscale
					);  
					cQuaternion q2_rel(
						pno2_in_sensor1_frame.ori[0],
						pno2_in_sensor1_frame.ori[1],
						pno2_in_sensor1_frame.ori[2],
						pno2_in_sensor1_frame.ori[3]
					);
					cMatrix3d r2_rel = quaternionToMatrix(q2_rel);

					cVector3d shift(0.0, 0.03, 0.0);
					cVector3d finalPosRel = shift + sensor2RelPosC3D;
					cylinderTopRel->setLocalPos(finalPosRel);
					cylinderTopRel->setLocalRot(r2_rel);

					PNODATA sensor2CrctPNOArc;
					sensor2CrctPNOArc.pos[0] = pno2_in_sensor1_frame.pos[0] * 10; //mm
					sensor2CrctPNOArc.pos[1] = pno2_in_sensor1_frame.pos[1] * 10;
					sensor2CrctPNOArc.pos[2] = pno2_in_sensor1_frame.pos[2] * 10;
					sensor2CrctPNOArc.ori[0] = pno2_in_sensor1_frame.ori[0];
					sensor2CrctPNOArc.ori[1] = pno2_in_sensor1_frame.ori[1];
					sensor2CrctPNOArc.ori[2] = pno2_in_sensor1_frame.ori[2];
					sensor2CrctPNOArc.ori[3] = pno2_in_sensor1_frame.ori[3];


					// 检查是否需要 boresight
					if (g_doBoresight)
					{
						boresightSensor1(); // 这里执行你的 boresight 函数
						boresightSensor2();
						sensor2PosOffset = relPos2 - cVector3d(0, 0, ResolvedRateControl.h_0 * 0.001);

						std::cout << "[INFO] sensor2PosOffset => "
							<< sensor2PosOffset.str() << std::endl;

						g_doBoresight = false;
					}

					std::cout <<  "   RELPos: " << sensor2CrctPNOArc.pos[0] << ", " << sensor2CrctPNOArc.pos[1] << ", " << sensor2CrctPNOArc.pos[2] << std::endl;

					//------------------------------Close loop Control---------------------------
					//------------------------------P to L model---------------------------------

					cVector3d targetPos = TrajTarget * 1000;  // 目标位置 (mm)
					if (g_enableControl)
					{
						auto result = ResolvedRateControl.updateMotionCloseLoop(targetPos, sensor2CrctPNOArc);
						//std::cout << "   RELPos: " << targetPos << "   RELPos: " << sensor2CrctPNOArc.pos[0] << ", " << sensor2CrctPNOArc.pos[1] << ", " << sensor2CrctPNOArc.pos[2] << std::endl;
						cVector3d newPos = result[0];       // 计算的新位置
						cVector3d newPressure = result[1];  // 计算的新压力
						double error = (newPos - targetPos).length();
						//std::cout << "Error: " << error << " = " << newPos << " - " << targetPos << std::endl;
						//std::cout << "newpos " << newPos << std::endl;
					
							//if (error < 1)
							//{
								cVector3d PressuretoArduino = presCurr;
								//cVector3d PressuretoArduino(20,20,20);
								std::string sendStr = PressuretoArduino.str();
								arduinoWriteData(50, sendStr);
								//std::cout << "sendStr: "
								//	<< sendStr << "\r";
							//}
							//else
							//{
								presCurr = newPressure; // 
							//}
					}
					else
					{
						std::string sendZero = ZeroPressure.str();
						arduinoWriteData(50, sendZero);
					}
			    //------------------------------------PID------------------------------------




				// ---------------------------Control End--------------------------------



					// 如果需要记录传感器数据到CSV
					if (recordSensorDataToCSV)
					{
						// 拼装CSV行
						std::string timeStr = std::to_string((double)ViperTimestamp);
						std::vector<std::string> row;
						row.push_back(timeStr);

						// Sensor1 pos
						row.push_back(std::to_string(pno_1.pos[0]));
						row.push_back(std::to_string(pno_1.pos[1]));
						row.push_back(std::to_string(pno_1.pos[2]));

						// Sensor1 ori
						row.push_back(std::to_string(pno_1.ori[0]));
						row.push_back(std::to_string(pno_1.ori[1]));
						row.push_back(std::to_string(pno_1.ori[2]));
						row.push_back(std::to_string(pno_1.ori[3]));

						// Sensor2 pos
						row.push_back(std::to_string(pno_2.pos[0]));
						row.push_back(std::to_string(pno_2.pos[1]));
						row.push_back(std::to_string(pno_2.pos[2]));

						// Sensor2 ori
						row.push_back(std::to_string(pno_2.ori[0]));
						row.push_back(std::to_string(pno_2.ori[1]));
						row.push_back(std::to_string(pno_2.ori[2]));
						row.push_back(std::to_string(pno_2.ori[3]));

						// Sensor2 in Sensor1 frame
						row.push_back(std::to_string(pno2_in_sensor1_frame.pos[0]));
						row.push_back(std::to_string(pno2_in_sensor1_frame.pos[1]));
						row.push_back(std::to_string(pno2_in_sensor1_frame.pos[2]));

						row.push_back(std::to_string(pno2_in_sensor1_frame.ori[0]));
						row.push_back(std::to_string(pno2_in_sensor1_frame.ori[1]));
						row.push_back(std::to_string(pno2_in_sensor1_frame.ori[2]));
						row.push_back(std::to_string(pno2_in_sensor1_frame.ori[3]));

						writeToCSV(row);
					}
				
			}
		
			*/

///////////////

		//tool->setDeviceLocalPos(GrabFramePNO((MYPNOTYPE*)&(f.pF)[0], pSD, 1)/25.4);

		//tool->updateFromDevice();



		// compute interaction forces
		tool->computeInteractionForces();


		// 在主触觉循环中，在现有碰撞检测代码后添加：

		/////////////////////////////////////////////////////////////////////////
		// 实验模式处理
		/////////////////////////////////////////////////////////////////////////
		if (experimentMode && userStudy && userStudy->isTrialActive()) {
			// 显示立方体
			leftCube->setEnabled(true);
			rightCube->setEnabled(true);

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

		}
		else {
			// 非实验模式时隐藏立方体
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
		//tool->applyToDevice();  
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

/*

void openCSV(const std::string& filename)
{
	csvFile.open(filename, std::ios::app);
	if (!csvFile.is_open()) {
		std::cerr << "Unable to open file" << std::endl;
		return;
	}
	// 写表头（只要在第一次写时写一次即可）
	csvFile << "ViperTimestamp,"
		<< "S1_posX,S1_posY,S1_posZ,S1_oriW,S1_oriX,S1_oriY,S1_oriZ,"
		<< "S2_posX,S2_posY,S2_posZ,S2_oriW,S2_oriX,S2_oriY,S2_oriZ,"
		<< "Rel_posX,Rel_posY,Rel_posZ,Rel_oriW,Rel_oriX,Rel_oriY,Rel_oriZ\n";
	csvFile.flush();
}
*/

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

void DisplayError(int32_t err, const char* szMsg)
{
	cout << szMsg << " : " << vpcmdif_errorStr(err);

	cout << endl;
}
void DisplaySEUs()
{
	const char* szrpt = 0;
	int r = 0;
	if (r = vpctx_dev_report(g_ctx, szrpt))
	{
		DisplayError(r, "vpctx_report");
	}

	cout << endl << "SEUs Info:" << endl;
	cout << "ID   : Serial Num       : Name              " << endl;
	cout << "------------------------------------------------" << endl;
	if (szrpt)
	{
		cout << szrpt;
	}
	else
		cout << "None found." << endl;

}
bool Connect()
{
	bool bRet = false;
	int  r = 0;
	string str;

	size_t ctxsize = sizeof(g_ctx);
	vpcmdif_trace(g_bTraceMode ? g_TraceLevel : 0);
	if (r = vpcmdif_init(g_ctx))
	{
		//error initializing vpcmdif lib
		DisplayError(r, "vpcmdif_init");
	}
	else if (r = usbcnx_fxn_info[g_TrackerType].func(g_ctx, g_hnd, g_pid))
	{
		DisplayError(r, usbcnx_fxn_info[g_TrackerType].szfunc);
	}
	else
	{
		bRet = true;
		DisplayError(r, "vpctx_connectusb");
		cout << r << "vpctx_connectusb";
		DisplaySEUs();
	}

	return bRet;
}

void Disconnect()
{
	int r = vpcmdif_release(g_ctx);
	DisplayError(r, "vpcmdif_release");
};

bool StartCont()
{
	int r = 0;
	if (r = vpcmd_dev_contpno(g_ctx, g_hnd, CMD_ACTION_SET))
		DisplayError(r, "vpcmd_dev_contpno SET");

	return r == 0;
}
bool StopCont()
{
	int r = 0;
	if (r = vpcmd_dev_contpno(g_ctx, g_hnd, CMD_ACTION_RESET))
		DisplayError(r, "vpcmd_dev_contpno RESET");

	return r == 0;
}



//------------------------------------------------------------------------------
// A function to set boresight on sensor #0 (which you call "sensor1").
// This makes the Polhemus hardware treat the current physical pose as "no rotation".
//------------------------------------------------------------------------------
bool boresightSensor1()
{
	// 说明：sensor1 对应 ID=0
	int sensorID = 0;

	//--- 0) 确保已进入连续采集 (StartCont 或 vpcmd_dev_contpno(...,SET))，否则 FIFO 无数据 ---

	//--- 1) 多次尝试读取 FIFO 帧 ---
	vpFrameInfo tmpFrame;
	const int maxTries = 50;    // 最多重试 50 次
	const int sleepMs = 10;    // 每次失败后等待10毫秒
	bool gotFrame = false;

	for (int i = 0; i < maxTries; i++)
	{
		int ret = vpctx_dev_fifopnof(g_ctx, g_hnd, tmpFrame);
		if (ret == 0)
		{
			// 成功读到帧
			if (tmpFrame.uiSize > 0 && tmpFrame.pF != nullptr)
			{
				gotFrame = true;
				break; // 跳出循环
			}
		}
		else if (ret != E_CTXERR_NO_MORE_DATA)
		{
			// 如果不是 E_CTXERR_NO_MORE_DATA 就是其它错误(无效句柄等)
			DisplayError(ret, "vpctx_dev_fifopnof in boresightSensor1()");
			return false;
		}
		// 若只是 E_CTXERR_NO_MORE_DATA 或空帧 => 等等再试
		cSleepMs(sleepMs);
	}

	if (!gotFrame)
	{
		std::cout << "[ERROR] boresightSensor1(): no valid FIFO frame after retry.\n";
		return false;
	}

	//--- 2) 解析这一帧 ---
	if (tmpFrame.uiSize == 0 || tmpFrame.pF == nullptr)
	{
		std::cout << "[ERROR] boresightSensor1(): empty frame.\n";
		return false;
	}

	MYPNOTYPE* pF = (MYPNOTYPE*)&(tmpFrame.pF)[0];
	if (!pF)
	{
		std::cout << "[ERROR] Null pointer in boresightSensor1().\n";
		return false;
	}

	// 解析 sensor1 (ID=0) 的 PNODATA
	PNODATA pnoCur = GrabFramePNO(pF, sensorID);

	//--- 3) 位置单位转换，如 cm->m ---
	//  假设 Polhemus 输出是 cm，则乘0.01 得到米
	double scale = 0.01;
	cVector3d pnoCurPos(
		pnoCur.pos[0] * scale,
		pnoCur.pos[1] * scale,
		pnoCur.pos[2] * scale
	);

	//--- 4) 记录软件 offset => 后续 pos - g_posOffset = 0 ---
	g_posOffset = pnoCurPos;

	//--- 5) 硬件 boresight => 让当前姿态视为无旋转 ---
	CBoresightCfg newBore;
	memset(&newBore, 0, sizeof(newBore));
	newBore.Units() = (eViperOriUnits)config.ori_units;

	if (config.ori_units == ORI_QUATERNION)
	{
		// quaternion => [w=1, x=y=z=0]
		newBore.params[0] = 1.0f;
		newBore.params[1] = 0.0f;
		newBore.params[2] = 0.0f;
		newBore.params[3] = 0.0f;
	}
	else
	{
		// euler => [Az=0, El=0, Roll=0]
		newBore.params[0] = 0.0f;
		newBore.params[1] = 0.0f;
		newBore.params[2] = 0.0f;
		newBore.params[3] = 0.0f;
	}

	int r2 = vpcmd_sns_boresight(g_ctx, g_hnd, sensorID, CMD_ACTION_SET, newBore);
	if (r2 != 0)
	{
		DisplayError(r2, "vpcmd_sns_boresight SET boresight");
		return false;
	}

	//--- 提示 ---
	std::cout << "[INFO] boresightSensor1() done. orientation=zero.\n"
		<< "       software offset => " << g_posOffset.str() << std::endl;

	return true;
}

bool boresightSensor2()
{
	int sensorID = 1; // sensor2

	vpFrameInfo tmpFrame;
	const int maxTries = 50;
	const int sleepMs = 10;
	bool gotFrame = false;

	// 多次尝试读取 FIFO
	for (int i = 0; i < maxTries; i++)
	{
		int ret = vpctx_dev_fifopnof(g_ctx, g_hnd, tmpFrame);
		if (ret == 0)
		{
			if (tmpFrame.uiSize > 0 && tmpFrame.pF != nullptr)
			{
				gotFrame = true;
				break;
			}
		}
		else if (ret != E_CTXERR_NO_MORE_DATA)
		{
			DisplayError(ret, "vpctx_dev_fifopnof in boresightSensor2_2()");
			return false;
		}
		cSleepMs(sleepMs);
	}

	if (!gotFrame)
	{
		std::cout << "[ERROR] boresightSensor2_2(): no valid FIFO frame after retry.\n";
		return false;
	}

	if (tmpFrame.uiSize == 0 || tmpFrame.pF == nullptr)
	{
		std::cout << "[ERROR] boresightSensor2_2(): empty frame.\n";
		return false;
	}
	MYPNOTYPE* pF = (MYPNOTYPE*)&(tmpFrame.pF)[0];
	if (!pF)
	{
		std::cout << "[ERROR] Null pointer in boresightSensor2_2().\n";
		return false;
	}

	// 抓取 sensor2
	PNODATA pnoCur = GrabFramePNO(pF, sensorID);

	// 若Polhemus输出单位是cm => *0.01 => m
	double scale = 0.01;
	cVector3d pnoCurPos(
		pnoCur.pos[0] * scale,
		pnoCur.pos[1] * scale,
		pnoCur.pos[2] * scale
	);

	// 如需给 sensor2 做位置 offset，可定义 g_posOffset2
	// g_posOffset2 = pnoCurPos; // (可选)

	// 设置硬件 boresight => [无旋转]
	CBoresightCfg newBore;
	memset(&newBore, 0, sizeof(newBore));
	newBore.Units() = (eViperOriUnits)config.ori_units;

	if (config.ori_units == ORI_QUATERNION)
	{
		newBore.params[0] = 1.0f;
		newBore.params[1] = 0.0f;
		newBore.params[2] = 0.0f;
		newBore.params[3] = 0.0f;
	}
	else
	{
		newBore.params[0] = 0.0f;
		newBore.params[1] = 0.0f;
		newBore.params[2] = 0.0f;
		newBore.params[3] = 0.0f;
	}

	int r2 = vpcmd_sns_boresight(g_ctx, g_hnd, sensorID, CMD_ACTION_SET, newBore);
	if (r2 != 0)
	{
		DisplayError(r2, "vpcmd_sns_boresight SET boresight sensor2");
		return false;
	}

	std::cout << "[INFO] boresightSensor2_2() done. orientation=zero.\n"
		//<< "pos offset => " << g_posOffset2.str() << std::endl;
		;
	return true;
}




bool clearBoresightSensor1()
{
	// The same ID used in boresightSensor1()
	int sensorID = 0;
	CBoresightCfg emptyBore;
	memset(&emptyBore, 0, sizeof(emptyBore));

	// 1) First clear the hardware boresight (reset orientation compensation)
	int r = vpcmd_sns_boresight(g_ctx, g_hnd, sensorID, CMD_ACTION_RESET, emptyBore);
	if (r != 0)
	{
		DisplayError(r, "vpcmd_sns_boresight RESET");
		return false;
	}

	// 2) Also reset the software displacement offset g_posOffset to (0,0,0)
	//    This way, any subsequent calculation like (pno.pos - g_posOffset) 
	//    becomes (pno.pos - 0), i.e., the original coordinates.
	g_posOffset.set(0.0, 0.0, 0.0);

	cout << "[INFO] clearBoresightSensor1: sensor #" << sensorID
		<< " boresight removed, and g_posOffset reset to zero." << endl;

	return true;
}






// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu
// Minimal function to display PNO frames
PNODATA GrabFramePNO(MYPNOTYPE* pv, uint32_t s_index)
{
	// We’ll ignore the optional 'pts' parameter for simplicity
	if (!pv)
	{
		/*std::cout << "Error: PNO pointer or SENFRAMEDATA pointer is null.\n";*/
		return { 1,1,1,1,1,1,1 };
	}

	// Retrieve SEU ID, frame number, and number of sensors from the header
	uint32_t seuID = pv->seupno.seuid;
	uint32_t frameNum = pv->seupno.frame;
	uint32_t sensorCount = pv->seupno.sensorCount;

	PNODATA currentPNO{};
	// We'll just return how many bytes we "consumed" from the sensor array
	int32_t totalBytes = 0;
	cVector3d prevPos(0, 0, 0); // Replace with the appropriate position type
	SENFRAMEDATA* pSD = &(pv->sarr[s_index]);

	// Print basic PNO: position [x, y, z], orientation [ori0, ori1, ori2, ori3]
	// If orientation is in quaternion form, ori[3] is the 'w' or final component;
	// if Euler angles, typically only ori[0..2] matter. 
	// For minimal example, just print what’s stored:

	// Get the current position

	currentPNO.pos[0] = pSD->pno.pos[0];
	currentPNO.pos[1] = pSD->pno.pos[1];
	currentPNO.pos[2] = pSD->pno.pos[2];
	currentPNO.ori[0] = pSD->pno.ori[0];
	currentPNO.ori[1] = pSD->pno.ori[1];
	currentPNO.ori[2] = pSD->pno.ori[2];
	currentPNO.ori[3] = pSD->pno.ori[3];
	//pno = cVector3d(pSD->pno.pos[0], pSD->pno.pos[1], pSD->pno.pos[2]);
	//cout << pno << endl;
	return currentPNO;
}


//double getCurrentTime()
//{
//	// 用 static 变量保存程序启动时的时间点，
//	// 这样每次调用 getCurrentTime() 都会以同一基准计算相对时间
//	static const auto startTime = std::chrono::system_clock::now();
//	auto now = std::chrono::system_clock::now();
//	// 计算从 startTime 到现在的时间间隔
//	auto duration = now - startTime;
//	// 将间隔转换为以微秒为单位的 double 数值，再除以 1e6 得到秒数
//	double seconds = std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(duration).count() / 1e6;
//	return seconds;
//}



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

bool mtZeroMarkers(mtw::MicronTracker& MT, double h0_mm)
{
	// 1) 更新一帧，确保识别成功
	if (!MT.update())
	{
		std::cerr << "[MT] update failed – cannot zero\n";
		return false;
	}

	// 2) 读取 fr / Actuator 的绝对姿态
	mtw::Pose fr, act;
	if (!MT.getPoseAndRotMat("fr", fr) ||
		!MT.getPoseAndRotMat("Actuator", act))
	{
		std::cerr << "[MT] pose query failed – cannot zero\n";
		return false;
	}

	// 3) 当前 Actuator 在 fr 坐标系下的位置、姿态
	double relPosNow[3];
	if (!MT.getRelPos("Actuator", "fr", relPosNow))
	{
		std::cerr << "[MT] getRelPos(Actuator,fr) failed – cannot zero\n";
		return false;
	}

	double relRNow[9];
	if (!MT.getRelRotMat("Actuator", "fr", relRNow))
	{
		std::cerr << "[MT] getRelRotMat(Actuator,fr) failed – cannot zero\n";
		return false;
	}

	// 4) 保存基准 --------------------------------------------------
	/* (1) fr / Actuator 绝对姿态 */
	for (int i = 0; i < 3; ++i) {
		g_mtZero.frPos[i] = fr.pos[i];
		g_mtZero.actPos[i] = act.pos[i];
	}
	for (int i = 0; i < 9; ++i) {
		g_mtZero.frR[i] = fr.rot[i];
		g_mtZero.actR[i] = act.rot[i];
	}

	// 计算 R0（零点姿态）的 z-轴向量（列 2）
	double z0[3] = { relRNow[6], relRNow[7], relRNow[8] };   // col-major → 第 3 列

	// t_off = relPosNow - R0 · (0,0,h0)
	g_mtZero.relPosOff[0] = relPosNow[0] - h0_mm * z0[0];
	g_mtZero.relPosOff[1] = relPosNow[1] - h0_mm * z0[1];
	g_mtZero.relPosOff[2] = relPosNow[2] - h0_mm * z0[2];

	/* (3) 相对旋转偏移 relROff */
	for (int i = 0; i < 9; ++i)
		g_mtZero.relROff[i] = relRNow[i];

	g_mtZero.valid = true;

	std::cout << "[MT] zero done.\n"
		<< "   relPosOff = (" << g_mtZero.relPosOff[0] << ", "
		<< g_mtZero.relPosOff[1] << ", "
		<< g_mtZero.relPosOff[2] << ") mm\n";
	return true;
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

//void openExperimentCSV(const std::string& filename)
//{
//	experimentCsvFile.open(filename, std::ios::out); // 使用out而不是app，每次创建新文件
//	if (!experimentCsvFile.is_open()) {
//		std::cerr << "Unable to open experiment file\n";
//		return;
//	}
//	// 写入实验数据表头
//	experimentCsvFile << "UserChoice,ReferenceStiffness,ComparisonStiffness,ReactionTime\n";
//	experimentCsvFile.flush();
//	std::cout << "Experiment CSV file created: " << filename << std::endl;
//}
//
//void writeExperimentDataToCSV(int userChoice, double refStiffness, double compStiffness, double reactionTime)
//{
//	if (experimentCsvFile.is_open()) {
//		experimentCsvFile << userChoice << ","
//			<< refStiffness << ","
//			<< compStiffness << ","
//			<< reactionTime << "\n";
//		experimentCsvFile.flush();
//	}
//	else {
//		std::cerr << "Experiment file not open for writing" << std::endl;
//	}
//}
//
//void closeExperimentCSV() {
//	if (experimentCsvFile.is_open()) {
//		experimentCsvFile.close();
//	}
//}