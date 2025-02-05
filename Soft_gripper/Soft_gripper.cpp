//==============================================================================
/*
	Software License Agreement (BSD License)
	Copyright (c) 2003-2016, CHAI3D.
	(www.chai3d.org)

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	* Redistributions of source code must retain the above copyright
	notice, this list of conditions and the following disclaimer.

	* Redistributions in binary form must reproduce the above
	copyright notice, this list of conditions and the following
	disclaimer in the documentation and/or other materials provided
	with the distribution.

	* Neither the name of CHAI3D nor the names of its contributors may
	be used to endorse or promote products derived from this software
	without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.

	\author    <http://www.chai3d.org>
	\author    Francois Conti
	\version   3.2.0 $Rev: 1925 $
*/

//============================Adruino===========================================
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

//---------------------------Pohelmus-------------------------------------------
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
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

#include "frameTrans.hpp"

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
// HAPTIC INVERSE KINEMATIC CLASS
//------------------------------------------------------------------------------
cVector3d test_pressure{ 0,0,0 };
class MatlabCalc
{
private:
	const int d;
	const double b;
	const double k;
	const double h_0;

	cVector3d my_pressure;
	cVector3d my_pos;
	double delta_t;
	double tolerance;
	const int MAX_ITERS;
	const double EPSILON;
	const double MULTIPLIER;
public:
	MatlabCalc(/* args */);
	~MatlabCalc();
	void setTargetPos(cVector3d& new_target_pos);
	cVector3d calcForwardPressurePos(cVector3d& x);
	cVector3d calcForwardPressurePosInverse(cVector3d& x);
	cVector3d MatlabCalc::TaskToConfig();
	cVector3d forwardKinematic(cVector3d& pressure, bool pressure_flag); // true, input is press
	double calcKappa(cVector3d& length);
	double calcPhi(cVector3d& length);
	double calcTheta(cVector3d& length);
	cMatrix3d calcJacobian(cVector3d& length);
	vector<cVector3d> updateMotion(cVector3d& pressure, cVector3d& pos);
	void reachTarget(cVector3d& target_pos);
	cVector3d getDevicePressure();
	double heaviside(double var);
};

MatlabCalc::MatlabCalc()
	: d(12),
	b(16.351),
	k(0.232),
	h_0(20),
	MAX_ITERS(5000),
	MULTIPLIER(0.2),
	EPSILON(1e-6),
	delta_t(0.02),
	tolerance(0.05),
	my_pressure{ 20, 20, 20 },
	my_pos{ 0, 0, 0 } // Initializing constants and other variables
{

}


MatlabCalc::~MatlabCalc()
{

}
double MatlabCalc::heaviside(double var)
{
	if (var < 0)
	{
		return 0;
	}
	else if (var > 0)
	{
		return 1;
	}
	else
	{
		return 0.5;
	}
}

void MatlabCalc::setTargetPos(cVector3d& new_target_pos)
{
	my_pos = new_target_pos;
}

cVector3d MatlabCalc::calcForwardPressurePos(cVector3d& x)
{
	// New modify
	size_t n{ 3 };
	cVector3d result;
	for (size_t i{ 0 }; i < 3; ++i)
	{
		result(i) = heaviside(x(i) - h_0) * (k * x(i) + b) + heaviside(h_0 - x(i)) * (k * x(i) + b);
	}
	return result;

}
cVector3d MatlabCalc::calcForwardPressurePosInverse(cVector3d& x)
{
	// New modify
	size_t n{ 3 };
	cVector3d result;
	for (size_t i{ 0 }; i < 3; ++i)
	{
		result(i) = heaviside(x(i) - h_0) * (x(i) - b) / k + heaviside(h_0 - x(i)) * (x(i) - b) / k;
	}
	return result;

}
double MatlabCalc::calcKappa(cVector3d& length)
{
	return 2 * sqrt(pow(length(0), 2) + pow(length(1), 2) + pow(length(2), 2)
		- length(0) * length(1) - length(1) * length(2) - length(0) * length(2))
		/ (d * (length(0) + length(1) + length(2)));
}

double MatlabCalc::calcPhi(cVector3d& length)
{
	return atan2(sqrt(3) * (length(1) + length(2) - 2 * length(0)), 3 * (length(1) - length(2)));
}

double MatlabCalc::calcTheta(cVector3d& length)
{
	return 2 * sqrt(pow(length(0), 2) + pow(length(1), 2) + pow(length(2), 2)
		- length(0) * length(1) - length(1) * length(2) - length(0) * length(2))
		/ (d * 3);
}

cVector3d MatlabCalc::forwardKinematic(cVector3d& pressure, bool pressure_flag)
{
	cVector3d length = pressure;


	if (pressure_flag)
	{
		length = calcForwardPressurePos(pressure);
	}


	double phi = calcPhi(length);
	double theta = calcTheta(length);
	double kappa = calcKappa(length);

	double x_out = cos(phi) * (1 - cos(theta)) / kappa;
	double y_out = sin(phi) * (1 - cos(theta)) / kappa;
	double z_out = sin(theta) / kappa;

	return cVector3d(x_out, y_out, z_out);
}

cMatrix3d MatlabCalc::calcJacobian(cVector3d& length)
{
	size_t n = 3;
	cMatrix3d J;
	J.identity();
	cVector3d l_perturbed = length;
	cVector3d origin = forwardKinematic(length, false);
	for (size_t i = 0; i < n; ++i)
	{
		l_perturbed = length;
		l_perturbed(i) += EPSILON;

		cVector3d origin_perturbed = forwardKinematic(l_perturbed, false);

		for (size_t j = 0; j < n; ++j)
		{
			J(j, i) = (origin_perturbed(j) - origin(j)) / EPSILON;
		}
	}
	return J;
}

vector<cVector3d> MatlabCalc::updateMotion(cVector3d& pressure, cVector3d& pos)
{
	if (pressure(0) == pressure(1) && pressure(1) == pressure(2))
	{
		pressure(2) += 0.00001;
	}
	cVector3d current_pos = forwardKinematic(pressure, true);
	cVector3d current_length = calcForwardPressurePos(pressure);
	cMatrix3d J = calcJacobian(current_length);
	cMatrix3d J_inverse = J;
	J_inverse.invert();
	cVector3d error = pos - current_pos;
	cVector3d target_vel = error / error.length() * MULTIPLIER;
	cVector3d l_dot = J_inverse * target_vel;
	cVector3d new_length = current_length + l_dot * delta_t;
	cVector3d new_pos = forwardKinematic(new_length, false);
	cVector3d new_pressure = calcForwardPressurePosInverse(new_length);
	return { new_pos, new_pressure };
}

void MatlabCalc::reachTarget(cVector3d& target_pos)
{
	size_t iter{ 0 };
	my_pressure = cVector3d(20, 20, 20);
	while (iter < MAX_ITERS)
	{

		my_pos = updateMotion(my_pressure, target_pos)[0];
		test_pressure = updateMotion(my_pressure, target_pos)[1];
		double error = (my_pos - target_pos).length();
		if (error < tolerance)
		{
			// SAFETY CHECK
			for (size_t i{ 0 }; i < 3; ++i)
			{
				if (test_pressure(i) < 0)
				{
					test_pressure(i) = 0;
					//cout << "\rWaiting for acutating.." << flush;
				}
				if (test_pressure(i) >= 35)
				{
					test_pressure(i) = 35;
					//cout << "\rAxis " << i << "reached limitation" << flush;
				}
			}
			//cout << "\033[F\033[F\033[F";
			//cout << "Target Reached, iteration: " << iter << endl;
			//cout << "Input Position: " << my_pos << endl;
			//cout << "Input Pressure: " << test_pressure << endl;
			std::cout.flush();
			break;
		}

		my_pressure = test_pressure;
		iter += 1;
	}
}
//cVector3d MatlabCalc::TaskToConfig() {}
cVector3d MatlabCalc::getDevicePressure()
{
	return my_pressure;
}
//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a file to store force
std::ofstream csvFile;
auto now{ std::chrono::system_clock::now() };
// Convert the time point to duration in microseconds
auto duration{ now.time_since_epoch() };
// Convert to microseconds and then to double seconds
string timestamp;
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
bool sendPosToArduino{ false };
bool writeForceToCSV{ false };
const char* portNumber = "\\\\.\\COM3";
void arduinoWriteData(unsigned int delay_time, string send_str);
cVector3d proxyPos;
double posMagnitude;
//------------------------------------------------------------------------------
// DECLARED Arduino Connection
//------------------------------------------------------------------------------
MatlabCalc myIK;
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
bool clearBoresightSensor1();
PNODATA GrabFramePNO(MYPNOTYPE* pv, uint32_t s_index);
//PNODATA convertToSensor1Frame(const PNODATA& sensor1, const PNODATA& sensor2);
/****************************************************************************
TEST Identifier
 ****************************************************************************/

//cMatrix3d quaternionToMatrix(const cQuaternion& q);
//cVector3d matrixVectorMultiply(const cMatrix3d& R, const cVector3d& v);
//cQuaternion quaternionConjugate(const cQuaternion& q);
//cQuaternion quaternionNormalize(const cQuaternion& q);
//cQuaternion quaternionMultiply(const cQuaternion& q1, const cQuaternion& q2);
//cMatrix3d matrixTranspose(const cMatrix3d& M);


//==============================================================================
/*
	DEMO:   13-primitives.cpp

	This example illustrates how to build simple triangle based mesh primitives
	using the functions provided in file graphics/CPrimitives.h
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
	cout << "Demo: 13-primitives" << endl;
	cout << "Copyright 2003-2016" << endl;
	cout << "-----------------------------------" << endl << endl << endl;
	cout << "Keyboard Options:" << endl << endl;
	cout << "[s] - Save copy of shadowmap to file" << endl;
	cout << "[f] - Enable/Disable full screen mode" << endl;
	cout << "[m] - Enable/Disable vertical mirroring" << endl;
	cout << "[q] - Exit application" << endl;
	cout << "[b] - Boresight sensor1 (0,0,0)+(0,0,0,1)" << endl;
	cout << "[r] - Clear boresight for sensor1 " << endl;

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
	camera->set(cVector3d(2, 0, 1.5),    // camera position (eye)
		cVector3d(0.0, 0.0, 0.5),    // lookat position (target)
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
	light->setLocalPos(2, 0, 1.5);

	// define the direction of the light beam
	light->setDir(-0.5, -0.5, -0.5);

	// enable this light source to generate shadows
	light->setShadowMapEnabled(true);

	// set the resolution of the shadow map
	//light->m_shadowMap->setQualityLow();
	light->m_shadowMap->setQualityMedium();

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
	double toolRadius = 0.0002;

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
	//world->addChild(base);

	// build mesh using a cylinder primitive
	cCreateCylinder(base,
		0.01,
		0.5,
		36,
		1,
		10,
		true,
		true,
		cVector3d(0.0, 0.0, -0.05),
		cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(0), C_EULER_ORDER_XYZ)
	);

	// set material properties
	base->m_material->setGrayGainsboro();
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
	cCreateBox(box, 0.3, 0.3, 0.1,
		cVector3d(-0.070, -0.020, 0.20), cIdentity3d(), cColorf(0, 1, 0));

	// position object
	box->setLocalPos(0.1, 0.0, 0.1);

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

	// set material properties
	cone->m_material->setBluePaleTurquoise();
	cone->m_material->setStiffness(0.95 * maxStiffness);

	// build collision detection tree
	cone->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	cone->setUseDisplayList(true);



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
	font = NEW_CFONTCALIBRI20();

	// create a label to display the haptic and graphic rate of the simulation
	labelRates = new cLabel(font);
	labelRates->m_fontColor.setBlack();
	camera->m_frontLayer->addChild(labelRates);

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
	config.pos_units = POS_CM;
	config.ori_units = ORI_QUATERNION;
	vpcmd_dev_units(g_ctx, g_hnd, CMD_ACTION_SET, &config);
	StartCont();
	// create a thread which starts the main haptics rendering loop
	hapticsThread = new cThread();
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

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
		boresightSensor1();
	}
	else if (a_key == GLFW_KEY_I)
	{
		sendPosToArduino = !sendPosToArduino;
		cout << "Sending Position Info to Arduino: " << sendPosToArduino << endl;
	}
	else if (a_key == GLFW_KEY_O)
	{
		openCSV("Jiaji_data");
		cout << "CSV file is rdy for saving...Press K for writing force into it" << endl;
	}
	else if (a_key == GLFW_KEY_K)
	{
		writeForceToCSV = !writeForceToCSV;
		cout << "Writing Status: " << writeForceToCSV << endl;
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
}

//------------------------------------------------------------------------------

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

	// Arduino Pos to Pressure, TODO
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

	try
	{
		// Spring Kp = 4
		myIK.reachTarget(proxyPos * posMagnitude * 4 + cVector3d(0, 0, 20));
	}
	catch (const std::exception& e)
	{
		cout << "Something is really wrong" << e.what() << endl;
		MatlabCalc myIK;

	}
	if (sendPosToArduino)
	{
		arduinoWriteData(1, test_pressure.str());
	}
	if (writeForceToCSV)
	{
		now = std::chrono::system_clock::now();
		duration = now.time_since_epoch();
		timestamp = std::to_string(std::chrono::duration<double>(duration).count());
		cVector3d posData = tool->getDeviceLocalForce();
		writeToCSV(std::vector<string> {timestamp, std::to_string(posData(0)), std::to_string(posData(1)), std::to_string(posData(2))}); // Waiting for testing
	}

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
	double radius = 0.1; // 12mm
	double height = 0.1; // 3mm
	cShapeCylinder* cylinderTop = new cShapeCylinder(radius, radius, height);
	cShapeCylinder* cylinderBot = new cShapeCylinder(radius, radius, height);

	world->addChild(cylinderTop);
	world->addChild(cylinderBot);

	cylinderTop->m_material->setRed();

	double relRadius = 0.05;
	double relHeight = 0.05;
	cShapeCylinder* cylinderTopRel = new cShapeCylinder(relRadius, relRadius, relHeight);
	world->addChild(cylinderTopRel);

	// 给它一个不同的颜色
	cylinderTopRel->m_material->setGreenForest();

	// 让它在 3D 场景里“靠左侧”一点, 以免与主场景重叠
	// (可在后面动态再加偏移)
	cylinderTopRel->setLocalPos(-0.5, 0.0, 0.0); // 先往 X 方向左移



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
		vpctx_dev_lastpnof(g_ctx, g_hnd, f);
	
		// 假设 updateHaptics() 或类似循环中：
		if ((MYPNOTYPE*)&(f.pF)[0])
		{
			// a) Sensor1 (cylinderBot)，使用 boresight 后姿态、软件 offset 后位置
			pno_1 = GrabFramePNO((MYPNOTYPE*)&(f.pF)[0], 0);
			double scale = 0.1; // cm->m
			cVector3d curPos1(
				pno_1.pos[0] * scale,
				pno_1.pos[1] * scale,
				pno_1.pos[2] * scale
			);
			cVector3d relPos1 = curPos1 - g_posOffset; // 让 boresight 那刻作为0
			cQuaternion q1(pno_1.ori[0], pno_1.ori[1], pno_1.ori[2], pno_1.ori[3]);
			cMatrix3d r1 = quaternionToMatrix(q1);

			cylinderBot->setLocalPos(relPos1);
			cylinderBot->setLocalRot(r1);

			// b) Sensor2 (cylinderTop)，不做 boresight，但减同一个 offset
			pno_2 = GrabFramePNO((MYPNOTYPE*)&(f.pF)[0], 1);
			cVector3d curPos2(
				pno_2.pos[0] * scale,
				pno_2.pos[1] * scale,
				pno_2.pos[2] * scale
			);
			cVector3d relPos2 = curPos2 - g_posOffset;
			cQuaternion q2(pno_2.ori[0], pno_2.ori[1], pno_2.ori[2], pno_2.ori[3]);
			cMatrix3d r2 = quaternionToMatrix(q2);

			cylinderTop->setLocalPos(relPos2);
			cylinderTop->setLocalRot(r2);

			// c) 计算 sensor2 相对于 sensor1 的相对位姿 => 显示到 cylinderTopRel
			PNODATA pno2_in_sensor1_frame = convertToSensor1Frame(pno_1, pno_2);

			// 把相对坐标(单位仍是 cm)转换到 m
			cVector3d sensor2RelPos(
				pno2_in_sensor1_frame.pos[0] * scale,
				pno2_in_sensor1_frame.pos[1] * scale,
				pno2_in_sensor1_frame.pos[2] * scale
			);
			cQuaternion q2_rel(
				pno2_in_sensor1_frame.ori[0],
				pno2_in_sensor1_frame.ori[1],
				pno2_in_sensor1_frame.ori[2],
				pno2_in_sensor1_frame.ori[3]
			);
			cMatrix3d r2_rel = quaternionToMatrix(q2_rel);

			// 如果我们也希望"相对圆柱" 以"sensor1世界位置"为基准 => 
			//   finalPosRel = sensor1世界位置 + sensor2对sensor1的局部位置
			//   再加一个小偏移 shift(例如Y方向0.3) 让它不与 sensor2 重叠
			cVector3d shift(0.0, 0.3, 0.0);
			cVector3d finalPosRel = shift + sensor2RelPos;
			//cVector3d finalPosRel(0.0, 0.3, 0.0);

			cylinderTopRel->setLocalPos(finalPosRel);
			cylinderTopRel->setLocalRot(r2_rel);

			// 也可以在此 debug:
			// std::cout << "Sensor2RelPos: " << sensor2RelPos.str() << "\n";
		}

		/////////////////////////////////////////////////////////////////////////
		// FRAME TRANS TEST
		/////////////////////////////////////////////////////////////////////////	

		//PNODATA sensor1_test, sensor2_test;



		//sensor1_test.pos[0] = 1; sensor1_test.pos[1] = 0; sensor1_test.pos[2] = 0;
		//sensor1_test.ori[0] = 0; sensor1_test.ori[1] = 0; sensor1_test.ori[2] = 1; sensor1_test.ori[3] = 0;
		//sensor2_test.pos[0] = 2; sensor2_test.pos[1] = 0; sensor2_test.pos[2] = 0;
		//sensor2_test.ori[0] = 0; sensor2_test.ori[1] = 0; sensor2_test.ori[2] = 1; sensor2_test.ori[3] = 0;

		//PNODATA result_test = convertToSensor1Frame(sensor1_test, sensor2_test);

		//// Print out the result 
		//std::cout << "Result: \n";
		//std::cout << "pos_rel_test: ("
		//	<< result_test.pos[0] << ", "
		//	<< result_test.pos[1] << ", "
		//	<< result_test.pos[2] << ")\n";
		//std::cout << "q_rel_test: ("
		//	<< result_test.ori[0] << ", "
		//	<< result_test.ori[1] << ", "
		//	<< result_test.ori[2] << ", "
		//	<< result_test.ori[3] << ")\n";

		/////////////////////////////////////////////////////////////////////////
		//Print out raw pno data 
		/////////////////////////////////////////////////////////////////////////
		//cout << "pno_1: "
		//	<< pno_1.pos[0] << ", " << pno_1.pos[1] << ", " << pno_1.pos[2] << ", "
		//	<< pno_1.ori[0] << ", " << pno_1.ori[1] << ", " << pno_1.ori[2] << ", " << pno_1.ori[3] << endl;

		//cout << "pno_2: "
		//	<< pno_2.pos[0] << ", " << pno_2.pos[1] << ", " << pno_2.pos[2] << ", "
		//	<< pno_2.ori[0] << ", " << pno_2.ori[1] << ", " << pno_2.ori[2] << ", " << pno_2.ori[3] << endl;
		// 
		//tool->setDeviceLocalPos(GrabFramePNO((MYPNOTYPE*)&(f.pF)[0], pSD, 1)/25.4);
		//tool->setDeviceLocalPos(0,0.1,0.1);

		tool->updateFromDevice();

		// compute interaction forces
		tool->computeInteractionForces();

		/////////////////////////////////////////////////////////////////////////
		// HAPTIC MANIPULATION
		/////////////////////////////////////////////////////////////////////////

		// compute transformation from world to tool (haptic device)
		cTransform world_T_tool = tool->getDeviceGlobalTransform();

		// get status of user switch
		bool button = tool->getUserSwitch(0);

		//
		// STATE 1:
		// Idle mode - user presses the user switch
		//
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

		//
		// STATE 2:
		// Selection mode - operator maintains user switch enabled and moves object
		//
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

		//
		// STATE 3:
		// Finalize Selection mode - operator releases user switch.
		//
		else
		{
			state = IDLE;
		}


		/////////////////////////////////////////////////////////////////////////
		// FINALIZE
		/////////////////////////////////////////////////////////////////////////

		// send forces to haptic device
		//tool->applyToDevice();  
	}

	// exit haptics thread
	simulationFinished = true;
}

//------------------------------------------------------------------------------
void arduinoWriteData(unsigned int delay_time, string send_str)
{
	static size_t send_count{ 0 };
	send_count += 1;
	send_str += "\n";
	if (send_count % delay_time == 0)
	{
		myDevice->writeSerialPort(send_str.c_str(), send_str.length());
		send_count = 0;
	}
	//Sleep(delay_time);
}

void openCSV(const std::string& filename) {
	csvFile.open(filename, std::ios::app);  // Open in append mode to keep writing data

	if (!csvFile.is_open()) {
		std::cerr << "Unable to open file" << std::endl;
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
	// 假设 sensorID=0 即“sensor1”，若硬件配置中 sensor1 是 ID=1，则改成1
	int sensorID = 0;

	// (1) 在调用 boresight 之前，先读取一帧当前姿态(位置+四元数)
	//     目的是获取“当前pos”以便软件端做 offset。
	// ------------------------------------------------------------------
	vpFrameInfo tmpFrame;
	int ret = vpctx_dev_lastpnof(g_ctx, g_hnd, tmpFrame);
	if (ret != 0)
	{
		DisplayError(ret, "vpctx_dev_lastpnof inside boresightSensor1()");
		return false;
	}

	MYPNOTYPE* pF = (MYPNOTYPE*)&(tmpFrame.pF)[0];
	if (!pF)
	{
		std::cout << "Error: Null pointer when reading Polhemus data in boresightSensor1()\n";
		return false;
	}

	// 读取 sensorID 对应 pno
	PNODATA pnoCur = GrabFramePNO(pF, sensorID);

	// （可选）若pos_units是CM，且我们要用米，则做换算
	//         取决于 config.pos_units
	//         如果你之前在 updateHaptics 中也做了 *0.1，这里就同步处理
	cVector3d pnoCurPos(
		pnoCur.pos[0] * 0.1,
		pnoCur.pos[1] * 0.1,
		pnoCur.pos[2] * 0.1
	);

	// 记录到全局 offset => 让后续 "pos - g_posOffset" = 0
	g_posOffset = pnoCurPos;

	// (2) 设置硬件 boresight => 让当前姿态变为“无旋转”
	// ------------------------------------------------------------------
	CBoresightCfg newBore;
	memset(&newBore, 0, sizeof(newBore));
	newBore.Units() = (eViperOriUnits)config.ori_units;

	if (config.ori_units == ORI_QUATERNION)
	{
		// [w, x, y, z] = [1, 0, 0, 0]
		newBore.params[0] = 1.0f;
		newBore.params[1] = 0.0f;
		newBore.params[2] = 0.0f;
		newBore.params[3] = 0.0f;
	}
	else
	{
		// 如果是 Euler 模式 => [Az,El,Roll] = [0,0,0]
		newBore.params[0] = 0.0f;
		newBore.params[1] = 0.0f;
		newBore.params[2] = 0.0f;
		newBore.params[3] = 0.0f;
	}

	// 调用 Polhemus API
	int r = vpcmd_sns_boresight(g_ctx, g_hnd, sensorID, CMD_ACTION_SET, newBore);
	if (r != 0)
	{
		DisplayError(r, "vpcmd_sns_boresight SET boresight");
		return false;
	}

	std::cout << "[INFO] boresightSensor1() done. Now hardware treats orientation as zero.\n"
		<< "       And software offset for position is stored => " << g_posOffset.str() << std::endl;

	return true;
}



bool clearBoresightSensor1()
{
	int sensorID = 0; // 与 boresightSensor1() 中使用的 ID 一致
	CBoresightCfg emptyBore;
	memset(&emptyBore, 0, sizeof(emptyBore));

	// 1) 先清除硬件端 boresight（姿态归零补偿）
	int r = vpcmd_sns_boresight(g_ctx, g_hnd, sensorID, CMD_ACTION_RESET, emptyBore);
	if (r != 0)
	{
		DisplayError(r, "vpcmd_sns_boresight RESET");
		return false;
	}

	// 2) 同时把软件端记录的位移偏移量 g_posOffset 重置为 (0,0,0)
	//    这样后续 (pno.pos - g_posOffset) => (pno.pos - 0) => 原始坐标
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
