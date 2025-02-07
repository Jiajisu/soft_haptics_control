# CHAI3D & Polhemus Viper & Soft Pneumatic Fingertip Haptic Gripper Integration

This project demonstrates how to integrate Polhemus Viper tracking data into a CHAI3D scene and interact with soft haptic gripper. It includes:

1.

2.

3.

4.

5.

Key Files:

1.soft_gripper.cpp: Contains the CHAI3D setup, rendering loop, and Polhemus data handling.

2.SerialPort.cpp/.hpp: Communicates with Arduino (if needed).

3.frameTrans.hpp: Provides helper functions like convertToSensor1Frame.

4.VPcmdIF.h, VPtrace.h: Polhemus Viper SDK headers for command and tracking data.

5.

How It Works:
 
1.Initialize the Polhemus device and CHAI3D world.

2.

Building & Running

1.Clone the repository and open it in your preferred IDE or command line environment.

2.Ensure you have the Polhemus Viper SDK headers and libraries plus CHAI3D installed.

