#ifndef CONVERT_TO_SENSOR1_FRAME_HPP
#define CONVERT_TO_SENSOR1_FRAME_HPP

#include "chai3d.h"
#include "VPcmdIF.h"
#include "VPtrace.h"

using namespace chai3d;

// Function to convert to sensor 1 frame
PNODATA convertToSensor1Frame(const PNODATA& sensor1, const PNODATA& sensor2);

// Function to convert a quaternion to a rotation matrix
cMatrix3d quaternionToMatrix(const cQuaternion& q);

// Function to compute the transpose of a matrix
cMatrix3d matrixTranspose(const cMatrix3d& M);

// Function to multiply a matrix with a vector
cVector3d matrixVectorMultiply(const cMatrix3d& M, const cVector3d& v);

// Function to compute the conjugate of a quaternion
cQuaternion quaternionConjugate(const cQuaternion& q);

// Function to multiply two quaternions
cQuaternion quaternionMultiply(const cQuaternion& q1, const cQuaternion& q2);

#endif // CONVERT_TO_SENSOR1_FRAME_HPP