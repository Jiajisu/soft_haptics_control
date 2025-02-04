#include "chai3d.h"

#include "VPcmdIF.h"
#include "VPtrace.h"

using namespace chai3d;

PNODATA convertToSensor1Frame(const PNODATA& sensor1, const PNODATA& sensor2);
cMatrix3d quaternionToMatrix(const cQuaternion& q);
cVector3d matrixVectorMultiply(const cMatrix3d& R, const cVector3d& v);
cQuaternion quaternionConjugate(const cQuaternion& q);
cQuaternion quaternionNormalize(const cQuaternion& q);
cQuaternion quaternionMultiply(const cQuaternion& q1, const cQuaternion& q2);
cMatrix3d matrixTranspose(const cMatrix3d& M);


// convert to sensor 1 frame
PNODATA convertToSensor1Frame(const PNODATA& sensor1, const PNODATA& sensor2) {
	PNODATA result;

	// -------------------------------
	// grab data from viper and calculate
	// -------------------------------
	cVector3d pos1(sensor1.pos[0], sensor1.pos[1], sensor1.pos[2]);
	cVector3d pos2(sensor2.pos[0], sensor2.pos[1], sensor2.pos[2]);

	cQuaternion q1(sensor1.ori[0], sensor1.ori[1], sensor1.ori[2], sensor1.ori[3]);
	cQuaternion q2(sensor2.ori[0], sensor2.ori[1], sensor2.ori[2], sensor2.ori[3]);




	cMatrix3d q1_rot = quaternionToMatrix(q1);
	cMatrix3d q1_rot_inv = matrixTranspose(q1_rot);

	cVector3d pos_diff = pos2 - pos1;
	cVector3d pos_rel;
	pos_rel = matrixVectorMultiply(q1_rot_inv, pos_diff);


	cQuaternion q1_inv = quaternionConjugate(q1);
	cQuaternion q_rel = quaternionMultiply(q1_inv, q2);


	// -------------------------------
	// save data
	// -------------------------------
	result.pos[0] = pos_rel.x();
	result.pos[1] = pos_rel.y();
	result.pos[2] = pos_rel.z();

	result.ori[0] = q_rel.w;
	result.ori[1] = q_rel.x;
	result.ori[2] = q_rel.y;
	result.ori[3] = q_rel.z;

	return result;
}

// quaternion to Matrix
cMatrix3d quaternionToMatrix(const cQuaternion& q) {
	double w = q.w, x = q.x, y = q.y, z = q.z;

	cMatrix3d R;
	R.set(
		1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
		2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
		2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)
	);

	return R;
}

// matrix transpose
cMatrix3d matrixTranspose(const cMatrix3d& M) {
	return cMatrix3d(
		M(0, 0), M(1, 0), M(2, 0),
		M(0, 1), M(1, 1), M(2, 1),
		M(0, 2), M(1, 2), M(2, 2)
	);
}

// matrix multiply vector
cVector3d matrixVectorMultiply(const cMatrix3d& M, const cVector3d& v) {
	return cVector3d(
		M(0, 0) * v.x() + M(0, 1) * v.y() + M(0, 2) * v.z(),
		M(1, 0) * v.x() + M(1, 1) * v.y() + M(1, 2) * v.z(),
		M(2, 0) * v.x() + M(2, 1) * v.y() + M(2, 2) * v.z()
	);
}


// calculate conjugate
cQuaternion quaternionConjugate(const cQuaternion & q) {
	return cQuaternion(q.w, -q.x, -q.y, -q.z);
}



// quaternion_1 multiply quaternion_2
cQuaternion quaternionMultiply(const cQuaternion & q1, const cQuaternion & q2) {
	return cQuaternion(
		q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
		q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
		q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
		q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
	);
}