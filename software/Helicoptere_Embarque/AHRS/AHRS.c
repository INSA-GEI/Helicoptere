#include <AHRS.h>
#include <math.h>

//#define USE_MAG

#define USE_MADGWICK_AHRS
//#define USE_MAHONY_AHRS

#define PI                         (float)     3.14159265f

#ifdef USE_MAHONY_AHRS
#include "MahonyAHRS.h"
#else
#include "MadgwickAHRS.h"
#endif

#define QW q[0]
#define QX q[1]
#define QY q[2]
#define QZ q[3]

void AHRS_GetValues(float * val);
void AHRS_MyFusion(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

float MagBuffer[3] = {0.0f}, AccBuffer[3] = {0.0f}, GyroBuffer[3] = {0.0f};
float QuaternionsBuffer[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float euler[3] = {0.0f};
uint8_t eulerArr[6] = {0};
ahrsStatus_t AHRS_Status=AHRS_STOP;

void AHRS_Init(void)
{
	/* Reset all buffers */
	MagBuffer[0] = 0.0f;
	MagBuffer[1] = 0.0f;
	MagBuffer[2] = 0.0f;

	AccBuffer[0] = 0.0f;
	AccBuffer[1] = 0.0f;
	AccBuffer[2] = 0.0f;

	GyroBuffer[0] = 0.0f;
	GyroBuffer[1] = 0.0f;
	GyroBuffer[2] = 0.0f;

	QuaternionsBuffer[0] = 1.0f;
	QuaternionsBuffer[1] = 0.0f;
	QuaternionsBuffer[2] = 0.0f;
	QuaternionsBuffer[3] = 0.0f;

	euler[0] = 0.0f;
	euler[1] = 0.0f;
	euler[2] = 0.0f;

	eulerArr[0] = 0;
	eulerArr[1] = 0;
	eulerArr[2] = 0;
	eulerArr[3] = 0;
	eulerArr[4] = 0;
	eulerArr[5] = 0;

	AHRS_Status=AHRS_RUN;
}

void AHRS_DeInit(void)
{
	AHRS_Status=AHRS_STOP;
}

//void AHRS_MyFusion(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
//	float recipNorm;
//	q0 = 0;
//	q1 = gx;
//	q2 = gy;
//	q3 = gz;
//
//	// Normalise quaternion
//	recipNorm = 1/sqrt((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
//	q0 *= recipNorm;
//	q1 *= recipNorm;
//	q2 *= recipNorm;
//	q3 *= recipNorm;
//}

void AHRS_UpdateQuaternions(void) {
	float val[9] = {0.0f};
	AHRS_GetValues(val);

#ifdef USE_MADGWICK_AHRS
#ifdef USE_MAG
	MadgwickAHRSupdate(val[0]*PI/180.0, val[1]*PI/180.0, val[2]*PI/180.0, val[3], val[4], val[5], val[6], val[7], val[8]);
#else
	MadgwickAHRSupdate(val[0]*PI/180.0, val[1]*PI/180.0, val[2]*PI/180.0, val[3], val[4], val[5], 0.0f, 0.0f, 0.0f);
	//MadgwickAHRSupdate(0.0f, 0.0f, 0.0f, 100, 100, 1100, 0.0f, 0.0f, 0.0f);
#endif //#ifdef USE_MAG

#elif defined USE_MAHONY_AHRS

#ifdef USE_MAG
	MahonyAHRSupdate(val[0]*PI/180.0, val[1]*PI/180.0, val[2]*PI/180.0, val[3], val[4], val[5], val[6], val[7], val[8]);
#else
	MahonyAHRSupdate(val[0]*PI/180.0, val[1]*PI/180.0, val[2]*PI/180.0, val[3], val[4], val[5], 0.0f, 0.0f, 0.0f);
#endif //#ifdef USE_MAG

#else
	AHRS_MyFusion(val[0]*PI/180.0, val[1]*PI/180.0, val[2]*PI/180.0, val[3], val[4], val[5], val[6], val[7], val[8]);
#endif

	QuaternionsBuffer[0] = q0;
	QuaternionsBuffer[1] = q1;
	QuaternionsBuffer[2] = q2;
	QuaternionsBuffer[3] = q3;
}

void AHRS_UpdateSensors(acceleration_t *acceleration, angularRate_t *angular_rate)
{
	/* Gyro values must be in dps, angular_rate give them in mdps */
	GyroBuffer[0] = angular_rate->x/1000.0;
	GyroBuffer[1] = angular_rate->y/1000.0;
	GyroBuffer[2] = angular_rate->z/1200.0;

	/* Accelero values must be in mg, already in this range for acceleration */
	AccBuffer[0] = acceleration->x;
	AccBuffer[1] = acceleration->y;
	AccBuffer[2] = acceleration->z;

	MagBuffer[0] = 0.0f;
	MagBuffer[1] = 0.0f;
	MagBuffer[2] = 0.0f;
}

void AHRS_GetValues(float * val) {
#if defined OUT_GYRO || defined OUT_ACCEL || defined OUT_MAG
	uint8_t USART_TempBuf[100];
	uint8_t byteCounter = 0;
#endif //#if defined OUT_GYRO || defined OUT_ACCEL || defined OUT_MAG

	/* Gyro values are already compensated (bias) by acc_gyro driver */
	val[0] = -GyroBuffer[1];
	val[1] = GyroBuffer[0];
	val[2] = GyroBuffer[2];

#ifdef OUT_GYRO
	byteCounter += sprintf((char*)(USART_TempBuf + byteCounter), "%f,%f,%f,", val[0], val[1], val[2]);
	//USART_printfWithDMA("%f,%f,%f,", val[0], val[1], val[2]);
#endif //#ifdef OUT_GYRO

	val[3] = AccBuffer[0];
	val[4] = AccBuffer[1];
	val[5] = AccBuffer[2];
#ifdef OUT_ACCEL
	byteCounter += sprintf((char*)(USART_TempBuf + byteCounter), "%f,%f,%f,", val[3], val[4], val[5]);
	//USART_printfWithDMA("%f,%f,%f,", val[3], val[4], val[5]);
#endif //#ifdef OUT_ACCEL

	val[6] = MagBuffer[0];
	val[7] = MagBuffer[1];
	val[8] = MagBuffer[2];
#ifdef OUT_MAG
	byteCounter += sprintf((char*)(USART_TempBuf + byteCounter), "%f,%f,%f,", val[6], val[7], val[8]);
	//USART_printfWithDMA("%f,%f,%f,", val[6], val[7], val[8]);
#endif //#ifdef OUT_MAG

#if defined OUT_GYRO || defined OUT_ACCEL || defined OUT_MAG
	sprintf((char*)(USART_TempBuf + byteCounter), "\r\n");
	USART_printfWithDMA("%s", USART_TempBuf);
	//USART_printfWithDMA("\r\n");
#endif
}

void AHRS_GetQuaternions(float *quaternions) {
	quaternions[0] = QuaternionsBuffer[0];
	quaternions[1] = QuaternionsBuffer[1];
	quaternions[2] = QuaternionsBuffer[2];
	quaternions[3] = QuaternionsBuffer[3];
}

/**
 *
 */
void AHRS_GetEulerAngles(eulerAngles_t* angles) {
	float *q = QuaternionsBuffer;
	float euler[3] = {0.0f};

//	euler[0] = atan2(2*q[1]*q[2]-2*q[0]*q[3], 2*q[0]*q[0]+2*q[1]*q[1]-1)*180/PI; // heading, yaw, phi
//	euler[1] = -asin(2*q[1]*q[3]+2*q[0]*q[2])*180/PI; // attitude, elevation, pitch, theta
//	euler[2] = atan2(2*q[2]*q[3]-2*q[0]*q[1], 2*q[0]*q[0]+2*q[3]*q[3]-1)*180/PI; // bank, roll, psi

	float test = QX*QY+QZ*QW;

//	if (test > 0.499) {
//		euler[0] = 2*atan2(QX, QW)*180/PI;
//		euler[1] = PI*180/(2*PI);
//		euler[2] = 0;
//	} else if (test< -0.499) {
//		euler[0] = -2*atan2(QX, QW)*180/PI;
//		euler[1] = -PI*180/(2*PI);
//		euler[2] = 0;
//	} else {
		euler[0] = atan2(2*QY*QW - 2*QX*QZ, 1 - 2*QY*QY - 2*QZ*QZ)*180/PI;
		euler[1] = asin(2*QX*QY + 2*QZ*QW)*180/PI;
		euler[2] = atan2(2*QX*QW - 2*QY*QZ, 1 - 2*QX*QX - 2*QZ*QZ)*180/PI;
//	}

	angles->yaw = euler[1]*2.0;
	angles->roll = euler[0]*2.0;
	angles->pitch = euler[2]*2.0;
}


void AHRS_GetEulerAsArray(uint8_t *eulerArr) {
	int16_t elrs = 0;

	// Axis X - third Euler
	elrs = (int16_t)(euler[0]);
	eulerArr[0] = (uint8_t)elrs;
	eulerArr[1] = (uint8_t)(elrs>>8);
	// Axis Y - second Euler
	elrs = (int16_t)(euler[1]);
	eulerArr[2] = (uint8_t)elrs;
	eulerArr[3] = (uint8_t)(elrs>>8);
	// Axis Z - first Euler
	elrs = (int16_t)(euler[2]);
	eulerArr[4] = (uint8_t)elrs;
	eulerArr[5] = (uint8_t)(elrs>>8);
}

