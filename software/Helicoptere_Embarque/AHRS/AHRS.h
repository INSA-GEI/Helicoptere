#ifndef SENSORS_H
#define SENSORS_H

#include "main.h"
#include "acc_gyro.h"

//#define OUT_QUATERNION
#define OUT_EULER
//#define OUT_ACCEL
//#define OUT_GYRO
//#define OUT_MAG

typedef struct
{
	float pitch;
	float roll;
	float yaw;
} eulerAngles_t;

typedef enum
{
	AHRS_STOP=0,
	AHRS_RUN=1,
	AHRS_ERROR=2
} ahrsStatus_t;

extern ahrsStatus_t AHRS_Status;

void AHRS_Init(void);
void AHRS_DeInit(void);
void AHRS_UpdateSensors(acceleration_t *acceleration, angularRate_t *angular_rate);

void AHRS_UpdateQuaternions(void);
void AHRS_GetQuaternions(float *quaternions);
void AHRS_GetEulerAngles(eulerAngles_t* angles);
void AHRS_GetEulerAsArray(uint8_t *eulerArr);

//void UpdateGyroBias(void);
//void Demo_GyroConfig(void);
//void Demo_CompassConfig(void);

#endif //#ifndef SENSORS_H
