#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

// Use the actual C++ class pointer for the handle
typedef struct GamepadMotion GamepadMotion;
typedef GamepadMotion* GamepadMotionHandle;

// Creation and destruction
GamepadMotionHandle gmhCreateGamepadMotion(void);
void gmhDeleteGamepadMotion(GamepadMotionHandle handle);

// Core motion processing
void gmhProcessMotion(GamepadMotionHandle handle, float gyroX, float gyroY, float gyroZ,
                   float accelX, float accelY, float accelZ, float deltaTime);
void gmhResetGamepadMotion(GamepadMotionHandle handle);
void gmhResetMotion(GamepadMotionHandle handle);

// State getters
void gmhGetCalibratedGyro(GamepadMotionHandle handle, float* x, float* y, float* z);
void gmhGetGravity(GamepadMotionHandle handle, float* x, float* y, float* z);
void gmhGetProcessedAcceleration(GamepadMotionHandle handle, float* x, float* y, float* z);
void gmhGetOrientation(GamepadMotionHandle handle, float* w, float* x, float* y, float* z);
void gmhGetPlayerSpaceGyro(GamepadMotionHandle handle, float* x, float* y, float yawRelaxFactor);
void gmhGetWorldSpaceGyro(GamepadMotionHandle handle, float* x, float* y, float sideReductionThreshold);

// Calibration
void gmhStartContinuousCalibration(GamepadMotionHandle handle);
void gmhPauseContinuousCalibration(GamepadMotionHandle handle);
void gmhResetContinuousCalibration(GamepadMotionHandle handle);
void gmhGetCalibrationOffset(GamepadMotionHandle handle, float* xOffset, float* yOffset, float* zOffset);
void gmhSetCalibrationOffset(GamepadMotionHandle handle, float xOffset, float yOffset, float zOffset, int weight);
float gmhGetAutoCalibrationConfidence(GamepadMotionHandle handle);
void gmhSetAutoCalibrationConfidence(GamepadMotionHandle handle, float newConfidence);
bool gmhGetAutoCalibrationIsSteady(GamepadMotionHandle handle);

// Calibration mode
int gmhGetCalibrationMode(GamepadMotionHandle handle);
void gmhSetCalibrationMode(GamepadMotionHandle handle, int mode);

// GamepadMotionSettings: Getters and Setters
void gmhSetMinStillnessSamples(GamepadMotionHandle handle, int value);
int  gmhGetMinStillnessSamples(GamepadMotionHandle handle);
void gmhSetMinStillnessCollectionTime(GamepadMotionHandle handle, float value);
float gmhGetMinStillnessCollectionTime(GamepadMotionHandle handle);
void gmhSetMinStillnessCorrectionTime(GamepadMotionHandle handle, float value);
float gmhGetMinStillnessCorrectionTime(GamepadMotionHandle handle);
void gmhSetMaxStillnessError(GamepadMotionHandle handle, float value);
float gmhGetMaxStillnessError(GamepadMotionHandle handle);
void gmhSetStillnessSampleDeteriorationRate(GamepadMotionHandle handle, float value);
float gmhGetStillnessSampleDeteriorationRate(GamepadMotionHandle handle);
void gmhSetStillnessErrorClimbRate(GamepadMotionHandle handle, float value);
float gmhGetStillnessErrorClimbRate(GamepadMotionHandle handle);
void gmhSetStillnessErrorDropOnRecalibrate(GamepadMotionHandle handle, float value);
float gmhGetStillnessErrorDropOnRecalibrate(GamepadMotionHandle handle);
void gmhSetStillnessCalibrationEaseInTime(GamepadMotionHandle handle, float value);
float gmhGetStillnessCalibrationEaseInTime(GamepadMotionHandle handle);
void gmhSetStillnessCalibrationHalfTime(GamepadMotionHandle handle, float value);
float gmhGetStillnessCalibrationHalfTime(GamepadMotionHandle handle);
void gmhSetStillnessConfidenceRate(GamepadMotionHandle handle, float value);
float gmhGetStillnessConfidenceRate(GamepadMotionHandle handle);
void gmhSetStillnessGyroDelta(GamepadMotionHandle handle, float value);
float gmhGetStillnessGyroDelta(GamepadMotionHandle handle);
void gmhSetStillnessAccelDelta(GamepadMotionHandle handle, float value);
float gmhGetStillnessAccelDelta(GamepadMotionHandle handle);
void gmhSetSensorFusionCalibrationSmoothingStrength(GamepadMotionHandle handle, float value);
float gmhGetSensorFusionCalibrationSmoothingStrength(GamepadMotionHandle handle);
void gmhSetSensorFusionAngularAccelerationThreshold(GamepadMotionHandle handle, float value);
float gmhGetSensorFusionAngularAccelerationThreshold(GamepadMotionHandle handle);
void gmhSetSensorFusionCalibrationEaseInTime(GamepadMotionHandle handle, float value);
float gmhGetSensorFusionCalibrationEaseInTime(GamepadMotionHandle handle);
void gmhSetSensorFusionCalibrationHalfTime(GamepadMotionHandle handle, float value);
float gmhGetSensorFusionCalibrationHalfTime(GamepadMotionHandle handle);
void gmhSetSensorFusionConfidenceRate(GamepadMotionHandle handle, float value);
float gmhGetSensorFusionConfidenceRate(GamepadMotionHandle handle);
void gmhSetGravityCorrectionShakinessMaxThreshold(GamepadMotionHandle handle, float value);
float gmhGetGravityCorrectionShakinessMaxThreshold(GamepadMotionHandle handle);
void gmhSetGravityCorrectionShakinessMinThreshold(GamepadMotionHandle handle, float value);
float gmhGetGravityCorrectionShakinessMinThreshold(GamepadMotionHandle handle);
void gmhSetGravityCorrectionStillSpeed(GamepadMotionHandle handle, float value);
float gmhGetGravityCorrectionStillSpeed(GamepadMotionHandle handle);
void gmhSetGravityCorrectionShakySpeed(GamepadMotionHandle handle, float value);
float gmhGetGravityCorrectionShakySpeed(GamepadMotionHandle handle);
void gmhSetGravityCorrectionGyroFactor(GamepadMotionHandle handle, float value);
float gmhGetGravityCorrectionGyroFactor(GamepadMotionHandle handle);
void gmhSetGravityCorrectionGyroMinThreshold(GamepadMotionHandle handle, float value);
float gmhGetGravityCorrectionGyroMinThreshold(GamepadMotionHandle handle);
void gmhSetGravityCorrectionGyroMaxThreshold(GamepadMotionHandle handle, float value);
float gmhGetGravityCorrectionGyroMaxThreshold(GamepadMotionHandle handle);
void gmhSetGravityCorrectionMinimumSpeed(GamepadMotionHandle handle, float value);
float gmhGetGravityCorrectionMinimumSpeed(GamepadMotionHandle handle);

#ifdef __cplusplus
}
#endif
