#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

// Use the actual C++ class pointer for the handle
typedef struct GamepadMotion GamepadMotion;
typedef GamepadMotion* GamepadMotionHandle;

// Creation and destruction
GamepadMotionHandle CreateGamepadMotion(void);
void DeleteGamepadMotion(GamepadMotionHandle handle);

// Core motion processing
void ProcessMotion(GamepadMotionHandle handle, float gyroX, float gyroY, float gyroZ,
                   float accelX, float accelY, float accelZ, float deltaTime);
void ResetGamepadMotion(GamepadMotionHandle handle);
void ResetMotion(GamepadMotionHandle handle);

// State getters
void GetCalibratedGyro(GamepadMotionHandle handle, float* x, float* y, float* z);
void GetGravity(GamepadMotionHandle handle, float* x, float* y, float* z);
void GetProcessedAcceleration(GamepadMotionHandle handle, float* x, float* y, float* z);
void GetOrientation(GamepadMotionHandle handle, float* w, float* x, float* y, float* z);
void GetPlayerSpaceGyro(GamepadMotionHandle handle, float* x, float* y, float yawRelaxFactor);
void GetWorldSpaceGyro(GamepadMotionHandle handle, float* x, float* y, float sideReductionThreshold);

// Calibration
void StartContinuousCalibration(GamepadMotionHandle handle);
void PauseContinuousCalibration(GamepadMotionHandle handle);
void ResetContinuousCalibration(GamepadMotionHandle handle);
void GetCalibrationOffset(GamepadMotionHandle handle, float* xOffset, float* yOffset, float* zOffset);
void SetCalibrationOffset(GamepadMotionHandle handle, float xOffset, float yOffset, float zOffset, int weight);
float GetAutoCalibrationConfidence(GamepadMotionHandle handle);
void SetAutoCalibrationConfidence(GamepadMotionHandle handle, float newConfidence);
bool GetAutoCalibrationIsSteady(GamepadMotionHandle handle);

// Calibration mode
int GetCalibrationMode(GamepadMotionHandle handle);
void SetCalibrationMode(GamepadMotionHandle handle, int mode);

// GamepadMotionSettings: Getters and Setters
void SetMinStillnessSamples(GamepadMotionHandle handle, int value);
int  GetMinStillnessSamples(GamepadMotionHandle handle);
void SetMinStillnessCollectionTime(GamepadMotionHandle handle, float value);
float GetMinStillnessCollectionTime(GamepadMotionHandle handle);
void SetMinStillnessCorrectionTime(GamepadMotionHandle handle, float value);
float GetMinStillnessCorrectionTime(GamepadMotionHandle handle);
void SetMaxStillnessError(GamepadMotionHandle handle, float value);
float GetMaxStillnessError(GamepadMotionHandle handle);
void SetStillnessSampleDeteriorationRate(GamepadMotionHandle handle, float value);
float GetStillnessSampleDeteriorationRate(GamepadMotionHandle handle);
void SetStillnessErrorClimbRate(GamepadMotionHandle handle, float value);
float GetStillnessErrorClimbRate(GamepadMotionHandle handle);
void SetStillnessErrorDropOnRecalibrate(GamepadMotionHandle handle, float value);
float GetStillnessErrorDropOnRecalibrate(GamepadMotionHandle handle);
void SetStillnessCalibrationEaseInTime(GamepadMotionHandle handle, float value);
float GetStillnessCalibrationEaseInTime(GamepadMotionHandle handle);
void SetStillnessCalibrationHalfTime(GamepadMotionHandle handle, float value);
float GetStillnessCalibrationHalfTime(GamepadMotionHandle handle);
void SetStillnessConfidenceRate(GamepadMotionHandle handle, float value);
float GetStillnessConfidenceRate(GamepadMotionHandle handle);
void SetStillnessGyroDelta(GamepadMotionHandle handle, float value);
float GetStillnessGyroDelta(GamepadMotionHandle handle);
void SetStillnessAccelDelta(GamepadMotionHandle handle, float value);
float GetStillnessAccelDelta(GamepadMotionHandle handle);
void SetSensorFusionCalibrationSmoothingStrength(GamepadMotionHandle handle, float value);
float GetSensorFusionCalibrationSmoothingStrength(GamepadMotionHandle handle);
void SetSensorFusionAngularAccelerationThreshold(GamepadMotionHandle handle, float value);
float GetSensorFusionAngularAccelerationThreshold(GamepadMotionHandle handle);
void SetSensorFusionCalibrationEaseInTime(GamepadMotionHandle handle, float value);
float GetSensorFusionCalibrationEaseInTime(GamepadMotionHandle handle);
void SetSensorFusionCalibrationHalfTime(GamepadMotionHandle handle, float value);
float GetSensorFusionCalibrationHalfTime(GamepadMotionHandle handle);
void SetSensorFusionConfidenceRate(GamepadMotionHandle handle, float value);
float GetSensorFusionConfidenceRate(GamepadMotionHandle handle);
void SetGravityCorrectionShakinessMaxThreshold(GamepadMotionHandle handle, float value);
float GetGravityCorrectionShakinessMaxThreshold(GamepadMotionHandle handle);
void SetGravityCorrectionShakinessMinThreshold(GamepadMotionHandle handle, float value);
float GetGravityCorrectionShakinessMinThreshold(GamepadMotionHandle handle);
void SetGravityCorrectionStillSpeed(GamepadMotionHandle handle, float value);
float GetGravityCorrectionStillSpeed(GamepadMotionHandle handle);
void SetGravityCorrectionShakySpeed(GamepadMotionHandle handle, float value);
float GetGravityCorrectionShakySpeed(GamepadMotionHandle handle);
void SetGravityCorrectionGyroFactor(GamepadMotionHandle handle, float value);
float GetGravityCorrectionGyroFactor(GamepadMotionHandle handle);
void SetGravityCorrectionGyroMinThreshold(GamepadMotionHandle handle, float value);
float GetGravityCorrectionGyroMinThreshold(GamepadMotionHandle handle);
void SetGravityCorrectionGyroMaxThreshold(GamepadMotionHandle handle, float value);
float GetGravityCorrectionGyroMaxThreshold(GamepadMotionHandle handle);
void SetGravityCorrectionMinimumSpeed(GamepadMotionHandle handle, float value);
float GetGravityCorrectionMinimumSpeed(GamepadMotionHandle handle);

#ifdef __cplusplus
}
#endif
