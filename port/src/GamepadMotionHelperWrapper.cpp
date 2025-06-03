#include "GamepadMotionHelperWrapper.h"
#include "GamepadMotion.hpp"

struct GamepadMotionOpaque {
    GamepadMotion impl;
};

extern "C" {

GamepadMotionHandle CreateGamepadMotion(void) {
    return new GamepadMotionOpaque();
}

void DeleteGamepadMotion(GamepadMotionHandle handle) {
    delete static_cast<GamepadMotionOpaque*>(handle);
}

void ProcessMotion(GamepadMotionHandle handle, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.ProcessMotion(gx, gy, gz, ax, ay, az, dt);
}

void Reset(GamepadMotionHandle handle) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Reset();
}

void ResetMotion(GamepadMotionHandle handle) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.ResetMotion();
}

void GetCalibratedGyro(GamepadMotionHandle handle, float* x, float* y, float* z) {
    if (handle && x && y && z) static_cast<GamepadMotionOpaque*>(handle)->impl.GetCalibratedGyro(*x, *y, *z);
}

void GetGravity(GamepadMotionHandle handle, float* x, float* y, float* z) {
    if (handle && x && y && z) static_cast<GamepadMotionOpaque*>(handle)->impl.GetGravity(*x, *y, *z);
}

void GetProcessedAcceleration(GamepadMotionHandle handle, float* x, float* y, float* z) {
    if (handle && x && y && z) static_cast<GamepadMotionOpaque*>(handle)->impl.GetProcessedAcceleration(*x, *y, *z);
}

void GetOrientation(GamepadMotionHandle handle, float* w, float* x, float* y, float* z) {
    if (handle && w && x && y && z) static_cast<GamepadMotionOpaque*>(handle)->impl.GetOrientation(*w, *x, *y, *z);
}

void GetPlayerSpaceGyro(GamepadMotionHandle handle, float* x, float* y, float yawRelaxFactor) {
    if (handle && x && y) static_cast<GamepadMotionOpaque*>(handle)->impl.GetPlayerSpaceGyro(*x, *y, yawRelaxFactor);
}

void GetWorldSpaceGyro(GamepadMotionHandle handle, float* x, float* y, float sideReductionThreshold) {
    if (handle && x && y) static_cast<GamepadMotionOpaque*>(handle)->impl.GetWorldSpaceGyro(*x, *y, sideReductionThreshold);
}

void StartContinuousCalibration(GamepadMotionHandle handle) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.StartContinuousCalibration();
}

void PauseContinuousCalibration(GamepadMotionHandle handle) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.PauseContinuousCalibration();
}

void ResetContinuousCalibration(GamepadMotionHandle handle) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.ResetContinuousCalibration();
}

void GetCalibrationOffset(GamepadMotionHandle handle, float* xOffset, float* yOffset, float* zOffset) {
    if (handle && xOffset && yOffset && zOffset) static_cast<GamepadMotionOpaque*>(handle)->impl.GetCalibrationOffset(*xOffset, *yOffset, *zOffset);
}

void SetCalibrationOffset(GamepadMotionHandle handle, float xOffset, float yOffset, float zOffset, int weight) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.SetCalibrationOffset(xOffset, yOffset, zOffset, weight);
}

float GetAutoCalibrationConfidence(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.GetAutoCalibrationConfidence();
    return 0.0f;
}

void SetAutoCalibrationConfidence(GamepadMotionHandle handle, float newConfidence) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.SetAutoCalibrationConfidence(newConfidence);
}

int GetAutoCalibrationIsSteady(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.GetAutoCalibrationIsSteady() ? 1 : 0;
    return 0;
}

int GetCalibrationMode(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.GetCalibrationMode();
    return 0;
}

void SetCalibrationMode(GamepadMotionHandle handle, int mode) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.SetCalibrationMode(static_cast<GamepadMotionHelpers::CalibrationMode>(mode));
}

// --- GamepadMotionSettings: Getters and Setters ---

void SetMinStillnessSamples(GamepadMotionHandle handle, int value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.MinStillnessSamples = value;
}
int GetMinStillnessSamples(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.MinStillnessSamples;
    return 0;
}

void SetMinStillnessCollectionTime(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.MinStillnessCollectionTime = value;
}
float GetMinStillnessCollectionTime(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.MinStillnessCollectionTime;
    return 0.0f;
}

void SetMinStillnessCorrectionTime(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.MinStillnessCorrectionTime = value;
}
float GetMinStillnessCorrectionTime(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.MinStillnessCorrectionTime;
    return 0.0f;
}

void SetMaxStillnessError(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.MaxStillnessError = value;
}
float GetMaxStillnessError(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.MaxStillnessError;
    return 0.0f;
}

void SetStillnessSampleDeteriorationRate(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessSampleDeteriorationRate = value;
}
float GetStillnessSampleDeteriorationRate(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessSampleDeteriorationRate;
    return 0.0f;
}

void SetStillnessErrorClimbRate(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessErrorClimbRate = value;
}
float GetStillnessErrorClimbRate(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessErrorClimbRate;
    return 0.0f;
}

void SetStillnessErrorDropOnRecalibrate(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessErrorDropOnRecalibrate = value;
}
float GetStillnessErrorDropOnRecalibrate(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessErrorDropOnRecalibrate;
    return 0.0f;
}

void SetStillnessCalibrationEaseInTime(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessCalibrationEaseInTime = value;
}
float GetStillnessCalibrationEaseInTime(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessCalibrationEaseInTime;
    return 0.0f;
}

void SetStillnessCalibrationHalfTime(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessCalibrationHalfTime = value;
}
float GetStillnessCalibrationHalfTime(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessCalibrationHalfTime;
    return 0.0f;
}

void SetStillnessConfidenceRate(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessConfidenceRate = value;
}
float GetStillnessConfidenceRate(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessConfidenceRate;
    return 0.0f;
}

void SetStillnessGyroDelta(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessGyroDelta = value;
}
float GetStillnessGyroDelta(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessGyroDelta;
    return 0.0f;
}

void SetStillnessAccelDelta(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessAccelDelta = value;
}
float GetStillnessAccelDelta(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.StillnessAccelDelta;
    return 0.0f;
}

void SetSensorFusionCalibrationSmoothingStrength(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.SensorFusionCalibrationSmoothingStrength = value;
}
float GetSensorFusionCalibrationSmoothingStrength(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.SensorFusionCalibrationSmoothingStrength;
    return 0.0f;
}

void SetSensorFusionAngularAccelerationThreshold(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.SensorFusionAngularAccelerationThreshold = value;
}
float GetSensorFusionAngularAccelerationThreshold(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.SensorFusionAngularAccelerationThreshold;
    return 0.0f;
}

void SetSensorFusionCalibrationEaseInTime(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.SensorFusionCalibrationEaseInTime = value;
}
float GetSensorFusionCalibrationEaseInTime(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.SensorFusionCalibrationEaseInTime;
    return 0.0f;
}

void SetSensorFusionCalibrationHalfTime(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.SensorFusionCalibrationHalfTime = value;
}
float GetSensorFusionCalibrationHalfTime(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.SensorFusionCalibrationHalfTime;
    return 0.0f;
}

void SetSensorFusionConfidenceRate(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.SensorFusionConfidenceRate = value;
}
float GetSensorFusionConfidenceRate(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.SensorFusionConfidenceRate;
    return 0.0f;
}

void SetGravityCorrectionShakinessMaxThreshold(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionShakinessMaxThreshold = value;
}
float GetGravityCorrectionShakinessMaxThreshold(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionShakinessMaxThreshold;
    return 0.0f;
}

void SetGravityCorrectionShakinessMinThreshold(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionShakinessMinThreshold = value;
}
float GetGravityCorrectionShakinessMinThreshold(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionShakinessMinThreshold;
    return 0.0f;
}

void SetGravityCorrectionStillSpeed(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionStillSpeed = value;
}
float GetGravityCorrectionStillSpeed(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionStillSpeed;
    return 0.0f;
}

void SetGravityCorrectionShakySpeed(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionShakySpeed = value;
}
float GetGravityCorrectionShakySpeed(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionShakySpeed;
    return 0.0f;
}

void SetGravityCorrectionGyroFactor(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionGyroFactor = value;
}
float GetGravityCorrectionGyroFactor(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionGyroFactor;
    return 0.0f;
}

void SetGravityCorrectionGyroMinThreshold(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionGyroMinThreshold = value;
}
float GetGravityCorrectionGyroMinThreshold(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionGyroMinThreshold;
    return 0.0f;
}

void SetGravityCorrectionGyroMaxThreshold(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionGyroMaxThreshold = value;
}
float GetGravityCorrectionGyroMaxThreshold(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionGyroMaxThreshold;
    return 0.0f;
}

void SetGravityCorrectionMinimumSpeed(GamepadMotionHandle handle, float value) {
    if (handle) static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionMinimumSpeed = value;
}
float GetGravityCorrectionMinimumSpeed(GamepadMotionHandle handle) {
    if (handle) return static_cast<GamepadMotionOpaque*>(handle)->impl.Settings.GravityCorrectionMinimumSpeed;
    return 0.0f;
}

} // extern "C"
