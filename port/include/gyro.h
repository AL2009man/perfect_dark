#ifndef _IN_GYRO_H
#define _IN_GYRO_H

#include <PR/ultratypes.h>

#ifdef __cplusplus
extern "C" {
#endif

// Called from inputInitController after SDL sensors are enabled.
// Allocates/configures the GamepadMotion instance for this controller slot.
void gyroInitController(s32 cidx);

// Called from inputCloseController before the SDL controller handle is closed.
// Frees GamepadMotion state and resets all gyro deltas/data for this slot.
void gyroCloseController(s32 cidx);

// Called from the SDL event filter when SDL_SENSOR_GYRO data arrives (rad/s).
void gyroFeedGyroData(s32 cidx, float gx, float gy, float gz);

// Called from the SDL event filter when SDL_SENSOR_ACCEL data arrives (normalised to g).
void gyroFeedAccelData(s32 cidx, float ax, float ay, float az);

// Called once per frame from inputUpdate.
// Runs calibration updates then per-controller gyro processing for all active pads.
void gyroUpdateAll(void);

// Called by inputGyroSetAutoCalibration after changing the stored mode.
// Handles internal state reset and reconfigures the calibration pipeline.
void gyroReconfigureCalibrationMode(s32 cidx);

// Returns the most recent processed gyro orientation (frame-scaled, radians).
void gyroGetOrientation(s32 cidx, f32 *yaw, f32 *pitch, f32 *roll);

#ifdef __cplusplus
}
#endif

#endif /* _IN_GYRO_H */
