// gyro.c — gyro processing, calibration, and output.
// Owns: GamepadMotion instances, raw sensor data, delta output, GyroCalibState.
// input.c owns: padsCfg (gyro config fields), SDL controller lifecycle.

#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <SDL.h>
#include <PR/ultratypes.h>
#include <PR/os_thread.h>
#include "platform.h"
#include "input.h"
#include "gyro.h"
#include "../gamepadmotionhelper/GamepadMotion.h"
#include "system.h"
#include "game/menu.h"
#include "bss.h"

#define GMH_STILLNESS_COLLECTION_TIME 2.5f
#define GMH_STILLNESS_CORRECTION_TIME 3.0f
#define GMH_STILLNESS_EASE_TIME       1.0f
#define GMH_MAX_STILLNESS_ERROR       1.5f

// Noise filter thresholds — compared as squared magnitudes (values in (rad/s)^2)
#define GYRO_NOISE_THRESHOLD  0.16f  // gyro magnitude^2 (~0.4 rad/s ≈ 23 deg/s)
#define GYRO_RATE_THRESHOLD   0.3f   // frame-to-frame gyro delta^2 (~0.55 rad/s)
#define ACCEL_GRAVITY_TOLERANCE 0.02f
#define ACCEL_DELTA_THRESHOLD   0.01f

typedef struct {
	// Manual calibration
	bool     manualCalibActive;
	Uint32   manualCalibStartTime;
	f32      manualOffsetX, manualOffsetY, manualOffsetZ;
	s32      manualWeight;
	bool     prevManualCalibPressed;

	// Auto-calibration
	bool     wasStill;
	Uint32   lastAutoCalibTime;
	Uint32   autoCalibStartTime;
	bool     isCalibrating;
	bool     justFinishedCalibrating;
	s32      prevAutoCalibMode;     // -1 = uninitialised
	bool     alwaysModeActive;
	bool     stationaryModeActive;

	// Axis mode change detection
	s32      previousAxisMode;      // -1 = uninitialised
	bool     axisModeInitialized;

	// Modifier toggle
	bool     gyroToggleState;       // true = gyro on
	s32      prevGyroModPressed;

	// Smoothing EMA
	f32      emaX, emaY, emaZ;

	// Noise filter previous-frame snapshots
	f32      prevGyroSnapshot[3];
	f32      prevAccelSnapshot[3];
	bool     noiseFilterInitialized;
} GyroCalibState;

static GamepadMotionHandle gpadMotion[INPUT_MAX_CONTROLLERS] = { NULL };
static f32  gyroOrientYaw[INPUT_MAX_CONTROLLERS];
static f32  gyroOrientPitch[INPUT_MAX_CONTROLLERS];
static f32  gyroOrientRoll[INPUT_MAX_CONTROLLERS];
static float gyroData[INPUT_MAX_CONTROLLERS][3]  = {0};
static float accelData[INPUT_MAX_CONTROLLERS][3] = {0};
static bool  hasSensorData[INPUT_MAX_CONTROLLERS] = {false};
static GyroCalibState gyroCalibState[INPUT_MAX_CONTROLLERS] = {0};

static void applyGyroAxisMapping(s32 cidx, float calibratedGyro[3], f32 *deltaX, f32 *deltaY, f32 *deltaZ);
static void applyGyroModifier(f32 *deltaX, f32 *deltaY, f32 *deltaZ, s32 activationMode, s32 idx);

static void applyGyroTightening(f32 *dx, f32 *dy, f32 *dz, f32 tightening);
static void applyGyroDeadzone(f32 *dx, f32 *dy, f32 *dz, f32 deadzone);
static void applyGyroSmoothing(f32 *deltaX, f32 *deltaY, f32 *deltaZ, f32 smoothing, s32 cidx);
static void inputConfigureGamepadMotionSettings(GamepadMotionHandle handle);
static void inputUpdateGyroCalibrationHandle(void);
static void inputUpdateGyroAutoCalibration(s32 cidx);
static void inputUpdateGyroManualCalibration(s32 cidx);
static void inputGyroManualCalibrationActivation(s32 cidx, bool start);
static void inputGyroCalibrationFinished(s32 cidx, bool finished);
static bool inputIsGyroCalibrationBlocked(s32 cidx);
static void inputConfigureGyroCalibrationMode(s32 cidx);
static void inputResetGyroCalibration(s32 cidx);
static void inputApplyGyroManualCalibrationOffset(s32 cidx);
static void inputSaveGyroCalibrationOffset(s32 cidx);
static bool inputIsControllerSensorNoiseThreshold(s32 cidx);
static bool inputGyroAutoCalibrationModes(s32 cidx);

static void inputInitGyroCalibState(s32 cidx)
{
	memset(&gyroCalibState[cidx], 0, sizeof(GyroCalibState));
	gyroCalibState[cidx].prevAutoCalibMode = -1;
	gyroCalibState[cidx].previousAxisMode  = -1;
	gyroCalibState[cidx].gyroToggleState   = true;
}

void gyroInitController(s32 cidx)
{
	if (!gpadMotion[cidx]) {
		gpadMotion[cidx] = gmhCreateGamepadMotion();
	}
	if (gpadMotion[cidx]) {
		inputInitGyroCalibState(cidx);
		inputConfigureGamepadMotionSettings(gpadMotion[cidx]);
		gmhResetMotion(gpadMotion[cidx]);
		inputConfigureGyroCalibrationMode(cidx);
	}
}

void gyroCloseController(s32 cidx)
{
	if (gpadMotion[cidx]) {
		gmhDeleteGamepadMotion(gpadMotion[cidx]);
		gpadMotion[cidx] = NULL;
	}
	gyroOrientYaw[cidx]   = 0.f;
	gyroOrientPitch[cidx] = 0.f;
	gyroOrientRoll[cidx]  = 0.f;
	gyroData[cidx][0]  = gyroData[cidx][1]  = gyroData[cidx][2]  = 0.f;
	accelData[cidx][0] = accelData[cidx][1] = accelData[cidx][2] = 0.f;
	hasSensorData[cidx] = false;
	inputInitGyroCalibState(cidx);
}

void gyroFeedGyroData(s32 cidx, float gx, float gy, float gz)
{
	gyroData[cidx][0] = gx;
	gyroData[cidx][1] = gy;
	gyroData[cidx][2] = gz;
	hasSensorData[cidx] = true;
}

void gyroFeedAccelData(s32 cidx, float ax, float ay, float az)
{
	accelData[cidx][0] = ax;
	accelData[cidx][1] = ay;
	accelData[cidx][2] = az;
	hasSensorData[cidx] = true;
}

static void inputConfigureGamepadMotionSettings(GamepadMotionHandle handle)
{
	if (!handle) return;
	gmhSetMinStillnessCollectionTime(handle, GMH_STILLNESS_COLLECTION_TIME);
	gmhSetMinStillnessCorrectionTime(handle, GMH_STILLNESS_CORRECTION_TIME);
	gmhSetStillnessGyroDelta(handle, -1.f);
	gmhSetStillnessAccelDelta(handle, -1.f);
	gmhSetStillnessCalibrationEaseInTime(handle, GMH_STILLNESS_EASE_TIME);
	gmhSetMaxStillnessError(handle, GMH_MAX_STILLNESS_ERROR);
}

static void applyGyroAxisMapping(s32 cidx, float calibratedGyro[3], f32 *deltaX, f32 *deltaY, f32 *deltaZ)
{
	if (!gpadMotion[cidx]) {
		*deltaX = *deltaY = *deltaZ = 0.f;
		return;
	}

	GyroCalibState *state = &gyroCalibState[cidx];
	const s32 currentAxisMode = inputGyroGetAxisMode(cidx);

	if (!state->axisModeInitialized) {
		state->previousAxisMode    = currentAxisMode;
		state->axisModeInitialized = true;
	}

	if (state->previousAxisMode != currentAxisMode) {
		gmhResetMotion(gpadMotion[cidx]);
		state->previousAxisMode = currentAxisMode;
	}

	switch (currentAxisMode) {
	case GYRO_YAW:
		*deltaX = -calibratedGyro[1];
		*deltaY = -calibratedGyro[0];
		break;
	case GYRO_ROLL:
		*deltaX =  calibratedGyro[2];
		*deltaY = -calibratedGyro[0];
		break;
	case GYRO_LOCAL:
		*deltaX = -calibratedGyro[1] + (calibratedGyro[2] * 0.85f);
		*deltaY = -calibratedGyro[0];
		*deltaZ =  calibratedGyro[2];
		break;
	case GYRO_PLAYER: {
		float playerX = 0.f, playerY = 0.f;
		gmhGetPlayerSpaceGyro(gpadMotion[cidx], &playerX, &playerY, 1.41f);
		*deltaX = -playerY;
		*deltaY = -playerX;
		*deltaZ =  calibratedGyro[2];
		break;
	}
	case GYRO_WORLD: {
		float worldX = 0.f, worldY = 0.f;
		gmhGetWorldSpaceGyro(gpadMotion[cidx], &worldX, &worldY, 0.125f);
		*deltaX = -worldY;
		*deltaY = -worldX;
		*deltaZ =  0.f;
		break;
	}
	default:
		*deltaX = *deltaY = *deltaZ = 0.f;
		break;
	}
}

static void applyGyroModifier(f32 *deltaX, f32 *deltaY, f32 *deltaZ, s32 activationMode, s32 idx)
{
	GyroCalibState *state = &gyroCalibState[idx];

	const int modPressed  = inputBindPressed(idx, CK_0080);
	const int justPressed = modPressed && !state->prevGyroModPressed;
	state->prevGyroModPressed = modPressed;

	bool gyroActive = false;

	switch (activationMode) {
	case GYRO_ALWAYS_ON:
		gyroActive = true;
		break;
	case GYRO_TOGGLE:
		if (justPressed) {
			state->gyroToggleState = !state->gyroToggleState;
		}
		gyroActive = state->gyroToggleState;
		break;
	case GYRO_ENABLE_HELD:
		gyroActive = modPressed;
		break;
	case GYRO_DISABLE_HELD:
		gyroActive = !modPressed;
		break;
	default:
		gyroActive = false;
		break;
	}

	if (!gyroActive) {
		*deltaX = 0.f;
		*deltaY = 0.f;
		*deltaZ = 0.f;
	}
}

static void applyGyroSmoothing(f32 *deltaX, f32 *deltaY, f32 *deltaZ, f32 smoothing, s32 cidx)
{
	if (!deltaX || !deltaY || !deltaZ || smoothing <= 0.0f) return;

	const f32 alpha = 1.0f - smoothing;
	GyroCalibState *state = &gyroCalibState[cidx];

	state->emaX = alpha * (*deltaX) + smoothing * state->emaX;
	state->emaY = alpha * (*deltaY) + smoothing * state->emaY;
	state->emaZ = alpha * (*deltaZ) + smoothing * state->emaZ;

	*deltaX = state->emaX;
	*deltaY = state->emaY;
	*deltaZ = state->emaZ;
}

static void applyGyroTightening(f32 *dx, f32 *dy, f32 *dz, f32 tightening)
{
	if (!dx || !dy || !dz || tightening <= 0.0f) return;

	f32 mag = sqrtf((*dx) * (*dx) + (*dy) * (*dy) + (*dz) * (*dz));
	if (mag > 0.f) {
		f32 scale = 1.0f;
		// Soft tiered scaling — http://gyrowiki.jibbsmart.com/blog:tight-and-smooth:soft-tiered-smoothing
		if (mag < tightening) {
			f32 ratio = mag / tightening;
			if (ratio < 0.25f) {
				scale = ratio * ratio * 4.0f;
			} else if (ratio < 0.5f) {
				f32 r = (ratio - 0.25f) / 0.25f;
				scale = 0.25f + r * r * 0.5f;
			} else if (ratio < 0.75f) {
				f32 r = (ratio - 0.5f) / 0.25f;
				scale = 0.75f + r * 0.2f;
			} else {
				f32 r = (ratio - 0.75f) / 0.25f;
				scale = 0.95f + r * r * 0.05f;
			}
		}
		*dx *= scale;
		*dy *= scale;
		*dz *= scale;
	}
}


static void inputProcessMotionSensorData(s32 cidx, float deltaTime, f32 *deltaX, f32 *deltaY, f32 *deltaZ)
{
	if (!gpadMotion[cidx]) {
		sysLogPrintf(LOG_WARNING, "GamepadMotion instance missing for controller %d, gyro will not function", cidx);
		return;
	}

	if (!hasSensorData[cidx]) {
		return;
	}

	gmhProcessMotion(gpadMotion[cidx],
		gyroData[cidx][0],  gyroData[cidx][1],  gyroData[cidx][2],
		accelData[cidx][0], accelData[cidx][1], accelData[cidx][2],
		deltaTime);

	float calibratedGyro[3] = {0.f};
	gmhGetCalibratedGyro(gpadMotion[cidx], &calibratedGyro[0], &calibratedGyro[1], &calibratedGyro[2]);
	applyGyroAxisMapping(cidx, calibratedGyro, deltaX, deltaY, deltaZ);
}

static void inputApplyGyroProcessing(s32 cidx, f32 *deltaX, f32 *deltaY, f32 *deltaZ)
{
	if (gyroCalibState[cidx].justFinishedCalibrating) {
		*deltaX = 0.f;
		*deltaY = 0.f;
		*deltaZ = 0.f;
		gyroCalibState[cidx].justFinishedCalibrating = false;
		return;
	}

	applyGyroModifier(deltaX, deltaY, deltaZ, inputGetGyroModifier(cidx), cidx);
	applyGyroSmoothing(deltaX, deltaY, deltaZ, inputGetGyroSmoothing(cidx), cidx);
	applyGyroTightening(deltaX, deltaY, deltaZ, inputGyroGetTightening(cidx));
	applyGyroDeadzone(deltaX, deltaY, deltaZ, inputGyroGetDeadzone(cidx));
}

static void applyGyroDeadzone(f32 *dx, f32 *dy, f32 *dz, f32 deadzone)
{
	if (deadzone <= 0.f || !dx || !dy || !dz) return;

	f32 mag = sqrtf((*dx) * (*dx) + (*dy) * (*dy) + (*dz) * (*dz));
	if (mag > 0.f) {
		f32 scale = 1.0f;
		if (mag <= deadzone) {
			scale = 0.0f;
		} else {
			f32 exitZone      = deadzone * 0.2f;
			f32 exitThreshold = deadzone + exitZone;
			if (mag < exitThreshold) {
				f32 exitRatio = (mag - deadzone) / exitZone;
				scale = exitRatio * exitRatio;
			}
		}
		*dx *= scale;
		*dy *= scale;
		*dz *= scale;
	}
}

void inputUpdateGyro(s32 cidx)
{
	float deltaTime = g_Vars.diffframe60f / 60.0f;
	float frameTime = g_Vars.lvupdate60freal;

	f32 deltaX = 0.f, deltaY = 0.f, deltaZ = 0.f;
	inputProcessMotionSensorData(cidx, deltaTime, &deltaX, &deltaY, &deltaZ);
	inputApplyGyroProcessing(cidx, &deltaX, &deltaY, &deltaZ);

	gyroOrientYaw[cidx]   = deltaX * frameTime;
	gyroOrientPitch[cidx] = deltaY * frameTime;
	gyroOrientRoll[cidx]  = deltaZ * frameTime;
}

static bool inputIsControllerSensorNoiseThreshold(s32 cidx)
{
	if (!gpadMotion[cidx] || !hasSensorData[cidx]) {
		return false;
	}

	GyroCalibState *state = &gyroCalibState[cidx];

	#define VEC3_LENGTH_SQ(v) ((v)[0]*(v)[0] + (v)[1]*(v)[1] + (v)[2]*(v)[2])

	#define UPDATE_PREV_STATE() \
		for (s32 j = 0; j < 3; ++j) { \
			state->prevGyroSnapshot[j]  = gyroData[cidx][j]; \
			state->prevAccelSnapshot[j] = accelData[cidx][j]; \
		} \
		state->noiseFilterInitialized = true

	if (VEC3_LENGTH_SQ(gyroData[cidx]) > GYRO_NOISE_THRESHOLD) {
		UPDATE_PREV_STATE();
		return false;
	}

	if (state->noiseFilterInitialized) {
		float gyroDelta[3] = {
			gyroData[cidx][0] - state->prevGyroSnapshot[0],
			gyroData[cidx][1] - state->prevGyroSnapshot[1],
			gyroData[cidx][2] - state->prevGyroSnapshot[2]
		};
		if (VEC3_LENGTH_SQ(gyroDelta) > GYRO_RATE_THRESHOLD) {
			UPDATE_PREV_STATE();
			return false;
		}

		float accelDelta[3] = {
			accelData[cidx][0] - state->prevAccelSnapshot[0],
			accelData[cidx][1] - state->prevAccelSnapshot[1],
			accelData[cidx][2] - state->prevAccelSnapshot[2]
		};
		if (VEC3_LENGTH_SQ(accelDelta) > ACCEL_DELTA_THRESHOLD) {
			UPDATE_PREV_STATE();
			return false;
		}
	}

	UPDATE_PREV_STATE();

	const float accelMagnitude = sqrtf(VEC3_LENGTH_SQ(accelData[cidx]));

	#undef VEC3_LENGTH_SQ
	#undef UPDATE_PREV_STATE

	return fabsf(accelMagnitude - 1.0f) < ACCEL_GRAVITY_TOLERANCE;
}

static bool inputGyroAutoCalibrationModes(s32 cidx)
{
	switch (inputGyroGetAutoCalibration(cidx)) {
	case GYRO_AUTOCALIBRATION_OFF:
		return false;
	case GYRO_AUTOCALIBRATION_MENU_ONLY:
		return g_Menus[cidx].curdialog != NULL;
	case GYRO_AUTOCALIBRATION_STATIONARY:
	case GYRO_AUTOCALIBRATION_ALWAYS:
		return true;
	default:
		return false;
	}
}

static void inputGyroCalibrationFinished(s32 cidx, bool finished)
{
	GyroCalibState *state = &gyroCalibState[cidx];
	state->justFinishedCalibrating = finished;
	if (finished) {
		inputSaveGyroCalibrationOffset(cidx);
	}
}

static void inputSaveGyroCalibrationOffset(s32 cidx)
{
	if (!gpadMotion[cidx]) return;

	const s32 mode = inputGyroGetAutoCalibration(cidx);
	GyroCalibState *state = &gyroCalibState[cidx];

	if (mode == GYRO_AUTOCALIBRATION_ALWAYS) {
		state->manualOffsetX = state->manualOffsetY = state->manualOffsetZ = 0.0f;
		state->manualWeight  = 0;
		return;
	}

	if (mode == GYRO_AUTOCALIBRATION_OFF || mode == GYRO_AUTOCALIBRATION_MENU_ONLY) {
		gmhGetCalibrationOffset(gpadMotion[cidx],
			&state->manualOffsetX, &state->manualOffsetY, &state->manualOffsetZ);
		state->manualWeight = 100;
	}
}

static void inputApplyGyroManualCalibrationOffset(s32 cidx)
{
	if (cidx < 0 || cidx >= INPUT_MAX_CONTROLLERS || !gpadMotion[cidx]) return;

	const s32 mode = inputGyroGetAutoCalibration(cidx);
	GyroCalibState *state = &gyroCalibState[cidx];

	const bool useManualOffset =
		(mode == GYRO_AUTOCALIBRATION_OFF || mode == GYRO_AUTOCALIBRATION_MENU_ONLY);

	if (useManualOffset && state->manualWeight > 0) {
		gmhSetCalibrationOffset(gpadMotion[cidx],
			state->manualOffsetX, state->manualOffsetY, state->manualOffsetZ,
			state->manualWeight);
	}
}

static void inputResetGyroCalibration(s32 cidx)
{
	if (!gpadMotion[cidx]) return;

	gmhResetGamepadMotion(gpadMotion[cidx]);
	inputConfigureGamepadMotionSettings(gpadMotion[cidx]);

	gyroOrientYaw[cidx]   = 0.f;
	gyroOrientPitch[cidx] = 0.f;
	gyroOrientRoll[cidx]  = 0.f;
	inputInitGyroCalibState(cidx);
	inputConfigureGyroCalibrationMode(cidx);
}

static void inputConfigureGyroCalibrationMode(s32 cidx)
{
	if (!gpadMotion[cidx]) return;

	const s32 mode = inputGyroGetAutoCalibration(cidx);

	gmhSetCalibrationMode(gpadMotion[cidx], CALIBRATIONMODE_STILLNESS | CALIBRATIONMODE_SENSORFUSION);

	if (mode == GYRO_AUTOCALIBRATION_ALWAYS) {
		if (!gyroCalibState[cidx].justFinishedCalibrating) {
			gmhResetContinuousCalibration(gpadMotion[cidx]);
		}
	} else {
		gmhPauseContinuousCalibration(gpadMotion[cidx]);
		gyroCalibState[cidx].justFinishedCalibrating = false;
		if (mode == GYRO_AUTOCALIBRATION_MENU_ONLY || mode == GYRO_AUTOCALIBRATION_OFF) {
			inputApplyGyroManualCalibrationOffset(cidx);
		}
	}
}

static bool inputIsGyroCalibrationBlocked(s32 cidx)
{
	return inputIsMenuGyroCalibrationActive(cidx);
}

static void inputGyroManualCalibrationActivation(s32 cidx, bool start)
{
	GyroCalibState *state = &gyroCalibState[cidx];
	if (!gpadMotion[cidx]) return;

	if (start) {
		state->manualCalibActive    = true;
		state->manualCalibStartTime = SDL_GetTicks();
		gmhSetCalibrationMode(gpadMotion[cidx], CALIBRATIONMODE_MANUAL);
		gmhResetContinuousCalibration(gpadMotion[cidx]);
		gmhStartContinuousCalibration(gpadMotion[cidx]);
		state->justFinishedCalibrating = false;
	} else {
		state->manualCalibActive = false;
		gmhPauseContinuousCalibration(gpadMotion[cidx]);
		inputSaveGyroCalibrationOffset(cidx);
		if (inputGyroGetAutoCalibration(cidx) != GYRO_AUTOCALIBRATION_ALWAYS) {
			inputConfigureGyroCalibrationMode(cidx);
		}
	}
}

static void inputUpdateGyroManualCalibration(s32 cidx)
{
	if (!inputControllerConnected(cidx) || !gpadMotion[cidx]) {
		gyroCalibState[cidx].manualCalibActive = false;
		return;
	}

	GyroCalibState *state = &gyroCalibState[cidx];

	if (inputIsGyroCalibrationBlocked(cidx)) {
		if (state->manualCalibActive) {
			inputGyroManualCalibrationActivation(cidx, false);
		}
		return;
	}

	const bool pressed     = inputBindPressed(cidx, CK_0100);
	const bool justPressed = pressed && !state->prevManualCalibPressed;

	if (justPressed && !state->manualCalibActive) {
		inputGyroManualCalibrationActivation(cidx, true);
	}

	if (state->manualCalibActive) {
		const Uint32 elapsed = SDL_GetTicks() - state->manualCalibStartTime;
		if (elapsed >= 500) {
			inputGyroManualCalibrationActivation(cidx, false);
		}
	}

	state->prevManualCalibPressed = pressed;
}

static void inputUpdateGyroAutoCalibration(s32 cidx)
{
	if (!gpadMotion[cidx] || inputGyroGetAutoCalibration(cidx) == GYRO_AUTOCALIBRATION_OFF) {
		return;
	}

	if (gyroCalibState[cidx].manualCalibActive && !inputIsGyroCalibrationBlocked(cidx)) {
		return;
	}

	if (inputIsMenuGyroCalibrationActive(cidx)) {
		gmhPauseContinuousCalibration(gpadMotion[cidx]);
		return;
	}

	GyroCalibState *state  = &gyroCalibState[cidx];
	const s32 mode         = inputGyroGetAutoCalibration(cidx);
	const Uint32 now       = SDL_GetTicks();
	const float confidence = gmhGetAutoCalibrationConfidence(gpadMotion[cidx]);
	const bool stillness   = gmhGetAutoCalibrationIsSteady(gpadMotion[cidx]);

	if (!inputGyroAutoCalibrationModes(cidx)) {
		gmhPauseContinuousCalibration(gpadMotion[cidx]);
		if (mode == GYRO_AUTOCALIBRATION_MENU_ONLY) {
			if (state->isCalibrating) {
				gmhResetContinuousCalibration(gpadMotion[cidx]);
				state->isCalibrating    = false;
				state->autoCalibStartTime = 0;
			}
			state->stationaryModeActive = false;
		}
		return;
	}

	if (state->prevAutoCalibMode != mode) {
		state->prevAutoCalibMode    = mode;
		gmhPauseContinuousCalibration(gpadMotion[cidx]);
		state->alwaysModeActive     = false;
		state->stationaryModeActive = false;
		state->isCalibrating        = false;
		state->wasStill             = false;
		state->lastAutoCalibTime    = 0;
		state->autoCalibStartTime   = 0;
		state->justFinishedCalibrating = false;
	}

	if (mode == GYRO_AUTOCALIBRATION_ALWAYS) {
		const Uint32 STARTUP_DELAY       = 120;
		const Uint32 INTERVAL            = 700;
		const float  CONFIDENCE_THRESHOLD = 0.50f;

		if (!state->alwaysModeActive) {
			state->wasStill          = false;
			state->lastAutoCalibTime = 0;
			state->isCalibrating     = false;
			state->alwaysModeActive  = true;
		}

		if (stillness) {
			if (!state->wasStill) {
				state->lastAutoCalibTime = now - STARTUP_DELAY;
			}
			if (now - state->lastAutoCalibTime >= INTERVAL && !state->isCalibrating) {
				if (confidence < CONFIDENCE_THRESHOLD) {
					gmhStartContinuousCalibration(gpadMotion[cidx]);
					state->lastAutoCalibTime = now;
					state->isCalibrating     = true;
				} else {
					state->lastAutoCalibTime = now;
				}
			}
		} else {
			if (state->wasStill) {
				gmhPauseContinuousCalibration(gpadMotion[cidx]);
				state->isCalibrating = false;
			}
		}

		state->wasStill = stillness;
		return;
	}

	if (mode == GYRO_AUTOCALIBRATION_STATIONARY || mode == GYRO_AUTOCALIBRATION_MENU_ONLY) {
		const Uint32 STARTUP_DELAY       = 2500;
		const Uint32 CALIBRATION_DURATION = 1200;
		const Uint32 COOLDOWN            = 10000;
		const float  CONFIDENCE_THRESHOLD_MENU_ONLY = 0.80f;

		if (!state->stationaryModeActive) {
			state->wasStill           = false;
			state->lastAutoCalibTime  = 0;
			state->autoCalibStartTime = 0;
			state->isCalibrating      = false;
			state->stationaryModeActive = true;
		}

		const bool   isStationary       = stillness && inputIsControllerSensorNoiseThreshold(cidx);
		const Uint32 timeSinceLastEvent  = now - state->lastAutoCalibTime;
		const Uint32 delayNeeded         = (state->autoCalibStartTime == 0) ? STARTUP_DELAY : COOLDOWN;
		const bool   timingConditionsMet = !state->isCalibrating && timeSinceLastEvent >= delayNeeded;

		if (isStationary) {
			if (!state->wasStill && state->lastAutoCalibTime == 0) {
				state->lastAutoCalibTime = now;
			}

			bool shouldCalibrate = false;
			if (mode == GYRO_AUTOCALIBRATION_MENU_ONLY) {
				if (timingConditionsMet) {
					if (confidence < CONFIDENCE_THRESHOLD_MENU_ONLY) {
						shouldCalibrate = true;
					} else {
						state->lastAutoCalibTime = now;
					}
				}
			} else {
				shouldCalibrate = timingConditionsMet;
			}

			if (shouldCalibrate) {
				gmhStartContinuousCalibration(gpadMotion[cidx]);
				state->autoCalibStartTime = now;
				state->isCalibrating      = true;
			}

			if (state->isCalibrating && (now - state->autoCalibStartTime >= CALIBRATION_DURATION)) {
				gmhPauseContinuousCalibration(gpadMotion[cidx]);
				inputGyroCalibrationFinished(cidx, true);
				state->isCalibrating     = false;
				state->lastAutoCalibTime = now;
			}
		} else {
			if (state->isCalibrating) {
				gmhPauseContinuousCalibration(gpadMotion[cidx]);
				gmhResetContinuousCalibration(gpadMotion[cidx]);
				state->isCalibrating = false;
			}
		}

		state->wasStill = isStationary;
		return;
	}
}

static void inputUpdateGyroCalibrationHandle(void)
{
	for (s32 cidx = 0; cidx < INPUT_MAX_CONTROLLERS; ++cidx) {
		if (!inputGyroIsEnabled(cidx)) continue;
		inputUpdateGyroManualCalibration(cidx);
		if (inputGyroGetAutoCalibration(cidx) != GYRO_AUTOCALIBRATION_OFF) {
			inputUpdateGyroAutoCalibration(cidx);
		}
	}
}

void gyroUpdateAll(void)
{
	inputUpdateGyroCalibrationHandle();
	for (s32 cidx = 0; cidx < INPUT_MAX_CONTROLLERS; ++cidx) {
		if (inputGyroIsEnabled(cidx)) {
			inputUpdateGyro(cidx);
		}
	}
}

void gyroReconfigureCalibrationMode(s32 cidx)
{
	if (cidx < 0 || cidx >= INPUT_MAX_CONTROLLERS) return;

	GyroCalibState *state    = &gyroCalibState[cidx];
	state->wasStill          = false;
	state->lastAutoCalibTime = 0;
	state->autoCalibStartTime = 0;

	if (gpadMotion[cidx]) {
		inputSaveGyroCalibrationOffset(cidx);
		inputConfigureGyroCalibrationMode(cidx);
	}
}

void inputGyroCalibration(s32 cidx, GyroCalibrationOp op, float *out_confidence, int *out_steady)
{
	if (cidx < 0 || cidx >= INPUT_MAX_CONTROLLERS) return;

	switch (op) {
	case GYRO_CALIB_START:
		inputGyroManualCalibrationActivation(cidx, true);
		break;
	case GYRO_CALIB_FINISH:
		inputGyroManualCalibrationActivation(cidx, false);
		break;
	case GYRO_CALIB_RESET:
		inputResetGyroCalibration(cidx);
		break;
	case GYRO_CALIB_QUERY:
		if (gpadMotion[cidx]) {
			if (out_confidence) *out_confidence = gmhGetAutoCalibrationConfidence(gpadMotion[cidx]);
			if (out_steady)     *out_steady     = gmhGetAutoCalibrationIsSteady(gpadMotion[cidx]);
		}
		break;
	case GYRO_CALIB_AUTO:
		inputUpdateGyroAutoCalibration(cidx);
		break;
	}
}

s32 inputGyroGetManualCalibration(s32 cidx)
{
	if (cidx < 0 || cidx >= INPUT_MAX_CONTROLLERS) return 0;
	return gyroCalibState[cidx].manualCalibActive ? 1 : 0;
}

void inputGyroSetManualCalibration(s32 cidx)
{
	if (cidx < 0 || cidx >= INPUT_MAX_CONTROLLERS) return;
	if (!gpadMotion[cidx]) return;
	if (inputIsGyroCalibrationBlocked(cidx)) return;
	inputGyroManualCalibrationActivation(cidx, true);
}

void gyroGetOrientation(s32 cidx, f32 *yaw, f32 *pitch, f32 *roll)
{
	if (yaw)   *yaw   = gyroOrientYaw[cidx];
	if (pitch) *pitch = gyroOrientPitch[cidx];
	if (roll)  *roll  = gyroOrientRoll[cidx];
}
