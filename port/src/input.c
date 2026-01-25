#include <string.h>
#include <math.h>
#include <ctype.h>
#include <SDL.h>
#include <SDL_sensor.h>
#include <PR/ultratypes.h>
#include <PR/os_thread.h>
#include <PR/os_cont.h>
#include "platform.h"
#include "input.h"
#include "../gamepadmotionhelper/GamepadMotion.h"
#include "video.h"
#include "config.h"
#include "utils.h"
#include "system.h"
#include "fs.h"
#include "game/menu.h"
#include "bss.h"

#if !SDL_VERSION_ATLEAST(2, 0, 14)
// this was added in 2.0.14
#define SDL_CONTROLLER_TYPE_VIRTUAL SDL_CONTROLLER_TYPE_UNKNOWN
#endif

#define CONTROLLERDB_FNAME "gamecontrollerdb.txt"
#define MAX_BIND_STR 256

#define TRIG_THRESHOLD (30 * 256)
#define DEFAULT_DEADZONE 4096
#define DEFAULT_DEADZONE_RY 6144

#define WHEEL_UP_MASK SDL_BUTTON(VK_MOUSE_WHEEL_UP - VK_MOUSE_BEGIN + 1)
#define WHEEL_DN_MASK SDL_BUTTON(VK_MOUSE_WHEEL_DN - VK_MOUSE_BEGIN + 1)

#define CURSOR_HIDE_THRESHOLD 1
#define CURSOR_HIDE_TIME 3000000 // us

// Gravity constant for converting accelerometer data from m/s^2 to units of gravity (g)
#define SDL_STANDARD_GRAVITY 9.80665f

// GamepadMotionHelper autocalibration constants 
#define GMH_STILLNESS_COLLECTION_TIME 2.5f // Time to collect stillness data before calibration starts
#define GMH_STILLNESS_CORRECTION_TIME 3.0f // Time to correct/build calibration confidence
#define GMH_STILLNESS_EASE_TIME 1.0f       // Time to ease calibration transitions
#define GMH_MAX_STILLNESS_ERROR 1.5f       // Maximum sensor error allowed during stillness detection

// Motion sensor noise threshold constants
#define GYRO_NOISE_THRESHOLD 0.16f            // Absolute gyro threshold (degrees/second)
#define GYRO_RATE_THRESHOLD 0.3f              // Frame-to-frame gyro change threshold (degrees/second)
#define ACCEL_GRAVITY_TOLERANCE 0.02f         // Gravity magnitude tolerance
#define ACCEL_DELTA_THRESHOLD 0.01f           // Accelerometer movement threshold in G units

static SDL_GameController *pads[INPUT_MAX_CONTROLLERS];

#define CONTROLLERCFG_DEFAULT { \
	.rumbleOn = 0, \
	.rumbleScale = 0.5f, \
	.axisMap = { \
		{ SDL_CONTROLLER_AXIS_LEFTX,  SDL_CONTROLLER_AXIS_LEFTY  }, \
		{ SDL_CONTROLLER_AXIS_RIGHTX, SDL_CONTROLLER_AXIS_RIGHTY }, \
	}, \
	.sens = { 1.f, 1.f, 1.f, 1.f }, \
	.deadzone = { DEFAULT_DEADZONE, DEFAULT_DEADZONE, DEFAULT_DEADZONE, DEFAULT_DEADZONE_RY }, \
	.stickCButtons = 0, \
	.swapSticks = 1, \
	.deviceIndex = -1, \
	.cancelCButtons = 0, \
	.gyroEnabled = 1, \
	.gyroAxisMode = GYRO_YAW, \
	.gyroAimMode = GYRO_AIM_CROSSHAIR, \
	.gyroModifier = GYRO_ALWAYS_ON, \
	.gyroAdvanced = 0, \
	.gyroSpeedX = 2.5f, \
	.gyroSpeedY = 2.5f, \
	.gyroAimSpeedX = 5.0f, \
	.gyroAimSpeedY = 5.0f, \
	.gyroVHMixer = 0.0f, \
	.gyroInvertX = 0, \
	.gyroInvertY = 0, \
	.gyroAimInvertX = 0, \
	.gyroAimInvertY = 0, \
	.gyroSmoothing = 0.18f, \
	.gyroTightening = 0.07f, \
	.gyroDeadzone = 0.03f, \
	.gyroAutoCalibration = 1, \
}

static struct controllercfg {
	s32 rumbleOn;
	f32 rumbleScale;
	u32 axisMap[2][2];
	f32 sens[4];
	s32 deadzone[4];
	s32 gyroSensorActive;
	s32 stickCButtons;
	s32 swapSticks;
	s32 deviceIndex;
	s32 cancelCButtons;
	s32 gyroEnabled;
	s32 gyroAxisMode;
	s32 gyroAimMode;
	s32 gyroModifier;
	s32 gyroAdvanced;
	f32 gyroSpeedX;
	f32 gyroSpeedY;
	f32 gyroAimSpeedX;
	f32 gyroAimSpeedY;
	f32 gyroVHMixer;
	s32 gyroInvertX;
	s32 gyroInvertY;
	s32 gyroAimInvertX;
	s32 gyroAimInvertY;
	f32 gyroSmoothing;
	f32 gyroTightening;
	f32 gyroDeadzone;
	s32 gyroAutoCalibration;
} padsCfg[INPUT_MAX_CONTROLLERS] = {
	CONTROLLERCFG_DEFAULT,
	CONTROLLERCFG_DEFAULT,
	CONTROLLERCFG_DEFAULT,
	CONTROLLERCFG_DEFAULT
};

static u32 binds[MAXCONTROLLERS][CK_TOTAL_COUNT][INPUT_MAX_BINDS];
static char bindStrs[MAXCONTROLLERS][CK_TOTAL_COUNT][MAX_BIND_STR];

static s32 fakeControllers = 0;
static s32 firstController = 0;
static s32 connectedMask = 0;

static s32 numJoysticks = 0;

static s32 useHIDAPI = 1;
static s32 useRawInput = 0;

static s32 mouseEnabled = 1;
static s32 mouseX, mouseY;
static s32 mouseDX, mouseDY;
static u32 mouseButtons;
static s32 mouseWheel = 0;

static s32 mouseLocked = 0;
static s32 mouseLockMode = MLOCK_AUTO;
static u64 mouseCursorTime = 0;
static s32 mouseShowCursor = 1;

static f32 mouseSensX = 2.5f;
static f32 mouseSensY = 2.5f;

static s32 lastKey = 0;
static char lastChar = 0;
static s32 textInput = 0;

static char *clipboardText = NULL;

// Motion Sensor data declarations
static GamepadMotionHandle gpadMotion[INPUT_MAX_CONTROLLERS] = { NULL };
static f32 gyroDeltaYaw[INPUT_MAX_CONTROLLERS], gyroDeltaPitch[INPUT_MAX_CONTROLLERS], gyroDeltaRoll[INPUT_MAX_CONTROLLERS];
static float gyroData[INPUT_MAX_CONTROLLERS][3] = {0};
static float accelData[INPUT_MAX_CONTROLLERS][3] = {0};
static bool hasSensorData[INPUT_MAX_CONTROLLERS] = {false};

// Gyro processing function declarations
static inline void applyGyroInvert(s32 cidx, f32* dx, f32* dy, bool useAimInvert);
static inline void applyGyroVHMixer(s32 cidx, f32* dx, f32* dy);
static void applyGyroAxisMapping(s32 cidx, float calibratedGyro[3], f32* deltaX, f32* deltaY, f32* deltaZ);
static void applyGyroModifier(f32* deltaX, f32* deltaY, f32* deltaZ, s32 activationMode, s32 idx);
static void applyGyroDeadzone(f32* dx, f32* dy, f32* dz, f32 deadzone);
static void applyGyroTightening(f32* dx, f32* dy, f32* dz, f32 tightening);
static void applyGyroSmoothing(f32* deltaX, f32* deltaY, f32* deltaZ, f32 smoothing, s32 cidx);

// Gyro calibration configuration
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

// Gyro calibration state structure
typedef struct {
	// Manual calibration state
	bool manualCalibActive;          // Is manual calibration currently active
	Uint32 manualCalibStartTime;     // When manual calibration started
	f32 manualOffsetX, manualOffsetY, manualOffsetZ; // Manual calibration offsets
	s32 manualWeight;                // Weight for manual calibration
	
	// Auto-calibration state
	bool wasStill;                   // Was the controller still during auto-calibration
	Uint32 lastAutoCalibTime;        // When controller last moved (for cooldown)
	Uint32 autoCalibStartTime;       // When auto-calibration started (for minimum duration check)
	bool isCalibrating;              // Is auto-calibration actively running

	// General state
	bool justFinishedCalibrating;    // Has the controller just finished calibrating
} GyroCalibState;

static GyroCalibState gyroCalibState[INPUT_MAX_CONTROLLERS] = {0};

static const char *ckNames[CK_TOTAL_COUNT] = {
	"R_CBUTTONS",
	"L_CBUTTONS",
	"D_CBUTTONS",
	"U_CBUTTONS",
	"R_TRIG",
	"L_TRIG",
	"X_BUTTON",
	"Y_BUTTON",
	"R_JPAD",
	"L_JPAD",
	"D_JPAD",
	"U_JPAD",
	"START_BUTTON",
	"Z_TRIG",
	"B_BUTTON",
	"A_BUTTON",
	"STICK_XNEG",
	"STICK_XPOS",
	"STICK_YNEG",
	"STICK_YPOS",
	"ACCEPT_BUTTON",
	"CANCEL_BUTTON",
	"CK_0040",
	"CK_0080",
	"CK_0100",
	"CK_0200",
	"CK_0400",
	"CK_0800",
	"CK_1000",
	"CK_2000",
	"CK_4000",
	"CK_8000"
};

static const char *vkPunctNames[] = {
	"MINUS", "EQUALS", "LEFTBRACKET", "RIGHTBRACKET", "BACKSLASH",
	"HASH", "SEMICOLON", "APOSTROPHE", "GRAVE", "COMMA", "PERIOD", "SLASH"
};

static const char *vkMouseNames[] = {
	"MOUSE_LEFT",
	"MOUSE_MIDDLE",
	"MOUSE_RIGHT",
	"MOUSE_X1",
	"MOUSE_X2",
	"MOUSE_WHEEL_UP",
	"MOUSE_WHEEL_DN",
};

static const char *vkJoyNames[] = {
	"JOY1_A",
	"JOY1_B",
	"JOY1_X",
	"JOY1_Y",
	"JOY1_BACK",
	"JOY1_GUIDE",
	"JOY1_START",
	"JOY1_LSTICK",
	"JOY1_RSTICK",
	"JOY1_LSHOULDER",
	"JOY1_RSHOULDER",
	"JOY1_DPAD_UP",
	"JOY1_DPAD_DOWN",
	"JOY1_DPAD_LEFT",
	"JOY1_DPAD_RIGHT",
	"JOY1_BUTTON_15",
	"JOY1_BUTTON_16",
	"JOY1_BUTTON_17",
	"JOY1_BUTTON_18",
	"JOY1_BUTTON_19",
	"JOY1_TOUCHPAD",
	"JOY1_BUTTON_21",
	"JOY1_BUTTON_22",
	"JOY1_BUTTON_23",
	"JOY1_BUTTON_24",
	"JOY1_BUTTON_25",
	"JOY1_BUTTON_26",
	"JOY1_BUTTON_27",
	"JOY1_BUTTON_28",
	"JOY1_BUTTON_29",
	"JOY1_LTRIGGER",
	"JOY1_RTRIGGER",
};

static char vkNames[VK_TOTAL_COUNT][64];

static s8 vkPrevState[VK_TOTAL_COUNT];

void inputSetDefaultKeyBinds(s32 cidx, s32 n64mode)
{
	// TODO: make VK constants for all these
	static const u32 pckbbinds[][3] = {
		{ CK_B,             SDL_SCANCODE_E,      0                   },
		{ CK_X,             SDL_SCANCODE_R,      0                   },
		{ CK_RTRIG,         VK_MOUSE_RIGHT,      SDL_SCANCODE_Z      },
		{ CK_LTRIG,         SDL_SCANCODE_F,      SDL_SCANCODE_X      },
		{ CK_ZTRIG,         VK_MOUSE_LEFT,       SDL_SCANCODE_SPACE  },
		{ CK_START,         SDL_SCANCODE_RETURN, SDL_SCANCODE_TAB    },
		{ CK_DPAD_D,        SDL_SCANCODE_Q,      VK_MOUSE_MIDDLE     },
		{ CK_DPAD_U,        0,                   0                   },
		{ CK_Y,             VK_MOUSE_WHEEL_DN,   0                   },
		{ CK_DPAD_L,        VK_MOUSE_WHEEL_UP,   0                   },
		{ CK_C_D,           SDL_SCANCODE_S,      0                   },
		{ CK_C_U,           SDL_SCANCODE_W,      0                   },
		{ CK_C_R,           SDL_SCANCODE_D,      0                   },
		{ CK_C_L,           SDL_SCANCODE_A,      0                   },
		{ CK_STICK_XNEG,    SDL_SCANCODE_LEFT,   0                   },
		{ CK_STICK_XPOS,    SDL_SCANCODE_RIGHT,  0                   },
		{ CK_STICK_YNEG,    SDL_SCANCODE_DOWN,   0                   },
		{ CK_STICK_YPOS,    SDL_SCANCODE_UP,     0                   },
		{ CK_0100,          SDL_SCANCODE_F10,    0                   },
		{ CK_4000,          SDL_SCANCODE_LSHIFT, 0                   },
		{ CK_2000,          SDL_SCANCODE_LCTRL,  0                   }
	};

	static const u32 pcjoybinds[][2] = {
		{ CK_A,      SDL_CONTROLLER_BUTTON_A             },
		{ CK_X,      SDL_CONTROLLER_BUTTON_X             },
		{ CK_Y,      SDL_CONTROLLER_BUTTON_Y             },
		{ CK_DPAD_L, SDL_CONTROLLER_BUTTON_B             },
		{ CK_DPAD_D, SDL_CONTROLLER_BUTTON_LEFTSHOULDER  },
		{ CK_LTRIG,  SDL_CONTROLLER_BUTTON_RIGHTSHOULDER },
		{ CK_RTRIG,  VK_JOY1_LTRIG - VK_JOY1_BEGIN       },
		{ CK_ZTRIG,  VK_JOY1_RTRIG - VK_JOY1_BEGIN       },
		{ CK_START,  SDL_CONTROLLER_BUTTON_START         },
		{ CK_C_D,    SDL_CONTROLLER_BUTTON_DPAD_DOWN     },
		{ CK_C_U,    SDL_CONTROLLER_BUTTON_DPAD_UP       },
		{ CK_C_R,    SDL_CONTROLLER_BUTTON_DPAD_RIGHT    },
		{ CK_C_L,    SDL_CONTROLLER_BUTTON_DPAD_LEFT     },
		{ CK_ACCEPT, SDL_CONTROLLER_BUTTON_A             },
		{ CK_CANCEL, SDL_CONTROLLER_BUTTON_B             },
		{ CK_0100,   SDL_CONTROLLER_BUTTON_BACK          },
		{ CK_8000,   SDL_CONTROLLER_BUTTON_LEFTSTICK     },
	};

	static const u32 n64kbbinds[][3] = {
		{ CK_A,          SDL_SCANCODE_Q,      0                  },
		{ CK_B,          SDL_SCANCODE_E,      0                  },
		{ CK_RTRIG,      VK_MOUSE_RIGHT,      SDL_SCANCODE_LALT  },
		{ CK_LTRIG,      SDL_SCANCODE_F,      0                  },
		{ CK_ZTRIG,      VK_MOUSE_LEFT,       SDL_SCANCODE_SPACE },
		{ CK_START,      SDL_SCANCODE_RETURN, 0                  },
		{ CK_C_D,        SDL_SCANCODE_S,      0                  },
		{ CK_C_U,        SDL_SCANCODE_W,      0                  },
		{ CK_C_R,        SDL_SCANCODE_D,      0                  },
		{ CK_C_L,        SDL_SCANCODE_A,      0                  },
		{ CK_DPAD_L,     SDL_SCANCODE_LEFT,   0                  },
		{ CK_DPAD_R,     SDL_SCANCODE_RIGHT,  0                  },
		{ CK_DPAD_D,     SDL_SCANCODE_DOWN,   0                  },
		{ CK_DPAD_U,     SDL_SCANCODE_UP,     0                  },
		{ CK_STICK_YNEG, SDL_SCANCODE_K,      0                  },
		{ CK_STICK_YPOS, SDL_SCANCODE_I,      0                  },
		{ CK_STICK_XNEG, SDL_SCANCODE_J,      0                  },
		{ CK_STICK_XPOS, SDL_SCANCODE_L,      0                  },
		{ CK_0100,       SDL_SCANCODE_F10,    0                  },
	};

	static const u32 n64joybinds[][2] = {
		{ CK_A,      SDL_CONTROLLER_BUTTON_A             },
		{ CK_B,      SDL_CONTROLLER_BUTTON_B             },
		{ CK_LTRIG,  SDL_CONTROLLER_BUTTON_LEFTSHOULDER  },
		{ CK_RTRIG,  SDL_CONTROLLER_BUTTON_RIGHTSHOULDER },
		{ CK_ZTRIG,  VK_JOY1_RTRIG - VK_JOY1_BEGIN       },
		{ CK_START,  SDL_CONTROLLER_BUTTON_START         },
		{ CK_DPAD_D, SDL_CONTROLLER_BUTTON_DPAD_DOWN     },
		{ CK_DPAD_U, SDL_CONTROLLER_BUTTON_DPAD_UP       },
		{ CK_DPAD_L, SDL_CONTROLLER_BUTTON_DPAD_LEFT     },
		{ CK_DPAD_R, SDL_CONTROLLER_BUTTON_DPAD_RIGHT    },
		{ CK_0100,   SDL_CONTROLLER_BUTTON_BACK          },
	};

	memset(binds[cidx], 0, sizeof(binds[cidx]));

	const u32 (*kbbinds)[3];
	const u32 (*joybinds)[2];
	u32 numkbbinds;
	u32 numjoybinds;
	if (n64mode) {
		kbbinds = n64kbbinds;
		joybinds = n64joybinds;
		numkbbinds = sizeof(n64kbbinds) / sizeof(n64kbbinds[0]);
		numjoybinds = sizeof(n64joybinds) / sizeof(n64joybinds[0]);
	} else {
		kbbinds = pckbbinds;
		joybinds = pcjoybinds;
		numkbbinds = sizeof(pckbbinds) / sizeof(pckbbinds[0]);
		numjoybinds = sizeof(pcjoybinds) / sizeof(pcjoybinds[0]);
	}

	if (cidx == 0) {
		for (u32 i = 0; i < numkbbinds; ++i) {
			for (s32 j = 1; j < 3; ++j) {
				if (kbbinds[i][j]) {
					inputKeyBind(cidx, kbbinds[i][0], j - 1, kbbinds[i][j]);
				}
			}
		}
	}

	for (u32 i = 0; i < numjoybinds; ++i) {
		inputKeyBind(cidx, joybinds[i][0], -1, VK_JOY_BEGIN + cidx * INPUT_MAX_CONTROLLER_BUTTONS + joybinds[i][1]);
	}
}

static inline s32 inputDeviceIndexFromId(const SDL_JoystickID id) {
	for (s32 jidx = 0; jidx < numJoysticks; ++jidx) {
		if (SDL_JoystickGetDeviceInstanceID(jidx) == id) {
			return jidx;
		}
	}
	return -1;
}

static inline SDL_JoystickID inputControllerGetId(SDL_GameController *ctrl)
{
	return SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(ctrl));
}

static inline void inputInitController(const s32 cidx, const s32 jidx)
{
#if SDL_VERSION_ATLEAST(2, 0, 18)
	// SDL_GameControllerHasRumble() appeared in 2.0.18 even though SDL_GameControllerRumble() is in 2.0.9
	padsCfg[cidx].rumbleOn = SDL_GameControllerHasRumble(pads[cidx]);
#else
	// assume that all joysticks with haptic feedback support will support rumble
	padsCfg[cidx].rumbleOn = SDL_JoystickIsHaptic(SDL_GameControllerGetJoystick(pads[cidx]));
	if (!padsCfg[cidx].rumbleOn) {
		// at least on Windows some controllers will report no haptics, but rumble will still function
		// just assume it's supported if the controller is of known type
		const SDL_GameControllerType ctype = SDL_GameControllerGetType(pads[cidx]);
		padsCfg[cidx].rumbleOn = ctype && (ctype != SDL_CONTROLLER_TYPE_VIRTUAL);
	}
#endif

	// make the LEDs on the controller indicate which player it's for
	SDL_GameControllerSetPlayerIndex(pads[cidx], cidx);

	// remember the joystick index
	padsCfg[cidx].deviceIndex = jidx;

	connectedMask |= (1 << cidx);

	sysLogPrintf(LOG_NOTE, "input: assigned controller '%d: (%s)' (id %d) to player %d",
		jidx, SDL_GameControllerName(pads[cidx]), inputControllerGetId(pads[cidx]), cidx);

	SDL_Joystick* joy = SDL_GameControllerGetJoystick(pads[cidx]);
	if (joy) {
		char guidStr[1024] = "";
		SDL_JoystickGUID guid = SDL_JoystickGetGUID(joy);
		SDL_JoystickGetGUIDString(guid, guidStr, sizeof(guidStr));
		sysLogPrintf(LOG_NOTE, "input: GUID for controller %d: %s", jidx, guidStr);
	}

#if SDL_VERSION_ATLEAST(2, 0, 14)
	// Try to initialize motion sensors (gyro + accelerometer)
	padsCfg[cidx].gyroSensorActive = 0;

	if (SDL_GameControllerHasSensor(pads[cidx], SDL_SENSOR_GYRO) && 
	    SDL_GameControllerHasSensor(pads[cidx], SDL_SENSOR_ACCEL)) {
		sysLogPrintf(LOG_NOTE, "input: detects assigned controller's motion sensors for player %d", cidx);

		if (SDL_GameControllerSetSensorEnabled(pads[cidx], SDL_SENSOR_GYRO, SDL_TRUE) == 0 &&
		    SDL_GameControllerSetSensorEnabled(pads[cidx], SDL_SENSOR_ACCEL, SDL_TRUE) == 0) {

			padsCfg[cidx].gyroSensorActive = 1;

			// Create or reset GamepadMotionHelper instance
			if (!gpadMotion[cidx]) {
				gpadMotion[cidx] = gmhCreateGamepadMotion();
			} else {
				gmhResetGamepadMotion(gpadMotion[cidx]);
			}

			// Configure GamepadMotionHelper settings and calibration
			if (gpadMotion[cidx]) {
				inputConfigureGamepadMotionSettings(gpadMotion[cidx]);
				gmhResetMotion(gpadMotion[cidx]);
				inputConfigureGyroCalibrationMode(cidx);
			}
		}
	}
#endif
}

// Pause gyro deltas and orientation to prevent gyro input leak
static inline void inputPauseGyro(s32 cidx)
{
    gyroDeltaYaw[cidx] = gyroDeltaPitch[cidx] = gyroDeltaRoll[cidx] = 0.f;
}

static inline void inputCloseController(const s32 cidx)
{
	sysLogPrintf(LOG_NOTE, "input: removed controller '%d: (%s)' (id %d) from player %d",
		padsCfg[cidx].deviceIndex, SDL_GameControllerName(pads[cidx]), inputControllerGetId(pads[cidx]), cidx);

	// reset player LEDs
	SDL_GameControllerSetPlayerIndex(pads[cidx], -1);

	SDL_GameControllerClose(pads[cidx]);

	pads[cidx] = NULL;
	padsCfg[cidx].rumbleOn = 0;
	padsCfg[cidx].gyroSensorActive = 0;

	// Clear sensor data
	hasSensorData[cidx] = false;
	gyroData[cidx][0] = gyroData[cidx][1] = gyroData[cidx][2] = 0.f;
	accelData[cidx][0] = accelData[cidx][1] = accelData[cidx][2] = 0.f;

	// Clean up GamepadMotion instance
	if (gpadMotion[cidx]) {
		gmhDeleteGamepadMotion(gpadMotion[cidx]);
		gpadMotion[cidx] = NULL;
	}

	inputPauseGyro(cidx);

	if (cidx) {
		connectedMask &= ~(1 << cidx);
	}
}

static inline s32 inputControllerGetIndex(SDL_GameController *ctrl)
{
	if (ctrl) {
		for (s32 i = 0; i < INPUT_MAX_CONTROLLERS; ++i) {
			if (pads[i] == ctrl) {
				return i;
			}
		}
	}
	return -1;
}

static inline s32 inputControllerGetIndexByDeviceIndex(const s32 jidx)
{
	for (s32 cidx = 0; cidx < INPUT_MAX_CONTROLLERS; ++cidx) {
		if (pads[cidx] && padsCfg[cidx].deviceIndex == jidx) {
			return cidx;
		}
	}
	return -1;
}

static inline s32 inputControllerGetIndexById(const SDL_JoystickID jid)
{
	for (s32 cidx = 0; cidx < INPUT_MAX_CONTROLLERS; ++cidx) {
		if (pads[cidx]) {
			if (inputControllerGetId(pads[cidx]) == jid) {
				return cidx;
			}
		}
	}
	return -1;
}

static inline void inputCloseAllControllers(void)
{
	for (s32 cidx = 0; cidx < INPUT_MAX_CONTROLLERS; ++cidx) {
		if (pads[cidx]) {
			inputCloseController(cidx);
			pads[cidx] = NULL;
		}
	}

	connectedMask = 1; // always report first controller as connected
}

static inline s32 inputTryController(const s32 cidx, const s32 jidx)
{
	if (!pads[cidx]) {
		pads[cidx] = SDL_GameControllerOpen(jidx);
		if (pads[cidx]) {
			inputInitController(cidx, jidx);
			return 1;
		}
	}
	return 0;
}

static inline void inputInitAllControllers(void)
{
	SDL_GameControllerUpdate();

	numJoysticks = SDL_NumJoysticks();

	connectedMask = 1; // always report first controller as connected

	// first try to assign the controllers that we had last time
	// we're still free to check by device index before any controller device events fire
	for (s32 cidx = 0; cidx < INPUT_MAX_CONTROLLERS; ++cidx) {
		const s32 jidx = padsCfg[cidx].deviceIndex;
		if (jidx >= 0 && jidx < numJoysticks) {
			if (SDL_IsGameController(jidx) && inputControllerGetIndexByDeviceIndex(jidx) < 0) {
				// using the full assign function in case user sets same index for several players
				if (inputTryController(cidx, jidx)) {
					// success
					continue;
				}
			}
			// nothing was there, forget it
			padsCfg[cidx].deviceIndex = -1;
		}
	}

	// now try autofilling the rest, starting with firstController
	for (s32 jidx = 0; jidx < numJoysticks; ++jidx) {
		if (SDL_IsGameController(jidx) && inputControllerGetIndexByDeviceIndex(jidx) < 0) {
			for (s32 cidx = firstController; cidx < INPUT_MAX_CONTROLLERS; ++cidx) {
				if (inputTryController(cidx, jidx)) {
					break;
				}
			}
		}
	}

	const s32 overrideMask = (1 << fakeControllers) - 1;
	if (overrideMask) {
		connectedMask = overrideMask;
	}
}

static int inputEventFilter(void *data, SDL_Event *event)
{
	switch (event->type) {
		case SDL_CONTROLLERDEVICEADDED:
			for (s32 i = firstController; i < INPUT_MAX_CONTROLLERS; ++i) {
				if (!pads[i]) {
					pads[i] = SDL_GameControllerOpen(event->cdevice.which);
					if (pads[i]) {
						inputInitController(i, event->cdevice.which);
					}
					break;
				}
			}
			break;

		case SDL_CONTROLLERDEVICEREMOVED: {
			SDL_GameController *ctrl = SDL_GameControllerFromInstanceID(event->cdevice.which);
			const s32 idx = inputControllerGetIndex(ctrl);
			if (idx >= 0) {
				inputCloseController(idx);
				padsCfg[idx].deviceIndex = -1;
			}
			break;
		}

		case SDL_JOYDEVICEADDED:
		case SDL_JOYDEVICEREMOVED:
			numJoysticks = SDL_NumJoysticks(); // joystick count has changed
			break;

		case SDL_MOUSEWHEEL:
			mouseWheel = event->wheel.y;
			if (!lastKey && mouseWheel) {
				lastKey = (mouseWheel < 0) + VK_MOUSE_WHEEL_UP;
			}
			break;

		case SDL_MOUSEBUTTONDOWN:
			if (!lastKey) {
				lastKey = VK_MOUSE_BEGIN - 1 + event->button.button;
			}
			break;

		case SDL_KEYDOWN:
			if (!lastKey) {
				lastKey = VK_KEYBOARD_BEGIN + event->key.keysym.scancode;
			}
			break;

		case SDL_CONTROLLERBUTTONDOWN:
			if (!lastKey) {
				lastKey = VK_JOY1_BEGIN + event->cbutton.button;
				SDL_GameController *ctrl = SDL_GameControllerFromInstanceID(event->cdevice.which);
				const s32 idx = inputControllerGetIndex(ctrl);
				if (idx >= 0) {
					lastKey += idx * INPUT_MAX_CONTROLLER_BUTTONS;
				}
			}
			break;

		case SDL_CONTROLLERAXISMOTION:
			if (!lastKey) {
				if (event->caxis.axis >= SDL_CONTROLLER_AXIS_TRIGGERLEFT && event->caxis.value > TRIG_THRESHOLD) {
					lastKey = VK_JOY1_LTRIG + (event->caxis.axis - SDL_CONTROLLER_AXIS_TRIGGERLEFT);
					SDL_GameController *ctrl = SDL_GameControllerFromInstanceID(event->cdevice.which);
					const s32 idx = inputControllerGetIndex(ctrl);
					if (idx >= 0) {
						lastKey += idx * INPUT_MAX_CONTROLLER_BUTTONS;
					}
				}
			}
			break;

		case SDL_CONTROLLERSENSORUPDATE: {
			SDL_GameController *ctrl = SDL_GameControllerFromInstanceID(event->csensor.which);
			const s32 idx = inputControllerGetIndex(ctrl);
			if (idx >= 0 && idx < INPUT_MAX_CONTROLLERS) {
				if (event->csensor.sensor == SDL_SENSOR_GYRO) {
					gyroData[idx][0] = event->csensor.data[0];
					gyroData[idx][1] = event->csensor.data[1];
					gyroData[idx][2] = event->csensor.data[2];
					hasSensorData[idx] = true;
				} else if (event->csensor.sensor == SDL_SENSOR_ACCEL) {
					accelData[idx][0] = event->csensor.data[0] / SDL_STANDARD_GRAVITY;
					accelData[idx][1] = event->csensor.data[1] / SDL_STANDARD_GRAVITY;
					accelData[idx][2] = event->csensor.data[2] / SDL_STANDARD_GRAVITY;
					hasSensorData[idx] = true;
				}
			}
			break;
		}

		case SDL_TEXTINPUT:
			if (!lastChar && event->text.text[0] && (u8)event->text.text[0] < 0x80) {
				lastChar = event->text.text[0];
			}
			break;

		default:
			break;
	}

	return 0;
}

static inline void inputGetScancodeName(const SDL_Scancode sc, char *out, size_t len)
{
		const char *scname = SDL_GetScancodeName(sc);
		if (scname) {
			strncpy(out, scname, len - 1);
			for (u32 i = 0; i < len && out[i]; ++i) {
				if (out[i] == ' ') {
					out[i] = '_';
				} else {
					out[i] = toupper(out[i]);
				}
			}
		} else {
			snprintf(out, len, "KEY%d", (s32)sc);
		}
}

static inline void inputInitKeyNames(void)
{
	for (SDL_Scancode key = SDL_SCANCODE_A; key <= SDL_SCANCODE_SPACE; ++key) {
		inputGetScancodeName(key, vkNames[key], sizeof(vkNames[key]));
	}

	// special characters
	for (SDL_Scancode key = SDL_SCANCODE_MINUS; key < SDL_SCANCODE_CAPSLOCK; ++key) {
		strcpy(vkNames[key], vkPunctNames[key - SDL_SCANCODE_MINUS]);
	}

	for (SDL_Scancode key = SDL_SCANCODE_CAPSLOCK; key <= SDL_SCANCODE_NUMLOCKCLEAR; ++key) {
		inputGetScancodeName(key, vkNames[key], sizeof(vkNames[key]));
	}

	// keypad names
	strcpy(vkNames[SDL_SCANCODE_KP_DIVIDE], "KP_DIVIDE");
	strcpy(vkNames[SDL_SCANCODE_KP_MULTIPLY], "KP_MULTIPLY");
	strcpy(vkNames[SDL_SCANCODE_KP_MINUS], "KP_MINUS");
	strcpy(vkNames[SDL_SCANCODE_KP_PLUS], "KP_PLUS");
	strcpy(vkNames[SDL_SCANCODE_KP_ENTER], "KP_ENTER");
	strcpy(vkNames[SDL_SCANCODE_KP_PERIOD], "KP_PERIOD");
	strcpy(vkNames[SDL_SCANCODE_KP_EQUALS], "KP_EQUALS");
	for (SDL_Scancode key = SDL_SCANCODE_KP_1; key < SDL_SCANCODE_KP_0; ++key) {
		char tmp[8] = "KP_1";
		tmp[3] = '1' + (key - SDL_SCANCODE_KP_1);
		strcpy(vkNames[key], tmp);
	}

	for (SDL_Scancode key = SDL_SCANCODE_LCTRL; key <= SDL_SCANCODE_RGUI; ++key) {
		inputGetScancodeName(key, vkNames[key], sizeof(vkNames[key]));
	}

	// mouse names
	for (u32 vk = VK_MOUSE_BEGIN; vk < VK_JOY1_BEGIN; ++vk) {
		strcpy(vkNames[vk], vkMouseNames[vk - VK_MOUSE_BEGIN]);
	}

	// joystick names
	for (u32 vk = VK_JOY1_BEGIN; vk < VK_TOTAL_COUNT; ++vk) {
		const u32 jidx = (vk - VK_JOY1_BEGIN) / INPUT_MAX_CONTROLLER_BUTTONS;
		const u32 jbtn = (vk - VK_JOY1_BEGIN) % INPUT_MAX_CONTROLLER_BUTTONS;
		strcpy(vkNames[vk], vkJoyNames[jbtn]);
		vkNames[vk][3] = '1' + jidx;
	}
}

void inputSaveBinds(void)
{
	char *bindstr;

	for (s32 i = 0; i < MAXCONTROLLERS; ++i) {
		for (u32 ck = 0; ck < CK_TOTAL_COUNT; ++ck) {
			bindstr = bindStrs[i][ck];
			bindstr[0] = '\0';
			for (s32 b = 0; b < INPUT_MAX_BINDS; ++b) {
				if (binds[i][ck][b]) {
					if (b) {
						strncat(bindstr, ", ", MAX_BIND_STR - 1);
					}
					strncat(bindstr, inputGetKeyName(binds[i][ck][b]), MAX_BIND_STR - 1);
				}
			}
			if (!bindstr[0]) {
				strcpy(bindstr, "NONE");
			}
		}
	}
}

static inline void inputParseBindString(const s32 ctrl, const u32 ck, char *bindstr)
{
	if (!bindstr[0]) {
		// empty string, keep defaults
		return;
	}

	// unbind all first
	memset(binds[ctrl][ck], 0, sizeof(binds[ctrl][ck]));

	if (!strcasecmp(bindstr, "NONE")) {
		// explicitly nothing bound
		return;
	}

	const char *tok = strtok(bindstr, ", ");
	while (tok) {
		if (tok[0]) {
			const s32 vk = inputGetKeyByName(tok);
			if (vk > 0) {
				inputKeyBind(ctrl, ck, -1, vk);
			}
		}
		tok = strtok(NULL, ", ");
	}
}

static inline void inputLoadBinds(void)
{
	for (s32 i = 0; i < MAXCONTROLLERS; ++i) {
		for (u32 ck = 0; ck < CK_TOTAL_COUNT; ++ck) {
			inputParseBindString(i, ck, bindStrs[i][ck]);
		}
	}
}

s32 inputInit(void)
{
	// Set SDL hints before initializing the controller subsystem.
	if (useHIDAPI) {
#if SDL_VERSION_ATLEAST(2, 0, 12)
		SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_GAMECUBE, "1");
#endif
#if SDL_VERSION_ATLEAST(2, 0, 14)
		SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS5, "1");
#endif
#if SDL_VERSION_ATLEAST(2, 0, 22)
		SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI, "1");
		SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS4, "1");
		SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_SWITCH, "1");
		SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_JOY_CONS, "1");
		// the two hints below enable Rumble and Motion Sensor for PS4/5 pads connected via bluetooth
		SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS4_RUMBLE, "1");
		SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS5_RUMBLE, "1");
		SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_STEAM, "1");
#endif
#if SDL_VERSION_ATLEAST(2, 23, 2)
		SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_COMBINE_JOY_CONS, "1");
#endif
#if SDL_VERSION_ATLEAST(2, 25, 1)
		SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS3, "1");
#endif
#if SDL_VERSION_ATLEAST(2, 26, 0)
		SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_WII, "1");
#endif
#if SDL_VERSION_ATLEAST(2, 30, 0)
		SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_STEAMDECK, "1");
#endif
	}
	if (useRawInput) {
		SDL_SetHint(SDL_HINT_JOYSTICK_RAWINPUT, "1");
		SDL_SetHint(SDL_HINT_JOYSTICK_RAWINPUT_CORRELATE_XINPUT, "1");
	}

	if (!SDL_WasInit(SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC)) {
		SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC);
	}

	// try to load controller db from an external file in the save folder
	if (fsFileSize("$S/" CONTROLLERDB_FNAME)) {
		const char *dbpath = fsFullPath("$S/" CONTROLLERDB_FNAME);
		const s32 dbcount = SDL_GameControllerAddMappingsFromFile(dbpath);
		if (dbcount >= 0) {
			sysLogPrintf(LOG_NOTE, "input: added %d controller mappings from %s", dbcount, dbpath);
		}
	}

	inputInitAllControllers();

	// since the main event loop is elsewhere, we can receive some events we need using a watcher
	SDL_AddEventWatch(inputEventFilter, NULL);

	inputInitKeyNames();

	for (s32 i = 0; i < INPUT_MAX_CONTROLLERS; ++i) {
		inputSetDefaultKeyBinds(i, 0);
	}

	if (mouseLockMode != MLOCK_AUTO) {
		inputLockMouse(mouseLockMode);
	}

	// update the axis maps
	// NOTE: by default sticks get swapped for 1.2: "right stick" here means left stick on your controller
	for (s32 i = 0; i < INPUT_MAX_CONTROLLERS; ++i) {
		inputControllerSetSticksSwapped(i, padsCfg[i].swapSticks);
	}

	inputLoadBinds();

	// if GyroAdvanced is disabled: GyroSpeed/GyroAimSpeed will sync to whatever is higher between X and Y
	for (s32 i = 0; i < INPUT_MAX_CONTROLLERS; ++i) {
		if (!padsCfg[i].gyroAdvanced) {
			padsCfg[i].gyroSpeedX = padsCfg[i].gyroSpeedY = fmaxf(padsCfg[i].gyroSpeedX, padsCfg[i].gyroSpeedY);
			padsCfg[i].gyroAimSpeedX = padsCfg[i].gyroAimSpeedY = fmaxf(padsCfg[i].gyroAimSpeedX, padsCfg[i].gyroAimSpeedY);
		}
	}

	return connectedMask;
}

s32 inputBindPressed(const s32 idx, const u32 ck)
{
	for (s32 i = 0; i < INPUT_MAX_BINDS; ++i) {
		if (binds[idx][ck][i]) {
			if (inputKeyPressed(binds[idx][ck][i])) {
				return 1;
			}
		}
	}
	return 0;
}

static inline s32 inputAxisScale(s32 x, const s32 deadzone, const f32 scale)
{
	if (abs(x) < deadzone) {
		return 0;
	} else {
		// rescale to fit the non-deadzone range
		if (x < 0) {
			x += deadzone;
		} else {
			x -= deadzone;
		}
		x = x * 32768 / (32768 - deadzone);
		// scale with sensitivity
		x *= scale;
		return (x > 32767) ? 32767 : ((x < -32768) ? -32768 : x);
	}
}

s32 inputReadController(s32 idx, OSContPad *npad)
{
	if (idx < 0 || idx >= INPUT_MAX_CONTROLLERS  || !npad) {
		return -1;
	}

	npad->button = 0;

	if (textInput) {
		npad->stick_x = 0;
		npad->stick_y = 0;
		npad->rstick_x = 0;
		npad->rstick_y = 0;
		return 0;
	}

	for (u32 i = 0; i < CONT_NUM_BUTTONS; ++i) {
		if (inputBindPressed(idx, i)) {
			npad->button |= 1U << i;
		}
	}

	const s32 xdiff = (inputBindPressed(idx, CK_STICK_XPOS) - inputBindPressed(idx, CK_STICK_XNEG));
	const s32 ydiff = (inputBindPressed(idx, CK_STICK_YPOS) - inputBindPressed(idx, CK_STICK_YNEG));
	npad->stick_x = xdiff < 0 ? -0x80 : (xdiff > 0 ? 0x7F : 0);
	npad->stick_y = ydiff < 0 ? -0x80 : (ydiff > 0 ? 0x7F : 0);

	const struct controllercfg *cfg = &padsCfg[idx];

	if (cfg->cancelCButtons) {
		// opposite C buttons cancel each other out
		if ((npad->button & (L_CBUTTONS | R_CBUTTONS)) == (L_CBUTTONS | R_CBUTTONS)) {
			npad->button &= ~(L_CBUTTONS | R_CBUTTONS);
		}
		if ((npad->button & (U_CBUTTONS | D_CBUTTONS)) == (U_CBUTTONS | D_CBUTTONS)) {
			npad->button &= ~(U_CBUTTONS | D_CBUTTONS);
		}
	}

	if (!pads[idx]) {
		return 0;
	}

	s32 leftX = SDL_GameControllerGetAxis(pads[idx], cfg->axisMap[0][0]);
	s32 leftY = SDL_GameControllerGetAxis(pads[idx], cfg->axisMap[0][1]);
	s32 rightX = SDL_GameControllerGetAxis(pads[idx], cfg->axisMap[1][0]);
	s32 rightY = SDL_GameControllerGetAxis(pads[idx], cfg->axisMap[1][1]);

	leftX = inputAxisScale(leftX, cfg->deadzone[cfg->axisMap[0][0]], cfg->sens[cfg->axisMap[0][0]]);
	leftY = inputAxisScale(leftY, cfg->deadzone[cfg->axisMap[0][1]], cfg->sens[cfg->axisMap[0][1]]);
	rightX = inputAxisScale(rightX, cfg->deadzone[cfg->axisMap[1][0]], cfg->sens[cfg->axisMap[1][0]]);
	rightY = inputAxisScale(rightY, cfg->deadzone[cfg->axisMap[1][1]], cfg->sens[cfg->axisMap[1][1]]);

	if (!npad->stick_x && leftX) {
		npad->stick_x = leftX / 0x100;
	}

	s32 stickY = -leftY / 0x100;
	if (!npad->stick_y && stickY) {
		npad->stick_y = (stickY == 128) ? 127 : stickY;
	}

	if (cfg->stickCButtons) {
		// rstick emulates C buttons
		if (rightX < -0x4000) npad->button |= L_CBUTTONS;
		if (rightX > +0x4000) npad->button |= R_CBUTTONS;
		if (rightY < -0x4000) npad->button |= U_CBUTTONS;
		if (rightY > +0x4000) npad->button |= D_CBUTTONS;
		npad->rstick_x = 0;
		npad->rstick_y = 0;
	} else {
		// rstick is an analog input
		if (rightX) {
			npad->rstick_x = rightX / 0x100;
		}
		s32 rStickY = -rightY / 0x100;
		if (rStickY) {
			npad->rstick_y = (rStickY == 128) ? 127 : rStickY;
		}
	}

	return 0;
}

static inline void inputUpdateMouse(void)
{
	s32 mx, my;
	mouseButtons = SDL_GetMouseState(&mx, &my);

	if (mouseWheel > 0) {
		mouseButtons |= WHEEL_UP_MASK;
	} else if (mouseWheel < 0) {
		mouseButtons |= WHEEL_DN_MASK;
	}

	mouseWheel = 0;

	s32 mdx = 0;
	s32 mdy = 0;
	SDL_GetRelativeMouseState(&mdx, &mdy);
	if (mouseLocked) {
		mouseDX = mdx;
		mouseDY = mdy;
	} else {
		mouseDX = mx - mouseX;
		mouseDY = my - mouseY;
	}

	mouseX = mx;
	mouseY = my;

	// if MLOCK_AUTO is enabled, disable cursor if mouse is unlocked
	// and we haven't moved it for a few seconds
	if (mouseLockMode == MLOCK_AUTO && !mouseLocked) {
		if (abs(mouseDX) > CURSOR_HIDE_THRESHOLD || abs(mouseDY) > CURSOR_HIDE_THRESHOLD) {
			if (!mouseShowCursor) {
				inputMouseShowCursor(1);
			}
		} else if (sysGetMicroseconds() > mouseCursorTime) {
			if (mouseShowCursor) {
				inputMouseShowCursor(0);
			}
		}
	}
}

s32 inputControllerMotionSensorsSupported(s32 cidx)
{
	if (cidx < 0 || cidx >= INPUT_MAX_CONTROLLERS) {
		return 0;
	}
	
	return padsCfg[cidx].gyroSensorActive;
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

static void inputProcessMotionSensorData(s32 cidx, float deltaTime, f32* deltaX, f32* deltaY, f32* deltaZ)
{
	if (!gpadMotion[cidx]) {
		sysLogPrintf(LOG_WARNING, "GamepadMotion instance missing for controller %d, gyro will not function", cidx);
		return;
	}

	if (!hasSensorData[cidx]) {
		return;
	}

	// Set calibration mode based on user preference and current context
	if (padsCfg[cidx].gyroAutoCalibration != GYRO_AUTOCALIBRATION_OFF && inputGyroAutoCalibrationModes(cidx)) {
		gmhSetCalibrationMode(gpadMotion[cidx], CALIBRATIONMODE_STILLNESS);
	} else {
		gmhSetCalibrationMode(gpadMotion[cidx], CALIBRATIONMODE_MANUAL);
	}

	// Feed data to GamepadMotionHelper
	gmhProcessMotion(gpadMotion[cidx],
		gyroData[cidx][0], gyroData[cidx][1], gyroData[cidx][2],
		accelData[cidx][0], accelData[cidx][1], accelData[cidx][2],
		deltaTime);

	// Get calibrated gyro output and apply axis mapping
	float calibratedGyro[3] = {0.f};
	gmhGetCalibratedGyro(gpadMotion[cidx], &calibratedGyro[0], &calibratedGyro[1], &calibratedGyro[2]);
	applyGyroAxisMapping(cidx, calibratedGyro, deltaX, deltaY, deltaZ);
}

static void inputApplyGyroProcessing(s32 cidx, f32* deltaX, f32* deltaY, f32* deltaZ)
{
	// Handle post-calibration jump prevention
	if (gyroCalibState[cidx].justFinishedCalibrating) {
		*deltaX = 0.f;
		*deltaY = 0.f;
		*deltaZ = 0.f;
		gyroCalibState[cidx].justFinishedCalibrating = false;
		return;
	}

	// Apply gyro processing pipeline
	applyGyroModifier(deltaX, deltaY, deltaZ, inputGetGyroModifier(cidx), cidx);
	applyGyroSmoothing(deltaX, deltaY, deltaZ, inputGetGyroSmoothing(cidx), cidx);
	applyGyroTightening(deltaX, deltaY, deltaZ, inputGyroGetTightening(cidx));
	applyGyroDeadzone(deltaX, deltaY, deltaZ, inputGyroGetDeadzone(cidx));
}

void inputUpdateGyro(s32 cidx)
{
	// Calculate frame time 
	float deltaTime = g_Vars.diffframe60f / 60.0f;

	// Process sensor data
	f32 deltaX = 0.f, deltaY = 0.f, deltaZ = 0.f;
	inputProcessMotionSensorData(cidx, deltaTime, &deltaX, &deltaY, &deltaZ);

	// Apply gyro processing pipeline
	inputApplyGyroProcessing(cidx, &deltaX, &deltaY, &deltaZ);

	// Store processed gyro deltas
	gyroDeltaYaw[cidx] = deltaX;
	gyroDeltaPitch[cidx] = deltaY;
	gyroDeltaRoll[cidx] = deltaZ;
}

void inputUpdate(void)
{
	SDL_GameControllerUpdate();

	if (mouseEnabled) {
		inputUpdateMouse();
	}

	// Handle gyro calibration systems
	inputUpdateGyroCalibrationHandle();

	// Process gyro input
	for (s32 cidx = 0; cidx < INPUT_MAX_CONTROLLERS; ++cidx) {
		if (padsCfg[cidx].gyroEnabled && inputControllerMotionSensorsSupported(cidx)) {
			inputUpdateGyro(cidx);
		}
	}
}

s32 inputControllerConnected(s32 idx)
{
	if (idx < 0 || idx >= INPUT_MAX_CONTROLLERS) {
		return 0;
	}
	return pads[idx] || (connectedMask & (1 << idx));
}

s32 inputRumbleSupported(s32 idx)
{
	if (idx < 0 || idx >= INPUT_MAX_CONTROLLERS) {
		return 0;
	}
	return padsCfg[idx].rumbleOn;
}

void inputRumble(s32 idx, f32 strength, f32 time)
{
	if (idx < 0 || idx >= INPUT_MAX_CONTROLLERS || !pads[idx]) {
		return;
	}

	if (padsCfg[idx].rumbleScale <= 0.f) {
		return;
	}

	if (padsCfg[idx].rumbleOn) {
		strength *= padsCfg[idx].rumbleScale;
		if (strength <= 0.f) {
			strength = 0.f;
			time = 0.f;
		} else {
			strength *= 65535.f;
			time *= 1000.f;
		}
		SDL_GameControllerRumble(pads[idx], (u16)strength, (u16)strength, (u32)time);
	}
}

f32 inputRumbleGetStrength(s32 cidx)
{
	return padsCfg[cidx].rumbleScale;
}

void inputRumbleSetStrength(s32 cidx, f32 val)
{
	padsCfg[cidx].rumbleScale = val;
}

s32 inputControllerMask(void)
{
	return connectedMask;
}

s32 inputControllerGetSticksSwapped(s32 cidx)
{
	return padsCfg[cidx].swapSticks;
}

void inputControllerSetSticksSwapped(s32 cidx, s32 swapped)
{
	padsCfg[cidx].swapSticks = swapped;
	if (swapped) {
		padsCfg[cidx].axisMap[0][0] = SDL_CONTROLLER_AXIS_RIGHTX;
		padsCfg[cidx].axisMap[0][1] = SDL_CONTROLLER_AXIS_RIGHTY;
		padsCfg[cidx].axisMap[1][0] = SDL_CONTROLLER_AXIS_LEFTX;
		padsCfg[cidx].axisMap[1][1] = SDL_CONTROLLER_AXIS_LEFTY;
	} else {
		padsCfg[cidx].axisMap[0][0] = SDL_CONTROLLER_AXIS_LEFTX;
		padsCfg[cidx].axisMap[0][1] = SDL_CONTROLLER_AXIS_LEFTY;
		padsCfg[cidx].axisMap[1][0] = SDL_CONTROLLER_AXIS_RIGHTX;
		padsCfg[cidx].axisMap[1][1] = SDL_CONTROLLER_AXIS_RIGHTY;
	}
}

s32 inputControllerGetDualAnalog(s32 cidx)
{
	return !padsCfg[cidx].stickCButtons;
}

void inputControllerSetDualAnalog(s32 cidx, s32 enable)
{
	padsCfg[cidx].stickCButtons = !enable;
}

s32 inputControllerGetCancelCButtons(s32 cidx)
{
	return padsCfg[cidx].cancelCButtons;
}

void inputControllerSetCancelCButtons(s32 cidx, s32 cancel)
{
	padsCfg[cidx].cancelCButtons = cancel;
}

f32 inputControllerGetAxisScale(s32 cidx, s32 stick, s32 axis)
{
	return padsCfg[cidx].sens[stick * 2 + axis];
}

void inputControllerSetAxisScale(s32 cidx, s32 stick, s32 axis, f32 value)
{
	padsCfg[cidx].sens[stick * 2 + axis] = value;
}

f32 inputControllerGetAxisDeadzone(s32 cidx, s32 stick, s32 axis)
{
	return (f32)padsCfg[cidx].deadzone[stick * 2 + axis] / 32767.f;
}

void inputControllerSetAxisDeadzone(s32 cidx, s32 stick, s32 axis, f32 value)
{
	padsCfg[cidx].deadzone[stick * 2 + axis] = value * 32767.f;
}

s32 inputGetConnectedControllers(s32 *out)
{
	s32 count = 0;

	for (s32 jidx = 0; jidx < numJoysticks; ++jidx) {
		if (SDL_IsGameController(jidx)) {
			if (out && count < INPUT_MAX_CONNECTED_CONTROLLERS) {
				out[count] = SDL_JoystickGetDeviceInstanceID(jidx);
			}
			++count;
		}
	}

	return count;
}

s32 inputGetAssignedControllerId(s32 cidx)
{
	if (cidx < 0 || cidx >= INPUT_MAX_CONTROLLERS) {
		return -1;
	}

	if (pads[cidx] == NULL) {
		return -1;
	}

	return inputControllerGetId(pads[cidx]);
}

const char *inputGetConnectedControllerName(s32 id)
{
	static char fullName[256];

	if (id < 0) {
		return "Invalid";
	}

	const s32 jidx = inputDeviceIndexFromId(id);
	if (jidx < 0) {
		return "Invalid";
	}

	const char *name = SDL_GameControllerNameForIndex(jidx);
	if (!name || !name[0]) {
		name = "Unnamed Controller";
	}

	snprintf(fullName, sizeof(fullName), "%d: %s", jidx, name);

	// replace non-ascii chars with spaces
	for (char *p = fullName; *p; ++p) {
		if ((u32)*p >= 0x7f) {
			*p = ' ';
		}
	}

	return fullName;
}

s32 inputAssignController(s32 cidx, s32 id)
{
	if (cidx < 0 || cidx >= INPUT_MAX_CONTROLLERS) {
		return 0;
	}

	if (id < 0) {
		// close current controller, if any
		if (pads[cidx]) {
			inputCloseController(cidx);
			return 1;
		}
		return 0;
	}

	const s32 jidx = inputDeviceIndexFromId(id);
	if (jidx < 0 || jidx >= SDL_NumJoysticks() || !SDL_IsGameController(jidx)) {
		return 0;
	}

	// try to unassign any other instances of this controller
	for (s32 i = 0; i < INPUT_MAX_CONTROLLERS; ++i) {
		if (pads[i] && inputControllerGetId(pads[i]) == id) {
			inputCloseController(i);
			pads[i] = NULL;
			padsCfg[i].deviceIndex = -1;
		}
	}

	SDL_GameController *newpad = SDL_GameControllerOpen(jidx);
	if (!newpad) {
		return 0;
	}

	if (pads[cidx]) {
		inputCloseController(cidx);
	}

	pads[cidx] = newpad;
	inputInitController(cidx, id);

	return 1;
}

void inputKeyBind(s32 idx, u32 ck, s32 bind, u32 vk)
{
	if (idx < 0 || idx >= INPUT_MAX_CONTROLLERS || bind >= INPUT_MAX_BINDS || ck >= CK_TOTAL_COUNT) {
		return;
	}

	if (bind < 0) {
		for (s32 i = 0; i < INPUT_MAX_BINDS; ++i) {
			if (binds[idx][ck][i] == 0) {
				bind = i;
				break;
			}
		}
		if (bind < 0) {
			bind = INPUT_MAX_BINDS - 1; // just overwrite last
		}
	}

	binds[idx][ck][bind] = vk;
}

const u32 *inputKeyGetBinds(s32 idx, u32 ck)
{
	if (idx < 0 || idx >= INPUT_MAX_CONTROLLERS || ck >= CK_TOTAL_COUNT) {
		return NULL;
	}
	return binds[idx][ck];
}

s32 inputKeyPressed(u32 vk)
{
	if (vk >= VK_KEYBOARD_BEGIN && vk < VK_MOUSE_BEGIN) {
		const u8 *state = SDL_GetKeyboardState(NULL);
		return state[vk - VK_KEYBOARD_BEGIN];
	}

	if (vk >= VK_MOUSE_BEGIN && vk < VK_JOY_BEGIN) {
		return (mouseButtons & SDL_BUTTON(vk - VK_MOUSE_BEGIN + 1)) != 0;
	}

	if (vk >= VK_JOY_BEGIN && vk < VK_TOTAL_COUNT) {
		vk -= VK_JOY_BEGIN;
		const s32 idx = vk / INPUT_MAX_CONTROLLER_BUTTONS;
		if (idx < 0 || idx >= INPUT_MAX_CONTROLLERS || !pads[idx]) {
			return 0;
		}
		vk = vk % INPUT_MAX_CONTROLLER_BUTTONS;
		// triggers
		if (vk == 30 || vk == 31) {
			const s32 trig = SDL_CONTROLLER_AXIS_TRIGGERLEFT + vk - 30;
			return SDL_GameControllerGetAxis(pads[idx], trig) > TRIG_THRESHOLD;
		}
		return SDL_GameControllerGetButton(pads[idx], vk);
	}

	return 0;
}

s32 inputKeyJustPressed(u32 vk)
{
	const s8 pressed = inputKeyPressed(vk);
	const s32 result = pressed && !vkPrevState[vk];
	vkPrevState[vk] = pressed;
	return result;
}

static inline u32 inputContToContKey(const u32 cont)
{
	if (cont == 0) {
		return 0;
	}
	// just a log2 to convert CONT_* to their indices
	return 32 - __builtin_clz(cont - 1);
}

s32 inputButtonPressed(s32 idx, u32 contbtn)
{
	if (idx < 0 || idx >= INPUT_MAX_CONTROLLERS) {
		return 0;
	}

	return inputBindPressed(idx, inputContToContKey(contbtn));
}


void inputLockMouse(s32 lock)
{
	mouseLocked = !!lock;
	SDL_SetRelativeMouseMode(mouseLocked);
}

s32 inputMouseIsLocked(void)
{
	return mouseLocked;
}

s32 inputMouseGetPosition(s32 *x, s32 *y)
{
	if (x) *x = mouseX * videoGetNativeWidth() / videoGetWidth();
	if (y) *y = mouseY * videoGetNativeHeight() / videoGetHeight();
	return (mouseDX != 0 || mouseDY != 0);
}

void inputMouseGetRawDelta(s32 *dx, s32 *dy)
{
	if (dx) *dx = mouseDX;
	if (dy) *dy = mouseDY;
}

void inputMouseGetScaledDelta(f32* dx, f32* dy)
{
	f32 mdx = 0.f, mdy = 0.f;
	if (mouseLocked) {
		mdx = mouseDX * (0.022f / 3.5f) * mouseSensX;
		mdy = mouseDY * (0.022f / 3.5f) * mouseSensY;
	}
	if (dx) *dx = mdx;
	if (dy) *dy = mdy;
}

void inputMouseGetAbsScaledDelta(f32* dx, f32* dy)
{
	f32 mdx = 0.f, mdy = 0.f;
	if (mouseLocked) {
		mdx = mouseDX * (0.022f / 3.5f) * fabsf(mouseSensX);
		mdy = mouseDY * (0.022f / 3.5f) * fabsf(mouseSensY);
	}
	if (dx) *dx = mdx;
	if (dy) *dy = mdy;
}

void inputMouseGetSpeed(f32 *x, f32 *y)
{
	*x = mouseSensX;
	*y = mouseSensY;
}

void inputMouseSetSpeed(f32 x, f32 y)
{
	mouseSensX = x;
	mouseSensY = y;
}

s32 inputMouseIsEnabled(void)
{
	return mouseEnabled;
}

void inputMouseEnable(s32 enabled)
{
	mouseEnabled = !!enabled;
	if (!mouseEnabled && mouseLockMode != MLOCK_ON && mouseLocked) {
		inputLockMouse(0);
	}
}

s32 inputAutoLockMouse(s32 wantlock)
{
	if (mouseEnabled && mouseLockMode == MLOCK_AUTO) {
		inputLockMouse(wantlock);
		return 1;
	}
	return 0;
}

void inputMouseShowCursor(s32 show)
{
	mouseShowCursor = !!show;
	SDL_ShowCursor(mouseShowCursor);
	if (show) {
		mouseCursorTime = sysGetMicroseconds() + CURSOR_HIDE_TIME;
	}
}

s32 inputGetMouseLockMode(void)
{
	return mouseLockMode;
}

void inputSetMouseLockMode(s32 lockmode)
{
	mouseLockMode = lockmode;
	if (lockmode == MLOCK_ON) {
		inputLockMouse(1);
	} else {
		inputLockMouse(0);
	}
}

s32 inputGyroIsEnabled(s32 cidx)
{
	if (cidx < 0 || cidx >= INPUT_MAX_CONTROLLERS) {
		return 0;
	}
	
	return padsCfg[cidx].gyroEnabled && inputControllerMotionSensorsSupported(cidx);
}

void inputGyroEnable(s32 cidx, s32 enabled)
{
	if (cidx < 0 || cidx >= INPUT_MAX_CONTROLLERS) {
		return;
	}
	
	padsCfg[cidx].gyroEnabled = (enabled != 0);
}

s32 inputGyroGetAdvanced(s32 cidx)
{
    return padsCfg[cidx].gyroAdvanced;
}

void inputGyroSetAdvanced(s32 cidx, s32 advanced)
{
    padsCfg[cidx].gyroAdvanced = advanced;
}

s32 inputGyroGetAxisMode(s32 cidx)
{
    return padsCfg[cidx].gyroAxisMode;
}

void inputGyroSetAxisMode(s32 cidx, s32 mode)
{
    padsCfg[cidx].gyroAxisMode = mode;
}

static void applyGyroAxisMapping(s32 cidx, float calibratedGyro[3], f32* deltaX, f32* deltaY, f32* deltaZ)
{
    if (!gpadMotion[cidx]) {
        *deltaX = *deltaY = *deltaZ = 0.f;
        return;
    }

    // Check if axis mode has changed and reset motion state to prevent bleedthrough
    static s32 previousAxisMode[INPUT_MAX_CONTROLLERS] = {-1, -1, -1, -1};
    static bool initialized[INPUT_MAX_CONTROLLERS] = {false, false, false, false};
    
    s32 currentAxisMode = inputGyroGetAxisMode(cidx);
    
    // Initialize on first use without triggering reset
    if (!initialized[cidx]) {
        previousAxisMode[cidx] = currentAxisMode;
        initialized[cidx] = true;
    }
    
    // Only reset if mode actually changed
    if (previousAxisMode[cidx] != currentAxisMode) {
        gmhResetMotion(gpadMotion[cidx]);
        previousAxisMode[cidx] = currentAxisMode;
    }

    switch (inputGyroGetAxisMode(cidx)) {
	case GYRO_YAW:
		*deltaX = -calibratedGyro[1];
		*deltaY = -calibratedGyro[0];
		break;
	case GYRO_ROLL:
		*deltaX = calibratedGyro[2];
		*deltaY = -calibratedGyro[0];
		break;
    case GYRO_LOCAL: {
        float orientation_w = 0.f;
        float orientation[3] = {0.f};
        gmhGetOrientation(gpadMotion[cidx], &orientation_w, &orientation[0], &orientation[1], &orientation[2]);
        *deltaX = -calibratedGyro[1] + (calibratedGyro[2] * 0.85f);
        *deltaY = -calibratedGyro[0];
        *deltaZ = calibratedGyro[2];
        break;
    }
    case GYRO_PLAYER: {
        float playerX = 0.f, playerY = 0.f;
        gmhGetPlayerSpaceGyro(gpadMotion[cidx], &playerX, &playerY, 1.41f);
        *deltaX = -playerY;
        *deltaY = -playerX;
        *deltaZ = calibratedGyro[2];
        break;
    }
    case GYRO_WORLD: {
        float worldX = 0.f, worldY = 0.f;
        gmhGetWorldSpaceGyro(gpadMotion[cidx], &worldX, &worldY, 0.125f);
        *deltaX = -worldY;
        *deltaY = -worldX;
        *deltaZ = 0.f;
        break;
    }
    default:
        *deltaX = *deltaY = *deltaZ = 0.f;
        break;
    }
}

s32 inputGetGyroAimMode(s32 cidx)
{
	return padsCfg[cidx].gyroAimMode;
}

void inputSetGyroAimMode(s32 cidx, s32 mode)
{
	padsCfg[cidx].gyroAimMode = mode;
}

void inputGyroGetRawDelta(s32 cidx, s32* dx, s32* dy, s32* dz)
{
	if (dx) *dx = (s32)gyroDeltaYaw[cidx];
	if (dy) *dy = (s32)gyroDeltaPitch[cidx];
	if (dz) *dz = (s32)gyroDeltaRoll[cidx];
}

void inputGyroGetScaledDelta(s32 cidx, f32* dx, f32* dy, f32* dz)
{
	if (!dx || !dy || !dz) return;

	f32 gdx = 0.f, gdy = 0.f, gdz = 0.f;

	if (padsCfg[cidx].gyroEnabled) {
		if (!isnan(gyroDeltaYaw[cidx]) && !isnan(gyroDeltaPitch[cidx]) && !isnan(gyroDeltaRoll[cidx])) {
			gdx = gyroDeltaYaw[cidx] * padsCfg[cidx].gyroSpeedX;
			gdy = gyroDeltaPitch[cidx] * padsCfg[cidx].gyroSpeedY;
			gdz = gyroDeltaRoll[cidx] * padsCfg[cidx].gyroSpeedX;
		}
	}

	*dx = gdx;
	*dy = gdy;
	*dz = gdz;

	applyGyroInvert(cidx, dx, dy, false);
	applyGyroVHMixer(cidx, dx, dy);
}

void inputGyroGetSpeed(s32 cidx, f32* sensX, f32* sensY)
{
    if (sensX) *sensX = padsCfg[cidx].gyroSpeedX;
    if (sensY) *sensY = padsCfg[cidx].gyroSpeedY;
}

void inputGyroSetSpeed(s32 cidx, f32 sensX, f32 sensY)
{
    padsCfg[cidx].gyroSpeedX = sensX;
    padsCfg[cidx].gyroSpeedY = sensY;
}

void inputGyroGetScaledDeltaCrosshair(s32 cidx, f32* dx, f32* dy)
{
	f32 gdx = 0.f, gdy = 0.f;

	if (padsCfg[cidx].gyroEnabled) {
		gdx = gyroDeltaYaw[cidx] * (0.022f / 2.0f) * padsCfg[cidx].gyroAimSpeedX;
		gdy = gyroDeltaPitch[cidx] * (0.022f / 2.0f) * padsCfg[cidx].gyroAimSpeedY;
	}

	if (dx) *dx = gdx;
	if (dy) *dy = gdy;

	if (dx && dy) {
		applyGyroInvert(cidx, dx, dy, true);
		applyGyroVHMixer(cidx, dx, dy);
	}
}

void inputGyroGetAimSpeed(s32 cidx, f32* sensX, f32* sensY)
{
    if (sensX) *sensX = padsCfg[cidx].gyroAimSpeedX;
    if (sensY) *sensY = padsCfg[cidx].gyroAimSpeedY;
}

void inputGyroSetAimSpeed(s32 cidx, f32 sensX, f32 sensY)
{
    padsCfg[cidx].gyroAimSpeedX = sensX;
    padsCfg[cidx].gyroAimSpeedY = sensY;
}

static inline void applyGyroInvert(s32 cidx, f32* dx, f32* dy, bool useAimInvert) {
    if (useAimInvert) {
        if (padsCfg[cidx].gyroAimInvertX) *dx = -*dx;
        if (padsCfg[cidx].gyroAimInvertY) *dy = -*dy;
    } else {
        if (padsCfg[cidx].gyroInvertX) *dx = -*dx;
        if (padsCfg[cidx].gyroInvertY) *dy = -*dy;
    }
}

void inputGyroGetInvert(s32 cidx, s32* out_invertx, s32* out_inverty)
{
    if (out_invertx) *out_invertx = padsCfg[cidx].gyroInvertX;
    if (out_inverty) *out_inverty = padsCfg[cidx].gyroInvertY;
}

void inputGyroSetInvert(s32 cidx, s32 invertx, s32 inverty)
{
    padsCfg[cidx].gyroInvertX = invertx ? 1 : 0;
    padsCfg[cidx].gyroInvertY = inverty ? 1 : 0;
}

void inputGyroGetAimInvert(s32 cidx, s32* out_invertx, s32* out_inverty)
{
    if (out_invertx) *out_invertx = padsCfg[cidx].gyroAimInvertX;
    if (out_inverty) *out_inverty = padsCfg[cidx].gyroAimInvertY;
}

void inputGyroSetAimInvert(s32 cidx, s32 invertx, s32 inverty)
{
    padsCfg[cidx].gyroAimInvertX = invertx ? 1 : 0;
    padsCfg[cidx].gyroAimInvertY = inverty ? 1 : 0;
}

static inline void applyGyroVHMixer(s32 cidx, f32* dx, f32* dy) {
	// Skip X/Y Output Mixer when Advanced Mode is enabled
	if (padsCfg[cidx].gyroAdvanced) {
		return;
	}

	float mix = fminf(fmaxf(padsCfg[cidx].gyroVHMixer, -1.0f), 1.0f);

	float hScale = 1.0f - fmaxf(0.0f, mix);
	float vScale = 1.0f + fminf(0.0f, mix);

	*dx *= hScale;
	*dy *= vScale;
}

f32 inputGetGyroVHMixer(s32 cidx)
{
	return padsCfg[cidx].gyroVHMixer;
}

void inputSetGyroVHMixer(s32 cidx, f32 value)
{
	if (value < -1.0f) value = -1.0f;
	if (value > 1.0f) value = 1.0f;
	padsCfg[cidx].gyroVHMixer = value;
}

s32 inputGetGyroModifier(s32 cidx)
{
	return padsCfg[cidx].gyroModifier;
}

void inputSetGyroModifier(s32 cidx, s32 mode)
{
	padsCfg[cidx].gyroModifier = mode;
}

static void applyGyroModifier(f32* deltaX, f32* deltaY, f32* deltaZ, s32 activationMode, s32 idx) {
	static bool toggleState[INPUT_MAX_CONTROLLERS] = { true, true, true, true };
	static int prevGyroMod[INPUT_MAX_CONTROLLERS] = { 0 };

	const int modPressed = inputBindPressed(idx, CK_0080);
	const int justPressed = modPressed && !prevGyroMod[idx];
	prevGyroMod[idx] = modPressed;

	bool gyroActive = false;

	switch (activationMode) {
		case GYRO_ALWAYS_ON:
			gyroActive = true;
			break;
		case GYRO_TOGGLE:
			if (justPressed) {
				toggleState[idx] = !toggleState[idx];
			}
			gyroActive = toggleState[idx];
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

f32 inputGetGyroSmoothing(s32 cidx)
{
	return padsCfg[cidx].gyroSmoothing;
}

void inputSetGyroSmoothing(s32 cidx, f32 smoothing)
{
	if (smoothing < 0.0f) smoothing = 0.0f;
	if (smoothing > 1.0f) smoothing = 1.0f;
	padsCfg[cidx].gyroSmoothing = smoothing;
}

static void applyGyroSmoothing(f32* deltaX, f32* deltaY, f32* deltaZ, f32 smoothing, s32 cidx)
{
    if (!deltaX || !deltaY || !deltaZ || smoothing <= 0.0f) return;

	// Use a circular buffer to store recent gyro deltas for smoothing
    static f32 history[INPUT_MAX_CONTROLLERS][3][8] = {0};
    static s32 historyIndex[INPUT_MAX_CONTROLLERS] = {0};

    s32 windowSize = 1 + (s32)(smoothing * 3.0f);

    s32 idx = historyIndex[cidx];
    history[cidx][0][idx] = *deltaX;
    history[cidx][1][idx] = *deltaY;
    history[cidx][2][idx] = *deltaZ;
    
    f32 avgX = 0.f, avgY = 0.f, avgZ = 0.f;
    for (s32 i = 0; i < windowSize; ++i) {
        s32 sampleIdx = (idx - i + 8) % 8;
        avgX += history[cidx][0][sampleIdx];
        avgY += history[cidx][1][sampleIdx];
        avgZ += history[cidx][2][sampleIdx];
    }
    
    avgX /= windowSize;
    avgY /= windowSize;
    avgZ /= windowSize;

    // Simple blend: 70% smoothed + 30% raw
    f32 blend = smoothing * 0.7f;
    *deltaX = avgX * blend + (*deltaX) * (1.0f - blend);
    *deltaY = avgY * blend + (*deltaY) * (1.0f - blend);
    *deltaZ = avgZ * blend + (*deltaZ) * (1.0f - blend);
    
    historyIndex[cidx] = (idx + 1) % 8;
}

f32 inputGyroGetTightening(s32 cidx)
{
    return padsCfg[cidx].gyroTightening;
}

void inputGyroSetTightening(s32 cidx, f32 tightening)
{
	if (tightening < 0.f) tightening = 0.f;
	if (tightening > 1.f) tightening = 1.f;
	padsCfg[cidx].gyroTightening = tightening;
}

static void applyGyroTightening(f32* dx, f32* dy, f32* dz, f32 tightening)
{
    if (!dx || !dy || !dz || tightening <= 0.0f) return;

    f32 mag = sqrtf((*dx) * (*dx) + (*dy) * (*dy) + (*dz) * (*dz));
    if (mag > 0.f) {
        f32 scale = 1.0f;
        
        // Soft tiered scaling with multiple breakpoints for smoother transitions
		// based on http://gyrowiki.jibbsmart.com/blog:tight-and-smooth:soft-tiered-smoothing
        if (mag < tightening) {
            f32 ratio = mag / tightening;
            
            if (ratio < 0.25f) {
                // Very small movements: moderate reduction instead of heavy
                scale = ratio * ratio * 4.0f;
            } else if (ratio < 0.5f) {
                // Small movements: gentler reduction with quadratic curve
                f32 adjustedRatio = (ratio - 0.25f) / 0.25f;
                scale = 0.25f + adjustedRatio * adjustedRatio * 0.5f;
            } else if (ratio < 0.75f) {
                // Medium-small movements: minimal reduction with linear interpolation
                f32 adjustedRatio = (ratio - 0.5f) / 0.25f;
                scale = 0.75f + adjustedRatio * 0.2f;
            } else {
                // Near-threshold movements: very minimal reduction
                f32 adjustedRatio = (ratio - 0.75f) / 0.25f;
                scale = 0.95f + adjustedRatio * adjustedRatio * 0.05f;
            }
        }
        
        *dx *= scale;
        *dy *= scale;
        *dz *= scale;
    }
}

f32 inputGyroGetDeadzone(s32 cidx)
{
    return padsCfg[cidx].gyroDeadzone;
}

void inputGyroSetDeadzone(s32 cidx, f32 deadzone)
{
    if (deadzone < 0.f) deadzone = 0.f;
    if (deadzone > 1.f) deadzone = 1.f;

    if (deadzone > 0.f && deadzone < 0.01f) deadzone = 0.01f;
    padsCfg[cidx].gyroDeadzone = deadzone;
}

static void applyGyroDeadzone(f32* dx, f32* dy, f32* dz, f32 deadzone)
{
    if (deadzone <= 0.f || !dx || !dy || !dz) return;

    f32 mag = sqrtf((*dx) * (*dx) + (*dy) * (*dy) + (*dz) * (*dz));
    if (mag > 0.f) {
        f32 scale = 1.0f;
        
        if (mag <= deadzone) {
            // Complete deadzone - no output
            scale = 0.0f;
        } else {
            // Gentle exit transition zone
            f32 exitZone = deadzone * 0.2f; // 20% of deadzone as exit zone
            f32 exitThreshold = deadzone + exitZone;
            
            if (mag < exitThreshold) {
                // Smooth transition in exit zone
                f32 exitRatio = (mag - deadzone) / exitZone;
                scale = exitRatio * exitRatio; // Quadratic for smooth ramp-up
            }
        }
        
        *dx *= scale;
        *dy *= scale;
        *dz *= scale;
    }
}

static void inputUpdateGyroCalibrationHandle(void)
{
	// Process calibration for all controllers
	for (s32 cidx = 0; cidx < INPUT_MAX_CONTROLLERS; ++cidx) {
		if (!padsCfg[cidx].gyroEnabled || !inputControllerMotionSensorsSupported(cidx)) {
			continue;
		}

		// Handle manual calibration input
		inputUpdateGyroManualCalibration(cidx);

		// Handle auto-calibration if enabled
		if (padsCfg[cidx].gyroAutoCalibration != GYRO_AUTOCALIBRATION_OFF) {
			inputUpdateGyroAutoCalibration(cidx);
		}
	}
}

// detect the motion sensor's noise threshold for additional safety-net while auto-calibrating for Stationary mode
// based on SDL TestController's motion sensor noise filter
static bool inputIsControllerSensorNoiseThreshold(s32 cidx)
{
	static float prevGyro[INPUT_MAX_CONTROLLERS][3] = {{0.f}};
	static float prevAccel[INPUT_MAX_CONTROLLERS][3] = {{0.f}};
	static bool firstRun[INPUT_MAX_CONTROLLERS] = {true, true, true, true};
	
	if (!gpadMotion[cidx]) {
		return false;
	}
	
	if (!hasSensorData[cidx]) {
		return false;
	}
	
	// Check per-axis gyro thresholds
	for (s32 i = 0; i < 3; ++i) {
		if (fabsf(gyroData[cidx][i]) > GYRO_NOISE_THRESHOLD) {
			return false;
		}
	}
	
	// Check total gyro magnitude
	float gyroMagnitude = sqrtf(
		gyroData[cidx][0] * gyroData[cidx][0] + 
		gyroData[cidx][1] * gyroData[cidx][1] + 
		gyroData[cidx][2] * gyroData[cidx][2]
	);
	if (gyroMagnitude > GYRO_NOISE_THRESHOLD) {
		return false;
	}
	
	if (!firstRun[cidx]) {
		// Check gyro rate of change
		for (s32 i = 0; i < 3; ++i) {
			float delta = fabsf(gyroData[cidx][i] - prevGyro[cidx][i]);
			if (delta > GYRO_RATE_THRESHOLD) {
				for (s32 j = 0; j < 3; ++j) {
					prevGyro[cidx][j] = gyroData[cidx][j];
					prevAccel[cidx][j] = accelData[cidx][j];
				}
				return false;
			}
		}
		
		// Check accelerometer delta
		float accelDelta[3];
		accelDelta[0] = accelData[cidx][0] - prevAccel[cidx][0];
		accelDelta[1] = accelData[cidx][1] - prevAccel[cidx][1];
		accelDelta[2] = accelData[cidx][2] - prevAccel[cidx][2];
		
		float accelDeltaMagnitude = sqrtf(
			accelDelta[0] * accelDelta[0] + 
			accelDelta[1] * accelDelta[1] + 
			accelDelta[2] * accelDelta[2]
		);
		
		if (accelDeltaMagnitude > ACCEL_DELTA_THRESHOLD) {
			for (s32 j = 0; j < 3; ++j) {
				prevGyro[cidx][j] = gyroData[cidx][j];
				prevAccel[cidx][j] = accelData[cidx][j];
			}
			return false;
		}
	}
	
	for (s32 i = 0; i < 3; ++i) {
		prevGyro[cidx][i] = gyroData[cidx][i];
		prevAccel[cidx][i] = accelData[cidx][i];
	}
	firstRun[cidx] = false;
	
	float accelMagnitude = sqrtf(
		accelData[cidx][0] * accelData[cidx][0] + 
		accelData[cidx][1] * accelData[cidx][1] + 
		accelData[cidx][2] * accelData[cidx][2]
	);
	
	return fabsf(accelMagnitude - 1.0f) < ACCEL_GRAVITY_TOLERANCE;
}

// Check if auto-calibration should be active based on the current mode and game state
static bool inputGyroAutoCalibrationModes(s32 cidx)
{
	s32 autoCalibMode = padsCfg[cidx].gyroAutoCalibration;
	
	switch (autoCalibMode) {
		case GYRO_AUTOCALIBRATION_OFF: 
			return false;
		case GYRO_AUTOCALIBRATION_MENU_ONLY:
			return g_MenuData.isdialogopen;
		case GYRO_AUTOCALIBRATION_STATIONARY:
			return true;
		case GYRO_AUTOCALIBRATION_ALWAYS:
			return true;
		default:
			return false;
	}
}

void inputGyroCalibration(s32 cidx, GyroCalibrationOp op, float* out_confidence, int* out_steady)
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
			if (out_steady) *out_steady = gmhGetAutoCalibrationIsSteady(gpadMotion[cidx]);
		}
		break;
	case GYRO_CALIB_AUTO:
		inputUpdateGyroAutoCalibration(cidx);
		break;
	}
}

static void inputUpdateGyroAutoCalibration(s32 cidx)
{
	if (!gpadMotion[cidx] || padsCfg[cidx].gyroAutoCalibration == GYRO_AUTOCALIBRATION_OFF) {
		return;
	}

	if (gyroCalibState[cidx].manualCalibActive && !inputIsGyroCalibrationBlocked(cidx)) {
		return;
	}
	
	if (inputIsMenuGyroCalibrationActive(cidx)) {
		gmhPauseContinuousCalibration(gpadMotion[cidx]);
		return;
	}

	GyroCalibState *state = &gyroCalibState[cidx];
	const s32 mode = padsCfg[cidx].gyroAutoCalibration;
	const Uint32 now = SDL_GetTicks();
	const float confidence = gmhGetAutoCalibrationConfidence(gpadMotion[cidx]);
	const bool stillness = gmhGetAutoCalibrationIsSteady(gpadMotion[cidx]);
	
	static s32 previousMode[INPUT_MAX_CONTROLLERS] = {-1, -1, -1, -1};
	static bool alwaysModeActive[INPUT_MAX_CONTROLLERS] = {false};
	static bool stationaryModeActive[INPUT_MAX_CONTROLLERS] = {false};
	
	// Pause and reset MENU_ONLY mode when not in active state
	if (!inputGyroAutoCalibrationModes(cidx)) {
		gmhPauseContinuousCalibration(gpadMotion[cidx]);
		if (mode == GYRO_AUTOCALIBRATION_MENU_ONLY) {
			if (state->isCalibrating) {
				gmhResetContinuousCalibration(gpadMotion[cidx]);
				state->isCalibrating = false;
				state->autoCalibStartTime = 0;
			}
			stationaryModeActive[cidx] = false;
		}
		return;
	}
	
	// Reset state if mode changed
	if (previousMode[cidx] != mode) {
		previousMode[cidx] = mode;
		gmhPauseContinuousCalibration(gpadMotion[cidx]);
		alwaysModeActive[cidx] = false;
		stationaryModeActive[cidx] = false;
		state->isCalibrating = false;
		state->wasStill = false;
		state->lastAutoCalibTime = 0;
		state->autoCalibStartTime = 0;
		state->justFinishedCalibrating = false;
	}
	
	// ALWAYS MODE: Will attempt to calibrate continuously whenever possible
	if (mode == GYRO_AUTOCALIBRATION_ALWAYS) {
		const Uint32 STARTUP_DELAY = 120;          	 // Grace period after becoming still
		const Uint32 INTERVAL = 700;                 // How often to retry calibration
		const float CONFIDENCE_THRESHOLD = 0.50f;    // Only recalibrate if below 50%
		
		if (!alwaysModeActive[cidx]) {
			state->wasStill = false;
			state->lastAutoCalibTime = 0;
			state->isCalibrating = false;
			alwaysModeActive[cidx] = true;
		}
		
		if (stillness) {
			if (!state->wasStill) {
				state->lastAutoCalibTime = now - STARTUP_DELAY;
			}
			
			if (now - state->lastAutoCalibTime >= INTERVAL && !state->isCalibrating) {
				if (confidence < CONFIDENCE_THRESHOLD) {
					gmhStartContinuousCalibration(gpadMotion[cidx]);
					state->lastAutoCalibTime = now;
					state->isCalibrating = true;
				} else {
					state->lastAutoCalibTime = now;
				}
			}
		}
		else {
			if (state->wasStill) {
				gmhPauseContinuousCalibration(gpadMotion[cidx]);
				state->isCalibrating = false;
			}
		}
		
		state->wasStill = stillness;
		return;
	}
	
	// STATIONARY/MENU_ONLY MODE: Calibrate when controller is placed on flat surface
	if (mode == GYRO_AUTOCALIBRATION_STATIONARY || mode == GYRO_AUTOCALIBRATION_MENU_ONLY) {
		const Uint32 STARTUP_DELAY = 2500;           // Initial delay before first calibration
		const Uint32 CALIBRATION_DURATION = 1200;    // How long to run calibration
		const Uint32 COOLDOWN = 10000;               // Cooldown after calibration finishes
		const float CONFIDENCE_THRESHOLD_MENU_ONLY = 0.80f;  // Only recalibrate if below 80% for MENU_ONLY mode
		
		if (!stationaryModeActive[cidx]) {
			state->wasStill = false;
			state->lastAutoCalibTime = 0;
			state->autoCalibStartTime = 0;
			state->isCalibrating = false;
			stationaryModeActive[cidx] = true;
		}
		
		const bool isStationary = stillness && inputIsControllerSensorNoiseThreshold(cidx);
		const Uint32 timeSinceLastEvent = now - state->lastAutoCalibTime;
		const Uint32 delayNeeded = (state->autoCalibStartTime == 0) ? STARTUP_DELAY : COOLDOWN;
		const bool timingConditionsMet = !state->isCalibrating && timeSinceLastEvent >= delayNeeded;
		
		if (isStationary) {
			// Reset last auto-calib time when movement is detected
			if (!state->wasStill) {
				if (state->lastAutoCalibTime == 0) {
					state->lastAutoCalibTime = now;
				}
			}
			
			bool shouldCalibrate = false;

			// MENU_ONLY mode uses confidence check to prevent accidental calibrations
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
				state->isCalibrating = true;
			}
			
			if (state->isCalibrating && (now - state->autoCalibStartTime >= CALIBRATION_DURATION)) {
				gmhPauseContinuousCalibration(gpadMotion[cidx]);
				inputGyroCalibrationFinished(cidx, true);
				state->isCalibrating = false;
				state->lastAutoCalibTime = now;
			}
		}
		else {
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

s32 inputGyroGetAutoCalibration(s32 cidx)
{
	return (cidx >= 0 && cidx < INPUT_MAX_CONTROLLERS) ? padsCfg[cidx].gyroAutoCalibration : 0;
}

void inputGyroSetAutoCalibration(s32 cidx, s32 enabled)
{
	if (cidx < 0 || cidx >= INPUT_MAX_CONTROLLERS) return;

	s32 wasEnabled = padsCfg[cidx].gyroAutoCalibration;
	padsCfg[cidx].gyroAutoCalibration = enabled;

	if (wasEnabled != padsCfg[cidx].gyroAutoCalibration) {
		const char* modeNames[] = {"Disabled", "In Menus Only", "While Stationary", "Always"};
		const char* modeName = (enabled >= 0 && enabled < 4) ? modeNames[enabled] : "Unknown";
		sysLogPrintf(LOG_NOTE, "Input: Gyro auto-calibration set to '%s' for controller %d.", modeName, cidx);

		GyroCalibState *state = &gyroCalibState[cidx];
		state->wasStill = false;
		state->lastAutoCalibTime = 0;
		state->autoCalibStartTime = 0;
		
		if (gpadMotion[cidx]) {
			inputSaveGyroCalibrationOffset(cidx);  // Save or clear offset based on the selected modes
			inputConfigureGyroCalibrationMode(cidx);
		}
	}
}

static void inputUpdateGyroManualCalibration(s32 cidx)
{
	if (!pads[cidx] || !gpadMotion[cidx]) {
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

	static bool prevPressed[INPUT_MAX_CONTROLLERS] = {false};
	const bool pressed = inputBindPressed(cidx, CK_0100);
	const bool justPressed = pressed && !prevPressed[cidx];
	
	// Start manual calibration on button press
	if (justPressed && !state->manualCalibActive) {
		inputGyroManualCalibrationActivation(cidx, true);
	}
	
	// Finish manual calibration after 500ms hold
	if (state->manualCalibActive) {
		const Uint32 elapsed = SDL_GetTicks() - state->manualCalibStartTime;
		if (elapsed >= 500) {
			inputGyroManualCalibrationActivation(cidx, false);
		}
	}
	
	prevPressed[cidx] = pressed;
}

static void inputGyroManualCalibrationActivation(s32 cidx, bool start)
{
	GyroCalibState *state = &gyroCalibState[cidx];

	if (!gpadMotion[cidx]) return;

	if (start) {
		state->manualCalibActive = true;
		state->manualCalibStartTime = SDL_GetTicks();
		gmhSetCalibrationMode(gpadMotion[cidx], CALIBRATIONMODE_MANUAL);
		gmhResetContinuousCalibration(gpadMotion[cidx]);
		gmhStartContinuousCalibration(gpadMotion[cidx]);
		gyroCalibState[cidx].justFinishedCalibrating = false;
	} else {
		state->manualCalibActive = false;
		gmhPauseContinuousCalibration(gpadMotion[cidx]);
		inputSaveGyroCalibrationOffset(cidx);

		if (padsCfg[cidx].gyroAutoCalibration != GYRO_AUTOCALIBRATION_ALWAYS) inputConfigureGyroCalibrationMode(cidx);
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
	if (!pads[cidx] || !gpadMotion[cidx]) return;

	if (inputIsGyroCalibrationBlocked(cidx)) return;

	inputGyroManualCalibrationActivation(cidx, true);
}

static void inputConfigureGyroCalibrationMode(s32 cidx)
{
	if (!gpadMotion[cidx]) return;

	const s32 mode = padsCfg[cidx].gyroAutoCalibration;
	
	gmhSetCalibrationMode(gpadMotion[cidx], CALIBRATIONMODE_STILLNESS | CALIBRATIONMODE_SENSORFUSION);

	// ALWAYS mode: Reset and start continuous calibration if not just finished
	if (mode == GYRO_AUTOCALIBRATION_ALWAYS) {
		if (!gyroCalibState[cidx].justFinishedCalibrating) {
			gmhResetContinuousCalibration(gpadMotion[cidx]);
		}
	} else {
		// Other modes: Pause calibration and clear finished flag
		gmhPauseContinuousCalibration(gpadMotion[cidx]);
		gyroCalibState[cidx].justFinishedCalibrating = false;
		
		// MENU_ONLY and OFF modes: Always apply saved manual calibration offset
		if (mode == GYRO_AUTOCALIBRATION_MENU_ONLY || mode == GYRO_AUTOCALIBRATION_OFF) {
			inputApplyGyroManualCalibrationOffset(cidx);
		}
	}
}

// Block gyro calibration if Gyro Calibration UI is active
static bool inputIsGyroCalibrationBlocked(s32 cidx)
{
	return inputIsMenuGyroCalibrationActive(cidx);
}

static void inputGyroCalibrationFinished(s32 cidx, bool finished)
{
	GyroCalibState *state = &gyroCalibState[cidx];
	state->justFinishedCalibrating = finished;
	if (finished) {
		inputSaveGyroCalibrationOffset(cidx);
	}
}

static void inputApplyGyroManualCalibrationOffset(s32 cidx)
{
	if (cidx < 0 || cidx >= INPUT_MAX_CONTROLLERS || !gpadMotion[cidx]) return;
	
	const s32 mode = padsCfg[cidx].gyroAutoCalibration;
	GyroCalibState *state = &gyroCalibState[cidx];
	
	// Only apply offset for modes that use persistent manual calibration
	const bool useManualOffset = (mode == GYRO_AUTOCALIBRATION_OFF || mode == GYRO_AUTOCALIBRATION_MENU_ONLY);
	
	if (useManualOffset && state->manualWeight > 0) {
		gmhSetCalibrationOffset(gpadMotion[cidx], state->manualOffsetX, state->manualOffsetY, state->manualOffsetZ, state->manualWeight);
	}
}

static void inputSaveGyroCalibrationOffset(s32 cidx)
{
	if (!gpadMotion[cidx]) return;
	
	const s32 mode = padsCfg[cidx].gyroAutoCalibration;
	GyroCalibState *state = &gyroCalibState[cidx];

	// ALWAYS mode: Clear saved offset so it starts fresh each time
	if (mode == GYRO_AUTOCALIBRATION_ALWAYS) {
		state->manualOffsetX = state->manualOffsetY = state->manualOffsetZ = 0.0f;
		state->manualWeight = 0;
		return;
	}
	
	// OFF/MENU_ONLY modes: Save current calibration offset for persistence
	if (mode == GYRO_AUTOCALIBRATION_OFF || mode == GYRO_AUTOCALIBRATION_MENU_ONLY) {
		gmhGetCalibrationOffset(gpadMotion[cidx], &state->manualOffsetX, &state->manualOffsetY, &state->manualOffsetZ);
		state->manualWeight = 100;
	}
}

static void inputResetGyroCalibration(s32 cidx)
{
	if (!gpadMotion[cidx]) return;
	
	// Reset GamepadMotion calibration
	gmhResetGamepadMotion(gpadMotion[cidx]);
	inputConfigureGamepadMotionSettings(gpadMotion[cidx]);
	
	// Reset all gyro state
	gyroDeltaYaw[cidx] = gyroDeltaPitch[cidx] = gyroDeltaRoll[cidx] = 0.f;
	memset(&gyroCalibState[cidx], 0, sizeof(GyroCalibState));
	inputConfigureGyroCalibrationMode(cidx);
}

const char *inputGetContKeyName(u32 ck)
{
	if (ck >= CK_TOTAL_COUNT) {
		return "";
	}
	return ckNames[ck];
}

s32 inputGetContKeyByName(const char *name)
{
	for (u32 i = 0; i < CK_TOTAL_COUNT; ++i) {
		if (!strcmp(name, ckNames[i])) {
			return i;
		}
	}
	sysLogPrintf(LOG_WARNING, "unknown bind name: `%s`", name);
	return -1;
}

const char *inputGetKeyName(s32 vk)
{
	if (vk < 0 || vk >= VK_TOTAL_COUNT) {
		vk = 0;
	}
	if (!vkNames[vk][0]) {
		snprintf(vkNames[vk], sizeof(vkNames[vk]), "UNKNOWN%d", vk);
	}
	return vkNames[vk];
}

s32 inputGetKeyByName(const char *name)
{
	s32 start = 0;
	s32 end = 0;

	if (!strncmp(name, "JOY", 3) && isdigit(name[3])) {
		const s32 idx = name[3] - '1';
		if (idx >= 0 && idx < INPUT_MAX_CONTROLLERS) {
			start = VK_JOY1_BEGIN + idx * INPUT_MAX_CONTROLLER_BUTTONS;
			end = start + INPUT_MAX_CONTROLLER_BUTTONS;
		}
	} else if (!strncmp(name, "MOUSE", 5)) {
		start = VK_MOUSE_BEGIN;
		end = VK_JOY1_BEGIN;
	} else if (!strncmp(name, "UNKNOWN", 7) && isdigit(name[7])) {
		const s32 key = atoi(name + 7);
		if (key >= 0 && key < VK_TOTAL_COUNT) {
			return key;
		}
	} else {
		end = VK_MOUSE_BEGIN;
	}

	for (s32 i = start; i < end; ++i) {
		if (!strcmp(vkNames[i], name)) {
			return i;
		}
	}

	sysLogPrintf(LOG_WARNING, "unknown key name: `%s`", name);

	return -1;
}

void inputClearLastKey(void)
{
	lastKey = 0;
}

s32 inputGetLastKey(void)
{
	return lastKey;
}

void inputStartTextInput(void)
{
	lastChar = 0;
	lastKey = 0;
	textInput = 1;
	SDL_StartTextInput();
}

void inputClearLastTextChar(void)
{
	lastChar = 0;
}

char inputGetLastTextChar(void)
{
	return lastChar;
}

static inline s32 filterChar(const char ch)
{
	return isalnum(ch) || ch == ' ' || ch == '?' || ch == '!' || ch == '.';
}

s32 inputTextHandler(char *out, const u32 outSize, s32 *curCol, s32 oskCharsOnly)
{
	const s32 ctrlHeld = inputGetKeyModState() & KM_CTRL;

	if (!ctrlHeld) {
		const char chr = inputGetLastTextChar();
		inputClearLastTextChar();
		const s32 valid = chr && (oskCharsOnly ? filterChar(chr) : isprint(chr));
		if (valid) {
			if (*curCol < outSize - 1) {
				out[(*curCol)++] = chr;
				out[*curCol] = '\0';
			}
		}
	}

	const s32 key = inputGetLastKey();
	inputClearLastKey();
	if (ctrlHeld && (key == VK_A + ('v' - 'a'))) {
		// CTRL+V; paste from clipboard
		const char *clip = inputGetClipboard();
		if (clip) {
			const s32 remain = outSize - *curCol - 1;
			inputClearClipboard();
			*curCol += snprintf(out + *curCol, remain, "%s", clip);
			if (*curCol > outSize) {
				*curCol = outSize;
			}
		}
	} else if (key == VK_BACKSPACE) {
		if (*curCol) {
			out[--*curCol] = '\0';
		} else {
			out[0] = '\0';
		}
	} else if (key == VK_RETURN) {
		if (out[0] && *curCol) {
			return 1;
		}
	} else if (key == VK_ESCAPE) {
		return -1;
	}

	return 0;
}

void inputClearClipboard(void)
{
	if (clipboardText) {
		SDL_free(clipboardText);
		clipboardText = NULL;
	}
}

const char *inputGetClipboard(void)
{
	if (!clipboardText) {
		char *text = SDL_GetClipboardText();
		if (text) {
			clipboardText = text;
			// remove non-printable and multibyte chars
			for (; *text; ++text) {
				if ((u8)*text < 0x20 || (u8)*text >= 0x7F) {
					*text = '?';
				}
			}
		}
	}
	return clipboardText;
}

void inputStopTextInput(void)
{
	SDL_StopTextInput();
	textInput = 0;
}

s32 inputIsTextInputActive(void)
{
	return textInput;
}

u32 inputGetKeyModState(void)
{
	return SDL_GetModState();
}

PD_CONSTRUCTOR static void inputConfigInit(void)
{
	configRegisterInt("Input.MouseEnabled", &mouseEnabled, 0, 1);
	configRegisterInt("Input.MouseLockMode", &mouseLockMode, MLOCK_OFF, MLOCK_AUTO);
	configRegisterFloat("Input.MouseSpeedX", &mouseSensX, -30.f, 30.f);
	configRegisterFloat("Input.MouseSpeedY", &mouseSensY, -30.f, 30.f);
	configRegisterInt("Input.FakeGamepads", &fakeControllers, 0, 4);
	configRegisterInt("Input.FirstGamepadNum", &firstController, 0, 3);
	configRegisterInt("Input.UseHIDAPI", &useHIDAPI, 0, 1);
	configRegisterInt("Input.UseRawInput", &useRawInput, 0, 1);

	char secname[] = "Input.Player1.Binds";
	char keyname[256] = { 0 };
	for (s32 c = 0; c < MAXCONTROLLERS; ++c) {
		secname[12] = '1' + c;
		secname[13] = '\0';
		configRegisterFloat(strFmt("%s.RumbleScale", secname), &padsCfg[c].rumbleScale, 0.f, 1.f);
		configRegisterInt(strFmt("%s.LStickDeadzoneX", secname), &padsCfg[c].deadzone[0], 0, 32767);
		configRegisterInt(strFmt("%s.LStickDeadzoneY", secname), &padsCfg[c].deadzone[1], 0, 32767);
		configRegisterInt(strFmt("%s.RStickDeadzoneX", secname), &padsCfg[c].deadzone[2], 0, 32767);
		configRegisterInt(strFmt("%s.RStickDeadzoneY", secname), &padsCfg[c].deadzone[3], 0, 32767);
		configRegisterFloat(strFmt("%s.LStickScaleX", secname), &padsCfg[c].sens[0], -10.f, 10.f);
		configRegisterFloat(strFmt("%s.LStickScaleY", secname), &padsCfg[c].sens[1], -10.f, 10.f);
		configRegisterFloat(strFmt("%s.RStickScaleX", secname), &padsCfg[c].sens[2], -10.f, 10.f);
		configRegisterFloat(strFmt("%s.RStickScaleY", secname), &padsCfg[c].sens[3], -10.f, 10.f);
		configRegisterInt(strFmt("%s.GyroEnabled", secname), &padsCfg[c].gyroEnabled, 0, 1);
		configRegisterInt(strFmt("%s.GyroAdvanced", secname), &padsCfg[c].gyroAdvanced, 0, 1);
		configRegisterInt(strFmt("%s.GyroAimMode", secname), &padsCfg[c].gyroAimMode, GYRO_AIM_CAMERA, GYRO_AIM_BOTH);
		configRegisterInt(strFmt("%s.GyroModifier", secname), &padsCfg[c].gyroModifier, GYRO_ALWAYS_ON, GYRO_DISABLE_HELD);
		configRegisterInt(strFmt("%s.GyroAxisMode", secname), &padsCfg[c].gyroAxisMode, GYRO_YAW, GYRO_WORLD);
		configRegisterInt(strFmt("%s.GyroAutoCalibration", secname), &padsCfg[c].gyroAutoCalibration, GYRO_AUTOCALIBRATION_OFF, GYRO_AUTOCALIBRATION_ALWAYS);
		configRegisterFloat(strFmt("%s.GyroSpeedX", secname), &padsCfg[c].gyroSpeedX, -30.f, 30.f);
		configRegisterFloat(strFmt("%s.GyroSpeedY", secname), &padsCfg[c].gyroSpeedY, -30.f, 30.f);
		configRegisterFloat(strFmt("%s.GyroAimSpeedX", secname), &padsCfg[c].gyroAimSpeedX, -10.f, 10.f);
		configRegisterFloat(strFmt("%s.GyroAimSpeedY", secname), &padsCfg[c].gyroAimSpeedY, -10.f, 10.f);
		configRegisterInt(strFmt("%s.GyroInvertX", secname), &padsCfg[c].gyroInvertX, 0, 1);
		configRegisterInt(strFmt("%s.GyroInvertY", secname), &padsCfg[c].gyroInvertY, 0, 1);
		configRegisterInt(strFmt("%s.GyroAimInvertX", secname), &padsCfg[c].gyroAimInvertX, 0, 1);
		configRegisterInt(strFmt("%s.GyroAimInvertY", secname), &padsCfg[c].gyroAimInvertY, 0, 1);
		configRegisterFloat(strFmt("%s.GyroVHMixer", secname), &padsCfg[c].gyroVHMixer, -1.0f, 1.0f);
		configRegisterFloat(strFmt("%s.GyroSmoothing", secname), &padsCfg[c].gyroSmoothing, 0.f, 1.f);
		configRegisterFloat(strFmt("%s.GyroTightening", secname), &padsCfg[c].gyroTightening, 0.f, 10.f);
		configRegisterFloat(strFmt("%s.GyroDeadzone", secname), &padsCfg[c].gyroDeadzone, 0.f, 1.f);
		configRegisterInt(strFmt("%s.StickCButtons", secname), &padsCfg[c].stickCButtons, 0, 1);
		configRegisterInt(strFmt("%s.CancelCButtons", secname), &padsCfg[c].cancelCButtons, 0, 1);
		configRegisterInt(strFmt("%s.SwapSticks", secname), &padsCfg[c].swapSticks, 0, 1);
		configRegisterInt(strFmt("%s.ControllerIndex", secname), &padsCfg[c].deviceIndex, -1, 0x7FFFFFFF);
		secname[13] = '.';
		for (u32 ck = 0; ck < CK_TOTAL_COUNT; ++ck) {
			snprintf(keyname, sizeof(keyname), "%s.%s", secname, inputGetContKeyName(ck));
			configRegisterString(keyname, bindStrs[c][ck], MAX_BIND_STR);
		}
	}
}