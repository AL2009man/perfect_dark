#include <string.h>
#include <ctype.h>
#include <SDL.h>
#include <PR/ultratypes.h>
#include <PR/os_thread.h>
#include <PR/os_cont.h>
#include "platform.h"
#include "input.h"
#include "video.h"
#include "config.h"
#include "utils.h"
#include "system.h"
#include "fs.h"

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

// Define gyro activation modes
#define GYRO_ALWAYS_ON 0
#define GYRO_TOGGLE 1
#define GYRO_HOLD 2
#define GYRO_HOLD_INVERTED 3

#define GYRO_AIM_CAMERA 0
#define GYRO_AIM_CROSSHAIR 1
#define GYRO_AIM_BOTH 2 // Default mode

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
}

static struct controllercfg {
	s32 rumbleOn;
	f32 rumbleScale;
	u32 axisMap[2][2];
	f32 sens[4];
	s32 deadzone[4];
	s32 stickCButtons;
	s32 swapSticks;
	s32 deviceIndex;
	s32 cancelCButtons;
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

static s32 mouseEnabled = 1;
static s32 mouseX, mouseY;
static s32 mouseDX, mouseDY;
static u32 mouseButtons;
static s32 mouseWheel = 0;

static s32 mouseLocked = 0;
static s32 mouseLockMode = MLOCK_AUTO;
static u64 mouseCursorTime = 0;
static s32 mouseShowCursor = 1;

static f32 mouseSensX = 1.5f;
static f32 mouseSensY = 1.5f;

static s32 gyroEnabled = 1; // Enable gyro aiming by default
static f32 gyroX, gyroY;
static s32 gyroDX, gyroDY;
static f32 accelX = 0.0f;
static f32 accelY = 0.0f;
static f32 accelZ = 0.0f;

static f32 gyroSensX = 2.50f;
static f32 gyroSensY = 2.50f;
static f32 gyroCrosshairSpeedX = 1.0f;
static f32 gyroCrosshairSpeedY = 1.0f;
static f32 gyroMinThreshold = 0.1f;

static s32 g_GyroActivationMode = GYRO_ALWAYS_ON;
static s32 g_GyroToggleState = 0;
static s32 g_GyroHoldState = 0;

static s32 g_GyroAxisMode = 0;
static s32 g_GyroAimMode = GYRO_AIM_BOTH;

// Gyro calibration variables
static f32 gyro_calibration_x = 0.0f;
static f32 gyro_calibration_y = 0.0f;
static f32 gyro_calibration_z = 0.0f;

// Gyro camera control variables
static f32 gyroCameraYaw = 0.0f;
static f32 gyroCameraPitch = 0.0f;
static f32 gyroCameraRoll = 0.0f;

static s32 lastKey = 0;
static char lastChar = 0;
static s32 textInput = 0;

static char *clipboardText = NULL;

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
		{ CK_4000,          SDL_SCANCODE_LSHIFT, 0                   },
		{ CK_2000,          SDL_SCANCODE_LCTRL,  0                   }
	};

	static const u32 pcjoybinds[][2] = {
		{ CK_A,      SDL_CONTROLLER_BUTTON_A             },
		{ CK_X,      SDL_CONTROLLER_BUTTON_X             },
		{ CK_Y,      SDL_CONTROLLER_BUTTON_Y             },
		{ CK_DPAD_L, SDL_CONTROLLER_BUTTON_B,            },
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

static inline SDL_JoystickID inputControllerGetId(SDL_GameController* ctrl)
{
		return SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(ctrl));
}

// Forward Declarations
static void inputInitSensor(SDL_GameController* controller, s32 jidx, SDL_SensorType sensorType, const char* sensorName);
static void inputUpdateMouse(void);
static void inputUpdateGyro(void);
static void handleSensorUpdate(SDL_Event* event);
static void updateCameraControl(f32 dx, f32 dy, f32 dz);
static void updateCrosshairPosition(float x, float y);

static inline void inputInitController(const s32 cidx, const s32 jidx)
{
#if SDL_VERSION_ATLEAST(2, 0, 18)
		// SDL_GameControllerHasRumble() appeared in 2.0.18 even though SDL_GameControllerRumble() is in 2.0.9
		padsCfg[cidx].rumbleOn = SDL_GameControllerHasRumble(pads[cidx]);
#else
		// Assume that all joysticks with haptic feedback support will support rumble
		padsCfg[cidx].rumbleOn = SDL_JoystickIsHaptic(SDL_GameControllerGetJoystick(pads[cidx]));
		if (!padsCfg[cidx].rumbleOn) {
				// At least on Windows, some controllers will report no haptics, but rumble will still function
				// Just assume it's supported if the controller is of a known type
				const SDL_GameControllerType ctype = SDL_GameControllerGetType(pads[cidx]);
				padsCfg[cidx].rumbleOn = ctype && (ctype != SDL_CONTROLLER_TYPE_VIRTUAL);
		}
#endif

		// Set LEDs to indicate player index
		SDL_GameControllerSetPlayerIndex(pads[cidx], cidx);

		// Remember the joystick index
		padsCfg[cidx].deviceIndex = jidx;

		connectedMask |= (1 << cidx);

		sysLogPrintf(LOG_NOTE, "input: Assigned controller '%d: (%s)' (ID %d) to player %d",
				jidx, SDL_GameControllerName(pads[cidx]), inputControllerGetId(pads[cidx]), cidx);

		SDL_Joystick* joy = SDL_GameControllerGetJoystick(pads[cidx]);
		if (joy) {
				char guidStr[1024] = "";
				SDL_JoystickGUID guid = SDL_JoystickGetGUID(joy);
				SDL_JoystickGetGUIDString(guid, guidStr, sizeof(guidStr));
				sysLogPrintf(LOG_NOTE, "input: GUID for controller %d: %s", jidx, guidStr);
		}

		// Initialize sensors
		inputInitSensor(pads[cidx], jidx, SDL_SENSOR_GYRO, "Gyroscope");
		inputInitSensor(pads[cidx], jidx, SDL_SENSOR_ACCEL, "Accelerometer");
}

static void inputInitSensor(SDL_GameController* controller, s32 jidx, SDL_SensorType sensorType, const char* sensorName) {
		if (!controller) {
				sysLogPrintf(LOG_WARNING, "input: Invalid controller passed for sensor initialization.");
				return;
		}

		// Check if the sensor is available
		if (SDL_GameControllerHasSensor(controller, sensorType)) {
				sysLogPrintf(LOG_NOTE, "input: %s sensor available for controller %d", sensorName, jidx);

				// Attempt to enable the sensor
				if (SDL_GameControllerSetSensorEnabled(controller, sensorType, SDL_TRUE) == 0) {
						float rate = SDL_GameControllerGetSensorDataRate(controller, sensorType);

						// Check if the data rate is sufficient
						if (rate >= 60.0f) {
								sysLogPrintf(LOG_NOTE, "input: %s sensor enabled for controller %d with data rate %.2f Hz", sensorName, jidx, rate);
						}
						else {
								sysLogPrintf(LOG_WARNING, "input: %s data rate may be insufficient for controller %d (%.2f Hz)", sensorName, jidx, rate);
						}
				}
				else {
						sysLogPrintf(LOG_WARNING, "input: Failed to enable %s sensor for controller %d", sensorName, jidx);
				}
		}
		else {
				sysLogPrintf(LOG_WARNING, "input: %s sensor not available for controller %d", sensorName, jidx);
		}
}

static void handleSensorUpdate(SDL_Event* event)
{
		if (event->csensor.sensor == SDL_SENSOR_GYRO || event->csensor.sensor == SDL_SENSOR_ACCEL) {
				float sensorData[3] = { 0 }; // Initialize sensor data array
				SDL_GameController* controller = SDL_GameControllerFromInstanceID(event->cdevice.which);

				if (!controller) {
						sysLogPrintf(LOG_WARNING, "input: Controller %d not found for sensor data retrieval", event->cdevice.which);
						return;
				}

				if (SDL_GameControllerGetSensorData(controller, event->csensor.sensor, sensorData, 3) == 0) {
						sysLogPrintf(LOG_NOTE, "input: %s sensor data for controller %d: X=%f, Y=%f, Z=%f",
								(event->csensor.sensor == SDL_SENSOR_GYRO ? "Gyroscope" : "Accelerometer"),
								event->cdevice.which, sensorData[0], sensorData[1], sensorData[2]);

						if (event->csensor.sensor == SDL_SENSOR_GYRO) {
								gyroDX = sensorData[0] * gyroSensX;
								gyroDY = sensorData[1] * gyroSensY;

								gyroDX = fabs(gyroDX) < gyroMinThreshold ? 0 : gyroDX;
								gyroDY = fabs(gyroDY) < gyroMinThreshold ? 0 : gyroDY;

								updateCameraControl(gyroDX, gyroDY, 0);
						}
						else if (event->csensor.sensor == SDL_SENSOR_ACCEL) {
								accelX = sensorData[0];
								accelY = sensorData[1];
								accelZ = sensorData[2];

								sysLogPrintf(LOG_NOTE, "Accelerometer data processed (placeholder logic)");
						}
				}
				else {
						sysLogPrintf(LOG_WARNING, "input: Failed to retrieve %s sensor data for controller %d",
								(event->csensor.sensor == SDL_SENSOR_GYRO ? "Gyroscope" : "Accelerometer"),
								event->cdevice.which);
				}
		}
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

static int inputEventFilter(void* data, SDL_Event* event)
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
				SDL_GameController* ctrl = SDL_GameControllerFromInstanceID(event->cdevice.which);
				const s32 idx = inputControllerGetIndex(ctrl);
				if (idx >= 0) {
						inputCloseController(idx);
						padsCfg[idx].deviceIndex = -1;
				}
				break;
		}

		case SDL_JOYDEVICEADDED:
		case SDL_JOYDEVICEREMOVED:
				numJoysticks = SDL_NumJoysticks();
				break;

		case SDL_CONTROLLERSENSORUPDATE:
				handleSensorUpdate(event); 
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
						SDL_GameController* ctrl = SDL_GameControllerFromInstanceID(event->cdevice.which);
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
								SDL_GameController* ctrl = SDL_GameControllerFromInstanceID(event->cdevice.which);
								const s32 idx = inputControllerGetIndex(ctrl);
								if (idx >= 0) {
										lastKey += idx * INPUT_MAX_CONTROLLER_BUTTONS;
								}
						}
				}
				break;

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
	if (!SDL_WasInit(SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC)) {
		SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC);
	}

	// Set SDL hints before loading controller mappings
	SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS3, "1");
	SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS4, "1");
	SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS5, "1");
	SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_SWITCH, "1");
	SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS4_RUMBLE, "1"); // This hint will enable PlayStation's more advanced feature sets when using Bluetooth connecivity, this includes Motion Sensors.
	SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS5_RUMBLE, "1"); // same case as PS4_Rumble hint.
	SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_JOY_CONS, "1");
	SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_COMBINE_JOY_CONS, "1");
	SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_GAMECUBE, "1");
	SDL_SetHint(SDL_HINT_JOYSTICK_RAWINPUT, "1");
	SDL_SetHint(SDL_HINT_JOYSTICK_RAWINPUT_CORRELATE_XINPUT, "1");

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

	return connectedMask;
}

static inline s32 inputBindPressed(const s32 idx, const u32 ck)
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

void inputUpdate(void)
{
		SDL_GameControllerUpdate();

		if (mouseEnabled) {
				inputUpdateMouse();
		}

		if (gyroEnabled) {
				inputUpdateGyro();
		}
}

static void updateCameraControl(f32 dx, f32 dy, f32 dz)
{
		// Update camera control logic using gyro inputs
		gyroCameraYaw += dx;
		gyroCameraPitch += dy;
		gyroCameraRoll += dz;

		// Log updated camera control values
		sysLogPrintf(LOG_NOTE, "input: Updated camera control - Yaw=%f, Pitch=%f, Roll=%f",
				gyroCameraYaw, gyroCameraPitch, gyroCameraRoll);
}

static void updateCrosshairPosition(float x, float y)
{
		gyroX = x;
		gyroY = y;

		sysLogPrintf(LOG_NOTE, "input: Updated crosshair position - X=%f, Y=%f", x, y);
}

static void inputUpdateMouse(void)
{
		s32 mx, my;
		mouseButtons = SDL_GetMouseState(&mx, &my);

		if (mouseWheel > 0) {
				mouseButtons |= WHEEL_UP_MASK;
		}
		else if (mouseWheel < 0) {
				mouseButtons |= WHEEL_DN_MASK;
		}

		mouseWheel = 0;

		s32 mdx = 0, mdy = 0;
		SDL_GetRelativeMouseState(&mdx, &mdy);
		if (mouseLocked) {
				mouseDX = mdx;
				mouseDY = mdy;
		}
		else {
				mouseDX = mx - mouseX;
				mouseDY = my - mouseY;
		}

		mouseX = mx;
		mouseY = my;

		if (mouseLockMode == MLOCK_AUTO && !mouseLocked) {
				if (abs(mouseDX) > CURSOR_HIDE_THRESHOLD || abs(mouseDY) > CURSOR_HIDE_THRESHOLD) {
						inputMouseShowCursor(1);
				}
				else if (sysGetMicroseconds() > mouseCursorTime) {
						inputMouseShowCursor(0);
				}
		}

		updateCameraControl(mouseDX * mouseSensX, mouseDY * mouseSensY, 0);
}

static void inputUpdateGyro(void)
{
		SDL_GameController* controller = pads[0]; // Assuming pads[0] is the primary controller
		if (!controller || !SDL_GameControllerHasSensor(controller, SDL_SENSOR_GYRO)) {
				sysLogPrintf(LOG_WARNING, "Gyroscope not available or enabled");
				return;
		}

		float gyroData[3] = { 0 }; // Initialize gyro data
		if (SDL_GameControllerGetSensorData(controller, SDL_SENSOR_GYRO, gyroData, 3) != 0) {
				sysLogPrintf(LOG_WARNING, "Failed to read gyroscope data");
				return;
		}

		sysLogPrintf(LOG_NOTE, "Gyro Data: X=%f, Y=%f, Z=%f", gyroData[0], gyroData[1], gyroData[2]);

		// Apply noise filtering
		gyroData[0] = fabs(gyroData[0]) < gyroMinThreshold ? 0 : gyroData[0];
		gyroData[1] = fabs(gyroData[1]) < gyroMinThreshold ? 0 : gyroData[1];

		// Handle gyro activation modes using the "Gyro Modifier" button
		s32 gyroActive = 0;
		switch (inputGetGyroActivationMode()) {
		case GYRO_TOGGLE:
				// Toggles the gyro on/off with a press of CK_GYRO_MOD
				if (inputKeyJustPressed(CK_GYRO_MOD)) {
						g_GyroToggleState = !g_GyroToggleState;
				}
				gyroActive = g_GyroToggleState;
				break;

		case GYRO_HOLD:
				// Activates the gyro while CK_GYRO_MOD is held
				gyroActive = inputKeyPressed(CK_GYRO_MOD);
				break;

		case GYRO_HOLD_INVERTED:
				// Deactivates the gyro while CK_GYRO_MOD is held
				gyroActive = !inputKeyPressed(CK_GYRO_MOD);
				break;

		default:
				// No additional handling for undefined modes
				break;
		}

		if (gyroActive) {
				// Relative movement logic to mirror mouse behavior
				s32 mdx = gyroData[0] - gyroX;
				s32 mdy = gyroData[1] - gyroY;

				gyroDX = mdx;
				gyroDY = mdy;

				gyroX = gyroData[0];
				gyroY = gyroData[1];

				// Handle gyro aiming modes and axis modes
				switch (inputGetGyroAxisMode()) {
				case 0: // Yaw mode
						updateCameraControl(gyroDX * gyroSensX, gyroDY * gyroSensY, 0);
						break;
				case 1: // Roll mode
						updateCameraControl(-gyroData[2] * gyroSensX, gyroDY * gyroSensY, gyroData[2] * gyroSensX);
						break;
				}

				// Update crosshair position
				float crosshairX = gyroDX * gyroCrosshairSpeedX;
				float crosshairY = gyroDY * gyroCrosshairSpeedY;
				updateCrosshairPosition(crosshairX, crosshairY);

				// Cursor behavior logic
				if (mouseLockMode == MLOCK_AUTO) {
						if (abs(gyroDX) > CURSOR_HIDE_THRESHOLD || abs(gyroDY) > CURSOR_HIDE_THRESHOLD) {
								inputMouseShowCursor(1); // Show cursor if there's significant movement
						}
						else if (sysGetMicroseconds() > mouseCursorTime) {
								inputMouseShowCursor(0); // Hide cursor if idle
						}
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

void inputMouseGetScaledDelta(f32 *dx, f32 *dy)
{
	f32 mdx, mdy;
	if (mouseLocked) {
		mdx = mouseSensX * (f32)mouseDX / 100.0f;
		mdy = mouseSensY * (f32)mouseDY / 100.0f;
	} else {
		mdx = 0.f;
		mdy = 0.f;
	}
	if (dx) *dx = mdx;
	if (dy) *dy = mdy;
}

void inputMouseGetAbsScaledDelta(f32 *dx, f32 *dy)
{
	f32 mdx, mdy;
	if (mouseLocked) {
		mdx = fabsf(mouseSensX) * (f32)mouseDX / 100.0f;
		mdy = fabsf(mouseSensY) * (f32)mouseDY / 100.0f;
	} else {
		mdx = 0.f;
		mdy = 0.f;
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

s32 inputGyroGetPosition(s32* x, s32* y)
{
		if (x) *x = gyroX * videoGetNativeWidth() / videoGetWidth();
		if (y) *y = gyroY * videoGetNativeHeight() / videoGetHeight();
		return (gyroDX != 0 || gyroDY != 0);
}

void inputGyroGetRawDelta(s32* dx, s32* dy)
{
		if (dx) *dx = gyroDX;
		if (dy) *dy = gyroDY;
}

void inputGyroGetScaledDelta(f32* dx, f32* dy)
{
		if (dx && dy) {
				*dx = gyroSensX * (f32)gyroDX / 100.0f;
				*dy = gyroSensY * (f32)gyroDY / 100.0f;
		}
}

void inputGyroGetAbsScaledDelta(f32* dx, f32* dy)
{
		if (dx && dy) {
				*dx = fabsf(gyroSensX) * (f32)gyroDX / 100.0f;
				*dy = fabsf(gyroSensY) * (f32)gyroDY / 100.0f;
		}
}

void inputGyroGetSpeed(f32* x, f32* y)
{
		if (x && y) {
				*x = gyroSensX;
				*y = gyroSensY;
		}
}

void inputGyroSetSpeed(f32 x, f32 y)
{
		gyroSensX = x;
		gyroSensY = y;
}

f32 inputGyroGetSpeedX(void)
{
		return gyroSensX;
}

void inputGyroSetSpeedX(f32 speed)
{
		gyroSensX = speed;
}

f32 inputGyroGetSpeedY(void)
{
		return gyroSensY;
}

void inputGyroSetSpeedY(f32 speed)
{
		gyroSensY = speed;
}

s32 inputGetGyroAxisMode(void)
{
		return g_GyroAxisMode;
}

void inputSetGyroAxisMode(s32 mode)
{
		g_GyroAxisMode = mode;
}

f32 inputGetGyroMinThreshold(void)
{
		return gyroMinThreshold;
}

void inputSetGyroMinThreshold(f32 threshold)
{
		gyroMinThreshold = threshold;
}

s32 inputGyroIsEnabled(void)
{
		return gyroEnabled;
}

void inputGyroEnable(s32 enabled)
{
		gyroEnabled = !!enabled;
}

s32 inputGetGyroActivationMode(void)
{
		return g_GyroActivationMode;
}

void inputSetGyroActivationMode(s32 mode)
{
		g_GyroActivationMode = mode;
}

s32 inputGetGyroAimMode(void)
{
		return g_GyroAimMode;
}

void inputSetGyroAimMode(s32 mode)
{
		g_GyroAimMode = mode;
}

f32 inputGyroGetCrosshairSpeedX(void)
{
		return gyroCrosshairSpeedX;
}

void inputGyroSetCrosshairSpeedX(f32 speed)
{
		gyroCrosshairSpeedX = speed;
}

f32 inputGyroGetCrosshairSpeedY(void)
{
		return gyroCrosshairSpeedY;
}

void inputGyroSetCrosshairSpeedY(f32 speed)
{
		gyroCrosshairSpeedY = speed;
}

void inputHandleGyroModifier(void)
{
		static s32 gyroToggleState = 0;

		// Determine the active gyro mode
		switch (inputGetGyroActivationMode()) {
		case GYRO_TOGGLE:
				if (inputKeyJustPressed(CK_GYRO_MOD)) {
						gyroToggleState = !gyroToggleState;
						inputGyroEnable(gyroToggleState);
				}
				break;

		case GYRO_HOLD:
				inputGyroEnable(inputKeyPressed(CK_GYRO_MOD));
				break;

		case GYRO_HOLD_INVERTED:
				inputGyroEnable(!inputKeyPressed(CK_GYRO_MOD));
				break;

		default:
				// Gyro Always On is omitted to avoid input checks
				break;
		}
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
		configRegisterFloat("Input.MouseSpeedX", &mouseSensX, -10.f, 10.f);
		configRegisterFloat("Input.MouseSpeedY", &mouseSensY, -10.f, 10.f);
		configRegisterInt("Input.GyroEnabled", &gyroEnabled, 0, 1);
		configRegisterFloat("Input.GyroSpeedX", &gyroSensX, -10.f, 10.f);
		configRegisterFloat("Input.GyroSpeedY", &gyroSensY, -10.f, 10.f);
		configRegisterFloat("Input.GyroCrosshairSpeedX", &gyroCrosshairSpeedX, -10.f, 10.f);
		configRegisterFloat("Input.GyroCrosshairSpeedY", &gyroCrosshairSpeedY, -10.f, 10.f);
		configRegisterFloat("Input.GyroMinThreshold", &gyroMinThreshold, 0.f, 5.f);
		configRegisterInt("Input.GyroActivationMode", &g_GyroActivationMode, GYRO_ALWAYS_ON, GYRO_HOLD_INVERTED);
		configRegisterInt("Input.GyroAimMode", &g_GyroAimMode, GYRO_AIM_CAMERA, GYRO_AIM_BOTH);
		configRegisterInt("Input.FakeGamepads", &fakeControllers, 0, 4);
		configRegisterInt("Input.FirstGamepadNum", &firstController, 0, 3);

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
