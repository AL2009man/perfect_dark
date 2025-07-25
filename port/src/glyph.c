#include <PR/ultratypes.h>
#include "glyph.h"

// Generic display names for controller buttons (maps to same indices as vkJoyNames)
const char *vkJoyDisplayNames[] = {
	"BTN_SOUTH",        // A button (Bottom face button)
	"BTN_EAST",         // B button (Right face button)
	"BTN_WEST",         // X button (Left face button)
	"BTN_NORTH",        // Y button (Top face button)
	"BTN_BACK",
	"BTN_GUIDE",
	"BTN_START",
	"STICK_LEFT_CLICK",
	"STICK_RIGHT_CLICK",
	"SHOULDER_LEFT",
	"SHOULDER_RIGHT",
	"D-PAD_UP",
	"D-PAD_DOWN",
	"D-PAD_LEFT",
	"D-PAD_RIGHT",
	"BTN_MISC1",        // Additional button (e.g. Xbox Series X share button, PS5 microphone button, Switch capture button)
	"RIGHT_PADDLE_1",   // Upper or primary paddle, under your right hand (e.g. Xbox Elite paddle P1)
	"LEFT_PADDLE_1",    // Upper or primary paddle, under your left hand (e.g. Xbox Elite paddle P3)
	"RIGHT_PADDLE_2",   // Lower or secondary paddle, under your right hand (e.g. Xbox Elite paddle P2)
	"LEFT_PADDLE_2",    // Lower or secondary paddle, under your left hand (e.g. Xbox Elite paddle P4)
	"BTN_TOUCHPAD",     // PS4/PS5 touchpad button
	"BTN_MISC2",
	"BTN_MISC3",
	"BTN_MISC4",
	"BTN_MISC5",
	"BTN_MISC6",
	"BTN_26",
	"BTN_27",
	"BTN_28",
	"BTN_29",
	"LEFT_TRIG",
	"RIGHT_TRIG",
};

// Controller-specific overrides

// Common Xbox controller button overrides
static const struct {
	int button_index;
	const char *name;
} xbox_prompts[] = {
	{ 0, "BTN_A" },
	{ 1, "BTN_B" },
	{ 2, "BTN_X" },
	{ 3, "BTN_Y" },
	{ 5, "BTN_XBOX"},
};

// Xbox 360 specific button overrides
static const struct {
	int button_index;
	const char *name;
} xbox360_overrides[] = {
	{ 4, "BTN_BACK" },
	{ 6, "BTN_START" },
};

// Xbox One specific button overrides
static const struct {
	int button_index;
	const char *name;
} xboxone_overrides[] = {
	{ 4, "BTN_VIEW" },
	{ 6, "BTN_MENU" },
	{ 15, "BTN_SHARE" },
	{ 16, "BTN_PADDLE1" },    // Xbox Elite right upper paddle
	{ 17, "BTN_PADDLE2" },    // Xbox Elite left upper paddle
	{ 18, "BTN_PADDLE3" },    // Xbox Elite right lower paddle
	{ 19, "BTN_PADDLE4" },    // Xbox Elite left lower paddle
};

// Common PlayStation controller button overrides
static const struct {
	int button_index;
	const char *name;
} playstation_prompts[] = {
	{ 0, "BTN_CROSS" },
	{ 1, "BTN_CIRCLE" },
	{ 2, "BTN_SQUARE" },
	{ 3, "BTN_TRIANGLE" },
	{ 5, "BTN_PS" },
	{ 7, "STICK_L3" },
	{ 8, "STICK_R3" },
	{ 9, "BTN_L1" },
	{ 10, "BTN_R1" },
	{ 30, "TRIG_L2" },
	{ 31, "TRIG_R2" },
};

// PS3 specific button overrides
static const struct {
	int button_index;
	const char *name;
} ps3_overrides[] = {
	{ 4, "BTN_SELECT" },
	{ 6, "BTN_START" },
};

// PS4 specific button overrides
static const struct {
	int button_index;
	const char *name;
} ps4_overrides[] = {
	{ 4, "BTN_SHARE" },
	{ 6, "BTN_OPTIONS" },
};

// PS5 specific button overrides
static const struct {
	int button_index;
	const char *name;
} ps5_overrides[] = {
	{ 4, "BTN_CREATE" },
	{ 6, "BTN_OPTIONS" },
	{ 15, "BTN_MIC" },       // PS5 microphone mute button
	{ 18, "BTN_LB" },        // DualSense Edge left back button
	{ 19, "BTN_RB" },        // DualSense Edge right back button
};

// Nintendo Switch controller button overrides
static const struct {
	int button_index;
	const char *name;
} switch_overrides[] = {
	{ 0, "BTN_B" },          // Nintendo layout
	{ 1, "BTN_A" },          // Nintendo layout
	{ 2, "BTN_Y" },          // Nintendo layout
	{ 3, "BTN_X" },          // Nintendo layout
	{ 4, "BTN_MINUS" },
	{ 5, "BTN_HOME" },
	{ 6, "BTN_PLUS" },
	{ 9, "BTN_L" },
	{ 10, "BTN_R" },
	{ 15, "BTN_CAPTURE" },
	{ 16, "BTN_SL" },
	{ 17, "BTN_SR" },
	{ 30, "TRIG_ZL" },
	{ 31, "TRIG_ZR" },
};

// Function to get controller-specific button name with fallback to generic
const char *glyphGetButtonName(int controllerType, int buttonIndex)
{
	switch (controllerType) {
		case CONTROLLER_ICON_XBOX360:
			// Check Xbox 360 specific overrides first
			for (int i = 0; i < sizeof(xbox360_overrides) / sizeof(xbox360_overrides[0]); i++) {
				if (xbox360_overrides[i].button_index == buttonIndex) {
					return xbox360_overrides[i].name;
				}
			}
			// Fallback to common Xbox overrides
			for (int i = 0; i < sizeof(xbox_prompts) / sizeof(xbox_prompts[0]); i++) {
				if (xbox_prompts[i].button_index == buttonIndex) {
					return xbox_prompts[i].name;
				}
			}
			break;
			
		case CONTROLLER_ICON_XBOXONE:
			// Check Xbox One specific overrides first
			for (int i = 0; i < sizeof(xboxone_overrides) / sizeof(xboxone_overrides[0]); i++) {
				if (xboxone_overrides[i].button_index == buttonIndex) {
					return xboxone_overrides[i].name;
				}
			}
			// Fallback to common Xbox overrides
			for (int i = 0; i < sizeof(xbox_prompts) / sizeof(xbox_prompts[0]); i++) {
				if (xbox_prompts[i].button_index == buttonIndex) {
					return xbox_prompts[i].name;
				}
			}
			break;
			
		case CONTROLLER_ICON_PS3:
			// Check PS3 specific overrides first
			for (int i = 0; i < sizeof(ps3_overrides) / sizeof(ps3_overrides[0]); i++) {
				if (ps3_overrides[i].button_index == buttonIndex) {
					return ps3_overrides[i].name;
				}
			}
			// Fallback to common PlayStation overrides
			for (int i = 0; i < sizeof(playstation_prompts) / sizeof(playstation_prompts[0]); i++) {
				if (playstation_prompts[i].button_index == buttonIndex) {
					return playstation_prompts[i].name;
				}
			}
			break;
			
		case CONTROLLER_ICON_PS4:
			// Check PS4 specific overrides first
			for (int i = 0; i < sizeof(ps4_overrides) / sizeof(ps4_overrides[0]); i++) {
				if (ps4_overrides[i].button_index == buttonIndex) {
					return ps4_overrides[i].name;
				}
			}
			// Fallback to common PlayStation overrides
			for (int i = 0; i < sizeof(playstation_prompts) / sizeof(playstation_prompts[0]); i++) {
				if (playstation_prompts[i].button_index == buttonIndex) {
					return playstation_prompts[i].name;
				}
			}
			break;
			
		case CONTROLLER_ICON_PS5:
			// Check PS5 specific overrides first
			for (int i = 0; i < sizeof(ps5_overrides) / sizeof(ps5_overrides[0]); i++) {
				if (ps5_overrides[i].button_index == buttonIndex) {
					return ps5_overrides[i].name;
				}
			}
			// Fallback to common PlayStation overrides
			for (int i = 0; i < sizeof(playstation_prompts) / sizeof(playstation_prompts[0]); i++) {
				if (playstation_prompts[i].button_index == buttonIndex) {
					return playstation_prompts[i].name;
				}
			}
			break;
			
		case CONTROLLER_ICON_NINTENDO_SWITCH:
			for (int i = 0; i < sizeof(switch_overrides) / sizeof(switch_overrides[0]); i++) {
				if (switch_overrides[i].button_index == buttonIndex) {
					return switch_overrides[i].name;
				}
			}
			break;
			
		case CONTROLLER_ICON_GENERIC: // Generic/fallback
		default:
			break;
	}
	
	// Fallback to generic names if no override found or unknown controller type
	if (buttonIndex >= 0 && buttonIndex < 32) {
		return vkJoyDisplayNames[buttonIndex];
	}
	
	return "UNKNOWN BUTTON";
}

