#include <PR/ultratypes.h>
#include "glyph.h"

// Generic display names for controller buttons (maps to same indices as vkJoyNames)
const char *vkJoyDisplayNames[] = {
	"SOUTH_BTN",        // A button (Bottom face button)
	"EAST_BTN",         // B button (Right face button)
	"WEST_BTN",         // X button (Left face button)
	"NORTH_BTN",        // Y button (Top face button)
	"BACK_BTN",
	"GUIDE_BTN",
	"START_BTN",
	"LEFT_STICK_CLICK",
	"RIGHT_STICK_CLICK",
	"LEFT_SHOULDER",
	"RIGHT_SHOULDER",
	"D-PAD_UP",
	"D-PAD_DOWN",
	"D-PAD_LEFT",
	"D-PAD_RIGHT",
	"MISC1_BTN",       // Additional button (e.g. Xbox Series X share button, PS5 microphone button, Switch capture button)
	"RIGHT_PADDLE_1",   // Upper or primary paddle, under your right hand (e.g. Xbox Elite paddle P1)
	"LEFT_PADDLE_1",    // Upper or primary paddle, under your left hand (e.g. Xbox Elite paddle P3)
	"RIGHT_PADDLE_2",   // Lower or secondary paddle, under your right hand (e.g. Xbox Elite paddle P2)
	"LEFT_PADDLE_2",    // Lower or secondary paddle, under your left hand (e.g. Xbox Elite paddle P4)
	"TOUCHPAD_BTN",     // PS4/PS5 touchpad button
	"MISC2_BTN",
	"MISC3_BTN",
	"MISC4_BTN",
	"MISC5_BTN",
	"MISC6_BTN",
	"BTN_26",
	"BTN_27",
	"BTN_28",
	"BTN_29",
	"LEFT_TRIG",
	"RIGHT_TRIG",
};

// Controller-specific overrides

// Common struct type for button overrides
struct button_override {
	int button_index;
	const char *name;
};

// Standard button overrides (shared across controllers)
static const struct button_override glyph_standard[] = {
	{  0, "A_BTN" },          // Bottom face button (A)
	{  1, "B_BTN" },          // Right face button (B)
	{  2, "X_BTN" },          // Left face button (X)
	{  3, "Y_BTN" },          // Top face button (Y)
	{  7, "L3_STICK_CLICK" }, // Left stick press (L3)
	{  8, "R3_STICK_CLICK" }, // Right stick press (R3)
	{  9, "L1_SHOULDER" },    // Left shoulder (L1)
	{ 10, "R1_SHOULDER" },    // Right shoulder (R1)
	{ 15, "M1_BTN" },         // M1 button (MISC 1)
	{ 16, "L4_BTN" },         // Left paddle (L4)
	{ 17, "L5_BTN" },         // Left paddle (L5)
	{ 18, "R4_BTN" },         // Right paddle (R4)
	{ 19, "R5_BTN" },         // Right paddle (R5)
	{ 21, "M2_BTN" },         // M2 button (MISC 2)
	{ 22, "M3_BTN" },         // M3 button (MISC 3)
	{ 23, "M4_BTN" },         // M4 button (MISC 4)
	{ 30, "L2_TRIG" },        // Left trigger (L2)
	{ 31, "R2_TRIG" },        // Right trigger (R2)
};

// Xbox-specific overrides
static const struct button_override xbox_specific[] = {
	{ 5,  "XBOX_BTN" },
	{ 9,  "LB_SHOULDER" },
	{ 10, "RB_SHOULDER" },
	{ 30, "LT_TRIG" },
	{ 31, "RT_TRIG" },
};

// Xbox 360 specific button overrides
static const struct button_override xbox360_overrides[] = {
	{ 4, "BACK_BTN" },
	{ 6, "START_BTN" },
};

// Xbox One/Series X|S-specific button overrides
static const struct button_override xboxone_overrides[] = {
	{ 4, "VIEW_BTN" },
	{ 6, "MENU_BTN" },
	{ 15, "SHARE_BTN" },
	{ 16, "PADDLE_1" },     // Xbox Elite right upper paddle
	{ 17, "PADDLE_2" },     // Xbox Elite left upper paddle
	{ 18, "PADDLE_3" },     // Xbox Elite right lower paddle
	{ 19, "PADDLE_4" },     // Xbox Elite left lower paddle
};

// PlayStation-specific overrides
static const struct button_override playstation_specific[] = {
	{ 0, "CROSS_BTN" },
	{ 1, "CIRCLE_BTN" },
	{ 2, "SQUARE_BTN" },
	{ 3, "TRIANGLE_BTN" },
	{ 5, "PS_BTN" },
	{ 20, "TOUCHPAD_BTN" },
};

// PS3-specific button overrides
static const struct button_override ps3_overrides[] = {
	{ 4, "SELECT_BTN" },
	{ 6, "START_BTN" },
};

// PS4-specific button overrides
static const struct button_override ps4_overrides[] = {
	{ 4, "SHARE_BTN" },
	{ 6, "OPTIONS_BTN" },
};

// PS5-specific button overrides
static const struct button_override ps5_overrides[] = {
	{ 4,  "CREATE_BTN" },
	{ 6,  "OPTIONS_BTN" },
	{ 15, "MIC_BTN" },
	{ 16, "RB_PADDLE" },          // DualSense Edge RB Button  
	{ 17, "LB_PADDLE" },          // DualSense Edge LB Button
	{ 18, "FN_RIGHT_BTN" },       // DualSense Edge right function button
	{ 19, "FN_LEFT_BTN" },        // DualSense Edge left function button
};

// Nintendo Switch-specific overrides
static const struct button_override switch_specific[] = {
	{ 4, "MINUS_BTN" },
	{ 5, "HOME_BTN" },
	{ 6, "PLUS_BTN" },
	{ 9, "L_SHOULDER" },
	{ 10, "R_SHOULDER" },
	{ 15, "CAPTURE_BTN" },  
	{ 16, "SL_BTN" },       // Left Joy-Con SL
	{ 17, "SR_BTN" },       // Left Joy-Con SR
	{ 18, "SL_BTN" },       // Right Joy-Con SL
	{ 19, "SR_BTN" },       // Right Joy-Con SR
	{ 30, "ZL_TRIG" },
	{ 31, "ZR_TRIG" },
};

// Function to search an override for a specific button index
static const char* searchOverrides(const struct button_override* overrides, int count, int buttonIndex) {
	for (int i = 0; i < count; i++) {
		if (overrides[i].button_index == buttonIndex) {
			return overrides[i].name;
		}
	}
	return NULL;
}

// Function to get controller-specific button names
const char *glyphGetButtonName(int controllerType, int buttonIndex)
{
	const char *result = NULL;

	switch (controllerType) {
		case CONTROLLER_ICON_XBOX360:
			// Xbox 360-specific overrides
			result = searchOverrides(xbox360_overrides, sizeof(xbox360_overrides) / sizeof(xbox360_overrides[0]), buttonIndex);
			if (result) return result;
			// Xbox overrides
			result = searchOverrides(xbox_specific, sizeof(xbox_specific) / sizeof(xbox_specific[0]), buttonIndex);
			if (result) return result;
			// Glyph standard (Face Button only)
			if (buttonIndex >= 0 && buttonIndex <= 3) {
				result = searchOverrides(glyph_standard, sizeof(glyph_standard) / sizeof(glyph_standard[0]), buttonIndex);
				if (result) return result;
			}
			break;
			
		case CONTROLLER_ICON_XBOXONE:
			// Xbox One/Series X|S-specific overrides
			result = searchOverrides(xboxone_overrides, sizeof(xboxone_overrides) / sizeof(xboxone_overrides[0]), buttonIndex);
			if (result) return result;
			// Xbox overrides
			result = searchOverrides(xbox_specific, sizeof(xbox_specific) / sizeof(xbox_specific[0]), buttonIndex);
			if (result) return result;
			// Glyph standard (Face Button only)
			if (buttonIndex >= 0 && buttonIndex <= 3) {
				result = searchOverrides(glyph_standard, sizeof(glyph_standard) / sizeof(glyph_standard[0]), buttonIndex);
				if (result) return result;
			}
			break;
			
		case CONTROLLER_ICON_PS3:
			// PS3-specific overrides
			result = searchOverrides(ps3_overrides, sizeof(ps3_overrides) / sizeof(ps3_overrides[0]), buttonIndex);
			if (result) return result;
			// PlayStation overrides
			result = searchOverrides(playstation_specific, sizeof(playstation_specific) / sizeof(playstation_specific[0]), buttonIndex);
			if (result) return result;
			// Glyph standard (Face, Shoulders, Triggers only)
			if ((buttonIndex >= 0 && buttonIndex <= 10) || (buttonIndex >= 30 && buttonIndex <= 31)) {
				result = searchOverrides(glyph_standard, sizeof(glyph_standard) / sizeof(glyph_standard[0]), buttonIndex);
				if (result) return result;
			}
			break;
			
		case CONTROLLER_ICON_PS4:
			// PS4-specific overrides
			result = searchOverrides(ps4_overrides, sizeof(ps4_overrides) / sizeof(ps4_overrides[0]), buttonIndex);
			if (result) return result;
			// PlayStation overrides
			result = searchOverrides(playstation_specific, sizeof(playstation_specific) / sizeof(playstation_specific[0]), buttonIndex);
			if (result) return result;
			// Glyph standard (Face, Shoulders, Triggers only)
			if ((buttonIndex >= 0 && buttonIndex <= 10) || (buttonIndex >= 30 && buttonIndex <= 31)) {
				result = searchOverrides(glyph_standard, sizeof(glyph_standard) / sizeof(glyph_standard[0]), buttonIndex);
				if (result) return result;
			}
			break;
			
		case CONTROLLER_ICON_PS5:
			// PS5-specific overrides
			result = searchOverrides(ps5_overrides, sizeof(ps5_overrides) / sizeof(ps5_overrides[0]), buttonIndex);
			if (result) return result;
			// PlayStation overrides
			result = searchOverrides(playstation_specific, sizeof(playstation_specific) / sizeof(playstation_specific[0]), buttonIndex);
			if (result) return result;
			// Glyph standard (Face, Shoulders, Triggers only)
			if ((buttonIndex >= 0 && buttonIndex <= 10) || (buttonIndex >= 30 && buttonIndex <= 31)) {
				result = searchOverrides(glyph_standard, sizeof(glyph_standard) / sizeof(glyph_standard[0]), buttonIndex);
				if (result) return result;
			}
			break;
			
		case CONTROLLER_ICON_NINTENDO_SWITCH:
			// Nintendo Switch specific overrides
			result = searchOverrides(switch_specific, sizeof(switch_specific) / sizeof(switch_specific[0]), buttonIndex);
			if (result) return result;
			// Glyph standard (swapped Face Buttons positions)
			if (buttonIndex >= 0 && buttonIndex <= 3) {
				int mappedIndex = buttonIndex;
				switch (buttonIndex) {
					case 0: mappedIndex = 1; break; // B (Bottom face button)
					case 1: mappedIndex = 0; break; // A (Right face button)
					case 2: mappedIndex = 3; break; // Y (Left face button)
					case 3: mappedIndex = 2; break; // X (Top face button)
				}
				result = searchOverrides(glyph_standard, sizeof(glyph_standard) / sizeof(glyph_standard[0]), mappedIndex);
				if (result) return result;
			}
			break;
			
		case CONTROLLER_ICON_GENERIC:
		default:
			break;
	}

	// Fallback to Generic type
	if (buttonIndex >= 0 && buttonIndex < 32) {
		return vkJoyDisplayNames[buttonIndex];
	}
	
	return "UNKNOWN BUTTON";
}