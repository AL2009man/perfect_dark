#ifndef _IN_GLYPH_H
#define _IN_GLYPH_H

#include <SDL.h>

// Controller Icon enumeration for button icon detection
typedef enum {
	CONTROLLER_ICON_GENERIC,
	CONTROLLER_ICON_XBOX360,
	CONTROLLER_ICON_XBOXONE,
	CONTROLLER_ICON_PS3,
	CONTROLLER_ICON_PS4,
	CONTROLLER_ICON_PS5,
	CONTROLLER_ICON_NINTENDO_SWITCH,
	CONTROLLER_ICON_NINTENDO_64,
	CONTROLLER_ICON_STEAM_CONTROLLER,
	CONTROLLER_ICON_STEAM_DECK,
} ControllerIconType;

// Steam/Valve Corporation VID/PID definitions
#define VALVE_VENDOR_ID 0x28de

// Steam Controller product IDs (all generations)
#define STEAM_CONTROLLER_LEGACY_PID     0x1101  // Valve Legacy Steam Controller (CHELL)
#define STEAM_CONTROLLER_WIRED_PID      0x1102  // Valve wired Steam Controller (D0G)
#define STEAM_CONTROLLER_BT_1_PID       0x1105  // Valve Bluetooth Steam Controller (D0G)
#define STEAM_CONTROLLER_BT_2_PID       0x1106  // Valve Bluetooth Steam Controller (D0G)
#define STEAM_CONTROLLER_WIRELESS_PID   0x1142  // Valve wireless Steam Controller
#define STEAM_CONTROLLER_V2_WIRED_PID   0x1201  // Valve wired Steam Controller (HEADCRAB)
#define STEAM_CONTROLLER_V2_BT_PID      0x1202  // Valve Bluetooth Steam Controller (HEADCRAB)

// Other Steam/Valve product IDs
#define STEAM_VIRTUAL_GAMEPAD_PID       0x11ff  // Steam Virtual Gamepad
#define STEAM_DECK_BUILTIN_PID          0x1205  // Valve Steam Deck Builtin

// Helper function to check if a product ID is any Steam Controller variant
static inline int isSteamControllerPID(unsigned short productId) {
	return (productId == STEAM_CONTROLLER_LEGACY_PID ||
	        productId == STEAM_CONTROLLER_WIRED_PID ||
	        productId == STEAM_CONTROLLER_BT_1_PID ||
	        productId == STEAM_CONTROLLER_BT_2_PID ||
	        productId == STEAM_CONTROLLER_WIRELESS_PID ||
	        productId == STEAM_CONTROLLER_V2_WIRED_PID ||
	        productId == STEAM_CONTROLLER_V2_BT_PID);
}

// Generic display names for controller buttons
extern const char *vkJoyDisplayNames[];

// Function to get controller-specific button name with fallback to generic
const char *glyphGetControllerButtonName(int controllerType, int buttonIndex);

// Function to replace baked-in button text with dynamic controller bindings
char* glyphInputBindingDetect(char* text, int textSize, const char* placeholder, 
                                       int controllerIndex, int controlKey, int forceController);

#endif