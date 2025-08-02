#ifndef _IN_GLYPH_H
#define _IN_GLYPH_H

// Controller Icon enumeration for button icon detection
typedef enum {
	CONTROLLER_ICON_GENERIC,
	CONTROLLER_ICON_XBOX360,
	CONTROLLER_ICON_XBOXONE,
	CONTROLLER_ICON_PS3,
	CONTROLLER_ICON_PS4,
	CONTROLLER_ICON_PS5,
	CONTROLLER_ICON_NINTENDO_SWITCH,
	CONTROLLER_ICON_STEAM_CONTROLLER,
	CONTROLLER_ICON_STEAM_DECK,
} ControllerIconType;

// Generic display names for controller buttons
extern const char *vkJoyDisplayNames[];

// Function to get controller-specific button name with fallback to generic
const char *glyphGetButtonName(int controllerType, int buttonIndex);

#endif