// Sprite coordinate mappings from spritesheetInfo.txt
// Grid calculation: Position = (tileX * 17, tileY * 17) due to 16px tiles + 1px margin

export const SPRITE_SIZE = 16;
export const SPRITE_MARGIN = 1;
export const SPRITE_STRIDE = SPRITE_SIZE + SPRITE_MARGIN;

export const SPRITE_COORDS = {
	// Water tiles
	WATER_1: { x: 0, y: 0 },
	WATER_2: { x: 1, y: 0 },
	WATER_3: { x: 3, y: 1 }, // Original coordinates (but not used in rendering)

	// Water corners (outer)
	WATER_CORNER_OUT_BOTTOM_RIGHT: { x: 0, y: 1 },
	WATER_CORNER_OUT_BOTTOM_LEFT: { x: 1, y: 1 },
	WATER_CORNER_OUT_TOP_RIGHT: { x: 0, y: 2 },
	WATER_CORNER_OUT_TOP_LEFT: { x: 1, y: 2 },

	// Water edges and corners (inner)
	WATER_CORNER_IN_TOP_LEFT: { x: 2, y: 0 },
	WATER_TOP: { x: 3, y: 0 },
	WATER_CORNER_IN_TOP_RIGHT: { x: 4, y: 0 },
	WATER_LEFT: { x: 2, y: 1 },
	WATER_RIGHT: { x: 4, y: 1 },
	WATER_CORNER_IN_BOTTOM_LEFT: { x: 2, y: 2 },
	WATER_BOTTOM: { x: 3, y: 2 },
	WATER_CORNER_IN_BOTTOM_RIGHT: { x: 4, y: 2 },

	// Terrain
	GRASS_1: { x: 5, y: 0 },
	GRASS_2: { x: 5, y: 1 },
	DIRT_1: { x: 6, y: 0 },
	DIRT_2: { x: 6, y: 1 },

	// Player/Boat
	PLAYER: { x: 25, y: 0 },
};

// Convert grid coordinates to pixel coordinates
export const gridToPixel = (gridX, gridY) => ({
	x: gridX * SPRITE_STRIDE,
	y: gridY * SPRITE_STRIDE,
});

// Get pixel coordinates for a sprite
export const getSpritePixelCoords = (spriteName) => {
	const coords = SPRITE_COORDS[spriteName];
	if (!coords) {
		console.warn(`Sprite ${spriteName} not found`);
		return { x: 0, y: 0 };
	}
	return gridToPixel(coords.x, coords.y);
};

