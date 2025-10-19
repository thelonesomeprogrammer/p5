// Utility functions for coordinate conversions

export const TILE_SIZE = 16;
export const SCALE = 2;
export const SCALED_TILE_SIZE = TILE_SIZE * SCALE;

// Convert grid coordinates to pixel coordinates
export const gridToPixel = (gridX, gridY) => ({
  x: gridX * SCALED_TILE_SIZE,
  y: gridY * SCALED_TILE_SIZE
});

// Convert pixel coordinates to grid coordinates
export const pixelToGrid = (pixelX, pixelY) => ({
  x: Math.floor(pixelX / SCALED_TILE_SIZE),
  y: Math.floor(pixelY / SCALED_TILE_SIZE)
});

// Convert u8 backend data to grid coordinates
export const u8ToGrid = (u8X, u8Y, gridWidth, gridHeight) => ({
  x: Math.floor((u8X / 255) * gridWidth),
  y: Math.floor((u8Y / 255) * gridHeight)
});

// Convert grid coordinates to u8 backend data
export const gridToU8 = (gridX, gridY, gridWidth, gridHeight) => ({
  x: Math.floor((gridX / gridWidth) * 255),
  y: Math.floor((gridY / gridHeight) * 255)
});

// Check if two positions are adjacent (for navigation validation)
export const areAdjacent = (pos1, pos2) => {
  const dx = Math.abs(pos1.x - pos2.x);
  const dy = Math.abs(pos1.y - pos2.y);
  return (dx <= 1 && dy <= 1) && (dx + dy > 0);
};