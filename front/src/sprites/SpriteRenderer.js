import { SPRITE_SIZE } from './spriteCoords.js';

export class SpriteRenderer {
  constructor(canvas, spriteSheet) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.spriteSheet = spriteSheet;
    this.scale = 2; // Scale sprites 2x for better visibility
  }

  // Clear the canvas
  clear() {
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
  }

  // Render a sprite at the given position
  drawSprite(spriteName, x, y, scale = this.scale) {
    const sprite = this.spriteSheet.getSprite(spriteName);
    if (!sprite) {
      console.warn(`Could not render sprite: ${spriteName}`);
      return;
    }

    const size = SPRITE_SIZE * scale;
    this.ctx.drawImage(sprite, x, y, size, size);
  }

  // Render a grid-based sprite (coordinates in grid units)
  drawSpriteGrid(spriteName, gridX, gridY, scale = this.scale) {
    const pixelX = gridX * SPRITE_SIZE * scale;
    const pixelY = gridY * SPRITE_SIZE * scale;
    this.drawSprite(spriteName, pixelX, pixelY, scale);
  }

  // Fill an area with a repeating sprite pattern
  fillArea(spriteName, x, y, width, height, scale = this.scale) {
    const spriteSize = SPRITE_SIZE * scale;
    const sprite = this.spriteSheet.getSprite(spriteName);

    if (!sprite) return;

    for (let py = y; py < y + height; py += spriteSize) {
      for (let px = x; px < x + width; px += spriteSize) {
        this.ctx.drawImage(sprite, px, py, spriteSize, spriteSize);
      }
    }
  }

  // Set canvas size
  setSize(width, height) {
    this.canvas.width = width;
    this.canvas.height = height;
  }
}