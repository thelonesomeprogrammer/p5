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



  // Set canvas size
  setSize(width, height) {
    this.canvas.width = width;
    this.canvas.height = height;
  }
}