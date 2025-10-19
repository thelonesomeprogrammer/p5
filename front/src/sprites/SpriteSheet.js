import { SPRITE_SIZE, getSpritePixelCoords } from './spriteCoords.js';

export class SpriteSheet {
  constructor(imagePath) {
    this.image = new Image();
    this.loaded = false;
    this.sprites = new Map();

    return new Promise((resolve, reject) => {
      this.image.onload = () => {
        this.loaded = true;
        console.log('Spritesheet loaded successfully');
        resolve(this);
      };

      this.image.onerror = () => {
        console.error('Failed to load spritesheet');
        reject(new Error('Failed to load spritesheet'));
      };

      this.image.src = imagePath;
    });
  }

  // Pre-cut a sprite from the sheet and cache it
  cutSprite(spriteName) {
    if (this.sprites.has(spriteName)) {
      return this.sprites.get(spriteName);
    }

    if (!this.loaded) {
      console.warn('Spritesheet not loaded yet');
      return null;
    }

    const coords = getSpritePixelCoords(spriteName);

    // Create a canvas for this sprite
    const canvas = document.createElement('canvas');
    canvas.width = SPRITE_SIZE;
    canvas.height = SPRITE_SIZE;
    const ctx = canvas.getContext('2d');

    // Draw the sprite from the main sheet
    ctx.drawImage(
      this.image,
      coords.x, coords.y, SPRITE_SIZE, SPRITE_SIZE,
      0, 0, SPRITE_SIZE, SPRITE_SIZE
    );

    this.sprites.set(spriteName, canvas);
    return canvas;
  }

  // Get a sprite (cut it if not already cached)
  getSprite(spriteName) {
    return this.cutSprite(spriteName);
  }

  // Pre-cut all sprites we'll need
  preloadSprites(spriteNames) {
    spriteNames.forEach(name => this.cutSprite(name));
  }
}