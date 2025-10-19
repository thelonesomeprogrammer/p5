// Infinite river generation using drunken walk algorithm

export class RiverGenerator {
  constructor(width) {
    this.width = width;
    this.currentX = Math.floor(width / 2);
    this.lastGeneratedY = -1;
    this.generatedTiles = new Map(); // Y -> array of tiles
    this.riverWidth = 3;

    // Drunken walk parameters
    this.walkBias = 0.1; // Tendency to continue in same direction
    this.lastDirection = 0; // -1 left, 0 straight, 1 right
  }

  // Drunken walk step - decides next direction
  getDrunkenStep() {
    const random = Math.random();

    // Bias towards continuing in the same direction
    if (random < this.walkBias) {
      return this.lastDirection;
    }

    // Random walk with equal probability
    if (random < 0.33) {
      this.lastDirection = -1; // Left
    } else if (random < 0.66) {
      this.lastDirection = 0;  // Straight
    } else {
      this.lastDirection = 1;  // Right
    }

    return this.lastDirection;
  }

  // Generate river tiles for a specific Y coordinate
  generateRowAt(y) {
    if (this.generatedTiles.has(y)) {
      return this.generatedTiles.get(y);
    }

    // Apply drunken walk step
    const direction = this.getDrunkenStep();
    this.currentX += direction * 0.7; // Move less aggressively

    // Keep river within bounds
    const halfWidth = Math.floor(this.riverWidth / 2);
    this.currentX = Math.max(halfWidth, Math.min(this.width - halfWidth - 1, this.currentX));

    // Occasionally vary river width for interest
    if (Math.random() < 0.05) {
      this.riverWidth = Math.max(2, Math.min(5, this.riverWidth + (Math.random() < 0.5 ? -1 : 1)));
    }

    // Generate water tiles for this row
    const tiles = [];
    const actualHalfWidth = Math.floor(this.riverWidth / 2);

    for (let dx = -actualHalfWidth; dx <= actualHalfWidth; dx++) {
      const tileX = Math.floor(this.currentX) + dx;
      if (tileX >= 0 && tileX < this.width) {
        tiles.push({ x: tileX, y, type: 'water' });
      }
    }

    this.generatedTiles.set(y, tiles);
    this.lastGeneratedY = Math.max(this.lastGeneratedY, y);

    return tiles;
  }

  // Get all river tiles in a Y range
  getRiverTiles(minY, maxY) {
    // First generate all rows
    for (let y = minY; y <= maxY; y++) {
      this.generateRowAt(y);
    }

    // Apply smoothing to remove single block indentations
    this.smoothRiverTiles(minY, maxY);

    // Collect all tiles
    const tiles = [];
    for (let y = minY; y <= maxY; y++) {
      if (this.generatedTiles.has(y)) {
        tiles.push(...this.generatedTiles.get(y));
      }
    }

    return tiles;
  }

  // Post-process tiles to remove single block indentations (fill gaps)
  // Keep protrusions since they can be handled with edge sprites
  smoothRiverTiles(minY, maxY) {
    for (let y = minY; y <= maxY; y++) {
      if (!this.generatedTiles.has(y)) continue;

      const currentRow = this.generatedTiles.get(y);
      const newTiles = [...currentRow];

      // Convert to set for easier lookup
      const currentWater = new Set(currentRow.map(t => t.x));

      // Get min and max X for current row to define the river bounds
      const minX = Math.min(...currentRow.map(t => t.x));
      const maxX = Math.max(...currentRow.map(t => t.x));

      // Look for single-block indentations (gaps) within the river bounds
      for (let x = minX + 1; x < maxX; x++) {
        if (!currentWater.has(x)) {
          // Check if this is a single-block gap (has water on both sides)
          const hasWaterLeft = currentWater.has(x - 1);
          const hasWaterRight = currentWater.has(x + 1);

          // Fill single block indentations
          if (hasWaterLeft && hasWaterRight) {
            newTiles.push({ x, y, type: 'water' });
          }
        }
      }

      this.generatedTiles.set(y, newTiles);
    }
  }

  // Check if a specific position is water
  isWater(x, y) {
    const rowTiles = this.generateRowAt(y);
    return rowTiles.some(tile => tile.x === x);
  }

  // Clean up old generated data to save memory
  cleanup(currentY, keepDistance = 100) {
    const keysToDelete = [];
    for (const y of this.generatedTiles.keys()) {
      if (y < currentY - keepDistance) {
        keysToDelete.push(y);
      }
    }
    keysToDelete.forEach(y => this.generatedTiles.delete(y));
  }

}