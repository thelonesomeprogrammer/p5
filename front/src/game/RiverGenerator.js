// Improved infinite river generation with constrained FIFO and bank validation
import { CircularBuffer } from '../utils/CircularBuffer.js';

export class RiverGenerator {
  constructor(mapWidth, options = {}) {
    // Core parameters
    this.mapWidth = mapWidth;
    this.pathHistorySize = options.pathHistorySize || 50;

    // Path data (FIFO) - stores river centerline segments
    this.pathSegments = new CircularBuffer(this.pathHistorySize);
    this.lastGeneratedY = -1;

    // River properties with constraints
    this.riverWidth = {
      min: options.minWidth || 3,
      max: options.maxWidth || 6,
      current: options.initialWidth || 3
    };
    this.centerX = Math.floor(mapWidth / 2);

    this.lastDirection = [0,0]; // -1 left, 0 straight, 1 right
    this.maxTurnRadius = options.maxTurnRadius || 2;

    // Tile cache (limited size with LRU)
    this.tileCache = new Map();
    this.maxCacheSize = options.maxCacheSize || 50;
    this.cacheAccessOrder = []; // For LRU tracking

    // Width change parameters
    this.widthChangeChance = options.widthChangeChance || 0.05;
    this.stepsSinceWidthChange = 0;
    this.minWidthRun = options.minWidthRun || 3;
  }

  // Fixed probability distribution for drunken walk
  getDrunkenStep() {
    const random = Math.random();

    if (this.lastDirection[0] === 0) {
      if (random < 1/3) {
        this.lastDirection[1] = this.lastDirection[0];
        this.lastDirection[0] = -1;
      } else if (random < 2/3) {
        this.lastDirection[1] = this.lastDirection[0];
        this.lastDirection[0] = 0;
      } else {
        this.lastDirection[1] = this.lastDirection[0];
        this.lastDirection[0] = 1;
      }
    } else if (this.lastDirection[0] === -1) {
        if (random < 0.5) {
          this.lastDirection[1] = this.lastDirection[0];
          this.lastDirection[0] = -1;
        } else {
          this.lastDirection[1] = this.lastDirection[0];
          this.lastDirection[0] = 0;
        }
    } else if (this.lastDirection[0] === 1) {
        if (random < 0.5) {
          this.lastDirection[1] = this.lastDirection[0];
          this.lastDirection[0] = 1;
        } else {
          this.lastDirection[1] = this.lastDirection[0];
          this.lastDirection[0] = 0;
        }
      }

    return this.lastDirection[0];
  }



  // Constrain position within map bounds considering river width
  constrainPosition(targetX, width) {
    const halfWidth = Math.floor(width / 2);
    const minX = halfWidth;
    const maxX = this.mapWidth - halfWidth - 1;

    return Math.max(minX, Math.min(maxX, targetX));
  }

  // Generate path segment with validation
  generatePathSegment(y) {
      const direction = this.getDrunkenStep();
      const newCenterX = this.constrainPosition(
        this.centerX + direction,
        this.riverWidth.current
      );

      // Occasionally vary width
      let newWidth = this.riverWidth.current;
      this.stepsSinceWidthChange++;

      if (direction === 0 && this.stepsSinceWidthChange >= this.minWidthRun) {
        if (Math.random() < this.widthChangeChance) {
          const widthChange = Math.random() < 0.5 ? -1 : 1;
          const proposedWidth = Math.max(
            this.riverWidth.min,
            Math.min(this.riverWidth.max, newWidth + widthChange)
          );

          if (proposedWidth !== newWidth) {
            newWidth = proposedWidth;
            this.stepsSinceWidthChange = 0;
          }
        }
      }

      const candidateSegment = {
        y,
        centerX: newCenterX,
        width: newWidth,
        direction
      };

      // Update generator state for next iteration
      this.centerX = newCenterX;
      this.riverWidth.current = newWidth;

      return candidateSegment;
  }

  // Generate tiles from a path segment
  generateTilesFromSegment(segment) {
    const tiles = [];
    const halfWidth = Math.floor(segment.width / 2);

    for (let dx = -halfWidth; dx <= halfWidth; dx++) {
      const tileX = segment.centerX + dx;
      if (tileX >= 0 && tileX < this.mapWidth) {
        tiles.push({ x: tileX, y: segment.y, type: 'water' });
      }
    }

    return tiles;
  }

  // LRU cache management
  manageTileCache(y) {
    // Update access order
    const index = this.cacheAccessOrder.indexOf(y);
    if (index > -1) {
      this.cacheAccessOrder.splice(index, 1);
    }
    this.cacheAccessOrder.push(y);

    // Remove oldest entries if cache is too large
    while (this.tileCache.size > this.maxCacheSize) {
      const oldestY = this.cacheAccessOrder.shift();
      this.tileCache.delete(oldestY);
    }
  }

  // Generate river tiles for a specific Y coordinate
  generateRowAt(y) {
    // Boundary check: No river before the start
    if (y < 0) return [];

    // Safety check: Don't regenerate segments that have dropped out of the history buffer.
    // Regenerating them would use the CURRENT generator state (position/width), creating
    // wildly incorrect segments and corrupting the generator's spatial continuity.
    if (this.lastGeneratedY > -1 && y < (this.lastGeneratedY - this.pathSegments.capacity)) {
      return [];
    }

    // Check tile cache first
    if (this.tileCache.has(y)) {
      this.manageTileCache(y);
      return this.tileCache.get(y);
    }

    // Ensure we have path segments up to this Y coordinate
    this.ensurePathGenerated(y);

    // Find the path segment for this Y
    let segment = null;
    for (let i = 0; i < this.pathSegments.size(); i++) {
      const pathSegment = this.pathSegments.get(i);
      if (pathSegment && pathSegment.y === y) {
        segment = pathSegment;
        break;
      }
    }

    if (!segment) {
      // Generate missing segment
      segment = this.generatePathSegment(y);
      this.pathSegments.push(segment);
    }

    // Generate tiles from segment
    const tiles = this.generateTilesFromSegment(segment);

    // Cache the result
    this.tileCache.set(y, tiles);
    this.manageTileCache(y);
    this.lastGeneratedY = Math.max(this.lastGeneratedY, y);

    return tiles;
  }

  // Ensure path segments are generated up to the specified Y coordinate
  ensurePathGenerated(targetY) {
    while (this.lastGeneratedY < targetY) {
      const nextY = this.lastGeneratedY + 1;
      const segment = this.generatePathSegment(nextY);
      this.pathSegments.push(segment);
      this.lastGeneratedY = nextY;
    }
  }

  // Get all river tiles in a Y range
  getRiverTiles(minY, maxY) {
    // Generate all requested rows
    const tiles = [];
    for (let y = minY; y <= maxY; y++) {
      const rowTiles = this.generateRowAt(y);
      tiles.push(...rowTiles);
    }

    return tiles;
  }

  // Check if a specific position is water
  isWater(x, y) {
    const rowTiles = this.generateRowAt(y);
    return rowTiles.some(tile => tile.x === x);
  }

  // No manual cleanup needed - FIFO and LRU cache handle memory automatically
  cleanup(currentY, keepDistance = 100) {
    // Legacy method kept for compatibility - no operation needed
    // Memory is automatically managed by:
    // - CircularBuffer for path segments (fixed size)
    // - LRU cache for tiles (limited size)
  }

  // Get river statistics for debugging
  getStats() {
    return {
      pathSegments: this.pathSegments.size(),
      cachedTiles: this.tileCache.size,
      lastGeneratedY: this.lastGeneratedY,
      currentWidth: this.riverWidth.current,
      centerX: this.centerX
    };
  }

}
