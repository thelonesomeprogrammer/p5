/**
 * HardwareInputManager - Fetches and processes hardware controller data from backend
 *
 * The backend provides lines in format:
 * "; vel: 0.00; acc: 0.00top: -0.00, -0.00, -0.00; but: -0.01, -0.00; des: 0.01; rad: 1.04; integral: 0.50; con: 13"
 *
 * rad value interpretation:
 * - 1.0 = straight/center
 * - 0.5 = max left
 * - 1.5 = max right
 */
export class HardwareInputManager {
  constructor(baseUrl = '/api') {
    this.baseUrl = baseUrl;
    this.lastRadValue = 1.0; // Default to center
    this.isConnected = false;
    this.errorCount = 0;
    this.lastUpdateTime = 0;
  }

  /**
   * Parse a line from the hardware backend to extract rad value
   * @param {string} line - Raw line from backend
   * @returns {number|null} - Parsed rad value or null if parsing failed
   */
  parseRadValue(line) {
    try {
      // Look for "rad: X.XX" pattern
      const radMatch = line.match(/rad:\s*([-+]?\d*\.?\d+)/);
      if (radMatch && radMatch[1]) {
        const radValue = parseFloat(radMatch[1]);
        console.log(`[Hardware] Parsed rad: ${radValue.toFixed(2)} from line: ${line}`);
        return radValue;
      }
      console.warn('[Hardware] Could not parse rad value from:', line);
      return null;
    } catch (error) {
      console.error('[Hardware] Parse error:', error);
      return null;
    }
  }

  /**
   * Fetch a single line from the hardware backend
   * @returns {Promise<number|null>} - rad value or null if failed
   */
  async fetchRadValue() {
    try {
      const response = await fetch(`${this.baseUrl}/line`);

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const line = await response.text();
      const radValue = this.parseRadValue(line);

      if (radValue !== null) {
        this.lastRadValue = radValue;
        this.isConnected = true;
        this.errorCount = 0;
        this.lastUpdateTime = Date.now();
        return radValue;
      }

      return null;
    } catch (error) {
      this.errorCount++;
      this.isConnected = false;

      // Only log every 10th error to avoid spam
      if (this.errorCount % 10 === 1) {
        console.error(`[Hardware] Connection failed (error #${this.errorCount}):`, error.message);
      }

      return null;
    }
  }

  /**
   * Map rad value to game movement direction
   * @param {number} radValue - Raw rad value (0.5 to 1.5 range)
   * @returns {number} - Direction: -1 (left), 0 (center), 1 (right)
   */
  getMovementDirection(radValue) {
    // rad: 1.0 is center (±0.5 range)
    // Map to left/center/right with deadzone
    const offset = radValue - 1.0; // Convert to -0.5 to +0.5 range

    const leftThreshold = -0.15;  // Tilt left threshold
    const rightThreshold = 0.15;  // Tilt right threshold

    if (offset < leftThreshold) {
      console.log(`[Hardware] rad=${radValue.toFixed(2)} offset=${offset.toFixed(2)} → LEFT`);
      return -1; // Move left
    } else if (offset > rightThreshold) {
      console.log(`[Hardware] rad=${radValue.toFixed(2)} offset=${offset.toFixed(2)} → RIGHT`);
      return 1; // Move right
    } else {
      console.log(`[Hardware] rad=${radValue.toFixed(2)} offset=${offset.toFixed(2)} → CENTER (deadzone)`);
      return 0; // Center/no movement
    }
  }

  /**
   * Get connection status for UI display
   * @returns {object} - Status info
   */
  getStatus() {
    const timeSinceUpdate = Date.now() - this.lastUpdateTime;
    const isStale = timeSinceUpdate > 1000; // Consider stale after 1 second

    return {
      connected: this.isConnected && !isStale,
      lastRadValue: this.lastRadValue,
      errorCount: this.errorCount,
      timeSinceUpdate,
    };
  }
}
