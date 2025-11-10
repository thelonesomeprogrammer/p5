// Circular buffer implementation for efficient FIFO operations
export class CircularBuffer {
  constructor(capacity) {
    this.capacity = capacity;
    this.buffer = new Array(capacity);
    this.head = 0;  // Points to the next write position
    this.tail = 0;  // Points to the oldest element
    this.count = 0; // Number of elements currently in buffer
  }

  // Add element to the buffer (overwrites oldest if full)
  push(element) {
    this.buffer[this.head] = element;
    this.head = (this.head + 1) % this.capacity;

    if (this.count < this.capacity) {
      this.count++;
    } else {
      // Buffer is full, move tail forward (overwrite oldest)
      this.tail = (this.tail + 1) % this.capacity;
    }
  }

  // Get element at index (0 = oldest, size()-1 = newest)
  get(index) {
    if (index < 0 || index >= this.count) {
      return undefined;
    }
    const actualIndex = (this.tail + index) % this.capacity;
    return this.buffer[actualIndex];
  }

  // Get the newest element
  getLast() {
    if (this.count === 0) return undefined;
    const lastIndex = (this.head - 1 + this.capacity) % this.capacity;
    return this.buffer[lastIndex];
  }

  // Get multiple recent elements (newest first)
  getRecent(count) {
    const result = [];
    const actualCount = Math.min(count, this.count);

    for (let i = 0; i < actualCount; i++) {
      const index = (this.head - 1 - i + this.capacity) % this.capacity;
      result.push(this.buffer[index]);
    }

    return result;
  }

  // Current number of elements
  size() {
    return this.count;
  }

  // Check if buffer is empty
  isEmpty() {
    return this.count === 0;
  }

  // Check if buffer is full
  isFull() {
    return this.count === this.capacity;
  }

  // Clear all elements
  clear() {
    this.head = 0;
    this.tail = 0;
    this.count = 0;
  }
}