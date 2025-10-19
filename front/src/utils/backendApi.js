// Backend API integration for position data

export class BackendApi {
  constructor(baseUrl = 'http://localhost:8080') {
    this.baseUrl = baseUrl;
    this.isConnected = false;
  }

  // Convert u8 position data to game coordinates
  decodePosition(u8Data) {
    return {
      x: u8Data.x / 255,
      y: u8Data.y / 255
    };
  }

  // Convert game coordinates to u8 position data
  encodePosition(normalizedPos) {
    return {
      x: Math.floor(normalizedPos.x * 255),
      y: Math.floor(normalizedPos.y * 255)
    };
  }

  // Get current position from backend
  async getCurrentPosition() {
    try {
      const response = await fetch(`${this.baseUrl}/api/position`);
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      return this.decodePosition(data);
    } catch (error) {
      console.warn('Failed to fetch position from backend:', error);
      return null;
    }
  }

  // Send position update to backend
  async updatePosition(normalizedPos) {
    try {
      const u8Pos = this.encodePosition(normalizedPos);
      const response = await fetch(`${this.baseUrl}/api/position`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(u8Pos)
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return true;
    } catch (error) {
      console.warn('Failed to update position to backend:', error);
      return false;
    }
  }

  // Subscribe to position updates via WebSocket
  subscribeToPositionUpdates(callback) {
    try {
      const wsUrl = this.baseUrl.replace('http', 'ws') + '/ws/position';
      const ws = new WebSocket(wsUrl);

      ws.onopen = () => {
        console.log('Connected to backend WebSocket');
        this.isConnected = true;
      };

      ws.onmessage = (event) => {
        try {
          const u8Data = JSON.parse(event.data);
          const normalizedPos = this.decodePosition(u8Data);
          callback(normalizedPos);
        } catch (error) {
          console.error('Failed to parse WebSocket message:', error);
        }
      };

      ws.onclose = () => {
        console.log('Disconnected from backend WebSocket');
        this.isConnected = false;
      };

      ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        this.isConnected = false;
      };

      return ws;
    } catch (error) {
      console.error('Failed to connect to WebSocket:', error);
      return null;
    }
  }

  // Health check
  async checkConnection() {
    try {
      const response = await fetch(`${this.baseUrl}/api/health`);
      this.isConnected = response.ok;
      return this.isConnected;
    } catch (error) {
      this.isConnected = false;
      return false;
    }
  }
}