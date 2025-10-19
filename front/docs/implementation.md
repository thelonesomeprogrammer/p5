# Implementation Details

## Architecture Overview

### Frontend Structure
- **React 19** with modern hooks and concurrent features
- **Rsbuild** for fast builds and development
- **Canvas/SVG rendering** for river visualization (to be implemented)

### Data Flow
1. Backend sends position data as u8 encoding
2. Frontend decodes position to coordinates
3. Path generation algorithm creates river segments
4. Rendering engine displays river and boat position

## Core Components (To Be Implemented)

### River System
- `RiverCanvas` - Main canvas component for river rendering
- `PathGenerator` - Algorithm for generating river segments
- `PositionDecoder` - Converts u8 data to coordinates

### Game Logic
- `GameState` - Manages current game state
- `BoatController` - Handles boat movement and physics
- `CollisionDetection` - River boundary collision detection

### Communication
- `WebSocketClient` - Real-time communication with backend
- `PositionHandler` - Processes incoming position updates

## Technical Considerations

### Position Encoding
- Backend uses u8 (0-255) for position data
- Need to map to screen coordinates
- Consider river width and boundaries

### Path Generation
- Procedural river generation algorithm
- Smooth curves and realistic flow
- Adjustable difficulty (width, turns)

### Performance
- Canvas optimization for smooth 60fps
- Efficient path rendering
- Memory management for long rivers

### State Management
- React Context for game state
- Local state for rendering optimizations
- WebSocket connection management

## File Structure (Planned)
```
src/
├── components/
│   ├── River/
│   │   ├── RiverCanvas.jsx
│   │   └── PathGenerator.js
│   ├── Boat/
│   │   └── BoatController.jsx
│   └── UI/
│       └── GameHUD.jsx
├── hooks/
│   ├── useWebSocket.js
│   └── useGameState.js
├── utils/
│   ├── positionDecoder.js
│   └── pathAlgorithms.js
└── constants/
    └── gameConfig.js
```