# Game Details

## Game Concept: River Sailing

### Core Gameplay
Players navigate a boat down a procedurally generated river, avoiding obstacles and staying within the river boundaries.

### Game Mechanics

#### Boat Movement
- Controlled movement left/right across river width
- Forward momentum handled by river current
- Physics-based movement with momentum and drift

#### River System
- Procedurally generated river path
- Varying width and curvature
- Natural flow and current effects
- Obstacles (rocks, fallen trees, narrow passages)

#### Scoring/Objectives
- Distance traveled down the river
- Time-based challenges
- Smooth navigation bonuses
- Penalty for collisions

### Visual Design

#### River Appearance
- Top-down or angled perspective
- Water texture with flow indicators
- River banks with natural vegetation
- Depth variations and shallows

#### Boat Design
- Simple sailboat sprite
- Sail animation based on wind/movement
- Wake effects behind boat
- Collision feedback (visual/audio)

### Technical Specifications

#### Position System
- Backend provides u8 position (0-255)
- Maps to river width coordinates
- Additional data for river curve direction
- Smooth interpolation between updates

#### Path Generation Algorithm
- Perlin noise for natural curves
- Configurable parameters:
  - River width (min/max)
  - Curve intensity
  - Segment length
  - Obstacle density

#### Difficulty Progression
- Gradually narrowing river
- Increased curve frequency
- More obstacles over time
- Faster current speed

### User Interface

#### HUD Elements
- Distance counter
- Current speed indicator
- Minimap of upcoming river section
- Score/time display

#### Controls
- Keyboard: Arrow keys or WASD
- Mouse: Click and drag
- Touch: Swipe gestures (mobile)

### Audio Design
- Water flow ambient sounds
- Wind effects for sailing
- Collision sound effects
- Success/achievement audio cues

### Game States
1. **Menu** - Start game, settings, high scores
2. **Playing** - Active gameplay
3. **Paused** - Game paused overlay
4. **Game Over** - Results and restart option

### Future Features
- Multiple boat types with different handling
- Weather effects (wind patterns, storms)
- Multiplayer racing mode
- River customization/editor
- Achievement system