import { RiverGenerator } from "./RiverGenerator.js";

export class GameState {
	constructor() {
		this.player = {
			x: 10, // Grid position
			y: 0,
			targetX: 10, // For smooth movement
			targetY: 0,
			visualX: 10, // For smooth visual interpolation
			visualY: 0,
			direction: 0, // 0 = north, 1 = east, 2 = south, 3 = west
			crashed: false, // Whether player has hit dirt
			crashedAtY: 0, // Y position where player crashed
		};

		this.river = {
			width: 20,
			generator: new RiverGenerator(20, {
				pathHistorySize: 200,
				maxCacheSize: 50,
				minWidth: 2,
				maxWidth: 6,
				initialWidth: 3,
				walkBias: 0.1,
				validateBanks: true,
				maxGenerationAttempts: 5,
				widthChangeChance: 0.05
			}),
		};

		this.viewport = {
			x: 0,
			y: 0,
			targetX: 0, // For smooth horizontal scrolling
			targetY: 0, // For smooth scrolling
			width: 800,
			height: 600,
		};

		// Auto-movement settings
		this.autoMove = true;
		this.moveSpeed = 1.2; // Starting tiles per second - slower for smoother movement
		this.baseMoveSpeed = 1.2; // Base speed for calculation
		this.maxMoveSpeed = 20.0; // Maximum speed cap (increased)
		this.speedIncreaseRate = 0.1; // How much speed increases per second (increased for more noticeable effect)
		this.lastMoveTime = 0;

		// Smooth interpolation settings
		this.lerpSpeed = 0.15; // How fast to interpolate positions

		// Score tracking
		this.distance = 0; // How far the player has traveled
		this.gameTime = 0; // Total game time in seconds
		this.lastLoggedTime = -1; // For debug logging

		// Game state
		this.gameOver = false;
		this.gameOverDistance = 4; // Distance camera continues after player crashes

		this.initializePlayer();
	}

	// Initialize player position on the river
	initializePlayer() {
		// Generate initial river tiles to find a starting position
		const startRowTiles = this.river.generator.generateRowAt(0);
		if (startRowTiles.length > 0) {
			this.player.x = startRowTiles[Math.floor(startRowTiles.length / 2)].x;
			this.player.targetX = this.player.x;
			this.player.visualX = this.player.x;
			this.player.visualY = this.player.y;
		}
	}

	// Check if a position is water
	isWater(x, y) {
		return this.river.generator.isWater(x, y);
	}

	// Update game state (called each frame)
	update(deltaTime) {
		this.lastMoveTime += deltaTime;
		this.gameTime += deltaTime / 1000; // Convert milliseconds to seconds

		// Gradually increase speed over time
		const speedIncrease = this.gameTime * this.speedIncreaseRate;
		this.moveSpeed = Math.min(
			this.baseMoveSpeed + speedIncrease,
			this.maxMoveSpeed,
		);

		// Debug logging every 5 seconds
		if (
			Math.floor(this.gameTime) % 5 === 0 &&
			Math.floor(this.gameTime) !== this.lastLoggedTime
		) {
			console.log(
				`Time: ${this.gameTime.toFixed(1)}s, Speed: ${this.moveSpeed.toFixed(2)}, Distance: ${this.distance}`,
			);
			this.lastLoggedTime = Math.floor(this.gameTime);
		}

		// Auto-move forward
		if (this.autoMove && this.lastMoveTime >= 1000 / this.moveSpeed) {
			// Check if player would collide with dirt at next position
			const nextY = this.player.y + 1;
			if (!this.isWater(this.player.x, nextY)) {
				// Collision! Player stays at current position, camera continues
				console.log(`Player hit dirt at (${this.player.x}, ${nextY}), staying at y=${this.player.y}`);
			} else {
				// Safe to move player forward
				this.player.y += 1;
				this.player.targetY = this.player.y;
			}

			this.distance += 1; // Camera always moves forward
			this.lastMoveTime = 0;

			// Check if player has fallen too far behind camera (game over)
			const relativePlayerY = this.player.y - (this.distance - this.gameOverDistance);
			if (relativePlayerY < -this.gameOverDistance) {
				this.gameOver = true;
				console.log('Game Over! Player fell too far behind.');
			}

			// Clean up old river data
			this.river.generator.cleanup(this.distance);
		}

		// Smooth interpolation for player position
		this.player.visualX +=
			(this.player.targetX - this.player.visualX) * this.lerpSpeed;
		this.player.visualY +=
			(this.player.targetY - this.player.visualY) * this.lerpSpeed;

		// Camera follows the distance (independent of player position)
		const tileSize = 32;
		const targetViewportX = this.player.visualX * tileSize - this.viewport.width / 2;
		const targetViewportY = this.distance * tileSize - this.viewport.height / 2;

		this.viewport.targetX = targetViewportX;
		this.viewport.targetY = targetViewportY;

		// Lerp viewport for smooth movement
		this.viewport.x +=
			(this.viewport.targetX - this.viewport.x) * this.lerpSpeed;
		this.viewport.y +=
			(this.viewport.targetY - this.viewport.y) * this.lerpSpeed;
	}

	// Move player horizontally (player can only move left/right)
	movePlayer(dx, dy) {
		// Only allow horizontal movement
		if (dy !== 0) return false;

		const newX = this.player.x + dx;

		// Check if new position is valid (within bounds and on water)
		if (
			newX >= 0 &&
			newX < this.river.width &&
			this.isWater(newX, this.player.y)
		) {
			this.player.x = newX;
			this.player.targetX = newX;
			return true;
		}
		return false;
	}




	// Get current game stats for display
	getStats() {
		return {
			distance: Math.floor(this.distance),
			speed: this.moveSpeed.toFixed(2), // Show 2 decimal places for more precision
			time: Math.floor(this.gameTime),
		};
	}

	// Check if game is over
	isGameOver() {
		return this.gameOver;
	}

	// Reset game to initial state
	resetGame() {
		this.player = {
			x: 10,
			y: 0,
			targetX: 10,
			targetY: 0,
			visualX: 10,
			visualY: 0,
			direction: 0,
			crashed: false,
			crashedAtY: 0,
		};

		this.viewport = {
			x: 0,
			y: 0,
			targetX: 0,
			targetY: 0,
			width: 800,
			height: 600,
		};

		this.autoMove = true;
		this.moveSpeed = this.baseMoveSpeed;
		this.lastMoveTime = 0;
		this.distance = 0;
		this.gameTime = 0;
		this.lastLoggedTime = -1;
		this.gameOver = false;

		// Reset river generator with same configuration
		this.river.generator = new RiverGenerator(this.river.width, {
			pathHistorySize: 200,
			maxCacheSize: 50,
			minWidth: 2,
			maxWidth: 6,
			initialWidth: 3,
			walkBias: 0.1,
			validateBanks: true,
			maxGenerationAttempts: 5,
			widthChangeChance: 0.05
		});

		this.initializePlayer();
	}
}
