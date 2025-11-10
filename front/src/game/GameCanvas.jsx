import { useEffect, useRef, useState } from "react";
import { SpriteSheet } from "../sprites/SpriteSheet.js";
import { SpriteRenderer } from "../sprites/SpriteRenderer.js";
import { GameState } from "./GameState.js";

const GameCanvas = () => {
	const canvasRef = useRef(null);
	const [gameState] = useState(() => {
		const state = new GameState();
		console.log('GameState initialized:', state);
		console.log('Player position:', state.player);
		return state;
	});
	const [spriteSheet, setSpriteSheet] = useState(null);
	const [renderer, setRenderer] = useState(null);
	const [showInstructions, setShowInstructions] = useState(true);
	const [showGameOver, setShowGameOver] = useState(false);
	const [finalScore, setFinalScore] = useState({ distance: 0, speed: '1.2', time: 0 });
	const [stats, setStats] = useState({ distance: 0, speed: '1.2', time: 0 });
	const animationFrameRef = useRef();
	const lastTimeRef = useRef(0);

	// Initialize sprite system
	useEffect(() => {
		const initSprites = async () => {
			try {
				console.log('Starting sprite initialization...');
				const sheet = await new SpriteSheet(
					"/assets/Spritesheet/roguelikeSheet_transparent.png",
				);

				console.log('Spritesheet loaded, preloading sprites...');
				// Preload all sprites we'll need
				sheet.preloadSprites([
					"WATER_1",
					"WATER_2",
					"WATER_TOP",
					"WATER_BOTTOM",
					"WATER_LEFT",
					"WATER_RIGHT",
					"WATER_CORNER_IN_TOP_LEFT",
					"WATER_CORNER_IN_TOP_RIGHT",
					"WATER_CORNER_IN_BOTTOM_LEFT",
					"WATER_CORNER_IN_BOTTOM_RIGHT",
					"WATER_CORNER_OUT_TOP_LEFT",
					"WATER_CORNER_OUT_TOP_RIGHT",
					"WATER_CORNER_OUT_BOTTOM_LEFT",
					"WATER_CORNER_OUT_BOTTOM_RIGHT",
					"GRASS_1",
					"GRASS_2",
					"DIRT_1",
					"DIRT_2",
					"PLAYER",
				]);

				setSpriteSheet(sheet);

				if (canvasRef.current) {
					console.log('Setting up renderer...');
					const spriteRenderer = new SpriteRenderer(canvasRef.current, sheet);
					// Set canvas to full window size
					const resizeCanvas = () => {
						spriteRenderer.setSize(window.innerWidth, window.innerHeight);
					};

					resizeCanvas();
					window.addEventListener('resize', resizeCanvas);
					setRenderer(spriteRenderer);
					console.log('Renderer setup complete');
				}
			} catch (error) {
				console.error("Failed to initialize sprites:", error);
			}
		};

		initSprites();
	}, []);

	// Render game
	const render = () => {
		if (!renderer || !spriteSheet) {
			return;
		}

		renderer.clear();

		// Render terrain layer
		renderTerrain();

		// Render water layer
		renderWater();

		// Render player
		renderPlayer();
	};


	const renderTerrain = () => {
		const tileSize = 32; // 16px * 2 scale
		const buffer = tileSize; // Extra buffer to prevent gaps

		// Calculate visible tile range with buffer
		const minTileX = Math.floor((gameState.viewport.x - buffer) / tileSize);
		const maxTileX = Math.ceil((gameState.viewport.x + window.innerWidth + buffer) / tileSize);
		const minTileY = Math.floor((gameState.viewport.y - buffer) / tileSize);
		const maxTileY = Math.ceil((gameState.viewport.y + window.innerHeight + buffer) / tileSize);

		// Fill background with varied terrain
		for (let worldY = minTileY; worldY <= maxTileY; worldY++) {
			for (let worldX = minTileX; worldX <= maxTileX; worldX++) {
				const screenX = worldX * tileSize - gameState.viewport.x;
				const screenY = worldY * tileSize - gameState.viewport.y;

				// Use grass for land areas
				if (!gameState.isWater(worldX, worldY)) {
					const grassType = (worldX + worldY) % 2 === 0 ? "GRASS_1" : "GRASS_2";
					renderer.drawSprite(grassType, screenX, screenY);
				}
			}
		}
	};

	// Helper function to determine which water/edge sprite to use
	const getWaterSprite = (x, y) => {
		const isWaterHere = gameState.isWater(x, y);
		if (!isWaterHere) return null;

		// Check all 8 directions for sophisticated corner detection
		const waterUp = gameState.isWater(x, y - 1);
		const waterDown = gameState.isWater(x, y + 1);
		const waterLeft = gameState.isWater(x - 1, y);
		const waterRight = gameState.isWater(x + 1, y);
		const waterUpLeft = gameState.isWater(x - 1, y - 1);
		const waterUpRight = gameState.isWater(x + 1, y - 1);
		const waterDownLeft = gameState.isWater(x - 1, y + 1);
		const waterDownRight = gameState.isWater(x + 1, y + 1);

		// Inner corner detection (water surrounded by land on two adjacent sides)
		// Check for water at the specified offsets as per the coordinate system
		if (waterRight && waterDown && !waterUp && !waterLeft) {
			return "WATER_CORNER_IN_TOP_LEFT"; // water at +1,0 and 0,+1 (right and down)
		}
		if (waterLeft && waterDown && !waterUp && !waterRight) {
			return "WATER_CORNER_IN_TOP_RIGHT"; // water at -1,0 and 0,+1 (left and down)
		}
		if (waterRight && waterUp && !waterDown && !waterLeft) {
			return "WATER_CORNER_IN_BOTTOM_LEFT"; // water at +1,0 and 0,-1 (right and up)
		}
		if (waterLeft && waterUp && !waterDown && !waterRight) {
			return "WATER_CORNER_IN_BOTTOM_RIGHT"; // water at -1,0 and 0,-1 (left and up)
		}

		// Outer corner detection (land surrounded by water on two adjacent sides)
		if (waterLeft && waterUp && !waterUpLeft) {
			return "WATER_CORNER_OUT_TOP_LEFT"; // water at -1,0 and 0,-1 (left and up)
		}
		if (waterRight && waterUp && !waterUpRight) {
			return "WATER_CORNER_OUT_TOP_RIGHT"; // water at +1,0 and 0,-1 (right and up)
		}
		if (waterLeft && waterDown && !waterDownLeft) {
			return "WATER_CORNER_OUT_BOTTOM_LEFT"; // water at -1,0 and 0,+1 (left and down)
		}
		if (waterRight && waterDown && !waterDownRight) {
			return "WATER_CORNER_OUT_BOTTOM_RIGHT"; // water at +1,0 and 0,+1 (right and down)
		}

		// Edge detection - prioritize corners over simple edges
		if (!waterUp && waterDown && waterLeft && waterRight) return "WATER_TOP"; // water at 0,+1 (down)
		if (!waterDown && waterUp && waterLeft && waterRight) return "WATER_BOTTOM"; // water at 0,-1 (up)
		if (!waterLeft && waterRight && waterUp && waterDown) return "WATER_LEFT"; // water at +1,0 (right)
		if (!waterRight && waterLeft && waterUp && waterDown) return "WATER_RIGHT"; // water at -1,0 (left)

		// If completely surrounded by water, use regular water sprite
		return (x + y) % 2 === 0 ? "WATER_1" : "WATER_2";
	};

	const renderWater = () => {
		const tileSize = 32;

		// Calculate which tiles are actually visible with extra buffer
		const buffer = tileSize * 2; // Extra buffer to prevent gaps
		const minTileX = Math.floor((gameState.viewport.x - buffer) / tileSize);
		const maxTileX = Math.ceil((gameState.viewport.x + window.innerWidth + buffer) / tileSize);
		const minTileY = Math.floor((gameState.viewport.y - buffer) / tileSize);
		const maxTileY = Math.ceil((gameState.viewport.y + window.innerHeight + buffer) / tileSize);

		// Get all river tiles in the visible range
		const visibleTiles = gameState.river.generator.getRiverTiles(minTileY, maxTileY);

		visibleTiles.forEach((tile) => {
			// Only render tiles that are actually in our X range
			if (tile.x >= minTileX && tile.x <= maxTileX) {
				const screenX = tile.x * tileSize - gameState.viewport.x;
				const screenY = tile.y * tileSize - gameState.viewport.y;

				// Use smart sprite selection based on adjacent tiles
				const waterSprite = getWaterSprite(tile.x, tile.y);
				if (waterSprite) {
					renderer.drawSprite(waterSprite, screenX, screenY);
				}
			}
		});
	};

	const renderPlayer = () => {
		const tileSize = 32;
		const screenX = gameState.player.visualX * tileSize - gameState.viewport.x;
		const screenY = gameState.player.visualY * tileSize - gameState.viewport.y;

		renderer.drawSprite("PLAYER", screenX, screenY);
	};

	// Game loop with delta time
	useEffect(() => {
		const gameLoop = (currentTime) => {
			const deltaTime = currentTime - lastTimeRef.current;
			lastTimeRef.current = currentTime;

			// Update game state
			gameState.update(deltaTime);

			// Check for game over
			if (gameState.isGameOver() && !showGameOver) {
				const finalStats = gameState.getStats();
				setFinalScore(finalStats);
				setShowGameOver(true);
				console.log('Game Over! Final score:', finalStats);
			}

			// Update stats for React rendering
			const newStats = gameState.getStats();
			setStats(newStats);

			// Render
			render();

			// Continue game loop only if game is not over
			if (!gameState.isGameOver()) {
				animationFrameRef.current = requestAnimationFrame(gameLoop);
			}
		};

		if (renderer && !showGameOver) {
			console.log('Starting game loop with renderer');
			lastTimeRef.current = performance.now();
			animationFrameRef.current = requestAnimationFrame(gameLoop);
		} else {
			console.log('No renderer available or game over, game loop not started');
		}

		return () => {
			if (animationFrameRef.current) {
				cancelAnimationFrame(animationFrameRef.current);
			}
		};
	}, [renderer, showGameOver]);

	// Backup stats update - in case game loop has issues
	useEffect(() => {
		const interval = setInterval(() => {
			if (!showInstructions) {
				const newStats = gameState.getStats();
				console.log('Backup stats update:', newStats);
				setStats(newStats);
			}
		}, 100); // Update every 100ms

		return () => clearInterval(interval);
	}, [gameState, showInstructions]);

	// Handle keyboard input
	useEffect(() => {
		const handleKeyPress = (event) => {
			// If instructions are showing, any key starts the game
			if (showInstructions) {
				setShowInstructions(false);
				console.log('Game started! Initial stats:', gameState.getStats());
				setStats(gameState.getStats()); // Force immediate stats update
				return;
			}

			// If game over is showing, any key restarts the game
			if (showGameOver) {
				gameState.resetGame();
				setShowGameOver(false);
				setStats(gameState.getStats());
				console.log('Game restarted!');
				return;
			}

			// Only allow horizontal movement (left/right) during normal gameplay
			switch (event.key) {
				case "ArrowLeft":
				case "a":
					gameState.movePlayer(-1, 0);
					break;
				case "ArrowRight":
				case "d":
					gameState.movePlayer(1, 0);
					break;
			}
		};

		window.addEventListener("keydown", handleKeyPress);
		return () => window.removeEventListener("keydown", handleKeyPress);
	}, [gameState, showInstructions, showGameOver]);

	return (
		<div
			style={{
				position: "fixed",
				top: 0,
				left: 0,
				width: "100vw",
				height: "100vh",
				overflow: "hidden",
			}}
		>
			<canvas
				ref={canvasRef}
				style={{
					display: "block",
					imageRendering: "pixelated",
				}}
			/>

			{/* Game Stats Overlay */}
			{!showInstructions && (
				<div
					style={{
						position: "absolute",
						top: "20px",
						left: "20px",
						color: "white",
						fontFamily: "monospace",
						fontSize: "16px",
						textShadow: "2px 2px 4px rgba(0,0,0,0.8)",
						backgroundColor: "rgba(0, 0, 0, 0.5)",
						padding: "10px",
						borderRadius: "8px",
						zIndex: 100,
					}}
				>
					<div>Distance: {stats.distance}</div>
					<div>Speed: {stats.speed}</div>
					<div>Time: {stats.time}s</div>
				</div>
			)}

			{showInstructions && (
				<div
					style={{
						position: "absolute",
						top: 0,
						left: 0,
						width: "100%",
						height: "100%",
						backgroundColor: "rgba(0, 0, 0, 0.8)",
						display: "flex",
						flexDirection: "column",
						justifyContent: "center",
						alignItems: "center",
						color: "white",
						fontSize: "1.2rem",
						textAlign: "center",
						zIndex: 1000,
					}}
				>
					<div
						style={{
							maxWidth: "600px",
							padding: "40px",
							borderRadius: "16px",
							backgroundColor: "rgba(16, 23, 37, 0.9)",
							border: "2px solid #444",
						}}
					>
						<h1 style={{ fontSize: "2.5rem", marginBottom: "30px", color: "#60A5FA" }}>
							River Sailing Adventure
						</h1>

						<div style={{ fontSize: "1.1rem", lineHeight: "1.6", marginBottom: "30px" }}>
							<p style={{ marginBottom: "20px" }}>
								Navigate your boat down the infinite winding river! The current carries you forward automatically - steer left and right to stay in the water.
							</p>

							<div style={{ textAlign: "left", margin: "20px 0" }}>
								<h3 style={{ color: "#60A5FA", marginBottom: "15px" }}>Controls:</h3>
								<p>• <strong>A / ←</strong> - Steer left</p>
								<p>• <strong>D / →</strong> - Steer right</p>
							</div>

							<div style={{ textAlign: "left", margin: "20px 0" }}>
								<h3 style={{ color: "#60A5FA", marginBottom: "15px" }}>Objective:</h3>
								<p>• Stay in the blue water tiles</p>
								<p>• The river moves you forward automatically</p>
								<p>• If you hit dirt, the camera keeps moving while you stay behind</p>
								<p>• Get back to water quickly or you'll fall too far behind!</p>
								<p>• See how far you can go!</p>
							</div>
						</div>

						<div
							style={{
								fontSize: "1.3rem",
								color: "#FCD34D",
								fontWeight: "bold",
								animation: "pulse 2s infinite",
							}}
						>
							Press any key to start sailing!
						</div>
					</div>
				</div>
			)}

			{showGameOver && (
				<div
					style={{
						position: "absolute",
						top: 0,
						left: 0,
						width: "100%",
						height: "100%",
						backgroundColor: "rgba(0, 0, 0, 0.9)",
						display: "flex",
						flexDirection: "column",
						justifyContent: "center",
						alignItems: "center",
						color: "white",
						fontSize: "1.2rem",
						textAlign: "center",
						zIndex: 1000,
					}}
				>
					<div
						style={{
							maxWidth: "500px",
							padding: "40px",
							borderRadius: "16px",
							backgroundColor: "rgba(16, 23, 37, 0.95)",
							border: "2px solid #EF4444",
						}}
					>
						<h1 style={{ fontSize: "3rem", marginBottom: "20px", color: "#EF4444" }}>
							Game Over!
						</h1>

						<div style={{ fontSize: "1.3rem", marginBottom: "30px", color: "#F87171" }}>
							You fell too far behind the current!
						</div>

						<div style={{ fontSize: "1.4rem", lineHeight: "1.8", marginBottom: "30px" }}>
							<div style={{ marginBottom: "15px" }}>
								<span style={{ color: "#60A5FA", fontWeight: "bold" }}>Final Distance:</span>{" "}
								<span style={{ color: "#FCD34D", fontWeight: "bold", fontSize: "1.6rem" }}>
									{finalScore.distance}
								</span>
							</div>
							<div style={{ marginBottom: "15px" }}>
								<span style={{ color: "#60A5FA" }}>Time Survived:</span>{" "}
								<span style={{ color: "#FCD34D" }}>{finalScore.time}s</span>
							</div>
							<div style={{ marginBottom: "15px" }}>
								<span style={{ color: "#60A5FA" }}>Final Speed:</span>{" "}
								<span style={{ color: "#FCD34D" }}>{finalScore.speed}</span>
							</div>
						</div>

						<div
							style={{
								fontSize: "1.3rem",
								color: "#FCD34D",
								fontWeight: "bold",
								animation: "pulse 2s infinite",
							}}
						>
							Press any key to play again!
						</div>
					</div>
				</div>
			)}
		</div>
	);
};

export default GameCanvas;

