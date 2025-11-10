import React from 'react';
import ReactDOM from 'react-dom/client';
import GameCanvas from "./game/GameCanvas.jsx";
import "./App.css";

const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(
  <React.StrictMode>
		<GameCanvas />
  </React.StrictMode>,
);
