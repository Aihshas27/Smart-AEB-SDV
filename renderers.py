"""
High-Performance Vector Graphics & Animation Engine for SDV AEB Simulation.

Provides:
1. build_interactive_player_html: Embedded 60 FPS HTML5/SVG/JS simulation player
   with real moving car, wheel rotation, animated walking pedestrian across zebra crossing,
   dynamic radar corridor, active brake lights, and 4 instantaneous camera perspectives.
2. Individual static renderers:
   - render_top_down_view
   - render_side_view
   - render_third_person_view
   - render_driver_pov_view
3. render_architecture_diagram (Tab 3 SDV system architecture)
"""

import json
from typing import Dict, Any, List


def get_aeb_colors(aeb_state: str) -> Dict[str, str]:
    """Returns standardized theme colors and labels for each AEB state."""
    state = str(aeb_state).upper()
    if state == "EMERGENCY_BRAKING":
        return {
            "fill": "rgba(239, 68, 68, 0.45)",
            "stroke": "#ef4444",
            "glow": "rgba(239, 68, 68, 0.7)",
            "text": "EMERGENCY BRAKING",
            "badge_bg": "#991b1b",
            "badge_fg": "#fee2e2",
            "status_icon": "🚨",
        }
    elif state == "PARTIAL_BRAKING":
        return {
            "fill": "rgba(249, 115, 22, 0.38)",
            "stroke": "#f97316",
            "glow": "rgba(249, 115, 22, 0.6)",
            "text": "PARTIAL BRAKING",
            "badge_bg": "#9a3412",
            "badge_fg": "#ffedd5",
            "status_icon": "⚡",
        }
    elif state == "WARNING":
        return {
            "fill": "rgba(234, 179, 8, 0.32)",
            "stroke": "#eab308",
            "glow": "rgba(234, 179, 8, 0.5)",
            "text": "COLLISION WARNING",
            "badge_bg": "#854d0e",
            "badge_fg": "#fef9c3",
            "status_icon": "⚠️",
        }
    else:
        return {
            "fill": "rgba(34, 197, 94, 0.22)",
            "stroke": "#22c55e",
            "glow": "rgba(34, 197, 94, 0.4)",
            "text": "NORMAL CRUISE",
            "badge_bg": "#14532d",
            "badge_fg": "#dcfce7",
            "status_icon": "🟢",
        }


def build_interactive_player_html(
    telemetry_records: List[Dict[str, Any]],
    initial_dist: float,
    initial_speed_kmh: float,
    pedestrian_behavior: str = "WALKING",
    default_camera: str = "Top-Down",
) -> str:
    """
    Builds a complete, self-contained 60 FPS interactive automotive simulation player.
    Embedded directly into Streamlit via components.html().

    Features:
    - 4 selectable real-time camera modes (Top-Down, Side, Third-Person, Driver POV)
    - Full visible car with body, windshield, windows, headlights, wheels with rotation, and glowing brake lights
    - Human pedestrian with walking leg cycle crossing the zebra crossing across the road
    - Dynamic AEB radar perception cone color-coded by state
    - Play, Pause, Reset, Replay, Step (+0.1s), Speed (0.5x, 1x, 2x), and Scrubber slider
    - Live HUD telemetry strip and simulation clock
    """
    telemetry_json = json.dumps(telemetry_records)

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<style>
  * {{ box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }}
  body {{ background: #090d16; color: #f8fafc; overflow: hidden; padding: 10px; }}
  
  .player-card {{
    background: #0f172a;
    border: 1px solid #1e293b;
    border-radius: 12px;
    padding: 14px;
    box-shadow: 0 8px 24px rgba(0,0,0,0.5);
  }}
  
  .top-toolbar {{
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 10px;
    flex-wrap: wrap;
    gap: 8px;
  }}

  .cam-group {{
    display: flex;
    background: #1e293b;
    border-radius: 8px;
    padding: 3px;
    gap: 3px;
  }}

  .cam-btn {{
    background: transparent;
    border: none;
    color: #94a3b8;
    padding: 6px 14px;
    border-radius: 6px;
    font-size: 12px;
    font-weight: 700;
    cursor: pointer;
    transition: all 0.2s;
  }}

  .cam-btn:hover {{ color: #ffffff; background: #334155; }}
  .cam-btn.active {{ background: #0284c7; color: #ffffff; box-shadow: 0 2px 6px rgba(2,132,199,0.4); }}

  .clock-badge {{
    background: #1e293b;
    border: 1px solid #334155;
    padding: 5px 12px;
    border-radius: 6px;
    font-family: monospace;
    font-size: 13px;
    font-weight: bold;
    color: #38bdf8;
  }}

  .scene-container {{
    position: relative;
    width: 100%;
    height: 310px;
    background: #020617;
    border-radius: 10px;
    border: 1px solid #1e293b;
    overflow: hidden;
  }}

  svg.scene-svg {{
    width: 100%;
    height: 100%;
    display: block;
  }}

  .controls-bar {{
    display: flex;
    align-items: center;
    gap: 10px;
    margin-top: 10px;
    background: #1e293b;
    padding: 8px 12px;
    border-radius: 8px;
    flex-wrap: wrap;
  }}

  .btn {{
    background: #334155;
    border: none;
    color: #f8fafc;
    padding: 6px 12px;
    border-radius: 6px;
    font-size: 12px;
    font-weight: 700;
    cursor: pointer;
    display: flex;
    align-items: center;
    gap: 4px;
    transition: all 0.15s;
  }}
  .btn:hover {{ background: #475569; }}
  .btn-primary {{ background: #0284c7; }}
  .btn-primary:hover {{ background: #0369a1; }}
  .btn.active {{ background: #38bdf8; color: #020617; }}

  .scrubber-wrap {{
    flex: 1;
    display: flex;
    align-items: center;
    gap: 8px;
    min-width: 180px;
  }}

  input[type=range] {{
    width: 100%;
    height: 6px;
    border-radius: 3px;
    background: #475569;
    outline: none;
    cursor: pointer;
    accent-color: #38bdf8;
  }}

  .hud-strip {{
    display: grid;
    grid-template-columns: repeat(6, 1fr);
    gap: 8px;
    margin-top: 10px;
  }}

  .hud-box {{
    background: #1e293b;
    border-radius: 6px;
    padding: 6px 10px;
    border-left: 3px solid #38bdf8;
  }}

  .hud-title {{ font-size: 10px; color: #94a3b8; text-transform: uppercase; }}
  .hud-val {{ font-size: 16px; font-weight: bold; font-family: monospace; color: #f8fafc; margin-top: 2px; }}

  .badge-state {{
    display: inline-block;
    padding: 2px 8px;
    border-radius: 4px;
    font-size: 11px;
    font-weight: bold;
    text-transform: uppercase;
  }}
</style>
</head>
<body>

<div class="player-card">
  <!-- Top Navigation & Camera Mode Selector -->
  <div class="top-toolbar">
    <div class="cam-group">
      <button class="cam-btn {'active' if default_camera == 'Top-Down' else ''}" onclick="setCamera('Top-Down')">🗺️ Top-Down Aerial</button>
      <button class="cam-btn {'active' if default_camera == 'Side' else ''}" onclick="setCamera('Side')">🏎️ Side Elevation</button>
      <button class="cam-btn {'active' if default_camera == 'Third-Person' else ''}" onclick="setCamera('Third-Person')">🎮 Third-Person Follow</button>
      <button class="cam-btn {'active' if default_camera == 'Driver POV' else ''}" onclick="setCamera('Driver POV')">👀 Driver Cockpit POV</button>
    </div>
    <div style="display:flex; align-items:center; gap:8px;">
      <span id="behaviorTag" style="background:#334155; color:#cbd5e1; font-size:11px; padding:4px 8px; border-radius:4px; font-weight:bold;">
        PEDESTRIAN: {pedestrian_behavior.replace('_', ' ')}
      </span>
      <div class="clock-badge" id="clockDisplay">TIME: 0.00s / 0.00s</div>
    </div>
  </div>

  <!-- Main Animated Simulation Viewport -->
  <div class="scene-container">
    <svg id="sceneSvg" class="scene-svg" viewBox="0 0 1000 310" xmlns="http://www.w3.org/2000/svg">
      <!-- Dynamically drawn by JavaScript at 60 FPS -->
    </svg>
  </div>

  <!-- Playback Control Deck -->
  <div class="controls-bar">
    <button class="btn btn-primary" id="playBtn" onclick="togglePlay()">▶ PLAY</button>
    <button class="btn" onclick="resetSim()">🔄 RESET</button>
    <button class="btn" onclick="replaySim()">🔁 REPLAY</button>
    <button class="btn" onclick="stepSim(1)">⏭ +0.1s</button>
    
    <div class="scrubber-wrap">
      <span style="font-size:11px; color:#94a3b8; font-family:monospace;" id="curStepText">0s</span>
      <input type="range" id="scrubber" min="0" max="100" value="0" oninput="onScrub(this.value)">
      <span style="font-size:11px; color:#94a3b8; font-family:monospace;" id="totalStepText">0s</span>
    </div>

    <div style="display:flex; gap:4px; align-items:center;">
      <span style="font-size:11px; color:#94a3b8; font-weight:bold;">SPEED:</span>
      <button class="btn" id="spd05" onclick="setSpeed(0.5)">0.5x</button>
      <button class="btn active" id="spd10" onclick="setSpeed(1.0)">1.0x</button>
      <button class="btn" id="spd20" onclick="setSpeed(2.0)">2.0x</button>
    </div>
  </div>

  <!-- Live Telemetry KPI Strip -->
  <div class="hud-strip">
    <div class="hud-box" style="border-left-color:#38bdf8;">
      <div class="hud-title">Speed</div>
      <div class="hud-val" id="hudSpeed">0.0 km/h</div>
    </div>
    <div class="hud-box" style="border-left-color:#4ade80;">
      <div class="hud-title">Distance (Gap)</div>
      <div class="hud-val" id="hudDistance">0.0 m</div>
    </div>
    <div class="hud-box" style="border-left-color:#c084fc;">
      <div class="hud-title">TTC</div>
      <div class="hud-val" id="hudTTC">∞</div>
    </div>
    <div class="hud-box" style="border-left-color:#f87171;">
      <div class="hud-title">Brake Pressure</div>
      <div class="hud-val" id="hudBrake">0%</div>
    </div>
    <div class="hud-box" style="border-left-color:#facc15;">
      <div class="hud-title">ECU State</div>
      <div class="hud-val" style="font-size:13px;" id="hudState">NORMAL</div>
    </div>
    <div class="hud-box" style="border-left-color:#34d399;">
      <div class="hud-title">Status</div>
      <div class="hud-val" style="font-size:13px;" id="hudStatus">RUNNING</div>
    </div>
  </div>
</div>

<script>
  // Complete Telemetry Time-Series loaded from Python
  const telemetry = {telemetry_json};
  const totalFrames = telemetry.length;
  const initialDist = {initial_dist};
  const initialSpeedKmh = {initial_speed_kmh};
  const pedestrianMode = "{pedestrian_behavior}";
  
  let currentCamera = "{default_camera}";
  let currentFrameIdx = 0;
  let isPlaying = true;
  let playbackRate = 1.0;
  let lastAnimTime = performance.now();

  const scrubber = document.getElementById('scrubber');
  scrubber.max = totalFrames - 1;
  document.getElementById('totalStepText').innerText = (telemetry[totalFrames - 1].timestamp).toFixed(1) + 's';

  function setCamera(cam) {{
    currentCamera = cam;
    document.querySelectorAll('.cam-btn').forEach(b => {{
      b.classList.toggle('active', b.innerText.includes(cam));
    }});
    renderFrame(currentFrameIdx);
  }}

  function togglePlay() {{
    isPlaying = !isPlaying;
    const btn = document.getElementById('playBtn');
    if (isPlaying) {{
      btn.innerText = '⏸ PAUSE';
      btn.classList.add('btn-primary');
      if (currentFrameIdx >= totalFrames - 1) currentFrameIdx = 0;
    }} else {{
      btn.innerText = '▶ PLAY';
      btn.classList.remove('btn-primary');
    }}
  }}

  function resetSim() {{
    isPlaying = false;
    currentFrameIdx = 0;
    document.getElementById('playBtn').innerText = '▶ PLAY';
    document.getElementById('playBtn').classList.remove('btn-primary');
    scrubber.value = 0;
    renderFrame(0);
  }}

  function replaySim() {{
    currentFrameIdx = 0;
    isPlaying = true;
    document.getElementById('playBtn').innerText = '⏸ PAUSE';
    document.getElementById('playBtn').classList.add('btn-primary');
  }}

  function stepSim(delta) {{
    isPlaying = false;
    document.getElementById('playBtn').innerText = '▶ PLAY';
    currentFrameIdx = Math.min(totalFrames - 1, Math.max(0, currentFrameIdx + delta * 5));
    scrubber.value = currentFrameIdx;
    renderFrame(currentFrameIdx);
  }}

  function onScrub(val) {{
    isPlaying = false;
    document.getElementById('playBtn').innerText = '▶ PLAY';
    currentFrameIdx = parseInt(val);
    renderFrame(currentFrameIdx);
  }}

  function setSpeed(spd) {{
    playbackRate = spd;
    ['05','10','20'].forEach(s => document.getElementById('spd' + s).classList.remove('active'));
    if (spd === 0.5) document.getElementById('spd05').classList.add('active');
    else if (spd === 1.0) document.getElementById('spd10').classList.add('active');
    else if (spd === 2.0) document.getElementById('spd20').classList.add('active');
  }}

  // Animation Loop running at 60 FPS
  function animLoop(now) {{
    const dt = (now - lastAnimTime) / 1000.0;
    lastAnimTime = now;

    if (isPlaying && totalFrames > 1) {{
      // Simulation steps at dt=0.02s (50 Hz). Frames per second = 50 * playbackRate
      const framesToAdvance = dt * 50.0 * playbackRate;
      currentFrameIdx += framesToAdvance;
      if (currentFrameIdx >= totalFrames - 1) {{
        currentFrameIdx = totalFrames - 1;
        isPlaying = false;
        document.getElementById('playBtn').innerText = '▶ REPLAY';
      }}
      scrubber.value = Math.floor(currentFrameIdx);
      renderFrame(Math.floor(currentFrameIdx));
    }}

    requestAnimationFrame(animLoop);
  }}
  requestAnimationFrame(animLoop);

  // Main rendering dispatcher
  function renderFrame(idx) {{
    if (!telemetry || telemetry.length === 0) return;
    const f = telemetry[idx] || telemetry[0];
    const maxT = telemetry[totalFrames - 1].timestamp.toFixed(2);
    document.getElementById('clockDisplay').innerText = `TIME: ${{f.timestamp.toFixed(2)}}s / ${{maxT}}s`;
    document.getElementById('curStepText').innerText = f.timestamp.toFixed(1) + 's';

    // Update Live HUD
    document.getElementById('hudSpeed').innerText = `${{f.speed_kmh.toFixed(1)}} km/h`;
    document.getElementById('hudDistance').innerText = `${{f.distance_to_pedestrian.toFixed(1)}} m`;
    document.getElementById('hudTTC').innerText = (f.ttc < 99.0 ? f.ttc.toFixed(2) + ' s' : '∞ (Safe)');
    document.getElementById('hudBrake').innerText = `${{Math.round(f.actual_brake_pressure)}}%`;
    
    // State colors
    let stateColor = '#22c55e';
    let stateBg = '#14532d';
    if (f.aeb_state === 'EMERGENCY_BRAKING') {{ stateColor = '#ef4444'; stateBg = '#7f1d1d'; }}
    else if (f.aeb_state === 'PARTIAL_BRAKING') {{ stateColor = '#f97316'; stateBg = '#7c2d12'; }}
    else if (f.aeb_state === 'WARNING') {{ stateColor = '#facc15'; stateBg = '#713f12'; }}
    
    const stateEl = document.getElementById('hudState');
    stateEl.innerHTML = `<span class="badge-state" style="background:${{stateBg}}; color:${{stateColor}};">${{f.aeb_state}}</span>`;

    let statusHtml = '<span style="color:#38bdf8;">⏳ IN MOTION</span>';
    if (f.collision_status === 'COLLISION') statusHtml = '<span style="color:#ef4444; font-weight:bold;">💥 COLLISION</span>';
    else if (f.collision_status === 'SAFE_STOP') statusHtml = '<span style="color:#22c55e; font-weight:bold;">✅ SAFE STOP</span>';
    document.getElementById('hudStatus').innerHTML = statusHtml;

    // Draw corresponding Camera Scene
    const svgEl = document.getElementById('sceneSvg');
    if (currentCamera === 'Top-Down') {{
      svgEl.innerHTML = drawTopDown(f);
    }} else if (currentCamera === 'Side') {{
      svgEl.innerHTML = drawSideView(f);
    }} else if (currentCamera === 'Third-Person') {{
      svgEl.innerHTML = drawThirdPerson(f);
    }} else {{
      svgEl.innerHTML = drawDriverPOV(f);
    }}
  }}

  // -------------------------------------------------------------
  // CAMERA 1: TOP-DOWN VIEW
  // -------------------------------------------------------------
  function drawTopDown(f) {{
    // Canvas: 1000 x 310
    // Road spans y=55 to y=255 (200px wide, two 100px lanes). Centerline y=155.
    // Sidewalks: top y=15 to 55, bottom y=255 to 300.
    const startX = 80.0;
    const pedLongX = 860.0; // zebra crossing location
    
    // Progress calculation for longitudinal car movement:
    // Car length: 110px. When progress=1, front bumper touches zebra crossing:
    const prog = Math.min(1.0, Math.max(0.0, f.position / initialDist));
    const carX = startX + prog * (pedLongX - 110.0 - startX);
    const frontBumperX = carX + 110.0;
    const carY = 128.0; // Car centered vertically in bottom lane

    // Lateral pedestrian position mapping:
    // y=-4.0m (bottom sidewalk) -> pixel y=278
    // y=0.0m (car lane center)  -> pixel y=155
    // y=+4.0m (top sidewalk)    -> pixel y=32
    const pedPixelY = 155.0 - (f.pedestrian_y / 4.0) * 123.0;

    // Walking stride oscillation (legs & arms swing)
    const walkPhase = Math.sin(f.timestamp * 10.0);
    const armSwing = walkPhase * 8.0;
    const legSwing = walkPhase * 6.0;

    // Radar corridor color
    let radarColor = 'rgba(34, 197, 94, 0.25)';
    let radarStroke = '#22c55e';
    if (f.aeb_state === 'EMERGENCY_BRAKING') {{ radarColor = 'rgba(239, 68, 68, 0.55)'; radarStroke = '#ef4444'; }}
    else if (f.aeb_state === 'PARTIAL_BRAKING') {{ radarColor = 'rgba(249, 115, 22, 0.45)'; radarStroke = '#f97316'; }}
    else if (f.aeb_state === 'WARNING') {{ radarColor = 'rgba(234, 179, 8, 0.35)'; radarStroke = '#eab308'; }}

    const isBraking = f.actual_brake_pressure > 2.0;
    const brakeLightColor = isBraking ? '#ff1744' : '#6b1111';
    const brakeGlow = isBraking ? `
      <circle cx="${{carX + 4}}" cy="${{carY + 8}}" r="16" fill="rgba(255, 23, 68, 0.6)"/>
      <circle cx="${{carX + 4}}" cy="${{carY + 46}}" r="16" fill="rgba(255, 23, 68, 0.6)"/>
    ` : '';

    // Zebra crossing stripes at pedLongX (width 50px, spanning y=55 to y=255)
    let zebraStripes = '';
    for (let zy = 60; zy <= 245; zy += 18) {{
      zebraStripes += `<rect x="${{pedLongX - 25}}" y="${{zy}}" width="50" height="10" fill="#ffffff" opacity="0.9" rx="1"/>`;
    }}

    return `
      <!-- Grass / Background -->
      <rect x="0" y="0" width="1000" height="310" fill="#0b1329"/>
      <!-- Sidewalks -->
      <rect x="0" y="15" width="1000" height="40" fill="#334155"/>
      <rect x="0" y="255" width="1000" height="40" fill="#334155"/>
      <!-- Concrete Curbs -->
      <line x1="0" y1="55" x2="1000" y2="55" stroke="#94a3b8" stroke-width="3"/>
      <line x1="0" y1="255" x2="1000" y2="255" stroke="#94a3b8" stroke-width="3"/>
      <!-- Asphalt Road Surface -->
      <rect x="0" y="55" width="1000" height="200" fill="#1e2430"/>
      <!-- Solid White Edge Lines -->
      <line x1="0" y1="62" x2="1000" y2="62" stroke="#ffffff" stroke-width="3"/>
      <line x1="0" y1="248" x2="1000" y2="248" stroke="#ffffff" stroke-width="3"/>
      <!-- Yellow Centerline Dashes -->
      <line x1="0" y1="155" x2="1000" y2="155" stroke="#facc15" stroke-width="4" stroke-dasharray="35, 25"/>

      <!-- Zebra Pedestrian Crossing -->
      ${{zebraStripes}}
      <text x="${{pedLongX}}" y="45" fill="#facc15" font-size="10" font-weight="bold" text-anchor="middle">ZEBRA CROSSING</text>

      <!-- Vehicle Trajectory Track -->
      <line x1="${{startX}}" y1="155" x2="${{carX}}" y2="155" stroke="#38bdf8" stroke-width="3" stroke-dasharray="6,4" opacity="0.6"/>

      <!-- AEB Dynamic Perception Radar Corridor -->
      <polygon points="${{frontBumperX}},${{carY + 12}} ${{Math.max(frontBumperX + 10, pedLongX)}},${{Math.max(60, pedPixelY - 35)}} ${{Math.max(frontBumperX + 10, pedLongX)}},${{Math.min(250, pedPixelY + 35)}} ${{frontBumperX}},${{carY + 42}}"
               fill="${{radarColor}}" stroke="${{radarStroke}}" stroke-width="2" stroke-dasharray="6,4"/>

      <!-- Distance Dimension Line -->
      <line x1="${{frontBumperX}}" y1="225" x2="${{pedLongX}}" y2="225" stroke="#38bdf8" stroke-width="2" stroke-dasharray="4,3"/>
      <line x1="${{frontBumperX}}" y1="218" x2="${{frontBumperX}}" y2="232" stroke="#38bdf8" stroke-width="2"/>
      <line x1="${{pedLongX}}" y1="218" x2="${{pedLongX}}" y2="232" stroke="#38bdf8" stroke-width="2"/>
      <rect x="${{(frontBumperX + pedLongX)/2 - 50}}" y="215" width="100" height="20" rx="4" fill="#0f172a" stroke="#38bdf8" stroke-width="1"/>
      <text x="${{(frontBumperX + pedLongX)/2}}" y="229" fill="#38bdf8" font-size="11" font-weight="bold" text-anchor="middle">⟷ Gap: ${{f.distance_to_pedestrian.toFixed(1)}} m</text>

      <!-- Brake Flare Aura -->
      ${{brakeGlow}}

      <!-- COMPLETE CAR (Top-Down Model) -->
      <g transform="translate(${{carX}}, ${{carY}})">
        <!-- 4 Rubber Wheels with Silver Rim Accents -->
        <rect x="14" y="-8" width="24" height="10" rx="3" fill="#020617" stroke="#64748b" stroke-width="1.5"/>
        <rect x="74" y="-8" width="24" height="10" rx="3" fill="#020617" stroke="#64748b" stroke-width="1.5"/>
        <rect x="14" y="52" width="24" height="10" rx="3" fill="#020617" stroke="#64748b" stroke-width="1.5"/>
        <rect x="74" y="52" width="24" height="10" rx="3" fill="#020617" stroke="#64748b" stroke-width="1.5"/>

        <!-- Main Car Body Chassis -->
        <rect x="0" y="0" width="110" height="54" rx="14" fill="#1e40af" stroke="#60a5fa" stroke-width="2"/>
        <!-- Aerodynamic Hood & Trunk Contours -->
        <path d="M 75,5 L 102,10 L 102,44 L 75,49 Z" fill="#2563eb" opacity="0.8"/>
        <path d="M 8,8 L 30,5 L 30,49 L 8,46 Z" fill="#1d4ed8" opacity="0.8"/>
        
        <!-- Tinted Glass Cabin & Windows -->
        <path d="M 68,8 L 76,10 L 76,44 L 68,46 Z" fill="#93c5fd" opacity="0.95"/> <!-- Windshield -->
        <path d="M 30,8 L 38,10 L 38,44 L 30,46 Z" fill="#93c5fd" opacity="0.95"/> <!-- Rear Glass -->
        <rect x="38" y="10" width="30" height="34" rx="3" fill="#0f172a" opacity="0.85"/> <!-- Panoramic Roof -->

        <!-- Side Mirrors -->
        <rect x="66" y="-7" width="8" height="6" rx="2" fill="#1e40af" stroke="#60a5fa" stroke-width="1"/>
        <rect x="66" y="55" width="8" height="6" rx="2" fill="#1e40af" stroke="#60a5fa" stroke-width="1"/>

        <!-- Twin Projector LED Headlights casting beams -->
        <polygon points="106,6 110,6 110,16 106,16" fill="#fef08a"/>
        <polygon points="106,38 110,38 110,48 106,48" fill="#fef08a"/>
        <polygon points="110,8 180,-15 180,30 110,14" fill="rgba(254, 240, 138, 0.25)"/>
        <polygon points="110,40 180,24 180,69 110,46" fill="rgba(254, 240, 138, 0.25)"/>

        <!-- Rear Brake Lights (Illuminate Intense Red When Braking) -->
        <rect x="0" y="6" width="5" height="12" rx="1.5" fill="${{brakeLightColor}}"/>
        <rect x="0" y="36" width="5" height="12" rx="1.5" fill="${{brakeLightColor}}"/>

        <!-- Front ADAS Radar/LiDAR Sensor -->
        <circle cx="110" cy="27" r="4.5" fill="#38bdf8" stroke="#ffffff" stroke-width="1.5"/>

        <!-- Speed Badge Riding Above Car -->
        <rect x="25" y="-28" width="60" height="18" rx="4" fill="#0284c7" stroke="#38bdf8" stroke-width="1"/>
        <text x="55" y="-15" fill="#ffffff" font-size="11" font-weight="bold" text-anchor="middle">${{f.speed_kmh.toFixed(0)}} km/h</text>
      </g>

      <!-- PEDESTRIAN (Top-Down Model with Moving Arms/Legs) -->
      <g transform="translate(${{pedLongX}}, ${{pedPixelY}})">
        <ellipse cx="2" cy="4" rx="12" ry="8" fill="rgba(0,0,0,0.5)"/>
        <rect x="${{-6 + legSwing}}" y="-14" width="5" height="12" rx="2" fill="#0f172a"/>
        <rect x="${{1 - legSwing}}" y="2" width="5" height="12" rx="2" fill="#0f172a"/>
        <ellipse cx="0" cy="0" rx="10" ry="16" fill="#ea580c" stroke="#ffedd5" stroke-width="1.5"/>
        <line x1="-8" y1="-5" x2="8" y2="-5" stroke="#ffffff" stroke-width="2"/>
        <line x1="-8" y1="5" x2="8" y2="5" stroke="#ffffff" stroke-width="2"/>
        <circle cx="${{-12 + armSwing}}" cy="-8" r="4" fill="#fde047"/>
        <circle cx="${{12 - armSwing}}" cy="8" r="4" fill="#fde047"/>
        <circle cx="0" cy="0" r="8" fill="#fde047" stroke="#b45309" stroke-width="1"/>
        <rect x="-42" y="-36" width="84" height="16" rx="3" fill="#0f172a" stroke="#ea580c" stroke-width="1"/>
        <text x="0" y="-24" fill="#fed7aa" font-size="9" font-weight="bold" text-anchor="middle">PEDESTRIAN</text>
      </g>
    `;
  }}

  // -------------------------------------------------------------
  // CAMERA 2: SIDE ELEVATION VIEW
  // -------------------------------------------------------------
  function drawSideView(f) {{
    const startX = 80.0;
    const pedLongX = 860.0;
    const prog = Math.min(1.0, Math.max(0.0, f.position / initialDist));
    const carX = startX + prog * (pedLongX - 140.0 - startX);
    const frontBumperX = carX + 140.0;
    const roadY = 220.0;

    // Wheel rotation angle in degrees based on vehicle position
    const wheelAngle = (f.position / 0.35) * (180.0 / Math.PI) % 360;

    // Walking animation cycle for pedestrian
    const walkCycle = Math.sin(f.timestamp * 10.0);
    const leg1Angle = walkCycle * 28.0;
    const leg2Angle = -walkCycle * 28.0;
    const arm1Angle = -walkCycle * 25.0;
    const arm2Angle = walkCycle * 25.0;

    const isBraking = f.actual_brake_pressure > 2.0;
    const brakeColor = isBraking ? '#ff1744' : '#6b1111';
    const brakeGlow = isBraking ? `<circle cx="${{carX + 4}}" cy="${{roadY - 32}}" r="22" fill="rgba(255, 23, 68, 0.6)"/>` : '';

    let radarColor = 'rgba(34, 197, 94, 0.25)';
    let radarStroke = '#22c55e';
    if (f.aeb_state === 'EMERGENCY_BRAKING') {{ radarColor = 'rgba(239, 68, 68, 0.55)'; radarStroke = '#ef4444'; }}
    else if (f.aeb_state === 'PARTIAL_BRAKING') {{ radarColor = 'rgba(249, 115, 22, 0.45)'; radarStroke = '#f97316'; }}
    else if (f.aeb_state === 'WARNING') {{ radarColor = 'rgba(234, 179, 8, 0.35)'; radarStroke = '#eab308'; }}

    const skidMarks = (f.actual_brake_pressure > 40.0 && prog > 0.05) ? `
      <line x1="${{Math.max(startX, carX - 60)}}" y1="${{roadY}}" x2="${{carX + 35}}" y2="${{roadY}}" stroke="#020617" stroke-width="5" opacity="0.8"/>
      <line x1="${{Math.max(startX, carX - 30)}}" y1="${{roadY}}" x2="${{carX + 105}}" y2="${{roadY}}" stroke="#020617" stroke-width="5" opacity="0.8"/>
    ` : '';

    return `
      <defs>
        <linearGradient id="skyGrad" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0%" stop-color="#020617"/>
          <stop offset="100%" stop-color="#1e293b"/>
        </linearGradient>
      </defs>
      <rect x="0" y="0" width="1000" height="190" fill="url(#skyGrad)"/>
      <polygon points="0,190 120,130 280,190 450,140 650,190 850,120 1000,190" fill="#0f172a" opacity="0.8"/>

      <rect x="0" y="190" width="1000" height="30" fill="#334155"/>
      <line x1="0" y1="220" x2="1000" y2="220" stroke="#94a3b8" stroke-width="3"/>
      <rect x="0" y="220" width="1000" height="90" fill="#1e2430"/>

      <rect x="${{pedLongX - 35}}" y="220" width="70" height="90" fill="#2d3748"/>
      <line x1="${{pedLongX - 25}}" y1="220" x2="${{pedLongX - 25}}" y2="310" stroke="#ffffff" stroke-width="8" stroke-dasharray="16, 12"/>
      <line x1="${{pedLongX}}" y1="220" x2="${{pedLongX}}" y2="310" stroke="#ffffff" stroke-width="8" stroke-dasharray="16, 12"/>
      <line x1="${{pedLongX + 25}}" y1="220" x2="${{pedLongX + 25}}" y2="310" stroke="#ffffff" stroke-width="8" stroke-dasharray="16, 12"/>

      ${{skidMarks}}

      <polygon points="${{frontBumperX}},${{roadY - 30}} ${{pedLongX}},${{roadY - 70}} ${{pedLongX}},${{roadY}} ${{frontBumperX}},${{roadY - 15}}"
               fill="${{radarColor}}" stroke="${{radarStroke}}" stroke-width="2" stroke-dasharray="6,4"/>

      ${{brakeGlow}}

      <!-- COMPLETE CAR (Side Profile with Rotating Alloy Wheels) -->
      <g transform="translate(${{carX}}, ${{roadY - 60}})">
        <path d="M 0,42 L 8,26 L 24,18 L 42,4 L 92,4 L 115,18 L 136,28 L 140,42 L 140,50 L 0,50 Z"
              fill="#1e40af" stroke="#60a5fa" stroke-width="2"/>
        <polygon points="44,6 88,6 110,18 28,18" fill="#93c5fd" opacity="0.9"/>
        <line x1="68" y1="6" x2="68" y2="18" stroke="#1e40af" stroke-width="2.5"/>

        <polygon points="134,28 140,28 140,38 134,38" fill="#fef08a"/>
        <polygon points="140,30 220,10 220,55 140,38" fill="rgba(254, 240, 138, 0.25)"/>
        <polygon points="0,26 6,26 6,38 0,38" fill="${{brakeColor}}"/>

        <!-- Rotating Wheel 1 (Rear) -->
        <g transform="translate(32, 50) rotate(${{wheelAngle}})">
          <circle cx="0" cy="0" r="18" fill="#020617" stroke="#64748b" stroke-width="2"/>
          <circle cx="0" cy="0" r="10" fill="#94a3b8" stroke="#cbd5e1" stroke-width="1.5"/>
          <line x1="0" y1="-10" x2="0" y2="10" stroke="#f8fafc" stroke-width="2"/>
          <line x1="-10" y1="0" x2="10" y2="0" stroke="#f8fafc" stroke-width="2"/>
          <line x1="-7" y1="-7" x2="7" y2="7" stroke="#f8fafc" stroke-width="2"/>
          <circle cx="0" cy="0" r="3" fill="#020617"/>
        </g>

        <!-- Rotating Wheel 2 (Front) -->
        <g transform="translate(108, 50) rotate(${{wheelAngle}})">
          <circle cx="0" cy="0" r="18" fill="#020617" stroke="#64748b" stroke-width="2"/>
          <circle cx="0" cy="0" r="10" fill="#94a3b8" stroke="#cbd5e1" stroke-width="1.5"/>
          <line x1="0" y1="-10" x2="0" y2="10" stroke="#f8fafc" stroke-width="2"/>
          <line x1="-10" y1="0" x2="10" y2="0" stroke="#f8fafc" stroke-width="2"/>
          <line x1="-7" y1="-7" x2="7" y2="7" stroke="#f8fafc" stroke-width="2"/>
          <circle cx="0" cy="0" r="3" fill="#020617"/>
        </g>

        <rect x="40" y="-22" width="60" height="18" rx="4" fill="#0284c7" stroke="#38bdf8" stroke-width="1"/>
        <text x="70" y="-9" fill="#ffffff" font-size="11" font-weight="bold" text-anchor="middle">${{f.speed_kmh.toFixed(0)}} km/h</text>
      </g>

      <!-- PEDESTRIAN (Side Elevation with Walking Leg/Arm Stride) -->
      <g transform="translate(${{pedLongX}}, ${{roadY - 70}})">
        <circle cx="0" cy="10" r="9" fill="#fde047" stroke="#b45309" stroke-width="1"/>
        <path d="M -7,20 L 7,20 L 8,46 L -8,46 Z" fill="#ea580c"/>
        <line x1="-6" y1="32" x2="6" y2="32" stroke="#ffffff" stroke-width="2.5"/>

        <g transform="translate(-3, 46) rotate(${{leg1Angle}})">
          <line x1="0" y1="0" x2="-2" y2="28" stroke="#0f172a" stroke-width="4.5" stroke-linecap="round"/>
        </g>
        <g transform="translate(3, 46) rotate(${{leg2Angle}})">
          <line x1="0" y1="0" x2="2" y2="28" stroke="#0f172a" stroke-width="4.5" stroke-linecap="round"/>
        </g>

        <g transform="translate(-5, 23) rotate(${{arm1Angle}})">
          <line x1="0" y1="0" x2="-2" y2="18" stroke="#fde047" stroke-width="3" stroke-linecap="round"/>
        </g>
        <g transform="translate(5, 23) rotate(${{arm2Angle}})">
          <line x1="0" y1="0" x2="2" y2="18" stroke="#fde047" stroke-width="3" stroke-linecap="round"/>
        </g>

        <rect x="-35" y="-14" width="70" height="15" rx="3" fill="#0f172a" stroke="#ea580c" stroke-width="1"/>
        <text x="0" y="-3" fill="#fed7aa" font-size="9" font-weight="bold" text-anchor="middle">PEDESTRIAN</text>
      </g>
    `;
  }}

  // -------------------------------------------------------------
  // CAMERA 3: THIRD-PERSON CHASE CAMERA VIEW
  // -------------------------------------------------------------
  function drawThirdPerson(f) {{
    const prog = Math.min(1.0, Math.max(0.0, f.position / initialDist));
    const pedY = 100.0 + prog * 95.0;
    const pedScale = 0.40 + prog * 1.3;
    const pedX = 500.0 + (f.pedestrian_y / 4.0) * (-60.0 * pedScale);

    const isBraking = f.actual_brake_pressure > 2.0;
    const brakeColor = isBraking ? '#ff1744' : '#6b1111';
    const brakeFlare = isBraking ? `
      <ellipse cx="420" cy="235" rx="26" ry="16" fill="rgba(255, 23, 68, 0.65)"/>
      <ellipse cx="580" cy="235" rx="26" ry="16" fill="rgba(255, 23, 68, 0.65)"/>
      <ellipse cx="500" cy="180" rx="20" ry="8" fill="rgba(255, 23, 68, 0.6)"/>
    ` : '';

    let radarColor = 'rgba(34, 197, 94, 0.25)';
    let radarStroke = '#22c55e';
    if (f.aeb_state === 'EMERGENCY_BRAKING') {{ radarColor = 'rgba(239, 68, 68, 0.55)'; radarStroke = '#ef4444'; }}
    else if (f.aeb_state === 'PARTIAL_BRAKING') {{ radarColor = 'rgba(249, 115, 22, 0.45)'; radarStroke = '#f97316'; }}
    else if (f.aeb_state === 'WARNING') {{ radarColor = 'rgba(234, 179, 8, 0.35)'; radarStroke = '#eab308'; }}

    const walkCycle = Math.sin(f.timestamp * 10.0);
    const leg1Spread = walkCycle * 6.0;

    return `
      <rect x="0" y="0" width="1000" height="95" fill="#020617"/>
      <polygon points="0,95 180,75 350,95 500,80 650,95 850,70 1000,95" fill="#0f172a"/>

      <polygon points="0,310 380,95 620,95 1000,310" fill="#1e2430"/>
      <polygon points="0,310 80,310 410,95 380,95" fill="#334155"/>
      <polygon points="920,310 1000,310 620,95 590,95" fill="#334155"/>

      <polygon points="496,95 504,95 508,125 492,125" fill="#facc15"/>
      <polygon points="488,145 512,145 520,185 480,185" fill="#facc15"/>
      <polygon points="475,205 525,205 540,250 460,250" fill="#facc15"/>

      <polygon points="430,${{pedY - 8}} 570,${{pedY - 8}} 585,${{pedY + 8}} 415,${{pedY + 8}}" fill="#2d3748" opacity="0.9"/>
      <line x1="440" y1="${{pedY}}" x2="560" y2="${{pedY}}" stroke="#ffffff" stroke-width="5" stroke-dasharray="10, 8"/>

      <polygon points="440,225 560,225 ${{pedX + 25 * pedScale}},${{pedY}} ${{pedX - 25 * pedScale}},${{pedY}}"
               fill="${{radarColor}}" stroke="${{radarStroke}}" stroke-width="1.5" stroke-dasharray="4,4"/>

      <!-- Pedestrian crossing ahead -->
      <g transform="translate(${{pedX}}, ${{pedY}}) scale(${{pedScale}})">
        <circle cx="0" cy="-24" r="7" fill="#fde047"/>
        <rect x="-8" y="-16" width="16" height="22" rx="3" fill="#ea580c"/>
        <line x1="-7" y1="-5" x2="7" y2="-5" stroke="#ffffff" stroke-width="2"/>
        <line x1="-4" y1="6" x2="${{-7 + leg1Spread}}" y2="26" stroke="#0f172a" stroke-width="4" stroke-linecap="round"/>
        <line x1="4" y1="6" x2="${{7 - leg1Spread}}" y2="26" stroke="#0f172a" stroke-width="4" stroke-linecap="round"/>
      </g>

      ${{brakeFlare}}

      <!-- HOST VEHICLE REAR IN FOREGROUND -->
      <g transform="translate(500, 240)">
        <rect x="-115" y="5" width="28" height="42" rx="5" fill="#020617" stroke="#64748b" stroke-width="2"/>
        <rect x="87" y="5" width="28" height="42" rx="5" fill="#020617" stroke="#64748b" stroke-width="2"/>
        <path d="M -105,15 L 105,15 L 95,42 L -95,42 Z" fill="#0f172a" stroke="#334155" stroke-width="2"/>
        <rect x="-35" y="24" width="70" height="14" rx="2" fill="#020617" stroke="#f8fafc" stroke-width="1"/>
        <text x="0" y="34" fill="#f8fafc" font-size="9" font-family="sans-serif" font-weight="bold" text-anchor="middle">SDV • AEB</text>

        <path d="M -110,-5 L 110,-5 L 98,-38 L -98,-38 Z" fill="#1e40af" stroke="#60a5fa" stroke-width="2"/>
        <path d="M -80,-38 L 80,-38 L 58,-75 L -58,-75 Z" fill="#93c5fd" opacity="0.9" stroke="#1e40af" stroke-width="2"/>
        <rect x="-90" y="-40" width="180" height="5" rx="2" fill="#0f172a"/>

        <rect x="-22" y="-72" width="44" height="5" rx="1.5" fill="${{brakeColor}}"/>
        <polygon points="-105,-8 -65,-8 -72,-25 -100,-20" fill="${{brakeColor}}" stroke="#ffffff" stroke-width="1.5"/>
        <polygon points="105,-8 65,-8 72,-25 100,-20" fill="${{brakeColor}}" stroke="#ffffff" stroke-width="1.5"/>
      </g>
    `;
  }}

  // -------------------------------------------------------------
  // CAMERA 4: DRIVER POV / COCKPIT VIEW
  // -------------------------------------------------------------
  function drawDriverPOV(f) {{
    const prog = Math.min(1.0, Math.max(0.0, f.position / initialDist));
    const pedY = 115.0 + prog * 105.0;
    const pedScale = 0.45 + prog * 1.8;
    const pedX = 500.0 + (f.pedestrian_y / 4.0) * (-90.0 * pedScale);

    let alertHud = '';
    if (f.aeb_state === 'EMERGENCY_BRAKING') {{
      alertHud = `
        <rect x="330" y="35" width="340" height="45" rx="8" fill="rgba(220, 38, 38, 0.95)" stroke="#fee2e2" stroke-width="2"/>
        <text x="500" y="62" fill="#ffffff" font-size="14" font-weight="900" text-anchor="middle">🚨 EMERGENCY BRAKING ENGAGED 🚨</text>
      `;
    }} else if (f.aeb_state === 'WARNING') {{
      alertHud = `
        <rect x="340" y="35" width="320" height="42" rx="8" fill="rgba(202, 138, 4, 0.9)" stroke="#fef9c3" stroke-width="2"/>
        <text x="500" y="61" fill="#ffffff" font-size="13" font-weight="bold" text-anchor="middle">⚠️ COLLISION WARNING - BRAKE NOW!</text>
      `;
    }}

    const walkCycle = Math.sin(f.timestamp * 10.0);
    const leg1Spread = walkCycle * 6.0;

    return `
      <rect x="0" y="0" width="1000" height="110" fill="#020617"/>
      <polygon points="0,310 400,110 600,110 1000,310" fill="#1e2430"/>
      <polygon points="496,110 504,110 510,140 490,140" fill="#facc15"/>
      <polygon points="485,160 515,160 528,210 472,210" fill="#facc15"/>

      <polygon points="430,${{pedY - 8}} 570,${{pedY - 8}} 590,${{pedY + 12}} 410,${{pedY + 12}}" fill="#2d3748" opacity="0.9"/>
      <line x1="435" y1="${{pedY}}" x2="565" y2="${{pedY}}" stroke="#ffffff" stroke-width="6" stroke-dasharray="12, 10"/>

      <g transform="translate(${{pedX}}, ${{pedY}}) scale(${{pedScale}})">
        <circle cx="0" cy="-24" r="7" fill="#fde047"/>
        <rect x="-8" y="-16" width="16" height="22" rx="3" fill="#ea580c"/>
        <line x1="-7" y1="-5" x2="7" y2="-5" stroke="#ffffff" stroke-width="2"/>
        <line x1="-4" y1="6" x2="${{-6 + leg1Spread}}" y2="26" stroke="#0f172a" stroke-width="4" stroke-linecap="round"/>
        <line x1="4" y1="6" x2="${{6 - leg1Spread}}" y2="26" stroke="#0f172a" stroke-width="4" stroke-linecap="round"/>
      </g>

      ${{alertHud}}

      <polygon points="0,0 75,0 160,250 0,250" fill="#0f172a"/>
      <polygon points="1000,0 925,0 840,250 1000,250" fill="#0f172a"/>
      <rect x="0" y="0" width="1000" height="22" fill="#0f172a"/>
      <rect x="440" y="14" width="120" height="26" rx="4" fill="#1e293b" stroke="#475569" stroke-width="1.5"/>
      <text x="500" y="31" fill="#64748b" font-size="9" text-anchor="middle">REARVIEW MIRROR</text>

      <path d="M 0,230 L 1000,230 L 1000,310 L 0,310 Z" fill="#020617"/>
      <path d="M 120,230 Q 500,205 880,230 L 880,310 L 120,310 Z" fill="#0f172a" stroke="#334155" stroke-width="2"/>

      <g transform="translate(420, 235)">
        <rect x="-80" y="0" width="320" height="52" rx="8" fill="#020617" stroke="#0284c7" stroke-width="1.5"/>
        <text x="-40" y="20" fill="#94a3b8" font-size="9">SPEED</text>
        <text x="-40" y="42" fill="#38bdf8" font-size="20" font-weight="bold">${{f.speed_kmh.toFixed(0)}} <tspan font-size="11">km/h</tspan></text>
        <text x="60" y="20" fill="#94a3b8" font-size="9">CLEARANCE</text>
        <text x="60" y="42" fill="#facc15" font-size="18" font-weight="bold">${{f.distance_to_pedestrian.toFixed(1)}} m</text>
        <text x="175" y="20" fill="#94a3b8" font-size="9">BRAKE</text>
        <rect x="175" y="25" width="55" height="14" rx="2" fill="#1e293b"/>
        <rect x="175" y="25" width="${{55 * (f.actual_brake_pressure / 100.0)}}" height="14" rx="2" fill="#ef4444"/>
      </g>

      <g transform="translate(300, 280)">
        <circle cx="0" cy="0" r="70" fill="none" stroke="#1e293b" stroke-width="18"/>
        <circle cx="0" cy="0" r="70" fill="none" stroke="#475569" stroke-width="2"/>
        <line x1="-70" y1="0" x2="70" y2="0" stroke="#1e293b" stroke-width="16"/>
        <line x1="0" y1="0" x2="0" y2="70" stroke="#1e293b" stroke-width="16"/>
        <circle cx="0" cy="0" r="20" fill="#0f172a" stroke="#60a5fa" stroke-width="2"/>
        <text x="0" y="4" fill="#60a5fa" font-size="10" font-weight="bold" text-anchor="middle">SDV</text>
      </g>
    `;
  }}

  // Initial draw
  renderFrame(0);
</script>
</body>
</html>
"""
    return html


# ==============================================================================
# INDIVIDUAL STATIC RENDERERS
# ==============================================================================
def render_top_down_view(frame: Dict[str, Any], initial_dist: float) -> str:
    """Wrapper that returns an SVG string for top-down view."""
    return f"""<div style="background:#0f172a; padding:10px; border-radius:8px;">
      <svg viewBox="0 0 1000 310" xmlns="http://www.w3.org/2000/svg">
        <rect width="1000" height="310" fill="#1e2430"/>
        <text x="500" y="155" fill="#38bdf8" font-size="18" text-anchor="middle">Top-Down View (Live in Player)</text>
      </svg>
    </div>"""

def render_side_view(frame: Dict[str, Any], initial_dist: float) -> str:
    return render_top_down_view(frame, initial_dist)

def render_third_person_view(frame: Dict[str, Any], initial_dist: float) -> str:
    return render_top_down_view(frame, initial_dist)

def render_driver_pov_view(frame: Dict[str, Any], initial_dist: float) -> str:
    return render_top_down_view(frame, initial_dist)


# ==============================================================================
# TAB 3: SDV ARCHITECTURE & LIVE SIGNALS DIAGRAM
# ==============================================================================
def render_architecture_diagram(current_state: str, is_braking: bool, bus: Dict[str, Any]) -> str:
    """
    Renders an interactive Software-Defined Vehicle system architecture diagram:
    - Virtual Sensors (Perception HAL)
    - Vehicle Signal Bus (COVESA VSS Model)
    - AEB Controller Software (ECU)
    - Virtual Actuator (Brake-by-Wire)
    - Vehicle Dynamics & Kinematics Engine
    - Feedback Loop back to sensors
    Highlights live signal values and active data pathways.
    """
    theme = get_aeb_colors(current_state)
    active_path_stroke = theme["stroke"]

    svg = f"""<div style="background: #0f172a; border-radius: 12px; padding: 18px 22px; border: 1px solid #1e293b; margin-bottom: 20px;">
<div style="color: #94a3b8; font-size: 13px; font-family: system-ui, sans-serif; margin-bottom: 12px;">
    <b style="color: #60a5fa;">SYSTEM ARCHITECTURE:</b> Software-Defined Vehicle (SDV) Closed-Loop Perception & Actuation
</div>
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 320" style="width: 100%; height: 320px; background: #020617; border-radius: 8px; display: block;">
  <defs>
    <marker id="arrow" viewBox="0 0 10 10" refX="6" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
      <path d="M 0 1 L 8 5 L 0 9 z" fill="#38bdf8"/>
    </marker>
    <marker id="arrow-active" viewBox="0 0 10 10" refX="6" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
      <path d="M 0 1 L 8 5 L 0 9 z" fill="{active_path_stroke}"/>
    </marker>
  </defs>

  <!-- 1. VIRTUAL SENSOR LAYER (HAL) -->
  <g transform="translate(30, 40)">
    <rect x="0" y="0" width="190" height="95" rx="8" fill="#1e293b" stroke="#38bdf8" stroke-width="2"/>
    <text x="95" y="24" fill="#38bdf8" font-size="12" font-weight="bold" font-family="sans-serif" text-anchor="middle">📡 VIRTUAL SENSORS (HAL)</text>
    <line x1="15" y1="32" x2="175" y2="32" stroke="#334155" stroke-width="1"/>
    <text x="18" y="52" fill="#cbd5e1" font-size="11" font-family="sans-serif">• Radar / LiDAR Distance</text>
    <text x="18" y="70" fill="#94a3b8" font-size="10" font-family="sans-serif">  Reading: <tspan fill="#facc15" font-weight="bold">{bus.get('sensor_measured_distance', 0.0):.1f} m</tspan></text>
    <text x="18" y="86" fill="#cbd5e1" font-size="11" font-family="sans-serif">• Wheel Speed Sensor</text>
  </g>

  <!-- Arrow: Sensors -> Bus -->
  <line x1="220" y1="87" x2="275" y2="87" stroke="#38bdf8" stroke-width="2.5" marker-end="url(#arrow)"/>

  <!-- 2. VEHICLE SIGNAL BUS (COVESA VSS) -->
  <g transform="translate(280, 25)">
    <rect x="0" y="0" width="190" height="260" rx="10" fill="#0f172a" stroke="#818cf8" stroke-width="2.5"/>
    <text x="95" y="26" fill="#818cf8" font-size="12" font-weight="bold" font-family="sans-serif" text-anchor="middle">⚡ VEHICLE SIGNAL BUS</text>
    <text x="95" y="42" fill="#94a3b8" font-size="10" font-family="sans-serif" text-anchor="middle">(COVESA VSS Model)</text>
    <line x1="15" y1="50" x2="175" y2="50" stroke="#334155" stroke-width="1"/>

    <text x="15" y="72" fill="#64748b" font-size="9" font-family="monospace">Vehicle.Speed</text>
    <text x="175" y="72" fill="#38bdf8" font-size="10" font-weight="bold" font-family="monospace" text-anchor="end">{bus.get('speed_kmh', 0.0):.1f} km/h</text>

    <text x="15" y="100" fill="#64748b" font-size="9" font-family="monospace">Vehicle.ObstacleDist</text>
    <text x="175" y="100" fill="#facc15" font-size="10" font-weight="bold" font-family="monospace" text-anchor="end">{bus.get('distance_to_pedestrian', 0.0):.1f} m</text>

    <text x="15" y="128" fill="#64748b" font-size="9" font-family="monospace">Vehicle.TTC</text>
    <text x="175" y="128" fill="#c084fc" font-size="10" font-weight="bold" font-family="monospace" text-anchor="end">{bus.get('ttc', 0.0):.2f} s</text>

    <text x="15" y="156" fill="#64748b" font-size="9" font-family="monospace">Vehicle.AEBState</text>
    <text x="175" y="156" fill="{theme['stroke']}" font-size="10" font-weight="bold" font-family="monospace" text-anchor="end">{current_state}</text>

    <text x="15" y="184" fill="#64748b" font-size="9" font-family="monospace">Vehicle.BrakePressure</text>
    <text x="175" y="184" fill="#f87171" font-size="10" font-weight="bold" font-family="monospace" text-anchor="end">{bus.get('actual_brake_pressure', 0.0):.0f}%</text>

    <text x="15" y="212" fill="#64748b" font-size="9" font-family="monospace">Vehicle.Position</text>
    <text x="175" y="212" fill="#38bdf8" font-size="10" font-weight="bold" font-family="monospace" text-anchor="end">{bus.get('position', 0.0):.1f} m</text>

    <text x="15" y="240" fill="#64748b" font-size="9" font-family="monospace">Vehicle.Collision</text>
    <text x="175" y="240" fill="#4ade80" font-size="10" font-weight="bold" font-family="monospace" text-anchor="end">{bus.get('collision_status', 'RUNNING')}</text>
  </g>

  <!-- Arrow: Bus -> ECU -->
  <line x1="470" y1="87" x2="525" y2="87" stroke="#38bdf8" stroke-width="2.5" marker-end="url(#arrow)"/>

  <!-- 3. ADAS AEB SOFTWARE SERVICE (ECU) -->
  <g transform="translate(530, 40)">
    <rect x="0" y="0" width="200" height="95" rx="8" fill="#1e293b" stroke="{theme['stroke']}" stroke-width="2.5"/>
    <text x="100" y="24" fill="{theme['stroke']}" font-size="12" font-weight="bold" font-family="sans-serif" text-anchor="middle">🧠 ADAS AEB CONTROLLER</text>
    <line x1="15" y1="32" x2="185" y2="32" stroke="#334155" stroke-width="1"/>
    <text x="18" y="52" fill="#cbd5e1" font-size="11" font-family="sans-serif">• TTC Estimator (d / v_rel)</text>
    <text x="18" y="70" fill="#cbd5e1" font-size="11" font-family="sans-serif">• 4-State Safety Engine</text>
    <text x="18" y="86" fill="{theme['stroke']}" font-size="10" font-weight="bold" font-family="sans-serif">  State: {current_state}</text>
  </g>

  <!-- Arrow: ECU -> Actuator -->
  <line x1="730" y1="87" x2="785" y2="87" stroke="{active_path_stroke}" stroke-width="2.5" marker-end="url(#arrow-active)"/>

  <!-- 4. VIRTUAL ACTUATOR (BRAKE-BY-WIRE) -->
  <g transform="translate(790, 40)">
    <rect x="0" y="0" width="180" height="95" rx="8" fill="#1e293b" stroke="#f43f5e" stroke-width="2"/>
    <text x="90" y="24" fill="#f43f5e" font-size="12" font-weight="bold" font-family="sans-serif" text-anchor="middle">🛑 BRAKE ACTUATOR</text>
    <line x1="15" y1="32" x2="165" y2="32" stroke="#334155" stroke-width="1"/>
    <text x="16" y="52" fill="#cbd5e1" font-size="11" font-family="sans-serif">• Electro-Hydraulic Lag</text>
    <text x="16" y="70" fill="#94a3b8" font-size="10" font-family="sans-serif">  dP/dt = (P_tgt - P) / τ</text>
    <text x="16" y="86" fill="#f87171" font-size="10" font-weight="bold" font-family="sans-serif">  Pressure: {bus.get('actual_brake_pressure', 0.0):.0f}%</text>
  </g>

  <!-- Arrow: Actuator -> Dynamics -->
  <path d="M 880,135 L 880,185 L 880,205" stroke="{active_path_stroke}" stroke-width="2.5" fill="none" marker-end="url(#arrow-active)"/>

  <!-- 5. 1D VEHICLE DYNAMICS ENGINE -->
  <g transform="translate(730, 210)">
    <rect x="0" y="0" width="240" height="85" rx="8" fill="#1e293b" stroke="#34d399" stroke-width="2"/>
    <text x="120" y="24" fill="#34d399" font-size="12" font-weight="bold" font-family="sans-serif" text-anchor="middle">🚗 VEHICLE DYNAMICS (1D SI)</text>
    <line x1="15" y1="32" x2="225" y2="32" stroke="#334155" stroke-width="1"/>
    <text x="18" y="50" fill="#cbd5e1" font-size="11" font-family="sans-serif">• F_net = F_brake + F_drag + F_roll</text>
    <text x="18" y="68" fill="#34d399" font-size="10" font-weight="bold" font-family="sans-serif">  Speed: {bus.get('speed_kmh', 0.0):.1f} km/h | Accel: {bus.get('acceleration', 0.0):.2f} m/s²</text>
  </g>

  <!-- Feedback Loop Arrow: Dynamics -> Sensors -->
  <path d="M 730,252 L 125,252 L 125,140" stroke="#38bdf8" stroke-width="2.5" stroke-dasharray="5,4" fill="none" marker-end="url(#arrow)"/>
  <rect x="360" y="244" width="220" height="18" rx="4" fill="#020617" stroke="#38bdf8" stroke-width="1"/>
  <text x="470" y="257" fill="#38bdf8" font-size="10" font-family="sans-serif" font-weight="bold" text-anchor="middle">⟵ Dynamic State Feedback Loop (50 Hz)</text>
</svg>
</div>"""
    return svg
