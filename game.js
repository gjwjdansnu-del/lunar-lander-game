// ─── Constants ───────────────────────────────────────────────────────────────
const G_MOON = 1.62;           // m/s² lunar gravity
const PIXELS_PER_METER = 8;  // rendering scale
const DT = 1 / 120;            // fixed physics timestep (s)
const MAX_SUBSTEPS = 4;

// Lander geometry (meters)
const LANDER_W = 3.2;
const LANDER_H = 2.4;
const LEG_LEN = 1.6;
const ENGINE_OFFSET_Y = LANDER_H / 2; // engine below center

// Physics
const LANDER_MASS = 1200;      // kg (with fuel)
const DRY_MASS = 400;          // kg (empty)
const MAX_FUEL = 100;          // %
const FUEL_MASS = LANDER_MASS - DRY_MASS;

// Thrust
const MAIN_THRUST = 3200;      // N
const MAIN_FUEL_RATE = 0.35;   // %/s at full thrust
const ATT_THRUST = 180;        // N per side thruster (torque only, applied at corners)
const ATT_FUEL_RATE = 0.12;    // %/s per active side thruster
const ATT_ARM = LANDER_W / 2;  // moment arm for attitude thrusters

// Landing criteria
const MAX_LANDING_VY = 2.5;    // m/s downward
const MAX_LANDING_VX = 1.5;    // m/s horizontal
const MAX_LANDING_ANGLE = 12;  // degrees from upright

// World
const WORLD_WIDTH_M = 4000;
const TERRAIN_SEGMENTS = 200;

// ─── Canvas setup ──────────────────────────────────────────────────────────────
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const W = 960;
const H = 540;
canvas.width = W;
canvas.height = H;

// ─── Input ───────────────────────────────────────────────────────────────────
const keys = {};
window.addEventListener('keydown', e => { keys[e.code] = true; e.preventDefault(); });
window.addEventListener('keyup', e => { keys[e.code] = false; });

// ─── Terrain generation (midpoint displacement) ─────────────────────────────
function generateTerrain(seed) {
  const rng = mulberry32(seed);
  const pts = [];
  const segW = WORLD_WIDTH_M / TERRAIN_SEGMENTS;

  // base heights
  const heights = new Float64Array(TERRAIN_SEGMENTS + 1);
  for (let i = 0; i <= TERRAIN_SEGMENTS; i++) {
    heights[i] = 80 + rng() * 60;
  }

  // midpoint displacement on coarse grid, then interpolate
  function displace(lo, hi, amount) {
    if (hi - lo <= 1) return;
    const mid = (lo + hi) >> 1;
    heights[mid] = (heights[lo] + heights[hi]) / 2 + (rng() - 0.5) * amount;
    const half = amount * 0.55;
    displace(lo, mid, half);
    displace(mid, hi, half);
  }
  displace(0, TERRAIN_SEGMENTS, 120);

  // smooth & add craters
  for (let i = 0; i <= TERRAIN_SEGMENTS; i++) {
    const x = i * segW;
    let y = heights[i];
    // craters
    const numCraters = 3 + Math.floor(rng() * 4);
    for (let c = 0; c < numCraters; c++) {
      const cx = rng() * WORLD_WIDTH_M;
      const cr = 30 + rng() * 80;
      const depth = 8 + rng() * 20;
      const dx = x - cx;
      if (Math.abs(dx) < cr) {
        const bowl = depth * (1 - (dx / cr) ** 2);
        y -= bowl;
      }
    }
    pts.push({ x, y });
  }
  return pts;
}

function mulberry32(seed) {
  return function () {
    seed |= 0; seed = seed + 0x6D2B79F5 | 0;
    let t = Math.imul(seed ^ seed >>> 15, 1 | seed);
    t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t;
    return ((t ^ t >>> 14) >>> 0) / 4294967296;
  };
}

function terrainHeightAt(terrain, x) {
  if (x <= terrain[0].x) return terrain[0].y;
  if (x >= terrain[terrain.length - 1].x) return terrain[terrain.length - 1].y;
  let lo = 0, hi = terrain.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (terrain[mid].x <= x) lo = mid; else hi = mid;
  }
  const t = (x - terrain[lo].x) / (terrain[hi].x - terrain[lo].x);
  return terrain[lo].y + t * (terrain[hi].y - terrain[lo].y);
}

function terrainSlopeAt(terrain, x) {
  const dx = 0.5;
  const y1 = terrainHeightAt(terrain, x - dx);
  const y2 = terrainHeightAt(terrain, x + dx);
  return Math.atan2(y2 - y1, dx);
}

// ─── Lander rigid body ─────────────────────────────────────────────────────────
class Lander {
  constructor(x, y, vx, vy, fuel) {
    this.x = x;
    this.y = y;
    this.vx = vx;
    this.vy = vy;
    this.theta = 0;       // radians, 0 = upright
    this.omega = 0;       // rad/s
    this.fuel = fuel;     // %
    this.alive = true;
    this.landed = false;
    this.crashed = false;
    this.thrustOn = false;
    this.attLeft = false;
    this.attRight = false;
    this.particles = [];
  }

  get mass() {
    return DRY_MASS + (this.fuel / 100) * FUEL_MASS;
  }

  get inertia() {
    // solid rectangle: I = m/12 * (w² + h²)
    const m = this.mass;
    return m * (LANDER_W ** 2 + LANDER_H ** 2) / 12;
  }

  // corner offsets in world space
  getCorners() {
    const cos = Math.cos(this.theta);
    const sin = Math.sin(this.theta);
    const hw = LANDER_W / 2;
    const hh = LANDER_H / 2;
    const local = [
      [-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]
    ];
    return local.map(([lx, ly]) => ({
      x: this.x + lx * cos - ly * sin,
      y: this.y + lx * sin + ly * cos,
    }));
  }

  // engine nozzle position in world space
  getEnginePos() {
    const cos = Math.cos(this.theta);
    const sin = Math.sin(this.theta);
    return {
      x: this.x + ENGINE_OFFSET_Y * sin,
      y: this.y + ENGINE_OFFSET_Y * cos,
    };
  }

  applyForce(fx, fy, px, py) {
    const m = this.mass;
    // linear: F = ma → a = F/m
    this.vx += (fx / m) * DT;
    this.vy += (fy / m) * DT;
    // torque: τ = r × F (2D: τ = rx*Fy - ry*Fx)
    const rx = px - this.x;
    const ry = py - this.y;
    const torque = rx * fy - ry * fx;
    this.omega += (torque / this.inertia) * DT;
  }

  update(terrain) {
    if (!this.alive) return;

    const thrustKey = keys['ArrowUp'] || keys['KeyW'];
    const leftKey = keys['ArrowLeft'] || keys['KeyA'];
    const rightKey = keys['ArrowRight'] || keys['KeyD'];

    this.thrustOn = thrustKey && this.fuel > 0;
    this.attLeft = leftKey && this.fuel > 0;
    this.attRight = rightKey && this.fuel > 0;

    // Gravity
    this.applyForce(0, this.mass * G_MOON, this.x, this.y);

    // Main engine thrust (applied at nozzle, direction = lander up vector)
    if (this.thrustOn) {
      const thrust = MAIN_THRUST * (this.fuel > 0 ? 1 : 0);
      const fx = -thrust * Math.sin(this.theta);
      const fy = -thrust * Math.cos(this.theta);
      const eng = this.getEnginePos();
      this.applyForce(fx, fy, eng.x, eng.y);
      this.fuel = Math.max(0, this.fuel - MAIN_FUEL_RATE * DT);
      this.spawnExhaust(eng, thrust);
    }

    // Attitude thrusters (at top corners, create pure torque)
    if (this.attLeft && !this.attRight) {
      const cos = Math.cos(this.theta);
      const sin = Math.sin(this.theta);
      const px = this.x - ATT_ARM * cos + (LANDER_H / 2) * sin;
      const py = this.y - ATT_ARM * sin - (LANDER_H / 2) * cos;
      // right-side thruster fires → clockwise torque
      const fx = ATT_THRUST * cos;
      const fy = ATT_THRUST * sin;
      this.applyForce(fx, fy, px, py);
      this.fuel = Math.max(0, this.fuel - ATT_FUEL_RATE * DT);
      this.spawnAttParticle(px, py, fx, fy);
    }
    if (this.attRight && !this.attLeft) {
      const cos = Math.cos(this.theta);
      const sin = Math.sin(this.theta);
      const px = this.x + ATT_ARM * cos + (LANDER_H / 2) * sin;
      const py = this.y + ATT_ARM * sin - (LANDER_H / 2) * cos;
      const fx = -ATT_THRUST * cos;
      const fy = -ATT_THRUST * sin;
      this.applyForce(fx, fy, px, py);
      this.fuel = Math.max(0, this.fuel - ATT_FUEL_RATE * DT);
      this.spawnAttParticle(px, py, fx, fy);
    }

    // Integrate position & angle (semi-implicit Euler)
    this.x += this.vx * DT;
    this.y += this.vy * DT;
    this.theta += this.omega * DT;

    // Normalize angle
    while (this.theta > Math.PI) this.theta -= 2 * Math.PI;
    while (this.theta < -Math.PI) this.theta += 2 * Math.PI;

    // Collision with terrain (leg tips)
    this.checkTerrainCollision(terrain);

    // Update particles
    this.particles = this.particles.filter(p => {
      p.life -= DT;
      p.x += p.vx * DT;
      p.y += p.vy * DT;
      p.vy += G_MOON * 0.3 * DT;
      return p.life > 0;
    });
  }

  checkTerrainCollision(terrain) {
    const cos = Math.cos(this.theta);
    const sin = Math.sin(this.theta);
    const hw = LANDER_W / 2;
    const legLocal = [
      { x: -hw * 0.7, y: LANDER_H / 2 + LEG_LEN },
      { x: hw * 0.7, y: LANDER_H / 2 + LEG_LEN },
    ];
    const legs = legLocal.map(l => ({
      x: this.x + l.x * cos - l.y * sin,
      y: this.y + l.x * sin + l.y * cos,
    }));

    // Also check hull bottom corners
    const hullBottom = [
      { x: -hw, y: LANDER_H / 2 },
      { x: hw, y: LANDER_H / 2 },
    ].map(l => ({
      x: this.x + l.x * cos - l.y * sin,
      y: this.y + l.x * sin + l.y * cos,
    }));

    const checkPts = [...legs, ...hullBottom];
    let minPen = 0;
    let contactX = 0;
    let contactY = 0;
    let anyContact = false;

    for (const pt of checkPts) {
      const groundY = terrainHeightAt(terrain, pt.x);
      const pen = pt.y - groundY;
      if (pen > 0) {
        anyContact = true;
        if (pen > minPen) {
          minPen = pen;
          contactX = pt.x;
          contactY = groundY;
        }
      }
    }

    if (anyContact) {
      // Push lander up
      this.y -= minPen;

      // Ground normal from terrain slope
      const slope = terrainSlopeAt(terrain, contactX);
      const nx = -Math.sin(slope);
      const ny = -Math.cos(slope);

      // Relative velocity at contact
      const rx = contactX - this.x;
      const ry = contactY - this.y;
      const pvx = this.vx - this.omega * ry;
      const pvy = this.vy + this.omega * rx;

      // Normal impulse (inelastic, no bounce)
      const vn = pvx * nx + pvy * ny;
      if (vn > 0) {
        const m = this.mass;
        const I = this.inertia;
        const rCrossN = rx * ny - ry * nx;
        const denom = 1 / m + rCrossN ** 2 / I;
        const j = -vn / denom;

        this.vx += j * nx / m;
        this.vy += j * ny / m;
        this.omega += j * rCrossN / I;
      }

      // Friction
      const tx = Math.cos(slope);
      const ty = -Math.sin(slope);
      const vt = pvx * tx + pvy * ty;
      const friction = 0.6;
      const m = this.mass;
      const I = this.inertia;
      const rCrossT = rx * ty - ry * tx;
      const denom = 1 / m + rCrossT ** 2 / I;
      const maxFriction = friction * Math.abs(vn > 0 ? 0 : 1);
      let jt = -vt / denom;
      jt = Math.max(-maxFriction * m * G_MOON * DT * 10, Math.min(maxFriction * m * G_MOON * DT * 10, jt));
      this.vx += jt * tx / m;
      this.vy += jt * ty / m;
      this.omega += jt * rCrossT / I;

      // Landing / crash detection
      if (!this.landed && !this.crashed) {
        const speed = Math.sqrt(this.vx ** 2 + this.vy ** 2);
        const angleDeg = Math.abs(this.theta * 180 / Math.PI);
        const slopeDeg = Math.abs(slope * 180 / Math.PI);

        if (Math.abs(this.vy) <= MAX_LANDING_VY &&
            Math.abs(this.vx) <= MAX_LANDING_VX &&
            angleDeg <= MAX_LANDING_ANGLE &&
            slopeDeg <= 20) {
          this.landed = true;
          this.alive = false;
        } else if (speed > 4 || angleDeg > 45) {
          this.crashed = true;
          this.alive = false;
        }
      }
    }
  }

  spawnExhaust(pos, thrust) {
    const intensity = thrust / MAIN_THRUST;
    for (let i = 0; i < 3; i++) {
      const spread = (Math.random() - 0.5) * 0.4;
      const speed = 8 + Math.random() * 12;
      this.particles.push({
        x: pos.x + (Math.random() - 0.5) * 0.3,
        y: pos.y + (Math.random() - 0.5) * 0.3,
        vx: -Math.sin(this.theta + spread) * speed + this.vx,
        vy: -Math.cos(this.theta + spread) * speed + this.vy,
        life: 0.3 + Math.random() * 0.3,
        maxLife: 0.6,
        size: 2 + intensity * 3,
        color: `hsl(${20 + Math.random() * 30}, 100%, ${50 + Math.random() * 30}%)`,
      });
    }
  }

  spawnAttParticle(px, py, fx, fy) {
    const speed = 4 + Math.random() * 4;
    const mag = Math.sqrt(fx * fx + fy * fy) || 1;
    this.particles.push({
      x: px, y: py,
      vx: (fx / mag) * speed + this.vx,
      vy: (fy / mag) * speed + this.vy,
      life: 0.15 + Math.random() * 0.1,
      maxLife: 0.25,
      size: 1.5,
      color: '#88ccff',
    });
  }

  draw(ctx, camX, camY) {
    const sx = (this.x - camX) * PIXELS_PER_METER + W / 2;
    const sy = (this.y - camY) * PIXELS_PER_METER + H / 2;

    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-this.theta); // canvas y is down, negate for correct visual
    ctx.scale(PIXELS_PER_METER, PIXELS_PER_METER);

    // Legs
    ctx.strokeStyle = '#aaa';
    ctx.lineWidth = 0.12;
    const hw = LANDER_W / 2;
    const hh = LANDER_H / 2;
    for (const lx of [-hw * 0.7, hw * 0.7]) {
      ctx.beginPath();
      ctx.moveTo(lx, hh);
      ctx.lineTo(lx * 1.1, hh + LEG_LEN);
      ctx.stroke();
      // foot pad
      ctx.fillStyle = '#888';
      ctx.fillRect(lx * 1.1 - 0.25, hh + LEG_LEN - 0.1, 0.5, 0.15);
    }

    // Body
    ctx.fillStyle = '#c8c8d0';
    ctx.strokeStyle = '#888';
    ctx.lineWidth = 0.08;
    ctx.beginPath();
    ctx.roundRect(-hw, -hh, LANDER_W, LANDER_H, 0.3);
    ctx.fill();
    ctx.stroke();

    // Window
    ctx.fillStyle = '#4488cc';
    ctx.beginPath();
    ctx.arc(0, -hh * 0.2, 0.45, 0, Math.PI * 2);
    ctx.fill();

    // Engine bell
    ctx.fillStyle = '#666';
    ctx.beginPath();
    ctx.moveTo(-0.5, hh);
    ctx.lineTo(0.5, hh);
    ctx.lineTo(0.3, hh + 0.5);
    ctx.lineTo(-0.3, hh + 0.5);
    ctx.closePath();
    ctx.fill();

    // Attitude thruster indicators
    if (this.attLeft) {
      ctx.fillStyle = '#ff8800';
      ctx.fillRect(hw - 0.1, -hh - 0.1, 0.3, 0.2);
    }
    if (this.attRight) {
      ctx.fillStyle = '#ff8800';
      ctx.fillRect(-hw - 0.2, -hh - 0.1, 0.3, 0.2);
    }

    ctx.restore();

    // Particles
    for (const p of this.particles) {
      const alpha = p.life / p.maxLife;
      const psx = (p.x - camX) * PIXELS_PER_METER + W / 2;
      const psy = (p.y - camY) * PIXELS_PER_METER + H / 2;
      ctx.globalAlpha = alpha;
      ctx.fillStyle = p.color;
      ctx.beginPath();
      ctx.arc(psx, psy, p.size * alpha, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.globalAlpha = 1;
  }
}

// ─── Game state ────────────────────────────────────────────────────────────────
let terrain, lander, camX, camY, seed, gameState;
const overlay = document.getElementById('overlay');
const messageEl = document.getElementById('message');
const restartBtn = document.getElementById('restart-btn');

function initGame() {
  seed = Math.floor(Math.random() * 1e9);
  terrain = generateTerrain(seed);

  // Find a reasonable landing zone (relatively flat area)
  const segW = WORLD_WIDTH_M / TERRAIN_SEGMENTS;
  let bestX = WORLD_WIDTH_M * 0.5;
  let bestFlatness = Infinity;
  for (let i = 10; i < TERRAIN_SEGMENTS - 10; i++) {
    const x = i * segW;
    const s1 = terrainSlopeAt(terrain, x - 20);
    const s2 = terrainSlopeAt(terrain, x + 20);
    const flatness = Math.abs(s1) + Math.abs(s2);
    if (flatness < bestFlatness) {
      bestFlatness = flatness;
      bestX = x;
    }
  }

  const startX = bestX - 300;
  const groundY = terrainHeightAt(terrain, startX);
  const startY = groundY - 120; // start 120m above ground
  const startVx = 18; // m/s horizontal (fixed)
  const startFuel = 100;

  lander = new Lander(startX, startY, startVx, 0, startFuel);
  camX = lander.x;
  camY = lander.y;
  gameState = 'playing';
  overlay.classList.add('hidden');

  // Draw landing zone marker
  lander.targetX = bestX;
}

function updateHUD() {
  const groundY = terrainHeightAt(terrain, lander.x);
  const alt = Math.max(0, groundY - lander.y);
  document.getElementById('fuel').textContent = lander.fuel.toFixed(1);
  document.getElementById('altitude').textContent = alt.toFixed(1);
  document.getElementById('vx').textContent = lander.vx.toFixed(2);
  document.getElementById('vy').textContent = lander.vy.toFixed(2);
  document.getElementById('angle').textContent = (lander.theta * 180 / Math.PI).toFixed(1);
}

function drawWorld() {
  // Sky gradient
  const grad = ctx.createLinearGradient(0, 0, 0, H);
  grad.addColorStop(0, '#000008');
  grad.addColorStop(1, '#0a0a18');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, W, H);

  // Stars
  const starRng = mulberry32(seed);
  for (let i = 0; i < 120; i++) {
    const sx = (starRng() * WORLD_WIDTH_M - camX) * PIXELS_PER_METER + W / 2;
    const sy = starRng() * H * 0.6;
    if (sx > -2 && sx < W + 2) {
      ctx.fillStyle = `rgba(255,255,255,${0.3 + starRng() * 0.7})`;
      ctx.fillRect(sx, sy, 1 + starRng(), 1 + starRng());
    }
  }

  // Terrain
  ctx.beginPath();
  let started = false;
  for (const pt of terrain) {
    const sx = (pt.x - camX) * PIXELS_PER_METER + W / 2;
    const sy = (pt.y - camY) * PIXELS_PER_METER + H / 2;
    if (!started) { ctx.moveTo(sx, sy); started = true; }
    else ctx.lineTo(sx, sy);
  }
  const bottomY = (terrain[terrain.length - 1].y + 200 - camY) * PIXELS_PER_METER + H / 2;
  const leftX = (terrain[0].x - camX) * PIXELS_PER_METER + W / 2;
  const rightX = (terrain[terrain.length - 1].x - camX) * PIXELS_PER_METER + W / 2;
  ctx.lineTo(rightX, bottomY);
  ctx.lineTo(leftX, bottomY);
  ctx.closePath();
  ctx.fillStyle = '#4a4035';
  ctx.fill();
  ctx.strokeStyle = '#6a6055';
  ctx.lineWidth = 1;
  ctx.stroke();

  // Landing zone indicator
  if (lander.targetX) {
    const tx = (lander.targetX - camX) * PIXELS_PER_METER + W / 2;
    const ty = (terrainHeightAt(terrain, lander.targetX) - camY) * PIXELS_PER_METER + H / 2;
    ctx.strokeStyle = 'rgba(0,255,0,0.4)';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(tx - 40, ty);
    ctx.lineTo(tx + 40, ty);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = 'rgba(0,255,0,0.6)';
    ctx.font = '11px monospace';
    ctx.fillText('착륙 지점', tx - 24, ty - 6);
  }

  // Velocity vector
  const vsx = (lander.x - camX) * PIXELS_PER_METER + W / 2;
  const vsy = (lander.y - camY) * PIXELS_PER_METER + H / 2;
  ctx.strokeStyle = 'rgba(255,255,0,0.6)';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(vsx, vsy);
  ctx.lineTo(vsx + lander.vx * 4, vsy + lander.vy * 4);
  ctx.stroke();

  lander.draw(ctx, camX, camY);
}

let lastTime = 0;
let accumulator = 0;

function gameLoop(timestamp) {
  if (!lastTime) lastTime = timestamp;
  let frameTime = Math.min((timestamp - lastTime) / 1000, 0.05);
  lastTime = timestamp;
  accumulator += frameTime;

  if (gameState === 'playing') {
    let steps = 0;
    while (accumulator >= DT && steps < MAX_SUBSTEPS) {
      lander.update(terrain);
      accumulator -= DT;
      steps++;
    }

    // Camera follow
    camX += (lander.x - camX) * 0.08;
    camY += (lander.y - camY) * 0.06;

    updateHUD();

    if (lander.landed) {
      gameState = 'won';
      const fuelLeft = lander.fuel.toFixed(1);
      messageEl.innerHTML = '🎉 착륙 성공!<br><span style="font-size:16px;color:#aaa">남은 연료: ' + fuelLeft + '%</span>';
      overlay.classList.remove('hidden');
    } else if (lander.crashed) {
      gameState = 'lost';
      messageEl.innerHTML = '💥 충돌!<br><span style="font-size:16px;color:#aaa">속도나 자세가 너무 위험했습니다</span>';
      overlay.classList.remove('hidden');
    } else if (lander.y > terrainHeightAt(terrain, lander.x) + 500) {
      gameState = 'lost';
      messageEl.innerHTML = '💥 지형과 충돌!<br><span style="font-size:16px;color:#aaa">고도가 너무 낮았습니다</span>';
      overlay.classList.remove('hidden');
    }
  }

  drawWorld();
  requestAnimationFrame(gameLoop);
}

restartBtn.addEventListener('click', initGame);

initGame();
requestAnimationFrame(gameLoop);
