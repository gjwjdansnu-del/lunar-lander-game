// ─── Constants ───────────────────────────────────────────────────────────────
const G_MOON = 1.62;           // m/s² lunar gravity
const PIXELS_PER_METER = 3.5;  // rendering scale (lower = more zoomed out)
const LANDER_VISUAL_SCALE = 2.4; // draw lander larger without changing physics/camera
const CAM_Y_LEAD = 0;          // 0 = lander always at screen center
const DT = 1 / 120;            // fixed physics timestep (s)
const MAX_SUBSTEPS = 4;

// Lander geometry (meters)
const LANDER_W = 3.2;
const LANDER_H = 2.4;
const LEG_LEN = 1.6;
const FOOT_PAD_DROP = 0.05;    // visual foot pad extends below leg tip
const COLLISION_SCALE = LANDER_VISUAL_SCALE; // match rendered lander size
const ENGINE_OFFSET_Y = LANDER_H / 2; // engine below center

// Physics
const LANDER_MASS = 1200;      // kg (with fuel)
const DRY_MASS = 400;          // kg (empty)
const MAX_FUEL = 100;          // %
const FUEL_MASS = LANDER_MASS - DRY_MASS;

// Thrust
const MAIN_THRUST = 3200;      // N
const MAIN_FUEL_RATE = 0.35;   // %/s at full thrust
const ATT_THRUST = 180;        // N per side thruster
const ATT_FUEL_RATE = 0.12;    // %/s per active side thruster
const ATT_ARM = LANDER_W / 2;  // moment arm for attitude thrusters

// Landing criteria
const MAX_LANDING_VN = 3.0;      // m/s into ground along surface normal
const MAX_LANDING_VT = 1.4;      // m/s along surface
const MAX_LANDING_ANGLE = 14;    // degrees body tilt from upright
const MAX_LANDING_OMEGA = 0.22;  // rad/s max spin at touchdown
const CRASH_SPEED = 5.0;
const CRASH_ANGLE = 50;
const CRASH_OMEGA = 0.55;        // rad/s

// World
const WORLD_WIDTH_M = 4000;
const TERRAIN_SEGMENTS = 300;
const START_FUEL = 20;       // actual fuel tank (1/5 of original 100%)
const START_VX = 36;           // m/s horizontal at start (2× original 18)
const FUEL_DISPLAY_SCALE = 100 / START_FUEL; // show 100% at full tank
const RCS_FLAME_SCALE = 5;

// Minimap
const MINIMAP = { w: 220, h: 130, margin: 10 };

// ─── Canvas setup ──────────────────────────────────────────────────────────────
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const W = 1280;
const H = 720;
canvas.width = W;
canvas.height = H;

// ─── Input ───────────────────────────────────────────────────────────────────
const keys = {};
window.addEventListener('keydown', e => { keys[e.code] = true; e.preventDefault(); });
window.addEventListener('keyup', e => { keys[e.code] = false; });

// ─── Terrain generation (midpoint displacement) ─────────────────────────────
let terrainCraters = [];

function generateTerrain(seed) {
  const rng = mulberry32(seed);
  const segW = WORLD_WIDTH_M / TERRAIN_SEGMENTS;

  const craters = [];
  const numCraters = 10 + Math.floor(rng() * 12);
  for (let c = 0; c < numCraters; c++) {
    craters.push({
      cx: rng() * WORLD_WIDTH_M,
      cr: 25 + rng() * 70,
      depth: 5 + rng() * 14,
      rim: 2 + rng() * 4,
    });
  }
  terrainCraters = craters;

  const heights = new Float64Array(TERRAIN_SEGMENTS + 1);
  for (let i = 0; i <= TERRAIN_SEGMENTS; i++) {
    heights[i] = 70 + rng() * 50;
  }

  function displace(lo, hi, amount) {
    if (hi - lo <= 1) return;
    const mid = (lo + hi) >> 1;
    heights[mid] = (heights[lo] + heights[hi]) / 2 + (rng() - 0.5) * amount;
    const half = amount * 0.5;
    displace(lo, mid, half);
    displace(mid, hi, half);
  }
  displace(0, TERRAIN_SEGMENTS, 90);

  const pts = [];
  for (let i = 0; i <= TERRAIN_SEGMENTS; i++) {
    const x = i * segW;
    let y = heights[i];

    // gentle rolling hills
    y += Math.sin(x * 0.005) * 12;
    y += Math.sin(x * 0.014 + 1.8) * 6;
    y += (rng() - 0.5) * 3;

    for (const crater of craters) {
      let dx = x - crater.cx;
      dx -= Math.round(dx / WORLD_WIDTH_M) * WORLD_WIDTH_M;
      const dist = Math.abs(dx);
      if (dist < crater.cr) {
        const t = dist / crater.cr;
        y -= crater.depth * (1 - t * t);
        if (dist > crater.cr * 0.8) {
          y += crater.rim * (1 - Math.abs(t - 0.95) / 0.05);
        }
      }
    }

    pts.push({ x, y });
  }

  // seamless horizontal loop
  const blend = 40;
  for (let i = 0; i <= blend; i++) {
    const t = i / blend;
    const ri = pts.length - 1 - i;
    const avg = (pts[i].y + pts[ri].y) / 2;
    pts[i].y = pts[i].y * (1 - t * 0.6) + avg * (t * 0.6);
    pts[ri].y = pts[ri].y * (1 - t * 0.6) + avg * (t * 0.6);
  }
  pts[pts.length - 1].y = pts[0].y;

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

function wrapX(x) {
  x = x % WORLD_WIDTH_M;
  if (x < 0) x += WORLD_WIDTH_M;
  return x;
}

function wrapDelta(wx, camX) {
  let dx = wx - camX;
  dx -= Math.round(dx / WORLD_WIDTH_M) * WORLD_WIDTH_M;
  return dx;
}

function toScreenX(wx, camX) {
  return wrapDelta(wx, camX) * PIXELS_PER_METER + W / 2;
}

function toScreenY(wy, camY) {
  return (wy - camY) * PIXELS_PER_METER + H / 2;
}

function terrainHeightAt(terrain, x) {
  x = wrapX(x);
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
      const nozzle = this.getAttNozzleWorldPos(1);
      this.spawnAttParticle(nozzle.x, nozzle.y, fx, fy);
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
      const nozzle = this.getAttNozzleWorldPos(-1);
      this.spawnAttParticle(nozzle.x, nozzle.y, fx, fy);
    }

    // Integrate position & angle (semi-implicit Euler)
    this.x += this.vx * DT;
    this.y += this.vy * DT;
    this.x = wrapX(this.x);
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

  getLegContactPoints() {
    const cos = Math.cos(this.theta);
    const sin = Math.sin(this.theta);
    const hw = (LANDER_W / 2) * COLLISION_SCALE;
    const legTipY = (LANDER_H / 2 + LEG_LEN + FOOT_PAD_DROP) * COLLISION_SCALE;
    const legLocal = [
      { x: -hw * 0.7 * 1.1, y: legTipY },
      { x: hw * 0.7 * 1.1, y: legTipY },
    ];
    return legLocal.map(l => ({
      x: this.x + l.x * cos - l.y * sin,
      y: this.y + l.x * sin + l.y * cos,
    }));
  }

  checkTerrainCollision(terrain) {
    const checkPts = this.getLegContactPoints();
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
        const vNormal = Math.abs(this.vx * nx + this.vy * ny);
        const vTangent = Math.abs(this.vx * tx + this.vy * ty);

        if (vTangent <= MAX_LANDING_VT &&
            vNormal <= MAX_LANDING_VN &&
            angleDeg <= MAX_LANDING_ANGLE &&
            Math.abs(this.omega) <= MAX_LANDING_OMEGA) {
          this.landed = true;
          this.alive = false;
        } else if (speed > CRASH_SPEED || angleDeg > CRASH_ANGLE || Math.abs(this.omega) > CRASH_OMEGA) {
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
      // exhaust goes opposite to thrust (down from nozzle)
      this.particles.push({
        x: pos.x + (Math.random() - 0.5) * 0.3,
        y: pos.y + (Math.random() - 0.5) * 0.3,
        vx: Math.sin(this.theta + spread) * speed + this.vx,
        vy: Math.cos(this.theta + spread) * speed + this.vy,
        life: 0.3 + Math.random() * 0.3,
        maxLife: 0.6,
        size: 2 + intensity * 3,
        color: `hsl(${20 + Math.random() * 30}, 100%, ${50 + Math.random() * 30}%)`,
      });
    }
  }

  getAttNozzleWorldPos(side) {
    const cos = Math.cos(this.theta);
    const sin = Math.sin(this.theta);
    const lx = side * (LANDER_W / 2) * LANDER_VISUAL_SCALE;
    const ly = -(LANDER_H / 2) * LANDER_VISUAL_SCALE;
    return {
      x: this.x + lx * cos - ly * sin,
      y: this.y + lx * sin + ly * cos,
    };
  }

  spawnAttParticle(px, py, fx, fy) {
    const mag = Math.sqrt(fx * fx + fy * fy) || 1;
    const ex = -(fx / mag);
    const ey = -(fy / mag);
    this.particles.push({
      x: px,
      y: py,
      vx: ex * (16 + Math.random() * 8) + this.vx,
      vy: ey * (16 + Math.random() * 8) + this.vy,
      life: 0.04 + Math.random() * 0.05,
      maxLife: 0.09,
      size: RCS_FLAME_SCALE,
      color: Math.random() > 0.45 ? '#eef6ff' : '#fff0b0',
      rcs: true,
    });
  }

  drawRcsFlame(ctx, nx, ny, dirX, dirY) {
    const s = RCS_FLAME_SCALE;
    const len = 0.5 * s;
    const halfW = 0.045 * s;
    const tipX = nx + dirX * len;
    const tipY = ny + dirY * len;
    const px = -dirY;
    const py = dirX;

    const grad = ctx.createLinearGradient(nx, ny, tipX, tipY);
    grad.addColorStop(0, 'rgba(240, 248, 255, 0.95)');
    grad.addColorStop(0.3, 'rgba(255, 235, 160, 0.8)');
    grad.addColorStop(0.65, 'rgba(255, 180, 60, 0.45)');
    grad.addColorStop(1, 'rgba(255, 120, 20, 0)');

    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.moveTo(nx + px * halfW, ny + py * halfW);
    ctx.lineTo(tipX, tipY);
    ctx.lineTo(nx - px * halfW, ny - py * halfW);
    ctx.closePath();
    ctx.fill();

    ctx.fillStyle = 'rgba(160, 210, 255, 0.85)';
    ctx.beginPath();
    ctx.ellipse(
      nx + dirX * 0.06 * s, ny + dirY * 0.06 * s,
      0.04 * s, 0.025 * s, Math.atan2(dirY, dirX), 0, Math.PI * 2
    );
    ctx.fill();
  }

  draw(ctx, camX, camY) {
    const sx = toScreenX(this.x, camX);
    const sy = toScreenY(this.y, camY);

    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-this.theta); // canvas y is down, negate for correct visual
    ctx.scale(PIXELS_PER_METER * LANDER_VISUAL_SCALE, PIXELS_PER_METER * LANDER_VISUAL_SCALE);

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

    // RCS thrusters at top corners
    ctx.fillStyle = '#666';
    ctx.beginPath();
    ctx.arc(hw, -hh, 0.09, 0, Math.PI * 2);
    ctx.arc(-hw, -hh, 0.09, 0, Math.PI * 2);
    ctx.fill();

    // ← : right corner fires +x (CCW torque) | → : left corner fires -x
    if (this.attLeft) {
      this.drawRcsFlame(ctx, hw, -hh, 1, 0);
    }
    if (this.attRight) {
      this.drawRcsFlame(ctx, -hw, -hh, -1, 0);
    }

    ctx.restore();

    // Particles
    for (const p of this.particles) {
      const alpha = p.life / p.maxLife;
      const psx = toScreenX(p.x, camX);
      const psy = toScreenY(p.y, camY);
      ctx.globalAlpha = alpha;
      ctx.fillStyle = p.color;
      if (p.rcs) {
        const flicker = p.size * (0.4 + (1 - alpha) * 0.2);
        ctx.fillRect(psx - flicker, psy - flicker * 0.5, flicker * 2, flicker);
      } else {
        ctx.beginPath();
        ctx.arc(psx, psy, p.size * alpha, 0, Math.PI * 2);
        ctx.fill();
      }
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

  const startX = wrapX(WORLD_WIDTH_M * (0.25 + Math.random() * 0.5));
  const groundY = terrainHeightAt(terrain, startX);
  const startY = groundY - 120; // start 120m above ground
  const startVx = START_VX;
  const startFuel = START_FUEL;

  lander = new Lander(startX, startY, startVx, 0, startFuel);
  camX = lander.x;
  camY = lander.y;
  gameState = 'playing';
  overlay.classList.add('hidden');
}

function fuelDisplayPct(actual) {
  return actual * FUEL_DISPLAY_SCALE;
}

function updateHUD() {
  const groundY = terrainHeightAt(terrain, lander.x);
  const alt = Math.max(0, groundY - lander.y);
  document.getElementById('fuel').textContent = fuelDisplayPct(lander.fuel).toFixed(1);
  document.getElementById('altitude').textContent = alt.toFixed(1);
  document.getElementById('vx').textContent = lander.vx.toFixed(2);
  document.getElementById('vy').textContent = lander.vy.toFixed(2);
  document.getElementById('angle').textContent = (lander.theta * 180 / Math.PI).toFixed(1);
}

function drawArrow(ctx, x1, y1, x2, y2, color, lineWidth = 1.5, headLen = 9) {
  const dx = x2 - x1;
  const dy = y2 - y1;
  const len = Math.sqrt(dx * dx + dy * dy);
  if (len < 4) return;

  const angle = Math.atan2(dy, dx);
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - headLen * Math.cos(angle - Math.PI / 7), y2 - headLen * Math.sin(angle - Math.PI / 7));
  ctx.lineTo(x2 - headLen * 0.6 * Math.cos(angle), y2 - headLen * 0.6 * Math.sin(angle));
  ctx.lineTo(x2 - headLen * Math.cos(angle + Math.PI / 7), y2 - headLen * Math.sin(angle + Math.PI / 7));
  ctx.closePath();
  ctx.fill();
}

function getTerrainBounds() {
  let minY = Infinity;
  let maxY = -Infinity;
  for (const pt of terrain) {
    if (pt.y < minY) minY = pt.y;
    if (pt.y > maxY) maxY = pt.y;
  }
  return { minY: minY - 180, maxY: maxY + 60 };
}

function drawMinimap() {
  const { w, h, margin } = MINIMAP;
  const mx = W - w - margin;
  const my = margin;
  const bounds = getTerrainBounds();
  const worldH = bounds.maxY - bounds.minY;
  const scaleX = w / WORLD_WIDTH_M;
  const scaleY = h / worldH;

  const toMapX = x => mx + x * scaleX;
  const toMapY = y => my + (y - bounds.minY) * scaleY;

  ctx.fillStyle = 'rgba(0, 0, 16, 0.82)';
  ctx.fillRect(mx, my, w, h);
  ctx.strokeStyle = 'rgba(0, 255, 100, 0.45)';
  ctx.lineWidth = 1;
  ctx.strokeRect(mx, my, w, h);

  ctx.font = '9px monospace';
  ctx.fillStyle = 'rgba(0, 255, 100, 0.7)';
  ctx.fillText('MAP', mx + 6, my + 12);

  // terrain fill (down to minimap bottom — no floating horizontal band)
  const mapBottom = my + h;
  ctx.beginPath();
  ctx.moveTo(toMapX(terrain[0].x), mapBottom);
  for (const pt of terrain) {
    ctx.lineTo(toMapX(pt.x), toMapY(pt.y));
  }
  ctx.lineTo(toMapX(terrain[terrain.length - 1].x), mapBottom);
  ctx.closePath();
  ctx.fillStyle = '#6e6e78';
  ctx.fill();

  ctx.beginPath();
  for (let i = 0; i < terrain.length; i++) {
    const pt = terrain[i];
    if (i === 0) ctx.moveTo(toMapX(pt.x), toMapY(pt.y));
    else ctx.lineTo(toMapX(pt.x), toMapY(pt.y));
  }
  ctx.strokeStyle = '#a8a8b2';
  ctx.lineWidth = 1;
  ctx.stroke();

  // viewport (centered on lander)
  const viewW = W / PIXELS_PER_METER;
  const viewH = H / PIXELS_PER_METER;
  const vx = wrapX(lander.x) - viewW / 2;
  const vy = lander.y - viewH / 2;
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.35)';
  ctx.lineWidth = 1;
  ctx.strokeRect(toMapX(vx), toMapY(vy), viewW * scaleX, viewH * scaleY);

  // lander
  const lx = toMapX(wrapX(lander.x));
  const ly = toMapY(lander.y);
  ctx.fillStyle = '#ffffff';
  ctx.beginPath();
  ctx.arc(lx, ly, 3, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#ffcc00';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(lx, ly);
  ctx.lineTo(lx + lander.vx * 1.2, ly + lander.vy * 1.2);
  ctx.stroke();
}

function drawTerrainTiles(camX, camY) {
  const screenBottom = H + 100;
  const MOON_FILL = '#7a7a84';
  const MOON_SURFACE = '#b0b0ba';
  const MAX_SEG_DX = W * 0.55;

  for (let tile = -1; tile <= 1; tile++) {
    const offset = tile * WORLD_WIDTH_M;

    const surfacePts = terrain.map(pt => ({
      sx: toScreenX(pt.x + offset, camX),
      sy: toScreenY(pt.y, camY),
    }));

    // segment quads below surface — never seal with a screen-wide horizontal line
    ctx.fillStyle = MOON_FILL;
    for (let i = 0; i < surfacePts.length - 1; i++) {
      const p0 = surfacePts[i];
      const p1 = surfacePts[i + 1];
      if (Math.abs(p1.sx - p0.sx) > MAX_SEG_DX) continue;
      ctx.beginPath();
      ctx.moveTo(p0.sx, p0.sy);
      ctx.lineTo(p1.sx, p1.sy);
      ctx.lineTo(p1.sx, screenBottom);
      ctx.lineTo(p0.sx, screenBottom);
      ctx.closePath();
      ctx.fill();
    }

    ctx.beginPath();
    let drawing = false;
    for (let i = 0; i < surfacePts.length; i++) {
      const p = surfacePts[i];
      if (i > 0 && Math.abs(p.sx - surfacePts[i - 1].sx) > MAX_SEG_DX) {
        drawing = false;
      }
      if (!drawing) {
        ctx.moveTo(p.sx, p.sy);
        drawing = true;
      } else {
        ctx.lineTo(p.sx, p.sy);
      }
    }
    ctx.strokeStyle = MOON_SURFACE;
    ctx.lineWidth = 1.2;
    ctx.stroke();
  }
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
    const sx = toScreenX(starRng() * WORLD_WIDTH_M, camX);
    const sy = starRng() * H * 0.6;
    if (sx > -2 && sx < W + 2) {
      ctx.fillStyle = `rgba(255,255,255,${0.3 + starRng() * 0.7})`;
      ctx.fillRect(sx, sy, 1 + starRng(), 1 + starRng());
    }
  }

  drawTerrainTiles(camX, camY);

  // Velocity vector with arrowhead (lander at screen center)
  const vsx = W / 2;
  const vsy = H / 2;
  const vex = vsx + lander.vx * 10;
  const vey = vsy + lander.vy * 10;
  drawArrow(ctx, vsx, vsy, vex, vey, 'rgba(255,255,0,0.75)', 2, 10);

  lander.draw(ctx, camX, camY);

  drawMinimap();
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

    camX = lander.x;
    camY = lander.y;

    updateHUD();

    if (lander.landed) {
      gameState = 'won';
      const fuelLeft = fuelDisplayPct(lander.fuel).toFixed(1);
      messageEl.innerHTML = '🎉 착륙 성공!<br><span style="font-size:16px;color:#aaa">남은 연료: ' + fuelLeft + '%</span>';
      overlay.classList.remove('hidden');
    } else if (lander.crashed) {
      gameState = 'lost';
      messageEl.innerHTML = '💥 충돌!<br><span style="font-size:16px;color:#aaa">속도·자세·회전이 너무 위험했습니다</span>';
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
