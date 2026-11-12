// Nordic UART Service (matches Bluefruit BLEUart in the firmware)
const UART_SERVICE = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const UART_TX_CHAR = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'; // device -> browser (notify)

// Posture from measured ranges (g). Each posture is a box [min, max] per axis.
// A reading inside a box scores 0; otherwise the score is how far outside it is.
// The closest posture wins, or Unknown if even the best is farther than TOLERANCE.
const POSTURES = {
  Supine: { x: [0.064, 0.194], y: [-0.977, -0.944], z: [0.087, 0.110] },
  Left:   { x: [0.172, 0.232], y: [-0.987, -0.954], z: [-0.089, -0.052] },
  Right:  { x: [0.015, 0.070], y: [-0.974, -0.950], z: [0.114, 0.284] },
};
const TOLERANCE = 0.08;
const outside = (v, [lo, hi]) => (v < lo ? lo - v : v > hi ? v - hi : 0);
function classify(x, y, z) {
  let best = 'Unknown', bd = Infinity;
  for (const [name, r] of Object.entries(POSTURES)) {
    const d = Math.hypot(outside(x, r.x), outside(y, r.y), outside(z, r.z));
    if (d < bd) { bd = d; best = name; }
  }
  return bd <= TOLERANCE ? best : 'Unknown';
}

const G = 9.80665;          // firmware sends m/s2 when it reports VIB_RMS
const VIB_THRESHOLD = 0.15; // same as VIBRATION_THRESHOLD in the firmware (m/s2)
const MAX_POINTS = 300;   // ~30 s at 10 Hz
const MAX_LOG_LINES = 200;

const $ = (id) => document.getElementById(id);
const els = {
  connect: $('connectBtn'), disconnect: $('disconnectBtn'), pause: $('pauseBtn'),
  clear: $('clearBtn'), csv: $('csvBtn'), dot: $('dot'), status: $('statusText'),
  vx: $('vx'), vy: $('vy'), vz: $('vz'), vm: $('vm'), count: $('count'), rate: $('rate'),
  posture: $('posture'), vibCard: $('vibCard'), vibState: $('vibState'), vibRms: $('vibRms'), vibCanvas: $('vibChart'), log: $('log'), canvas: $('chart'), unsupported: $('unsupported'),
};

let device = null, txChar = null;
let buffer = '';
let paused = false;
let points = [];      // {t, x, y, z} for chart
let allSamples = [];  // for CSV
let total = 0;
let recent = [];      // timestamps for rate calc
const decoder = new TextDecoder();

if (!navigator.bluetooth) {
  els.unsupported.hidden = false;
  els.connect.disabled = true;
}

function setStatus(text, on) {
  els.status.textContent = text;
  els.dot.classList.toggle('on', !!on);
}

function setConnected(c) {
  els.connect.disabled = c;
  els.disconnect.disabled = !c;
  els.pause.disabled = !c;
}

async function connect() {
  try {
    setStatus('Scanning…', false);
    device = await navigator.bluetooth.requestDevice({
      filters: [{ namePrefix: 'SleepQure' }, { services: [UART_SERVICE] }],
      optionalServices: [UART_SERVICE],
    });
    device.addEventListener('gattserverdisconnected', onDisconnected);
    await openGatt();
  } catch (err) {
    setStatus(err.name === 'NotFoundError' ? 'Disconnected' : 'Error: ' + err.message, false);
    console.error(err);
  }
}

async function openGatt() {
  setStatus('Connecting…', false);
  const server = await device.gatt.connect();
  const service = await server.getPrimaryService(UART_SERVICE);
  txChar = await service.getCharacteristic(UART_TX_CHAR);
  txChar.addEventListener('characteristicvaluechanged', onData);
  await txChar.startNotifications();
  buffer = '';
  setConnected(true);
  setStatus('Connected: ' + (device.name || 'device'), true);
}

function onDisconnected() {
  setConnected(false);
  setStatus('Disconnected', false);
  txChar = null;
}


function disconnect() {
  if (device && device.gatt.connected) device.gatt.disconnect();
}

// BLE notifications are chunked (~20 bytes), so assemble full lines first.
function onData(e) {
  buffer += decoder.decode(e.target.value);
  let idx;
  while ((idx = buffer.indexOf('\n')) >= 0) {
    const line = buffer.slice(0, idx).trim();
    buffer = buffer.slice(idx + 1);
    if (line) handleLine(line);
  }
}

// Expected: "X: 0.012 m/s2 | Y: -0.003 m/s2 | Z: 9.810 m/s2" (also accepts X=..,Y=..,Z=..)
// Browser-side vibration: remove slow drift (baseline), then RMS over the last few samples.
// Returns m/s2 so it matches the firmware's VIB_RMS and VIB_THRESHOLD.
let base = null, vibWin = [];
const BASE_ALPHA = 0.2, VIB_WINDOW = 5;
function computeRms(x, y, z) {
  if (!base) base = { x, y, z };
  base.x += BASE_ALPHA * (x - base.x);
  base.y += BASE_ALPHA * (y - base.y);
  base.z += BASE_ALPHA * (z - base.z);
  const dx = x - base.x, dy = y - base.y, dz = z - base.z;
  vibWin.push(dx * dx + dy * dy + dz * dz);
  if (vibWin.length > VIB_WINDOW) vibWin.shift();
  return Math.sqrt(vibWin.reduce((s, v) => s + v, 0) / vibWin.length) * G;
}

function handleLine(line) {
  const m = line.match(/X[:=]\s*(-?\d+\.?\d*).*?Y[:=]\s*(-?\d+\.?\d*).*?Z[:=]\s*(-?\d+\.?\d*)/);
  if (!m) return;
  if (paused) return;
  // Vibration firmware: "X=0.045,Y=-0.080,Z=9.760,VIB_RMS=0.2200,VIBRATION=DETECTED" (m/s2)
  const vr = line.match(/VIB_RMS\s*=\s*(-?\d+\.?\d*)/);
  const hasVib = !!vr;
  const k = hasVib ? 1 / G : 1;  // convert m/s2 -> g
  const x = +m[1] * k, y = +m[2] * k, z = +m[3] * k;
  let vib, detected;
  if (hasVib) {                       // device sent its own RMS
    vib = +vr[1];
    detected = /VIBRATION\s*[=:]\s*DETECTED/i.test(line);
  } else {                            // no RMS from device: compute from X/Y/Z movement
    vib = computeRms(x, y, z);
    detected = vib >= VIB_THRESHOLD;
  }
  const t = Date.now();

  total++;
  points.push({ t, x, y, z, vib, detected });
  if (points.length > MAX_POINTS) points.shift();
  allSamples.push({ t, x, y, z, vib, detected });
  recent.push(t);
  while (recent.length && t - recent[0] > 2000) recent.shift();

  els.vx.textContent = x.toFixed(3);
  els.vy.textContent = y.toFixed(3);
  els.vz.textContent = z.toFixed(3);
  els.vm.textContent = Math.sqrt(x * x + y * y + z * z).toFixed(3);
  const w = points.slice(-5);  // smooth over last 5 samples
  const avg = (k) => w.reduce((s, p) => s + p[k], 0) / w.length;
  els.posture.textContent = classify(avg('x'), avg('y'), avg('z'));
  {
    els.vibState.textContent = detected ? 'DETECTED' : 'None';
    els.vibRms.textContent = 'RMS ' + vib.toFixed(3) + ' m/s²';
    els.vibCard.classList.toggle('on', detected);
  }
  els.count.textContent = total;
  els.rate.textContent = (recent.length / 2).toFixed(1);
  els.csv.disabled = false;

  els.log.textContent += line + '\n';
  const lines = els.log.textContent.split('\n');
  if (lines.length > MAX_LOG_LINES) els.log.textContent = lines.slice(-MAX_LOG_LINES).join('\n');
  els.log.scrollTop = els.log.scrollHeight;

  draw();
  drawVib();
}

function draw() {
  const c = els.canvas, ctx = c.getContext('2d');
  const W = c.width, H = c.height, pad = 36;
  ctx.clearRect(0, 0, W, H);

  const range = 2; // +/- 2 g sensor range
  const yOf = (v) => H / 2 - (Math.max(-range, Math.min(range, v)) / range) * (H / 2 - pad / 2);

  ctx.font = '12px sans-serif';
  ctx.fillStyle = '#94a3b8';
  ctx.strokeStyle = '#243247';
  ctx.lineWidth = 1;
  for (let g = -2; g <= 2; g++) {
    const y = yOf(g);
    ctx.beginPath(); ctx.moveTo(pad, y); ctx.lineTo(W, y); ctx.stroke();
    ctx.fillText(g + 'g', 4, y + 4);
  }

  const step = (W - pad) / (MAX_POINTS - 1);
  const series = [['x', '#f87171'], ['y', '#4ade80'], ['z', '#60a5fa']];
  for (const [key, color] of series) {
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    points.forEach((p, i) => {
      const px = pad + i * step;
      i ? ctx.lineTo(px, yOf(p[key])) : ctx.moveTo(px, yOf(p[key]));
    });
    ctx.stroke();
  }
}

function drawVib() {
  const c = els.vibCanvas, ctx = c.getContext('2d');
  const W = c.width, H = c.height, pad = 36;
  ctx.clearRect(0, 0, W, H);
  const vals = points.filter((p) => p.vib !== null);
  const top = Math.max(VIB_THRESHOLD * 2, ...vals.map((p) => p.vib)) * 1.1;
  const yOf = (v) => H - 8 - (v / top) * (H - 16);
  ctx.font = '12px sans-serif';
  ctx.fillStyle = '#94a3b8';
  ctx.fillText('0', 4, H - 4);
  ctx.fillText(top.toFixed(2), 4, 14);
  // threshold
  ctx.strokeStyle = '#fbbf24'; ctx.setLineDash([6, 4]); ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(pad, yOf(VIB_THRESHOLD)); ctx.lineTo(W, yOf(VIB_THRESHOLD)); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = '#fbbf24';
  ctx.fillText(VIB_THRESHOLD, 4, yOf(VIB_THRESHOLD) + 4);
  // bars, red when detected
  const step = (W - pad) / (MAX_POINTS - 1);
  const bw = Math.max(2, step * 0.8);
  points.forEach((p, i) => {
    if (p.vib === null) return;
    ctx.fillStyle = p.detected ? '#f87171' : '#4ade80';
    const y = yOf(p.vib);
    ctx.fillRect(pad + i * step, y, bw, H - 8 - y);
  });
}

function clearAll() {
  base = null; vibWin = [];
  points = []; allSamples = []; total = 0; recent = [];
  els.log.textContent = '';
  ['vx', 'vy', 'vz', 'vm'].forEach((k) => (els[k].textContent = '—'));
  els.count.textContent = '0';
  els.rate.textContent = '0';
  els.csv.disabled = true;
  els.vibState.textContent = '—'; els.vibRms.textContent = 'RMS —'; els.vibCard.classList.remove('on');
  draw();
  drawVib();
}

function downloadCsv() {
  const rows = ['timestamp_iso,x_g,y_g,z_g,vib_rms,vibration',
    ...allSamples.map((s) => `${new Date(s.t).toISOString()},${s.x},${s.y},${s.z},${s.vib ?? ''},${s.vib === null ? '' : s.detected ? 'DETECTED' : 'NO'}`)];
  const url = URL.createObjectURL(new Blob([rows.join('\n')], { type: 'text/csv' }));
  const a = Object.assign(document.createElement('a'), {
    href: url, download: `sleepqure_${Date.now()}.csv`,
  });
  a.click();
  URL.revokeObjectURL(url);
}

els.connect.onclick = connect;
els.disconnect.onclick = disconnect;
els.pause.onclick = () => {
  paused = !paused;
  els.pause.textContent = paused ? 'Resume' : 'Pause';
};
els.clear.onclick = clearAll;
els.csv.onclick = downloadCsv;

draw();
drawVib();
