#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ================= WIFI =================
const char* ssid = "OMOTEC";
const char* password = "Omotech@23";

// ================= WEB SERVER =================
WebServer server(80);

// ================= FSR PINS =================
#define FSR1_PIN A0
#define FSR2_PIN A2

// ================= MPU6050 =================
Adafruit_MPU6050 mpu;

// ================= WINDOW SIZE FOR STATS =================
#define WINDOW_SIZE 50

// ================= DATA LOGGER =================
#define MAX_RECORDS 1000

struct DataRecord {
  unsigned long timestamp;
  int   fsr1, fsr2;
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float temperatureC;
  float fsrMean, fsrStdDev;
  int   fsrPeaks;
  float accMag, motionVar, gyroMag, jerk, corr;
};

DataRecord logBuffer[MAX_RECORDS];
int  logCount    = 0;
bool isRecording = false;

// ================= RAW SENSOR VALUES =================
int fsr1Value = 0;
int fsr2Value = 0;

float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float temperatureC = 0;

// ================= DERIVED: FSR =================
float fsrMean   = 0;
float fsrStdDev = 0;
int   fsrPeaks  = 0;

// ================= DERIVED: IMU =================
float accMagnitude     = 0;
float motionVariability = 0;
float gyroMagnitude    = 0;
float jerk             = 0;

// ================= DERIVED: CORRELATION =================
float pressureMotionCorr = 0;

// ================= ROLLING WINDOWS =================
float fsrWindow[WINDOW_SIZE];
float accMagWindow[WINDOW_SIZE];
int   windowIndex = 0;
bool  windowFull  = false;

// ================= JERK HELPER =================
float prevAccMag = 0;
unsigned long prevJerkTime = 0;

// ================= HELPERS =================
float computeMean(float* arr, int n) {
  float sum = 0;
  for (int i = 0; i < n; i++) sum += arr[i];
  return sum / n;
}

float computeStdDev(float* arr, int n, float mean) {
  float sumSq = 0;
  for (int i = 0; i < n; i++) {
    float diff = arr[i] - mean;
    sumSq += diff * diff;
  }
  return sqrt(sumSq / n);
}

int countPeaks(float* arr, int n) {
  int peaks = 0;
  for (int i = 1; i < n - 1; i++) {
    if (arr[i] > arr[i - 1] && arr[i] > arr[i + 1]) peaks++;
  }
  return peaks;
}

float computeCorrelation(float* x, float* y, int n) {
  float mx = computeMean(x, n);
  float my = computeMean(y, n);
  float num = 0, dx2 = 0, dy2 = 0;
  for (int i = 0; i < n; i++) {
    float dx = x[i] - mx;
    float dy = y[i] - my;
    num += dx * dy;
    dx2 += dx * dx;
    dy2 += dy * dy;
  }
  float denom = sqrt(dx2 * dy2);
  if (denom == 0) return 0;
  return num / denom;
}

// ================= READ & COMPUTE SENSORS =================
void readSensors() {
  fsr1Value = analogRead(FSR1_PIN);
  fsr2Value = analogRead(FSR2_PIN);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
  temperatureC = temp.temperature;

  float fsrCombined = (fsr1Value + fsr2Value) / 2.0f;
  accMagnitude  = sqrt(accX*accX + accY*accY + accZ*accZ);
  gyroMagnitude = sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ);

  unsigned long now = millis();
  if (prevJerkTime > 0) {
    float dt = (now - prevJerkTime) / 1000.0f;
    if (dt > 0) jerk = fabs(accMagnitude - prevAccMag) / dt;
  }
  prevAccMag   = accMagnitude;
  prevJerkTime = now;

  fsrWindow[windowIndex]    = fsrCombined;
  accMagWindow[windowIndex] = accMagnitude;
  windowIndex++;
  if (windowIndex >= WINDOW_SIZE) { windowIndex = 0; windowFull = true; }

  int n = windowFull ? WINDOW_SIZE : windowIndex;
  if (n < 2) return;

  fsrMean   = computeMean(fsrWindow, n);
  fsrStdDev = computeStdDev(fsrWindow, n, fsrMean);
  fsrPeaks  = countPeaks(fsrWindow, n);

  float accMagMean  = computeMean(accMagWindow, n);
  motionVariability = computeStdDev(accMagWindow, n, accMagMean);
  pressureMotionCorr = computeCorrelation(fsrWindow, accMagWindow, n);
}

// ================= HTML PAGE =================
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>NeuroWrite Dashboard</title>
  <style>
    :root {
      --bg: #f5f7fb;
      --panel: #ffffff;
      --panel-soft: #f8fafc;
      --text: #0f172a;
      --muted: #64748b;
      --line: #dbe3ef;
      --blue: #2563eb;
      --cyan: #0891b2;
      --green: #16a34a;
      --orange: #ea580c;
      --red: #dc2626;
      --purple: #7c3aed;
      --shadow: 0 12px 30px rgba(15, 23, 42, 0.08);
    }

    *, *::before, *::after { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Arial, Helvetica, sans-serif;
      background:
        radial-gradient(circle at top left, rgba(37,99,235,0.12), transparent 34%),
        linear-gradient(180deg, #ffffff 0%, var(--bg) 100%);
      color: var(--text);
      min-height: 100vh;
    }

    .header {
      text-align: center;
      padding: 30px 20px 12px;
    }
    .header h1 {
      margin: 0;
      font-size: clamp(1.8rem, 4vw, 2.6rem);
      letter-spacing: 0.5px;
      color: #10213f;
    }
    .header .subtitle {
      color: var(--muted);
      font-size: 0.98rem;
      margin-top: 8px;
    }
    .live-dot {
      display: inline-block;
      width: 9px;
      height: 9px;
      background: var(--green);
      border-radius: 50%;
      margin-right: 6px;
      animation: pulse 1.4s infinite;
      box-shadow: 0 0 0 5px rgba(22,163,74,0.12);
    }
    @keyframes pulse { 0%,100% { opacity:1; } 50% { opacity:0.35; } }

    .zone {
      max-width: 1220px;
      margin: 0 auto;
      padding: 0 20px 30px;
    }
    .zone-label {
      display: flex;
      align-items: center;
      gap: 12px;
      margin: 28px 0 16px;
    }
    .zone-label span {
      font-size: 0.72rem;
      font-weight: 800;
      letter-spacing: 2px;
      text-transform: uppercase;
      padding: 6px 13px;
      border-radius: 999px;
      background: #eef4ff;
      color: var(--blue);
      border: 1px solid #cfe0ff;
    }
    .zone-line { flex: 1; height: 1px; background: var(--line); }
    .sectionTitle {
      margin: 22px 0 12px 2px;
      font-size: 0.82rem;
      font-weight: 800;
      letter-spacing: 1.5px;
      text-transform: uppercase;
      color: #475569;
    }

    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
      gap: 14px;
    }
    .card-live, .chart-card {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 18px;
      box-shadow: var(--shadow);
    }
    .card-live {
      padding: 20px 18px;
      transition: transform 0.15s ease, border-color 0.15s ease;
    }
    .card-live:hover { transform: translateY(-2px); border-color: #bcd2ff; }
    .card-live h2 {
      margin: 0 0 10px;
      font-size: 0.78rem;
      font-weight: 800;
      letter-spacing: 1px;
      text-transform: uppercase;
      color: var(--blue);
    }
    .card-live .val {
      font-size: 2.1rem;
      font-weight: 800;
      color: var(--text);
      line-height: 1;
    }
    .card-live .unit {
      font-size: 0.78rem;
      color: var(--muted);
      margin-top: 7px;
    }

    .chart-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
      gap: 16px;
    }
    .chart-card {
      padding: 16px;
      overflow: hidden;
    }
    .chart-head {
      display: flex;
      justify-content: space-between;
      align-items: flex-start;
      gap: 12px;
      flex-wrap: wrap;
      margin-bottom: 8px;
    }
    .chart-title {
      margin: 0;
      font-size: 1rem;
      color: #10213f;
    }
    .axis-note {
      margin-top: 8px;
      font-size: 0.78rem;
      color: var(--muted);
    }
    .legend {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      font-size: 0.78rem;
      color: #334155;
    }
    .legend-item {
      display: inline-flex;
      align-items: center;
      gap: 5px;
      white-space: nowrap;
    }
    .legend-swatch {
      width: 16px;
      height: 3px;
      border-radius: 999px;
      display: inline-block;
    }
    canvas {
      width: 100%;
      height: 310px;
      display: block;
      background: #fbfdff;
      border: 1px solid #e2e8f0;
      border-radius: 14px;
    }

    .logger-bar {
      background: rgba(255,255,255,0.92);
      border-top: 1px solid var(--line);
      border-bottom: 1px solid var(--line);
      padding: 16px 20px;
      box-shadow: 0 8px 20px rgba(15,23,42,0.04);
    }
    .logger-inner {
      max-width: 1220px;
      margin: 0 auto;
      display: flex;
      align-items: center;
      gap: 20px;
      flex-wrap: wrap;
    }
    .logger-left {
      display: flex;
      align-items: center;
      gap: 12px;
      min-width: 220px;
    }
    .rec-dot { width: 13px; height: 13px; border-radius: 50%; flex-shrink: 0; }
    .rec-dot.idle { background: #94a3b8; }
    .rec-dot.recording { background: var(--red); animation: pulse 0.8s infinite; }
    .rec-dot.full { background: #f59e0b; }
    .logger-text { display: flex; flex-direction: column; gap: 3px; }
    .rec-label { font-size: 0.86rem; font-weight: 800; color: var(--text); }
    .rec-count { font-size: 0.76rem; color: var(--muted); }
    .logger-progress-wrap { flex: 1; min-width: 160px; }
    .logger-progress-bg { background: #e5eaf3; border-radius: 8px; height: 9px; overflow: hidden; }
    .logger-progress-fill {
      height: 100%;
      border-radius: 8px;
      background: linear-gradient(90deg, #22c55e, #16a34a);
      transition: width 0.4s ease;
    }
    .logger-btns { display: flex; gap: 10px; flex-wrap: wrap; }
    .logger-btns button {
      padding: 9px 16px;
      border: none;
      border-radius: 10px;
      font-size: 0.82rem;
      font-weight: 800;
      cursor: pointer;
      transition: opacity 0.15s, transform 0.1s;
    }
    .logger-btns button:hover { opacity: 0.88; }
    .logger-btns button:active { transform: scale(0.97); }
    .btn-rec { background: var(--green); color: #fff; }
    .btn-rec.active { background: var(--red); }
    .btn-down { background: var(--blue); color: #fff; }
    .btn-clr {
      background: #fee2e2;
      color: #b91c1c;
      border: 1px solid #fecaca !important;
    }

    .statusbar {
      text-align: center;
      padding: 14px;
      font-size: 0.85rem;
      color: var(--muted);
    }
    #status { color: var(--cyan); font-weight: 700; }

    @media(max-width:700px) {
      .card-live .val { font-size: 1.65rem; }
      .logger-inner { flex-direction: column; align-items: flex-start; }
      .chart-grid { grid-template-columns: 1fr; }
      canvas { height: 270px; }
    }
  </style>
</head>
<body>

<div class="header">
  <h1>NeuroWrite Dashboard</h1>
<!-- <div class="subtitle">
  //   <span class="live-dot"></span>XIAO ESP32-C3 &nbsp;|&nbsp; Live neuromotor monitoring
  </div>-->
</div>

<!-- ZONE 1: LIVE RAW READINGS -->
<div class="zone">
  <div class="zone-label">
    <span>Live Readings</span>
    <div class="zone-line"></div>
  </div>

  <div class="sectionTitle">FSR Grip Force</div>
  <div class="grid">
    <div class="card-live"><h2>Grip - Thumb Force</h2><div class="val" id="fsr1">--</div><div class="unit">ADC raw / force proxy</div></div>
    <div class="card-live"><h2>Grip - Index Finger Force</h2><div class="val" id="fsr2">--</div><div class="unit">ADC raw / force proxy</div></div>
  </div>

  <div class="sectionTitle">MPU6050 - Acceleration</div>
  <div class="grid">
    <div class="card-live"><h2>Accel X</h2><div class="val" id="accX">--</div><div class="unit">m/s2</div></div>
    <div class="card-live"><h2>Accel Y</h2><div class="val" id="accY">--</div><div class="unit">m/s2</div></div>
    <div class="card-live"><h2>Accel Z</h2><div class="val" id="accZ">--</div><div class="unit">m/s2</div></div>
  </div>

  <div class="sectionTitle">Live Graphs</div>
  <div class="chart-grid">
    <div class="chart-card">
      <div class="chart-head">
        <h2 class="chart-title">Acceleration vs Time</h2>
        <div class="legend">
          <span class="legend-item"><span class="legend-swatch" style="background:#2563eb"></span>Ax</span>
          <span class="legend-item"><span class="legend-swatch" style="background:#16a34a"></span>Ay</span>
          <span class="legend-item"><span class="legend-swatch" style="background:#dc2626"></span>Az</span>
        </div>
      </div>
      <canvas id="accChart"></canvas>
      <div class="axis-note">Y-axis: Acceleration (m/s2) &nbsp; | &nbsp; X-axis: Time (s)</div>
    </div>

    <div class="chart-card">
      <div class="chart-head">
        <h2 class="chart-title">FSR Force Value vs Time</h2>
        <div class="legend">
          <span class="legend-item"><span class="legend-swatch" style="background:#ea580c"></span>Thumb Grip</span>
          <span class="legend-item"><span class="legend-swatch" style="background:#0891b2"></span>Index Finger Grip</span>
        </div>
      </div>
      <canvas id="fsrChart"></canvas>
      <div class="axis-note">Y-axis: FSR force value (ADC raw) &nbsp; | &nbsp; X-axis: Time (s)</div>
    </div>
  </div>

  <div class="sectionTitle">MPU6050 - Gyroscope</div>
  <div class="grid">
    <div class="card-live"><h2>Gyro X</h2><div class="val" id="gyroX">--</div><div class="unit">rad/s</div></div>
    <div class="card-live"><h2>Gyro Y</h2><div class="val" id="gyroY">--</div><div class="unit">rad/s</div></div>
    <div class="card-live"><h2>Gyro Z</h2><div class="val" id="gyroZ">--</div><div class="unit">rad/s</div></div>
  </div>

  <div class="sectionTitle">MPU6050 - Temperature</div>
  <div class="grid">
    <div class="card-live" style="max-width:260px;">
      <h2>Temperature</h2><div class="val" id="temp">--</div><div class="unit">deg C</div>
    </div>
  </div>
</div>

<!-- DATA LOGGER BAR -->
<div class="logger-bar">
  <div class="logger-inner">
    <div class="logger-left">
      <span id="recDot" class="rec-dot idle"></span>
      <div class="logger-text">
        <span id="recLabel" class="rec-label">Not Recording</span>
        <span id="recCount" class="rec-count">0 / 1000 samples stored</span>
      </div>
    </div>
    <div class="logger-progress-wrap">
      <div class="logger-progress-bg">
        <div class="logger-progress-fill" id="recBar" style="width:0%"></div>
      </div>
    </div>
    <div class="logger-btns">
      <button class="btn-rec" id="btnRecord" onclick="toggleRecord()">&#9679;&nbsp;Start Recording</button>
      <button class="btn-down" onclick="downloadCSV()">&#8595;&nbsp;Download CSV</button>
      <button class="btn-clr" onclick="clearHistory()">&#128465;&nbsp;Clear History</button>
    </div>
  </div>
</div>

<div class="statusbar">
  <span id="status">Connecting...</span>
  &nbsp;|&nbsp; refreshes every 500 ms &nbsp;|&nbsp; graph window = last 120 samples
</div>

<script>
  let recording = false;
  const MAX_GRAPH_POINTS = 120;
  let graphStartTime = null;
  let graphHistory = [];

  function safeNumber(value, fallback) {
    const n = Number(value);
    return Number.isFinite(n) ? n : fallback;
  }

  function addGraphSample(d) {
    const nowMs = Date.now();
    if (graphStartTime === null) graphStartTime = nowMs;
    const t = (nowMs - graphStartTime) / 1000.0;

    graphHistory.push({
      t: t,
      ax: safeNumber(d.accX, 0),
      ay: safeNumber(d.accY, 0),
      az: safeNumber(d.accZ, 0),
      fsr1: safeNumber(d.fsr1, 0),
      fsr2: safeNumber(d.fsr2, 0)
    });

    while (graphHistory.length > MAX_GRAPH_POINTS) graphHistory.shift();
  }

  function niceLabel(value) {
    if (Math.abs(value) >= 100) return value.toFixed(0);
    if (Math.abs(value) >= 10) return value.toFixed(1);
    return value.toFixed(2);
  }

  function drawChart(canvasId, series, yLabel, xLabel, options) {
    const canvas = document.getElementById(canvasId);
    const ctx = canvas.getContext('2d');
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;

    if (canvas.width !== Math.round(rect.width * dpr) || canvas.height !== Math.round(rect.height * dpr)) {
      canvas.width = Math.round(rect.width * dpr);
      canvas.height = Math.round(rect.height * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    const w = rect.width;
    const h = rect.height;
    const margin = { left: 64, right: 20, top: 22, bottom: 54 };
    const plotW = w - margin.left - margin.right;
    const plotH = h - margin.top - margin.bottom;

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#fbfdff';
    ctx.fillRect(0, 0, w, h);

    if (graphHistory.length < 2) {
      ctx.fillStyle = '#64748b';
      ctx.font = '14px Arial';
      ctx.textAlign = 'center';
      ctx.fillText('Waiting for live sensor data...', w / 2, h / 2);
      return;
    }

    let xMin = graphHistory[0].t;
    let xMax = graphHistory[graphHistory.length - 1].t;
    if (xMax - xMin < 1) xMax = xMin + 1;

    let values = [];
    for (let i = 0; i < graphHistory.length; i++) {
      for (let s = 0; s < series.length; s++) {
        const val = series[s].get(graphHistory[i]);
        if (Number.isFinite(val)) values.push(val);
      }
    }

    let yMin = options && options.fixedMin !== undefined ? options.fixedMin : Math.min.apply(null, values.concat([0]));
    let yMax = options && options.fixedMax !== undefined ? options.fixedMax : Math.max.apply(null, values.concat([0]));
    if (yMin === yMax) { yMin -= 1; yMax += 1; }
    if (!(options && options.fixedMin !== undefined && options.fixedMax !== undefined)) {
      const pad = Math.max((yMax - yMin) * 0.12, 0.5);
      yMin -= pad;
      yMax += pad;
    }

    const xToPx = x => margin.left + ((x - xMin) / (xMax - xMin)) * plotW;
    const yToPx = y => margin.top + (1 - ((y - yMin) / (yMax - yMin))) * plotH;

    ctx.strokeStyle = '#cbd5e1';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(margin.left, margin.top);
    ctx.lineTo(margin.left, margin.top + plotH);
    ctx.lineTo(margin.left + plotW, margin.top + plotH);
    ctx.stroke();

    ctx.font = '11px Arial';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let i = 0; i <= 5; i++) {
      const y = yMin + ((yMax - yMin) * i / 5);
      const py = yToPx(y);
      ctx.strokeStyle = '#e2e8f0';
      ctx.beginPath();
      ctx.moveTo(margin.left, py);
      ctx.lineTo(margin.left + plotW, py);
      ctx.stroke();
      ctx.fillStyle = '#64748b';
      ctx.fillText(niceLabel(y), margin.left - 8, py);
    }

    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let i = 0; i <= 5; i++) {
      const x = xMin + ((xMax - xMin) * i / 5);
      const px = xToPx(x);
      ctx.strokeStyle = '#eef2f7';
      ctx.beginPath();
      ctx.moveTo(px, margin.top);
      ctx.lineTo(px, margin.top + plotH);
      ctx.stroke();
      ctx.fillStyle = '#64748b';
      ctx.fillText(x.toFixed(1), px, margin.top + plotH + 9);
    }

    ctx.save();
    ctx.translate(15, margin.top + plotH / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillStyle = '#334155';
    ctx.font = '12px Arial';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(yLabel, 0, 0);
    ctx.restore();

    ctx.fillStyle = '#334155';
    ctx.font = '12px Arial';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    ctx.fillText(xLabel, margin.left + plotW / 2, h - 8);

    for (let s = 0; s < series.length; s++) {
      ctx.strokeStyle = series[s].color;
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      let started = false;
      for (let i = 0; i < graphHistory.length; i++) {
        const p = graphHistory[i];
        const val = series[s].get(p);
        if (!Number.isFinite(val)) continue;
        const px = xToPx(p.t);
        const py = yToPx(val);
        if (!started) {
          ctx.moveTo(px, py);
          started = true;
        } else {
          ctx.lineTo(px, py);
        }
      }
      ctx.stroke();
    }
  }

  function drawAllCharts() {
    drawChart('accChart', [
      { name: 'Ax', color: '#2563eb', get: p => p.ax },
      { name: 'Ay', color: '#16a34a', get: p => p.ay },
      { name: 'Az', color: '#dc2626', get: p => p.az }
    ], 'Acceleration (m/s2)', 'Time (s)', {});

    drawChart('fsrChart', [
      { name: 'FSR1', color: '#ea580c', get: p => p.fsr1 },
      { name: 'FSR2', color: '#0891b2', get: p => p.fsr2 }
    ], 'FSR force value (ADC raw)', 'Time (s)', { fixedMin: 0, fixedMax: 4095 });
  }

  function updateData() {
    fetch('/data')
      .then(r => r.json())
      .then(d => {
        document.getElementById('fsr1').innerText  = d.fsr1;
        document.getElementById('fsr2').innerText  = d.fsr2;
        document.getElementById('accX').innerText  = d.accX.toFixed(2);
        document.getElementById('accY').innerText  = d.accY.toFixed(2);
        document.getElementById('accZ').innerText  = d.accZ.toFixed(2);
        document.getElementById('gyroX').innerText = d.gyroX.toFixed(2);
        document.getElementById('gyroY').innerText = d.gyroY.toFixed(2);
        document.getElementById('gyroZ').innerText = d.gyroZ.toFixed(2);
        document.getElementById('temp').innerText  = d.temp.toFixed(2);

        recording = d.recording;
        addGraphSample(d);
        drawAllCharts();
        updateLoggerUI(d.recording, d.recordCount, d.maxRecords);
        document.getElementById('status').innerText = 'Live update running';
      })
      .catch(() => { document.getElementById('status').innerText = 'Connection error - retrying...'; });
  }

  function updateLoggerUI(rec, count, max) {
    const dot   = document.getElementById('recDot');
    const label = document.getElementById('recLabel');
    const cntEl = document.getElementById('recCount');
    const bar   = document.getElementById('recBar');
    const btn   = document.getElementById('btnRecord');
    const pct   = Math.min((count / max) * 100, 100).toFixed(1);

    cntEl.innerText = count + ' / ' + max + ' samples stored';
    bar.style.width = pct + '%';

    if (count >= max) {
      dot.className = 'rec-dot full';
      label.innerText = 'Buffer Full';
      btn.className = 'btn-rec';
      btn.innerHTML = '&#9679;&nbsp;Start Recording';
      bar.style.background = 'linear-gradient(90deg,#f59e0b,#d97706)';
    } else if (rec) {
      dot.className = 'rec-dot recording';
      label.innerText = 'Recording...';
      btn.className = 'btn-rec active';
      btn.innerHTML = '&#9632;&nbsp;Stop Recording';
      bar.style.background = 'linear-gradient(90deg,#22c55e,#16a34a)';
    } else {
      dot.className = 'rec-dot idle';
      label.innerText = count > 0 ? 'Paused - ' + count + ' samples' : 'Not Recording';
      btn.className = 'btn-rec';
      btn.innerHTML = '&#9679;&nbsp;Start Recording';
      bar.style.background = 'linear-gradient(90deg,#22c55e,#16a34a)';
    }
  }

  function toggleRecord() {
    const action = recording ? 'stop' : 'start';
    fetch('/record?action=' + action)
      .then(r => r.json())
      .then(d => { recording = d.recording; updateLoggerUI(d.recording, d.recordCount, d.maxRecords); })
      .catch(() => alert('Could not reach ESP32'));
  }

  function downloadCSV() { window.location.href = '/download'; }

  function clearHistory() {
    if (!confirm('Clear all stored samples?')) return;
    fetch('/clear')
      .then(r => r.json())
      .then(() => {
        recording = false;
        graphStartTime = null;
        graphHistory = [];
        drawAllCharts();
        updateLoggerUI(false, 0, 1000);
      })
      .catch(() => alert('Could not reach ESP32'));
  }

  window.addEventListener('resize', drawAllCharts);
  setInterval(updateData, 500);
  updateData();
</script>
</body>
</html>
)rawliteral";

// ================= ROOT PAGE =================
void handleRoot() { server.send(200, "text/html", webpage); }

// ================= JSON DATA ENDPOINT =================
void handleData() {
  readSensors();

  if (isRecording && logCount < MAX_RECORDS) {
    DataRecord& r  = logBuffer[logCount++];
    r.timestamp    = millis();
    r.fsr1         = fsr1Value;  r.fsr2 = fsr2Value;
    r.accX         = accX;  r.accY = accY;  r.accZ = accZ;
    r.gyroX        = gyroX; r.gyroY = gyroY; r.gyroZ = gyroZ;
    r.temperatureC = temperatureC;
    r.fsrMean      = fsrMean;  r.fsrStdDev = fsrStdDev;  r.fsrPeaks = fsrPeaks;
    r.accMag       = accMagnitude;  r.motionVar = motionVariability;
    r.gyroMag      = gyroMagnitude; r.jerk = jerk; r.corr = pressureMotionCorr;
    if (logCount >= MAX_RECORDS) isRecording = false;
  }

  String json = "{";
  json += "\"fsr1\":"       + String(fsr1Value)           + ",";
  json += "\"fsr2\":"       + String(fsr2Value)           + ",";
  json += "\"accX\":"       + String(accX,  2)            + ",";
  json += "\"accY\":"       + String(accY,  2)            + ",";
  json += "\"accZ\":"       + String(accZ,  2)            + ",";
  json += "\"gyroX\":"      + String(gyroX, 2)            + ",";
  json += "\"gyroY\":"      + String(gyroY, 2)            + ",";
  json += "\"gyroZ\":"      + String(gyroZ, 2)            + ",";
  json += "\"temp\":"       + String(temperatureC, 2)     + ",";
  json += "\"recording\":"  + String(isRecording ? "true" : "false") + ",";
  json += "\"recordCount\":" + String(logCount)           + ",";
  json += "\"maxRecords\":"  + String(MAX_RECORDS);
  json += "}";

  server.send(200, "application/json", json);
}

// ================= TOGGLE RECORDING =================
void handleRecord() {
  if (server.hasArg("action")) {
    String a = server.arg("action");
    if (a == "start") isRecording = true;
    if (a == "stop")  isRecording = false;
  }
  String json = "{\"recording\":" + String(isRecording ? "true" : "false") +
                ",\"recordCount\":" + String(logCount) +
                ",\"maxRecords\":"  + String(MAX_RECORDS) + "}";
  server.send(200, "application/json", json);
}

// ================= DOWNLOAD CSV =================
void handleDownload() {
  String csv =
    "Timestamp_ms,FSR1,FSR2,"
    "AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Temp_C,"
    "FSR_Mean,FSR_StdDev,FSR_Peaks,"
    "AccMag,MotionVar,GyroMag,Jerk,Corr\r\n";

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Disposition", "attachment; filename=\"neurowrite_data.csv\"");
  server.sendHeader("Content-Type", "text/csv");
  server.send(200);
  server.sendContent(csv);

  for (int i = 0; i < logCount; i++) {
    DataRecord& r = logBuffer[i];
    String row = "";
    row += String(r.timestamp) + ",";
    row += String(r.fsr1) + "," + String(r.fsr2) + ",";
    row += String(r.accX,3)  + "," + String(r.accY,3)  + "," + String(r.accZ,3)  + ",";
    row += String(r.gyroX,3) + "," + String(r.gyroY,3) + "," + String(r.gyroZ,3) + ",";
    row += String(r.temperatureC,2) + ",";
    row += String(r.fsrMean,2) + "," + String(r.fsrStdDev,3) + "," + String(r.fsrPeaks) + ",";
    row += String(r.accMag,2)  + "," + String(r.motionVar,3) + ",";
    row += String(r.gyroMag,3) + "," + String(r.jerk,2) + "," + String(r.corr,3);
    row += "\r\n";
    server.sendContent(row);
  }
  server.sendContent("");
}

// ================= CLEAR HISTORY =================
void handleClear() {
  logCount    = 0;
  isRecording = false;
  server.send(200, "application/json", "{\"status\":\"cleared\",\"recordCount\":0}");
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting NeuroWrite XIAO ESP32-C3 dashboard...");

  memset(fsrWindow,    0, sizeof(fsrWindow));
  memset(accMagWindow, 0, sizeof(accMagWindow));

  Wire.begin(6, 7);  // SDA=GPIO6, SCL=GPIO7

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Check wiring.");
    while (1) { delay(100); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 connected.");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/",         handleRoot);
  server.on("/data",     handleData);
  server.on("/record",   handleRecord);
  server.on("/download", handleDownload);
  server.on("/clear",    handleClear);
  server.begin();
  Serial.println("Web server started");
}

// ================= LOOP =================
void loop() {
  server.handleClient();
}
