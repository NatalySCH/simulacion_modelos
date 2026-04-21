// ═══════════════════════════════════════════
//  MODELO MATEMÁTICO — RK4
// ═══════════════════════════════════════════
let m = 2, b = 2, k = 5;
let px = 0, pv = 0;
const ELASTIC_LIMIT = 2.0;
let simTime = 0;
let last = null;

// ── Límites físicos seguros ──
const MAX_ACC = 50;   // m/s²  — evita explosión numérica
const MAX_VEL = 10;   // m/s   — amortiguador real no supera esto

function clampParams() {
  m = Math.max(m, 0.5);   // evita división por masa ~0
  b = Math.max(b, 0.1);   // amortiguación mínima
  k = Math.min(k, 50);    // rigidez máxima estable
}

function rk4Step(px, pv, F_ext, dt) {
  const f = (x, v) => {
    let acc = (F_ext - b * v - k * x) / m;
    acc = Math.max(-MAX_ACC, Math.min(MAX_ACC, acc)); // clamp aceleración
    return { dx: v, dv: acc };
  };
  const k1 = f(px, pv);
  const k2 = f(px + 0.5*dt*k1.dx, pv + 0.5*dt*k1.dv);
  const k3 = f(px + 0.5*dt*k2.dx, pv + 0.5*dt*k2.dv);
  const k4 = f(px + dt*k3.dx, pv + dt*k3.dv);
  return {
    nx: px + (dt/6)*(k1.dx + 2*k2.dx + 2*k3.dx + k4.dx),
    nv: pv + (dt/6)*(k1.dv + 2*k2.dv + 2*k3.dv + k4.dv)
  };
}

// ── Substepping + clamp velocidad ──
function physicsStep(dt, carX) {
  const steps = 6;
  const subDt = dt / steps;
  for (let i = 0; i < steps; i++) {
    const F_ext = getForce(carX);
    const { nx, nv } = rk4Step(px, pv, F_ext, subDt);
    px = nx;
    pv = Math.max(-MAX_VEL, Math.min(MAX_VEL, nv)); // clamp velocidad
  }
}

const MODES = {
  sub:   { m:2, b:2,    k:5 },
  crit:  { m:2, b:6.32, k:5 },
  sobre: { m:2, b:20,   k:5 },
};
let currentMode = 'sub';

function setMode(md) {
  currentMode = md;
  const cfg = MODES[md];
  m = cfg.m; b = cfg.b; k = cfg.k;
  document.getElementById('sm').value = m;
  document.getElementById('sb').value = b;
  document.getElementById('sk').value = k;
  document.getElementById('lm').textContent = m.toFixed(1);
  document.getElementById('lb').textContent = b.toFixed(1);
  document.getElementById('lk').textContent = k.toFixed(1);
  resetPhysics();
  updateModeBtns(md);
}

function updateModeBtns(md) {
  ['sub','crit','sobre'].forEach(id => {
    const btn = document.getElementById('btn-'+id);
    btn.className = 'mode-btn' + (id===md ? ' active-'+id : '');
  });
}

// ═══════════════════════════════════════════
//  ALERTAS
// ═══════════════════════════════════════════
let alertThreshold = 0.3;
let outOfBoundsSince = null;
let alertFired = false;
let peakLog = [];
let lastPeakSign = 0;
let lastPeakVal = 0;
let lastPeakTime = -999;
let lastAboveThreshold = 0;
let dismissedAlertLevel = null;

function recordPeak(t, x) {
  const sign = Math.sign(x);
  if (Math.abs(x) > alertThreshold) {
    if (sign !== lastPeakSign || (t - lastPeakTime) > 0.5) {
      const absX = Math.abs(x);
      if (absX > Math.abs(lastPeakVal) || (t - lastPeakTime) > 0.5) {
        peakLog.push({
          tiempo_s: parseFloat(t.toFixed(3)),
          posicion_m: parseFloat(x.toFixed(4)),
          velocidad_mps: parseFloat(pv.toFixed(4)),
          m_kg: m, b_Nsm: b, k_Nm: k,
          supera_umbral: Math.abs(x) > alertThreshold
        });
        lastPeakSign = sign;
        lastPeakVal = x;
        lastPeakTime = t;
      }
    }
  }
}

function checkAlerts(t) {
  const absX = Math.abs(px);
  recordPeak(t, px);
  if (absX > alertThreshold) {
    if (!outOfBoundsSince) outOfBoundsSince = t;
    lastAboveThreshold = t;
  }
  if (outOfBoundsSince && !alertFired) {
    const elapsed = t - outOfBoundsSince;
    const timeSinceLast = t - lastAboveThreshold;
    if (timeSinceLast > 1.5) {
      outOfBoundsSince = null;
      dismissedAlertLevel = null;
      setAlertOk();
    } else {
      if (elapsed >= 3.0) { alertFired = true; setAlertError(); }
      else setAlertWarn(elapsed);
    }
  }
}

function dismissAlert() {
  const notif = document.getElementById('alertNotification');
  if (notif) notif.classList.remove('show');
  dismissedAlertLevel = alertFired ? 'error' : 'warn';
}

function resetAlerts() {
  outOfBoundsSince = null; alertFired = false;
  dismissedAlertLevel = null; lastAboveThreshold = 0;
  setAlertOk();
}

function resetPhysics() {
  px=0; pv=0; graphData=[]; peakLog=[];
  resetAlerts();
}

function setAlertOk() {
  document.getElementById('alertBox').className = 'alert-box ok';
  document.getElementById('alertText').innerHTML =
    '● SISTEMA NOMINAL<br>Oscilaciones dentro del rango.<br>Tiempo fuera de umbral: 0.0 s';
  const n = document.getElementById('alertNotification');
  if (n) n.className = 'alert-notification';
}
function setAlertWarn(elapsed) {
  document.getElementById('alertBox').className = 'alert-box warn';
  document.getElementById('alertText').innerHTML =
    '⚠ ALERTA ACTIVA<br>Oscilación sostenida: ' + elapsed.toFixed(1) + ' s';
  const n = document.getElementById('alertNotification');
  if (n && dismissedAlertLevel !== 'warn' && dismissedAlertLevel !== 'error') {
    n.className = 'alert-notification warn show';
    n.innerHTML = '⚠ ADVERTENCIA<br>Oscilación sostenida: ' + elapsed.toFixed(1) + ' s<br>Umbral: ±' + alertThreshold.toFixed(2) + ' m' +
      '<button class="alert-close" onclick="dismissAlert()">×</button>';
  }
}
function setAlertError() {
  document.getElementById('alertBox').className = 'alert-box error';
  document.getElementById('alertText').innerHTML =
    '✖ ERROR CRÍTICO<br>Amortiguación fallida.<br>Requiere mantenimiento.';
  const n = document.getElementById('alertNotification');
  if (n && dismissedAlertLevel !== 'error') {
    n.className = 'alert-notification error show';
    n.innerHTML = '✖ ERROR CRÍTICO<br>Amortiguador desgastado.<br>t_estab &gt; 3.0 s' +
      '<button class="alert-close" onclick="dismissAlert()">×</button>';
  }
}

// ── Exportar ──
function exportJSON() {
  const wn = Math.sqrt(k/m);
  const zeta = b/(2*wn*m);
  let estado = zeta < 0.95 ? 'SUBAMORTIGUADO' : zeta < 1.05 ? 'CRÍTICO' : 'SOBREAMORTIGUADO';
  const payload = {
    metadatos: {
      fecha_exportacion: new Date().toISOString(),
      parametros: { m_kg: m, b_Nsm: b, k_Nm: k },
      diagnostico: { wn_rads: parseFloat(wn.toFixed(4)), zeta: parseFloat(zeta.toFixed(4)), estado, alerta_disparada: alertFired, umbral_m: alertThreshold }
    },
    picos_vibracion: peakLog
  };
  const blob = new Blob([JSON.stringify(payload, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = 'suspension_picos.json'; a.click();
  URL.revokeObjectURL(url);
  const t = document.getElementById('toast');
  t.textContent = '✓ Exportado: ' + peakLog.length + ' picos';
  t.classList.add('show');
  setTimeout(() => t.classList.remove('show'), 4000);
}

// ═══════════════════════════════════════════
//  BACHES
// ═══════════════════════════════════════════
let bumps = [];
let bumpTimer = 0;
let bumpInterval = 2.8;
const ROAD_SPEED = 95;
let autoBumpsEnabled = true;

function toggleAutoBumps() {
  autoBumpsEnabled = !autoBumpsEnabled;
  const btn = document.getElementById('toggleAutoBumps');
  btn.classList.toggle('off', !autoBumpsEnabled);
  const txt = btn.querySelector('svg').nextSibling;
  // update text node
  btn.childNodes[btn.childNodes.length-1].textContent = autoBumpsEnabled ? ' Automático' : ' Manual';
  if (autoBumpsEnabled) bumpTimer = 0;
}

function spawnBump(forced=false) {
  const isUp = Math.random() > 0.5;
  const depth = (forced ? 0.35 : 0.1) + Math.random() * 0.55;
  const widthPx = 45 + Math.random() * 75;
  bumps.push({ x: SW() + widthPx, depth, widthPx, isUp });
  bumpInterval = 1.8 + Math.random() * 2.8;
}

function spawnManualBump() { spawnBump(true); }

function bumpProfileAt(bx, bmp) {
  const dx = bx - bmp.x;
  if (Math.abs(dx) < bmp.widthPx/2) {
    const t = 1 - 2*Math.abs(dx)/bmp.widthPx;
    return bmp.depth * Math.pow(Math.sin(t*Math.PI/2), 2);
  }
  return 0;
}

function getForce(carX) {
  let F = 0;
  for (const bmp of bumps) {
    const h = bumpProfileAt(carX, bmp);
    if (h > 0) F += (bmp.isUp ? 1 : -1) * h * 20; // fuerza fija, independiente de k
  }
  return F;
}

// ═══════════════════════════════════════════
//  CANVAS + RENDER (tema claro)
// ═══════════════════════════════════════════
const simCanvas   = document.getElementById('simCanvas');
const graphCanvas = document.getElementById('graphCanvas');
const ctx  = simCanvas.getContext('2d');
const gctx = graphCanvas.getContext('2d');
const DPR  = window.devicePixelRatio || 1;

function resizeCanvases() {
  simCanvas.width  = simCanvas.offsetWidth  * DPR;
  simCanvas.height = simCanvas.offsetHeight * DPR;
  ctx.scale(DPR, DPR);
  graphCanvas.width  = graphCanvas.offsetWidth  * DPR;
  graphCanvas.height = graphCanvas.offsetHeight * DPR;
  gctx.scale(DPR, DPR);
}
resizeCanvases();
window.addEventListener('resize', () => { resizeCanvases(); });

const SW = () => simCanvas.width/DPR;
const SH = () => simCanvas.height/DPR;
const GW = () => graphCanvas.width/DPR;
const GH = () => graphCanvas.height/DPR;

const ROAD_Y_FRAC = 0.65;
const CAR_X_FRAC  = 0.28;
const WHEEL_R     = 22;
const REST_OFFSET = 80;
const SCALE_PX    = 55;

let roadOffset = 0;
let wheelAngle = 0;
const MAX_PTS = 400;
let graphData = [];

// ── Sky (claro: cielo diurno) ──
function drawSky(W, H, rY) {
  // Cielo de día
  const grad = ctx.createLinearGradient(0,0,0,rY);
  grad.addColorStop(0, '#c9dff5');
  grad.addColorStop(1, '#e8f4fc');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, W, rY);

  // Nubes simples
  ctx.fillStyle = 'rgba(255,255,255,0.7)';
  const cloudPositions = [
    {x: W*0.1, y: rY*0.2, r: 20},
    {x: W*0.15, y: rY*0.22, r: 28},
    {x: W*0.2, y: rY*0.19, r: 18},
    {x: W*0.55, y: rY*0.3, r: 16},
    {x: W*0.6, y: rY*0.28, r: 24},
    {x: W*0.65, y: rY*0.31, r: 18},
    {x: W*0.85, y: rY*0.15, r: 20},
    {x: W*0.9, y: rY*0.13, r: 14},
  ];
  for (const c of cloudPositions) {
    ctx.beginPath();
    ctx.arc(c.x, c.y, c.r, 0, Math.PI*2);
    ctx.fill();
  }

  // Sol
  ctx.beginPath();
  ctx.arc(W*0.88, rY*0.18, 22, 0, Math.PI*2);
  ctx.fillStyle = '#ffe566';
  ctx.fill();
  ctx.beginPath();
  ctx.arc(W*0.88, rY*0.18, 26, 0, Math.PI*2);
  ctx.strokeStyle = 'rgba(255,229,102,0.3)';
  ctx.lineWidth = 6;
  ctx.stroke();

  // Colinas
  ctx.fillStyle = '#b8d4a0';
  ctx.beginPath();
  ctx.moveTo(0, rY);
  const pts = [0,0, .08,.22, .14,.32, .10,.45, .26,.58, .12,.70, .20,.83, .08,1, 0];
  for(let i=0;i<pts.length-1;i+=2) ctx.lineTo(pts[i]*W, rY - pts[i+1]*rY*.28);
  ctx.lineTo(W, rY);
  ctx.closePath();
  ctx.fill();

  // Segunda capa de colinas más oscura
  ctx.fillStyle = '#8fbf7a';
  ctx.beginPath();
  ctx.moveTo(0, rY);
  const pts2 = [0,.05, .1,.18, .22,.08, .35,.22, .5,.14, .68,.19, .85,.1, 1,.05, 1,0];
  for(let i=0;i<pts2.length;i+=2) ctx.lineTo(pts2[i]*W, rY - pts2[i+1]*rY*.18);
  ctx.lineTo(W, rY);
  ctx.closePath();
  ctx.fill();
}

// ── Carretera ──
function drawRoad(W, H, rY) {
  // Asfalto
  ctx.fillStyle = '#6b7280';
  ctx.fillRect(0, rY, W, H - rY);

  // Textura lateral
  ctx.fillStyle = '#9ca3af';
  ctx.fillRect(0, rY, W, 6);

  // Franjas laterales blancas
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 4;
  ctx.beginPath(); ctx.moveTo(0, rY+4); ctx.lineTo(W, rY+4); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0, H-4); ctx.lineTo(W, H-4); ctx.stroke();

  // Línea central amarilla discontinua
  ctx.strokeStyle = '#fbbf24';
  ctx.lineWidth = 3;
  ctx.setLineDash([38, 30]);
  ctx.lineDashOffset = -(roadOffset % 68);
  ctx.beginPath(); ctx.moveTo(0, rY+(H-rY)*0.5); ctx.lineTo(W, rY+(H-rY)*0.5); ctx.stroke();
  ctx.setLineDash([]);
}

// ── Baches ──
function drawBumps(rY) {
  for (const bmp of bumps) {
    const hw = bmp.widthPx;
    const dpx = bmp.depth * SCALE_PX * 0.7;
    if (bmp.isUp) {
      ctx.beginPath(); ctx.moveTo(bmp.x-hw/2, rY);
      for(let i=0;i<=hw;i++){
        const bx=bmp.x-hw/2+i, d=bumpProfileAt(bx, bmp);
        ctx.lineTo(bx, rY - d*SCALE_PX*0.7);
      }
      ctx.lineTo(bmp.x+hw/2, rY); ctx.closePath();
      ctx.fillStyle='#d97706'; ctx.fill();
      ctx.strokeStyle='#f59e0b'; ctx.lineWidth=2; ctx.stroke();
      ctx.fillStyle='#92400e'; ctx.font='bold 9px monospace';
      ctx.textAlign='center'; ctx.fillText('BACHE', bmp.x, rY - dpx - 8);
    } else {
      ctx.beginPath(); ctx.moveTo(bmp.x-hw/2, rY);
      for(let i=0;i<=hw;i++){
        const bx=bmp.x-hw/2+i, d=bumpProfileAt(bx, bmp);
        ctx.lineTo(bx, rY + d*SCALE_PX*0.7);
      }
      ctx.lineTo(bmp.x+hw/2, rY); ctx.closePath();
      ctx.fillStyle='#374151'; ctx.fill();
      ctx.strokeStyle='#6b7280'; ctx.lineWidth=2; ctx.stroke();
      ctx.fillStyle='#1f2937'; ctx.font='bold 9px monospace';
      ctx.textAlign='center'; ctx.fillText('HOYO', bmp.x, rY + dpx + 16);
    }
  }
}

// ── Suspensión visual ──
function drawSpring(x0, y1, y2) {
  const coils=6, amp=5, seg=coils*4, h=y2-y1;
  ctx.beginPath(); ctx.moveTo(x0, y1);
  for(let i=0;i<=seg;i++){
    const fy=y1+h*i/seg, fx=x0+amp*Math.sin(i*Math.PI/2);
    ctx.lineTo(fx, fy);
  }
  ctx.strokeStyle='#b91c1c'; ctx.lineWidth=2; ctx.stroke();
}

function drawDamper(x0, y1, y2) {
  const mid=(y1+y2)/2, hw=8;
  ctx.strokeStyle='#3b82f6'; ctx.lineWidth=2.5;
  ctx.beginPath(); ctx.moveTo(x0, y1); ctx.lineTo(x0, mid-6); ctx.stroke();
  ctx.fillStyle='#2563eb'; ctx.fillRect(x0-hw, mid-6, hw*2, 13);
  ctx.beginPath(); ctx.moveTo(x0, mid+7); ctx.lineTo(x0, y2); ctx.stroke();
}

function drawWheel(wx, wy) {
  ctx.save(); ctx.translate(wx, wy); ctx.rotate(wheelAngle);
  ctx.beginPath(); ctx.arc(0,0,WHEEL_R,0,Math.PI*2);
  ctx.fillStyle='#1f2937'; ctx.fill();
  ctx.strokeStyle='#374151'; ctx.lineWidth=2; ctx.stroke();
  for(let t=0;t<8;t++){
    const a=t/8*Math.PI*2;
    ctx.beginPath(); ctx.arc(0,0,WHEEL_R,a,a+.2);
    ctx.strokeStyle='#4b5563'; ctx.lineWidth=3; ctx.stroke();
  }
  ctx.beginPath(); ctx.arc(0,0,WHEEL_R*.55,0,Math.PI*2);
  ctx.fillStyle='#d1d5db'; ctx.fill();
  for(let s=0;s<5;s++){
    const a=s/5*Math.PI*2;
    ctx.beginPath();
    ctx.moveTo(Math.cos(a)*WHEEL_R*.15, Math.sin(a)*WHEEL_R*.15);
    ctx.lineTo(Math.cos(a)*WHEEL_R*.50, Math.sin(a)*WHEEL_R*.50);
    ctx.strokeStyle='#9ca3af'; ctx.lineWidth=2.5; ctx.stroke();
  }
  ctx.beginPath(); ctx.arc(0,0,WHEEL_R*.1,0,Math.PI*2);
  ctx.fillStyle='#6b7280'; ctx.fill();
  ctx.restore();
}

// ── Auto Deportivo (tema claro) ──
function drawCar(carX, chassisY, roadY) {
  const w1x=carX-55, w2x=carX+55, wy=roadY;

  // Suspensión
  ctx.strokeStyle='rgba(0,0,0,0.1)'; ctx.lineWidth=2;
  [w1x,w2x].forEach(wx=>{
    ctx.beginPath(); ctx.moveTo(wx, wy-WHEEL_R); ctx.lineTo(wx, chassisY+36); ctx.stroke();
  });

  [w1x,w2x].forEach(wx=>{
    const top=chassisY+34, bot=wy-WHEEL_R+2;
    drawDamper(wx-4, top, bot);
    drawSpring(wx+4, top, bot);
  });

  // Sombra
  ctx.beginPath(); ctx.ellipse(carX, wy+4, 80, 6, 0, 0, Math.PI*2);
  ctx.fillStyle='rgba(0,0,0,0.18)'; ctx.fill();

  // Chasis deportivo - más bajo y largo
  const carColor = '#f3f4f6'; // blanco
  const carDarker = '#d1d5db';
  const detailBlue = '#3b82f6';
  ctx.beginPath();
  ctx.moveTo(carX-95, chassisY+28);
  ctx.lineTo(carX+95, chassisY+28);
  ctx.lineTo(carX+100, chassisY+18);
  ctx.lineTo(carX+75, chassisY+5);
  ctx.lineTo(carX+30, chassisY-8);
  ctx.lineTo(carX-15, chassisY-28);
  ctx.lineTo(carX-50, chassisY-28);
  ctx.lineTo(carX-85, chassisY-5);
  ctx.lineTo(carX-100, chassisY+12);
  ctx.closePath();
  ctx.fillStyle = carColor; ctx.fill();
  ctx.strokeStyle = carDarker; ctx.lineWidth = 2; ctx.stroke();

  // Franja lateral deportiva - azul
  ctx.beginPath();
  ctx.moveTo(carX-98, chassisY+16);
  ctx.lineTo(carX+92, chassisY+16);
  ctx.lineTo(carX+88, chassisY+20);
  ctx.lineTo(carX-96, chassisY+20);
  ctx.closePath();
  ctx.fillStyle = detailBlue; ctx.fill();

  // Ventanas laterales
  ctx.beginPath();
  ctx.moveTo(carX+30, chassisY-2);
  ctx.lineTo(carX-10, chassisY-22);
  ctx.lineTo(carX-45, chassisY-22);
  ctx.lineTo(carX-80, chassisY-2);
  ctx.closePath();
  ctx.fillStyle = 'rgba(147,197,253,0.5)'; ctx.fill();
  ctx.strokeStyle = 'rgba(255,255,255,0.5)'; ctx.lineWidth=1; ctx.stroke();

  // Parabrisas Sport - muy inclinado
  ctx.beginPath();
  ctx.moveTo(carX+28, chassisY-3);
  ctx.lineTo(carX-12, chassisY-26);
  ctx.lineTo(carX-48, chassisY-26);
  ctx.lineTo(carX-82, chassisY-4);
  ctx.closePath();
  ctx.fillStyle = 'rgba(147,197,253,0.6)'; ctx.fill();
  ctx.strokeStyle = 'rgba(255,255,255,0.4)'; ctx.lineWidth=1; ctx.stroke();

  // Reflejo parabrisas
  ctx.beginPath();
  ctx.moveTo(carX-5, chassisY-24);
  ctx.lineTo(carX+12, chassisY-6);
  ctx.lineTo(carX+3, chassisY-6);
  ctx.lineTo(carX-12, chassisY-24);
  ctx.fillStyle = 'rgba(255,255,255,0.25)'; ctx.fill();

  // Faros LED delgados
  ctx.beginPath();
  ctx.moveTo(carX+78, chassisY+8);
  ctx.lineTo(carX+98, chassisY+8);
  ctx.lineTo(carX+95, chassisY+14);
  ctx.lineTo(carX+75, chassisY+14);
  ctx.closePath();
  ctx.fillStyle = '#fefce8'; ctx.fill();
  ctx.strokeStyle = '#facc15'; ctx.lineWidth=1; ctx.stroke();

  // Línea LED frontal
  ctx.beginPath();
  ctx.moveTo(carX+75, chassisY+18);
  ctx.lineTo(carX+98, chassisY+18);
  ctx.strokeStyle = '#3b82f6'; ctx.lineWidth = 2; ctx.stroke();

  // Luz antiniebla
  ctx.beginPath();
  ctx.arc(carX+96, chassisY+22, 4, 0, Math.PI*2);
  ctx.fillStyle = '#fef3c7'; ctx.fill();

  // Alerón trasero - un soporte inclinado
  // Soporte central inclinado
  ctx.beginPath();
  ctx.moveTo(carX-75, chassisY+5);
  ctx.lineTo(carX-70, chassisY-12);
  ctx.lineTo(carX-65, chassisY-12);
  ctx.lineTo(carX-70, chassisY+5);
  ctx.closePath();
  ctx.fillStyle = carDarker; ctx.fill();
  // Ala superior
  ctx.beginPath();
  ctx.moveTo(carX-100, chassisY-12);
  ctx.lineTo(carX-55, chassisY-12);
  ctx.lineTo(carX-50, chassisY-8);
  ctx.lineTo(carX-95, chassisY-8);
  ctx.closePath();
  ctx.fillStyle = carDarker; ctx.fill();
  ctx.strokeStyle = '#9ca3af'; ctx.lineWidth=1; ctx.stroke();

  // Luces traseras LED - azul
  ctx.beginPath();
  ctx.moveTo(carX-98, chassisY+8);
  ctx.lineTo(carX-98, chassisY+16);
  ctx.lineTo(carX-78, chassisY+16);
  ctx.lineTo(carX-75, chassisY+8);
  ctx.closePath();
  ctx.fillStyle = detailBlue; ctx.fill();
  // Línea LED
  ctx.beginPath();
  ctx.moveTo(carX-96, chassisY+12);
  ctx.lineTo(carX-80, chassisY+12);
  ctx.strokeStyle = '#60a5fa'; ctx.lineWidth=2; ctx.stroke();

  // Escape deportivo
  ctx.beginPath();
  ctx.moveTo(carX-100, chassisY+28);
  ctx.lineTo(carX-115, chassisY+32);
  ctx.lineTo(carX-115, chassisY+35);
  ctx.lineTo(carX-100, chassisY+32);
  ctx.closePath();
  ctx.fillStyle = '#374151'; ctx.fill();

  // Alerta de límite elástico
  if (Math.abs(px) > ELASTIC_LIMIT*0.85) {
    ctx.save(); ctx.globalAlpha = 0.3 + 0.3*Math.sin(simTime*20);
    ctx.strokeStyle = '#dc2626'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.rect(carX-110, chassisY-35, 215, 75);
    ctx.stroke(); ctx.restore();
  }

  drawWheel(w1x, wy);
  drawWheel(w2x, wy);
}

// ── Gráfica x(t) — tema claro ──
function drawGraph() {
  const W=GW(), H=GH();
  gctx.clearRect(0,0,W,H);

  // Fondo blanco suave
  gctx.fillStyle = '#fff';
  gctx.fillRect(0,0,W,H);

  if(graphData.length < 2) return;

  // Padding mayor para ejes
  const padL=40, padB=35, padT=8, padR=8;
  const graphW = W - padL - padR;
  const graphH = H - padT - padB;
  const midY = padT + graphH/2;

  const maxV = Math.max(...graphData.map(Math.abs), alertThreshold+.05, .2);
  const minX = 0;
  const maxX = MAX_PTS;

  // Grid vertical
  gctx.strokeStyle='rgba(0,0,0,0.05)'; gctx.lineWidth=1;
  for(let i=0;i<=5;i++){
    const y=padT+(graphH)*i/5;
    gctx.beginPath(); gctx.moveTo(padL,y); gctx.lineTo(W-padR,y); gctx.stroke();
  }
  // Grid horizontal
  for(let i=0;i<=4;i++){
    const x=padL+(graphW)*i/4;
    gctx.beginPath(); gctx.moveTo(x,padT); gctx.lineTo(x,padT+graphH); gctx.stroke();
  }

  // Línea cero (eje X)
  gctx.strokeStyle='rgba(0,0,0,0.3)'; gctx.lineWidth=1;
  gctx.beginPath(); gctx.moveTo(padL,midY); gctx.lineTo(W-padR,midY); gctx.stroke();

  // Eje Y
  gctx.beginPath(); gctx.moveTo(padL,padT); gctx.lineTo(padL,padT+graphH); gctx.stroke();

  // Umbral
  const thPx=(alertThreshold/maxV)*(graphH/2);
  gctx.strokeStyle='rgba(185,28,28,0.4)'; gctx.lineWidth=1;
  gctx.setLineDash([5,4]);
  gctx.beginPath(); gctx.moveTo(padL,midY-thPx); gctx.lineTo(W-padR,midY-thPx); gctx.stroke();
  gctx.beginPath(); gctx.moveTo(padL,midY+thPx); gctx.lineTo(W-padR,midY+thPx); gctx.stroke();
  gctx.setLineDash([]);

  // Relleno
  gctx.beginPath();
  graphData.forEach((val,i)=>{
    const gx=padL+(i/MAX_PTS)*graphW;
    const gy=midY-(val/maxV)*(graphH/2);
    i===0?gctx.moveTo(gx,gy):gctx.lineTo(gx,gy);
  });
  gctx.lineTo(padL+((graphData.length-1)/MAX_PTS)*graphW, midY);
  gctx.lineTo(padL, midY);
  gctx.closePath();
  gctx.fillStyle='rgba(26,95,180,0.08)'; gctx.fill();

  // Curva
  const colors = {sub:'#1a5fb4', crit:'#d97706', sobre:'#b91c1c'};
  gctx.strokeStyle = colors[currentMode]||'#1a5fb4';
  gctx.lineWidth = 2;
  gctx.lineJoin = 'round';
  gctx.beginPath();
  graphData.forEach((val,i)=>{
    const gx=padL+(i/MAX_PTS)*graphW;
    const gy=midY-(val/maxV)*(graphH/2);
    i===0?gctx.moveTo(gx,gy):gctx.lineTo(gx,gy);
  });
  gctx.stroke();

  // ── EJE Y: Valores ──
  gctx.fillStyle='#6b6760';
  gctx.font='9px "DM Mono", monospace';
  gctx.textAlign='right';
  const yLabels = [maxV, maxV/2, 0, -maxV/2, -maxV];
  for(let i=0;i<=4;i++){
    const y = padT + graphH*i/5;
    const val = yLabels[4-i];
    gctx.fillText(val.toFixed(2), padL-5, y+3);
    // Tick
    gctx.beginPath(); gctx.moveTo(padL-3,y); gctx.lineTo(padL,y); gctx.stroke();
  }
  // Etiqueta eje Y
  gctx.save();
  gctx.translate(12, padT+graphH/2);
  gctx.rotate(-Math.PI/2);
  gctx.textAlign='center';
  gctx.font='10px "DM Mono", monospace';
  gctx.fillText('x(t) [m]', 0, 0);
  gctx.restore();

  // ── EJE X: Valores ──
  gctx.textAlign='center';
  const timeSpan = 10; // segundos visibles
  const xLabels = [0, timeSpan/4, timeSpan/2, 3*timeSpan/4, timeSpan];
  const axisY = padT + graphH; // línea del eje X
  for(let i=0;i<=4;i++){
    const x = padL + graphW*i/4;
    const val = xLabels[i];
    gctx.fillText(val.toFixed(1)+'s', x, axisY + 12);
    // Tick
    gctx.beginPath(); gctx.moveTo(x, axisY-3); gctx.lineTo(x, axisY); gctx.stroke();
  }
  // Etiqueta eje X
  gctx.font='10px "DM Mono", monospace';
  gctx.fillText('Tiempo [s]', padL+graphW/2, axisY + 28);

  // Etiquetas umbral
  gctx.fillStyle='rgba(185,28,28,0.6)';
  gctx.font='8px monospace'; gctx.textAlign='right';
  gctx.fillText('+'+alertThreshold.toFixed(2)+'m', W-padR-2, midY-thPx-3);
  gctx.fillText('-'+alertThreshold.toFixed(2)+'m', W-padR-2, midY+thPx+9);
}

// ── Métricas ──
function updateMetrics() {
  const wn   = Math.sqrt(k/m);
  const zeta = b/(2*wn*m);

  document.getElementById('mwn').innerHTML   = wn.toFixed(2) + '<span class="m-unit">rad/s</span>';
  document.getElementById('mzeta').textContent = zeta.toFixed(3);
  document.getElementById('mvel').innerHTML   = pv.toFixed(3) + '<span class="m-unit">m/s</span>';
  document.getElementById('mth2').innerHTML   = alertThreshold.toFixed(2) + '<span class="m-unit">m</span>';

  // Big pos
  document.getElementById('bigPos').textContent = px.toFixed(3);

  // Zeta bar
  const zPct = Math.min(zeta/2, 1)*100;
  const bar = document.getElementById('zetaBar');
  bar.style.width = zPct + '%';
  bar.style.background = zeta < 0.95 ? 'var(--green)' : zeta < 1.05 ? 'var(--amber)' : 'var(--red)';

  // Zeta number
  document.getElementById('zetaNum').textContent = zeta.toFixed(3);

  // Zeta state tag
  const stateEl = document.getElementById('zetaState');
  if (zeta < 0.95) {
    stateEl.className = 'zeta-state sub'; stateEl.textContent = 'SUBAMORT.';
  } else if (zeta < 1.05) {
    stateEl.className = 'zeta-state crit'; stateEl.textContent = 'CRÍTICO';
  } else {
    stateEl.className = 'zeta-state sobre'; stateEl.textContent = 'SOBREAMORT.';
  }
}

// ═══════════════════════════════════════════
//  LOOP PRINCIPAL
// ═══════════════════════════════════════════
function loop(ts) {
  if(!last) last=ts;
  const dt = Math.min((ts-last)/1000, 0.033);
  last=ts; simTime+=dt;

  const cW=SW(), cH=SH(), rY=cH*ROAD_Y_FRAC, carX=cW*CAR_X_FRAC;

  physicsStep(dt, carX);  // RK4 con substepping (6 pasos) + clamp acc/vel

  if (px > ELASTIC_LIMIT)  { px =  ELASTIC_LIMIT; pv = 0; }
  if (px < -ELASTIC_LIMIT) { px = -ELASTIC_LIMIT; pv = 0; }

  roadOffset += ROAD_SPEED*dt;
  for(const bmp of bumps) bmp.x -= ROAD_SPEED*dt;
  bumps = bumps.filter(bmp => bmp.x > -200);

  if(autoBumpsEnabled){ bumpTimer+=dt; if(bumpTimer>=bumpInterval){ bumpTimer=0; spawnBump(); } }
  wheelAngle += (ROAD_SPEED/20)*dt;

  graphData.push(px);
  if(graphData.length > MAX_PTS) graphData.shift();

  checkAlerts(simTime);

  ctx.clearRect(0,0,cW,cH);
  drawSky(cW,cH,rY);
  drawRoad(cW,cH,rY);
  drawBumps(rY);
  const chassisY = rY - REST_OFFSET - px*SCALE_PX;
  drawCar(carX, chassisY, rY);

  drawGraph();
  updateMetrics();
  requestAnimationFrame(loop);
}

// ── Sliders ──
function autoUpdateMode() {
  const wn = Math.sqrt(k/m);
  const zeta = b/(2*wn*m);
  let md = 'crit';
  if(zeta < 0.95) md='sub';
  else if(zeta > 1.05) md='sobre';
  currentMode = md;
  updateModeBtns(md);
  resetAlerts();
}

document.getElementById('sm').addEventListener('input', e => {
  m=+e.target.value; clampParams();
  document.getElementById('lm').textContent=m.toFixed(1);
  autoUpdateMode();
});
document.getElementById('sb').addEventListener('input', e => {
  b=+e.target.value; clampParams();
  document.getElementById('lb').textContent=b.toFixed(1);
  autoUpdateMode();
});
document.getElementById('sk').addEventListener('input', e => {
  k=+e.target.value; clampParams();
  document.getElementById('lk').textContent=k.toFixed(1);
  autoUpdateMode();
});
document.getElementById('sth').addEventListener('input', e => {
  alertThreshold=+e.target.value;
  document.getElementById('lth').textContent=alertThreshold.toFixed(2);
  resetAlerts();
});

spawnBump();
requestAnimationFrame(loop);