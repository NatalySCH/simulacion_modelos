// ═══════════════════════════════════════════════════════════
//  MODELO MATEMÁTICO — Digital Twin (Basado en SuspensionDigitalTwin)
//  Ecuación:  m·x'' + b·x' + k·x = F(t)
//  Integración numérica: Runge-Kutta 4 (equivalente a odeint de scipy)
// ═══════════════════════════════════════════════════════════

let m = 2, b = 2, k = 5;
let px = 0, pv = 0;            // posición [m] y velocidad [m/s]

const ELASTIC_LIMIT = 2.0;     // límite elástico en metros
let simTime = 0;
let last = null;

// ── Integrador Runge-Kutta 4 (equivalente a scipy.odeint) ──
// dy/dt = f(y, t)  donde y = [x, v]
function rk4Step(px, pv, F_ext, dt) {
  const f = (x, v) => {
    const acc = (F_ext - b * v - k * x) / m;
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

// ── Modos preconfigurados ──
const MODES = {
  sub:   { m:2,   b:2,     k:5,   zClass:'active-sub'   },
  crit:  { m:2,   b:6.32,  k:5,   zClass:'active-crit'  },
  sobre: { m:2,   b:20,    k:5,   zClass:'active-sobre' },
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
  ['sub','crit','sobre'].forEach(id => {
    document.getElementById('btn-'+id).className =
      'mode-btn' + (id===md ? ' '+MODES[id].zClass : '');
  });
}

function resetAlerts() {
  outOfBoundsSince = null;
  alertFired = false;
  dismissedAlertLevel = null;
  lastAboveThreshold = 0;
  setAlertOk();
}

function resetPhysics() {
  px=0; pv=0; graphData=[];
  peakLog=[];
  resetAlerts();
}

// ═══════════════════════════════════════════════════════════
//  SISTEMA DE ALERTAS — Lógica de Falla
//  Si oscilación x(t) tarda >3s en estabilizarse → ERROR
// ═══════════════════════════════════════════════════════════

let alertThreshold = 0.3;
let outOfBoundsSince = null;
let alertFired = false;

// Registro de picos para exportación JSON
let peakLog = [];
let lastPeakSign = 0;
let lastPeakVal = 0;
let lastPeakTime = -999;

function recordPeak(t, x) {
  const sign = Math.sign(x);
  if (Math.abs(x) > alertThreshold) {
    if (sign !== lastPeakSign || (t - lastPeakTime) > 0.5) {
      const absX = Math.abs(x);
      if (absX > Math.abs(lastPeakVal) || (t - lastPeakTime) > 0.5) {
        peakLog.push({
          tiempo_s:       parseFloat(t.toFixed(3)),
          posicion_m:     parseFloat(x.toFixed(4)),
          velocidad_mps:  parseFloat(pv.toFixed(4)),
          m_kg:           m, b_Nsm: b, k_Nm: k,
          supera_umbral:  Math.abs(x) > alertThreshold
        });
        lastPeakSign = sign;
        lastPeakVal  = x;
        lastPeakTime = t;
      }
    }
  }
}

let lastAboveThreshold = 0;

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
      // Si se estabiliza por 1.5s debajo del umbral, se resetea
      outOfBoundsSince = null;
      dismissedAlertLevel = null;
      setAlertOk();
    } else {
      if (elapsed >= 3.0) {
        alertFired = true;
        setAlertError();
        console.error('ERROR: Amortiguador desgastado. Requiere mantenimiento.');
      } else {
        setAlertWarn(elapsed);
      }
    }
  }
}

let dismissedAlertLevel = null;

function dismissAlert() {
  const notif = document.getElementById('alertNotification');
  if (notif) notif.classList.remove('show');
  dismissedAlertLevel = alertFired ? 'error' : 'warn';
}

function setAlertOk() {
  document.getElementById('alertBox').className = 'alert-box ok';
  document.getElementById('alertText').innerHTML =
    '● SISTEMA NOMINAL<br>Oscilaciones dentro del rango.<br>Tiempo fuera de umbral: 0.0 s';
  const notif = document.getElementById('alertNotification');
  if (notif) notif.className = 'alert-notification';
}
function setAlertWarn(elapsed) {
  document.getElementById('alertBox').className = 'alert-box warn';
  document.getElementById('alertText').innerHTML =
    '⚠ ALERTA ACTIVA<br>Revise las notificaciones para más<br>detalles del evento.';
  const notif = document.getElementById('alertNotification');
  if (notif && dismissedAlertLevel !== 'warn' && dismissedAlertLevel !== 'error') {
    notif.className = 'alert-notification warn show';
    notif.innerHTML = '⚠ ADVERTENCIA<br>Oscilación sostenida: ' + elapsed.toFixed(1) + ' s<br>Umbral: ±' + alertThreshold.toFixed(2) + ' m' + 
      '<span class="alert-close" onclick="dismissAlert()">×</span>';
  }
}
function setAlertError() {
  document.getElementById('alertBox').className = 'alert-box error';
  document.getElementById('alertText').innerHTML =
    '✖ ERROR CRÍTICO<br>El sistema de amortiguación ha fallado.<br>Requiere asistencia.';
  const notif = document.getElementById('alertNotification');
  if (notif && dismissedAlertLevel !== 'error') {
    notif.className = 'alert-notification error show';
    notif.innerHTML = '✖ ERROR CRÍTICO<br>Amortiguador desgastado.<br>Requiere mantenimiento.<br><br>t_estab &gt; 3.0 s' + 
      '<span class="alert-close" onclick="dismissAlert()">×</span>';
  }
}

// ── Exportar JSON ──
function exportJSON() {
  const wn   = Math.sqrt(k / m);
  const zeta = b / (2 * wn * m);
  let estado = zeta < 0.95 ? 'SUBAMORTIGUADO' : zeta < 1.05 ? 'CRÍTICO' : 'SOBREAMORTIGUADO';

  const payload = {
    metadatos: {
      fecha_exportacion: new Date().toISOString(),
      parametros: { m_kg: m, b_Nsm: b, k_Nm: k },
      diagnostico: {
        wn_rads:  parseFloat(wn.toFixed(4)),
        zeta:     parseFloat(zeta.toFixed(4)),
        estado:   estado,
        alerta_disparada: alertFired,
        umbral_m: alertThreshold
      }
    },
    picos_vibracion: peakLog
  };

  const blob = new Blob([JSON.stringify(payload, null, 2)], { type: 'application/json' });
  const url  = URL.createObjectURL(blob);
  const a    = document.createElement('a');
  a.href = url;
  a.download = 'suspension_picos.json';
  a.click();
  URL.revokeObjectURL(url);

  // Toast
  const t = document.getElementById('toast');
  t.textContent = '✓ Exportado: ' + peakLog.length + ' picos guardados';
  t.classList.add('show');
  setTimeout(() => t.classList.remove('show'), 6000);
}

// ═══════════════════════════════════════════════════════════
//  BACHES — Entrada de fuerza F(t) al sistema
// ═══════════════════════════════════════════════════════════
let bumps = [];
let bumpTimer = 0;
let bumpInterval = 2.8;
const ROAD_SPEED = 95;

function spawnBump(forced=false) {
  const isUp    = Math.random() > 0.5;
  const depth   = (forced ? 0.35 : 0.1) + Math.random() * 0.55;
  const widthPx = 45 + Math.random() * 75;
  bumps.push({
    x: simCanvas.width / (window.devicePixelRatio||1) + widthPx,
    depth, widthPx, isUp,
    color: isUp ? '#ff9a00' : '#8855ff',
  });
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

// F(t) = fuerza del bache sobre el sistema
function getForce(carX) {
  let F = 0;
  for (const bmp of bumps) {
    const h = bumpProfileAt(carX, bmp);
    if (h > 0) F += (bmp.isUp ? 1 : -1) * h * k * 1.1;
  }
  return F;
}

// ═══════════════════════════════════════════════════════════
//  CANVAS + RENDER
// ═══════════════════════════════════════════════════════════
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
window.addEventListener('resize', resizeCanvases);

const SW = () => simCanvas.width/DPR;
const SH = () => simCanvas.height/DPR;
const GW = () => graphCanvas.width/DPR;
const GH = () => graphCanvas.height/DPR;

const ROAD_Y_FRAC = 0.66;
const CAR_X_FRAC  = 0.27;
const WHEEL_R     = 22;
const REST_OFFSET = 80;
const SCALE_PX    = 55;

let roadOffset = 0;
let wheelAngle = 0;

// ── Gráfica ──
const MAX_PTS = 400;
let graphData = [];

// ── Cielo y fondo ──
function drawSky(W, H, rY) {
  const grad = ctx.createLinearGradient(0,0,0,rY);
  grad.addColorStop(0,'#070a18'); grad.addColorStop(1,'#131830');
  ctx.fillStyle=grad; ctx.fillRect(0,0,W,rY);

  ctx.fillStyle='rgba(255,255,255,.6)';
  for(let i=0;i<50;i++){
    const sx=(i*137.5+10)%W, sy=(i*89.3+5)%(rY*.75), sz=i%4===0?2:1;
    ctx.fillRect(sx,sy,sz,sz);
  }

  // Luna
  ctx.beginPath(); ctx.arc(W*.85,rY*.2,18,0,Math.PI*2);
  ctx.fillStyle='#e8e4c0'; ctx.fill();
  ctx.beginPath(); ctx.arc(W*.85+8,rY*.2-4,14,0,Math.PI*2);
  ctx.fillStyle='#131830'; ctx.fill();

  // Colinas
  ctx.fillStyle='#0e1520';
  ctx.beginPath(); ctx.moveTo(0,rY);
  const pts=[0,0,.08,.22,.14,.32,.10,.45,.26,.58,.12,.70,.20,.83,.08,1,0];
  for(let i=0;i<pts.length;i+=2) ctx.lineTo(pts[i]*W, rY-pts[i+1]*rY*.35);
  ctx.lineTo(W,rY); ctx.closePath(); ctx.fill();
}

// ── Carretera ──
function drawRoad(W,H,rY) {
  ctx.fillStyle='#1e2130'; ctx.fillRect(0,rY,W,H-rY);
  ctx.strokeStyle='rgba(255,255,255,.03)'; ctx.lineWidth=1;
  for(let i=0;i<8;i++){
    const y=rY+(H-rY)*i/8;
    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke();
  }
  ctx.strokeStyle='#4a4f6a'; ctx.lineWidth=2;
  ctx.beginPath(); ctx.moveTo(0,rY); ctx.lineTo(W,rY); ctx.stroke();

  ctx.strokeStyle='#d4aa00'; ctx.lineWidth=4;
  ctx.setLineDash([38,30]); ctx.lineDashOffset=-(roadOffset%68);
  ctx.beginPath(); ctx.moveTo(0,rY+(H-rY)*.5); ctx.lineTo(W,rY+(H-rY)*.5); ctx.stroke();
  ctx.setLineDash([]);

  ctx.strokeStyle='rgba(255,255,255,.15)'; ctx.lineWidth=2;
  ctx.setLineDash([20,15]); ctx.lineDashOffset=-(roadOffset%35);
  ctx.beginPath(); ctx.moveTo(0,rY+10); ctx.lineTo(W,rY+10); ctx.stroke();
  ctx.setLineDash([]);
}

// ── Baches ──
function drawBumps(rY) {
  for(const bmp of bumps){
    const hw=bmp.widthPx, dpx=bmp.depth*SCALE_PX*.7;
    if(bmp.isUp){
      ctx.beginPath(); ctx.moveTo(bmp.x-hw/2,rY);
      for(let i=0;i<=hw;i++){
        const bx=bmp.x-hw/2+i, d=bumpProfileAt(bx,bmp);
        ctx.lineTo(bx, rY-d*SCALE_PX*.7);
      }
      ctx.lineTo(bmp.x+hw/2,rY); ctx.closePath();
      ctx.fillStyle='#b85500'; ctx.fill();
      ctx.strokeStyle='#ff9a00'; ctx.lineWidth=3; ctx.stroke();
      ctx.fillStyle='#ff9a00'; ctx.font='bold 9px Share Tech Mono';
      ctx.textAlign='center'; ctx.fillText('↑ BACHE',bmp.x,rY-dpx-8);
    } else {
      ctx.beginPath(); ctx.moveTo(bmp.x-hw/2,rY);
      for(let i=0;i<=hw;i++){
        const bx=bmp.x-hw/2+i, d=bumpProfileAt(bx,bmp);
        ctx.lineTo(bx, rY+d*SCALE_PX*.7);
      }
      ctx.lineTo(bmp.x+hw/2,rY); ctx.closePath();
      ctx.fillStyle='#0f0a20'; ctx.fill();
      ctx.strokeStyle='#aa55ff'; ctx.lineWidth=3; ctx.stroke();
      ctx.fillStyle='#aa55ff'; ctx.font='bold 9px Share Tech Mono';
      ctx.textAlign='center'; ctx.fillText('↓ HOYO',bmp.x,rY+dpx+16);
    }
  }
}

// ── Suspensión visual ──
function drawSpring(x0,y1,y2){
  const coils=6,amp=5,seg=coils*4, h=y2-y1;
  ctx.beginPath(); ctx.moveTo(x0,y1);
  for(let i=0;i<=seg;i++){
    const fy=y1+h*i/seg, fx=x0+amp*Math.sin(i*Math.PI/2);
    ctx.lineTo(fx,fy);
  }
  ctx.strokeStyle='#00cc66'; ctx.lineWidth=2; ctx.stroke();
}
function drawDamper(x0,y1,y2){
  const mid=(y1+y2)/2, hw=8;
  ctx.strokeStyle='#4488dd'; ctx.lineWidth=3;
  ctx.beginPath(); ctx.moveTo(x0,y1); ctx.lineTo(x0,mid-6); ctx.stroke();
  ctx.fillStyle='#2266bb'; ctx.fillRect(x0-hw,mid-6,hw*2,13);
  ctx.strokeStyle='#4488dd';
  ctx.beginPath(); ctx.moveTo(x0,mid+7); ctx.lineTo(x0,y2); ctx.stroke();
  ctx.strokeStyle='#88aaee'; ctx.lineWidth=1.5;
  ctx.beginPath(); ctx.moveTo(x0,mid-6); ctx.lineTo(x0,mid+7); ctx.stroke();
}

// ── Rueda ──
function drawWheel(wx,wy){
  ctx.save(); ctx.translate(wx,wy); ctx.rotate(wheelAngle);
  ctx.beginPath(); ctx.arc(0,0,WHEEL_R,0,Math.PI*2);
  ctx.fillStyle='#1a1a2a'; ctx.fill();
  ctx.strokeStyle='#3a3a5a'; ctx.lineWidth=2; ctx.stroke();
  for(let t=0;t<8;t++){
    const a=t/8*Math.PI*2;
    ctx.beginPath(); ctx.arc(0,0,WHEEL_R,a,a+.2);
    ctx.strokeStyle='#555'; ctx.lineWidth=3; ctx.stroke();
  }
  ctx.beginPath(); ctx.arc(0,0,WHEEL_R*.55,0,Math.PI*2);
  ctx.fillStyle='#c0c0d0'; ctx.fill();
  for(let s=0;s<5;s++){
    const a=s/5*Math.PI*2;
    ctx.beginPath();
    ctx.moveTo(Math.cos(a)*WHEEL_R*.15, Math.sin(a)*WHEEL_R*.15);
    ctx.lineTo(Math.cos(a)*WHEEL_R*.50, Math.sin(a)*WHEEL_R*.50);
    ctx.strokeStyle='#808090'; ctx.lineWidth=2.5; ctx.stroke();
  }
  ctx.beginPath(); ctx.arc(0,0,WHEEL_R*.1,0,Math.PI*2);
  ctx.fillStyle='#404050'; ctx.fill();
  ctx.restore();
}

// ── Auto Deportivo ──
function drawCar(carX,chassisY,roadY){
  const w1x=carX-52, w2x=carX+52, wy=roadY;

  ctx.strokeStyle='#333355'; ctx.lineWidth=2;
  [w1x,w2x].forEach(wx=>{
    ctx.beginPath(); ctx.moveTo(wx,wy-WHEEL_R); ctx.lineTo(wx,chassisY+34); ctx.stroke();
  });

  [w1x,w2x].forEach(wx=>{
    const top=chassisY+32, bot=wy-WHEEL_R+2;
    drawDamper(wx-5,top,bot);
    drawSpring(wx+5,top,bot);
  });

  ctx.beginPath(); ctx.ellipse(carX,wy+6,75,6,0,0,Math.PI*2);
  ctx.fillStyle='rgba(0,0,0,.4)'; ctx.fill();

  ctx.fillStyle='#111'; ctx.fillRect(carX-65,chassisY+26,130,8);

  const carColor = '#00e5ff';
  
  ctx.beginPath();
  ctx.moveTo(carX-85, chassisY+26);
  ctx.lineTo(carX+85, chassisY+26);
  ctx.lineTo(carX+88, chassisY+15);
  ctx.lineTo(carX+65, chassisY);
  ctx.lineTo(carX+25, chassisY-6);
  ctx.lineTo(carX-10, chassisY-24);
  ctx.lineTo(carX-40, chassisY-24);
  ctx.lineTo(carX-75, chassisY);
  ctx.lineTo(carX-88, chassisY+8);
  ctx.closePath();
  ctx.fillStyle = carColor; ctx.fill();
  ctx.strokeStyle = '#008b99'; ctx.lineWidth = 1.5; ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(carX-86, chassisY+12);
  ctx.lineTo(carX+80, chassisY+12);
  ctx.lineTo(carX+75, chassisY+16);
  ctx.lineTo(carX-84, chassisY+16);
  ctx.fillStyle = '#fff'; ctx.fill();
  
  ctx.fillStyle = '#111';
  ctx.beginPath();
  ctx.moveTo(carX-20, chassisY+18);
  ctx.lineTo(carX, chassisY+18);
  ctx.lineTo(carX-5, chassisY+24);
  ctx.lineTo(carX-25, chassisY+24);
  ctx.fill();

  ctx.beginPath();
  ctx.moveTo(carX+20, chassisY-6);  // base frente
  ctx.lineTo(carX-8, chassisY-21);  // techo frente
  ctx.lineTo(carX-35, chassisY-21); // techo atrás
  ctx.lineTo(carX-65, chassisY-3);  // base atrás
  ctx.lineTo(carX+20, chassisY-6);
  ctx.closePath();
  ctx.fillStyle = '#0a0a0f'; ctx.fill();
  
  ctx.strokeStyle = carColor; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(carX-6, chassisY-21); ctx.lineTo(carX-12, chassisY-6); ctx.stroke();
  
  ctx.beginPath();
  ctx.moveTo(carX-2, chassisY-20);
  ctx.lineTo(carX+14, chassisY-7);
  ctx.lineTo(carX+5, chassisY-7);
  ctx.lineTo(carX-10, chassisY-20);
  ctx.fillStyle = 'rgba(255,255,255,.15)'; ctx.fill();

  ctx.beginPath();
  ctx.moveTo(carX-70, chassisY);
  ctx.lineTo(carX-75, chassisY-18);
  ctx.lineTo(carX-95, chassisY-15);
  ctx.lineTo(carX-90, chassisY-10);
  ctx.lineTo(carX-76, chassisY-10);
  ctx.lineTo(carX-76, chassisY);
  ctx.fillStyle = '#111'; ctx.fill();
  ctx.strokeStyle= carColor; ctx.lineWidth=1; ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(carX+70, chassisY+6);
  ctx.lineTo(carX+86, chassisY+8);
  ctx.lineTo(carX+82, chassisY+13);
  ctx.lineTo(carX+65, chassisY+11);
  ctx.closePath();
  ctx.fillStyle = '#eeffff'; ctx.fill();

  ctx.beginPath();
  ctx.moveTo(carX+84, chassisY+10);
  ctx.lineTo(carX+220, chassisY+25);
  ctx.lineTo(carX+220, chassisY-5);
  ctx.closePath();
  ctx.fillStyle = 'rgba(150,220,255,.15)'; ctx.fill();
  
  ctx.beginPath();
  ctx.moveTo(carX-87, chassisY+8);
  ctx.lineTo(carX-88, chassisY+11);
  ctx.lineTo(carX-80, chassisY+11);
  ctx.lineTo(carX-77, chassisY+8);
  ctx.closePath();
  ctx.fillStyle = '#ff0033'; ctx.fill();
  
  ctx.beginPath(); ctx.arc(carX-85, chassisY+9, 10, 0, Math.PI*2);
  ctx.fillStyle = 'rgba(255,0,50,.3)'; ctx.fill();

  if(Math.abs(px) > ELASTIC_LIMIT*.85){
    ctx.save(); ctx.globalAlpha=.3+.3*Math.sin(simTime*20);
    ctx.strokeStyle='#ff3355'; ctx.lineWidth=3;
    ctx.beginPath();
    if(ctx.roundRect) ctx.roundRect(carX-100,chassisY-30,195,65,8);
    else ctx.rect(carX-100,chassisY-30,195,65);
    ctx.stroke(); ctx.restore();
  }

  drawWheel(w1x,wy); drawWheel(w2x,wy);
}

// ── Gráfica x(t) ──
function drawGraph(){
  const W=GW(), H=GH();
  gctx.clearRect(0,0,W,H);
  gctx.fillStyle='#080a12'; gctx.fillRect(0,0,W,H);
  if(graphData.length<2) return;

  const pad=8, midY=H/2;
  const maxV=Math.max(...graphData.map(Math.abs),alertThreshold+.05,.2);

  gctx.strokeStyle='rgba(255,255,255,.04)'; gctx.lineWidth=1;
  for(let i=0;i<5;i++){
    const y=pad+(H-pad*2)*i/4;
    gctx.beginPath(); gctx.moveTo(pad,y); gctx.lineTo(W-pad,y); gctx.stroke();
  }

  // Línea cero
  gctx.strokeStyle='rgba(255,214,0,.35)'; gctx.lineWidth=1.5;
  gctx.setLineDash([4,4]);
  gctx.beginPath(); gctx.moveTo(pad,midY); gctx.lineTo(W-pad,midY); gctx.stroke();
  gctx.setLineDash([]);

  // Líneas de umbral
  const thPx=(alertThreshold/maxV)*(midY-pad);
  gctx.strokeStyle='rgba(255,51,85,.6)'; gctx.lineWidth=1.5;
  gctx.setLineDash([6,4]);
  gctx.beginPath(); gctx.moveTo(pad,midY-thPx); gctx.lineTo(W-pad,midY-thPx); gctx.stroke();
  gctx.beginPath(); gctx.moveTo(pad,midY+thPx); gctx.lineTo(W-pad,midY+thPx); gctx.stroke();
  gctx.setLineDash([]);

  // Relleno
  gctx.beginPath();
  graphData.forEach((val,i)=>{
    const gx=pad+(i/MAX_PTS)*(W-pad*2);
    const gy=midY-(val/maxV)*(midY-pad);
    i===0?gctx.moveTo(gx,gy):gctx.lineTo(gx,gy);
  });
  gctx.lineTo(pad+((graphData.length-1)/MAX_PTS)*(W-pad*2),midY);
  gctx.lineTo(pad,midY); gctx.closePath();
  gctx.fillStyle='rgba(0,212,255,.07)'; gctx.fill();

  // Curva
  const colors={sub:'#00d4ff',crit:'#ffd600',sobre:'#ff6644'};
  gctx.strokeStyle=colors[currentMode]||'#00d4ff'; gctx.lineWidth=2;
  gctx.beginPath();
  graphData.forEach((val,i)=>{
    const gx=pad+(i/MAX_PTS)*(W-pad*2);
    const gy=midY-(val/maxV)*(midY-pad);
    i===0?gctx.moveTo(gx,gy):gctx.lineTo(gx,gy);
  });
  gctx.stroke();

  // Etiquetas umbral
  gctx.fillStyle='rgba(255,51,85,.7)';
  gctx.font='9px Share Tech Mono'; gctx.textAlign='right';
  gctx.fillText('+'+alertThreshold.toFixed(2)+'m',W-pad-2,midY-thPx-3);
  gctx.fillText('-'+alertThreshold.toFixed(2)+'m',W-pad-2,midY+thPx+9);
}

// ── Métricas ──
function updateMetrics(){
  const wn   = Math.sqrt(k/m);
  const zeta = b/(2*wn*m);
  document.getElementById('mwn').textContent   = wn.toFixed(2);
  document.getElementById('mzeta').textContent = zeta.toFixed(3);
  document.getElementById('mpos').textContent  = px.toFixed(3);
  document.getElementById('mvel').textContent  = pv.toFixed(3);

  const zPct=Math.min(zeta/2,1)*100;
  const bar=document.getElementById('zetaBar');
  bar.style.width=zPct+'%';
  bar.style.background=zeta<.95?'#00ff88':zeta<1.05?'#ffd600':'#ff3355';
}

// ═══════════════════════════════════════════════════════════
//  LOOP PRINCIPAL
// ═══════════════════════════════════════════════════════════
function loop(ts){
  if(!last) last=ts;
  const dt=Math.min((ts-last)/1000, 0.033);
  last=ts;
  simTime+=dt;

  const cW=SW(), cH=SH(), rY=cH*ROAD_Y_FRAC, carX=cW*CAR_X_FRAC;

  // ── Física RK4 (Backend matemático) ──
  const F_ext = getForce(carX);
  const { nx, nv } = rk4Step(px, pv, F_ext, dt);
  px = nx; pv = nv;

  // Límite elástico
  if(Math.abs(px)>=ELASTIC_LIMIT){
    px=Math.sign(px)*ELASTIC_LIMIT;
    pv*=-0.3;
  }

  // ── Baches ──
  roadOffset+=ROAD_SPEED*dt;
  for(const bmp of bumps){ bmp.x-=ROAD_SPEED*dt; }
  bumps=bumps.filter(bmp=>bmp.x>-200);
  bumpTimer+=dt;
  if(bumpTimer>=bumpInterval){ bumpTimer=0; spawnBump(); }
  wheelAngle+=(ROAD_SPEED/20)*dt;

  // ── Datos gráfica ──
  graphData.push(px);
  if(graphData.length>MAX_PTS) graphData.shift();

  // ── Lógica de alertas ──
  checkAlerts(simTime);

  // ── Render ──
  ctx.clearRect(0,0,cW,cH);
  drawSky(cW,cH,rY);
  drawRoad(cW,cH,rY);
  drawBumps(rY);
  const chassisY=rY-REST_OFFSET-px*SCALE_PX;
  drawCar(carX,chassisY,rY);

  drawGraph();
  updateMetrics();

  requestAnimationFrame(loop);
}

// ── Sliders ──
function autoUpdateMode() {
  const wn = Math.sqrt(k/m);
  const zeta = b/(2*wn*m);
  let md = 'crit';
  if(zeta < 0.95) md = 'sub';
  else if(zeta > 1.05) md = 'sobre';
  
  currentMode = md;
  ['sub','crit','sobre'].forEach(id => {
    document.getElementById('btn-'+id).className =
      'mode-btn' + (id===md ? ' '+MODES[id].zClass : '');
  });
  resetAlerts();
}

document.getElementById('sm').addEventListener('input',e=>{
  m=+e.target.value; document.getElementById('lm').textContent=m.toFixed(1);
  autoUpdateMode();
});
document.getElementById('sb').addEventListener('input',e=>{
  b=+e.target.value; document.getElementById('lb').textContent=b.toFixed(1);
  autoUpdateMode();
});
document.getElementById('sk').addEventListener('input',e=>{
  k=+e.target.value; document.getElementById('lk').textContent=k.toFixed(1);
  autoUpdateMode();
});
document.getElementById('sth').addEventListener('input',e=>{
  alertThreshold=+e.target.value;
  document.getElementById('lth').textContent=alertThreshold.toFixed(2);
  resetAlerts();
});

spawnBump();
requestAnimationFrame(loop);

// ── Hamburger Menu Toggle ──
function toggleSidebar() {
  document.body.classList.toggle('sidebar-closed');
  // Resize canvas smoothly during and after transition
  let start = performance.now();
  requestAnimationFrame(function animateResize(time) {
    resizeCanvases();
    if (time - start < 350) requestAnimationFrame(animateResize);
  });
}
