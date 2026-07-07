import { useState, useEffect, useRef, useCallback, useMemo } from "react";

/*══════════════════════════════════════════════════════════════
  SIMULADOR DE MONTAÑA RUSA — Mecánica Clásica
  
  FÓRMULAS FÍSICAS UTILIZADAS:
  ─────────────────────────────
  • Energía Cinética:       Ec = ½·m·v²
  • Energía Potencial:      Ep = m·g·h
  • Conservación Energía:   Ei = Ef + W_fricción
  • Fuerza Centrípeta:      Fc = m·v²/R
  • Fuerza Normal (loop):   N = m·v²/R ± m·g
  • Velocidad mínima loop:  v_min = √(g·R)
  • Trabajo Fricción:       Wf = μ·N·d
  • Segunda Ley Newton:     ΣF = m·a
══════════════════════════════════════════════════════════════*/

/*──────────────────────────────────────────────
  CONSTRUCTOR DE PISTA — trayectoria continua sin saltos
  Cada sección está modelada con funciones matemáticas:
  - Colina: función coseno q(x) = A·cos(ω·x) + A
  - Loop: ecuación paramétrica circular (x-cx)²+(y-cy)²=R²
  - Camelback: función coseno de menor amplitud
  - Bunny Hops: función coseno de baja amplitud
──────────────────────────────────────────────*/
function buildTrack(params) {
  const pts = [];
  const ds = 0.08; // resolución fina de muestreo
  const H = params.hillHeight;       // altura máxima de la colina (m)
  const A = H / 2;                   // amplitud del coseno (mitad de la altura)
  const R = params.loopRadius;       // radio del loop circular (m)
  const camelH = params.camelbackHeight; // altura del camelback (m)
  const bunnyH = params.bunnyHeight;     // altura de cada bunny hop (m)

  const push = (x, y) => pts.push({ x, y });
  const addRange = (x0, x1, fn) => {
    for (let x = x0; x <= x1 + 0.001; x += ds) push(x, fn(x));
  };

  // ── SECCIÓN 1: Tramo plano de cadena [0, 12] ──
  // El carrito es arrastrado a velocidad constante
  addRange(0, 12, () => 0);

  // ── SECCIÓN 2: Colina + Primera Caída [12, 36] ──
  // Función: q(x) = A·cos(π/12·(x-24)) + A
  // Pico en x=24, y=H. Regresa a y=0 en x=36
  addRange(12 + ds, 36, (x) => A * Math.cos((Math.PI / 12) * (x - 24)) + A);

  // ── SECCIÓN 3: Transición plana al loop [36, 50] ──
  addRange(36 + ds, 50, () => 0);

  // ── SECCIÓN 4: Loop vertical ──
  // Ecuación circular: (x-50)² + (y-R)² = R²
  // Recorrido completo en sentido horario desde la base
  const loopN = 300;
  for (let i = 1; i <= loopN; i++) {
    const th = (2 * Math.PI * i) / loopN;
    // Parametrización: x = cx + R·sin(θ), y = R - R·cos(θ)
    push(50 + R * Math.sin(th), R - R * Math.cos(th));
  }

  // ── SECCIÓN 5: Transición plana post-loop [50, 56] ──
  addRange(50 + ds, 56, () => 0);

  // ── SECCIÓN 6: Camelback ──
  // Función: w(x) = (camelH/2)·cos(π/12·(x-68)) + camelH/2
  const camelA = camelH / 2;
  addRange(56 + ds, 80, (x) => camelA * Math.cos((Math.PI / 12) * (x - 68)) + camelA);

  // ── SECCIÓN 7: Transición corta [80, 84] ──
  addRange(80 + ds, 84, () => 0);

// ── SECCIÓN 8: Bunny Hops (3 colinas redondeadas) [84, 106] ──
// Función suavizada con sin² para eliminar puntas en los valles
const bunnyA = bunnyH;
const bunnyPeriod = 7.4; // ancho de cada hop
addRange(84 + ds, 106, (x) => {
  const phase = Math.sin(Math.PI * (x - 84) / bunnyPeriod);
  return bunnyA * phase * phase; // sin²(x) = colinas tipo campana
});

  // ── SECCIÓN 9: Zona de frenado [106, 118] ──
  addRange(106 + ds, 118, () => 0);

  // ── Cálculo de longitud de arco acumulada y ángulo local ──
  let totalArc = 0;
  const processed = pts.map((p, i) => {
    if (i === 0) return { ...p, s: 0, angle: 0, dydx: 0 };
    const dx = p.x - pts[i - 1].x;
    const dy = p.y - pts[i - 1].y;
    const seg = Math.sqrt(dx * dx + dy * dy); // ds = √(dx²+dy²)
    totalArc += seg;
    return { ...p, s: totalArc, angle: Math.atan2(dy, dx), dydx: seg > 1e-9 ? dy / seg : 0 };
  });

  // ── Determinar dónde termina la cadena (pico de la colina) ──
  let chainEndS = 0;
  let maxH = 0;
  for (const p of processed) {
    if (p.y > maxH) { maxH = p.y; chainEndS = p.s; }
    if (p.x > 30) break;
  }

  return { points: processed, totalArc, chainEndS };
}

/*──────────────────────────────────────────────
  INTERPOLACIÓN — posición exacta en la pista
  dado un valor de longitud de arco s
──────────────────────────────────────────────*/
function interpAt(track, s) {
  const pts = track.points;
  if (s <= 0) return { ...pts[0], idx: 0 };
  if (s >= track.totalArc) return { ...pts[pts.length - 1], idx: pts.length - 1 };
  let lo = 0, hi = pts.length - 1;
  while (lo < hi - 1) {
    const mid = (lo + hi) >> 1;
    if (pts[mid].s <= s) lo = mid; else hi = mid;
  }
  const i = hi;
  const t = pts[i].s > pts[i - 1].s ? (s - pts[i - 1].s) / (pts[i].s - pts[i - 1].s) : 0;
  return {
    x: pts[i - 1].x + t * (pts[i].x - pts[i - 1].x),
    y: pts[i - 1].y + t * (pts[i].y - pts[i - 1].y),
    angle: pts[i].angle,
    idx: i,
  };
}

/*──────────────────────────────────────────────
  MOTOR DE FÍSICA
  
  Cadena: velocidad constante hasta el pico
  Libre: ΣF = m·a a lo largo de la pista
  
  • a_gravitatoria = -g·sin(α)
  • a_fricción = -μ·|N|/m (opuesta al movimiento)
  • En pista normal: N = m·g·|cos(α)|
  • En el loop: N = m·v²/R - m·g·cos(α)
──────────────────────────────────────────────*/
function physicsStep(state, track, params, dt) {
  const pos = interpAt(track, state.s);
  const { gravity: g, friction: mu, mass: m, loopRadius: R } = params;

  // Sección de cadena: velocidad constante
  if (state.s <= track.chainEndS) {
    const cs = params.initialVelocity / 3.6; // km/h → m/s
    const newS = state.s + cs * dt;
    return { s: Math.min(newS, track.chainEndS + 0.01), v: cs, wF: 0, onChain: true };
  }

  const v = state.v;
  const angle = pos.angle;
  const sinA = Math.sin(angle);
  const cosA = Math.cos(angle);

  // Fuerza normal según la sección
  let N_force;
  const inLoop = pos.x > 44 && pos.x < 56 && pos.y > 0.5;
  if (inLoop) {
    // N = m·v²/R - m·g·cos(α) dentro del loop
    const centripetal = (v * v) / R;
    // N_force = m * centripetal - m * g * cosA;
    N_force = m * centripetal - m * g * cosA;
  } else {
    // N = m·g·|cos(α)| en pista normal
    N_force = m * g * Math.abs(cosA);
  }

  // Fricción: a_f = μ·|N|/m, opuesta al movimiento
  const frictionAccel = mu * Math.abs(N_force) / m;
  const frictionDir = v > 0 ? -1 : v < 0 ? 1 : 0;

  // Gravedad tangencial: a_g = -g·sin(α)
  const gravAccel = -g * sinA;

  // Aceleración total
  const a = gravAccel + frictionDir * frictionAccel;

  // Integración Euler
  let newV = v + a * dt;
  const newS = state.s + v * dt + 0.5 * a * dt * dt;
  if (newV < 0.01) newV = 0.01;

  // Trabajo de fricción: Wf += μ·|N|·ds
  const dsFriction = Math.abs(v * dt + 0.5 * a * dt * dt);
  const newWF = state.wF + mu * Math.abs(N_force) * dsFriction;

  return { s: Math.max(track.chainEndS, newS), v: newV, wF: newWF, onChain: false };
}

/*──────────────────────────────────────────────
  CARRITO SVG — diseño con bordes redondeados
──────────────────────────────────────────────*/
function CartSVG({ cx, cy, angle, scale, v }) {
  const deg = (-angle * 180) / Math.PI;
  const s = scale;
  return (
    <g transform={`translate(${cx},${cy}) rotate(${deg})`}>
      {/* Chasis con esquinas redondeadas (rx, ry) */}
      <rect x={-16*s} y={-14*s} width={32*s} height={12*s} rx={4*s} ry={4*s} fill="url(#cartBody)" stroke="#611" strokeWidth={0.6}/>
      {/* Ventanas redondeadas */}
      <rect x={-14*s} y={-13*s} width={10*s} height={5*s} rx={2.5*s} ry={2.5*s} fill="#0a2040" stroke="#2668aa" strokeWidth={0.3} opacity={0.85}/>
      <rect x={2*s} y={-13*s} width={10*s} height={5*s} rx={2.5*s} ry={2.5*s} fill="#0a2040" stroke="#2668aa" strokeWidth={0.3} opacity={0.85}/>
      {/* Base redondeada */}
      <rect x={-17*s} y={-3*s} width={34*s} height={3*s} rx={1.5*s} ry={1.5*s} fill="#1a1a1a"/>
      {/* Ruedas */}
      <circle cx={-10*s} cy={1.5*s} r={2.8*s} fill="#222" stroke="#555" strokeWidth={0.7}/>
      <circle cx={10*s} cy={1.5*s} r={2.8*s} fill="#222" stroke="#555" strokeWidth={0.7}/>
      <circle cx={-10*s} cy={1.5*s} r={1.2*s} fill="#777"/>
      <circle cx={10*s} cy={1.5*s} r={1.2*s} fill="#777"/>
      {/* Pasajeros */}
      <circle cx={-7*s} cy={-17*s} r={3.2*s} fill="#f0c89a" stroke="#c09060" strokeWidth={0.4}/>
      <circle cx={5*s} cy={-17*s} r={3.2*s} fill="#e8b888" stroke="#c09060" strokeWidth={0.4}/>
      <path d={`M${-10.5*s},${-19*s} Q${-7*s},${-22*s} ${-3.5*s},${-19*s}`} fill="#3a1a0a"/>
      <path d={`M${1.8*s},${-19*s} Q${5*s},${-23*s} ${8.2*s},${-19*s}`} fill="#111"/>
      <circle cx={-8.2*s} cy={-17.5*s} r={0.55*s} fill="#222"/>
      <circle cx={-5.8*s} cy={-17.5*s} r={0.55*s} fill="#222"/>
      <circle cx={3.8*s} cy={-17.5*s} r={0.55*s} fill="#222"/>
      <circle cx={6.2*s} cy={-17.5*s} r={0.55*s} fill="#222"/>
      {/* Luz trasera */}
      <circle cx={14*s} cy={-9*s} r={1.8*s} fill={v>8?"#ff8":"#aa8"} opacity={v>8?0.9:0.4}/>
      {v>8 && <circle cx={14*s} cy={-9*s} r={3.5*s} fill="#ff8" opacity={0.12}/>}
    </g>
  );
}

/*══════════════════════════════════════════════════════════════
  COMPONENTE PRINCIPAL
  
  Todos los parámetros son editables:
  - Geometría: altura colina, radio loop, camelback, bunny hops
  - Física: masa, gravedad, rozamiento, velocidad cadena
  
  Interfaz completamente redondeada (border-radius en todo)
══════════════════════════════════════════════════════════════*/
export default function App() {
  const [params, setParams] = useState({
    mass: 1900,             // kg
    gravity: 9.81,          // m/s²
    friction: 0.02,         // coeficiente μ
    hillHeight: 24,         // m
    loopRadius: 6,          // m
    camelbackHeight: 12,    // m
    bunnyHeight: 3,         // m
    initialVelocity: 7,     // km/h
  });

  const [sim, setSim] = useState({ s: 0, v: 0, wF: 0, onChain: true });
  const [running, setRunning] = useState(false);
  const [done, setDone] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [panel, setPanel] = useState(true);
  const raf = useRef(null);
  const lt = useRef(null);
  const sr = useRef(sim);

  // Reconstruir pista cuando cambian parámetros geométricos
  const track = useMemo(() => buildTrack(params), [
    params.hillHeight, params.loopRadius, params.camelbackHeight, params.bunnyHeight
  ]);

  const reset = useCallback(() => {
    const init = { s: 0, v: params.initialVelocity / 3.6, wF: 0, onChain: true };
    setSim(init); sr.current = init;
    setRunning(false); setDone(false); lt.current = null;
    if (raf.current) cancelAnimationFrame(raf.current);
  }, [params.initialVelocity]);

  useEffect(() => { reset(); }, [params.hillHeight, params.loopRadius, params.camelbackHeight, params.bunnyHeight, reset]);

  // Bucle de animación
  useEffect(() => {
    if (!running || !track) return;
    const animate = (t) => {
      if (!lt.current) lt.current = t;
      const rawDt = ((t - lt.current) / 1000) * speed;
      lt.current = t;
      const dt = Math.min(rawDt, 0.03);
      let st = { ...sr.current };
      const steps = 10; // sub-pasos para estabilidad
      for (let i = 0; i < steps; i++) {
        st = physicsStep(st, track, params, dt / steps);
      }
      if (st.s >= track.totalArc - 0.1) {
        setDone(true); setRunning(false);
        sr.current = st; setSim(st);
        return;
      }
      sr.current = st; setSim(st);
      raf.current = requestAnimationFrame(animate);
    };
    raf.current = requestAnimationFrame(animate);
    return () => { if (raf.current) cancelAnimationFrame(raf.current); lt.current = null; };
  }, [running, track, params, speed]);

  // Valores derivados
  const pos = interpAt(track, sim.s);
  const h = pos.y;
  const v = sim.v;
  const m = params.mass;
  const g = params.gravity;
  const KE = 0.5 * m * v * v;          // Ec = ½·m·v²
  const PE = m * g * h;                 // Ep = m·g·h
  const maxE = m * g * params.hillHeight + 0.5 * m * (params.initialVelocity / 3.6) ** 2 + 1;

  // G-Force: G = (v²·κ + g·cos(α)) / g donde κ = curvatura
  const gForce = useMemo(() => {
    const idx = pos.idx || 0;
    const pts = track.points;
    if (idx < 2 || idx >= pts.length - 2 || v < 0.5) return 1;
    const p0 = pts[idx - 1], p1 = pts[idx], p2 = pts[idx + 1];
    const dx1 = p1.x - p0.x, dy1 = p1.y - p0.y;
    const dx2 = p2.x - p1.x, dy2 = p2.y - p1.y;
    const ds1 = Math.sqrt(dx1*dx1+dy1*dy1), ds2 = Math.sqrt(dx2*dx2+dy2*dy2);
    if (ds1 < 1e-8 || ds2 < 1e-8) return 1;
    const dAngle = Math.atan2(dy2,dx2) - Math.atan2(dy1,dx1);
    const curvature = dAngle / ((ds1+ds2)/2);
    const ac = v * v * curvature;
    return (ac + g * Math.cos(pos.angle)) / g;
  }, [pos, v, track, g]);

  // Sección actual
  let section = "Cadena";
  if (pos.x > 12 && pos.x <= 24) section = "Lift Hill";
  else if (pos.x > 24 && pos.x <= 36) section = "First Drop";
  else if (pos.x > 36 && pos.x <= 50) section = "Transicion";
  else if (pos.x > 44 && pos.x <= 56 && pos.y > 0.5) section = "Looping";
  else if (pos.x > 56 && pos.x <= 80) section = "Camelback";
  else if (pos.x > 80 && pos.x <= 106) section = "Bunny Hops";
  else if (pos.x > 106) section = "Frenado";
  if (done) section = "Recorrido completado";

  // Coordenadas SVG con escala dinámica
  const W = 860, H_ = 430, pL = 30, pR = 10, pT = 25, pB = 30;
  const pW = W - pL - pR, pH_ = H_ - pT - pB;
  const MX = Math.max(122, (track.points[track.points.length - 1]?.x || 118) + 5);
  const MY = Math.max(34, params.hillHeight + 10, params.loopRadius * 2 + 10);
  const sx = (x) => pL + (x / MX) * pW;
  const sy = (y) => pT + pH_ - (y / MY) * pH_;

  const trackPath = track.points.map((p, i) =>
    `${i === 0 ? "M" : "L"}${sx(p.x).toFixed(2)},${sy(p.y).toFixed(2)}`
  ).join(" ");

  // Durmientes
  const ties = useMemo(() => {
    const t = [];
    for (let i = 0; i < track.points.length; i += 8) {
      const p = track.points[i], a = p.angle + Math.PI / 2, len = 3.5;
      t.push({ x1: sx(p.x)+Math.cos(a)*len, y1: sy(p.y)-Math.sin(a)*len, x2: sx(p.x)-Math.cos(a)*len, y2: sy(p.y)+Math.sin(a)*len });
    }
    return t;
  }, [track]);

  // Columnas de soporte
  const supports = useMemo(() => {
    const s = [];
    for (let xc = 3; xc <= 115; xc += 2.5) {
      const pt = track.points.reduce((b, p) => (Math.abs(p.x - xc) < Math.abs(b.x - xc) ? p : b));
      if (pt.y > 0.8) s.push({ x: pt.x, y: pt.y });
    }
    return s;
  }, [track]);

  // Estrellas
  const stars = useMemo(() => Array.from({length: 70}, () => ({
    cx: Math.random()*W, cy: Math.random()*(H_*0.4), r: Math.random()*1+0.2, o: Math.random()*0.5+0.2
  })), []);

  const updateParam = (k, val) => {
    setParams(p => ({ ...p, [k]: val }));
    const init = { s: 0, v: (k === "initialVelocity" ? val : params.initialVelocity) / 3.6, wF: 0, onChain: true };
    setSim(init); sr.current = init; setRunning(false); setDone(false);
  };

  const progress = (sim.s / track.totalArc) * 100;
  // v_min = √(g·R) — velocidad mínima en cima del loop
  const vMinLoop = Math.sqrt(params.gravity * params.loopRadius);

  // ═══ RADIOS DE ESQUINAS (todo redondeado) ═══
  const RD = { card: 16, btn: 12, input: 10, badge: 20, bar: 8 };

  const cardSt = {
    background: "rgba(8,8,20,0.9)",
    borderRadius: RD.card,
    border: "1px solid rgba(30,30,60,0.8)",
  };

  const btnSt = {
    border: "none",
    borderRadius: RD.btn,
    fontSize: 13,
    fontWeight: 600,
    cursor: "pointer",
    transition: "all 0.2s ease",
    outline: "none",
  };

  return (
    <div style={{ background: "#06060e", minHeight: "100vh", fontFamily: "'Inter',system-ui,sans-serif", color: "#ddd" }}>

      {/* Encabezado */}
      <div style={{ padding: "14px 16px 6px", display: "flex", alignItems: "center", justifyContent: "space-between", flexWrap: "wrap" }}>
        <div>
          <h1 style={{ margin: 0, fontSize: 24, fontWeight: 800, letterSpacing: -0.5, background: "linear-gradient(90deg,#ff4466,#ffaa33,#44aaff)", WebkitBackgroundClip: "text", WebkitTextFillColor: "transparent" }}>
            Simulador Montana Rusa 
          </h1>
          <p style={{ margin: "2px 0 0", fontSize: 10, color: "#445", letterSpacing: 0.5 }}>
            Mecanica Clasica UTP-ICA 2026 
          </p>
        </div>
        <button onClick={() => setPanel(p => !p)} style={{ ...btnSt, background: panel ? "rgba(68,102,170,0.2)" : "rgba(18,18,42,0.8)", color: panel ? "#7af" : "#556", padding: "6px 14px", fontSize: 11, border: "1px solid rgba(40,40,80,0.5)" }}>
          {panel ? "Ocultar Panel" : "Mostrar Panel"}
        </button>
      </div>

      <div style={{ display: "flex", flexWrap: "wrap", gap: 10, padding: "4px 14px 14px" }}>
        {/* Columna principal */}
        <div style={{ flex: "1 1 600px", minWidth: 0 }}>

          {/* Barra de progreso redondeada */}
          <div style={{ height: 4, background: "rgba(15,15,30,0.8)", borderRadius: RD.bar, marginBottom: 6, overflow: "hidden" }}>
            <div style={{ height: "100%", width: `${progress}%`, background: done ? "linear-gradient(90deg,#22aa44,#44dd66)" : "linear-gradient(90deg,#ff3355,#ffaa33,#3388ff)", borderRadius: RD.bar, transition: "width 80ms ease" }} />
          </div>

          {/* Escena SVG */}
          <div style={{ ...cardSt, overflow: "hidden", padding: 0 }}>
            <svg viewBox={`0 0 ${W} ${H_}`} style={{ width: "100%", display: "block" }}>
              <defs>
                <linearGradient id="sky" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="0%" stopColor="#050515"/><stop offset="50%" stopColor="#0a1030"/><stop offset="100%" stopColor="#081018"/>
                </linearGradient>
                <linearGradient id="gnd" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="0%" stopColor="#1a2a12"/><stop offset="100%" stopColor="#0a1508"/>
                </linearGradient>
                <linearGradient id="cartBody" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="0%" stopColor="#ee2244"/><stop offset="100%" stopColor="#aa1133"/>
                </linearGradient>
              </defs>

              <rect width={W} height={H_} fill="url(#sky)"/>
              {stars.map((s,i) => <circle key={i} cx={s.cx} cy={s.cy} r={s.r} fill="#dde" opacity={s.o}/>)}
              <circle cx={W-70} cy={42} r={9} fill="#ffeedd" opacity={0.5}/>
              <circle cx={W-67} cy={40} r={7} fill="#06060e" opacity={0.25}/>
              <path d={`M0,${sy(-1)} Q${W*0.2},${sy(3)} ${W*0.35},${sy(-0.5)} Q${W*0.55},${sy(2)} ${W*0.75},${sy(-1)} Q${W*0.9},${sy(1.5)} ${W},${sy(-1.5)} V${H_} H0Z`} fill="#0c1810" opacity={0.5}/>
              <rect x={pL} y={sy(0)} width={pW} height={H_-sy(0)} fill="url(#gnd)"/>
              <line x1={pL} y1={sy(0)} x2={pL+pW} y2={sy(0)} stroke="#2a4a1a" strokeWidth={1.5}/>

              {[0,6,12,18,24,30].filter(yv => yv <= MY).map(yv =>
                <g key={yv}><line x1={pL} y1={sy(yv)} x2={pL+pW} y2={sy(yv)} stroke="#0d0d20" strokeWidth={0.3}/><text x={pL-4} y={sy(yv)+3} fill="#334" fontSize={6.5} textAnchor="end" fontFamily="monospace">{yv}</text></g>
              )}

              {supports.map((s,i) => (
                <g key={i}>
                  <line x1={sx(s.x)} y1={sy(s.y)+3} x2={sx(s.x)} y2={sy(0)} stroke="#282840" strokeWidth={1.3}/>
                  <rect x={sx(s.x)-2} y={sy(0)-1} width={4} height={2.5} rx={1} ry={1} fill="#222238"/>
                </g>
              ))}

              {ties.map((t,i) => <line key={i} x1={t.x1} y1={t.y1} x2={t.x2} y2={t.y2} stroke="#3a3a55" strokeWidth={0.7}/>)}

              <path d={trackPath} fill="none" stroke="#2244aa" strokeWidth={6} opacity={0.07}/>
              <path d={trackPath} fill="none" stroke="#667799" strokeWidth={3} strokeLinecap="round" strokeLinejoin="round"/>
              <path d={trackPath} fill="none" stroke="#99aacc" strokeWidth={0.6} opacity={0.35}/>

              {[[6,1.5,"CADENA"],[20,params.hillHeight+3,"LIFT HILL"],[30,params.hillHeight+3,"FIRST DROP"],[50,params.loopRadius*2+3.5,"LOOP"],[68,params.camelbackHeight+2.5,"CAMELBACK"],[95,params.bunnyHeight+2,"BUNNY HOPS"],[112,1.5,"FRENO"]].map(([x,y,t],i) =>
                <text key={i} x={sx(x)} y={sy(y)} fill="#223344" fontSize={6} textAnchor="middle" fontWeight="700" letterSpacing={1} fontFamily="monospace">{t}</text>
              )}

              {v > 6 && Array.from({length: Math.min(Math.floor(v/3), 10)}).map((_,i) => {
                const off = (i+1)*3.5;
                return <circle key={i} cx={sx(pos.x)-Math.cos(pos.angle)*off} cy={sy(pos.y)+Math.sin(pos.angle)*off} r={1.8-i*0.14} fill="#ff4466" opacity={0.35-i*0.03}/>;
              })}

              <CartSVG cx={sx(pos.x)} cy={sy(pos.y)} angle={pos.angle} scale={0.55} v={v}/>

              {/* HUD redondeado */}
              <rect x={W-175} y={6} width={167} height={76} rx={14} ry={14} fill="#000" opacity={0.6}/>
              <text x={W-163} y={24} fill="#ff6" fontSize={14} fontWeight="800" fontFamily="monospace">{v.toFixed(1)} <tspan fontSize={8} fill="#aa8">m/s</tspan></text>
              <text x={W-163} y={40} fill="#6f6" fontSize={10} fontFamily="monospace">h = {h.toFixed(1)} m</text>
              <text x={W-163} y={54} fill="#f88" fontSize={10} fontFamily="monospace">{(v*3.6).toFixed(0)} km/h</text>
              <text x={W-163} y={68} fill={Math.abs(gForce)>2.5?"#f44":"#aaa"} fontSize={9} fontFamily="monospace">G: {gForce.toFixed(1)}g</text>
              <text x={W-82} y={24} fill={sim.onChain?"#fa4":"#4a4"} fontSize={8} fontFamily="monospace">{sim.onChain?"CADENA":"LIBRE"}</text>

              {done && (
                <g>
                  <rect x={W/2-90} y={H_/2-28} width={180} height={56} rx={14} ry={14} fill="#000" opacity={0.75}/>
                  <text x={W/2} y={H_/2+7} fill="#4f4" fontSize={15} fontWeight="800" textAnchor="middle" fontFamily="monospace">RECORRIDO COMPLETADO</text>
                </g>
              )}
            </svg>
          </div>

          {/* Botones de control */}
          <div style={{ display: "flex", gap: 6, marginTop: 8, alignItems: "center", flexWrap: "wrap" }}>
            <button onClick={() => { if (done) reset(); else setRunning(r => !r); }}
              style={{ ...btnSt, padding: "10px 24px", color: "#fff",
                background: running ? "linear-gradient(135deg,#bb2233,#881122)" : done ? "linear-gradient(135deg,#3366aa,#224488)" : "linear-gradient(135deg,#1a8a3a,#146a28)",
                boxShadow: `0 4px 20px ${running?"rgba(187,34,51,0.3)":done?"rgba(51,102,170,0.3)":"rgba(26,138,58,0.3)"}` }}>
              {done ? "Reiniciar" : running ? "Pausar" : "Iniciar"}
            </button>
            <button onClick={reset} style={{ ...btnSt, padding: "10px 16px", background: "rgba(24,24,40,0.8)", color: "#667", border: "1px solid rgba(40,40,60,0.5)" }}>Reset</button>
            <div style={{ display: "flex", gap: 2, marginLeft: 8, background: "rgba(10,10,24,0.8)", borderRadius: RD.btn, padding: 3, border: "1px solid rgba(30,30,60,0.5)" }}>
              {[0.25,0.5,1,2,3].map(s => (
                <button key={s} onClick={() => setSpeed(s)}
                  style={{ ...btnSt, padding: "4px 10px", fontSize: 10, background: speed===s?"rgba(42,74,106,0.8)":"transparent", color: speed===s?"#fff":"#445", borderRadius: RD.input }}>{s}x</button>
              ))}
            </div>
            <div style={{ marginLeft: "auto", fontSize: 11, color: done?"#4f4":"#7af", fontWeight: 600, padding: "5px 14px", background: "rgba(13,13,32,0.8)", borderRadius: RD.badge, border: "1px solid rgba(30,30,60,0.5)" }}>{section}</div>
          </div>

          {/* Tarjetas de datos */}
          <div style={{ display: "grid", gridTemplateColumns: "repeat(auto-fit, minmax(105px, 1fr))", gap: 5, marginTop: 8 }}>
            {[
              {l:"Velocidad",v:`${v.toFixed(1)}`,u:"m/s",s:`${(v*3.6).toFixed(0)} km/h`,c:"#ffcc33"},
              {l:"Altura",v:`${h.toFixed(1)}`,u:"m",c:"#44dd66"},
              {l:"Ec = ½mv²",v:`${(KE/1000).toFixed(1)}`,u:"kJ",c:"#ff5555"},
              {l:"Ep = mgh",v:`${(PE/1000).toFixed(1)}`,u:"kJ",c:"#4499ff"},
              {l:"E Total",v:`${((KE+PE)/1000).toFixed(1)}`,u:"kJ",c:"#ffaa44"},
              {l:"W Friccion",v:`${(sim.wF/1000).toFixed(2)}`,u:"kJ",c:"#aa66aa"},
              {l:"G-Force",v:`${gForce.toFixed(1)}`,u:"g",c:Math.abs(gForce)>2.5?"#ff4444":"#aaaaaa"},
            ].map(d => (
              <div key={d.l} style={{ ...cardSt, padding: "7px 9px" }}>
                <div style={{ fontSize: 8, color: "#445", textTransform: "uppercase", letterSpacing: 0.5 }}>{d.l}</div>
                <div style={{ fontSize: 16, fontWeight: 800, color: d.c, fontFamily: "monospace" }}>{d.v}<span style={{ fontSize: 8, color: "#445", marginLeft: 2 }}>{d.u}</span></div>
                {d.s && <div style={{ fontSize: 9, color: "#445" }}>{d.s}</div>}
              </div>
            ))}
          </div>

          {/* Barra de energía — Ei = Ef + Wf */}
          <div style={{ ...cardSt, marginTop: 6, padding: "8px 10px" }}>
            <div style={{ fontSize: 8, color: "#334", textTransform: "uppercase", letterSpacing: 1, marginBottom: 3 }}>
              Distribucion de Energia — Ei = Ef + Wf
            </div>
            <div style={{ height: 14, borderRadius: RD.bar, overflow: "hidden", display: "flex", background: "rgba(6,6,16,0.8)" }}>
              <div style={{ width: `${(KE/maxE)*100}%`, background: "linear-gradient(90deg,#aa1111,#ff4444)", borderRadius: `${RD.bar}px 0 0 ${RD.bar}px`, transition: "width 80ms ease" }}/>
              <div style={{ width: `${(PE/maxE)*100}%`, background: "linear-gradient(90deg,#1a4a8a,#4488ff)", transition: "width 80ms ease" }}/>
              <div style={{ width: `${(sim.wF/maxE)*100}%`, background: "linear-gradient(90deg,#663366,#aa44aa)", borderRadius: `0 ${RD.bar}px ${RD.bar}px 0`, transition: "width 80ms ease" }}/>
            </div>
            <div style={{ display: "flex", gap: 14, marginTop: 3, fontSize: 8, color: "#556" }}>
              <span><span style={{ color: "#f55" }}>●</span> Cinetica (½mv²)</span>
              <span><span style={{ color: "#4af" }}>●</span> Potencial (mgh)</span>
              <span><span style={{ color: "#a6a" }}>●</span> Friccion (μNd)</span>
            </div>
          </div>
        </div>

        {/* ═══ PANEL LATERAL — todos los parámetros editables ═══ */}
        {panel && (
          <div style={{ flex: "0 0 210px", display: "flex", flexDirection: "column", gap: 6 }}>

            {/* Geometría de pista editable */}
            <div style={{ ...cardSt, padding: 12 }}>
              <h3 style={{ margin: "0 0 10px", fontSize: 12, color: "#66aaff", fontWeight: 700, borderBottom: "1px solid rgba(40,60,100,0.3)", paddingBottom: 6 }}>
                Geometria de Pista
              </h3>
              {[
                { k: "hillHeight", l: "Altura Colina", min: 10, max: 45, step: 1, u: "m", c: "#4ecdc4" },
                { k: "loopRadius", l: "Radio Loop", min: 3, max: 12, step: 0.5, u: "m", c: "#ffa94d" },
                { k: "camelbackHeight", l: "Altura Camelback", min: 3, max: 25, step: 1, u: "m", c: "#c084fc" },
                { k: "bunnyHeight", l: "Altura Bunny Hops", min: 1, max: 10, step: 0.5, u: "m", c: "#ff6b9d" },
              ].map(p => (
                <div key={p.k} style={{ marginBottom: 10 }}>
                  <div style={{ display: "flex", justifyContent: "space-between", fontSize: 10, color: "#667", marginBottom: 2 }}>
                    <span>{p.l}</span>
                    <span style={{ color: p.c, fontFamily: "monospace", fontWeight: 700, background: "rgba(255,255,255,0.04)", padding: "1px 6px", borderRadius: 6 }}>
                      {params[p.k]} {p.u}
                    </span>
                  </div>
                  <input type="range" min={p.min} max={p.max} step={p.step} value={params[p.k]}
                    onChange={e => updateParam(p.k, parseFloat(e.target.value))}
                    style={{ width: "100%", accentColor: p.c, height: 4, borderRadius: 999 }}/>
                </div>
              ))}
            </div>

            {/* Parámetros físicos editables */}
            <div style={{ ...cardSt, padding: 12 }}>
              <h3 style={{ margin: "0 0 10px", fontSize: 12, color: "#44dd88", fontWeight: 700, borderBottom: "1px solid rgba(40,80,60,0.3)", paddingBottom: 6 }}>
                Parametros Fisicos
              </h3>
              {[
                { k: "mass", l: "Masa (m)", min: 100, max: 5000, step: 50, u: "kg", c: "#7eb8ff" },
                { k: "gravity", l: "Gravedad (g)", min: 0.5, max: 30, step: 0.1, u: "m/s²", c: "#ffd93d" },
                { k: "friction", l: "Rozamiento (μ)", min: 0, max: 0.2, step: 0.005, u: "", c: "#ff8a65" },
                { k: "initialVelocity", l: "V. Cadena", min: 1, max: 30, step: 0.5, u: "km/h", c: "#69db7c" },
              ].map(p => (
                <div key={p.k} style={{ marginBottom: 10 }}>
                  <div style={{ display: "flex", justifyContent: "space-between", fontSize: 10, color: "#667", marginBottom: 2 }}>
                    <span>{p.l}</span>
                    <span style={{ color: p.c, fontFamily: "monospace", fontWeight: 700, background: "rgba(255,255,255,0.04)", padding: "1px 6px", borderRadius: 6 }}>
                      {params[p.k]} {p.u}
                    </span>
                  </div>
                  <input type="range" min={p.min} max={p.max} step={p.step} value={params[p.k]}
                    onChange={e => updateParam(p.k, parseFloat(e.target.value))}
                    style={{ width: "100%", accentColor: p.c, height: 4, borderRadius: 999 }}/>
                </div>
              ))}
            </div>

            {/* Presets de gravedad por planeta */}
            <div style={{ ...cardSt, padding: 12 }}>
              <h3 style={{ margin: "0 0 8px", fontSize: 12, color: "#ffd93d", fontWeight: 700, borderBottom: "1px solid rgba(80,70,30,0.3)", paddingBottom: 6 }}>
                Gravedad por Planeta
              </h3>
              {[
                { n: "Tierra", g: 9.81 },
                { n: "Luna", g: 1.62 },
                { n: "Marte", g: 3.72 },
                { n: "Jupiter", g: 24.79 },
              ].map(pr => (
                <button key={pr.n} onClick={() => updateParam("gravity", pr.g)}
                  style={{ ...btnSt, display: "block", width: "100%", marginBottom: 3, fontSize: 11, padding: "7px 10px", textAlign: "left",
                    background: params.gravity === pr.g ? "rgba(18,32,26,0.8)" : "rgba(8,8,15,0.6)",
                    color: params.gravity === pr.g ? "#6f6" : "#556",
                    border: `1px solid ${params.gravity === pr.g ? "rgba(30,60,35,0.6)" : "rgba(20,20,40,0.5)"}`,
                    borderRadius: RD.input }}>
                  {pr.n}
                  <span style={{ float: "right", fontFamily: "monospace", fontSize: 10, opacity: 0.8 }}>{pr.g} m/s²</span>
                </button>
              ))}
            </div>

            {/* Modelo físico — fórmulas */}
            <div style={{ ...cardSt, padding: 12 }}>
              <h3 style={{ margin: "0 0 6px", fontSize: 12, color: "#ffaa44", fontWeight: 700, borderBottom: "1px solid rgba(80,60,30,0.3)", paddingBottom: 6 }}>
                Modelo Fisico
              </h3>
              <div style={{ fontSize: 10, color: "#556", lineHeight: 2, fontFamily: "monospace" }}>
                {/* Fórmulas del modelo de mecánica clásica */}
                Ec = ½mv²<br/>
                Ep = mgh<br/>
                Wf = μ·N·d<br/>
                Ei = Ef + Wf<br/>
                <span style={{ color: "#4ecdc4" }}>v_min = √(g·R) = {vMinLoop.toFixed(2)} m/s</span><br/>
                <span style={{ color: "#445" }}>N_loop = mv²/R ± mg</span><br/>
                <span style={{ color: "#445" }}>Fc = mv²/R</span>
              </div>
            </div>

            {/* Indicador de velocidad mínima del loop */}
            <div style={{ ...cardSt, padding: 10, borderColor: v > vMinLoop ? "rgba(40,80,40,0.5)" : "rgba(80,40,40,0.5)" }}>
              <div style={{ fontSize: 9, color: "#556", marginBottom: 3 }}>
                v_min para completar loop
              </div>
              <div style={{ fontSize: 13, fontWeight: 700, fontFamily: "monospace", color: "#ffd93d" }}>
                √(g·R) = {vMinLoop.toFixed(2)} m/s
              </div>
              <div style={{ fontSize: 9, color: "#445", marginTop: 2 }}>
                = {(vMinLoop * 3.6).toFixed(1)} km/h
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}