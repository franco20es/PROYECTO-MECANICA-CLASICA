import { useState, useEffect, useRef, useCallback, useMemo } from "react";

/*──────────────────────────────────────────────
  TRACK BUILDER — continuous path, no gaps
──────────────────────────────────────────────*/
function buildTrack(params) {
  const pts = [];
  const ds = 0.08; // finer resolution
  const H = params.hillHeight;
  const A = H / 2;
  const R = params.loopRadius;

  const push = (x, y) => pts.push({ x, y });
  const addRange = (x0, x1, fn) => {
    for (let x = x0; x <= x1 + 0.001; x += ds) push(x, fn(x));
  };

  // 1) Flat chain start: x [0, 12]
  addRange(0, 12, () => 0);

  // 2) Lift hill + first drop (cosine): x [12, 36]
  //    Peak at x=24, y=H. Returns to y=0 at x=36
  addRange(12 + ds, 36, (x) => A * Math.cos((Math.PI / 12) * (x - 24)) + A);

  // 3) Flat transition to loop base: x [36, 50]
  //    IMPORTANT: extends all the way to x=50 where loop bottom is
  addRange(36 + ds, 50, () => 0);

  // 4) Vertical loop: center = (50, R), radius R
  //    Starts at bottom (50, 0), goes clockwise
  const loopN = 300;
  for (let i = 1; i <= loopN; i++) {
    const th = (2 * Math.PI * i) / loopN;
    push(50 + R * Math.sin(th), R - R * Math.cos(th));
  }

  // 5) Flat after loop exit back to ground: x [50, 56]
  addRange(50 + ds, 56, () => 0);

  // 6) Camelback (height 12m, peak at x=68): x [56, 80]
  addRange(56 + ds, 80, (x) => 6 * Math.cos((Math.PI / 12) * (x - 68)) + 6);

  // 7) Short flat: x [80, 84]
  addRange(80 + ds, 84, () => 0);

  // 8) Bunny hops (3 hills, h≈3m each): x [84, 106]
  addRange(84 + ds, 106, (x) => 1.5 * Math.cos(0.85 * (x - 84)) + 1.5);

  // 9) Final brake run: x [106, 118]
  addRange(106 + ds, 118, () => 0);

  // Compute cumulative arc length + local angle
  let totalArc = 0;
  const processed = pts.map((p, i) => {
    if (i === 0) return { ...p, s: 0, angle: 0, dydx: 0 };
    const dx = p.x - pts[i - 1].x;
    const dy = p.y - pts[i - 1].y;
    const seg = Math.sqrt(dx * dx + dy * dy);
    totalArc += seg;
    return { ...p, s: totalArc, angle: Math.atan2(dy, dx), dydx: seg > 1e-9 ? dy / seg : 0 };
  });

  // Mark where chain ends (peak of hill)
  let chainEndS = 0;
  let maxH = 0;
  for (const p of processed) {
    if (p.y > maxH) { maxH = p.y; chainEndS = p.s; }
    if (p.x > 30) break; // only look within the hill
  }

  return { points: processed, totalArc, chainEndS };
}

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
  PHYSICS — energy conservation + friction
  Chain drags at constant speed to hill peak,
  then cart is FREE and uses gravity/friction.
──────────────────────────────────────────────*/
function physicsStep(state, track, params, dt) {
  const pos = interpAt(track, state.s);
  const { gravity: g, friction: mu, mass: m, loopRadius: R } = params;

  // CHAIN SECTION: constant speed to the peak
  if (state.s <= track.chainEndS) {
    const cs = params.initialVelocity / 3.6;
    const newS = state.s + cs * dt;
    return { s: Math.min(newS, track.chainEndS + 0.01), v: cs, wF: 0, onChain: true };
  }

  // FREE SECTION: Newton's 2nd law along the track
  const v = state.v;
  const angle = pos.angle;
  const sinA = Math.sin(angle);
  const cosA = Math.cos(angle);

  // Determine normal force
  let N_force;
  const inLoop = pos.x > 44 && pos.x < 56 && pos.y > 0.5;
  if (inLoop) {
    // In the loop, centripetal acceleration contributes to normal force
    // N = mv²/R + mg·cos(α) when going around (α measured from track)
    // But sign depends on position in loop
    const centripetal = (v * v) / R;
    // At top of loop: N = m(v²/R - g), at bottom: N = m(v²/R + g)
    // General: use the angle to determine gravity component toward center
    N_force = m * centripetal - m * g * cosA;
  } else {
    // Normal track: N = mg·cos(slope)
    N_force = m * g * Math.abs(cosA);
  }

  // Friction force (always opposes motion)
  const frictionAccel = mu * Math.abs(N_force) / m;
  const frictionDir = v > 0 ? -1 : v < 0 ? 1 : 0;

  // Gravity component along track (negative sinA means gravity aids motion going downhill)
  const gravAccel = -g * sinA;

  // Total acceleration
  const a = gravAccel + frictionDir * frictionAccel;

  // Integrate (Velocity Verlet-like)
  let newV = v + a * dt;
  const newS = state.s + v * dt + 0.5 * a * dt * dt;

  // Clamp minimum speed to prevent getting stuck
  if (newV < 0.01) newV = 0.01;

  const dsFriction = Math.abs(v * dt + 0.5 * a * dt * dt);
  const newWF = state.wF + mu * Math.abs(N_force) * dsFriction / m * m; // = mu * |N| * ds

  return { s: Math.max(track.chainEndS, newS), v: newV, wF: newWF, onChain: false };
}

/*──────────────────────────────────────────────
  CART SVG
──────────────────────────────────────────────*/
function CartSVG({ cx, cy, angle, scale, v }) {
  const deg = (-angle * 180) / Math.PI;
  const s = scale;
  return (
    <g transform={`translate(${cx},${cy}) rotate(${deg})`}>
      <rect x={-16*s} y={-14*s} width={32*s} height={12*s} rx={3*s} fill="url(#cartBody)" stroke="#611" strokeWidth={0.6}/>
      <rect x={-14*s} y={-13*s} width={10*s} height={5*s} rx={1.5*s} fill="#0a2040" stroke="#2668aa" strokeWidth={0.3} opacity={0.85}/>
      <rect x={2*s} y={-13*s} width={10*s} height={5*s} rx={1.5*s} fill="#0a2040" stroke="#2668aa" strokeWidth={0.3} opacity={0.85}/>
      <rect x={-17*s} y={-3*s} width={34*s} height={3*s} rx={1*s} fill="#1a1a1a"/>
      <circle cx={-10*s} cy={1.5*s} r={2.8*s} fill="#222" stroke="#555" strokeWidth={0.7}/>
      <circle cx={10*s} cy={1.5*s} r={2.8*s} fill="#222" stroke="#555" strokeWidth={0.7}/>
      <circle cx={-10*s} cy={1.5*s} r={1.2*s} fill="#777"/>
      <circle cx={10*s} cy={1.5*s} r={1.2*s} fill="#777"/>
      <circle cx={-7*s} cy={-17*s} r={3.2*s} fill="#f0c89a" stroke="#c09060" strokeWidth={0.4}/>
      <circle cx={5*s} cy={-17*s} r={3.2*s} fill="#e8b888" stroke="#c09060" strokeWidth={0.4}/>
      <path d={`M${-10.5*s},${-19*s} Q${-7*s},${-22*s} ${-3.5*s},${-19*s}`} fill="#3a1a0a"/>
      <path d={`M${1.8*s},${-19*s} Q${5*s},${-23*s} ${8.2*s},${-19*s}`} fill="#111"/>
      <circle cx={-8.2*s} cy={-17.5*s} r={0.55*s} fill="#222"/>
      <circle cx={-5.8*s} cy={-17.5*s} r={0.55*s} fill="#222"/>
      <circle cx={3.8*s} cy={-17.5*s} r={0.55*s} fill="#222"/>
      <circle cx={6.2*s} cy={-17.5*s} r={0.55*s} fill="#222"/>
      <circle cx={14*s} cy={-9*s} r={1.8*s} fill={v>8?"#ff8":"#aa8"} opacity={v>8?0.9:0.4}/>
      {v>8 && <circle cx={14*s} cy={-9*s} r={3.5*s} fill="#ff8" opacity={0.12}/>}
    </g>
  );
}

/*──────────────────────────────────────────────
  MAIN COMPONENT
──────────────────────────────────────────────*/
export default function App() {
  const [params, setParams] = useState({
    mass: 1900, gravity: 9.81, friction: 0.02,
    hillHeight: 24, loopRadius: 6, initialVelocity: 7,
  });
  const [sim, setSim] = useState({ s: 0, v: 0, wF: 0, onChain: true });
  const [running, setRunning] = useState(false);
  const [done, setDone] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [panel, setPanel] = useState(true);
  const raf = useRef(null);
  const lt = useRef(null);
  const sr = useRef(sim);

  const track = useMemo(() => buildTrack(params), [params.hillHeight, params.loopRadius]);

  const reset = useCallback(() => {
    const init = { s: 0, v: params.initialVelocity / 3.6, wF: 0, onChain: true };
    setSim(init); sr.current = init;
    setRunning(false); setDone(false); lt.current = null;
    if (raf.current) cancelAnimationFrame(raf.current);
  }, [params.initialVelocity]);

  useEffect(() => { reset(); }, [params.hillHeight, params.loopRadius, reset]);

  useEffect(() => {
    if (!running || !track) return;
    const animate = (t) => {
      if (!lt.current) lt.current = t;
      const rawDt = ((t - lt.current) / 1000) * speed;
      lt.current = t;
      const dt = Math.min(rawDt, 0.03);
      let st = { ...sr.current };
      // Sub-stepping for stability
      const steps = 10;
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

  // Derived values
  const pos = interpAt(track, sim.s);
  const h = pos.y;
  const v = sim.v;
  const m = params.mass;
  const g = params.gravity;
  const KE = 0.5 * m * v * v;
  const PE = m * g * h;
  const maxE = m * g * params.hillHeight + 0.5 * m * (params.initialVelocity / 3.6) ** 2 + 1;

  // G-Force calculation
  const gForce = useMemo(() => {
    const idx = pos.idx || 0;
    const pts = track.points;
    if (idx < 2 || idx >= pts.length - 2 || v < 0.5) return 1;
    const p0 = pts[idx - 1], p1 = pts[idx], p2 = pts[idx + 1];
    const dx1 = p1.x - p0.x, dy1 = p1.y - p0.y, dx2 = p2.x - p1.x, dy2 = p2.y - p1.y;
    const ds1 = Math.sqrt(dx1*dx1+dy1*dy1), ds2 = Math.sqrt(dx2*dx2+dy2*dy2);
    if (ds1 < 1e-8 || ds2 < 1e-8) return 1;
    const dAngle = Math.atan2(dy2,dx2) - Math.atan2(dy1,dx1);
    const curvature = dAngle / ((ds1+ds2)/2);
    const ac = v * v * curvature;
    return (ac + g * Math.cos(pos.angle)) / g;
  }, [pos, v, track, g]);

  let section = "➡ Cadena";
  if (pos.x > 12 && pos.x <= 24) section = "⛰ Lift Hill";
  else if (pos.x > 24 && pos.x <= 36) section = "🔽 First Drop";
  else if (pos.x > 36 && pos.x <= 50) section = "➡ Transición";
  else if (pos.x > 44 && pos.x <= 56 && pos.y > 0.5) section = "🔄 Looping";
  else if (pos.x > 56 && pos.x <= 80) section = "🐫 Camelback";
  else if (pos.x > 80 && pos.x <= 106) section = "🐇 Bunny Hops";
  else if (pos.x > 106) section = "🛑 Frenado";
  if (done) section = "✅ ¡Recorrido completado!";

  // SVG coordinates
  const W = 860, H_ = 430, pL = 30, pR = 10, pT = 25, pB = 30;
  const pW = W - pL - pR, pH_ = H_ - pT - pB;
  const MX = 122, MY = 34;
  const sx = (x) => pL + (x / MX) * pW;
  const sy = (y) => pT + pH_ - (y / MY) * pH_;

  const trackPath = track.points.map((p, i) => `${i === 0 ? "M" : "L"}${sx(p.x).toFixed(2)},${sy(p.y).toFixed(2)}`).join(" ");

  const ties = useMemo(() => {
    const t = [];
    for (let i = 0; i < track.points.length; i += 8) {
      const p = track.points[i], a = p.angle + Math.PI / 2, len = 3.5;
      t.push({ x1: sx(p.x)+Math.cos(a)*len, y1: sy(p.y)-Math.sin(a)*len, x2: sx(p.x)-Math.cos(a)*len, y2: sy(p.y)+Math.sin(a)*len });
    }
    return t;
  }, [track]);

  const supports = useMemo(() => {
    const s = [];
    for (let xc = 3; xc <= 115; xc += 2.5) {
      const pt = track.points.reduce((b, p) => (Math.abs(p.x - xc) < Math.abs(b.x - xc) ? p : b));
      if (pt.y > 0.8) s.push({ x: pt.x, y: pt.y });
    }
    return s;
  }, [track]);

  const stars = useMemo(() => Array.from({length: 70}, () => ({
    cx: Math.random()*W, cy: Math.random()*(H_*0.4), r: Math.random()*1+0.2, o: Math.random()*0.5+0.2
  })), []);

  const updateParam = (k, val) => {
    setParams(p => ({ ...p, [k]: val }));
    if (k !== "hillHeight" && k !== "loopRadius") {
      const init = { s: 0, v: (k === "initialVelocity" ? val : params.initialVelocity) / 3.6, wF: 0, onChain: true };
      setSim(init); sr.current = init; setRunning(false); setDone(false);
    }
  };

  const btn = { border: "none", borderRadius: 8, fontSize: 13, fontWeight: 600, cursor: "pointer" };

  // Percentage through track
  const progress = (sim.s / track.totalArc) * 100;

  return (
    <div style={{ background: "#06060e", minHeight: "100vh", fontFamily: "'Inter',system-ui,sans-serif", color: "#ddd" }}>
      {/* Header */}
      <div style={{ padding: "10px 14px 4px", display: "flex", alignItems: "center", justifyContent: "space-between", flexWrap: "wrap" }}>
        <div>
          <h1 style={{ margin: 0, fontSize: 17, fontWeight: 800, background: "linear-gradient(90deg,#ff4466,#ffaa33,#44aaff)", WebkitBackgroundClip: "text", WebkitTextFillColor: "transparent" }}>
            🎢 Simulador Montaña Rusa
          </h1>
          <p style={{ margin: 0, fontSize: 10, color: "#445" }}>Mecánica Clásica — UTP Ica 2026 — Grupo N°9</p>
        </div>
        <button onClick={() => setPanel(p => !p)} style={{ ...btn, background: "#12122a", color: "#7af", padding: "4px 10px", fontSize: 10, border: "1px solid #222" }}>
          {panel ? "Ocultar ⚙" : "⚙ Config"}
        </button>
      </div>

      <div style={{ display: "flex", flexWrap: "wrap", gap: 8, padding: "4px 12px 12px" }}>
        <div style={{ flex: "1 1 600px", minWidth: 0 }}>
          {/* Progress bar */}
          <div style={{ height: 3, background: "#111", borderRadius: 2, marginBottom: 4, overflow: "hidden" }}>
            <div style={{ height: "100%", width: `${progress}%`, background: done ? "#4a4" : "linear-gradient(90deg,#f44,#fa4,#4af)", transition: "width 80ms" }} />
          </div>

          {/* SVG Scene */}
          <div style={{ background: "#080814", borderRadius: 12, overflow: "hidden", border: "1px solid #181830" }}>
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

              {/* Distant hills silhouette */}
              <path d={`M0,${sy(-1)} Q${W*0.2},${sy(3)} ${W*0.35},${sy(-0.5)} Q${W*0.55},${sy(2)} ${W*0.75},${sy(-1)} Q${W*0.9},${sy(1.5)} ${W},${sy(-1.5)} V${H_} H0Z`} fill="#0c1810" opacity={0.5}/>

              {/* Ground */}
              <rect x={pL} y={sy(0)} width={pW} height={H_-sy(0)} fill="url(#gnd)"/>
              <line x1={pL} y1={sy(0)} x2={pL+pW} y2={sy(0)} stroke="#2a4a1a" strokeWidth={1.5}/>

              {/* Grid lines */}
              {[0,6,12,18,24,30].map(yv => <g key={yv}><line x1={pL} y1={sy(yv)} x2={pL+pW} y2={sy(yv)} stroke="#0d0d20" strokeWidth={0.3}/><text x={pL-4} y={sy(yv)+3} fill="#223" fontSize={6.5} textAnchor="end" fontFamily="monospace">{yv}</text></g>)}

              {/* Supports */}
              {supports.map((s,i) => (
                <g key={i}>
                  <line x1={sx(s.x)} y1={sy(s.y)+3} x2={sx(s.x)} y2={sy(0)} stroke="#282840" strokeWidth={1.3}/>
                  <rect x={sx(s.x)-2} y={sy(0)-1} width={4} height={2.5} rx={0.5} fill="#222238"/>
                </g>
              ))}

              {/* Rail ties */}
              {ties.map((t,i) => <line key={i} x1={t.x1} y1={t.y1} x2={t.x2} y2={t.y2} stroke="#3a3a55" strokeWidth={0.7}/>)}

              {/* Track glow */}
              <path d={trackPath} fill="none" stroke="#2244aa" strokeWidth={6} opacity={0.07}/>
              {/* Track main rail */}
              <path d={trackPath} fill="none" stroke="#667799" strokeWidth={3} strokeLinecap="round" strokeLinejoin="round"/>
              {/* Track highlight */}
              <path d={trackPath} fill="none" stroke="#99aacc" strokeWidth={0.6} opacity={0.35}/>

              {/* Section labels */}
              {[[6,1.5,"CADENA"],[20,params.hillHeight+3,"LIFT HILL"],[30,params.hillHeight+3,"FIRST DROP"],[50,params.loopRadius*2+3.5,"LOOP"],[68,14.5,"CAMELBACK"],[95,5,"BUNNY HOPS"],[112,1.5,"FRENO"]].map(([x,y,t],i) =>
                <text key={i} x={sx(x)} y={sy(y)} fill="#223344" fontSize={6} textAnchor="middle" fontWeight="700" letterSpacing={1} fontFamily="monospace">{t}</text>
              )}

              {/* Speed trail */}
              {v > 6 && Array.from({length: Math.min(Math.floor(v/3), 10)}).map((_,i) => {
                const off = (i+1)*3.5;
                return <circle key={i} cx={sx(pos.x)-Math.cos(pos.angle)*off} cy={sy(pos.y)+Math.sin(pos.angle)*off} r={1.8-i*0.14} fill="#ff4466" opacity={0.35-i*0.03}/>;
              })}

              {/* CART */}
              <CartSVG cx={sx(pos.x)} cy={sy(pos.y)} angle={pos.angle} scale={0.55} v={v}/>

              {/* HUD overlay */}
              <rect x={W-170} y={6} width={162} height={72} rx={8} fill="#000" opacity={0.55}/>
              <text x={W-160} y={23} fill="#ff6" fontSize={14} fontWeight="800" fontFamily="monospace">{v.toFixed(1)} <tspan fontSize={8} fill="#aa8">m/s</tspan></text>
              <text x={W-160} y={38} fill="#6f6" fontSize={10} fontFamily="monospace">h = {h.toFixed(1)} m</text>
              <text x={W-160} y={52} fill="#f88" fontSize={10} fontFamily="monospace">{(v*3.6).toFixed(0)} km/h</text>
              <text x={W-160} y={66} fill={Math.abs(gForce)>2.5?"#f44":"#aaa"} fontSize={9} fontFamily="monospace">G: {gForce.toFixed(1)}g</text>
              <text x={W-80} y={23} fill={sim.onChain?"#fa4":"#4a4"} fontSize={8} fontFamily="monospace">{sim.onChain?"CADENA":"LIBRE"}</text>

              {/* Completion badge */}
              {done && (
                <g>
                  <rect x={W/2-80} y={H_/2-25} width={160} height={50} rx={10} fill="#000" opacity={0.7}/>
                  <text x={W/2} y={H_/2+5} fill="#4f4" fontSize={14} fontWeight="800" textAnchor="middle" fontFamily="monospace">✅ ¡COMPLETADO!</text>
                </g>
              )}
            </svg>
          </div>

          {/* Controls */}
          <div style={{ display: "flex", gap: 5, marginTop: 6, alignItems: "center", flexWrap: "wrap" }}>
            <button onClick={() => { if (done) reset(); else setRunning(r => !r); }}
              style={{ ...btn, padding: "8px 20px", color: "#fff",
                background: running ? "linear-gradient(135deg,#b22,#811)" : done ? "linear-gradient(135deg,#36a,#248)" : "linear-gradient(135deg,#1a8a3a,#146a28)",
                boxShadow: `0 0 12px ${running?"#b224":done?"#36a4":"#1a8a3a44"}` }}>
              {done ? "⟲ Reiniciar" : running ? "⏸ Pausar" : "▶ Iniciar"}
            </button>
            <button onClick={reset} style={{ ...btn, padding: "8px 14px", background: "#181828", color: "#888" }}>⟲</button>
            <div style={{ display: "flex", gap: 2, marginLeft: 6, background: "#0a0a18", borderRadius: 6, padding: 2 }}>
              {[0.25,0.5,1,2,3].map(s => (
                <button key={s} onClick={() => setSpeed(s)}
                  style={{ ...btn, padding: "3px 9px", fontSize: 10, background: speed===s?"#2a4a6a":"transparent", color: speed===s?"#fff":"#556", borderRadius: 4 }}>{s}x</button>
              ))}
            </div>
            <div style={{ marginLeft: "auto", fontSize: 11, color: done?"#4f4":"#7af", fontWeight: 600, padding: "3px 10px", background: "#0d0d20", borderRadius: 6, border: "1px solid #1a1a30" }}>{section}</div>
          </div>

          {/* Data cards */}
          <div style={{ display: "grid", gridTemplateColumns: "repeat(auto-fit, minmax(100px, 1fr))", gap: 4, marginTop: 6 }}>
            {[
              {l:"Velocidad",v:`${v.toFixed(1)}`,u:"m/s",s:`${(v*3.6).toFixed(0)} km/h`,c:"#ff6"},
              {l:"Altura",v:`${h.toFixed(1)}`,u:"m",c:"#6f6"},
              {l:"Ec",v:`${(KE/1000).toFixed(1)}`,u:"kJ",c:"#f55"},
              {l:"Ep",v:`${(PE/1000).toFixed(1)}`,u:"kJ",c:"#4af"},
              {l:"E Mecánica",v:`${((KE+PE)/1000).toFixed(1)}`,u:"kJ",c:"#fa6"},
              {l:"W Fricción",v:`${(sim.wF/1000).toFixed(2)}`,u:"kJ",c:"#a6a"},
              {l:"G-Force",v:`${gForce.toFixed(1)}`,u:"g",c:Math.abs(gForce)>2.5?"#f44":"#aaa"},
            ].map(d => (
              <div key={d.l} style={{ background: "#0a0a16", borderRadius: 7, padding: "5px 7px", border: "1px solid #141428" }}>
                <div style={{ fontSize: 8, color: "#445", textTransform: "uppercase", letterSpacing: 0.5 }}>{d.l}</div>
                <div style={{ fontSize: 15, fontWeight: 800, color: d.c, fontFamily: "monospace" }}>{d.v}<span style={{ fontSize: 8, color: "#445", marginLeft: 2 }}>{d.u}</span></div>
                {d.s && <div style={{ fontSize: 9, color: "#445" }}>{d.s}</div>}
              </div>
            ))}
          </div>

          {/* Energy bar */}
          <div style={{ marginTop: 5, padding: "6px 9px", background: "#0a0a14", borderRadius: 7, border: "1px solid #141428" }}>
            <div style={{ fontSize: 8, color: "#334", textTransform: "uppercase", letterSpacing: 1, marginBottom: 2 }}>Energía</div>
            <div style={{ height: 12, borderRadius: 6, overflow: "hidden", display: "flex", background: "#060610" }}>
              <div style={{ width: `${(KE/maxE)*100}%`, background: "linear-gradient(90deg,#a11,#f44)", transition: "width 80ms" }}/>
              <div style={{ width: `${(PE/maxE)*100}%`, background: "linear-gradient(90deg,#1a4a8a,#48f)", transition: "width 80ms" }}/>
              <div style={{ width: `${(sim.wF/maxE)*100}%`, background: "linear-gradient(90deg,#636,#a4a)", transition: "width 80ms" }}/>
            </div>
            <div style={{ display: "flex", gap: 12, marginTop: 2, fontSize: 8, color: "#556" }}>
              <span><span style={{ color: "#f55" }}>●</span> Cinética</span>
              <span><span style={{ color: "#4af" }}>●</span> Potencial</span>
              <span><span style={{ color: "#a6a" }}>●</span> Fricción</span>
            </div>
          </div>
        </div>

        {/* Config Panel */}
        {panel && (
          <div style={{ flex: "0 0 195px", display: "flex", flexDirection: "column", gap: 5 }}>
            <div style={{ background: "#0a0a16", borderRadius: 9, padding: 10, border: "1px solid #141428" }}>
              <h3 style={{ margin: "0 0 6px", fontSize: 11, color: "#8af", fontWeight: 700 }}>⚙ Parámetros</h3>
              {[
                {k:"mass",l:"Masa",min:100,max:5000,step:50,u:"kg"},
                {k:"gravity",l:"Gravedad",min:0.5,max:30,step:0.1,u:"m/s²"},
                {k:"friction",l:"Rozamiento μ",min:0,max:0.2,step:0.005,u:""},
                {k:"initialVelocity",l:"V₀ cadena",min:1,max:30,step:0.5,u:"km/h"},
                {k:"hillHeight",l:"Altura colina",min:10,max:40,step:1,u:"m"},
                {k:"loopRadius",l:"Radio loop",min:3,max:10,step:0.5,u:"m"},
              ].map(p => (
                <div key={p.k} style={{ marginBottom: 7 }}>
                  <div style={{ display: "flex", justifyContent: "space-between", fontSize: 9, color: "#556" }}>
                    <span>{p.l}</span><span style={{ color: "#99a", fontFamily: "monospace", fontWeight: 600 }}>{params[p.k]}{p.u}</span>
                  </div>
                  <input type="range" min={p.min} max={p.max} step={p.step} value={params[p.k]}
                    onChange={e => updateParam(p.k, parseFloat(e.target.value))}
                    style={{ width: "100%", accentColor: "#4466aa", height: 3 }}/>
                </div>
              ))}
            </div>
            <div style={{ background: "#0a0a16", borderRadius: 9, padding: 10, border: "1px solid #141428" }}>
              <h3 style={{ margin: "0 0 5px", fontSize: 11, color: "#6f6", fontWeight: 700 }}>🌍 Gravedad</h3>
              {[{n:"🌎 Tierra",g:9.81},{n:"🌙 Luna",g:1.62},{n:"🔴 Marte",g:3.72},{n:"🪐 Júpiter",g:24.79}].map(pr => (
                <button key={pr.n} onClick={() => updateParam("gravity",pr.g)}
                  style={{ ...btn, display:"block", width:"100%", marginBottom:2, fontSize:10, padding:"4px 7px", textAlign:"left",
                    background: params.gravity===pr.g?"#12201a":"#08080f", color: params.gravity===pr.g?"#6f6":"#556",
                    border:`1px solid ${params.gravity===pr.g?"#1a3a20":"#141428"}` }}>
                  {pr.n} <span style={{ float:"right", fontFamily:"monospace", fontSize:9 }}>{pr.g}</span>
                </button>
              ))}
            </div>
            <div style={{ background: "#0a0a16", borderRadius: 9, padding: 10, border: "1px solid #141428" }}>
              <h3 style={{ margin: "0 0 4px", fontSize: 11, color: "#fa6", fontWeight: 700 }}>📐 Modelo</h3>
              <div style={{ fontSize: 9, color: "#556", lineHeight: 1.9, fontFamily: "monospace" }}>
                Ec = ½mv²<br/>Ep = mgh<br/>Wf = μ·N·d<br/>
                Ei = Ef + Wf<br/>
                <span style={{color:"#445"}}>v_min = √(g·r)</span><br/>
                <span style={{color:"#445"}}>N_loop = mv²/R ± mg</span>
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}