import { useRef, useMemo, useState, useEffect, useCallback } from 'react';
import { Canvas, useFrame, useLoader } from '@react-three/fiber';
import { Text, Billboard } from '@react-three/drei';
import * as THREE from 'three';

const ExpandIcon = () => (
  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <polyline points="15 3 21 3 21 9" />
    <polyline points="9 21 3 21 3 15" />
    <line x1="21" y1="3" x2="14" y2="10" />
    <line x1="3" y1="21" x2="10" y2="14" />
  </svg>
);

const CollapseIcon = () => (
  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <polyline points="4 14 10 14 10 20" />
    <polyline points="20 10 14 10 14 4" />
    <line x1="14" y1="10" x2="21" y2="3" />
    <line x1="3" y1="21" x2="10" y2="14" />
  </svg>
);

function AxisGrid({ isDark }) {
  const ticks = [-1, -0.75, -0.5, -0.25, 0, 0.25, 0.5, 0.75, 1];
  const lineColor = isDark ? '#ffffff' : '#000000';
  const lineOpacity = isDark ? 0.35 : 0.25;
  const tickColor = isDark ? '#ddd' : '#333';

  const lines = useMemo(() => {
    const segs = [];
    for (const v of ticks) {
      segs.push(-1, -1, v, 1, -1, v);
      segs.push(v, -1, -1, v, -1, 1);
    }
    for (const v of ticks) {
      segs.push(-1, -1, v, -1, 1, v);
      segs.push(-1, v, -1, -1, v, 1);
    }
    for (const v of ticks) {
      segs.push(v, -1, 1, v, 1, 1);
      segs.push(-1, v, 1, 1, v, 1);
    }
    const arr = new Float32Array(segs);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(arr, 3));
    return geo;
  }, []);

  return (
    <group>
      <lineSegments geometry={lines}>
        <lineBasicMaterial color={lineColor} opacity={lineOpacity} transparent />
      </lineSegments>
      {[-1, -0.5, 0, 0.5, 1].map(v => (
        <Billboard key={`x${v}`} position={[v, -1.15, -1.15]}>
          <Text fontSize={0.09} color={tickColor} anchorX="center">{v.toFixed(1)}</Text>
        </Billboard>
      ))}
      {[-1, -0.5, 0, 0.5, 1].map(v => (
        <Billboard key={`y${v}`} position={[1.15, -1.15, v]}>
          <Text fontSize={0.09} color={tickColor} anchorX="center">{v.toFixed(1)}</Text>
        </Billboard>
      ))}
      {[-1, -0.5, 0, 0.5, 1].map(v => (
        <Billboard key={`z${v}`} position={[1.12, v, 1]}>
          <Text fontSize={0.09} color={tickColor} anchorX="right">{v.toFixed(1)}</Text>
        </Billboard>
      ))}
    </group>
  );
}

/* ---------- Realistic sounding-rocket geometry ---------- */

const BODY_R = 0.08;   // main body radius
const BODY_SEG = 48;   // cylinder segments

/** Tangent-ogive nose cone (tip up, base down connecting to body) */
function NoseCone() {
  const profile = useMemo(() => {
    const pts = [];
    const len = 0.28;
    const steps = 32;
    for (let i = 0; i <= steps; i++) {
      const t = i / steps; // 0 = base (wide), 1 = tip (point)
      const r = BODY_R * Math.sqrt(1 - Math.pow(t, 2.5));
      pts.push(new THREE.Vector2(r, t * len));
    }
    return pts;
  }, []);

  return (
    <mesh position={[0, 0.56, 0]}>
      <latheGeometry args={[profile, BODY_SEG]} />
      <meshStandardMaterial color="#f0f0f0" roughness={0.2} metalness={0.55} />
    </mesh>
  );
}

/** Clipped-delta fin using BoxGeometry + shear (solid, visible) */
function Fin({ angle }) {
  const rad = (angle * Math.PI) / 180;
  // Use a box for each fin, sheared into a trapezoid via vertex manipulation
  const geo = useMemo(() => {
    // Dimensions: the fin is a box that we reshape
    const rootLen = 0.32;    // root chord (along Y / body axis)
    const tipLen = 0.10;     // tip chord
    const span = 0.22;       // how far it sticks out (radial, along Z here)
    const thickness = 0.016; // X thickness
    const sweepBack = 0.16;  // tip leading edge set back

    // Build as 8-corner prism, then form triangulated faces
    // Corners: root LE/TE at inner (Z=0) and outer (Z=span)
    const rLE_i = [0, 0, 0];                        // root leading edge, inner
    const rTE_i = [0, -rootLen, 0];                  // root trailing edge, inner
    const tLE_o = [0, -sweepBack, span];             // tip leading edge, outer
    const tTE_o = [0, -sweepBack - tipLen, span];    // tip trailing edge, outer

    // Extrude along X for thickness
    const ht = thickness / 2;
    const faces = [];

    function addQuad(a, b, c, d) {
      faces.push(...a, ...b, ...c, ...a, ...c, ...d);
    }

    // +X face
    const rLE_p = [ht, rLE_i[1], rLE_i[2]];
    const rTE_p = [ht, rTE_i[1], rTE_i[2]];
    const tLE_p = [ht, tLE_o[1], tLE_o[2]];
    const tTE_p = [ht, tTE_o[1], tTE_o[2]];
    // -X face
    const rLE_n = [-ht, rLE_i[1], rLE_i[2]];
    const rTE_n = [-ht, rTE_i[1], rTE_i[2]];
    const tLE_n = [-ht, tLE_o[1], tLE_o[2]];
    const tTE_n = [-ht, tTE_o[1], tTE_o[2]];

    addQuad(rLE_p, rTE_p, tTE_p, tLE_p); // +X
    addQuad(rLE_n, tLE_n, tTE_n, rTE_n); // -X
    addQuad(rLE_p, tLE_p, tLE_n, rLE_n); // leading edge
    addQuad(rTE_n, tTE_n, tTE_p, rTE_p); // trailing edge
    addQuad(tLE_p, tTE_p, tTE_n, tLE_n); // tip
    addQuad(rLE_n, rTE_n, rTE_p, rLE_p); // root

    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(faces), 3));
    g.computeVertexNormals();
    return g;
  }, []);

  return (
    <mesh geometry={geo} position={[0, -0.46, 0]} rotation={[0, rad, 0]}>
      <meshStandardMaterial color="#8c8c8c" roughness={0.3} metalness={0.5} side={THREE.DoubleSide} />
    </mesh>
  );
}

/** Flag decal — a small plane with flag texture curved onto the body */
function FlagDecal({ y }) {
  const texture = useLoader(THREE.TextureLoader, '/flag-uzbekistan.png');
  const geo = useMemo(() => {
    // Create a curved plane that wraps around the cylinder surface
    const arcAngle = Math.PI * 0.55; // 55% of circumference
    const halfArc = arcAngle / 2;
    const segsX = 16, segsY = 1;
    const height = 0.09;
    const r = BODY_R + 0.002; // slightly above body surface

    const positions = [];
    const uvs = [];
    const indices = [];

    for (let iy = 0; iy <= segsY; iy++) {
      for (let ix = 0; ix <= segsX; ix++) {
        const u = ix / segsX;
        const v = iy / segsY;
        const angle = -halfArc + u * arcAngle;
        positions.push(Math.sin(angle) * r, (v - 0.5) * height, Math.cos(angle) * r);
        uvs.push(u, v);
      }
    }
    for (let iy = 0; iy < segsY; iy++) {
      for (let ix = 0; ix < segsX; ix++) {
        const a = iy * (segsX + 1) + ix;
        const b = a + 1;
        const c = a + (segsX + 1);
        const d = c + 1;
        indices.push(a, b, c, b, d, c);
      }
    }

    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3));
    g.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(uvs), 2));
    g.setIndex(indices);
    g.computeVertexNormals();
    return g;
  }, []);

  return (
    <mesh geometry={geo} position={[0, y, 0]} rotation={[0, (135 * Math.PI) / 180, 0]}>
      <meshStandardMaterial map={texture} transparent roughness={0.4} metalness={0.2} />
    </mesh>
  );
}

/** Separation band ring */
function Band({ y, color = '#555', width = 0.012 }) {
  return (
    <mesh position={[0, y, 0]}>
      <cylinderGeometry args={[BODY_R + 0.003, BODY_R + 0.003, width, BODY_SEG]} />
      <meshStandardMaterial color={color} roughness={0.45} metalness={0.65} />
    </mesh>
  );
}

/** Boat-tail (aft taper) using lathe */
function BoatTail() {
  const profile = useMemo(() => {
    const pts = [];
    const steps = 16;
    const topR = BODY_R, botR = 0.055, len = 0.14;
    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      // smooth cubic taper
      const r = topR + (botR - topR) * (3*t*t - 2*t*t*t);
      pts.push(new THREE.Vector2(r, t * len));
    }
    return pts;
  }, []);

  return (
    <mesh position={[0, -0.58, 0]} rotation={[Math.PI, 0, 0]}>
      <latheGeometry args={[profile, BODY_SEG]} />
      <meshStandardMaterial color="#b8b8b8" roughness={0.3} metalness={0.5} />
    </mesh>
  );
}

function Missile({ roll, pitch, yaw }) {
  const ref = useRef();
  const currentQuat = useRef(new THREE.Quaternion());
  const targetQuat = useRef(new THREE.Quaternion());
  const euler = useRef(new THREE.Euler());

  useFrame(() => {
    if (!ref.current) return;
    euler.current.set(
      (pitch * Math.PI) / 180,
      (yaw * Math.PI) / 180,
      (roll * Math.PI) / 180,
      'XYZ'
    );
    targetQuat.current.setFromEuler(euler.current);
    currentQuat.current.slerp(targetQuat.current, 0.1);
    ref.current.quaternion.copy(currentQuat.current);
  });

  return (
    <group ref={ref} position={[0, 0, 0]}>
      {/* Nose cone */}
      <NoseCone />

      {/* Nose-body band */}
      <Band y={0.56} color="#888" />

      {/* Payload section */}
      <mesh position={[0, 0.38, 0]}>
        <cylinderGeometry args={[BODY_R, BODY_R, 0.36, BODY_SEG]} />
        <meshStandardMaterial color="#e8e8e8" roughness={0.25} metalness={0.45} />
      </mesh>

      {/* Uzbekistan flag on payload section */}
      <FlagDecal y={0.44} />

      {/* Sensor window band (dark) */}
      <mesh position={[0, 0.26, 0]}>
        <cylinderGeometry args={[BODY_R + 0.001, BODY_R + 0.001, 0.025, BODY_SEG]} />
        <meshStandardMaterial color="#1a1a2e" roughness={0.15} metalness={0.7} />
      </mesh>

      {/* Primary body marking - red stripe */}
      <mesh position={[0, 0.32, 0]}>
        <cylinderGeometry args={[BODY_R + 0.001, BODY_R + 0.001, 0.022, BODY_SEG]} />
        <meshStandardMaterial color="#cc2233" roughness={0.3} metalness={0.35} />
      </mesh>

      {/* Mid separation band */}
      <Band y={0.20} color="#666" />

      {/* Main body tube */}
      <mesh position={[0, 0.0, 0]}>
        <cylinderGeometry args={[BODY_R, BODY_R, 0.40, BODY_SEG]} />
        <meshStandardMaterial color="#d8d8d8" roughness={0.28} metalness={0.42} />
      </mesh>

      {/* Secondary marking - thin green stripe */}
      <mesh position={[0, -0.05, 0]}>
        <cylinderGeometry args={[BODY_R + 0.001, BODY_R + 0.001, 0.012, BODY_SEG]} />
        <meshStandardMaterial color="#2d6a4f" roughness={0.3} metalness={0.35} />
      </mesh>

      {/* Aft band */}
      <Band y={-0.20} color="#666" />

      {/* Motor section */}
      <mesh position={[0, -0.39, 0]}>
        <cylinderGeometry args={[BODY_R, BODY_R, 0.38, BODY_SEG]} />
        <meshStandardMaterial color="#c0c0c0" roughness={0.32} metalness={0.48} />
      </mesh>

      {/* Aft-motor band */}
      <Band y={-0.58} color="#555" width={0.016} />

      {/* Boat tail */}
      <BoatTail />

      {/* Combustion chamber — wider bulge */}
      <mesh position={[0, -0.74, 0]}>
        <cylinderGeometry args={[0.058, 0.058, 0.04, 32]} />
        <meshStandardMaterial color="#D4A844" roughness={0.3} metalness={0.7} />
      </mesh>

      {/* Chamber top cap ring */}
      <mesh position={[0, -0.718, 0]}>
        <cylinderGeometry args={[0.060, 0.054, 0.008, 32]} />
        <meshStandardMaterial color="#D4A844" roughness={0.25} metalness={0.75} />
      </mesh>

      {/* Chamber bottom cap ring */}
      <mesh position={[0, -0.762, 0]}>
        <cylinderGeometry args={[0.054, 0.060, 0.008, 32]} />
        <meshStandardMaterial color="#D4A844" roughness={0.25} metalness={0.75} />
      </mesh>

      {/* Nozzle — converging throat */}
      <mesh position={[0, -0.785, 0]}>
        <cylinderGeometry args={[0.050, 0.032, 0.04, 32]} />
        <meshStandardMaterial color="#D4A844" roughness={0.3} metalness={0.65} />
      </mesh>

      {/* Nozzle — diverging bell */}
      <mesh position={[0, -0.815, 0]}>
        <cylinderGeometry args={[0.032, 0.048, 0.03, 32]} />
        <meshStandardMaterial color="#D4A844" roughness={0.3} metalness={0.65} />
      </mesh>

      {/* Nozzle exit rim */}
      <mesh position={[0, -0.832, 0]}>
        <cylinderGeometry args={[0.048, 0.050, 0.006, 32]} />
        <meshStandardMaterial color="#D4A844" roughness={0.25} metalness={0.75} />
      </mesh>

      {/* 4 swept fins */}
      <Fin angle={0} />
      <Fin angle={90} />
      <Fin angle={180} />
      <Fin angle={270} />
    </group>
  );
}

function RocketScene({ roll, pitch, yaw, isDark }) {
  return (
    <Canvas
      camera={{ position: [3.6, 0.5, -3.6], fov: 38 }}
      style={{ pointerEvents: 'none' }}
    >
      <ambientLight intensity={0.55} />
      <directionalLight position={[4, 6, -4]} intensity={1.3} />
      <directionalLight position={[-3, 2, 3]} intensity={0.35} />
      <directionalLight position={[0, -4, 0]} intensity={0.12} />
      <group position={[-0.15, 0.15, 0]} rotation={[0, (10 * Math.PI) / 180, 0]}>
        <Missile roll={roll} pitch={pitch} yaw={yaw} />
        <AxisGrid isDark={isDark} />
      </group>
    </Canvas>
  );
}

export default function Rocket3D({ roll, pitch, yaw, theme, standalone }) {
  const isDark = theme !== 'light';

  const [expanded, setExpanded] = useState(false);

  useEffect(() => {
    if (!window.go?.backend?.App?.IsWindowOpen) return;
    window.go.backend.App.IsWindowOpen('3d').then(setExpanded).catch(() => {});
    const poll = setInterval(() => {
      window.go.backend.App.IsWindowOpen('3d').then(setExpanded).catch(() => {});
    }, 2000);
    return () => clearInterval(poll);
  }, []);

  const toggleWindow = useCallback(async () => {
    if (window.go?.backend?.App?.ToggleWindow) {
      const isNowOpen = await window.go.backend.App.ToggleWindow('3d');
      setExpanded(isNowOpen);
    } else {
      window.open(`${window.location.origin}/#3d`, '3d-orientation', 'width=900,height=600');
    }
  }, []);

  if (standalone) {
    return (
      <div className="standalone-view">
        <div className="standalone-view-body rocket-canvas">
          <RocketScene roll={roll} pitch={pitch} yaw={yaw} isDark={isDark} />
        </div>
        <div className="standalone-view-footer">
          <span style={{ color: '#ff6b6b' }}>Roll: {roll.toFixed(1)}&deg;</span>
          <span style={{ color: '#4caf50' }}>Pitch: {pitch.toFixed(1)}&deg;</span>
          <span style={{ color: '#4a9eff' }}>Yaw: {yaw.toFixed(1)}&deg;</span>
        </div>
      </div>
    );
  }

  return (
    <div className="viz-card">
      <div className="viz-card-header">
        <h4>3D Orientation</h4>
        <button className={`viz-expand-btn${expanded ? ' expanded' : ''}`} onClick={toggleWindow} title={expanded ? 'Close window' : 'Open in new window'}>
          {expanded ? <CollapseIcon /> : <ExpandIcon />}
        </button>
      </div>
      <div className="viz-content rocket-canvas">
        <RocketScene roll={roll} pitch={pitch} yaw={yaw} isDark={isDark} />
      </div>
      <div className="orientation-values">
        <span style={{ color: '#ff6b6b' }}>Roll: {roll.toFixed(1)}&deg;</span>
        <span style={{ color: '#4caf50' }}>Pitch: {pitch.toFixed(1)}&deg;</span>
        <span style={{ color: '#4a9eff' }}>Yaw: {yaw.toFixed(1)}&deg;</span>
      </div>
    </div>
  );
}
