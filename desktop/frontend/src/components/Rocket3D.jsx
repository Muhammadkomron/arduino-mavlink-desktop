import { useRef, useMemo } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { Text, Billboard } from '@react-three/drei';
import * as THREE from 'three';

/*
 * Matching the reference image exactly:
 *   Camera: front-center, slightly elevated → looking at far corner
 *   Floor: y = -1, full grid
 *   Left wall: x = -1 (goes up behind-left)
 *   Right wall: z = 1 (goes up behind-right)
 *   Walls meet at far corner (-1, y, 1)
 *   X labels: bottom-left floor edge (z = -1 side)
 *   Y labels: bottom-right floor edge (x = 1 side)
 *   Z labels: right vertical edge (x = 1, z = 1)
 */

function AxisGrid({ isDark }) {
  const ticks = [-1, -0.75, -0.5, -0.25, 0, 0.25, 0.5, 0.75, 1];
  const lineColor = isDark ? '#ffffff' : '#000000';
  const lineOpacity = isDark ? 0.35 : 0.25;
  const tickColor = isDark ? '#ddd' : '#333';

  const lines = useMemo(() => {
    const segs = [];

    // Floor (y = -1)
    for (const v of ticks) {
      segs.push(-1, -1, v, 1, -1, v);
      segs.push(v, -1, -1, v, -1, 1);
    }
    // Left wall (x = -1)
    for (const v of ticks) {
      segs.push(-1, -1, v, -1, 1, v);
      segs.push(-1, v, -1, -1, v, 1);
    }
    // Right wall (z = 1)
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

      {/* X ticks — front floor edge (z = -1), show every 0.5 */}
      {[-1, -0.5, 0, 0.5, 1].map(v => (
        <Billboard key={`x${v}`} position={[v, -1.15, -1.15]}>
          <Text fontSize={0.09} color={tickColor} anchorX="center">{v.toFixed(1)}</Text>
        </Billboard>
      ))}
      {/* Y ticks — right floor edge (x = 1), show every 0.5 */}
      {[-1, -0.5, 0, 0.5, 1].map(v => (
        <Billboard key={`y${v}`} position={[1.15, -1.15, v]}>
          <Text fontSize={0.09} color={tickColor} anchorX="center">{v.toFixed(1)}</Text>
        </Billboard>
      ))}
      {/* Z ticks — right-back vertical edge (x=1, z=1) */}
      {[-1, -0.5, 0, 0.5, 1].map(v => (
        <Billboard key={`z${v}`} position={[1.12, v, 1]}>
          <Text fontSize={0.09} color={tickColor} anchorX="right">{v.toFixed(1)}</Text>
        </Billboard>
      ))}
    </group>
  );
}

function Fin({ angle }) {
  const geo = useMemo(() => {
    const vertices = new Float32Array([
      0, 0, 0.006, 0.18, 0, 0.006, 0, 0.35, 0.006,
      0, 0, -0.006, 0, 0.35, -0.006, 0.18, 0, -0.006,
    ]);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
    g.computeVertexNormals();
    return g;
  }, []);

  const rad = (angle * Math.PI) / 180;
  return (
    <mesh geometry={geo} position={[0, -0.70, 0]} rotation={[0, rad, 0]}>
      <meshStandardMaterial color="#5b9bd5" transparent opacity={0.75} roughness={0.4} metalness={0.3} side={THREE.DoubleSide} />
    </mesh>
  );
}

function Rocket({ roll, pitch, yaw }) {
  const ref = useRef();
  const target = useRef({ x: 0, y: 0, z: 0 });

  target.current = {
    x: (pitch * Math.PI) / 180,
    y: (yaw * Math.PI) / 180,
    z: (roll * Math.PI) / 180,
  };

  useFrame(() => {
    if (!ref.current) return;
    ref.current.rotation.x = THREE.MathUtils.lerp(ref.current.rotation.x, target.current.x, 0.08);
    ref.current.rotation.y = THREE.MathUtils.lerp(ref.current.rotation.y, target.current.y, 0.08);
    ref.current.rotation.z = THREE.MathUtils.lerp(ref.current.rotation.z, target.current.z, 0.08);
  });

  /*
   * Rocket: taller/thinner body, flared bottom base (wider at bottom).
   * Positioned so bottom of flare sits near floor (y ≈ -0.85).
   * Total span: nose tip ~0.85, flare bottom ~-0.85 → centered in box.
   */

  return (
    <group ref={ref} position={[0, -0.05, 0]}>
      {/* Nose cone — smaller and pointy */}
      <mesh position={[0, 0.62, 0]}>
        <coneGeometry args={[0.06, 0.40, 32]} />
        <meshStandardMaterial color="#5b9bd5" roughness={0.4} metalness={0.3} />
      </mesh>
      {/* Main body cylinder — taller and thinner */}
      <mesh position={[0, 0.02, 0]}>
        <cylinderGeometry args={[0.06, 0.06, 0.80, 32]} />
        <meshStandardMaterial color="#5b9bd5" roughness={0.4} metalness={0.3} />
      </mesh>
      {/* Bottom taper — body narrows slightly toward tail */}
      <mesh position={[0, -0.50, 0]}>
        <cylinderGeometry args={[0.06, 0.045, 0.25, 32]} />
        <meshStandardMaterial color="#5b9bd5" roughness={0.4} metalness={0.3} />
      </mesh>
      {/* Nozzle tip */}
      <mesh position={[0, -0.66, 0]}>
        <cylinderGeometry args={[0.045, 0.025, 0.08, 24]} />
        <meshStandardMaterial color="#3a7cc2" roughness={0.35} metalness={0.35} />
      </mesh>
      <Fin angle={0} />
      <Fin angle={90} />
      <Fin angle={180} />
      <Fin angle={270} />
    </group>
  );
}

export default function Rocket3D({ roll, pitch, yaw, theme }) {
  const isDark = theme !== 'light';
  return (
    <div className="viz-card">
      <div className="viz-card-header">
        <h4>3D Orientation</h4>
      </div>
      <div className="viz-content rocket-canvas">
        <Canvas
          camera={{ position: [3.2, -0.2, -3.2], fov: 38 }}
          style={{ pointerEvents: 'none' }}
        >
          <ambientLight intensity={0.7} />
          <directionalLight position={[3, 5, -3]} intensity={1} />
          <directionalLight position={[-3, 3, 2]} intensity={0.3} />
          <group position={[-0.15, 0.15, 0]} rotation={[0, (10 * Math.PI) / 180, 0]}>
            <Rocket roll={roll} pitch={pitch} yaw={yaw} />
            <AxisGrid isDark={isDark} />
          </group>
        </Canvas>
      </div>
      <div className="orientation-values">
        <span style={{ color: '#ff6b6b' }}>R: {roll.toFixed(1)}°</span>
        <span style={{ color: '#4caf50' }}>P: {pitch.toFixed(1)}°</span>
        <span style={{ color: '#4a9eff' }}>Y: {yaw.toFixed(1)}°</span>
      </div>
    </div>
  );
}
