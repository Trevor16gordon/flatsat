/**
 * The vehicle, as configured and as flying.
 *
 * Left: a live 3D scene built from the SAME vehicle file the flight
 * software runs. The satellite's components sit on their configured
 * mounting axes (translucent, so the internals read), the body is
 * oriented by the star-tracker attitude that crossed the radio, and it
 * rides its configured orbit around a small Earth. The orbit PHASE is
 * not downlinked — it is inferred from the attitude (the projection of
 * the instrument axis onto the orbit plane, slew-rate-limited), and
 * the pane says so: when the vehicle truly tracks nadir the cone
 * visibly looks at the Earth, and when it doesn't, it visibly doesn't.
 *
 * Right: the composition table. Click a row to spotlight that
 * component in the scene — components the config gives no geometry
 * (the flight computer, the IMU) light the bus itself, which is where
 * they live.
 */

import { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import type { MissionBlob } from '../data/types';

export interface VehicleInfo {
  name: string;
  mass_kg: number;
  strategy: string;
  estimator: string;
  rate_hz: number;
  orbit: { altitude_m?: number; inclination_deg?: number; raan_deg?: number };
  sensors: { name: string; kind: string; realness: string; rate_hz: number }[];
  actuators: {
    name: string;
    kind: string;
    realness: string;
    position_m: number[];
    axis: number[];
  }[];
  platform: { name: string; kind: string; realness: string }[];
}

interface Props {
  vehicle: VehicleInfo;
  blob: MissionBlob;
}

function last(blob: MissionBlob, channel: string): number | undefined {
  const s = blob.series[channel];
  return s && s.v.length > 0 ? s.v[s.v.length - 1] : undefined;
}

/** MRP sigma_BN -> body-to-world rotation. */
function mrpToQuaternion(sx: number, sy: number, sz: number): THREE.Quaternion {
  const s2 = sx * sx + sy * sy + sz * sz;
  const q = new THREE.Quaternion(
    (2 * sx) / (1 + s2),
    (2 * sy) / (1 + s2),
    (2 * sz) / (1 + s2),
    (1 - s2) / (1 + s2),
  );
  return q.invert();
}

const EARTH_R = 1.0; // scene units
const ORBIT_R = 1.55; // exaggerated altitude so the geometry reads

export function VehiclePanel({ vehicle, blob }: Props) {
  const mountRef = useRef<HTMLDivElement | null>(null);
  const blobRef = useRef(blob);
  blobRef.current = blob;
  const [selected, setSelected] = useState<string | null>(null);
  const selectedRef = useRef(selected);
  selectedRef.current = selected;

  useEffect(() => {
    const mount = mountRef.current;
    if (!mount) return;
    const width = mount.clientWidth || 380;
    const height = 300;
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    mount.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(38, width / height, 0.1, 100);
    camera.position.set(3.4, 2.2, 3.4);
    camera.lookAt(0, 0, 0);
    scene.add(new THREE.AmbientLight(0x334455, 0.5));
    const sun = new THREE.DirectionalLight(0xfff2dd, 2.8);
    sun.position.set(6, 3, 2);
    scene.add(sun);

    // ------------------------------------------------------------ Earth --
    const earthTexture = new THREE.TextureLoader().load('/earth.jpg');
    earthTexture.colorSpace = THREE.SRGBColorSpace;
    const earth = new THREE.Mesh(
      new THREE.SphereGeometry(EARTH_R, 64, 48),
      new THREE.MeshStandardMaterial({
        map: earthTexture,
        color: 0xffffff,
        roughness: 1.0,
        metalness: 0.0,
      }),
    );
    scene.add(earth);
    const atmosphere = new THREE.Mesh(
      new THREE.SphereGeometry(EARTH_R * 1.03, 48, 32),
      new THREE.MeshBasicMaterial({ color: 0x66aaff, transparent: true, opacity: 0.12 }),
    );
    scene.add(atmosphere);

    // Orbit ring from the CONFIG elements (inclination, raan).
    const inc = ((vehicle.orbit.inclination_deg ?? 0) * Math.PI) / 180;
    const raan = ((vehicle.orbit.raan_deg ?? 0) * Math.PI) / 180;
    const orbitFrame = new THREE.Group();
    orbitFrame.rotation.set(0, raan, 0);
    const incFrame = new THREE.Group();
    incFrame.rotation.set(inc, 0, 0);
    orbitFrame.add(incFrame);
    scene.add(orbitFrame);
    const ringPts: THREE.Vector3[] = [];
    for (let k = 0; k <= 128; k++) {
      const a = (k / 128) * Math.PI * 2;
      ringPts.push(new THREE.Vector3(Math.cos(a) * ORBIT_R, 0, Math.sin(a) * ORBIT_R));
    }
    incFrame.add(
      new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(ringPts),
        new THREE.LineBasicMaterial({ color: 0x44608a, transparent: true, opacity: 0.8 }),
      ),
    );

    // ------------------------------------------------------- spacecraft --
    const SAT_SCALE = 0.22;
    const carrier = new THREE.Group(); // position on the orbit
    incFrame.add(carrier);
    const body = new THREE.Group(); // attitude from downlink
    body.scale.setScalar(SAT_SCALE);
    carrier.add(body);

    const meshes = new Map<string, THREE.Mesh[]>();
    const track = (name: string, mesh: THREE.Mesh) => {
      const bucket = meshes.get(name) ?? [];
      bucket.push(mesh);
      meshes.set(name, bucket);
    };
    const material = (color: number, opacity = 0.55) =>
      new THREE.MeshStandardMaterial({
        color,
        transparent: true,
        opacity,
        metalness: 0.4,
        roughness: 0.45,
        depthWrite: false,
      });

    const bus = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.9, 0.9), material(0x3a4d63, 0.35));
    body.add(bus);
    track('bus', bus);
    track('flight computer', bus);
    track('imu0', bus);
    track('mag0', bus);
    track('thermal_tj', bus);

    const face = new THREE.Mesh(new THREE.ConeGeometry(0.17, 0.32, 24), material(0x4a90d9, 0.8));
    face.rotation.x = Math.PI / 2;
    face.position.set(0, 0, 0.6);
    body.add(face);
    track('css0', face);

    // Star tracker: boresight -z per its spec.
    const tracker = new THREE.Mesh(
      new THREE.CylinderGeometry(0.09, 0.12, 0.26, 16),
      material(0x9a6ad9, 0.8),
    );
    tracker.rotation.x = Math.PI / 2;
    tracker.position.set(0, 0, -0.58);
    body.add(tracker);
    track('st0', tracker);

    for (const side of [-1, 1]) {
      const panel = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.02, 0.5), material(0x1a2c50, 0.6));
      panel.position.set(side * 1.25, 0, 0);
      body.add(panel);
    }

    const wheels: { spinner: THREE.Object3D; index: number }[] = [];
    let wheelIndex = 0;
    for (const actuator of vehicle.actuators) {
      const axis = new THREE.Vector3(...(actuator.axis as [number, number, number])).normalize();
      if (actuator.kind.includes('wheel')) {
        const holder = new THREE.Group();
        const disk = new THREE.Mesh(
          new THREE.CylinderGeometry(0.17, 0.17, 0.07, 24),
          material(0xd9b45b, 0.85),
        );
        const marker = new THREE.Mesh(
          new THREE.BoxGeometry(0.035, 0.075, 0.15),
          material(0x333333, 0.9),
        );
        marker.position.set(0.1, 0, 0);
        const spinner = new THREE.Group();
        spinner.add(disk);
        spinner.add(marker);
        holder.add(spinner);
        holder.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);
        holder.position.copy(axis.clone().multiplyScalar(0.34));
        body.add(holder);
        wheels.push({ spinner, index: wheelIndex });
        track(actuator.name, disk);
        wheelIndex += 1;
      } else if (actuator.kind.includes('magnetorquer')) {
        const rod = new THREE.Mesh(
          new THREE.CylinderGeometry(0.035, 0.035, 1.15, 12),
          material(0xb0722c, 0.85),
        );
        rod.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);
        body.add(rod);
        track(actuator.name, rod);
      }
    }

    // Nadir sightline: satellite -> Earth center, world frame.
    const sightGeom = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(),
      new THREE.Vector3(),
    ]);
    const sight = new THREE.Line(
      sightGeom,
      new THREE.LineBasicMaterial({ color: 0x7bc96f, transparent: true, opacity: 0.7 }),
    );
    scene.add(sight);

    // ------------------------------------------------------ animation --
    let raf = 0;
    let lastT = performance.now();
    let phase = 0; // orbit angle, slew-limited toward the attitude-inferred target
    const orbitRate = 0.06; // rad/s of SCENE time — readable, not real-time
    const spinRates = new Map<number, number>();
    const baseOpacity = new Map<THREE.Mesh, number>();
    for (const bucket of meshes.values())
      for (const mesh of bucket)
        baseOpacity.set(mesh, (mesh.material as THREE.MeshStandardMaterial).opacity);

    const animate = (now: number) => {
      const dt = Math.min(0.1, (now - lastT) / 1000);
      lastT = now;
      const b = blobRef.current;

      // Attitude from the downlink.
      const sx = last(b, 'downlink/att.sigma_x');
      const sy = last(b, 'downlink/att.sigma_y');
      const sz = last(b, 'downlink/att.sigma_z');
      if (sx !== undefined && sy !== undefined && sz !== undefined) {
        body.quaternion.slerp(mrpToQuaternion(sx, sy, sz), 0.08);
      }

      // Orbit phase inferred from attitude: project the instrument axis
      // (+z body, in world) onto the orbit plane; the satellite belongs
      // where that axis meets the Earth. Slew-limited so a tumbling
      // vehicle doesn't teleport around the ring.
      const zWorld = new THREE.Vector3(0, 0, 1).applyQuaternion(body.quaternion);
      const planeInverse = incFrame.getWorldQuaternion(new THREE.Quaternion()).invert();
      const dirLocal = zWorld.clone().negate().applyQuaternion(planeInverse);
      const target = Math.atan2(-dirLocal.z, dirLocal.x);
      let err = target - phase;
      while (err > Math.PI) err -= 2 * Math.PI;
      while (err < -Math.PI) err += 2 * Math.PI;
      const maxStep = 2 * orbitRate * dt;
      phase += Math.max(-maxStep, Math.min(maxStep, err));
      carrier.position.set(Math.cos(phase) * ORBIT_R, 0, -Math.sin(phase) * ORBIT_R);

      // Sightline satellite -> Earth center.
      const satWorld = carrier.getWorldPosition(new THREE.Vector3());
      const positions = sight.geometry.getAttribute('position') as THREE.BufferAttribute;
      positions.setXYZ(0, satWorld.x, satWorld.y, satWorld.z);
      positions.setXYZ(1, 0, 0, 0);
      positions.needsUpdate = true;

      // The Earth turns beneath the orbit (scene-scaled, like the rest).
      earth.rotation.y += dt * 0.015;

      // Chase camera: ride just outside the satellite, Earth in frame.
      const outward = satWorld.clone().normalize();
      const chase = satWorld
        .clone()
        .add(outward.multiplyScalar(0.95))
        .add(new THREE.Vector3(0, 0.38, 0));
      camera.position.lerp(chase, 0.06);
      camera.lookAt(satWorld.clone().multiplyScalar(0.45));

      // Wheel spin from downlinked rates.
      for (const w of wheels) {
        const rate = last(b, `downlink/wheel${w.index}.speed_rad_s`);
        if (rate !== undefined) spinRates.set(w.index, rate);
        w.spinner.rotateY((spinRates.get(w.index) ?? 0) * dt * 0.15);
      }

      // Selection spotlight.
      const sel = selectedRef.current;
      const pulse = 0.5 + 0.5 * Math.sin(now / 180);
      for (const [name, bucket] of meshes) {
        for (const mesh of bucket) {
          const mat = mesh.material as THREE.MeshStandardMaterial;
          const base = baseOpacity.get(mesh) ?? 0.6;
          if (sel && meshes.has(sel) && name === sel) {
            mat.opacity = Math.min(1, base + 0.4);
            mat.emissive.setHex(0x2266ff);
            mat.emissiveIntensity = 0.6 + pulse;
          } else {
            mat.opacity = sel && meshes.has(sel) ? base * 0.5 : base;
            mat.emissive.setHex(0x000000);
            mat.emissiveIntensity = 0;
          }
        }
      }

      renderer.render(scene, camera);
      raf = requestAnimationFrame(animate);
    };
    raf = requestAnimationFrame(animate);
    return () => {
      cancelAnimationFrame(raf);
      renderer.dispose();
      mount.removeChild(renderer.domElement);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [vehicle]);

  const starValid = last(blob, 'downlink/att.star_valid');
  const rows = [
    ...vehicle.platform,
    ...vehicle.sensors.map((s) => ({ name: s.name, kind: s.kind, realness: s.realness })),
    ...vehicle.actuators.map((a) => ({ name: a.name, kind: a.kind, realness: a.realness })),
  ];

  return (
    <section className="vehicle-panel">
      <div className="cmd-header">
        VEHICLE — {vehicle.name}
        <span className="cmd-note">
          {vehicle.mass_kg} kg · {vehicle.strategy} @ {vehicle.rate_hz} Hz ·{' '}
          {vehicle.orbit.altitude_m
            ? `${Math.round((vehicle.orbit.altitude_m ?? 0) / 1000)} km / ${vehicle.orbit.inclination_deg}°`
            : 'no orbit'}{' '}
          · attitude downlinked · orbit phase inferred from attitude
          {starValid === 0 && <b className="veh-blind"> · TRACKER BLINDED — holding last</b>}
        </span>
      </div>
      <div className="veh-row">
        <div className="veh-3d" ref={mountRef} />
        <div className="veh-table">
          {rows.map((r) => (
            <div
              key={r.name}
              className={`veh-line veh-clickable${selected === r.name ? ' veh-selected' : ''}`}
              onClick={() => setSelected(selected === r.name ? null : r.name)}
            >
              <span className="veh-name">{r.name}</span>
              <span className="veh-kind">{r.kind}</span>
              <span className={`veh-chip veh-${r.realness}`}>{r.realness}</span>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}
