/**
 * The vehicle, as configured and as flying.
 *
 * Left: a live 3D render built from the SAME vehicle file the flight
 * software runs — wheel and rod glyphs sit on their configured mounting
 * axes, and the body is oriented by the star tracker attitude that
 * crossed the radio (MRP → rotation). Wheels visually spin at their
 * downlinked rates. When the tracker is blinded the model holds its
 * last known attitude and says so — the ground only knows what it
 * heard.
 *
 * Right: the composition table — what is real hardware and what is a
 * simulated device behind the same driver contract.
 */

import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import type { MissionBlob } from '../data/types';

export interface VehicleInfo {
  name: string;
  mass_kg: number;
  strategy: string;
  estimator: string;
  rate_hz: number;
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

/** Last value of a channel, or undefined. */
function last(blob: MissionBlob, channel: string): number | undefined {
  const s = blob.series[channel];
  return s && s.v.length > 0 ? s.v[s.v.length - 1] : undefined;
}

/** MRP sigma_BN -> body-to-world rotation for the scene. */
function mrpToQuaternion(sx: number, sy: number, sz: number): THREE.Quaternion {
  const s2 = sx * sx + sy * sy + sz * sz;
  // MRP -> quaternion (scalar-first), then invert: sigma_BN orients
  // body FROM inertial; the scene wants body IN inertial.
  const q = new THREE.Quaternion(
    (2 * sx) / (1 + s2),
    (2 * sy) / (1 + s2),
    (2 * sz) / (1 + s2),
    (1 - s2) / (1 + s2),
  );
  return q.invert();
}

export function VehiclePanel({ vehicle, blob }: Props) {
  const mountRef = useRef<HTMLDivElement | null>(null);
  const sceneRef = useRef<{
    body: THREE.Group;
    wheels: { name: string; spinner: THREE.Object3D; axis: THREE.Vector3 }[];
    renderer: THREE.WebGLRenderer;
    camera: THREE.PerspectiveCamera;
    scene: THREE.Scene;
  } | null>(null);
  const blobRef = useRef(blob);
  blobRef.current = blob;

  useEffect(() => {
    const mount = mountRef.current;
    if (!mount) return;
    const width = mount.clientWidth || 340;
    const height = 260;
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    mount.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(40, width / height, 0.1, 100);
    camera.position.set(2.6, 1.8, 2.6);
    camera.lookAt(0, 0, 0);
    scene.add(new THREE.AmbientLight(0x8899aa, 1.2));
    const sun = new THREE.DirectionalLight(0xfff2dd, 2.0);
    sun.position.set(4, 3, 2);
    scene.add(sun);

    const body = new THREE.Group();
    scene.add(body);

    // Bus structure.
    const bus = new THREE.Mesh(
      new THREE.BoxGeometry(0.9, 0.9, 0.9),
      new THREE.MeshStandardMaterial({ color: 0x2c3a4a, metalness: 0.4, roughness: 0.5 }),
    );
    body.add(bus);

    // Instrument face: +z, the thing nadir pointing aims at the Earth.
    const face = new THREE.Mesh(
      new THREE.ConeGeometry(0.16, 0.3, 24),
      new THREE.MeshStandardMaterial({ color: 0x4a90d9, emissive: 0x123, roughness: 0.3 }),
    );
    face.rotation.x = Math.PI / 2;
    face.position.set(0, 0, 0.6);
    body.add(face);

    // Solar-ish panels for silhouette.
    for (const side of [-1, 1]) {
      const panel = new THREE.Mesh(
        new THREE.BoxGeometry(1.5, 0.02, 0.5),
        new THREE.MeshStandardMaterial({ color: 0x1a2c50, metalness: 0.7, roughness: 0.3 }),
      );
      panel.position.set(side * 1.25, 0, 0);
      body.add(panel);
    }

    // Wheels and rods from the CONFIG's mounting axes.
    const wheels: { name: string; spinner: THREE.Object3D; axis: THREE.Vector3 }[] = [];
    for (const actuator of vehicle.actuators) {
      const axis = new THREE.Vector3(...(actuator.axis as [number, number, number])).normalize();
      if (actuator.kind.includes('wheel')) {
        const holder = new THREE.Group();
        const disk = new THREE.Mesh(
          new THREE.CylinderGeometry(0.16, 0.16, 0.06, 24),
          new THREE.MeshStandardMaterial({ color: 0xd9b45b, metalness: 0.8, roughness: 0.35 }),
        );
        const marker = new THREE.Mesh(
          new THREE.BoxGeometry(0.03, 0.062, 0.14),
          new THREE.MeshStandardMaterial({ color: 0x222222 }),
        );
        marker.position.set(0.09, 0, 0);
        const spinner = new THREE.Group();
        spinner.add(disk);
        spinner.add(marker);
        holder.add(spinner);
        // Cylinder spins about its local Y: align local Y to the mount axis.
        holder.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);
        holder.position.copy(axis.clone().multiplyScalar(0.32));
        body.add(holder);
        wheels.push({ name: actuator.name, spinner, axis: new THREE.Vector3(0, 1, 0) });
      } else if (actuator.kind.includes('magnetorquer')) {
        const rod = new THREE.Mesh(
          new THREE.CylinderGeometry(0.03, 0.03, 1.1, 12),
          new THREE.MeshStandardMaterial({ color: 0x8a5a2c, metalness: 0.6, roughness: 0.4 }),
        );
        rod.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);
        rod.position.copy(axis.clone().multiplyScalar(0.0));
        body.add(rod);
      }
    }

    // Inertial reference triad, faint.
    scene.add(new THREE.AxesHelper(1.6));

    sceneRef.current = { body, wheels, renderer, camera, scene };

    let raf = 0;
    let lastT = performance.now();
    const spinRates = new Map<string, number>();
    const animate = (now: number) => {
      const dt = Math.min(0.1, (now - lastT) / 1000);
      lastT = now;
      const b = blobRef.current;
      const sx = last(b, 'downlink/att.sigma_x');
      const sy = last(b, 'downlink/att.sigma_y');
      const sz = last(b, 'downlink/att.sigma_z');
      if (sx !== undefined && sy !== undefined && sz !== undefined) {
        body.quaternion.slerp(mrpToQuaternion(sx, sy, sz), 0.08);
      }
      for (const [i, w] of wheels.entries()) {
        const rate = last(b, `downlink/wheel${i}.speed_rad_s`);
        if (rate !== undefined) spinRates.set(w.name, rate);
        const spin = spinRates.get(w.name) ?? 0;
        w.spinner.rotateOnAxis(w.axis, spin * dt * 0.15); // scaled for visibility
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
          {vehicle.mass_kg} kg · {vehicle.strategy} @ {vehicle.rate_hz} Hz · attitude from the
          downlinked star tracker
          {starValid === 0 && <b className="veh-blind"> · TRACKER BLINDED — holding last</b>}
        </span>
      </div>
      <div className="veh-row">
        <div className="veh-3d" ref={mountRef} />
        <div className="veh-table">
          {rows.map((r) => (
            <div key={r.name} className="veh-line">
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
