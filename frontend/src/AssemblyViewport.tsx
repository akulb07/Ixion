import { useEffect, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { assemblyLayout } from "./assembly.mjs";
import { replayTransform } from "./replay.mjs";

export type AssemblyProject = {
  assembly: { wheel_radius_m: number; track_width_m: number };
  components: { id: string; category: string }[];
};
const palette: Record<string, number> = {
  chassis: 0x414957,
  wheel: 0x222832,
  motor: 0x9f9070,
  encoder: 0x3e7a6e,
  mcu: 0x406e68,
  motor_driver: 0x66537d,
  battery: 0x525c77,
  sensor: 0x658088,
  regulator: 0x77724b,
  interface: 0x695e7b,
  caster: 0x303641,
};

export function AssemblyViewport({
  project,
  selected,
  onSelect,
  motion,
}: {
  project: AssemblyProject;
  selected: string;
  onSelect: (id: string) => void;
  motion?: { distance_m: number; grade_rad: number };
}) {
  const host = useRef<HTMLDivElement>(null);
  const controls = useRef<{
    select: (id: string) => void;
    camera: (top: boolean) => void;
    move: (value: { distance_m: number; grade_rad: number }) => void;
  } | null>(null);
  const selectCallback = useRef(onSelect),
    selection = useRef(selected);
  const recordedMotion = useRef(motion);
  const [error, setError] = useState("");
  const layout = useMemo(() => {
    try {
      return { value: assemblyLayout(project), error: "" };
    } catch (error) {
      return {
        value: null,
        error: error instanceof Error ? error.message : String(error),
      };
    }
  }, [project]);
  useEffect(() => {
    selectCallback.current = onSelect;
  }, [onSelect]);
  useEffect(() => {
    selection.current = selected;
    controls.current?.select(selected);
  }, [selected]);
  useEffect(() => {
    recordedMotion.current = motion;
    if (motion) controls.current?.move(motion);
  }, [motion]);
  useEffect(() => {
    const container = host.current,
      plan = layout.value;
    if (!container || !plan) return;
    setError("");
    let renderer: THREE.WebGLRenderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true });
    } catch {
      setError(
        "3D preview needs WebGL 2. The part editor and hardware experiments are still available.",
      );
      return;
    }
    renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 2));
    renderer.domElement.setAttribute(
      "aria-label",
      "Interactive 3D reference rover assembly",
    );
    renderer.domElement.setAttribute("role", "img");
    container.append(renderer.domElement);
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b0e14);
    const camera = new THREE.PerspectiveCamera(
      40,
      1,
      plan.span / 1000,
      plan.span * 100,
    );
    camera.up.set(0, 0, 1);
    const orbit = new OrbitControls(camera, renderer.domElement);
    orbit.target.fromArray(plan.center);
    orbit.minDistance = plan.span * 0.3;
    orbit.maxDistance = plan.span * 10;
    const hemi = new THREE.HemisphereLight(0xd9e3ff, 0x202434, 2);
    hemi.position.set(0, 0, 1);
    scene.add(hemi);
    const light = new THREE.DirectionalLight(0xffffff, 3);
    light.position.set(1, -1, 2);
    scene.add(light);
    const grid = new THREE.GridHelper(plan.span * 4, 24, 0x404355, 0x202735);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -plan.radius * 0.01;
    scene.add(grid);
    scene.add(new THREE.AxesHelper(plan.span * 0.4));
    const objects = new Map<string, THREE.Group>(),
      pickable: THREE.Mesh[] = [];
    const rover = new THREE.Group();
    scene.add(rover);
    for (const part of plan.parts) {
      const group = new THREE.Group();
      group.position.fromArray(part.position);
      group.userData.componentId = part.id;
      const geometry =
        part.shape === "cylinder"
          ? new THREE.CylinderGeometry(
              part.size[0],
              part.size[0],
              part.size[1],
              32,
            )
          : part.shape === "sphere"
            ? new THREE.SphereGeometry(part.size[0], 24, 12)
            : new THREE.BoxGeometry(part.size[0], part.size[1], part.size[2]);
      const material = new THREE.MeshStandardMaterial({
        color: palette[part.category] ?? 0x5e6779,
        roughness: 0.58,
        metalness: 0.25,
      });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.userData.componentId = part.id;
      group.add(mesh);
      pickable.push(mesh);
      const edge = new THREE.LineSegments(
        new THREE.EdgesGeometry(geometry),
        new THREE.LineBasicMaterial({
          color: 0x778094,
          transparent: true,
          opacity: 0.32,
        }),
      );
      group.add(edge);
      if (part.category === "wheel") {
        const hub = new THREE.Mesh(
          new THREE.CylinderGeometry(
            plan.radius * 0.4,
            plan.radius * 0.4,
            part.size[1] * 1.04,
            24,
          ),
          new THREE.MeshStandardMaterial({
            color: 0x8994a6,
            metalness: 0.65,
            roughness: 0.35,
          }),
        );
        hub.userData.componentId = part.id;
        group.add(hub);
        pickable.push(hub);
        const spoke = new THREE.Mesh(
          new THREE.BoxGeometry(
            plan.radius * 1.6,
            part.size[1] * 1.08,
            plan.radius * 0.08,
          ),
          new THREE.MeshStandardMaterial({ color: 0xb496ff }),
        );
        spoke.userData.componentId = part.id;
        group.add(spoke);
        pickable.push(spoke);
      }
      rover.add(group);
      objects.set(part.id, group);
    }
    const box = new THREE.BoxHelper(new THREE.Object3D(), 0xb496ff);
    box.visible = false;
    scene.add(box);
    const render = () => {
      if (box.visible) box.update();
      renderer.render(scene, camera);
    };
    const select = (id: string) => {
      const object = objects.get(id);
      box.visible = !!object;
      if (object) box.setFromObject(object);
      render();
    };
    const positionCamera = (top: boolean) => {
      camera.position.set(
        rover.position.x + plan.span * (top ? 0.001 : 1.4),
        plan.span * (top ? -0.001 : -1.6),
        rover.position.z + plan.span * (top ? 2.8 : 1.2),
      );
      orbit.target.fromArray(plan.center).add(rover.position);
      orbit.update();
      render();
    };
    const move = (value: { distance_m: number; grade_rad: number }) => {
      try {
        const pose = replayTransform(
          value.distance_m,
          plan.radius,
          value.grade_rad,
        );
        const next = new THREE.Vector3(pose.x, 0, pose.z);
        const delta = next.clone().sub(rover.position);
        rover.position.copy(next);
        rover.rotation.y = pose.pitch;
        for (const part of plan.parts)
          if (part.category === "wheel")
            objects.get(part.id)!.rotation.y = pose.wheelAngle;
        camera.position.add(delta);
        orbit.target.add(delta);
        grid.rotation.set(Math.PI / 2, 0, -pose.pitch);
        grid.position.copy(next);
        grid.position.z -= plan.radius * 0.01;
        rover.visible = true;
        setError("");
        orbit.update();
        render();
      } catch (error) {
        rover.visible = false;
        setError(error instanceof Error ? error.message : String(error));
        render();
      }
    };
    const resize = () => {
      const { width, height } = container.getBoundingClientRect();
      if (width > 0 && height > 0) {
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
        renderer.setSize(width, height, false);
        render();
      }
    };
    const observer = new ResizeObserver(resize);
    observer.observe(container);
    orbit.addEventListener("change", render);
    const down = { x: 0, y: 0 };
    const pointerDown = (event: PointerEvent) => {
      down.x = event.clientX;
      down.y = event.clientY;
    };
    const pointerUp = (event: PointerEvent) => {
      if (
        Math.hypot(event.clientX - down.x, event.clientY - down.y) > 5 ||
        event.button !== 0
      )
        return;
      const bounds = renderer.domElement.getBoundingClientRect(),
        ray = new THREE.Raycaster();
      ray.setFromCamera(
        new THREE.Vector2(
          ((event.clientX - bounds.left) / bounds.width) * 2 - 1,
          (-(event.clientY - bounds.top) / bounds.height) * 2 + 1,
        ),
        camera,
      );
      const hit = ray.intersectObjects(pickable, false)[0];
      if (hit) selectCallback.current(hit.object.userData.componentId);
    };
    renderer.domElement.addEventListener("pointerdown", pointerDown);
    renderer.domElement.addEventListener("pointerup", pointerUp);
    controls.current = { select, camera: positionCamera, move };
    if (recordedMotion.current) move(recordedMotion.current);
    positionCamera(false);
    resize();
    select(selection.current);
    return () => {
      controls.current = null;
      observer.disconnect();
      orbit.dispose();
      renderer.domElement.removeEventListener("pointerdown", pointerDown);
      renderer.domElement.removeEventListener("pointerup", pointerUp);
      const geometries = new Set<THREE.BufferGeometry>(),
        materials = new Set<THREE.Material>();
      scene.traverse((object) => {
        const value = object as THREE.Mesh;
        if (value.geometry) geometries.add(value.geometry);
        if (value.material)
          for (const material of Array.isArray(value.material)
            ? value.material
            : [value.material])
            materials.add(material);
      });
      geometries.forEach((value) => value.dispose());
      materials.forEach((value) => value.dispose());
      renderer.dispose();
      renderer.domElement.remove();
    };
  }, [layout]);
  return (
    <section className="assembly-preview">
      <div className="assembly-toolbar">
        <strong>
          {motion ? "RECORDED MOTION · FOLLOW CAMERA" : "REFERENCE ASSEMBLY"}
        </strong>
        <span>Selected: {selected.replaceAll("_", " ")}</span>
        <button onClick={() => controls.current?.camera(false)}>
          Reset camera
        </button>
        <button onClick={() => controls.current?.camera(true)}>Top view</button>
      </div>
      {(error || layout.error) && (
        <p className="message error" role="status">
          {error || layout.error}
        </p>
      )}
      <div className="assembly-canvas" ref={host} />
      <p className="hint">
        Wheel diameter {(project.assembly.wheel_radius_m * 2).toPrecision(4)} m
        · track {project.assembly.track_width_m.toPrecision(4)} m · X forward /
        Y left / Z up
      </p>
      <p className="hint">
        {motion &&
          "Recorded straight-line travel; wheel spin is distance/radius under the no-slip model, not encoder feedback. "}
        Wheel radius and axle spacing come from the assembly. Wheel width,
        chassis, board shapes and part placement are illustrative. This view
        doesn't calculate contacts or 3D dynamics. Drag to orbit, scroll to
        zoom, or select a part.
      </p>
    </section>
  );
}
