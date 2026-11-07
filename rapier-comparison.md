# Rapier vs this engine (3D)

A feature and performance comparison with [dimforge/rapier](https://github.com/dimforge/rapier),
plus the features and optimizations that were ported over.

Rapier is a Rust engine (the JavaScript bindings run it through WebAssembly), so some of its
performance work has no direct JavaScript equivalent. Everything below is about the 3D engine only;
2D is out of scope.

Sources: the [Rapier user guide](https://rapier.rs/docs/) (colliders, joints, rigid bodies, scene
queries, character controller, determinism, serialization), the
[repository README](https://github.com/dimforge/rapier) and the
[integration parameters source](https://github.com/dimforge/rapier/blob/master/src/dynamics/integration_parameters.rs).

## 1. What Rapier provides (3D)

- **Rigid bodies**: dynamic, fixed and kinematic (position or velocity based); forces and impulses;
  mass/inertia from collider density, mass or explicit mass properties; locked translation/rotation
  axes; per-body solver iteration overrides.
- **Colliders**: ball, cuboid, capsule, cylinder, cone, half-space, segment, triangle, convex hull,
  convex mesh, triangle mesh, heightfield, voxel shape, compound shapes, convex decomposition,
  rounded variants of several shapes, contact skin (a standoff margin).
- **Joints**: fixed, spherical, revolute, prismatic; a generic joint covering free, Cartesian,
  planar, cylindrical, pin-slot, rectangular and universal constraints; impulse and multibody
  (reduced-coordinate) formulations; joint limits and PD motors; `contactsEnabled`.
- **CCD**: automatic sweeping of fast dynamic bodies against fixed colliders, opt-in sweeping
  against kinematic/dynamic bodies, configurable CCD substeps and a global on/off switch.
- **Sensors and events**: sensor colliders detect intersections but produce no forces; collision
  (start/stop) events and contact-force events are opt-in per collider.
- **Filtering**: collision groups (narrowphase), solver groups (which contacts produce forces),
  physics hooks for custom pair filtering and contact modification.
- **Materials**: per-collider friction and restitution with combine rules, contact skin.
- **Sleeping**: island-based; islands are solved as a unit and sleep/wake together.
- **Solver**: projected Gauss-Seidel with warm starting, contact-manifold clustering (at most four
  contact points per contact plane in 3D), optional simplified friction model.
- **Scene queries**: ray casts, shape casts/sweeps, closest-point projection, point containment,
  exact shape intersections, conservative AABB intersections, all with filters and served by the
  broadphase BVH.
- **Controllers**: kinematic character controller (move and slide, slopes, autostep, snap-to-ground)
  and a raycast-suspension vehicle controller.
- **Serialization**: full physics world snapshots with `serde` (Rust).
- **Determinism**: locally deterministic; cross-platform determinism behind an
  `enhanced-determinism` feature.
- **Multithreading/SIMD**: Rayon-parallel pipeline (`parallel` feature), SIMD-batched constraint
  solving and contact processing (four manifolds per instruction), optional `simd8`.

## 2. Feature comparison

| Capability                                                           | Rapier (3D)                    | This engine                                                                  | Notes                                                 |
| -------------------------------------------------------------------- | ------------------------------ | ---------------------------------------------------------------------------- | ----------------------------------------------------- |
| Dynamic / static / kinematic bodies                                  | yes                            | yes                                                                          |                                                       |
| Sphere, box, plane, convex, cylinder, heightfield, trimesh, particle | yes                            | yes                                                                          |                                                       |
| Capsule                                                              | yes (exact)                    | yes (tessellated convex)                                                     | new `Capsule`, see limitations                        |
| Cone                                                                 | yes                            | yes                                                                          | new `Cone`, a convex polyhedron                       |
| Convex hull generation from points                                   | yes                            | no                                                                           | `ConvexPolyhedron` takes vertices/faces               |
| Voxel shape, convex decomposition                                    | yes                            | no                                                                           | not needed for the current use cases                  |
| Rounded shapes, contact skin                                         | yes                            | no                                                                           |                                                       |
| Compound colliders                                                   | yes                            | bodies support multiple shapes                                               | per-shape material/filter/response                    |
| Point-to-point / hinge / lock / distance / cone-twist joints         | —                              | yes                                                                          |                                                       |
| Prismatic (slider) joint                                             | yes                            | yes                                                                          | new `PrismaticConstraint`                             |
| Generic / 6-DOF joint                                                | yes                            | no                                                                           |                                                       |
| Joint limits, joint motors                                           | yes                            | motors only (`HingeConstraint`, `RotationalMotorEquation`), no slider limits |                                                       |
| CCD                                                                  | yes                            | yes                                                                          | discrete+sweep with sampling and bisection            |
| Sensors / triggers                                                   | yes                            | yes                                                                          | `isTrigger`, `beginTrigger`/`endTrigger`, body events |
| Collision start/stop events                                          | yes                            | yes                                                                          | `beginContact`/`endContact`, `beginShapeContact`      |
| Contact-force events                                                 | yes                            | partial                                                                      | contacts expose `getImpactVelocityAlongNormal()`      |
| Collision filtering                                                  | groups + solver groups + hooks | group/mask on shapes                                                         | no separate solver groups                             |
| Per-shape materials and response                                     | yes                            | yes                                                                          |                                                       |
| Sleeping                                                             | island based                   | per body                                                                     |                                                       |
| Warm starting, manifold clustering                                   | yes                            | no                                                                           | see optimizations                                     |
| Ray casts                                                            | yes                            | yes                                                                          |                                                       |
| Shape cast / sweep                                                   | yes                            | yes                                                                          | new `World.castShape`                                 |
| Overlap / intersection query                                         | yes                            | yes                                                                          | new `World.intersectShape`; `aabbQuery` also exists   |
| Closest point / point containment                                    | yes                            | no                                                                           | could be built on the cast code                       |
| Character controller                                                 | yes                            | no                                                                           | see necessity table                                   |
| Vehicle controller                                                   | yes                            | yes (`RaycastVehicle`, `RigidVehicle`)                                       |                                                       |
| Serialization / snapshots                                            | yes (serde)                    | no                                                                           |                                                       |
| Determinism                                                          | local, opt-in cross-platform   | local (floating point, same code path)                                       | no cross-platform guarantee                           |
| Multithreading, SIMD                                                 | yes                            | n/a                                                                          | JavaScript                                            |
| Soft bodies                                                          | experimental                   | no                                                                           | `SPHSystem` exists for fluids                         |

## 3. Missing features: necessity and decision

Necessity is judged for a lightweight 3D engine used from JavaScript (games, demos, robot/vehicle
experiments), which is what this fork targets.

| Missing feature                         | Necessity                                                                  | Decision                                                                                        |
| --------------------------------------- | -------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| Capsule shape                           | **high** — characters, limbs, capsules are the standard character collider | **implemented** as a tessellated `ConvexPolyhedron`, so it works with every existing shape pair |
| Cone shape                              | medium — props, funnels, debris                                            | **implemented** as a convex polyhedron                                                          |
| Prismatic (slider) joint                | **high** — doors, elevators, pistons, suspension links                     | **implemented** (`PrismaticConstraint`); limits and motors are not implemented                  |
| Shape cast / sweep                      | **high** — ground probing, movement prediction, AI, character controllers  | **implemented** (`World.castShape`) using the broadphase BVH plus sampled/bisected narrowphase  |
| Shape overlap query                     | **high** — triggers, placement tests, area effects                         | **implemented** (`World.intersectShape`)                                                        |
| Closest point / point containment       | low — app can use a tiny sphere cast                                       | not implemented                                                                                 |
| Character controller                    | medium — very useful, but behaviour is application specific                | not implemented; `castShape` + a kinematic body is the building block                           |
| Generic 6-DOF joint                     | low/medium — advanced articulation                                         | not implemented; prismatic + hinge + cone-twist cover most cases                                |
| Joint limits / PD motors for all joints | medium                                                                     | motors exist for hinges; slider limits not implemented                                          |
| Contact-force events                    | low                                                                        | contacts are exposed and carry impact velocity                                                  |
| Rounded shapes / contact skin           | low                                                                        | would require margin support in every narrowphase pair                                          |
| Voxel shape / convex decomposition      | low                                                                        | `ConvexPolyhedron` + `Heightfield`/`Trimesh` cover typical scenes                               |
| Serialization                           | medium — useful for save games and networking                              | not implemented; world state is plain objects, so an app-level dump is straightforward          |
| Cross-platform determinism              | low                                                                        | not guaranteed; the engine is locally deterministic                                             |
| Multithreading / SIMD                   | n/a in JavaScript                                                          | not applicable                                                                                  |
| Soft bodies                             | low                                                                        | experimental even in Rapier; `SPHSystem` covers simple fluids                                   |

## 4. Optimizations from Rapier that were applied

| Optimization                                       | Status                                                                                                              |
| -------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| BVH broadphase that also serves scene queries      | already present (`AABBTreeBroadphase`, default); the new shape queries use it                                       |
| Swept CCD with a time of impact                    | already present; now with a square-root-free fast path for resting bodies                                           |
| Contact/manifold result reuse                      | `convexConvex` no longer allocates a result array per pair, and the shape queries reuse contact pools               |
| Removing debug output from hot code                | the convex clipping code logged every clamped contact depth; removed                                                |
| Reusing equation pools for queries and CCD         | yes, query contacts/friction are recycled                                                                           |
| Warm starting contact impulses                     | **not applied** — requires persistent contact identity across steps, a much larger change to the narrowphase/solver |
| Contact manifold clustering (≤ 4 points per plane) | **not applied** — changes stacking behaviour; the existing `enableFrictionReduction` option is available            |
| Island-based sleeping                              | **not applied** — sleeping is per body                                                                              |
| SIMD batching, parallel pipeline, WASM             | **not applicable** to JavaScript                                                                                    |

## 5. Performance against the original cannon-es

`benchmark/compare-original.js` runs the same scenes in this engine and in the published
`cannon-es@0.20.0` package (installed under `.baseline/`), taking the minimum of three runs.
Run it with `yarn bench:compare`.

| Scenario (ms, lower is better)   | this engine, default | original, default | this engine, SAP | original, SAP | this engine, Naive | original, Naive |
| -------------------------------- | -------------------: | ----------------: | ---------------: | ------------: | -----------------: | --------------: |
| 125-box stack, 100 steps         |                  441 |               421 |              412 |           393 |                402 |             403 |
| 400 falling spheres, 100 steps   |               **82** |               133 |               87 |            63 |                152 |             124 |
| 500 boxes + 1000 rays            |              **1.1** |               9.3 |          **3.6** |           8.8 |            **4.4** |             8.9 |
| 800 bodies, 100 pair generations |             **16.6** |               386 |             33.9 |          20.5 |                426 |             401 |

What the numbers say:

- The default configuration is what users get, and it is the big win: the AABB tree generates
  broadphase pairs **23× faster** than the original naive default, runs ray casts **8× faster**
  (the tree doubles as a query structure) and handles a sphere shower **38% faster**.
- Box stacks are at parity (within ~5%): they are dominated by narrowphase and solver work, not by
  the broadphase. The original is slightly ahead there because this fork runs its per-step CCD pass
  and trigger bookkeeping even when nothing is fast or triggering.
- Explicit `SAPBroadphase` is unchanged, and on sphere showers the original is still faster with
  SAP (63 vs 87 ms). Pair generation with SAP is also faster in the original (20.5 vs 33.9 ms).
  This is the same trade-off documented for the broadphase work: the tree wins on queries, dense
  scenes and the default out-of-the-box experience; SAP remains competitive for pure pair generation.

## 6. New API

```ts
import { World, Body, Capsule, Cone, PrismaticConstraint, Sphere, Vec3, Quaternion, RaycastResult } from 'cannon-es'

// Capsule: radius, height of the cylindrical part, segments, cap rings
const capsule = new Capsule(0.4, 1.2, 10, 4)

// Cone: base radius, height, segments
const cone = new Cone(0.5, 1.5, 12)

// Slider joint along bodyA's local X axis
world.addConstraint(new PrismaticConstraint(rail, slider, { axis: new Vec3(1, 0, 0) }))

// Overlap query: bodies whose shapes overlap the given shape at a world transform
const hits = world.intersectShape(new Sphere(1), new Vec3(0, 2, 0), new Quaternion())

// Shape cast: first body hit by a sphere swept from `from` to `to`
const result = new RaycastResult()
if (world.castShape(new Sphere(0.5), from, to, new Quaternion(), {}, result)) {
  console.log(result.body, result.distance, result.hitPointWorld, result.hitNormalWorld)
}
```

## 7. Limitations of the new code

- `Capsule` is a tessellated convex polyhedron, not an exact capsule: its surface is faceted and
  contact quality depends on `numSegments`/`numCapSegments`. It keeps the `CONVEXPOLYHEDRON` shape
  type so all existing narrowphase pairs and ray casts work unchanged.
- `Cone` is likewise tessellated around its base and keeps the `CONVEXPOLYHEDRON` type.
- `World.castShape` samples the sweep (at most one sample per half bounding radius, capped by
  `maxSamples`) and refines with bisection. Rotation is not swept, and very thin obstacles can be
  missed if the sample budget is too small.
- Shape queries inherit the narrowphase: a sphere whose center is deep inside a convex shape may
  not report a contact, exactly like a normal collision step.
- `PrismaticConstraint` removes five degrees of freedom (two translations, three rotations) but has
  no joint limits and no motor.
- Warm starting, manifold clustering and island sleeping from Rapier were not ported; they require
  persistent contacts and a different solver architecture.
