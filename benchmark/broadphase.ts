/**
 * Broadphase benchmark: Socket-and-prune vs dynamic AABB tree.
 *
 * Run with: npx ts-node --transpile-only benchmark/broadphase.ts
 */
import { World } from '../src/world/World'
import { NaiveBroadphase } from '../src/collision/NaiveBroadphase'
import { SAPBroadphase } from '../src/collision/SAPBroadphase'
import { AABBTreeBroadphase } from '../src/collision/AABBTreeBroadphase'
import { AABB } from '../src/collision/AABB'
import { Body } from '../src/objects/Body'
import { Sphere } from '../src/shapes/Sphere'
import { Vec3 } from '../src/math/Vec3'
import type { Broadphase } from '../src/collision/Broadphase'

const STEPS = 60
const WARMUP = 5

function createRandom(seed: number): () => number {
  let s = seed >>> 0
  return () => {
    s = (s + 0x6d2b79f5) >>> 0
    let t = s
    t = Math.imul(t ^ (t >>> 15), t | 1)
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61)
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

type Layout = 'grid' | 'cluster' | 'scattered'

function createWorld(BroadphaseClass: new () => Broadphase, count: number, layout: Layout, radius: number): World {
  const broadphase = new BroadphaseClass()
  // Compare like with like: every broadphase uses AABB pair tests
  broadphase.useBoundingBoxes = true
  const world = new World({ broadphase, gravity: new Vec3(0, 0, 0) })
  const random = createRandom(12345)
  const side = Math.ceil(Math.cbrt(count))

  for (let i = 0; i < count; i++) {
    let x: number
    let y: number
    let z: number

    if (layout === 'grid') {
      x = ((i % side) - side / 2) * radius * 2.2
      y = (Math.floor(i / side) % side) - side / 2
      z = (Math.floor(i / (side * side)) % side) - side / 2
      y *= radius * 2.2
      z *= radius * 2.2
    } else if (layout === 'cluster') {
      x = random() * 10 - 5
      y = random() * 10 - 5
      z = random() * 10 - 5
    } else {
      x = random() * 200 - 100
      y = random() * 200 - 100
      z = random() * 200 - 100
    }

    const body = new Body({ mass: 1, shape: new Sphere(radius), position: new Vec3(x, y, z) })
    world.addBody(body)
  }

  return world
}

type Motion = 'static' | 'coherent' | 'jitter'

function run(broadphase: Broadphase, world: World, motion: Motion): { ms: number; pairs: number } {
  const p1: Body[] = []
  const p2: Body[] = []
  const random = createRandom(99)
  const dt = 1 / 60

  // collisionPairs() is normally called from World.internalStep(), which sets dt.
  // Broadphases may use it to predict motion.
  world.dt = dt

  const step = () => {
    for (let i = 0; i < world.bodies.length; i++) {
      const body = world.bodies[i]
      if (motion === 'jitter') {
        // Random movement, the position changes direction every step
        body.position.x += (random() - 0.5) * 2
        body.position.y += (random() - 0.5) * 2
        body.position.z += (random() - 0.5) * 2
      } else if (motion === 'coherent') {
        // Constant velocity of ~10 units per second, like a body in free flight.
        // The velocity is set so broadphases can use it for prediction.
        const vx = (((body.id * 37) % 21) - 10) * 0.9
        const vy = (((body.id * 53) % 21) - 10) * 0.9
        const vz = (((body.id * 71) % 21) - 10) * 0.9
        body.velocity.set(vx, vy, vz)
        body.position.x += vx * dt
        body.position.y += vy * dt
        body.position.z += vz * dt
      } else {
        body.position.x += 0.01
      }
      body.aabbNeedsUpdate = true
    }
    p1.length = 0
    p2.length = 0
    broadphase.collisionPairs(world, p1, p2)
    return p1.length
  }

  for (let i = 0; i < WARMUP; i++) {
    step()
  }

  let pairs = 0
  const start = performance.now()
  for (let i = 0; i < STEPS; i++) {
    pairs = step()
  }
  const ms = performance.now() - start

  return { ms, pairs }
}

function runAabbQueries(broadphase: Broadphase, world: World): number {
  const random = createRandom(7)
  const query = new AABB()
  const result: Body[] = []
  const QUERIES = 2000

  for (let i = 0; i < 100; i++) {
    broadphase.aabbQuery(world, query, result)
  }

  const start = performance.now()
  for (let i = 0; i < QUERIES; i++) {
    const x = random() * 100 - 50
    const y = random() * 100 - 50
    const z = random() * 100 - 50
    query.lowerBound.set(x, y, z)
    query.upperBound.set(x + 5, y + 5, z + 5)
    result.length = 0
    broadphase.aabbQuery(world, query, result)
  }

  return ((performance.now() - start) / QUERIES) * 1000
}

const scenarios: Array<{ name: string; count: number; layout: Layout; radius: number }> = [
  { name: 'grid 1000 (r=0.5)', count: 1000, layout: 'grid', radius: 0.5 },
  { name: 'dense cluster 500 (r=0.5)', count: 500, layout: 'cluster', radius: 0.5 },
  { name: 'scattered 2000 (r=0.5)', count: 2000, layout: 'scattered', radius: 0.5 },
  { name: 'large bodies 500 (r=2)', count: 500, layout: 'scattered', radius: 2 },
]

const implementations: Array<{ name: string; ctor: new () => Broadphase }> = [
  { name: 'Naive', ctor: NaiveBroadphase },
  { name: 'SAP', ctor: SAPBroadphase },
  { name: 'AABBTree', ctor: AABBTreeBroadphase },
]

console.log(`collisionPairs(): ${STEPS} steps after ${WARMUP} warmup steps`)
console.log(`World's default broadphase is AABBTreeBroadphase`)
console.log(`"moving" gives each body a constant velocity, "jitter" reverses direction every step\n`)

for (const scenario of scenarios) {
  console.log(`--- ${scenario.name} ---`)
  for (const implementation of implementations) {
    const world = createWorld(implementation.ctor, scenario.count, scenario.layout, scenario.radius)
    const { ms, pairs } = run(world.broadphase, world, 'static')
    const { ms: coherentMs } = run(world.broadphase, world, 'coherent')
    const { ms: jitterMs } = run(world.broadphase, world, 'jitter')
    const queryUs = runAabbQueries(world.broadphase, world)
    console.log(
      `${implementation.name.padEnd(9)} static ${(ms / STEPS).toFixed(3).padStart(6)} ms/step   ` +
        `moving ${(coherentMs / STEPS).toFixed(3).padStart(6)} ms/step   ` +
        `jitter ${(jitterMs / STEPS).toFixed(3).padStart(6)} ms/step   ` +
        `${String(pairs).padStart(6)} pairs   aabbQuery ${queryUs.toFixed(2).padStart(7)} us`
    )
  }
  console.log('')
}
