import { AABBTreeBroadphase } from './AABBTreeBroadphase'
import { NaiveBroadphase } from './NaiveBroadphase'
import { SAPBroadphase } from './SAPBroadphase'
import { AABB } from './AABB'
import { World } from '../world/World'
import { Body } from '../objects/Body'
import { Box } from '../shapes/Box'
import { Plane } from '../shapes/Plane'
import { Sphere } from '../shapes/Sphere'
import { Vec3 } from '../math/Vec3'
import { Quaternion } from '../math/Quaternion'
import type { Broadphase } from './Broadphase'

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

// Body ids are global across worlds, indices are not, so compare with indices.
function pairKey(bodyA: Body, bodyB: Body): string {
  return bodyA.index < bodyB.index ? `${bodyA.index}-${bodyB.index}` : `${bodyB.index}-${bodyA.index}`
}

function pairKeys(broadphase: Broadphase, world: World): string[] {
  const p1: Body[] = []
  const p2: Body[] = []
  broadphase.useBoundingBoxes = true
  broadphase.collisionPairs(world, p1, p2)

  const keys: string[] = []
  for (let i = 0; i < p1.length; i++) {
    keys.push(pairKey(p1[i], p2[i]))
  }
  return keys.sort()
}

function createWorld(broadphase: Broadphase, bodyCount: number, seed: number): World {
  const world = new World({ broadphase, gravity: new Vec3(0, 0, 0) })
  const random = createRandom(seed)

  for (let i = 0; i < bodyCount; i++) {
    const body = new Body({
      mass: 1,
      shape: new Box(new Vec3(0.5, 0.5, 0.5)),
      position: new Vec3(random() * 20 - 10, random() * 20 - 10, random() * 20 - 10),
      quaternion: new Quaternion().setFromAxisAngle(new Vec3(0, 1, 0), random() * Math.PI),
    })
    world.addBody(body)
  }

  return world
}

describe('AABBTreeBroadphase', () => {
  test('finds the same collision pairs as NaiveBroadphase', () => {
    const tree = new AABBTreeBroadphase()
    const naive = new NaiveBroadphase()
    const treeWorld = createWorld(tree, 120, 1234)
    const naiveWorld = createWorld(naive, 120, 1234)

    expect(pairKeys(tree, treeWorld)).toEqual(pairKeys(naive, naiveWorld))
  })

  test('finds the same collision pairs after bodies move', () => {
    const tree = new AABBTreeBroadphase()
    const naive = new NaiveBroadphase()
    const treeWorld = createWorld(tree, 120, 42)
    const naiveWorld = createWorld(naive, 120, 42)

    const random = createRandom(7)
    const move = (world: World) => {
      for (let i = 0; i < world.bodies.length; i++) {
        const body = world.bodies[i]
        body.position.x += random() * 4 - 2
        body.position.y += random() * 4 - 2
        body.position.z += random() * 4 - 2
        body.aabbNeedsUpdate = true
      }
    }

    for (let step = 0; step < 5; step++) {
      move(treeWorld)
      move(naiveWorld)
      naiveWorld.bodies.forEach((body, i) => {
        // Move both worlds identically
        body.position.copy(treeWorld.bodies[i].position)
        body.aabbNeedsUpdate = true
      })

      expect(pairKeys(tree, treeWorld)).toEqual(pairKeys(naive, naiveWorld))
    }
  })

  test('handles planes', () => {
    const tree = new AABBTreeBroadphase()
    const world = new World({ broadphase: tree, gravity: new Vec3(0, 0, 0) })

    const ground = new Body({
      mass: 0,
      shape: new Plane(),
      quaternion: new Quaternion().setFromAxisAngle(new Vec3(1, 0, 0), -Math.PI / 2),
    })
    const ball = new Body({ mass: 1, shape: new Sphere(1), position: new Vec3(0, 0.5, 0) })
    const farAway = new Body({ mass: 1, shape: new Sphere(1), position: new Vec3(0, -100, 0) })

    world.addBody(ground)
    world.addBody(ball)
    world.addBody(farAway)

    const keys = pairKeys(tree, world)
    expect(keys).toContain(pairKey(ground, ball))
    expect(keys).toContain(pairKey(ground, farAway))
    expect(keys).not.toContain(pairKey(ball, farAway))
  })

  test('updates when bodies are added and removed', () => {
    const tree = new AABBTreeBroadphase()
    const world = new World({ broadphase: tree, gravity: new Vec3(0, 0, 0) })

    const bodyA = new Body({ mass: 1, shape: new Sphere(1) })
    const bodyB = new Body({ mass: 1, shape: new Sphere(1), position: new Vec3(1.5, 0, 0) })
    world.addBody(bodyA)
    world.addBody(bodyB)

    expect(pairKeys(tree, world)).toEqual([pairKey(bodyA, bodyB)])

    const bodyC = new Body({ mass: 1, shape: new Sphere(1), position: new Vec3(-1.5, 0, 0) })
    world.addBody(bodyC)
    expect(pairKeys(tree, world)).toEqual([pairKey(bodyA, bodyB), pairKey(bodyA, bodyC)].sort())

    world.removeBody(bodyB)
    expect(pairKeys(tree, world)).toEqual([pairKey(bodyA, bodyC)])

    world.removeBody(bodyA)
    world.removeBody(bodyC)
    expect(pairKeys(tree, world)).toEqual([])
  })

  test('aabbQuery returns the same bodies as NaiveBroadphase', () => {
    const tree = new AABBTreeBroadphase()
    const naive = new NaiveBroadphase()
    const treeWorld = createWorld(tree, 200, 99)
    const naiveWorld = createWorld(naive, 200, 99)

    const random = createRandom(5)
    const query = new AABB()
    for (let i = 0; i < 20; i++) {
      query.lowerBound.set(random() * 20 - 10, random() * 20 - 10, random() * 20 - 10)
      query.upperBound.copy(query.lowerBound)
      query.upperBound.x += 4
      query.upperBound.y += 4
      query.upperBound.z += 4

      const treeResult: Body[] = []
      const naiveResult: Body[] = []
      tree.aabbQuery(treeWorld, query, treeResult)
      naive.aabbQuery(naiveWorld, query, naiveResult)

      const treeIndices = treeResult.map((body) => body.index).sort((a, b) => a - b)
      const naiveIndices = naiveResult.map((body) => body.index).sort((a, b) => a - b)
      expect(treeIndices).toEqual(naiveIndices)
    }
  })

  test('simulates a body resting on a plane', () => {
    const world = new World({ broadphase: new AABBTreeBroadphase(), gravity: new Vec3(0, -9.81, 0) })

    const ground = new Body({
      mass: 0,
      shape: new Plane(),
      quaternion: new Quaternion().setFromAxisAngle(new Vec3(1, 0, 0), -Math.PI / 2),
    })
    const ball = new Body({ mass: 1, shape: new Sphere(0.5), position: new Vec3(0, 5, 0) })

    world.addBody(ground)
    world.addBody(ball)

    for (let i = 0; i < 180; i++) {
      world.step(1 / 60)
    }

    expect(Number.isFinite(ball.position.y)).toBe(true)
    expect(ball.position.y).toBeGreaterThan(0.4)
    expect(ball.position.y).toBeLessThan(0.6)
  })

  test('SAPBroadphase finds the same pairs as NaiveBroadphase', () => {
    const sap = new SAPBroadphase(null as unknown as World)
    const naive = new NaiveBroadphase()
    const sapWorld = createWorld(sap, 80, 321)
    const naiveWorld = createWorld(naive, 80, 321)

    expect(pairKeys(sap, sapWorld)).toEqual(pairKeys(naive, naiveWorld))
  })
})
