import { World } from './World'
import { Vec3 } from '../math/Vec3'
import { Quaternion } from '../math/Quaternion'
import { Body } from '../objects/Body'
import { Box } from '../shapes/Box'
import { Sphere } from '../shapes/Sphere'
import { RaycastResult } from '../collision/RaycastResult'

describe('World shape queries', () => {
  test('intersectShape finds overlapping bodies', () => {
    const world = new World()

    const near = new Body({ mass: 0, shape: new Box(new Vec3(1, 1, 1)) })
    const far = new Body({ mass: 0, shape: new Box(new Vec3(1, 1, 1)), position: new Vec3(10, 0, 0) })
    world.addBody(near)
    world.addBody(far)

    // Overlapping the +X face of the box.
    const bodies = world.intersectShape(new Sphere(0.5), new Vec3(1.3, 0, 0), new Quaternion())

    expect(bodies).toEqual([near])
  })

  test('intersectShape honours ignoreBody and collision filters', () => {
    const world = new World()

    const box = new Body({ mass: 0, shape: new Box(new Vec3(1, 1, 1)) })
    world.addBody(box)

    expect(world.intersectShape(new Sphere(0.5), new Vec3(1.3, 0, 0), new Quaternion(), { ignoreBody: box })).toEqual(
      []
    )

    box.collisionFilterGroup = 2
    expect(world.intersectShape(new Sphere(0.5), new Vec3(1.3, 0, 0), new Quaternion())).toEqual([box])
    expect(
      world.intersectShape(new Sphere(0.5), new Vec3(1.3, 0, 0), new Quaternion(), { collisionFilterMask: 1 })
    ).toEqual([])
  })

  test('castShape finds the first obstacle', () => {
    const world = new World()

    const wall = new Body({
      mass: 0,
      shape: new Box(new Vec3(0.5, 2, 2)),
      position: new Vec3(5, 0, 0),
    })
    world.addBody(wall)

    const result = new RaycastResult()
    const hit = world.castShape(new Sphere(0.5), new Vec3(0, 0, 0), new Vec3(10, 0, 0), new Quaternion(), {}, result)

    expect(hit).toBe(true)
    expect(result.hasHit).toBe(true)
    expect(result.body).toBe(wall)
    // The box starts at x = 4.5, so the sphere of radius 0.5 touches it at x = 4.
    expect(result.distance).toBeGreaterThan(3.9)
    expect(result.distance).toBeLessThan(4.1)
    expect(result.hitNormalWorld.x).toBeLessThan(-0.9)
  })

  test('castShape reports a miss when the path is clear', () => {
    const world = new World()

    world.addBody(
      new Body({
        mass: 0,
        shape: new Box(new Vec3(0.5, 2, 2)),
        position: new Vec3(5, 0, 0),
      })
    )

    const result = new RaycastResult()
    const hit = world.castShape(new Sphere(0.5), new Vec3(0, 5, 0), new Vec3(10, 5, 0), new Quaternion(), {}, result)

    expect(hit).toBe(false)
    expect(result.hasHit).toBe(false)
    expect(result.distance).toBe(-1)
  })

  test('castShape skips ignored bodies', () => {
    const world = new World()

    const wall = new Body({
      mass: 0,
      shape: new Box(new Vec3(0.5, 2, 2)),
      position: new Vec3(5, 0, 0),
    })
    world.addBody(wall)

    const result = new RaycastResult()
    const hit = world.castShape(
      new Sphere(0.5),
      new Vec3(0, 0, 0),
      new Vec3(10, 0, 0),
      new Quaternion(),
      {
        ignoreBody: wall,
      },
      result
    )

    expect(hit).toBe(false)
  })
})
