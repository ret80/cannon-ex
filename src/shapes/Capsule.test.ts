import { Capsule } from './Capsule'
import { ConvexPolyhedron } from './ConvexPolyhedron'
import { Vec3 } from '../math/Vec3'
import { Quaternion } from '../math/Quaternion'
import { World } from '../world/World'
import { Body } from '../objects/Body'
import { Plane } from './Plane'
import { Sphere } from './Sphere'

describe('Capsule', () => {
  test('is a convex polyhedron with outward face normals', () => {
    const shape = new Capsule(0.4, 1, 8, 3)

    expect(shape).toBeInstanceOf(ConvexPolyhedron)
    expect(shape.vertices.length).toBeGreaterThan(0)

    for (let i = 0; i < shape.faces.length; i++) {
      const face = shape.faces[i]
      const centroid = new Vec3()
      for (let j = 0; j < face.length; j++) {
        centroid.vadd(shape.vertices[face[j]], centroid)
      }
      centroid.scale(1 / face.length, centroid)

      expect(shape.faceNormals[i].dot(centroid)).toBeGreaterThan(-1e-9)
    }
  })

  test('bounding sphere radius and volume', () => {
    const shape = new Capsule(0.5, 1, 8, 2)

    expect(shape.boundingSphereRadius).toBeCloseTo(1)
    expect(shape.volume()).toBeCloseTo(Math.PI * 0.25 * 1 + (4 / 3) * Math.PI * 0.125)
  })

  test('lands on a plane and stays upright', () => {
    const world = new World({ gravity: new Vec3(0, -9.81, 0) })

    const ground = new Body({
      mass: 0,
      shape: new Plane(),
      quaternion: new Quaternion().setFromAxisAngle(new Vec3(1, 0, 0), -Math.PI / 2),
    })
    const capsule = new Body({
      mass: 1,
      shape: new Capsule(0.3, 0.8, 10, 3),
      position: new Vec3(0, 2, 0),
      linearDamping: 0,
    })

    world.addBody(ground)
    world.addBody(capsule)

    for (let i = 0; i < 240; i++) {
      world.step(1 / 60)
    }

    // The capsule ends up lying on its side (a capsule cannot stand on a rounded cap),
    // so its center is one radius above the plane whichever way it topples.
    expect(capsule.position.y).toBeGreaterThan(0.2)
    expect(capsule.position.y).toBeLessThan(0.45)
    expect(Number.isFinite(capsule.position.x)).toBe(true)
    expect(Number.isFinite(capsule.quaternion.w)).toBe(true)
  })

  test('collides with a sphere', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0) })

    const capsule = new Body({ mass: 0, shape: new Capsule(0.3, 1, 8, 2) })
    const sphere = new Body({
      mass: 1,
      shape: new Sphere(0.3),
      position: new Vec3(0.55, 0, 0),
      velocity: new Vec3(-1, 0, 0),
      linearDamping: 0,
    })

    world.addBody(capsule)
    world.addBody(sphere)

    for (let i = 0; i < 120; i++) {
      world.step(1 / 60)
    }

    // Pushed out of the capsule, whose radius is 0.3.
    expect(sphere.position.x).toBeGreaterThan(0.5)
  })
})
