import { Cone } from './Cone'
import { ConvexPolyhedron } from './ConvexPolyhedron'
import { Vec3 } from '../math/Vec3'
import { Quaternion } from '../math/Quaternion'
import { World } from '../world/World'
import { Body } from '../objects/Body'
import { Plane } from './Plane'
import { Sphere } from './Sphere'

describe('Cone', () => {
  test('is a convex polyhedron with outward face normals', () => {
    const shape = new Cone(0.5, 1.5, 10)

    expect(shape).toBeInstanceOf(ConvexPolyhedron)

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

  test('volume and bounding sphere', () => {
    const shape = new Cone(0.5, 1.5, 12)

    expect(shape.volume()).toBeCloseTo((Math.PI * 0.25 * 1.5) / 3)
    expect(shape.boundingSphereRadius).toBeCloseTo(Math.sqrt(0.25 + 0.5625))
  })

  test('rests on its base on a plane', () => {
    const world = new World({ gravity: new Vec3(0, -9.81, 0) })

    const ground = new Body({
      mass: 0,
      shape: new Plane(),
      quaternion: new Quaternion().setFromAxisAngle(new Vec3(1, 0, 0), -Math.PI / 2),
    })
    const cone = new Body({
      mass: 1,
      shape: new Cone(0.4, 1.5, 10),
      position: new Vec3(0, 2, 0),
      linearDamping: 0,
    })

    world.addBody(ground)
    world.addBody(cone)

    for (let i = 0; i < 240; i++) {
      world.step(1 / 60)
    }

    // The base sits on the plane, the origin is at half the height.
    expect(cone.position.y).toBeGreaterThan(0.6)
    expect(cone.position.y).toBeLessThan(0.95)
    expect(Math.abs(cone.quaternion.x)).toBeLessThan(0.2)
    expect(Math.abs(cone.quaternion.z)).toBeLessThan(0.2)
  })

  test('collides with a sphere', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0) })

    const cone = new Body({ mass: 0, shape: new Cone(0.5, 1.5, 10) })
    const sphere = new Body({
      mass: 1,
      shape: new Sphere(0.3),
      position: new Vec3(1, 0, 0),
      velocity: new Vec3(-1, 0, 0),
      linearDamping: 0,
    })

    world.addBody(cone)
    world.addBody(sphere)

    for (let i = 0; i < 120; i++) {
      world.step(1 / 60)
    }

    expect(sphere.position.x).toBeGreaterThan(0.2)
  })
})
