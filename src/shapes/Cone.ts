import { ConvexPolyhedron } from '../shapes/ConvexPolyhedron'
import { Vec3 } from '../math/Vec3'
import { Shape } from './Shape'

/**
 * A cone with its apex pointing along the local +Y axis, centered on its origin.
 *
 * Like `Cylinder` it is a convex polyhedron, so all the convex narrowphase pairs work with it.
 *
 * @example
 *     const cone = new CANNON.Cone(0.5, 1.5, 12)
 *     const body = new CANNON.Body({ mass: 1, shape: cone })
 *     world.addBody(body)
 */
export class Cone extends ConvexPolyhedron {
  /** Radius of the base. */
  radius: number
  /** Height of the cone. */
  height: number
  /** Number of segments used for the base. */
  numSegments: number

  /**
   * @param radius Radius of the base.
   * @param height Height of the cone.
   * @param numSegments Number of segments used for the base.
   */
  constructor(radius = 1, height = 2, numSegments = 8) {
    if (!(radius > 0)) {
      throw new Error('The cone radius must be positive.')
    }
    if (!(height > 0)) {
      throw new Error('The cone height must be positive.')
    }

    const N = Math.max(3, numSegments)
    const vertices: Vec3[] = []
    const faces: number[][] = []
    const base: number[] = []

    for (let i = 0; i < N; i++) {
      const theta = ((2 * Math.PI) / N) * i
      base.push(vertices.length)
      vertices.push(new Vec3(-radius * Math.sin(theta), -height / 2, radius * Math.cos(theta)))
    }

    const apex = vertices.length
    vertices.push(new Vec3(0, height / 2, 0))

    for (let i = 0; i < N; i++) {
      faces.push([base[i], apex, base[(i + 1) % N]])
    }
    faces.push(base)

    super({ vertices, faces, boundingSphereRadius: Math.sqrt(radius * radius + (height * height) / 4) })

    this.type = Shape.types.CONVEXPOLYHEDRON
    this.radius = radius
    this.height = height
    this.numSegments = N
  }

  /**
   * Volume of the cone.
   */
  volume(): number {
    return (Math.PI * this.radius ** 2 * this.height) / 3
  }

  /**
   * Computes the bounding sphere radius.
   */
  updateBoundingSphereRadius(): void {
    this.boundingSphereRadius = Math.sqrt(this.radius ** 2 + this.height ** 2 / 4)
  }

  /**
   * Calculates the inertia of the cone about its origin.
   * @see http://en.wikipedia.org/wiki/List_of_moments_of_inertia
   */
  calculateLocalInertia(mass: number, target: Vec3): void {
    const r = this.radius
    const h = this.height
    const radial = mass * ((3 * r * r) / 20 + (3 * h * h) / 80)
    target.set(radial, (3 * mass * r * r) / 10, radial)
  }
}
