import { ConvexPolyhedron } from '../shapes/ConvexPolyhedron'
import { Vec3 } from '../math/Vec3'
import { Shape } from './Shape'

/**
 * A capsule: a cylinder of `height` capped by two hemispheres of `radius`.
 *
 * The shape is a tessellated convex polyhedron (`ConvexPolyhedron`), so it works with every
 * shape pair the narrowphase already supports (spheres, boxes, planes, other convex shapes,
 * cylinders, heightfields). Increase `numSegments` / `numCapSegments` for a smoother surface.
 *
 * The local Y axis is the capsule axis and the shape is centered on its origin.
 * Total height is `height + 2 * radius`.
 *
 * @example
 *     const capsule = new CANNON.Capsule(0.4, 1.2, 10, 4)
 *     const body = new CANNON.Body({ mass: 5, shape: capsule })
 *     world.addBody(body)
 */
export class Capsule extends ConvexPolyhedron {
  /** Radius of the capsule. */
  radius: number
  /** Height of the cylindrical part (the caps add `2 * radius`). */
  height: number
  /** Number of segments around the axis. */
  numSegments: number
  /** Number of rings used to tessellate each hemisphere. */
  numCapSegments: number

  /**
   * @param radius Radius of the capsule.
   * @param height Height of the cylindrical part.
   * @param numSegments Number of segments around the axis.
   * @param numCapSegments Number of rings used to tessellate each hemisphere.
   */
  constructor(radius = 0.5, height = 1, numSegments = 8, numCapSegments = 4) {
    if (!(radius > 0)) {
      throw new Error('The capsule radius must be positive.')
    }
    if (height < 0) {
      throw new Error('The capsule height cannot be negative.')
    }

    const N = Math.max(3, numSegments)
    const C = Math.max(1, numCapSegments)

    const vertices: Vec3[] = []
    const faces: number[][] = []
    const rings: number[][] = []

    // Add a ring of N vertices at the given height and radius.
    const addRing = (y: number, r: number): void => {
      const ring: number[] = []
      for (let i = 0; i < N; i++) {
        const theta = ((2 * Math.PI) / N) * i
        ring.push(vertices.length)
        vertices.push(new Vec3(-r * Math.sin(theta), y, r * Math.cos(theta)))
      }
      rings.push(ring)
    }

    // Rings from the bottom cap up to the top cap, poles excluded.
    for (let c = C - 1; c >= 1; c--) {
      const phi = (c / C) * (Math.PI / 2)
      addRing(-height / 2 - radius * Math.sin(phi), radius * Math.cos(phi))
    }
    addRing(-height / 2, radius)
    addRing(height / 2, radius)
    for (let c = 1; c <= C - 1; c++) {
      const phi = (c / C) * (Math.PI / 2)
      addRing(height / 2 + radius * Math.sin(phi), radius * Math.cos(phi))
    }

    const bottomPole = vertices.length
    vertices.push(new Vec3(0, -height / 2 - radius, 0))
    const topPole = vertices.length
    vertices.push(new Vec3(0, height / 2 + radius, 0))

    // Bottom fan.
    const bottomRing = rings[0]
    for (let i = 0; i < N; i++) {
      faces.push([bottomPole, bottomRing[i], bottomRing[(i + 1) % N]])
    }

    // Strips between neighbouring rings. Only the cylindrical strip has planar quads; the
    // hemispherical strips are triangulated so every face stays planar.
    const cylinderStrip = C - 1
    for (let r = 0; r < rings.length - 1; r++) {
      const lower = rings[r]
      const upper = rings[r + 1]
      for (let i = 0; i < N; i++) {
        const next = (i + 1) % N
        if (r === cylinderStrip) {
          faces.push([lower[i], upper[i], upper[next], lower[next]])
        } else {
          faces.push([lower[i], upper[i], upper[next]])
          faces.push([lower[i], upper[next], lower[next]])
        }
      }
    }

    // Top fan.
    const topRing = rings[rings.length - 1]
    for (let i = 0; i < N; i++) {
      faces.push([topRing[i], topPole, topRing[(i + 1) % N]])
    }

    // The bounding radius has to be passed to the constructor: ConvexPolyhedron calls
    // updateBoundingSphereRadius() before the subclass fields are assigned.
    super({ vertices, faces, boundingSphereRadius: height / 2 + radius })

    this.type = Shape.types.CONVEXPOLYHEDRON
    this.radius = radius
    this.height = height
    this.numSegments = N
    this.numCapSegments = C
  }

  /**
   * Volume of the capsule.
   */
  volume(): number {
    return Math.PI * this.radius ** 2 * this.height + (4 / 3) * Math.PI * this.radius ** 3
  }

  /**
   * Computes the bounding sphere radius.
   */
  updateBoundingSphereRadius(): void {
    this.boundingSphereRadius = this.height / 2 + this.radius
  }

  /**
   * Calculates the inertia of the capsule, approximated by the cylinder that encloses it.
   */
  calculateLocalInertia(mass: number, target: Vec3): void {
    const r = this.radius
    const h = this.height + 2 * r
    const radial = (mass * (3 * r * r + h * h)) / 12
    target.set(radial, 0.5 * mass * r * r, radial)
  }
}
