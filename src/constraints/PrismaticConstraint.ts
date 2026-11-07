import { Constraint } from '../constraints/Constraint'
import { ContactEquation } from '../equations/ContactEquation'
import { RotationalEquation } from '../equations/RotationalEquation'
import { Vec3 } from '../math/Vec3'
import type { Body } from '../objects/Body'

export type PrismaticConstraintOptions = {
  /** Pivot point in bodyA, in bodyA's local frame. Defaults to the midpoint between the bodies. */
  pivotA?: Vec3
  /** Pivot point in bodyB, in bodyB's local frame. Defaults to the midpoint between the bodies. */
  pivotB?: Vec3
  /** The sliding axis, in bodyA's local frame. @default (1, 0, 0) */
  axis?: Vec3
  /** Maximum force applied to keep the bodies together. @default 1e6 */
  maxForce?: number
  /** Set to false if the connected bodies should not collide. @default true */
  collideConnected?: boolean
}

/**
 * A slider (prismatic) joint. Allows one translational degree of freedom along an axis and
 * removes the other two translations and all three rotations.
 *
 * Joint limits and motors are not implemented.
 *
 * @example
 *     const bodyA = new Body({ mass: 0 }) // static rail
 *     const bodyB = new Body({ mass: 1, position: new Vec3(1, 0, 0) })
 *     world.addBody(bodyA)
 *     world.addBody(bodyB)
 *     world.addConstraint(new PrismaticConstraint(bodyA, bodyB, { axis: new Vec3(1, 0, 0) }))
 */
export class PrismaticConstraint extends Constraint {
  /** Pivot point in bodyA, local frame. */
  pivotA: Vec3
  /** Pivot point in bodyB, local frame. */
  pivotB: Vec3
  /** The sliding axis, in bodyA's local frame. */
  axisA: Vec3

  /** Constraint along the first axis perpendicular to the slider axis. */
  equationX: ContactEquation
  /** Constraint along the second axis perpendicular to the slider axis. */
  equationY: ContactEquation

  /** Rotation lock, bodyA's X axis against bodyB's Y axis. */
  rotationalEquation1: RotationalEquation
  /** Rotation lock, bodyA's Y axis against bodyB's Z axis. */
  rotationalEquation2: RotationalEquation
  /** Rotation lock, bodyA's Z axis against bodyB's X axis. */
  rotationalEquation3: RotationalEquation

  xA: Vec3
  xB: Vec3
  yA: Vec3
  yB: Vec3
  zA: Vec3
  zB: Vec3

  /** World oriented sliding axis. */
  worldAxis: Vec3
  /** First world oriented axis perpendicular to the sliding axis. */
  worldNormal1: Vec3
  /** Second world oriented axis perpendicular to the sliding axis. */
  worldNormal2: Vec3

  constructor(bodyA: Body, bodyB: Body, options: PrismaticConstraintOptions = {}) {
    super(bodyA, bodyB, { collideConnected: options.collideConnected })

    const maxForce = options.maxForce !== undefined ? options.maxForce : 1e6

    const midpoint = new Vec3()
    if (options.pivotA && options.pivotB) {
      this.pivotA = options.pivotA.clone()
      this.pivotB = options.pivotB.clone()
    } else {
      bodyA.position.vadd(bodyB.position, midpoint)
      midpoint.scale(0.5)
      this.pivotA = options.pivotA ? options.pivotA.clone() : bodyA.pointToLocalFrame(midpoint, new Vec3())
      this.pivotB = options.pivotB ? options.pivotB.clone() : bodyB.pointToLocalFrame(midpoint, new Vec3())
    }

    this.axisA = options.axis ? options.axis.clone() : new Vec3(1, 0, 0)

    // Rotations of the bodies at construction time, kept orthogonal by the rotational equations.
    this.xA = bodyA.vectorToLocalFrame(Vec3.UNIT_X)
    this.xB = bodyB.vectorToLocalFrame(Vec3.UNIT_X)
    this.yA = bodyA.vectorToLocalFrame(Vec3.UNIT_Y)
    this.yB = bodyB.vectorToLocalFrame(Vec3.UNIT_Y)
    this.zA = bodyA.vectorToLocalFrame(Vec3.UNIT_Z)
    this.zB = bodyB.vectorToLocalFrame(Vec3.UNIT_Z)

    const x = (this.equationX = new ContactEquation(bodyA, bodyB))
    const y = (this.equationY = new ContactEquation(bodyA, bodyB))

    x.minForce = y.minForce = -maxForce
    x.maxForce = y.maxForce = maxForce

    const r1 = (this.rotationalEquation1 = new RotationalEquation(bodyA, bodyB, { maxForce }))
    const r2 = (this.rotationalEquation2 = new RotationalEquation(bodyA, bodyB, { maxForce }))
    const r3 = (this.rotationalEquation3 = new RotationalEquation(bodyA, bodyB, { maxForce }))

    this.equations.push(x, y, r1, r2, r3)

    this.worldAxis = new Vec3()
    this.worldNormal1 = new Vec3()
    this.worldNormal2 = new Vec3()
  }

  /**
   * Update the constraint equations with the current body transforms.
   */
  update(): void {
    const bodyA = this.bodyA
    const bodyB = this.bodyB
    const x = this.equationX
    const y = this.equationY
    const r1 = this.rotationalEquation1
    const r2 = this.rotationalEquation2
    const r3 = this.rotationalEquation3

    // Keep the pivots at the same point in the two directions perpendicular to the axis.
    bodyA.quaternion.vmult(this.pivotA, x.ri)
    bodyB.quaternion.vmult(this.pivotB, x.rj)
    y.ri.copy(x.ri)
    y.rj.copy(x.rj)

    const axis = bodyA.vectorToWorldFrame(this.axisA, this.worldAxis)
    axis.normalize()

    // Build an orthonormal basis with the sliding axis.
    if (Math.abs(axis.x) < 0.9) {
      axis.cross(Vec3.UNIT_X, this.worldNormal1)
    } else {
      axis.cross(Vec3.UNIT_Y, this.worldNormal1)
    }
    this.worldNormal1.normalize()
    axis.cross(this.worldNormal1, this.worldNormal2)
    this.worldNormal2.normalize()

    x.ni.copy(this.worldNormal1)
    y.ni.copy(this.worldNormal2)

    // Lock the relative rotation.
    bodyA.vectorToWorldFrame(this.xA, r1.axisA)
    bodyB.vectorToWorldFrame(this.yB, r1.axisB)

    bodyA.vectorToWorldFrame(this.yA, r2.axisA)
    bodyB.vectorToWorldFrame(this.zB, r2.axisB)

    bodyA.vectorToWorldFrame(this.zA, r3.axisA)
    bodyB.vectorToWorldFrame(this.xB, r3.axisB)
  }
}
