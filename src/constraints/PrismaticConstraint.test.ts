import { World } from '../world/World'
import { Body } from '../objects/Body'
import { PrismaticConstraint } from './PrismaticConstraint'
import { Vec3 } from '../math/Vec3'
import { Box } from '../shapes/Box'

describe('PrismaticConstraint', () => {
  test('slides along the axis and blocks the other translations', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0) })
    const rail = new Body({ mass: 0 })
    const slider = new Body({ mass: 1 })
    world.addBody(rail)
    world.addBody(slider)

    world.addConstraint(new PrismaticConstraint(rail, slider, { axis: new Vec3(1, 0, 0) }))

    slider.velocity.set(2, 1, 0)

    for (let i = 0; i < 120; i++) {
      world.step(1 / 60)
    }

    expect(slider.position.x).toBeGreaterThan(0.5)
    expect(Math.abs(slider.position.y)).toBeLessThan(0.05)
    expect(Math.abs(slider.position.z)).toBeLessThan(0.05)
  })

  test('locks the relative rotation', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0) })
    const shape = new Box(new Vec3(0.5, 0.5, 0.5))
    const rail = new Body({ mass: 0, shape })
    const slider = new Body({ mass: 1, shape })
    world.addBody(rail)
    world.addBody(slider)

    world.addConstraint(new PrismaticConstraint(rail, slider, { axis: new Vec3(1, 0, 0) }))

    slider.angularVelocity.set(0, 5, 0)

    for (let i = 0; i < 60; i++) {
      world.step(1 / 60)
    }

    expect(Math.abs(slider.quaternion.x)).toBeLessThan(0.05)
    expect(Math.abs(slider.quaternion.y)).toBeLessThan(0.05)
    expect(Math.abs(slider.quaternion.z)).toBeLessThan(0.05)
  })

  test('works with pivots and a tilted axis', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0) })
    const rail = new Body({ mass: 0 })
    // Start the slider one unit along the local Y axis.
    const slider = new Body({ mass: 1, position: new Vec3(0, 1, 0) })
    world.addBody(rail)
    world.addBody(slider)

    world.addConstraint(
      new PrismaticConstraint(rail, slider, {
        axis: new Vec3(0, 1, 0),
        pivotA: new Vec3(0, 0, 0),
        pivotB: new Vec3(0, 0, 0),
      })
    )

    slider.velocity.set(1, 2, 0)

    for (let i = 0; i < 60; i++) {
      world.step(1 / 60)
    }

    expect(slider.position.y).toBeGreaterThan(1.3)
    expect(Math.abs(slider.position.x)).toBeLessThan(0.05)
  })
})
