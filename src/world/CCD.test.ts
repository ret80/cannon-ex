import { Vec3 } from '../math/Vec3'
import { Quaternion } from '../math/Quaternion'
import { Box } from '../shapes/Box'
import { Plane } from '../shapes/Plane'
import { Sphere } from '../shapes/Sphere'
import { Body } from '../objects/Body'
import { World } from '../world/World'

function runBulletWorld(options: { ccd?: boolean; disableOnBody?: boolean } = {}) {
  const world = new World({ gravity: new Vec3(0, 0, 0), ccd: options.ccd })

  const wall = new Body({ mass: 0, shape: new Box(new Vec3(0.1, 5, 5)), position: new Vec3(5, 0, 0) })
  const bullet = new Body({ mass: 1, linearDamping: 0, shape: new Sphere(0.5), position: new Vec3(-5, 0, 0) })
  bullet.velocity.set(1000, 0, 0)

  if (options.disableOnBody) {
    bullet.ccdEnabled = false
  }

  world.addBody(wall)
  world.addBody(bullet)

  world.step(1 / 60)

  return { world, wall, bullet }
}

describe('CCD', () => {
  test('a fast body tunnels through a wall without CCD', () => {
    const { bullet } = runBulletWorld({ ccd: false })

    // 1000 units/s * 1/60 s = ~16.7 units, the wall at x = 5 is skipped
    expect(bullet.position.x).toBeGreaterThan(10)
  })

  test('stops a fast body at the wall', () => {
    const { bullet } = runBulletWorld({ ccd: true })

    // The sphere (r=0.5) touches the wall (left face at x=4.9) at x=4.4
    expect(bullet.position.x).toBeGreaterThan(4.3)
    expect(bullet.position.x).toBeLessThan(4.5)
    expect(bullet.velocity.x).toBeCloseTo(0)
  })

  test('can be disabled per body', () => {
    const { bullet } = runBulletWorld({ ccd: true, disableOnBody: true })

    expect(bullet.position.x).toBeGreaterThan(10)
  })

  test('does not change the motion of slow bodies', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0), ccd: true })
    const ball = new Body({ mass: 1, linearDamping: 0, shape: new Sphere(0.5) })
    ball.velocity.set(1, 0, 0)
    world.addBody(ball)

    world.step(1 / 60)

    expect(ball.position.x).toBeCloseTo(1 / 60)
    expect(ball.velocity.x).toBeCloseTo(1)
  })

  test('stops a fast body on a plane', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0), ccd: true })

    const ground = new Body({
      mass: 0,
      shape: new Plane(),
      quaternion: new Quaternion().setFromAxisAngle(new Vec3(1, 0, 0), -Math.PI / 2),
    })
    const ball = new Body({ mass: 1, linearDamping: 0, shape: new Sphere(0.5), position: new Vec3(0, 20, 0) })
    ball.velocity.set(0, -1500, 0)

    world.addBody(ground)
    world.addBody(ball)

    world.step(1 / 60)

    expect(ball.position.y).toBeGreaterThan(0.4)
    expect(ball.position.y).toBeLessThan(1)
    expect(ball.velocity.y).toBeCloseTo(0)
  })

  test('does not slow bodies down when nothing is in the way', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0), ccd: true })
    const bullet = new Body({ mass: 1, linearDamping: 0, shape: new Sphere(0.5), position: new Vec3(-50, 0, 0) })
    bullet.velocity.set(1000, 0, 0)
    world.addBody(bullet)

    world.step(1 / 60)

    expect(bullet.position.x).toBeCloseTo(-50 + 1000 / 60)
    expect(bullet.velocity.x).toBeCloseTo(1000)
  })
})
