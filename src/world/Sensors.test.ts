import { Vec3 } from '../math/Vec3'
import { Box } from '../shapes/Box'
import { Sphere } from '../shapes/Sphere'
import { Body } from '../objects/Body'
import { World } from '../world/World'

describe('Sensors (triggers)', () => {
  test('emits enter and exit events without generating contacts or forces', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0) })

    const sensor = new Body({ isTrigger: true, shape: new Box(new Vec3(1, 1, 1)) })
    const ball = new Body({ mass: 1, linearDamping: 0, shape: new Sphere(0.25), position: new Vec3(-4, 0, 0) })
    ball.velocity.set(4, 0, 0)

    world.addBody(sensor)
    world.addBody(ball)

    const worldEvents: string[] = []
    world.addEventListener('beginTrigger', (event: any) => {
      worldEvents.push(`begin:${event.bodyA.id}:${event.bodyB.id}`)
    })
    world.addEventListener('endTrigger', (event: any) => {
      worldEvents.push(`end:${event.bodyA.id}:${event.bodyB.id}`)
    })

    let ballEnters = 0
    let ballExits = 0
    ball.addEventListener('triggerEnter', (event: any) => {
      expect(event.body).toBe(sensor)
      ballEnters++
    })
    ball.addEventListener('triggerExit', (event: any) => {
      expect(event.body).toBe(sensor)
      ballExits++
    })

    let sensorEnters = 0
    sensor.addEventListener('triggerEnter', (event: any) => {
      expect(event.body).toBe(ball)
      sensorEnters++
    })

    let collideEvents = 0
    ball.addEventListener('collide', () => collideEvents++)

    let maxContacts = 0
    for (let i = 0; i < 120; i++) {
      world.step(1 / 60)
      maxContacts = Math.max(maxContacts, world.contacts.length)
    }

    expect(worldEvents.length).toBe(2)
    expect(worldEvents[0]).toBe(`begin:${sensor.id}:${ball.id}`)
    expect(worldEvents[1]).toBe(`end:${sensor.id}:${ball.id}`)
    expect(ballEnters).toBe(1)
    expect(ballExits).toBe(1)
    expect(sensorEnters).toBe(1)

    // A sensor must never generate contacts, forces or collide events
    expect(maxContacts).toBe(0)
    expect(collideEvents).toBe(0)
    expect(ball.velocity.x).toBeCloseTo(4)
    expect(ball.position.x).toBeGreaterThan(3)
  })

  test('a trigger body does not stop a moving body', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0) })

    const sensor = new Body({ isTrigger: true, shape: new Box(new Vec3(1, 1, 1)) })
    const box = new Body({
      mass: 1,
      linearDamping: 0,
      shape: new Box(new Vec3(0.5, 0.5, 0.5)),
      position: new Vec3(-2, 0, 0),
    })
    box.velocity.set(3, 0, 0)

    world.addBody(sensor)
    world.addBody(box)

    for (let i = 0; i < 60; i++) {
      world.step(1 / 60)
    }

    expect(box.position.x).toBeCloseTo(1, 5)
    expect(box.velocity.x).toBeCloseTo(3)
  })

  test('a regular (non trigger) body still collides', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0) })

    const wall = new Body({ mass: 0, shape: new Box(new Vec3(0.5, 5, 5)) })
    const ball = new Body({ mass: 1, shape: new Sphere(0.5), position: new Vec3(-2, 0, 0) })
    ball.velocity.set(4, 0, 0)

    world.addBody(wall)
    world.addBody(ball)

    for (let i = 0; i < 120; i++) {
      world.step(1 / 60)
    }

    // The ball must be stopped by the wall
    expect(ball.position.x).toBeLessThan(0)
  })

  test('a fast body can still tunnel through a sensor', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0), ccd: true })

    const sensor = new Body({ isTrigger: true, shape: new Box(new Vec3(1, 1, 1)) })
    const bullet = new Body({ mass: 1, linearDamping: 0, shape: new Sphere(0.25), position: new Vec3(-5, 0, 0) })
    bullet.velocity.set(1000, 0, 0)

    world.addBody(sensor)
    world.addBody(bullet)

    for (let i = 0; i < 30; i++) {
      world.step(1 / 60)
    }

    // Sensors are not solid, so the bullet keeps going
    expect(bullet.position.x).toBeGreaterThan(5)
    expect(bullet.velocity.x).toBeCloseTo(1000)
  })
})
