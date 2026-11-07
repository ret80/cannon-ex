#!/usr/bin/env node
/**
 * Head to head benchmark: this engine vs the original cannon-es 0.20.0 (pmndrs).
 *
 * The original package is expected in .baseline/node_modules/cannon-es (gitignored).
 * Install it once with:
 *   mkdir -p .baseline && cd .baseline && npm init -y && npm install cannon-es@0.20.0
 *
 * Run with:
 *   node benchmark/compare-original.js
 */
const fs = require('fs')
const path = require('path')

const baselinePath = path.join(__dirname, '..', '.baseline', 'node_modules', 'cannon-es')
if (!fs.existsSync(baselinePath)) {
  console.error('Original cannon-es not found in .baseline/node_modules/cannon-es')
  console.error('Install it with: mkdir -p .baseline && cd .baseline && npm init -y && npm i cannon-es@0.20.0')
  process.exit(1)
}

require('ts-node').register({
  transpileOnly: true,
  compilerOptions: { module: 'commonjs', declaration: false, emitDeclarationOnly: false },
})

const Ours = require('../src/cannon-es.ts')
const Original = require(baselinePath)

const RUNS = 3
const WARMUP = 1

function hrtime() {
  return Number(process.hrtime.bigint()) / 1e6
}

/**
 * Build a world for the given scenario and measure the time of `steps` ticks.
 * Returns the best (minimum) duration of RUNS runs, in ms.
 */
function measure(module, setup, steps) {
  let best = Infinity

  for (let run = 0; run < WARMUP + RUNS; run++) {
    const { world, tick } = setup(module)
    const start = hrtime()
    for (let i = 0; i < steps; i++) {
      tick(world)
    }
    const elapsed = hrtime() - start
    if (run >= WARMUP && elapsed < best) {
      best = elapsed
    }
  }

  return best
}

function setBroadphase(world, module, kind) {
  if (kind === 'naive') {
    world.broadphase = new module.NaiveBroadphase()
  } else if (kind === 'sap') {
    world.broadphase = new module.SAPBroadphase(world)
  }
  // kind === 'default': keep the engine default
}

const scenarios = {
  'stack 125 boxes': {
    steps: 100,
    setup(module) {
      const world = new module.World({ gravity: new module.Vec3(0, -9.81, 0) })
      const ground = new module.Body({
        mass: 0,
        shape: new module.Plane(),
        quaternion: new module.Quaternion().setFromAxisAngle(new module.Vec3(1, 0, 0), -Math.PI / 2),
      })
      world.addBody(ground)

      for (let layer = 0; layer < 5; layer++) {
        for (let x = 0; x < 5; x++) {
          for (let z = 0; z < 5; z++) {
            world.addBody(
              new module.Body({
                mass: 1,
                shape: new module.Box(new module.Vec3(0.5, 0.5, 0.5)),
                position: new module.Vec3(x - 2, 0.55 + layer * 1.02, z - 2),
              })
            )
          }
        }
      }

      return { world, tick: (w) => w.step(1 / 60) }
    },
  },

  '400 falling spheres': {
    steps: 100,
    setup(module) {
      const world = new module.World({ gravity: new module.Vec3(0, -9.81, 0) })
      world.addBody(
        new module.Body({
          mass: 0,
          shape: new module.Plane(),
          quaternion: new module.Quaternion().setFromAxisAngle(new module.Vec3(1, 0, 0), -Math.PI / 2),
        })
      )

      for (let i = 0; i < 400; i++) {
        world.addBody(
          new module.Body({
            mass: 1,
            shape: new module.Sphere(0.4),
            position: new module.Vec3((i % 20) - 10 + 0.1, 1 + Math.floor(i / 20) * 0.9, (((i / 20) | 0) % 20) - 10),
          })
        )
      }

      return { world, tick: (w) => w.step(1 / 60) }
    },
  },

  '500 boxes + 1000 rays': {
    steps: 1000,
    setup(module) {
      const world = new module.World()
      for (let i = 0; i < 500; i++) {
        world.addBody(
          new module.Body({
            mass: 0,
            shape: new module.Box(new module.Vec3(0.5, 0.5, 0.5)),
            position: new module.Vec3((i % 25) * 2 - 25, 0, Math.floor(i / 25) * 2 - 20),
          })
        )
      }

      const from = new module.Vec3()
      const to = new module.Vec3()
      const result = new module.RaycastResult()
      let hits = 0

      return {
        world,
        tick: () => {
          from.set((hits % 25) * 2 - 25, 10, ((hits * 7) % 500) / 25 - 10)
          to.set(from.x, -10, from.z)
          if (world.raycastClosest(from, to, { skipBackfaces: true }, result)) hits++
        },
      }
    },
  },

  '800 bodies, pair generation': {
    steps: 100,
    setup(module) {
      const world = new module.World()
      const pairs1 = []
      const pairs2 = []

      // Dynamic bodies so the broadphase does not skip static-static pairs.
      for (let i = 0; i < 800; i++) {
        const body = new module.Body({
          mass: 1,
          shape: new module.Sphere(0.3),
          position: new module.Vec3(((i * 37) % 200) / 10 - 10, ((i * 91) % 200) / 10 - 10, ((i * 53) % 200) / 10 - 10),
        })
        world.addBody(body)
      }

      return {
        world,
        tick: (w) => {
          pairs1.length = 0
          pairs2.length = 0
          w.broadphase.collisionPairs(w, pairs1, pairs2)
        },
      }
    },
  },
}

const configs = [
  { label: 'default', kind: 'default' },
  { label: 'SAP', kind: 'sap' },
  { label: 'Naive', kind: 'naive' },
]

function runEngine(name, module) {
  const rows = []

  for (const [scenarioName, scenario] of Object.entries(scenarios)) {
    const row = { scenario: scenarioName }
    for (const config of configs) {
      row[config.label] = measure(
        module,
        (m) => {
          const built = scenario.setup(m)
          setBroadphase(built.world, m, config.kind)
          return built
        },
        scenario.steps
      )
    }
    rows.push(row)
  }

  return { name, rows }
}

const results = [runEngine('this engine', Ours), runEngine('cannon-es 0.20.0', Original)]

function fmt(ms) {
  return ms.toFixed(2).padStart(9)
}

for (const { name, rows } of results) {
  console.log(`\n${name}`)
  console.log('scenario'.padEnd(28) + 'default'.padStart(9) + 'SAP'.padStart(11) + 'Naive'.padStart(11))
  for (const row of rows) {
    console.log(row.scenario.padEnd(28) + fmt(row.default) + fmt(row.SAP) + fmt(row.Naive))
  }
}

console.log('\nMilliseconds for the whole scenario (lower is better).')
console.log('This engine default broadphase: AABBTreeBroadphase. Original default: NaiveBroadphase.')
