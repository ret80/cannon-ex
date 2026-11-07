import { Broadphase } from '../collision/Broadphase'
import { AABB } from '../collision/AABB'
import type { Body } from '../objects/Body'
import type { World } from '../world/World'

/**
 * A node in the dynamic AABB tree.
 * Leaf nodes hold a body, internal nodes hold two children.
 */
class TreeNode {
  /** Fat AABB, used for the tree. Leaves are padded so they rarely have to be reinserted. */
  aabb: AABB
  parent: TreeNode | null
  child1: TreeNode | null
  child2: TreeNode | null
  height: number
  body: Body | null

  constructor() {
    this.aabb = new AABB()
    this.parent = null
    this.child1 = null
    this.child2 = null
    this.height = 0
    this.body = null
  }

  get isLeaf(): boolean {
    return this.child1 === null
  }
}

/**
 * Broadphase that keeps all bodies in a dynamic (incrementally updated) AABB tree,
 * also known as a bounding volume hierarchy.
 *
 * Unlike {@link SAPBroadphase}, which only prunes along a single axis, the tree
 * prunes in all three dimensions, so it scales better with large, sparse worlds,
 * large objects (such as planes) and highly rotated configurations. It also gives
 * `O(log n)` overlap queries and raycasts instead of the `O(n)` scan that
 * SAPBroadphase performs.
 *
 * Leaves use fat AABBs (tight AABB + a small margin + a velocity prediction) so
 * that an insertion only has to be redone when a body leaves its padded volume.
 * The tree is kept balanced with incremental rotations, so no full rebuild is
 * ever needed.
 *
 * @example
 *     const world = new CANNON.World({ broadphase: new CANNON.AABBTreeBroadphase() })
 */
export class AABBTreeBroadphase extends Broadphase {
  /**
   * Root of the tree.
   */
  root: TreeNode | null

  /**
   * Extra padding, in world units, added to every leaf AABB.
   * Larger values mean fewer reinsertions, but more candidate pairs.
   * @default 0.1
   */
  margin: number

  /**
   * The world this broadphase belongs to.
   */
  world: World | null

  private bodyToLeaf: Map<Body, TreeNode>
  private nodePool: TreeNode[]

  private _addBodyHandler: (event: { body: Body }) => void
  private _removeBodyHandler: (event: { body: Body }) => void

  constructor(world?: World) {
    super()

    this.root = null
    this.margin = 0.1
    this.world = null
    this.bodyToLeaf = new Map()
    this.nodePool = []

    // The tree is built from AABBs, so the pair test should use them too.
    this.useBoundingBoxes = true

    this._addBodyHandler = (event: { body: Body }) => {
      this.insertBody(event.body)
    }

    this._removeBodyHandler = (event: { body: Body }) => {
      this.removeBody(event.body)
    }

    if (world) {
      this.setWorld(world)
    }
  }

  /**
   * Change the world.
   */
  setWorld(world: World): void {
    if (this.world) {
      this.world.removeEventListener('addBody', this._addBodyHandler)
      this.world.removeEventListener('removeBody', this._removeBodyHandler)
    }

    this.root = null
    this.bodyToLeaf.clear()
    this.nodePool.length = 0
    this.world = world

    world.addEventListener('addBody', this._addBodyHandler)
    world.addEventListener('removeBody', this._removeBodyHandler)

    for (let i = 0; i < world.bodies.length; i++) {
      this.insertBody(world.bodies[i])
    }

    this.dirty = true
  }

  /**
   * Add a body to the tree.
   */
  insertBody(body: Body): void {
    if (this.bodyToLeaf.has(body)) {
      return
    }

    if (body.aabbNeedsUpdate) {
      body.updateAABB()
    }

    const leaf = this.createNode()
    leaf.body = body
    this.computeFatAABB(body, leaf.aabb)

    this.bodyToLeaf.set(body, leaf)
    this.insertLeaf(leaf)
    this.dirty = true
  }

  /**
   * Remove a body from the tree.
   */
  removeBody(body: Body): void {
    const leaf = this.bodyToLeaf.get(body)
    if (!leaf) {
      return
    }

    this.removeLeaf(leaf)
    this.releaseNode(leaf)
    this.bodyToLeaf.delete(body)
    this.dirty = true
  }

  /**
   * Compute all collision pairs using the tree.
   */
  collisionPairs(world: World, pairs1: Body[], pairs2: Body[]): void {
    if (this.world !== world) {
      this.setWorld(world)
    }

    this.updateLeaves()

    const root = this.root
    if (root) {
      this.selfCheck(root, pairs1, pairs2)
    }
  }

  /**
   * Walk each subtree and cross-check its two halves, so every pair of leaves is
   * tested exactly once (at its lowest common ancestor) instead of one tree query
   * per body.
   */
  private selfCheck(node: TreeNode, pairs1: Body[], pairs2: Body[]): void {
    if (node.isLeaf) {
      return
    }

    const child1 = node.child1 as TreeNode
    const child2 = node.child2 as TreeNode

    this.selfCheck(child1, pairs1, pairs2)
    this.selfCheck(child2, pairs1, pairs2)
    this.crossCheck(child1, child2, pairs1, pairs2)
  }

  /**
   * Check all leaf pairs between two nodes.
   */
  private crossCheck(a: TreeNode, b: TreeNode, pairs1: Body[], pairs2: Body[]): void {
    if (!a.aabb.overlaps(b.aabb)) {
      return
    }

    if (a.isLeaf && b.isLeaf) {
      const bodyA = a.body as Body
      const bodyB = b.body as Body

      if (bodyA !== bodyB && this.needBroadphaseCollision(bodyA, bodyB)) {
        this.intersectionTest(bodyA, bodyB, pairs1, pairs2)
      }
      return
    }

    // Descend into the larger node first
    if (b.isLeaf || (!a.isLeaf && surfaceArea(a.aabb) >= surfaceArea(b.aabb))) {
      this.crossCheck(a.child1 as TreeNode, b, pairs1, pairs2)
      this.crossCheck(a.child2 as TreeNode, b, pairs1, pairs2)
    } else {
      this.crossCheck(a, b.child1 as TreeNode, pairs1, pairs2)
      this.crossCheck(a, b.child2 as TreeNode, pairs1, pairs2)
    }
  }

  /**
   * Returns all the bodies within the AABB. Traverses only the part of the tree
   * that overlaps the query, which is `O(log n + k)` when bodies are sparse.
   */
  aabbQuery(world: World, aabb: AABB, result: Body[] = []): Body[] {
    if (this.world !== world) {
      this.setWorld(world)
    }

    // The leaves are refreshed once per step (in collisionPairs). Only pay for a
    // refresh when bodies may have moved since then, e.g. after integration.
    if (this.dirty) {
      this.updateLeaves()
    }

    const root = this.root
    if (!root) {
      return result
    }

    const stack: TreeNode[] = [root]
    while (stack.length) {
      const node = stack.pop() as TreeNode

      if (!node.aabb.overlaps(aabb)) {
        continue
      }

      if (node.isLeaf) {
        const body = node.body
        if (body) {
          if (body.aabbNeedsUpdate) {
            body.updateAABB()
          }
          if (body.aabb.overlaps(aabb)) {
            result.push(body)
          }
        }
      } else {
        stack.push(node.child1 as TreeNode)
        stack.push(node.child2 as TreeNode)
      }
    }

    return result
  }

  /**
   * Refresh the leaves of all bodies. Leaves that still contain the tight AABB
   * of their body are left untouched.
   */
  private updateLeaves(): void {
    const world = this.world
    if (!world) {
      return
    }

    const bodies = world.bodies
    for (let i = 0, N = bodies.length; i !== N; i++) {
      const body = bodies[i]
      if (body.aabbNeedsUpdate) {
        body.updateAABB()
      }

      let leaf = this.bodyToLeaf.get(body)
      if (!leaf) {
        leaf = this.createNode()
        leaf.body = body
        this.bodyToLeaf.set(body, leaf)
        this.computeFatAABB(body, leaf.aabb)
        this.insertLeaf(leaf)
      } else if (!containsAABB(leaf.aabb, body.aabb)) {
        this.removeLeaf(leaf)
        this.computeFatAABB(body, leaf.aabb)
        this.insertLeaf(leaf)
      }
    }

    this.dirty = false
  }

  /**
   * Compute the padded, velocity-extended AABB of a body.
   */
  private computeFatAABB(body: Body, target: AABB): void {
    const tight = body.aabb
    const dt = this.world && this.world.dt > 0 ? this.world.dt : 0
    const margin = this.margin

    // Predict a few steps ahead, not just one, so a moving body does not have to be
    // reinserted every other step. Same idea as Box2D's b2_aabbMultiplier.
    const prediction = AABBTreeBroadphase_prediction
    const vx = body.velocity.x * dt * prediction
    const vy = body.velocity.y * dt * prediction
    const vz = body.velocity.z * dt * prediction

    target.lowerBound.x = Math.min(tight.lowerBound.x, tight.lowerBound.x + vx) - margin
    target.lowerBound.y = Math.min(tight.lowerBound.y, tight.lowerBound.y + vy) - margin
    target.lowerBound.z = Math.min(tight.lowerBound.z, tight.lowerBound.z + vz) - margin
    target.upperBound.x = Math.max(tight.upperBound.x, tight.upperBound.x + vx) + margin
    target.upperBound.y = Math.max(tight.upperBound.y, tight.upperBound.y + vy) + margin
    target.upperBound.z = Math.max(tight.upperBound.z, tight.upperBound.z + vz) + margin

    // Shapes such as Plane use infinite bounds. Clamp them to keep the
    // surface-area heuristic away from Infinity - Infinity (NaN).
    clampBound(target.lowerBound)
    clampBound(target.upperBound)
  }

  /**
   * Insert a leaf, choosing the sibling that minimizes the surface area growth.
   */
  private insertLeaf(leaf: TreeNode): void {
    if (!this.root) {
      this.root = leaf
      leaf.parent = null
      return
    }

    // Find the best sibling for the new leaf
    const leafAABB = leaf.aabb
    let index = this.root

    while (!index.isLeaf) {
      const child1 = index.child1 as TreeNode
      const child2 = index.child2 as TreeNode

      const area = surfaceArea(index.aabb)
      const combined = combineAABB(index.aabb, leafAABB, tree_combined)
      const combinedArea = surfaceArea(combined)
      const cost = 2 * combinedArea
      const inheritanceCost = 2 * (combinedArea - area)

      let cost1: number
      const newArea1 = surfaceArea(combineAABB(leafAABB, child1.aabb, tree_combined2))
      if (child1.isLeaf) {
        cost1 = newArea1 + inheritanceCost
      } else {
        cost1 = newArea1 - surfaceArea(child1.aabb) + inheritanceCost
      }

      let cost2: number
      const newArea2 = surfaceArea(combineAABB(leafAABB, child2.aabb, tree_combined3))
      if (child2.isLeaf) {
        cost2 = newArea2 + inheritanceCost
      } else {
        cost2 = newArea2 - surfaceArea(child2.aabb) + inheritanceCost
      }

      // Descend according to the minimum cost
      if (cost < cost1 && cost < cost2) {
        break
      }
      index = cost1 < cost2 ? child1 : child2
    }

    const sibling = index

    // Create a new parent for the sibling and the new leaf
    const oldParent = sibling.parent
    const newParent = this.createNode()
    newParent.parent = oldParent
    combineAABB(leafAABB, sibling.aabb, newParent.aabb)
    newParent.height = sibling.height + 1

    if (oldParent) {
      if (oldParent.child1 === sibling) {
        oldParent.child1 = newParent
      } else {
        oldParent.child2 = newParent
      }
    } else {
      this.root = newParent
    }

    newParent.child1 = sibling
    newParent.child2 = leaf
    sibling.parent = newParent
    leaf.parent = newParent

    // Walk back up the tree, fixing heights and balancing
    index = leaf.parent as TreeNode
    while (index) {
      index = this.balance(index)
      const child1 = index.child1 as TreeNode
      const child2 = index.child2 as TreeNode
      index.height = 1 + Math.max(child1.height, child2.height)
      combineAABB(child1.aabb, child2.aabb, index.aabb)
      index = index.parent as TreeNode
    }
  }

  /**
   * Remove a leaf and destroy its parent.
   */
  private removeLeaf(leaf: TreeNode): void {
    if (leaf === this.root) {
      this.root = null
      leaf.parent = null
      return
    }

    const parent = leaf.parent as TreeNode
    const grandParent = parent.parent
    const sibling = parent.child1 === leaf ? (parent.child2 as TreeNode) : (parent.child1 as TreeNode)

    if (grandParent) {
      if (grandParent.child1 === parent) {
        grandParent.child1 = sibling
      } else {
        grandParent.child2 = sibling
      }
      sibling.parent = grandParent

      leaf.parent = null
      this.releaseNode(parent)

      let index: TreeNode | null = grandParent
      while (index) {
        index = this.balance(index)
        const child1 = index.child1 as TreeNode
        const child2 = index.child2 as TreeNode
        index.height = 1 + Math.max(child1.height, child2.height)
        combineAABB(child1.aabb, child2.aabb, index.aabb)
        index = index.parent
      }
    } else {
      this.root = sibling
      sibling.parent = null
      leaf.parent = null
      this.releaseNode(parent)
    }
  }

  /**
   * Get a node from the pool or create a new one.
   */
  private createNode(): TreeNode {
    const node = this.nodePool.pop()
    if (node) {
      return node
    }
    return new TreeNode()
  }

  /**
   * Return a node to the pool so insertions and removals do not allocate.
   */
  private releaseNode(node: TreeNode): void {
    node.parent = null
    node.child1 = null
    node.child2 = null
    node.body = null
    node.height = 0
    this.nodePool.push(node)
  }

  /**
   * Perform a left or right rotation if node A is imbalanced.
   * @return The new root of the subtree.
   */
  private balance(iA: TreeNode): TreeNode {
    if (iA.isLeaf || iA.height < 2) {
      return iA
    }

    let iB = iA.child1 as TreeNode
    let iC = iA.child2 as TreeNode
    const balance = iC.height - iB.height

    // Rotate C up
    if (balance > 1) {
      const iF = iC.child1 as TreeNode
      const iG = iC.child2 as TreeNode

      iC.child1 = iA
      iC.parent = iA.parent
      iA.parent = iC

      if (iC.parent) {
        if (iC.parent.child1 === iA) {
          iC.parent.child1 = iC
        } else {
          iC.parent.child2 = iC
        }
      } else {
        this.root = iC
      }

      if (iF.height > iG.height) {
        iC.child2 = iF
        iA.child2 = iG
        iG.parent = iA
        combineAABB(iB.aabb, iG.aabb, iA.aabb)
        combineAABB(iA.aabb, iF.aabb, iC.aabb)
        iA.height = 1 + Math.max(iB.height, iG.height)
        iC.height = 1 + Math.max(iA.height, iF.height)
      } else {
        iC.child2 = iG
        iA.child2 = iF
        iF.parent = iA
        combineAABB(iB.aabb, iF.aabb, iA.aabb)
        combineAABB(iA.aabb, iG.aabb, iC.aabb)
        iA.height = 1 + Math.max(iB.height, iF.height)
        iC.height = 1 + Math.max(iA.height, iG.height)
      }

      return iC
    }

    // Rotate B up
    if (balance < -1) {
      const iD = iB.child1 as TreeNode
      const iE = iB.child2 as TreeNode

      iB.child1 = iA
      iB.parent = iA.parent
      iA.parent = iB

      if (iB.parent) {
        if (iB.parent.child1 === iA) {
          iB.parent.child1 = iB
        } else {
          iB.parent.child2 = iB
        }
      } else {
        this.root = iB
      }

      if (iD.height > iE.height) {
        iB.child2 = iD
        iA.child1 = iE
        iE.parent = iA
        combineAABB(iC.aabb, iE.aabb, iA.aabb)
        combineAABB(iA.aabb, iD.aabb, iB.aabb)
        iA.height = 1 + Math.max(iC.height, iE.height)
        iB.height = 1 + Math.max(iA.height, iD.height)
      } else {
        iB.child2 = iE
        iA.child1 = iD
        iD.parent = iA
        combineAABB(iC.aabb, iD.aabb, iA.aabb)
        combineAABB(iA.aabb, iE.aabb, iB.aabb)
        iA.height = 1 + Math.max(iC.height, iD.height)
        iB.height = 1 + Math.max(iA.height, iE.height)
      }

      return iB
    }

    return iA
  }
}

/** How many steps of motion a leaf AABB covers. */
const AABBTreeBroadphase_prediction = 4

/** Bound used to keep infinite shape bounds (Plane) out of the tree math. */
const MAX_BOUND = 1e12

function clampBound(v: { x: number; y: number; z: number }): void {
  if (v.x > MAX_BOUND) v.x = MAX_BOUND
  else if (v.x < -MAX_BOUND) v.x = -MAX_BOUND
  if (v.y > MAX_BOUND) v.y = MAX_BOUND
  else if (v.y < -MAX_BOUND) v.y = -MAX_BOUND
  if (v.z > MAX_BOUND) v.z = MAX_BOUND
  else if (v.z < -MAX_BOUND) v.z = -MAX_BOUND
}

/**
 * Surface area of an AABB. Used by the insertion heuristic.
 */
function surfaceArea(aabb: AABB): number {
  const dx = aabb.upperBound.x - aabb.lowerBound.x
  const dy = aabb.upperBound.y - aabb.lowerBound.y
  const dz = aabb.upperBound.z - aabb.lowerBound.z
  return 2 * (dx * dy + dy * dz + dz * dx)
}

function combineAABB(a: AABB, b: AABB, target: AABB): AABB {
  target.lowerBound.x = Math.min(a.lowerBound.x, b.lowerBound.x)
  target.lowerBound.y = Math.min(a.lowerBound.y, b.lowerBound.y)
  target.lowerBound.z = Math.min(a.lowerBound.z, b.lowerBound.z)
  target.upperBound.x = Math.max(a.upperBound.x, b.upperBound.x)
  target.upperBound.y = Math.max(a.upperBound.y, b.upperBound.y)
  target.upperBound.z = Math.max(a.upperBound.z, b.upperBound.z)
  return target
}

function containsAABB(fat: AABB, tight: AABB): boolean {
  return (
    fat.lowerBound.x <= tight.lowerBound.x &&
    fat.lowerBound.y <= tight.lowerBound.y &&
    fat.lowerBound.z <= tight.lowerBound.z &&
    fat.upperBound.x >= tight.upperBound.x &&
    fat.upperBound.y >= tight.upperBound.y &&
    fat.upperBound.z >= tight.upperBound.z
  )
}

// Scratch AABBs, never used across a reentrant call.
const tree_combined = new AABB()
const tree_combined2 = new AABB()
const tree_combined3 = new AABB()
