export class Box {
  constructor() {
    this.width = 0;
    this.height = 0;
    this.depth = 0;
  }
}

export class Sphere {
  constructor() {
    this.radius = 0;
  }
}

export class RigidBody {
  constructor() {
    this.object = null;
    this.weight = 0;
    this.restitution = 1;
    this.friction = 1;
    this.linearDamping = 0;
    this.angularDamping = 0;
    this.linearVelocity = {x: 0, y: 0, z: 0};
  }
}
