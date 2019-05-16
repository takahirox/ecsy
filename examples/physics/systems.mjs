import { System } from "../../build/ecsy.module.js";

import {
  Box,
  Sphere,
  RigidBody
} from './components.mjs';

export class PhysicsSystem extends System {
  init() {
    this._physicsWorld = this._createWorld();
    this._transform = new Ammo.btTransform();
    this._quaternion = new Ammo.btQuaternion(0, 0, 0, 1);
    return {
      entities: [RigidBody]
    }
  }

  _createWorld() {
    const config = new Ammo.btDefaultCollisionConfiguration();
    const dispatcher = new Ammo.btCollisionDispatcher(config);
    const cache = new Ammo.btDbvtBroadphase();
    const solver = new Ammo.btSequentialImpulseConstraintSolver();
    const world = new Ammo.btDiscreteDynamicsWorld(dispatcher, cache, solver, config);
    world.setGravity(new Ammo.btVector3(0, -9.8 * 10, 0));
    return world;
  }

  _createShape(entity) {
    if(entity.hasComponent(Box)) {
      const box = entity.getComponent(Box);
      return new Ammo.btBoxShape(new Ammo.btVector3(box.width, box.height, box.depth));
    }
    if(entity.hasComponent(Sphere)) {
      const sphere = entity.getComponent(Sphere);
      return new Ammo.btSphereShape(sphere.radius);
    }
    return new Ammo.btBoxShape(new Ammo.btVector3(1.0, 1.0, 1.0));
  }

  _createRigidBody(entity) {
    const rigidBody = entity.getComponent(RigidBody);
    const object = rigidBody.object;
    const shape = this._createShape(entity);
    const localInertia = new Ammo.btVector3(0, 0, 0);
    shape.calculateLocalInertia(rigidBody.weight, localInertia);
    const form = new Ammo.btTransform();
    form.setIdentity();
    form.setOrigin(new Ammo.btVector3(object.position.x, object.position.y, object.position.z));
    const state = new Ammo.btDefaultMotionState(form);
    const info = new Ammo.btRigidBodyConstructionInfo(rigidBody.weight, state, shape, localInertia);
    return new Ammo.btRigidBody(info);
  }

  _setupRigidBody(body, entity) {
    const rigidBody = entity.getComponent(RigidBody);
    const linearVelocity = rigidBody.linearVelocity;
    body.setRestitution(rigidBody.restitution);
    body.setFriction(rigidBody.friction);
    body.setDamping(rigidBody.linearDamping, rigidBody.angularDamping);
    body.setSleepingThresholds(0, 0);
    body.setLinearVelocity(new Ammo.btVector3(linearVelocity.x, linearVelocity.y, linearVelocity.z));
    return body;
  }

  execute(delta) {
    const entities = this.queries.entities;

    // @TODO: Move to ENTITY_ADD event handler
    for(let i = 0, il = entities.length; i < il; i++) {
      const entity = entities[i];
      const rigidBody = entity.getComponent(RigidBody);
      const object = rigidBody.object;

      if(!object.userData.body) {
        const body = this._setupRigidBody(this._createRigidBody(entity), entity);
        object.userData.body = body;
        this._physicsWorld.addRigidBody(body);
      }
    }

    this._physicsWorld.stepSimulation(delta, 2, 1 / 60);

    for(let i = 0, il = entities.length; i < il; i++) {
      const entity = entities[i];
      const rigidBody = entity.getComponent(RigidBody);

      if(rigidBody.weight === 0.0) continue;

      const object = rigidBody.object;
      const body = object.userData.body;
      const form = this._transform;
      const q = this._quaternion;

      body.getMotionState().getWorldTransform(form);
      const o = form.getOrigin();
      form.getBasis().getRotation(q);

      // Update instance's position and quaternion

      object.position.set(o.x(), o.y(), o.z());
      object.quaternion.set(q.x(), q.y(), q.z(), q.w());
    }
  }
}
