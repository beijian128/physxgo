package physx

/*
#include "bridge.h"
*/
import "C"
import "runtime"

// ──────────────────────────────────────────────────────────────────────────────
// Actor
// ──────────────────────────────────────────────────────────────────────────────

// ActorHandle wraps a PxRigidActor (static or dynamic).
type ActorHandle struct{ h C.PxActorHandle }

// Actor type constants.
const (
	ActorTypeStatic  = 0
	ActorTypeDynamic = 1
)

// ── Creation ────────────────────────────────────────────────────────────────

// CreateRigidDynamic creates an empty dynamic rigid body.
func CreateRigidDynamic(physics *PhysicsHandle, px, py, pz, qx, qy, qz, qw float32) *ActorHandle {
	h := C.physx_create_rigid_dynamic(physics.h,
		C.float(px), C.float(py), C.float(pz),
		C.float(qx), C.float(qy), C.float(qz), C.float(qw))
	if h == nil {
		return nil
	}
	a := &ActorHandle{h: h}
	runtime.SetFinalizer(a, func(a *ActorHandle) { a.Release() })
	return a
}

// CreateRigidStatic creates an empty static rigid body.
func CreateRigidStatic(physics *PhysicsHandle, px, py, pz, qx, qy, qz, qw float32) *ActorHandle {
	h := C.physx_create_rigid_static(physics.h,
		C.float(px), C.float(py), C.float(pz),
		C.float(qx), C.float(qy), C.float(qz), C.float(qw))
	if h == nil {
		return nil
	}
	a := &ActorHandle{h: h}
	runtime.SetFinalizer(a, func(a *ActorHandle) { a.Release() })
	return a
}

// CreateDynamicBox creates a dynamic rigid body with a box shape.
func CreateDynamicBox(physics *PhysicsHandle, px, py, pz, hx, hy, hz float32, mat *MaterialHandle, density float32) *ActorHandle {
	h := C.physx_create_dynamic_box(physics.h,
		C.float(px), C.float(py), C.float(pz),
		C.float(hx), C.float(hy), C.float(hz),
		mat.h, C.float(density))
	if h == nil {
		return nil
	}
	a := &ActorHandle{h: h}
	runtime.SetFinalizer(a, func(a *ActorHandle) { a.Release() })
	return a
}

// CreateDynamicSphere creates a dynamic rigid body with a sphere shape.
func CreateDynamicSphere(physics *PhysicsHandle, px, py, pz, radius float32, mat *MaterialHandle, density float32) *ActorHandle {
	h := C.physx_create_dynamic_sphere(physics.h,
		C.float(px), C.float(py), C.float(pz),
		C.float(radius), mat.h, C.float(density))
	if h == nil {
		return nil
	}
	a := &ActorHandle{h: h}
	runtime.SetFinalizer(a, func(a *ActorHandle) { a.Release() })
	return a
}

// CreateDynamicCapsule creates a dynamic rigid body with a capsule shape.
func CreateDynamicCapsule(physics *PhysicsHandle, px, py, pz, radius, halfHeight float32, mat *MaterialHandle, density float32) *ActorHandle {
	h := C.physx_create_dynamic_capsule(physics.h,
		C.float(px), C.float(py), C.float(pz),
		C.float(radius), C.float(halfHeight), mat.h, C.float(density))
	if h == nil {
		return nil
	}
	a := &ActorHandle{h: h}
	runtime.SetFinalizer(a, func(a *ActorHandle) { a.Release() })
	return a
}

// CreateStaticPlane creates a static plane actor.
func CreateStaticPlane(physics *PhysicsHandle, nx, ny, nz, d float32, mat *MaterialHandle) *ActorHandle {
	h := C.physx_create_static_plane(physics.h,
		C.float(nx), C.float(ny), C.float(nz), C.float(d), mat.h)
	if h == nil {
		return nil
	}
	a := &ActorHandle{h: h}
	runtime.SetFinalizer(a, func(a *ActorHandle) { a.Release() })
	return a
}

// Release releases the actor.
func (a *ActorHandle) Release() {
	if a == nil || a.h == nil {
		return
	}
	C.physx_release_actor(a.h)
	a.h = nil
}

// SetInvalid clears the internal handle without releasing C++ memory.
// Use when the C++ object is already freed (e.g. by scene release).
func (a *ActorHandle) SetInvalid() {
	if a == nil {
		return
	}
	a.h = nil
}

// ── Pose ─────────────────────────────────────────────────────────────────────

// GetGlobalPose returns the actor's world-space pose as (px,py,pz, qx,qy,qz,qw).
func (a *ActorHandle) GetGlobalPose() (px, py, pz, qx, qy, qz, qw float32) {
	var cpx, cpy, cpz, cqx, cqy, cqz, cqw C.float
	C.physx_actor_get_global_pose(a.h, &cpx, &cpy, &cpz, &cqx, &cqy, &cqz, &cqw)
	return float32(cpx), float32(cpy), float32(cpz), float32(cqx), float32(cqy), float32(cqz), float32(cqw)
}

// GetTransform returns the actor's world-space transform.
func (a *ActorHandle) GetTransform() Transform {
	px, py, pz, qx, qy, qz, qw := a.GetGlobalPose()
	return NewTransform(px, py, pz, qx, qy, qz, qw)
}

// SetGlobalPose sets the actor's world-space pose.
func (a *ActorHandle) SetGlobalPose(px, py, pz, qx, qy, qz, qw float32, autowake bool) error {
	aw := 0
	if autowake {
		aw = 1
	}
	return errOrNil(int(C.physx_actor_set_global_pose(a.h,
		C.float(px), C.float(py), C.float(pz),
		C.float(qx), C.float(qy), C.float(qz), C.float(qw),
		C.int(aw))))
}

// SetTransform sets the actor's world-space transform.
func (a *ActorHandle) SetTransform(t Transform, autowake bool) error {
	return a.SetGlobalPose(t.P.X, t.P.Y, t.P.Z, t.Q.X, t.Q.Y, t.Q.Z, t.Q.W, autowake)
}

// ── Velocity ─────────────────────────────────────────────────────────────────

// GetLinearVelocity returns linear velocity.
func (a *ActorHandle) GetLinearVelocity() (float32, float32, float32) {
	var x, y, z C.float
	C.physx_actor_get_linear_velocity(a.h, &x, &y, &z)
	return float32(x), float32(y), float32(z)
}

// SetLinearVelocity sets linear velocity.
func (a *ActorHandle) SetLinearVelocity(x, y, z float32) error {
	return errOrNil(int(C.physx_actor_set_linear_velocity(a.h, C.float(x), C.float(y), C.float(z))))
}

// GetAngularVelocity returns angular velocity.
func (a *ActorHandle) GetAngularVelocity() (float32, float32, float32) {
	var x, y, z C.float
	C.physx_actor_get_angular_velocity(a.h, &x, &y, &z)
	return float32(x), float32(y), float32(z)
}

// SetAngularVelocity sets angular velocity.
func (a *ActorHandle) SetAngularVelocity(x, y, z float32) error {
	return errOrNil(int(C.physx_actor_set_angular_velocity(a.h, C.float(x), C.float(y), C.float(z))))
}

// ── Forces ───────────────────────────────────────────────────────────────────

// Force mode constants.
const (
	ForceModeForce          = 0
	ForceModeImpulse        = 1
	ForceModeVelocityChange = 2
	ForceModeAcceleration   = 3
)

// AddForce applies a force to the actor.
func (a *ActorHandle) AddForce(fx, fy, fz float32, mode int, autowake bool) error {
	aw := 0
	if autowake {
		aw = 1
	}
	return errOrNil(int(C.physx_actor_add_force(a.h, C.float(fx), C.float(fy), C.float(fz), C.CPxForceMode(mode), C.int(aw))))
}

// AddTorque applies a torque to the actor.
func (a *ActorHandle) AddTorque(tx, ty, tz float32, mode int, autowake bool) error {
	aw := 0
	if autowake {
		aw = 1
	}
	return errOrNil(int(C.physx_actor_add_torque(a.h, C.float(tx), C.float(ty), C.float(tz), C.CPxForceMode(mode), C.int(aw))))
}

// ClearForce clears accumulated force.
func (a *ActorHandle) ClearForce(mode int) error {
	return errOrNil(int(C.physx_actor_clear_force(a.h, C.CPxForceMode(mode))))
}

// ClearTorque clears accumulated torque.
func (a *ActorHandle) ClearTorque(mode int) error {
	return errOrNil(int(C.physx_actor_clear_torque(a.h, C.CPxForceMode(mode))))
}

// ── Mass ─────────────────────────────────────────────────────────────────────

// SetMass sets the actor's mass.
func (a *ActorHandle) SetMass(mass float32) error {
	return errOrNil(int(C.physx_actor_set_mass(a.h, C.float(mass))))
}

// GetMass returns the actor's mass.
func (a *ActorHandle) GetMass() float32 {
	return float32(C.physx_actor_get_mass(a.h))
}

// UpdateMassAndInertia computes mass & inertia from geometry and uniform density.
func (a *ActorHandle) UpdateMassAndInertia(density float32) error {
	return errOrNil(int(C.physx_actor_update_mass_and_inertia(a.h, C.float(density))))
}

// ── Sleep ────────────────────────────────────────────────────────────────────

// IsSleeping returns whether the actor is sleeping.
func (a *ActorHandle) IsSleeping() bool {
	return C.physx_actor_is_sleeping(a.h) != 0
}

// WakeUp wakes the actor.
func (a *ActorHandle) WakeUp() error {
	return errOrNil(int(C.physx_actor_wake_up(a.h)))
}

// PutToSleep puts the actor to sleep.
func (a *ActorHandle) PutToSleep() error {
	return errOrNil(int(C.physx_actor_put_to_sleep(a.h)))
}

// ── Kinematic ────────────────────────────────────────────────────────────────

// SetKinematicTarget sets the kinematic target.
func (a *ActorHandle) SetKinematicTarget(px, py, pz, qx, qy, qz, qw float32) error {
	return errOrNil(int(C.physx_actor_set_kinematic_target(a.h,
		C.float(px), C.float(py), C.float(pz),
		C.float(qx), C.float(qy), C.float(qz), C.float(qw))))
}

// ── Damping ──────────────────────────────────────────────────────────────────

// SetLinearDamping sets linear damping.
func (a *ActorHandle) SetLinearDamping(d float32) error {
	return errOrNil(int(C.physx_actor_set_linear_damping(a.h, C.float(d))))
}

// SetAngularDamping sets angular damping.
func (a *ActorHandle) SetAngularDamping(d float32) error {
	return errOrNil(int(C.physx_actor_set_angular_damping(a.h, C.float(d))))
}

// GetLinearDamping returns linear damping.
func (a *ActorHandle) GetLinearDamping() float32 {
	return float32(C.physx_actor_get_linear_damping(a.h))
}

// GetAngularDamping returns angular damping.
func (a *ActorHandle) GetAngularDamping() float32 {
	return float32(C.physx_actor_get_angular_damping(a.h))
}

// ── Flags ────────────────────────────────────────────────────────────────────

// Actor flags.
const (
	ActorFlagVisualization     = 1 << 0
	ActorFlagDisableGravity    = 1 << 1
	ActorFlagSendSleepNotifies = 1 << 2
	ActorFlagDisableSimulation = 1 << 3
)

// Rigid body flags.
const (
	RigidBodyFlagKinematic                         = 1 << 0
	RigidBodyFlagUseKinematicTargetForSceneQueries = 1 << 1
	RigidBodyFlagEnableCCD                          = 1 << 2
	RigidBodyFlagEnableCCDFriction                  = 1 << 3
	RigidBodyFlagEnablePoseIntegrationPreview       = 1 << 4
	RigidBodyFlagEnableSpeculativeCCD               = 1 << 5
	RigidBodyFlagEnableCCDMaxContactImpulse         = 1 << 6
)

// Rigid dynamic lock flags.
const (
	LockLinearX  = 1 << 0
	LockLinearY  = 1 << 1
	LockLinearZ  = 1 << 2
	LockAngularX = 1 << 3
	LockAngularY = 1 << 4
	LockAngularZ = 1 << 5
)

// SetActorFlags sets actor flags.
func (a *ActorHandle) SetActorFlags(flags uint32) error {
	return errOrNil(int(C.physx_actor_set_actor_flags(a.h, C.uint32_t(flags))))
}

// GetActorFlags returns actor flags.
func (a *ActorHandle) GetActorFlags() uint32 {
	return uint32(C.physx_actor_get_actor_flags(a.h))
}

// SetRigidBodyFlags sets rigid body flags.
func (a *ActorHandle) SetRigidBodyFlags(flags uint32) error {
	return errOrNil(int(C.physx_actor_set_rigid_body_flags(a.h, C.uint32_t(flags))))
}

// GetRigidBodyFlags returns rigid body flags.
func (a *ActorHandle) GetRigidBodyFlags() uint32 {
	return uint32(C.physx_actor_get_rigid_body_flags(a.h))
}

// SetRigidDynamicLockFlags sets lock flags.
func (a *ActorHandle) SetRigidDynamicLockFlags(flags uint32) error {
	return errOrNil(int(C.physx_actor_set_rigid_dynamic_lock_flags(a.h, C.uint32_t(flags))))
}

// GetRigidDynamicLockFlags returns lock flags.
func (a *ActorHandle) GetRigidDynamicLockFlags() uint32 {
	return uint32(C.physx_actor_get_rigid_dynamic_lock_flags(a.h))
}

// ── Utility ──────────────────────────────────────────────────────────────────

// GetNbShapes returns the number of attached shapes.
func (a *ActorHandle) GetNbShapes() int {
	return int(C.physx_actor_get_nb_shapes(a.h))
}

// GetType returns ActorTypeStatic or ActorTypeDynamic.
func (a *ActorHandle) GetType() int {
	return int(C.physx_actor_get_type(a.h))
}

// GetWorldBounds returns the actor's world-space bounds.
func (a *ActorHandle) GetWorldBounds() Bounds3 {
	var b C.CPxBounds3
	C.physx_actor_get_world_bounds(a.h, &b)
	return Bounds3{Min: vec3FromC(b.minimum), Max: vec3FromC(b.maximum)}
}
