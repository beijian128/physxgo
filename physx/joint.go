package physx

/*
#include "bridge.h"
*/
import "C"
import "runtime"

// ──────────────────────────────────────────────────────────────────────────────
// Joints
// ──────────────────────────────────────────────────────────────────────────────

// JointHandle wraps a PxJoint.
type JointHandle struct{ h C.PxJointHandle }

// Joint constraint flags. Must match PxConstraintFlag::Enum from PxConstraint.h.
// ⚠️ Bit positions verified against PhysX 3.4 source — ePROJECTION is a combo (1<<1|1<<2), not a unique bit.
const (
	JointFlagBroken               = 1 << 0 // eBROKEN
	JointFlagProjectToActor0      = 1 << 1 // ePROJECT_TO_ACTOR0
	JointFlagProjectToActor1      = 1 << 2 // ePROJECT_TO_ACTOR1
	JointFlagCollisionEnabled     = 1 << 3 // eCOLLISION_ENABLED
	JointFlagVisualization        = 1 << 4 // eVISUALIZATION   ← NOT 1<<5!
	JointFlagDriveLimitsAreForces = 1 << 5 // eDRIVE_LIMITS_ARE_FORCES
	JointFlagImprovedSLERP        = 1 << 7 // eIMPROVED_SLERP
	JointFlagDisablePreprocessing = 1 << 8 // eDISABLE_PREPROCESSING
)

// ── Creation helpers ────────────────────────────────────────────────────────

func createJoint(h C.PxJointHandle) *JointHandle {
	if h == nil {
		return nil
	}
	j := &JointHandle{h: h}
	runtime.SetFinalizer(j, func(j *JointHandle) { j.Release() })
	return j
}

func cfloat(v float32) C.float { return C.float(v) }

// CreateFixedJoint creates a fixed joint.
func CreateFixedJoint(physics *PhysicsHandle, a0 *ActorHandle, f0 Transform, a1 *ActorHandle, f1 Transform) *JointHandle {
	return createJoint(C.physx_create_fixed_joint(physics.h, a0.h,
		cfloat(f0.P.X), cfloat(f0.P.Y), cfloat(f0.P.Z), cfloat(f0.Q.X), cfloat(f0.Q.Y), cfloat(f0.Q.Z), cfloat(f0.Q.W),
		a1.h,
		cfloat(f1.P.X), cfloat(f1.P.Y), cfloat(f1.P.Z), cfloat(f1.Q.X), cfloat(f1.Q.Y), cfloat(f1.Q.Z), cfloat(f1.Q.W)))
}

// CreateRevoluteJoint creates a revolute (hinge) joint.
func CreateRevoluteJoint(physics *PhysicsHandle, a0 *ActorHandle, f0 Transform, a1 *ActorHandle, f1 Transform) *JointHandle {
	return createJoint(C.physx_create_revolute_joint(physics.h, a0.h,
		cfloat(f0.P.X), cfloat(f0.P.Y), cfloat(f0.P.Z), cfloat(f0.Q.X), cfloat(f0.Q.Y), cfloat(f0.Q.Z), cfloat(f0.Q.W),
		a1.h,
		cfloat(f1.P.X), cfloat(f1.P.Y), cfloat(f1.P.Z), cfloat(f1.Q.X), cfloat(f1.Q.Y), cfloat(f1.Q.Z), cfloat(f1.Q.W)))
}

// CreateSphericalJoint creates a spherical (ball-and-socket) joint.
func CreateSphericalJoint(physics *PhysicsHandle, a0 *ActorHandle, f0 Transform, a1 *ActorHandle, f1 Transform) *JointHandle {
	return createJoint(C.physx_create_spherical_joint(physics.h, a0.h,
		cfloat(f0.P.X), cfloat(f0.P.Y), cfloat(f0.P.Z), cfloat(f0.Q.X), cfloat(f0.Q.Y), cfloat(f0.Q.Z), cfloat(f0.Q.W),
		a1.h,
		cfloat(f1.P.X), cfloat(f1.P.Y), cfloat(f1.P.Z), cfloat(f1.Q.X), cfloat(f1.Q.Y), cfloat(f1.Q.Z), cfloat(f1.Q.W)))
}

// CreatePrismaticJoint creates a prismatic (slider) joint.
func CreatePrismaticJoint(physics *PhysicsHandle, a0 *ActorHandle, f0 Transform, a1 *ActorHandle, f1 Transform) *JointHandle {
	return createJoint(C.physx_create_prismatic_joint(physics.h, a0.h,
		cfloat(f0.P.X), cfloat(f0.P.Y), cfloat(f0.P.Z), cfloat(f0.Q.X), cfloat(f0.Q.Y), cfloat(f0.Q.Z), cfloat(f0.Q.W),
		a1.h,
		cfloat(f1.P.X), cfloat(f1.P.Y), cfloat(f1.P.Z), cfloat(f1.Q.X), cfloat(f1.Q.Y), cfloat(f1.Q.Z), cfloat(f1.Q.W)))
}

// CreateDistanceJoint creates a distance joint.
func CreateDistanceJoint(physics *PhysicsHandle, a0 *ActorHandle, f0 Transform, a1 *ActorHandle, f1 Transform) *JointHandle {
	return createJoint(C.physx_create_distance_joint(physics.h, a0.h,
		cfloat(f0.P.X), cfloat(f0.P.Y), cfloat(f0.P.Z), cfloat(f0.Q.X), cfloat(f0.Q.Y), cfloat(f0.Q.Z), cfloat(f0.Q.W),
		a1.h,
		cfloat(f1.P.X), cfloat(f1.P.Y), cfloat(f1.P.Z), cfloat(f1.Q.X), cfloat(f1.Q.Y), cfloat(f1.Q.Z), cfloat(f1.Q.W)))
}

// CreateD6Joint creates a D6 joint.
func CreateD6Joint(physics *PhysicsHandle, a0 *ActorHandle, f0 Transform, a1 *ActorHandle, f1 Transform) *JointHandle {
	return createJoint(C.physx_create_d6_joint(physics.h, a0.h,
		cfloat(f0.P.X), cfloat(f0.P.Y), cfloat(f0.P.Z), cfloat(f0.Q.X), cfloat(f0.Q.Y), cfloat(f0.Q.Z), cfloat(f0.Q.W),
		a1.h,
		cfloat(f1.P.X), cfloat(f1.P.Y), cfloat(f1.P.Z), cfloat(f1.Q.X), cfloat(f1.Q.Y), cfloat(f1.Q.Z), cfloat(f1.Q.W)))
}

// Release releases the joint.
func (j *JointHandle) Release() {
	if j == nil || j.h == nil {
		return
	}
	C.physx_release_joint(j.h)
	j.h = nil
}

// SetInvalid clears the internal handle without releasing C++ memory.
func (j *JointHandle) SetInvalid() {
	if j == nil {
		return
	}
	j.h = nil
}

// ── Common joint methods ────────────────────────────────────────────────────

// SetBreakForce sets break force and torque limits.
func (j *JointHandle) SetBreakForce(force, torque float32) error {
	return errOrNil(int(C.physx_joint_set_break_force(j.h, C.float(force), C.float(torque))))
}

// GetBreakForce returns break force and torque limits.
func (j *JointHandle) GetBreakForce() (float32, float32) {
	var force, torque C.float
	C.physx_joint_get_break_force(j.h, &force, &torque)
	return float32(force), float32(torque)
}

// SetConstraintFlags sets all constraint flags.
func (j *JointHandle) SetConstraintFlags(flags uint32) error {
	return errOrNil(int(C.physx_joint_set_constraint_flags(j.h, C.uint32_t(flags))))
}

// SetConstraintFlag enables/disables a single constraint flag.
func (j *JointHandle) SetConstraintFlag(flag uint32, enabled bool) error {
	return errOrNil(int(C.physx_joint_set_constraint_flag(j.h, C.uint32_t(flag), C.int(boolToInt(enabled)))))
}

// GetConstraintFlags returns all constraint flags.
func (j *JointHandle) GetConstraintFlags() uint32 {
	return uint32(C.physx_joint_get_constraint_flags(j.h))
}

// ── Revolute joint ──────────────────────────────────────────────────────────

// SetRevoluteLimit sets the angular limit.
func (j *JointHandle) SetRevoluteLimit(lower, upper, stiffness, damping float32) error {
	return errOrNil(int(C.physx_revolute_joint_set_limit(j.h,
		C.float(lower), C.float(upper), C.float(stiffness), C.float(damping))))
}

// GetRevoluteAngle returns the current angle.
func (j *JointHandle) GetRevoluteAngle() float32 {
	return float32(C.physx_revolute_joint_get_angle(j.h))
}

// GetRevoluteVelocity returns the current angular velocity.
func (j *JointHandle) GetRevoluteVelocity() float32 {
	return float32(C.physx_revolute_joint_get_velocity(j.h))
}

// SetRevoluteDriveVelocity sets the drive velocity target.
func (j *JointHandle) SetRevoluteDriveVelocity(velocity float32) error {
	return errOrNil(int(C.physx_revolute_joint_set_drive_velocity(j.h, C.float(velocity))))
}

// SetRevoluteDriveForceLimit sets the drive force limit.
func (j *JointHandle) SetRevoluteDriveForceLimit(limit float32) error {
	return errOrNil(int(C.physx_revolute_joint_set_drive_force_limit(j.h, C.float(limit))))
}

// ── Spherical joint ─────────────────────────────────────────────────────────

// SetSphericalLimitCone sets the limit cone.
func (j *JointHandle) SetSphericalLimitCone(yAngle, zAngle, stiffness, damping float32) error {
	return errOrNil(int(C.physx_spherical_joint_set_limit_cone(j.h,
		C.float(yAngle), C.float(zAngle), C.float(stiffness), C.float(damping))))
}

// ── Prismatic joint ─────────────────────────────────────────────────────────

// SetPrismaticLimit sets the linear limit.
func (j *JointHandle) SetPrismaticLimit(lower, upper, stiffness, damping float32) error {
	return errOrNil(int(C.physx_prismatic_joint_set_limit(j.h,
		C.float(lower), C.float(upper), C.float(stiffness), C.float(damping))))
}

// GetPrismaticPosition returns the current position.
func (j *JointHandle) GetPrismaticPosition() float32 {
	return float32(C.physx_prismatic_joint_get_position(j.h))
}

// GetPrismaticVelocity returns the current velocity.
func (j *JointHandle) GetPrismaticVelocity() float32 {
	return float32(C.physx_prismatic_joint_get_velocity(j.h))
}

// ── Distance joint ──────────────────────────────────────────────────────────

// SetDistanceMinDistance sets min distance.
func (j *JointHandle) SetDistanceMinDistance(dist float32) error {
	return errOrNil(int(C.physx_distance_joint_set_min_distance(j.h, C.float(dist))))
}

// SetDistanceMaxDistance sets max distance.
func (j *JointHandle) SetDistanceMaxDistance(dist float32) error {
	return errOrNil(int(C.physx_distance_joint_set_max_distance(j.h, C.float(dist))))
}

// SetDistanceSpring sets the spring.
func (j *JointHandle) SetDistanceSpring(stiffness, damping float32) error {
	return errOrNil(int(C.physx_distance_joint_set_spring(j.h, C.float(stiffness), C.float(damping))))
}

// ── D6 joint ────────────────────────────────────────────────────────────────

// D6 axis constants.
const (
	D6AxisX      = 0
	D6AxisY      = 1
	D6AxisZ      = 2
	D6AxisTwist  = 3
	D6AxisSwing1 = 4
	D6AxisSwing2 = 5
)

// D6 motion constants.
const (
	D6MotionLocked  = 0
	D6MotionLimited = 1
	D6MotionFree    = 2
)

// D6 drive constants.
const (
	D6DriveX     = 0
	D6DriveY     = 1
	D6DriveZ     = 2
	D6DriveSwing = 3
	D6DriveTwist = 4
	D6DriveSLERP = 5
)

// D6JointDrive describes a D6 joint drive.
type D6JointDrive struct {
	Stiffness  float32
	Damping    float32
	ForceLimit float32
	Flags      int32
}

// SetD6Motion sets the motion for a D6 axis.
func (j *JointHandle) SetD6Motion(axis, motion int) error {
	return errOrNil(int(C.physx_d6_joint_set_motion(j.h, C.CPxD6Axis(axis), C.CPxD6Motion(motion))))
}

// SetD6Drive sets a D6 drive.
func (j *JointHandle) SetD6Drive(drive int, d D6JointDrive) error {
	cd := C.CPxD6JointDrive{
		stiffness:  C.float(d.Stiffness),
		damping:    C.float(d.Damping),
		forceLimit: C.float(d.ForceLimit),
		flags:      C.int(d.Flags),
	}
	return errOrNil(int(C.physx_d6_joint_set_drive(j.h, C.CPxD6Drive(drive), &cd)))
}

// SetD6DrivePosition sets the drive target pose.
func (j *JointHandle) SetD6DrivePosition(px, py, pz, qx, qy, qz, qw float32) error {
	return errOrNil(int(C.physx_d6_joint_set_drive_position(j.h,
		C.float(px), C.float(py), C.float(pz),
		C.float(qx), C.float(qy), C.float(qz), C.float(qw))))
}
