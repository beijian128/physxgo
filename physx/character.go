package physx

/*
#include "bridge.h"
*/
import "C"
import "runtime"

// ──────────────────────────────────────────────────────────────────────────────
// Character Controller
// ──────────────────────────────────────────────────────────────────────────────

// ControllerManagerHandle wraps a PxControllerManager.
type ControllerManagerHandle struct{ h C.PxControllerMgrHandle }

// ControllerHandle wraps a PxController (box or capsule).
type ControllerHandle struct{ h C.PxControllerHandle }

// CreateControllerManager creates a character controller manager for a scene.
func CreateControllerManager(scene *SceneHandle) *ControllerManagerHandle {
	h := C.physx_create_controller_manager(scene.h)
	if h == nil {
		return nil
	}
	m := &ControllerManagerHandle{h: h}
	runtime.SetFinalizer(m, func(m *ControllerManagerHandle) { m.Release() })
	return m
}

// Release releases the controller manager.
func (m *ControllerManagerHandle) Release() {
	if m == nil || m.h == nil {
		return
	}
	C.physx_release_controller_manager(m.h)
	m.h = nil
}

// CreateBoxController creates a box character controller.
func (m *ControllerManagerHandle) CreateBoxController(physics *PhysicsHandle,
	halfHeight, halfSideExtent, px, py, pz float32, mat *MaterialHandle) *ControllerHandle {
	h := C.physx_create_box_controller(m.h, physics.h,
		C.float(halfHeight), C.float(halfSideExtent),
		C.float(px), C.float(py), C.float(pz),
		mat.h)
	if h == nil {
		return nil
	}
	c := &ControllerHandle{h: h}
	runtime.SetFinalizer(c, func(c *ControllerHandle) { c.Release() })
	return c
}

// CreateCapsuleController creates a capsule character controller.
func (m *ControllerManagerHandle) CreateCapsuleController(physics *PhysicsHandle,
	radius, height, px, py, pz float32, mat *MaterialHandle) *ControllerHandle {
	h := C.physx_create_capsule_controller(m.h, physics.h,
		C.float(radius), C.float(height),
		C.float(px), C.float(py), C.float(pz),
		mat.h)
	if h == nil {
		return nil
	}
	c := &ControllerHandle{h: h}
	runtime.SetFinalizer(c, func(c *ControllerHandle) { c.Release() })
	return c
}

// Release releases the controller.
func (c *ControllerHandle) Release() {
	if c == nil || c.h == nil {
		return
	}
	C.physx_release_controller(c.h)
	c.h = nil
}

// GetActor returns the underlying kinematic actor of the controller.
func (c *ControllerHandle) GetActor() *ActorHandle {
	h := C.physx_controller_get_actor(c.h)
	if h == nil {
		return nil
	}
	return &ActorHandle{h: h}
}

// Move moves the controller. Returns collision flags.
func (c *ControllerHandle) Move(dx, dy, dz, minDist, dt float32) int {
	return int(C.physx_controller_move(c.h, C.float(dx), C.float(dy), C.float(dz), C.float(minDist), C.float(dt)))
}

// GetPosition returns the controller position.
func (c *ControllerHandle) GetPosition() (float32, float32, float32) {
	var x, y, z C.float
	C.physx_controller_get_position(c.h, &x, &y, &z)
	return float32(x), float32(y), float32(z)
}

// SetPosition sets the controller position.
func (c *ControllerHandle) SetPosition(x, y, z float32) error {
	return errOrNil(int(C.physx_controller_set_position(c.h, C.float(x), C.float(y), C.float(z))))
}

// GetFootPosition returns the foot position.
func (c *ControllerHandle) GetFootPosition() (float32, float32, float32) {
	var x, y, z C.float
	C.physx_controller_get_foot_position(c.h, &x, &y, &z)
	return float32(x), float32(y), float32(z)
}

// SetFootPosition sets the foot position.
func (c *ControllerHandle) SetFootPosition(x, y, z float32) error {
	return errOrNil(int(C.physx_controller_set_foot_position(c.h, C.float(x), C.float(y), C.float(z))))
}

// SetStepOffset sets the step offset.
func (c *ControllerHandle) SetStepOffset(offset float32) error {
	return errOrNil(int(C.physx_controller_set_step_offset(c.h, C.float(offset))))
}

// SetSlopeLimit sets the slope limit in radians.
func (c *ControllerHandle) SetSlopeLimit(limit float32) error {
	return errOrNil(int(C.physx_controller_set_slope_limit(c.h, C.float(limit))))
}
