package physx

/*
#include "bridge.h"
*/
import "C"
import "runtime"

// ──────────────────────────────────────────────────────────────────────────────
// Shape
// ──────────────────────────────────────────────────────────────────────────────

// ShapeHandle wraps a PxShape.
type ShapeHandle struct{ h C.PxShapeHandle }

// Shape flags.
const (
	ShapeFlagSimulationShape = 1 << 0
	ShapeFlagSceneQueryShape = 1 << 1
	ShapeFlagTriggerShape    = 1 << 2
	ShapeFlagVisualization   = 1 << 3
)

// CreateBoxShape creates a box shape.
func CreateBoxShape(physics *PhysicsHandle, hx, hy, hz float32, mat *MaterialHandle, exclusive bool) *ShapeHandle {
	geom := C.CPxBoxGeometry{
		_type:         C.CPxGeometryType(C.CPxGeometryType_BOX),
		halfExtentsX:  C.float(hx),
		halfExtentsY:  C.float(hy),
		halfExtentsZ:  C.float(hz),
	}
	exc := boolToInt(exclusive)
	h := C.physx_create_shape(physics.h, &geom, mat.h, C.int(exc))
	if h == nil {
		return nil
	}
	s := &ShapeHandle{h: h}
	runtime.SetFinalizer(s, func(s *ShapeHandle) { s.Release() })
	return s
}

// CreateSphereShape creates a sphere shape.
func CreateSphereShape(physics *PhysicsHandle, radius float32, mat *MaterialHandle, exclusive bool) *ShapeHandle {
	exc := boolToInt(exclusive)
	h := C.physx_create_shape_sphere(physics.h, C.float(radius), mat.h, C.int(exc))
	if h == nil {
		return nil
	}
	s := &ShapeHandle{h: h}
	runtime.SetFinalizer(s, func(s *ShapeHandle) { s.Release() })
	return s
}

// CreateCapsuleShape creates a capsule shape.
func CreateCapsuleShape(physics *PhysicsHandle, radius, halfHeight float32, mat *MaterialHandle, exclusive bool) *ShapeHandle {
	exc := boolToInt(exclusive)
	h := C.physx_create_shape_capsule(physics.h, C.float(radius), C.float(halfHeight), mat.h, C.int(exc))
	if h == nil {
		return nil
	}
	s := &ShapeHandle{h: h}
	runtime.SetFinalizer(s, func(s *ShapeHandle) { s.Release() })
	return s
}

// Release releases the shape.
func (s *ShapeHandle) Release() {
	if s == nil || s.h == nil {
		return
	}
	C.physx_release_shape(s.h)
	s.h = nil
}

// SetLocalPose sets the shape's local pose.
func (s *ShapeHandle) SetLocalPose(px, py, pz, qx, qy, qz, qw float32) error {
	return errOrNil(int(C.physx_shape_set_local_pose(s.h,
		C.float(px), C.float(py), C.float(pz),
		C.float(qx), C.float(qy), C.float(qz), C.float(qw))))
}

// SetFlags sets shape flags.
func (s *ShapeHandle) SetFlags(flags uint32) error {
	return errOrNil(int(C.physx_shape_set_flags(s.h, C.uint32_t(flags))))
}

// GetFlags returns shape flags.
func (s *ShapeHandle) GetFlags() uint32 {
	return uint32(C.physx_shape_get_flags(s.h))
}

// SetAsTrigger configures the shape as a trigger.
func (s *ShapeHandle) SetAsTrigger(isTrigger bool) error {
	return errOrNil(int(C.physx_shape_set_as_trigger(s.h, C.int(boolToInt(isTrigger)))))
}

// SetSimulationFilterData sets simulation filter data.
func (s *ShapeHandle) SetSimulationFilterData(data FilterData) error {
	cdata := data.toC()
	return errOrNil(int(C.physx_shape_set_simulation_filter_data(s.h, &cdata)))
}

// SetQueryFilterData sets query filter data.
func (s *ShapeHandle) SetQueryFilterData(data FilterData) error {
	cdata := data.toC()
	return errOrNil(int(C.physx_shape_set_query_filter_data(s.h, &cdata)))
}

// SetContactOffset sets the contact offset distance.
func (s *ShapeHandle) SetContactOffset(offset float32) error {
	return errOrNil(int(C.physx_shape_set_contact_offset(s.h, C.float(offset))))
}

// GetContactOffset returns the contact offset.
func (s *ShapeHandle) GetContactOffset() float32 {
	return float32(C.physx_shape_get_contact_offset(s.h))
}

// ── Actor shape attachment ──────────────────────────────────────────────────

// AttachShape attaches a shape to this actor.
func (a *ActorHandle) AttachShape(shape *ShapeHandle) error {
	return errOrNil(int(C.physx_actor_attach_shape(a.h, shape.h)))
}

// DetachShape detaches a shape from this actor.
func (a *ActorHandle) DetachShape(shape *ShapeHandle) error {
	return errOrNil(int(C.physx_actor_detach_shape(a.h, shape.h)))
}
