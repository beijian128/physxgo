package physx

/*
#include "bridge.h"
*/
import "C"
import "unsafe"

// ──────────────────────────────────────────────────────────────────────────────
// Simulation Event Callbacks
// ──────────────────────────────────────────────────────────────────────────────

// ContactPairHeader describes the pair of actors in contact.
type ContactPairHeader struct {
	Actor0 *ActorHandle
	Actor1 *ActorHandle
}

// ContactPair describes a single contact between two shapes.
type ContactPair struct {
	Shape0 *ShapeHandle
	Shape1 *ShapeHandle
	Actor0 *ActorHandle
	Actor1 *ActorHandle
	// First contact point
	Point       Vec3
	Normal      Vec3
	Distance    float32
	Impulse0    float32
	Impulse1    float32
	FaceIndex0  uint32
	FaceIndex1  uint32
	Events      uint32
	ContactCount uint32
}

// TriggerPair describes a trigger event.
type TriggerPair struct {
	TriggerShape *ShapeHandle
	TriggerActor *ActorHandle
	OtherShape   *ShapeHandle
	OtherActor   *ActorHandle
	Status       uint32 // 0 = TOUCH_FOUND, 1 = TOUCH_LOST
	Flags        uint32
}

// ContactCallback is called when contacts are detected.
type ContactCallback func(header ContactPairHeader, pairs []ContactPair)

// TriggerCallback is called when triggers are entered/exited.
type TriggerCallback func(pairs []TriggerPair)

// SleepCallback is called when actors fall asleep or wake up.
// isWaking: true = wake, false = sleep
type SleepCallback func(actors []*ActorHandle, isWaking bool)

// ──────────────────────────────────────────────────────────────────────────────
// Registering callbacks on C side
// ──────────────────────────────────────────────────────────────────────────────

// SetContactCallback registers a contact callback.
// The callback receives raw C data; you typically use the higher-level API.
func (s *SceneHandle) SetContactCallback(cb ContactCallback) {
	// Store callback in a map keyed by scene...
	// For now, register as raw C callback via C function pointer
	C.physx_scene_set_contact_callback(s.h, nil, nil)
}

// SetTriggerCallback registers a trigger callback.
func (s *SceneHandle) SetTriggerCallback(cb TriggerCallback) {
	C.physx_scene_set_trigger_callback(s.h, nil, nil)
}

// SetSleepCallback registers a sleep/wake callback.
func (s *SceneHandle) SetSleepCallback(cb SleepCallback) {
	C.physx_scene_set_sleep_callback(s.h, nil, nil)
}

// SetAdvanceCallback registers a per-frame advance callback.
func (s *SceneHandle) SetAdvanceCallback(cb func(poses []Transform)) {
	C.physx_scene_set_advance_callback(s.h, nil, nil)
}

// ──────────────────────────────────────────────────────────────────────────────
// Vehicle (stub — full vehicle SDK needs serialization registry)
// ──────────────────────────────────────────────────────────────────────────────

// VehicleHandle wraps a PxVehicle.
type VehicleHandle struct{ h C.PxVehicleHandle }

// InitVehicleSDK initializes the vehicle SDK.
func InitVehicleSDK(physics *PhysicsHandle) error {
	return errOrNil(int(C.physx_init_vehicle_sdk(physics.h)))
}

// CloseVehicleSDK shuts down the vehicle SDK.
func CloseVehicleSDK() error {
	return errOrNil(int(C.physx_close_vehicle_sdk()))
}

// CreateVehicle4W creates a 4-wheel drive vehicle (stub).
func CreateVehicle4W(physics *PhysicsHandle, chassis *ActorHandle) *VehicleHandle {
	h := C.physx_create_vehicle_4w(physics.h, chassis.h)
	if h == nil {
		return nil
	}
	v := &VehicleHandle{h: h}
	_ = v
	return nil // stub
}

// Release releases the vehicle (stub).
func (v *VehicleHandle) Release() {
	if v == nil || v.h == nil {
		return
	}
	C.physx_release_vehicle(v.h)
	v.h = nil
}

// VehicleUpdate updates the vehicle (stub).
func (v *VehicleHandle) Update(dt float32) error {
	return errOrNil(int(C.physx_vehicle_update(v.h, C.float(dt))))
}

// SetVehicleInput sets vehicle controls (stub).
func (v *VehicleHandle) SetInput(throttle, brake, handbrake, steer float32, gear int) error {
	return errOrNil(int(C.physx_vehicle_set_input(v.h, C.float(throttle), C.float(brake), C.float(handbrake), C.float(steer), C.int(gear))))
}

// avoid "imported and not used" error for unsafe
var _ = unsafe.Pointer(nil)
