package physx

/*
#include "bridge.h"
*/
import "C"
import (
	"errors"
	"runtime"
)

// ──────────────────────────────────────────────────────────────────────────────
// Material
// ──────────────────────────────────────────────────────────────────────────────

// MaterialHandle wraps a PxMaterial.
type MaterialHandle struct{ h C.PxMaterialHandle }

// Material combine modes.
const (
	CombineAverage  = 0
	CombineMin      = 1
	CombineMultiply = 2
	CombineMax      = 3
)

// CreateMaterial creates a physics material.
func CreateMaterial(physics *PhysicsHandle, staticFriction, dynamicFriction, restitution float32) *MaterialHandle {
	h := C.physx_create_material(physics.h, C.float(staticFriction), C.float(dynamicFriction), C.float(restitution))
	if h == nil {
		return nil
	}
	m := &MaterialHandle{h: h}
	runtime.SetFinalizer(m, func(m *MaterialHandle) { m.Release() })
	return m
}

// Release releases the material.
func (m *MaterialHandle) Release() {
	if m == nil || m.h == nil {
		return
	}
	C.physx_release_material(m.h)
	m.h = nil
}

// SetFriction sets both static and dynamic friction.
func (m *MaterialHandle) SetFriction(sf, df float32) error {
	return errOrNil(int(C.physx_material_set_friction(m.h, C.float(sf), C.float(df))))
}

// GetFriction returns static and dynamic friction.
func (m *MaterialHandle) GetFriction() (float32, float32) {
	var sf, df C.float
	C.physx_material_get_friction(m.h, &sf, &df)
	return float32(sf), float32(df)
}

// SetRestitution sets restitution (bounciness).
func (m *MaterialHandle) SetRestitution(r float32) error {
	return errOrNil(int(C.physx_material_set_restitution(m.h, C.float(r))))
}

// GetRestitution returns restitution.
func (m *MaterialHandle) GetRestitution() float32 {
	return float32(C.physx_material_get_restitution(m.h))
}

// SetFrictionCombineMode sets friction combine mode.
func (m *MaterialHandle) SetFrictionCombineMode(mode int) error {
	return errOrNil(int(C.physx_material_set_friction_combine_mode(m.h, C.int(mode))))
}

// SetRestitutionCombineMode sets restitution combine mode.
func (m *MaterialHandle) SetRestitutionCombineMode(mode int) error {
	return errOrNil(int(C.physx_material_set_restitution_combine_mode(m.h, C.int(mode))))
}

// ──────────────────────────────────────────────────────────────────────────────
// Error helpers
// ──────────────────────────────────────────────────────────────────────────────

func errOrNil(code int) error {
	if code == 0 {
		return nil
	}
	return errors.New(C.GoString(C.physx_get_last_error_message()))
}
