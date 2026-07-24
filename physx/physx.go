// Package physx provides Go bindings for the NVIDIA PhysX 3.4 physics engine via cgo.
package physx

/*
#cgo CXXFLAGS: -std=c++11 -DNDEBUG
#cgo LDFLAGS: -lPhysX3Extensions -lPhysX3_x64 -lPhysX3Common_x64 -lPhysX3CharacterKinematic_x64 -lPhysX3Cooking_x64
#cgo LDFLAGS: -lPxFoundation_x64 -lPxPvdSDK_x64
#cgo LDFLAGS: -lstdc++ -lm -lpthread -ldl

#include <stdlib.h>
#include "bridge.h"
*/
import "C"
import (
	"runtime"
	"unsafe"
)

// ──────────────────────────────────────────────────────────────────────────────
// Foundation
// ──────────────────────────────────────────────────────────────────────────────

// FoundationHandle wraps a PxFoundation.
type FoundationHandle struct{ h C.PxFoundationHandle }

// CreateFoundation creates the PhysX foundation. Must be called first.
func CreateFoundation() *FoundationHandle {
	h := C.physx_create_foundation()
	if h == nil {
		return nil
	}
	f := &FoundationHandle{h: h}
	runtime.SetFinalizer(f, func(f *FoundationHandle) { f.Release() })
	return f
}

// Release releases the foundation.
func (f *FoundationHandle) Release() {
	if f == nil || f.h == nil {
		return
	}
	C.physx_release_foundation(f.h)
	f.h = nil
}

// ──────────────────────────────────────────────────────────────────────────────
// Physics
// ──────────────────────────────────────────────────────────────────────────────

// PhysicsHandle wraps a PxPhysics instance.
type PhysicsHandle struct{ h C.PxPhysicsHandle }

// CreatePhysics creates a PxPhysics instance, optionally with PVD.
// Pass "" for pvdHost to disable PVD.
func CreatePhysics(foundation *FoundationHandle, pvdHost string) *PhysicsHandle {
	var cHost *C.char
	if pvdHost != "" {
		cHost = C.CString(pvdHost)
		defer C.free(unsafe.Pointer(cHost))
	}
	h := C.physx_create_physics(foundation.h, cHost)
	if h == nil {
		return nil
	}
	p := &PhysicsHandle{h: h}
	runtime.SetFinalizer(p, func(p *PhysicsHandle) { p.Release() })
	return p
}

// Release releases the physics instance.
func (p *PhysicsHandle) Release() {
	if p == nil || p.h == nil {
		return
	}
	C.physx_release_physics(p.h)
	p.h = nil
}

// ──────────────────────────────────────────────────────────────────────────────
// Error reporting
// ──────────────────────────────────────────────────────────────────────────────

// LastErrorCode returns the last bridge error code (0 = success).
func LastErrorCode() int { return int(C.physx_get_last_error_code()) }

// LastErrorMessage returns the last bridge error message.
func LastErrorMessage() string { return C.GoString(C.physx_get_last_error_message()) }
