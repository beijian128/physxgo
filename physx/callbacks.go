// Package physx — Go API for simulation event callbacks.
package physx

/*
#include "bridge.h"

// C trampolines (implemented in bridge.cpp) that forward to Go //export callbacks.
extern uint32_t cFilterShaderTramp(uint32_t attr0, const CPxFilterData* fd0,
    uint32_t attr1, const CPxFilterData* fd1, uint32_t* pairFlags, void* userdata);
extern void cContactTramp(void* userdata, const CPxContactPairHeader* header,
    const CPxContactPair* pairs, int nbPairs);
extern void cTriggerTramp(void* userdata, const CPxTriggerPair* pairs, int nbPairs);
extern void cSleepTramp(void* userdata, PxActorHandle* actors, int nbActors, int isWaking);
extern void cContactModifyTramp(void* userdata, const CPxContactModifyPair* pairs, int nbPairs);

// Go //export callbacks receive uintptr_t userdata (not unsafe.Pointer, for cgo compat).
extern uint32_t goFilterShaderCB(uintptr_t userdata, uint32_t attr0, CPxFilterData* fd0,
    uint32_t attr1, CPxFilterData* fd1, uint32_t* pairFlags);
extern void goContactCB(uintptr_t userdata, CPxContactPairHeader* header,
    CPxContactPair* pairs, int nbPairs);
extern void goTriggerCB(uintptr_t userdata, CPxTriggerPair* pairs, int nbPairs);
extern void goSleepCB(uintptr_t userdata, PxActorHandle* actors, int nbActors, int isWaking);
extern void goContactModifyCB(uintptr_t userdata, CPxContactModifyPair* pairs, int nbPairs);
*/
import "C"
import (
	"sync"
	"unsafe"
)

// ── Helper: get typed C func ptrs from trampolines ─────────────────────────

func _cFilterShaderTramp() C.PhysxFilterShaderCallback {
	return C.PhysxFilterShaderCallback(C.cFilterShaderTramp)
}
func _cContactTramp() C.PhysxContactCallback {
	return C.PhysxContactCallback(C.cContactTramp)
}
func _cTriggerTramp() C.PhysxTriggerCallback {
	return C.PhysxTriggerCallback(C.cTriggerTramp)
}
func _cSleepTramp() C.PhysxSleepCallback {
	return C.PhysxSleepCallback(C.cSleepTramp)
}
func _cContactModifyTramp() C.PhysxContactModifyCallback {
	return C.PhysxContactModifyCallback(C.cContactModifyTramp)
}

// ── Exported Go callbacks (called by C trampolines in bridge.cpp) ────────────

//export goFilterShaderCB
func goFilterShaderCB(userdata C.uintptr_t,
	attr0 C.uint32_t, fd0 *C.CPxFilterData,
	attr1 C.uint32_t, fd1 *C.CPxFilterData,
	pairFlags *C.uint32_t,
) C.uint32_t {
	return _dispatchFilterShader(unsafe.Pointer(uintptr(userdata)), attr0, fd0, attr1, fd1, pairFlags)
}

//export goContactCB
func goContactCB(userdata C.uintptr_t,
	header *C.CPxContactPairHeader,
	pairs *C.CPxContactPair, nbPairs C.int) {
	_dispatchContact(unsafe.Pointer(uintptr(userdata)), header, pairs, nbPairs)
}

//export goTriggerCB
func goTriggerCB(userdata C.uintptr_t, pairs *C.CPxTriggerPair, nbPairs C.int) {
	_dispatchTrigger(unsafe.Pointer(uintptr(userdata)), pairs, nbPairs)
}

//export goSleepCB
func goSleepCB(userdata C.uintptr_t, actors *C.PxActorHandle, nbActors C.int, isWaking C.int) {
	_dispatchSleep(unsafe.Pointer(uintptr(userdata)), actors, nbActors, isWaking)
}

//export goContactModifyCB
func goContactModifyCB(userdata C.uintptr_t,
	pairs *C.CPxContactModifyPair, nbPairs C.int) {
	_dispatchContactModify(unsafe.Pointer(uintptr(userdata)), pairs, nbPairs)
}

// ──────────────────────────────────────────────────────────────────────────────
// Callback registry
// ──────────────────────────────────────────────────────────────────────────────

var (
	filterShaderCbs  = make(map[unsafe.Pointer]FilterShaderCallback)
	filterShaderMu   sync.Mutex

	contactCbs       = make(map[unsafe.Pointer]ContactCallback)
	contactMu        sync.Mutex

	triggerCbs       = make(map[unsafe.Pointer]TriggerCallback)
	triggerMu        sync.Mutex

	sleepCbs         = make(map[unsafe.Pointer]SleepCallback)
	sleepMu          sync.Mutex

	contactModifyCbs = make(map[unsafe.Pointer]ContactModifyCallback)
	contactModifyMu  sync.Mutex
)

// ──────────────────────────────────────────────────────────────────────────────
// Pair / Filter flag constants
// ──────────────────────────────────────────────────────────────────────────────

const (
	PairFlagSolveContact                = 1 << 0
	PairFlagModifyContacts              = 1 << 1
	PairFlagNotifyTouchFound            = 1 << 2
	PairFlagNotifyTouchPersists         = 1 << 3
	PairFlagNotifyTouchLost             = 1 << 4
	PairFlagNotifyTouchCCD              = 1 << 5
	PairFlagNotifyThresholdForceFound   = 1 << 6
	PairFlagNotifyThresholdForcePersists = 1 << 7
	PairFlagNotifyThresholdForceLost    = 1 << 8
	PairFlagNotifyContactPoints         = 1 << 9
	PairFlagDetectDiscreteContact       = 1 << 10
	PairFlagDetectCCDContact            = 1 << 11
	PairFlagPreSolverVelocity           = 1 << 12
	PairFlagPostSolverVelocity          = 1 << 13
	PairFlagContactEventPose            = 1 << 14
)

const (
	FilterFlagDefault  = 0
	FilterFlagKill     = 1 << 0
	FilterFlagSuppress = 1 << 1
	FilterFlagCallback = 1 << 2
	FilterFlagNotify   = 1 << 3
)

const PairFlagContactDefault = PairFlagSolveContact | PairFlagDetectDiscreteContact

// ──────────────────────────────────────────────────────────────────────────────
// Filter Shader
// ──────────────────────────────────────────────────────────────────────────────

type FilterShaderCallback func(
	attributes0 uint32, fd0 *FilterData,
	attributes1 uint32, fd1 *FilterData,
) (pairFlags uint32, filterFlags uint32)

func _dispatchFilterShader(userdata unsafe.Pointer,
	attr0 C.uint32_t, fd0 *C.CPxFilterData,
	attr1 C.uint32_t, fd1 *C.CPxFilterData,
	pairFlags *C.uint32_t,
) C.uint32_t {
	filterShaderMu.Lock()
	cb := filterShaderCbs[userdata]
	filterShaderMu.Unlock()
	if cb == nil {
		return C.uint32_t(FilterFlagDefault)
	}
	gfd0 := FilterData{
		Word0: uint32(fd0.word0), Word1: uint32(fd0.word1),
		Word2: uint32(fd0.word2), Word3: uint32(fd0.word3),
	}
	gfd1 := FilterData{
		Word0: uint32(fd1.word0), Word1: uint32(fd1.word1),
		Word2: uint32(fd1.word2), Word3: uint32(fd1.word3),
	}
	pf, ff := cb(uint32(attr0), &gfd0, uint32(attr1), &gfd1)
	if pairFlags != nil {
		*pairFlags = C.uint32_t(pf)
	}
	return C.uint32_t(ff)
}

func (s *SceneHandle) SetFilterShader(cb FilterShaderCallback) error {
	key := unsafe.Pointer(s.h)
	filterShaderMu.Lock()
	if cb != nil {
		filterShaderCbs[key] = cb
	} else {
		delete(filterShaderCbs, key)
	}
	filterShaderMu.Unlock()
	var cCB C.PhysxFilterShaderCallback
	if cb != nil {
		cCB = _cFilterShaderTramp()
	}
	return errOrNil(int(C.physx_scene_set_filter_shader(s.h, cCB, key)))
}

// ──────────────────────────────────────────────────────────────────────────────
// Contact callback
// ──────────────────────────────────────────────────────────────────────────────

type ContactPairHeader struct {
	Actor0 *ActorHandle
	Actor1 *ActorHandle
}

type ContactPair struct {
	Actor0             *ActorHandle
	Actor1             *ActorHandle
	Shape0             *ShapeHandle
	Shape1             *ShapeHandle
	ContactPoint       Vec3
	ContactNormal      Vec3
	ContactDistance    float32
	Impulse            Vec3
	InternalFaceIndex0 uint32
	InternalFaceIndex1 uint32
	Events             uint32
	ContactCount       uint32
}

type ContactPairPoint struct {
	Position           Vec3
	Normal             Vec3
	Impulse            Vec3
	Separation         float32
	InternalFaceIndex0 uint32
	InternalFaceIndex1 uint32
}

func (cp *ContactPair) ExtractContacts(maxPoints int) []ContactPairPoint {
	if cp == nil || maxPoints <= 0 {
		return nil
	}
	buf := make([]C.CPxContactPairPoint, maxPoints)
	ccp := C.CPxContactPair{
		shapes: [2]C.uint64_t{
			C.uint64_t(uintptr(unsafe.Pointer(cp.Shape0.h))),
			C.uint64_t(uintptr(unsafe.Pointer(cp.Shape1.h))),
		},
	}
	n := C.physx_contact_pair_extract_contacts(&ccp, &buf[0], C.int(maxPoints))
	result := make([]ContactPairPoint, int(n))
	for i := 0; i < int(n); i++ {
		result[i] = ContactPairPoint{
			Position:           vec3FromC(buf[i].position),
			Normal:             vec3FromC(buf[i].normal),
			Impulse:            vec3FromC(buf[i].impulse),
			Separation:         float32(buf[i].separation),
			InternalFaceIndex0: uint32(buf[i].internalFaceIndex0),
			InternalFaceIndex1: uint32(buf[i].internalFaceIndex1),
		}
	}
	return result
}

type ContactCallback func(header ContactPairHeader, pairs []ContactPair)

func _dispatchContact(userdata unsafe.Pointer, header *C.CPxContactPairHeader,
	pairs *C.CPxContactPair, nbPairs C.int) {
	contactMu.Lock()
	cb := contactCbs[userdata]
	contactMu.Unlock()
	if cb == nil || nbPairs == 0 {
		return
	}
	hdr := ContactPairHeader{
		Actor0: &ActorHandle{h: C.PxActorHandle(unsafe.Pointer(uintptr(header.actors[0])))},
		Actor1: &ActorHandle{h: C.PxActorHandle(unsafe.Pointer(uintptr(header.actors[1])))},
	}
	n := int(nbPairs)
	gpairs := make([]ContactPair, n)
	cArr := (*[1 << 16]C.CPxContactPair)(unsafe.Pointer(pairs))[:n:n]
	for i := 0; i < n; i++ {
		cpi := &cArr[i]
		gpairs[i] = ContactPair{
			Actor0:             hdr.Actor0,
			Actor1:             hdr.Actor1,
			Shape0:             &ShapeHandle{h: C.PxShapeHandle(unsafe.Pointer(uintptr(cpi.shapes[0])))},
			Shape1:             &ShapeHandle{h: C.PxShapeHandle(unsafe.Pointer(uintptr(cpi.shapes[1])))},
			ContactPoint:       vec3FromC(cpi.contactPoint),
			ContactNormal:      vec3FromC(cpi.contactNormal),
			ContactDistance:    float32(cpi.contactDistance),
			Impulse:            NewVec3(float32(cpi.impulse[0]), float32(cpi.impulse[1]), 0),
			InternalFaceIndex0: uint32(cpi.internalFaceIndex0),
			InternalFaceIndex1: uint32(cpi.internalFaceIndex1),
			Events:             uint32(cpi.events),
			ContactCount:       uint32(cpi.contactCount),
		}
	}
	cb(hdr, gpairs)
}

func (s *SceneHandle) SetContactCallback(cb ContactCallback) error {
	key := unsafe.Pointer(s.h)
	contactMu.Lock()
	if cb != nil {
		contactCbs[key] = cb
	} else {
		delete(contactCbs, key)
	}
	contactMu.Unlock()
	var cCB C.PhysxContactCallback
	if cb != nil {
		cCB = _cContactTramp()
	}
	return errOrNil(int(C.physx_scene_set_contact_callback(s.h, cCB, key)))
}

// ──────────────────────────────────────────────────────────────────────────────
// Trigger callback
// ──────────────────────────────────────────────────────────────────────────────

const (
	TriggerTouchFound = 0
	TriggerTouchLost  = 1
)

type TriggerPair struct {
	TriggerShape *ShapeHandle
	TriggerActor *ActorHandle
	OtherShape   *ShapeHandle
	OtherActor   *ActorHandle
	Status       uint32
	Flags        uint32
}

type TriggerCallback func(pairs []TriggerPair)

func _dispatchTrigger(userdata unsafe.Pointer, pairs *C.CPxTriggerPair, nbPairs C.int) {
	triggerMu.Lock()
	cb := triggerCbs[userdata]
	triggerMu.Unlock()
	if cb == nil || nbPairs == 0 {
		return
	}
	n := int(nbPairs)
	gpairs := make([]TriggerPair, n)
	cArr := (*[1 << 16]C.CPxTriggerPair)(unsafe.Pointer(pairs))[:n:n]
	for i := 0; i < n; i++ {
		gpairs[i] = TriggerPair{
			TriggerShape: &ShapeHandle{h: C.PxShapeHandle(unsafe.Pointer(uintptr(cArr[i].triggerShape)))},
			TriggerActor: &ActorHandle{h: C.PxActorHandle(unsafe.Pointer(uintptr(cArr[i].triggerActor)))},
			OtherShape:   &ShapeHandle{h: C.PxShapeHandle(unsafe.Pointer(uintptr(cArr[i].otherShape)))},
			OtherActor:   &ActorHandle{h: C.PxActorHandle(unsafe.Pointer(uintptr(cArr[i].otherActor)))},
			Status:       uint32(cArr[i].status),
			Flags:        uint32(cArr[i].flags),
		}
	}
	cb(gpairs)
}

func (s *SceneHandle) SetTriggerCallback(cb TriggerCallback) error {
	key := unsafe.Pointer(s.h)
	triggerMu.Lock()
	if cb != nil {
		triggerCbs[key] = cb
	} else {
		delete(triggerCbs, key)
	}
	triggerMu.Unlock()
	var cCB C.PhysxTriggerCallback
	if cb != nil {
		cCB = _cTriggerTramp()
	}
	return errOrNil(int(C.physx_scene_set_trigger_callback(s.h, cCB, key)))
}

// ──────────────────────────────────────────────────────────────────────────────
// Sleep callback
// ──────────────────────────────────────────────────────────────────────────────

type SleepCallback func(actors []*ActorHandle, isWaking bool)

func _dispatchSleep(userdata unsafe.Pointer, actors *C.PxActorHandle, nbActors C.int, isWaking C.int) {
	sleepMu.Lock()
	cb := sleepCbs[userdata]
	sleepMu.Unlock()
	if cb == nil || nbActors == 0 {
		return
	}
	n := int(nbActors)
	gactors := make([]*ActorHandle, n)
	cArr := (*[1 << 16]C.PxActorHandle)(unsafe.Pointer(actors))[:n:n]
	for i := 0; i < n; i++ {
		gactors[i] = &ActorHandle{h: cArr[i]}
	}
	cb(gactors, isWaking != 0)
}

func (s *SceneHandle) SetSleepCallback(cb SleepCallback) error {
	key := unsafe.Pointer(s.h)
	sleepMu.Lock()
	if cb != nil {
		sleepCbs[key] = cb
	} else {
		delete(sleepCbs, key)
	}
	sleepMu.Unlock()
	var cCB C.PhysxSleepCallback
	if cb != nil {
		cCB = _cSleepTramp()
	}
	return errOrNil(int(C.physx_scene_set_sleep_callback(s.h, cCB, key)))
}

// ──────────────────────────────────────────────────────────────────────────────
// Contact Modify callback
// ──────────────────────────────────────────────────────────────────────────────

type ContactModifyPair struct {
	Actor0     *ActorHandle
	Actor1     *ActorHandle
	Shape0     *ShapeHandle
	Shape1     *ShapeHandle
	Transform0 Transform
	Transform1 Transform
}

type ContactModifyCallback func(pairs []ContactModifyPair)

func _dispatchContactModify(userdata unsafe.Pointer,
	pairs *C.CPxContactModifyPair, nbPairs C.int) {
	contactModifyMu.Lock()
	cb := contactModifyCbs[userdata]
	contactModifyMu.Unlock()
	if cb == nil || nbPairs == 0 {
		return
	}
	n := int(nbPairs)
	gpairs := make([]ContactModifyPair, n)
	cArr := (*[1 << 16]C.CPxContactModifyPair)(unsafe.Pointer(pairs))[:n:n]
	for i := 0; i < n; i++ {
		gpairs[i] = ContactModifyPair{
			Actor0:     &ActorHandle{h: C.PxActorHandle(unsafe.Pointer(uintptr(cArr[i].actors[0])))},
			Actor1:     &ActorHandle{h: C.PxActorHandle(unsafe.Pointer(uintptr(cArr[i].actors[1])))},
			Shape0:     &ShapeHandle{h: C.PxShapeHandle(unsafe.Pointer(uintptr(cArr[i].shapes[0])))},
			Shape1:     &ShapeHandle{h: C.PxShapeHandle(unsafe.Pointer(uintptr(cArr[i].shapes[1])))},
			Transform0: transformFromC(cArr[i].transforms[0]),
			Transform1: transformFromC(cArr[i].transforms[1]),
		}
	}
	cb(gpairs)
}

func (s *SceneHandle) SetContactModifyCallback(cb ContactModifyCallback) error {
	key := unsafe.Pointer(s.h)
	contactModifyMu.Lock()
	if cb != nil {
		contactModifyCbs[key] = cb
	} else {
		delete(contactModifyCbs, key)
	}
	contactModifyMu.Unlock()
	var cCB C.PhysxContactModifyCallback
	if cb != nil {
		cCB = _cContactModifyTramp()
	}
	return errOrNil(int(C.physx_scene_set_contact_modify_callback(s.h, cCB, key)))
}

func SetContactModifyInvMassScale(pairIndex, actorIndex int, scale float32) error {
	return errOrNil(int(C.physx_contact_modify_set_inv_mass_scale(
		C.int(pairIndex), C.int(actorIndex), C.float(scale))))
}

func SetContactModifyInvInertiaScale(pairIndex, actorIndex int, scale float32) error {
	return errOrNil(int(C.physx_contact_modify_set_inv_inertia_scale(
		C.int(pairIndex), C.int(actorIndex), C.float(scale))))
}
