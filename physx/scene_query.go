package physx

/*
#include "bridge.h"
*/
import "C"
import "unsafe"

// ──────────────────────────────────────────────────────────────────────────────
// Scene Queries: Raycast, Sweep, Overlap
// ──────────────────────────────────────────────────────────────────────────────

// Hit flags.
const (
	HitFlagPosition  = 1 << 0
	HitFlagNormal    = 1 << 1
	HitFlagDistance  = 1 << 2
	HitFlagUV        = 1 << 3
	HitFlagFaceIndex = 1 << 7
)

// Query flags.
const (
	QueryFlagStatic     = 1 << 0
	QueryFlagDynamic    = 1 << 1
	QueryFlagPrefilter  = 1 << 2
	QueryFlagPostfilter = 1 << 3
	QueryFlagAnyHit     = 1 << 4
	QueryFlagNoBlock    = 1 << 5
)

// RaycastHit describes a raycast result.
type RaycastHit struct {
	FaceIndex uint32
	Position  Vec3
	Normal    Vec3
	Distance  float32
	Flags     uint32
	Actor     *ActorHandle
	Shape     *ShapeHandle
}

func raycastHitFromC(c C.CPxRaycastHit) RaycastHit {
	actor := &ActorHandle{h: C.PxActorHandle(unsafe.Pointer(uintptr(c.actor)))}
	shape := &ShapeHandle{h: C.PxShapeHandle(unsafe.Pointer(uintptr(c.shape)))}
	return RaycastHit{
		FaceIndex: uint32(c.faceIndex),
		Position:  vec3FromC(c.position),
		Normal:    vec3FromC(c.normal),
		Distance:  float32(c.distance),
		Flags:     uint32(c.flags),
		Actor:     actor,
		Shape:     shape,
	}
}

// SweepHit describes a sweep result.
type SweepHit struct {
	FaceIndex uint32
	Position  Vec3
	Normal    Vec3
	Distance  float32
	Flags     uint32
	Actor     *ActorHandle
	Shape     *ShapeHandle
}

func sweepHitFromC(c C.CPxSweepHit) SweepHit {
	return SweepHit{
		FaceIndex: uint32(c.faceIndex),
		Position:  vec3FromC(c.position),
		Normal:    vec3FromC(c.normal),
		Distance:  float32(c.distance),
		Flags:     uint32(c.flags),
		Actor:     &ActorHandle{h: C.PxActorHandle(unsafe.Pointer(uintptr(c.actor)))},
		Shape:     &ShapeHandle{h: C.PxShapeHandle(unsafe.Pointer(uintptr(c.shape)))},
	}
}

// OverlapHit describes an overlap result.
type OverlapHit struct {
	FaceIndex uint32
	Actor     *ActorHandle
	Shape     *ShapeHandle
}

func overlapHitFromC(c C.CPxOverlapHit) OverlapHit {
	return OverlapHit{
		FaceIndex: uint32(c.faceIndex),
		Actor:     &ActorHandle{h: C.PxActorHandle(unsafe.Pointer(uintptr(c.actor)))},
		Shape:     &ShapeHandle{h: C.PxShapeHandle(unsafe.Pointer(uintptr(c.shape)))},
	}
}

// Raycast performs a raycast against the scene.
func (s *SceneHandle) Raycast(origin, direction Vec3, maxDist float32, hitFlags, queryFlags uint32, filterData *FilterData, maxHits int) []RaycastHit {
	buf := make([]C.CPxRaycastHit, maxHits)
	co := origin.toC()
	cd := direction.toC()
	var fd *C.CPxFilterData
	if filterData != nil {
		f := filterData.toC()
		fd = &f
	}
	count := C.physx_scene_raycast(s.h, &co, &cd, C.float(maxDist),
		C.uint32_t(hitFlags), C.uint32_t(queryFlags), fd, &buf[0], C.int(maxHits))
	result := make([]RaycastHit, int(count))
	for i := 0; i < int(count); i++ {
		result[i] = raycastHitFromC(buf[i])
	}
	return result
}

// Sweep performs a geometry sweep against the scene.
func (s *SceneHandle) Sweep(geometry interface{}, geomType GeometryType, pose Transform, direction Vec3, maxDist float32, hitFlags, queryFlags uint32, filterData *FilterData, maxHits int) []SweepHit {
	buf := make([]C.CPxSweepHit, maxHits)

	var cGeom unsafe.Pointer
	cType := C.int(geomType)
	switch g := geometry.(type) {
	case SphereGeometry:
		cg := g.toC()
		cGeom = unsafe.Pointer(&cg)
	case BoxGeometry:
		cg := g.toC()
		cGeom = unsafe.Pointer(&cg)
	case CapsuleGeometry:
		cg := g.toC()
		cGeom = unsafe.Pointer(&cg)
	case PlaneGeometry:
		cg := g.toC()
		cGeom = unsafe.Pointer(&cg)
	}

	cp := pose.toC()
	cd := direction.toC()
	var fd *C.CPxFilterData
	if filterData != nil {
		f := filterData.toC()
		fd = &f
	}
	count := C.physx_scene_sweep(s.h, cGeom, cType, &cp, &cd, C.float(maxDist),
		C.uint32_t(hitFlags), C.uint32_t(queryFlags), fd, &buf[0], C.int(maxHits))
	result := make([]SweepHit, int(count))
	for i := 0; i < int(count); i++ {
		result[i] = sweepHitFromC(buf[i])
	}
	return result
}

// Overlap performs an overlap test against the scene.
func (s *SceneHandle) Overlap(geometry interface{}, geomType GeometryType, pose Transform, queryFlags uint32, filterData *FilterData, maxHits int) []OverlapHit {
	buf := make([]C.CPxOverlapHit, maxHits)

	var cGeom unsafe.Pointer
	cType := C.int(geomType)
	switch g := geometry.(type) {
	case SphereGeometry:
		cg := g.toC()
		cGeom = unsafe.Pointer(&cg)
	case BoxGeometry:
		cg := g.toC()
		cGeom = unsafe.Pointer(&cg)
	case CapsuleGeometry:
		cg := g.toC()
		cGeom = unsafe.Pointer(&cg)
	case PlaneGeometry:
		cg := g.toC()
		cGeom = unsafe.Pointer(&cg)
	}

	cp := pose.toC()
	var fd *C.CPxFilterData
	if filterData != nil {
		f := filterData.toC()
		fd = &f
	}
	count := C.physx_scene_overlap(s.h, cGeom, cType, &cp,
		C.uint32_t(queryFlags), fd, &buf[0], C.int(maxHits))
	result := make([]OverlapHit, int(count))
	for i := 0; i < int(count); i++ {
		result[i] = overlapHitFromC(buf[i])
	}
	return result
}
