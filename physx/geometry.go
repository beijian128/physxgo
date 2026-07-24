package physx

/*
#include "bridge.h"
*/
import "C"

// ──────────────────────────────────────────────────────────────────────────────
// Geometry types (Go-side wrappers around C geometry structs)
// ──────────────────────────────────────────────────────────────────────────────

// GeometryType identifies the type of geometry.
type GeometryType int

const (
	GeomSphere       GeometryType = 0
	GeomPlane        GeometryType = 1
	GeomCapsule      GeometryType = 2
	GeomBox          GeometryType = 3
	GeomConvexMesh   GeometryType = 4
	GeomTriangleMesh GeometryType = 5
	GeomHeightField  GeometryType = 6
)

// SphereGeometry describes a sphere collision shape.
type SphereGeometry struct {
	Radius float32
}

func (g SphereGeometry) toC() C.CPxSphereGeometry {
	return C.CPxSphereGeometry{
		_type:  C.CPxGeometryType(C.CPxGeometryType_SPHERE),
		radius: C.float(g.Radius),
	}
}

// BoxGeometry describes a box collision shape.
type BoxGeometry struct {
	HalfExtentsX, HalfExtentsY, HalfExtentsZ float32
}

func (g BoxGeometry) toC() C.CPxBoxGeometry {
	return C.CPxBoxGeometry{
		_type:         C.CPxGeometryType(C.CPxGeometryType_BOX),
		halfExtentsX:  C.float(g.HalfExtentsX),
		halfExtentsY:  C.float(g.HalfExtentsY),
		halfExtentsZ:  C.float(g.HalfExtentsZ),
	}
}

// CapsuleGeometry describes a capsule collision shape.
type CapsuleGeometry struct {
	Radius     float32
	HalfHeight float32
}

func (g CapsuleGeometry) toC() C.CPxCapsuleGeometry {
	return C.CPxCapsuleGeometry{
		_type:      C.CPxGeometryType(C.CPxGeometryType_CAPSULE),
		radius:     C.float(g.Radius),
		halfHeight: C.float(g.HalfHeight),
	}
}

// PlaneGeometry describes an infinite plane collision shape (x <= 0 in local space).
type PlaneGeometry struct{}

func (g PlaneGeometry) toC() C.CPxPlaneGeometry {
	return C.CPxPlaneGeometry{
		_type: C.CPxGeometryType(C.CPxGeometryType_PLANE),
	}
}
