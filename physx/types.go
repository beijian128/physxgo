package physx

/*
#include "bridge.h"
*/
import "C"
import "fmt"

// Vec3 is a 3D vector (x, y, z) — layout-compatible with PxVec3.
type Vec3 struct{ X, Y, Z float32 }

// ZeroVec3 returns the zero vector.
func ZeroVec3() Vec3 { return Vec3{} }

// NewVec3 creates a Vec3.
func NewVec3(x, y, z float32) Vec3 { return Vec3{x, y, z} }

func (v Vec3) toC() C.CPxVec3    { return C.CPxVec3{x: C.float(v.X), y: C.float(v.Y), z: C.float(v.Z)} }
func vec3FromC(c C.CPxVec3) Vec3 { return Vec3{float32(c.x), float32(c.y), float32(c.z)} }

func (v Vec3) String() string { return fmt.Sprintf("(%0.3f, %0.3f, %0.3f)", v.X, v.Y, v.Z) }

// Quat is a quaternion (x, y, z, w) — layout-compatible with PxQuat.
// (x,y,z) = imaginary part, w = real part.
type Quat struct{ X, Y, Z, W float32 }

// IdentityQuat returns the identity quaternion.
func IdentityQuat() Quat { return Quat{0, 0, 0, 1} }

// NewQuat creates a Quat.
func NewQuat(x, y, z, w float32) Quat { return Quat{x, y, z, w} }

func (q Quat) toC() C.CPxQuat {
	return C.CPxQuat{x: C.float(q.X), y: C.float(q.Y), z: C.float(q.Z), w: C.float(q.W)}
}
func quatFromC(c C.CPxQuat) Quat { return Quat{float32(c.x), float32(c.y), float32(c.z), float32(c.w)} }

func (q Quat) String() string { return fmt.Sprintf("(%0.3f, %0.3f, %0.3f, %0.3f)", q.X, q.Y, q.Z, q.W) }

// Transform is a pose: rotation (q) followed by translation (p).
type Transform struct {
	Q Quat
	P Vec3
}

// IdentityTransform returns the identity transform.
func IdentityTransform() Transform { return Transform{Q: IdentityQuat(), P: ZeroVec3()} }

// NewTransform creates a Transform from position and quaternion.
func NewTransform(px, py, pz, qx, qy, qz, qw float32) Transform {
	return Transform{P: NewVec3(px, py, pz), Q: NewQuat(qx, qy, qz, qw)}
}

func (t Transform) toC() C.CPxTransform {
	return C.CPxTransform{q: t.Q.toC(), p: t.P.toC()}
}
func transformFromC(c C.CPxTransform) Transform {
	return Transform{Q: quatFromC(c.q), P: vec3FromC(c.p)}
}

// Bounds3 is a 3D axis-aligned bounding box.
type Bounds3 struct {
	Min, Max Vec3
}

// FilterData is collision/query filter data (4 x uint32 words).
type FilterData struct {
	Word0, Word1, Word2, Word3 uint32
}

func (f FilterData) toC() C.CPxFilterData {
	return C.CPxFilterData{
		word0: C.uint32_t(f.Word0),
		word1: C.uint32_t(f.Word1),
		word2: C.uint32_t(f.Word2),
		word3: C.uint32_t(f.Word3),
	}
}
