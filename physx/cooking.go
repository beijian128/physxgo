package physx

/*
#include "bridge.h"
*/
import "C"
import "runtime"

// ──────────────────────────────────────────────────────────────────────────────
// Cooking (mesh generation)
// ──────────────────────────────────────────────────────────────────────────────

// CookingHandle wraps a PxCooking instance.
type CookingHandle struct{ h C.PxCookingHandle }

// ConvexMeshHandle wraps a PxConvexMesh.
type ConvexMeshHandle struct{ h C.PxConvexMeshHandle }

// TriangleMeshHandle wraps a PxTriangleMesh.
type TriangleMeshHandle struct{ h C.PxTriangleMeshHandle }

// CreateCooking creates a cooking instance for mesh generation.
func CreateCooking() *CookingHandle {
	h := C.physx_create_cooking()
	if h == nil {
		return nil
	}
	c := &CookingHandle{h: h}
	runtime.SetFinalizer(c, func(c *CookingHandle) { c.Release() })
	return c
}

// Release releases the cooking instance.
func (c *CookingHandle) Release() {
	if c == nil || c.h == nil {
		return
	}
	C.physx_release_cooking(c.h)
	c.h = nil
}

// CookConvexMesh cooks a convex mesh from vertex data.
// Returns the convex mesh and an error code (0 = success).
func (c *CookingHandle) CookConvexMesh(vertices []Vec3) (*ConvexMeshHandle, int) {
	if len(vertices) < 3 {
		return nil, -1
	}
	cVerts := make([]C.CPxVec3, len(vertices))
	for i, v := range vertices {
		cVerts[i] = v.toC()
	}
	var outErr C.int
	h := C.physx_cook_convex_mesh(c.h, &cVerts[0], C.int(len(vertices)), &outErr)
	if h == nil {
		return nil, int(outErr)
	}
	m := &ConvexMeshHandle{h: h}
	runtime.SetFinalizer(m, func(m *ConvexMeshHandle) { m.Release() })
	return m, int(outErr)
}

// CookTriangleMesh cooks a triangle mesh from vertex and index data.
// Indices are triples (every 3 indices = 1 triangle).
func (c *CookingHandle) CookTriangleMesh(vertices []Vec3, indices []uint32) (*TriangleMeshHandle, int) {
	if len(vertices) < 3 || len(indices) < 3 {
		return nil, -1
	}
	cVerts := make([]C.CPxVec3, len(vertices))
	for i, v := range vertices {
		cVerts[i] = v.toC()
	}
	cIndices := make([]C.uint32_t, len(indices))
	for i, idx := range indices {
		cIndices[i] = C.uint32_t(idx)
	}
	var outErr C.int
	h := C.physx_cook_triangle_mesh(c.h, &cVerts[0], C.int(len(vertices)),
		&cIndices[0], C.int(len(indices)), &outErr)
	if h == nil {
		return nil, int(outErr)
	}
	m := &TriangleMeshHandle{h: h}
	runtime.SetFinalizer(m, func(m *TriangleMeshHandle) { m.Release() })
	return m, int(outErr)
}

// Release releases the convex mesh.
func (m *ConvexMeshHandle) Release() {
	if m == nil || m.h == nil {
		return
	}
	C.physx_release_convex_mesh(m.h)
	m.h = nil
}

// Release releases the triangle mesh.
func (m *TriangleMeshHandle) Release() {
	if m == nil || m.h == nil {
		return
	}
	C.physx_release_triangle_mesh(m.h)
	m.h = nil
}

// CreateConvexMeshShape creates a shape from a cooked convex mesh.
func CreateConvexMeshShape(physics *PhysicsHandle, mesh *ConvexMeshHandle, mat *MaterialHandle, exclusive bool) *ShapeHandle {
	exc := boolToInt(exclusive)
	h := C.physx_create_convex_mesh_shape(physics.h, mesh.h, mat.h, C.int(exc))
	if h == nil {
		return nil
	}
	s := &ShapeHandle{h: h}
	runtime.SetFinalizer(s, func(s *ShapeHandle) { s.Release() })
	return s
}

// CreateTriangleMeshShape creates a shape from a cooked triangle mesh.
func CreateTriangleMeshShape(physics *PhysicsHandle, mesh *TriangleMeshHandle, mat *MaterialHandle, exclusive bool) *ShapeHandle {
	exc := boolToInt(exclusive)
	h := C.physx_create_triangle_mesh_shape(physics.h, mesh.h, mat.h, C.int(exc))
	if h == nil {
		return nil
	}
	s := &ShapeHandle{h: h}
	runtime.SetFinalizer(s, func(s *ShapeHandle) { s.Release() })
	return s
}
