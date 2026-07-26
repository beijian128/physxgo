package physx

/*
#include "bridge.h"
*/
import "C"
import "runtime"

// ──────────────────────────────────────────────────────────────────────────────
// Scene
// ──────────────────────────────────────────────────────────────────────────────

// SceneHandle wraps a PxScene.
type SceneHandle struct{ h C.PxSceneHandle }

// CreateScene creates a PhysX simulation scene.
func CreateScene(physics *PhysicsHandle, numThreads int, gx, gy, gz float32) *SceneHandle {
	h := C.physx_create_scene(physics.h, C.int(numThreads), C.float(gx), C.float(gy), C.float(gz))
	if h == nil {
		return nil
	}
	s := &SceneHandle{h: h}
	runtime.SetFinalizer(s, func(s *SceneHandle) { s.Release() })
	return s
}

// Release releases the scene.
func (s *SceneHandle) Release() {
	if s == nil || s.h == nil {
		return
	}
	C.physx_release_scene(s.h)
	s.h = nil
}

// Simulate advances the simulation by dt seconds (blocking).
func (s *SceneHandle) Simulate(dt float32) error {
	return errOrNil(int(C.physx_scene_simulate(s.h, C.float(dt))))
}

// SimulateStart begins an async simulation step.
func (s *SceneHandle) SimulateStart(dt float32) error {
	return errOrNil(int(C.physx_scene_simulate_start(s.h, C.float(dt))))
}

// FetchResults waits for async simulation to finish.
func (s *SceneHandle) FetchResults(block bool) error {
	b := 0
	if block {
		b = 1
	}
	return errOrNil(int(C.physx_scene_fetch_results(s.h, C.int(b))))
}

// SetGravity sets scene gravity.
func (s *SceneHandle) SetGravity(x, y, z float32) error {
	return errOrNil(int(C.physx_scene_set_gravity(s.h, C.float(x), C.float(y), C.float(z))))
}

// GetGravity returns scene gravity.
func (s *SceneHandle) GetGravity() (float32, float32, float32) {
	var x, y, z C.float
	C.physx_scene_get_gravity(s.h, &x, &y, &z)
	return float32(x), float32(y), float32(z)
}

// AddActor adds an actor to the scene.
func (s *SceneHandle) AddActor(actor *ActorHandle) error {
	return errOrNil(int(C.physx_scene_add_actor(s.h, actor.h)))
}

// RemoveActor removes an actor from the scene.
func (s *SceneHandle) RemoveActor(actor *ActorHandle) error {
	return errOrNil(int(C.physx_scene_remove_actor(s.h, actor.h)))
}

// SetPVDFlags enables/disables PVD transmission flags.
func (s *SceneHandle) SetPVDFlags(constraints, contacts, sceneQueries bool) error {
	c, ct, sq := boolToInt(constraints), boolToInt(contacts), boolToInt(sceneQueries)
	return errOrNil(int(C.physx_scene_set_pvd_flags(s.h, C.int(c), C.int(ct), C.int(sq))))
}

// PVD visualization parameters (must match PxVisualizationParameter::Enum).
const (
	VisScale            = 0  // overall scale (master switch, must be >0)
	VisWorldAxes        = 1  // world axes
	VisBodyAxes         = 2  // body axes
	VisBodyMassAxes     = 3  // body mass axes
	VisBodyLinVelocity  = 4  // linear velocity
	VisBodyAngVelocity  = 5  // angular velocity
	// 6 = eDEPRECATED_BODY_JOINT_GROUPS
	VisContactPoint     = 7  // contact points
	VisContactNormal    = 8  // contact normals
	VisContactError     = 9  // contact errors
	VisContactForce     = 10 // contact forces
	VisActorAxes        = 11 // actor axes
	VisCollisionAABBs   = 12 // collision AABBs
	VisCollisionShapes  = 13 // collision shapes
	VisCollisionAxes    = 14 // collision axes
	VisCollisionCompounds = 15 // compound AABBs
	VisCollisionFNormals  = 16 // mesh face normals
	VisCollisionEdges   = 17 // active edges
	VisCollisionStatic  = 18 // static pruning
	VisCollisionDynamic = 19 // dynamic pruning
	// 20 = eDEPRECATED_COLLISION_PAIRS
	VisJointLocalFrames = 21 // joint local axes
	VisJointLimits      = 22 // joint limits
)

// SetVisualizationParameter sets a PVD visualization parameter.
// Key params: VisJointLocalFrames and VisJointLimits control joint visualization size.
// Typical value: 2.0-5.0 to make joints clearly visible.
func (s *SceneHandle) SetVisualizationParameter(paramID int, value float32) error {
	return errOrNil(int(C.physx_scene_set_vis_param(s.h, C.int(paramID), C.float(value))))
}

// EnableCCD enables/disables Continuous Collision Detection at the scene level.
func (s *SceneHandle) EnableCCD(enabled bool, maxPasses int) error {
	en := boolToInt(enabled)
	return errOrNil(int(C.physx_scene_enable_ccd(s.h, C.int(en), C.int(maxPasses))))
}

func boolToInt(b bool) int {
	if b {
		return 1
	}
	return 0
}
