package main

import (
	"physx-go/physx"
)

// ── Tracked actor: used to serialize geometry info for the web viewer ────────

type trackedActor struct {
	actor    *physx.ActorHandle
	geomType string  // "box", "sphere", "capsule"
	hx, hy, hz float32 // half-extents (box) or radius (sphere) — only hx used for sphere
}

type snippetState struct {
	foundation *physx.FoundationHandle
	physics    *physx.PhysicsHandle
	scene      *physx.SceneHandle
	material   *physx.MaterialHandle
	actors     []trackedActor
	joints     []*physx.JointHandle  // tracked for cleanup
	contacts   []contactEvent
	stackZ     float32
	time       float64
}

type contactEvent struct {
	pos    physx.Vec3
	normal physx.Vec3
}

// ──────────────────────────────────────────────────────────────────────────────
// Snippet 1: HelloWorld — 5 stacks of 55 cubes each (faithful to original)
// ──────────────────────────────────────────────────────────────────────────────

func createHelloWorld() *snippetState {
	s := &snippetState{}

	s.foundation = physx.CreateFoundation()
	s.physics = physx.CreatePhysics(s.foundation, "")
	s.scene = physx.CreateScene(s.physics, 2, 0, -9.81, 0)
	s.material = physx.CreateMaterial(s.physics, 0.5, 0.5, 0.6)

	// Ground plane
	ground := physx.CreateStaticPlane(s.physics, 0, 1, 0, 0, s.material)
	s.scene.AddActor(ground)
	s.actors = append(s.actors, trackedActor{ground, "plane", 0, 0, 0})

	// 5 stacks of 55 boxes each (pyramid: 10+9+8+...+1 = 55 per stack)
	// Matches original: size=10, halfExtent=2.0, stackZ decreases by 10 each
	s.stackZ = 10.0
	for st := 0; st < 5; st++ {
		createStack(s, physx.NewTransform(0, 0, s.stackZ, 0, 0, 0, 1), 10, 2.0)
		s.stackZ -= 10.0
	}

	// Cannonball (matches original: pos(0,40,100), vel(0,-50,-100), radius 10)
	ball := physx.CreateDynamicSphere(s.physics, 0, 40, 100, 10, s.material, 10.0)
	ball.SetLinearVelocity(0, -50, -100)
	ball.SetAngularDamping(0.5)
	s.scene.AddActor(ball)
	s.actors = append(s.actors, trackedActor{ball, "sphere", 10, 0, 0})

	// Enable contact reporting for contact visualization
	setupContactFilter(s)

	return s
}

func createStack(s *snippetState, baseTransform physx.Transform, size uint32, halfExtent float32) {
	shape := physx.CreateBoxShape(s.physics, halfExtent, halfExtent, halfExtent, s.material, false)
	defer shape.Release()

	for i := uint32(0); i < size; i++ {
		for j := uint32(0); j < size-i; j++ {
			x := (float32(j)*2 - float32(size-i)) * halfExtent
			y := (float32(i)*2 + 1) * halfExtent
			z := float32(0)

			worldPos := physx.NewTransform(
				baseTransform.P.X+x, baseTransform.P.Y+y, baseTransform.P.Z+z,
				0, 0, 0, 1,
			)

			body := physx.CreateRigidDynamic(s.physics,
				worldPos.P.X, worldPos.P.Y, worldPos.P.Z,
				worldPos.Q.X, worldPos.Q.Y, worldPos.Q.Z, worldPos.Q.W)
			body.AttachShape(shape)
			body.UpdateMassAndInertia(10.0)
			body.SetAngularDamping(0.5)
			s.scene.AddActor(body)
			s.actors = append(s.actors, trackedActor{body, "box", halfExtent, halfExtent, halfExtent})
		}
	}
}

// ──────────────────────────────────────────────────────────────────────────────
// Snippet 2: JointChains — 3 chains (spherical, breakable-fixed, damped-D6)
// ──────────────────────────────────────────────────────────────────────────────

func createJointChains() *snippetState {
	s := &snippetState{}

	s.foundation = physx.CreateFoundation()
	s.physics = physx.CreatePhysics(s.foundation, "")
	s.scene = physx.CreateScene(s.physics, 2, 0, -9.81, 0)
	s.material = physx.CreateMaterial(s.physics, 0.5, 0.5, 0.6)

	// Ground plane
	ground := physx.CreateStaticPlane(s.physics, 0, 1, 0, 0, s.material)
	s.scene.AddActor(ground)
	s.actors = append(s.actors, trackedActor{ground, "plane", 0, 0, 0})

	halfExtentX := float32(2.0) // box half-extent along X (plank length)
	halfExtentY := float32(0.5) // box half-extent along Y
	halfExtentZ := float32(0.5) // box half-extent along Z
	separation := float32(4.0)  // distance between links
	numLinks := uint32(5)

	// ── Chain 1: Limited Spherical ──────────────────────────────────────────
	makeChain(s, physx.NewTransform(0, 20, 0, 0, 0, 0, 1),
		numLinks, halfExtentX, halfExtentY, halfExtentZ, separation,
		"spherical")

	// ── Chain 2: Breakable Fixed ────────────────────────────────────────────
	makeChain(s, physx.NewTransform(0, 20, -10, 0, 0, 0, 1),
		numLinks, halfExtentX, halfExtentY, halfExtentZ, separation,
		"fixed")

	// ── Chain 3: Damped D6 ─────────────────────────────────────────────────
	makeChain(s, physx.NewTransform(0, 20, -20, 0, 0, 0, 1),
		numLinks, halfExtentX, halfExtentY, halfExtentZ, separation,
		"d6")

	return s
}

func makeChain(s *snippetState, root physx.Transform, length uint32,
	hx, hy, hz, sep float32, jointType string) {

	shape := physx.CreateBoxShape(s.physics, hx, hy, hz, s.material, false)
	defer shape.Release()

	offset := physx.NewTransform(sep/2, 0, 0, 0, 0, 0, 1)
	localTm := offset
	var prev *physx.ActorHandle

	for i := uint32(0); i < length; i++ {
		worldTm := physx.NewTransform(
			root.P.X+localTm.P.X, root.P.Y+localTm.P.Y, root.P.Z+localTm.P.Z,
			0, 0, 0, 1,
		)
		current := physx.CreateRigidDynamic(s.physics,
			worldTm.P.X, worldTm.P.Y, worldTm.P.Z,
			worldTm.Q.X, worldTm.Q.Y, worldTm.Q.Z, worldTm.Q.W)
		current.AttachShape(shape)
		current.SetAngularDamping(0.5)
		current.SetLinearDamping(0.1)
		s.scene.AddActor(current)
		s.actors = append(s.actors, trackedActor{current, "box", hx, hy, hz})

		if prev != nil {
			frame0 := physx.NewTransform(sep/2, 0, 0, 0, 0, 0, 1)
			frame1 := physx.NewTransform(-sep/2, 0, 0, 0, 0, 0, 1)
			var j *physx.JointHandle

			switch jointType {
			case "spherical":
				j = physx.CreateSphericalJoint(s.physics, prev, frame0, current, frame1)
				j.SetSphericalLimitCone(0.785, 0.785, 0, 0.05)
				j.SetConstraintFlag(physx.JointFlagVisualization, true)
			case "fixed":
				j = physx.CreateFixedJoint(s.physics, prev, frame0, current, frame1)
				j.SetBreakForce(1000, 100000)
				j.SetConstraintFlag(physx.JointFlagDriveLimitsAreForces, true)
				j.SetConstraintFlag(physx.JointFlagDisablePreprocessing, true)
				j.SetConstraintFlag(physx.JointFlagVisualization, true)
			case "d6":
				j = physx.CreateD6Joint(s.physics, prev, frame0, current, frame1)
				j.SetD6Motion(physx.D6AxisSwing1, physx.D6MotionFree)
				j.SetD6Motion(physx.D6AxisSwing2, physx.D6MotionFree)
				j.SetD6Motion(physx.D6AxisTwist, physx.D6MotionFree)
				drive := physx.D6JointDrive{Stiffness: 0, Damping: 1000, ForceLimit: 3.4e38, Flags: 1}
				j.SetD6Drive(physx.D6DriveSLERP, drive)
				j.SetConstraintFlag(physx.JointFlagVisualization, true)
			}
			if j != nil {
				s.joints = append(s.joints, j)
			}
			}
			prev = current
		localTm.P.X += sep
	}
}

// ──────────────────────────────────────────────────────────────────────────────
// Snippet 3: ContactReport — falling objects with contact visualization
// ──────────────────────────────────────────────────────────────────────────────

func createContactReport() *snippetState {
	s := &snippetState{}

	s.foundation = physx.CreateFoundation()
	s.physics = physx.CreatePhysics(s.foundation, "")
	s.scene = physx.CreateScene(s.physics, 2, 0, -9.81, 0)
	s.material = physx.CreateMaterial(s.physics, 0.5, 0.5, 0.6)

	// Ground plane
	ground := physx.CreateStaticPlane(s.physics, 0, 1, 0, 0, s.material)
	s.scene.AddActor(ground)
	s.actors = append(s.actors, trackedActor{ground, "plane", 0, 0, 0})

	// Falling spheres (various sizes and positions)
	spheres := []struct{ x, y, z, r, mass float32 }{
		{0, 8, 0, 1.5, 50},
		{2, 12, 2, 1.0, 20},
		{-2, 10, -1, 1.2, 30},
		{4, 15, -2, 0.8, 10},
		{-4, 9, 3, 1.8, 70},
		{1, 18, -3, 0.6, 5},
		{-1, 20, 1, 1.4, 40},
		{3, 14, 0, 0.9, 15},
	}
	for _, sp := range spheres {
		sphere := physx.CreateDynamicSphere(s.physics, sp.x, sp.y, sp.z, sp.r, s.material, 10.0)
		sphere.UpdateMassAndInertia(sp.mass)
		sphere.SetAngularDamping(0.5)
		sphere.SetLinearDamping(0.1)
		s.scene.AddActor(sphere)
		s.actors = append(s.actors, trackedActor{sphere, "sphere", sp.r, 0, 0})
	}

	// Some falling boxes
	for i := 0; i < 5; i++ {
		x := float32(i)*2 - 4
		box := physx.CreateDynamicBox(s.physics, x, float32(5+i*3), float32(i-2)*2,
			0.8, 1.0, 0.6, s.material, 10.0)
		box.UpdateMassAndInertia(20)
		box.SetAngularDamping(0.5)
		s.scene.AddActor(box)
		s.actors = append(s.actors, trackedActor{box, "box", 0.8, 1.0, 0.6})
	}

	// Enable contact reporting + collect contact events
	setupContactFilter(s)

	return s
}

// ──────────────────────────────────────────────────────────────────────────────
// Contact filter setup (enables contact notification and collects events)
// ──────────────────────────────────────────────────────────────────────────────

func setupContactFilter(s *snippetState) {
	s.scene.SetFilterShader(func(attr0 uint32, fd0 *physx.FilterData, attr1 uint32, fd1 *physx.FilterData) (uint32, uint32) {
		pf := uint32(physx.PairFlagSolveContact |
			physx.PairFlagDetectDiscreteContact |
			physx.PairFlagNotifyTouchFound |
			physx.PairFlagNotifyContactPoints)
		return pf, physx.FilterFlagDefault
	})

	s.scene.SetContactCallback(func(header physx.ContactPairHeader, pairs []physx.ContactPair) {
		for _, cp := range pairs {
			if cp.Events&physx.PairFlagNotifyTouchFound != 0 {
				s.contacts = append(s.contacts, contactEvent{
					pos:    cp.ContactPoint,
					normal: cp.ContactNormal,
				})
			}
		}
		// Keep only the latest 100 contacts
		if len(s.contacts) > 100 {
			s.contacts = s.contacts[len(s.contacts)-100:]
		}
	})
}

// ──────────────────────────────────────────────────────────────────────────────
// Cleanup
// ──────────────────────────────────────────────────────────────────────────────

func (s *snippetState) release() {
	// NOTE: clearActorHandles() MUST be called before release()
	// to prevent Go finalizers from double-freeing PhysX objects.
	if s.scene != nil {
		s.scene.Release()
	}
	if s.material != nil {
		s.material.Release()
	}
	if s.physics != nil {
		s.physics.Release()
	}
	if s.foundation != nil {
		s.foundation.Release()
	}
}
