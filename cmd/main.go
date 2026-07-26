package main

import (
	"fmt"
	"os"
	"os/exec"
	"strings"
	"time"

	"physx-go/physx"
)

// getWindowsHostIP returns the Windows host IP (WSL2 default gateway).
func getWindowsHostIP() string {
	out, err := exec.Command("sh", "-c",
		"ip route show default 2>/dev/null | awk '{print $3}'").Output()
	if err == nil {
		ip := strings.TrimSpace(string(out))
		if ip != "" {
			return ip
		}
	}
	out, err = exec.Command("sh", "-c",
		"cat /etc/resolv.conf 2>/dev/null | grep nameserver | awk '{print $2}'").Output()
	if err == nil {
		ip := strings.TrimSpace(string(out))
		if ip != "" {
			return ip
		}
	}
	return "127.0.0.1"
}

func main() {
	if len(os.Args) < 2 {
		fmt.Println("Usage: physx-demo <example>")
		fmt.Println("  physics:  sphere, boxes, joints, d6joint, kinematic")
		fmt.Println("  queries:  raycast, sweep")
		fmt.Println("  character: character")
		fmt.Println("  other:    trigger, cooking")
		fmt.Println("  contact:  contact, contact-ccd, contact-modify")
		fmt.Println()
		fmt.Println("Defaulting to 'sphere' demo...")
		runSphereDemo()
		return
	}

	switch os.Args[1] {
	case "sphere":
		runSphereDemo()
	case "boxes":
		runBoxesDemo()
	case "joints":
		runJointsDemo()
	case "raycast":
		runRaycastDemo()
	case "trigger":
		runTriggerDemo()
	case "sweep":
		runSweepDemo()
	case "kinematic":
		runKinematicDemo()
	case "d6joint":
		runD6JointDemo()
	case "character":
		runCharacterDemo()
		case "cooking":
			runCookingDemo()
		case "contact":
			runContactDemo()
		case "contact-ccd":
			runContactCCDDemo()
		case "contact-modify":
			runContactModifyDemo()
		default:
		fmt.Printf("Unknown example: %s\n", os.Args[1])
		os.Exit(1)
	}
}

// ──────────────────────────────────────────────────────────────────────────────
// Demo 1: Falling sphere (original demo, rewritten with new API)
// ──────────────────────────────────────────────────────────────────────────────

func runSphereDemo() {
	fmt.Println("=== Demo 1: Falling Sphere ===")
	fmt.Println()

	hostIP := getWindowsHostIP()
	fmt.Printf("PVD host: %s (start PVD on Windows first!)\n", hostIP)

	// 1. Create foundation
	foundation := physx.CreateFoundation()
	if foundation == nil {
		fmt.Println("ERROR: Failed to create PhysX foundation!")
		return
	}
	defer foundation.Release()

	// 2. Create physics (with PVD)
	physics := physx.CreatePhysics(foundation, hostIP)
	if physics == nil {
		fmt.Println("ERROR: Failed to create PhysX physics!")
		return
	}
	defer physics.Release()

	// 3. Create scene
	scene := physx.CreateScene(physics, 2, 0, -9.81, 0)
	if scene == nil {
		fmt.Println("ERROR: Failed to create PhysX scene!")
		return
	}
	defer scene.Release()

	// Enable PVD flags
	scene.SetPVDFlags(true, true, true)

	// 4. Create material
	material := physx.CreateMaterial(physics, 0.5, 0.5, 0.6)
	if material == nil {
		fmt.Println("ERROR: Failed to create material!")
		return
	}
	defer material.Release()

	// 5. Ground plane
	ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, material)
	if ground == nil {
		fmt.Println("ERROR: Failed to create ground!")
		return
	}
	defer ground.Release()
	scene.AddActor(ground)

	// 6. Falling sphere
	sphere := physx.CreateDynamicSphere(physics, 0, 20, 0, 1.0, material, 10.0)
	if sphere == nil {
		fmt.Println("ERROR: Failed to create sphere!")
		return
	}
	defer sphere.Release()
	sphere.SetLinearVelocity(5, 0, 0)
	scene.AddActor(sphere)

	fmt.Println("Setup complete. Simulating 10 seconds...")
	dt := float32(1.0 / 60.0)

	for i := 0; i < 600; i++ {
		scene.Simulate(dt)

		if i%60 == 0 {
			px, py, pz, _, _, _, _ := sphere.GetGlobalPose()
			vx, vy, vz := sphere.GetLinearVelocity()
			fmt.Printf("  t=%.1fs  pos=(%.3f, %.3f, %.3f)  vel=(%.3f, %.3f, %.3f)\n",
				float32(i)/60.0, px, py, pz, vx, vy, vz)
		}
		time.Sleep(16 * time.Millisecond)
	}

	fmt.Println("Simulation complete!")
}

// ──────────────────────────────────────────────────────────────────────────────
// Demo 2: Box stack
// ──────────────────────────────────────────────────────────────────────────────

func runBoxesDemo() {
	fmt.Println("=== Demo 2: Box Stack ===")
	fmt.Println()

	hostIP := getWindowsHostIP()
	foundation := physx.CreateFoundation()
	if foundation == nil {
		fmt.Println("ERROR: foundation")
		return
	}
	defer foundation.Release()

	physics := physx.CreatePhysics(foundation, hostIP)
	if physics == nil {
		fmt.Println("ERROR: physics")
		return
	}
	defer physics.Release()

	scene := physx.CreateScene(physics, 4, 0, -9.81, 0)
	if scene == nil {
		fmt.Println("ERROR: scene")
		return
	}
	defer scene.Release()
	scene.SetPVDFlags(true, true, true)

	mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.3)
	if mat == nil {
		fmt.Println("ERROR: material")
		return
	}
	defer mat.Release()

	// Ground
	ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
	scene.AddActor(ground)

	// Stack boxes in a pyramid
	boxSize := float32(1.0)
	density := float32(10.0)
	rows := 5

	for row := 0; row < rows; row++ {
		for col := 0; col <= row; col++ {
			x := (float32(col) - float32(row)*0.5) * boxSize * 1.05
			y := float32(rows-row-1)*boxSize*1.05 + boxSize*0.5 + 0.05
			z := float32(0)

			box := physx.CreateDynamicBox(physics, x, y, z,
				boxSize*0.45, boxSize*0.45, boxSize*0.45, mat, density)
			if box != nil {
				scene.AddActor(box)
			}
		}
	}

	fmt.Printf("Created %d boxes. Simulating 5 seconds...\n", rows*(rows+1)/2)
	dt := float32(1.0 / 60.0)

	for i := 0; i < 300; i++ {
		scene.Simulate(dt)
		if i%30 == 0 {
			fmt.Printf("  t=%.1fs\n", float32(i)/60.0)
		}
		time.Sleep(16 * time.Millisecond)
	}
	fmt.Println("Simulation complete!")
}

// ──────────────────────────────────────────────────────────────────────────────
// Demo 3: Joints (pendulum chain)
// ──────────────────────────────────────────────────────────────────────────────

func runJointsDemo() {
	fmt.Println("=== Demo 3: Pendulum Chain ===")
	fmt.Println()

	hostIP := getWindowsHostIP()
	foundation := physx.CreateFoundation()
	if foundation == nil {
		fmt.Println("ERROR: foundation")
		return
	}
	defer foundation.Release()

	physics := physx.CreatePhysics(foundation, hostIP)
	if physics == nil {
		fmt.Println("ERROR: physics")
		return
	}
	defer physics.Release()

	scene := physx.CreateScene(physics, 4, 0, -9.81, 0)
	if scene == nil {
		fmt.Println("ERROR: scene")
		return
	}
	defer scene.Release()

	// PVD visualization — set before first simulate
	scene.SetPVDFlags(true, true, true)
	scene.SetVisualizationParameter(physx.VisScale, 1.0)
	scene.SetVisualizationParameter(physx.VisJointLocalFrames, 50.0)  // extra large to see
	scene.SetVisualizationParameter(physx.VisJointLimits, 50.0)

	mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.3)
	defer mat.Release()

	// Ground
	ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
	scene.AddActor(ground)

	// Create a pendulum chain: 4 boxes hanging from an anchor
	numLinks := 4
	linkHalfH := float32(0.75) // half-height of each link
	linkHalfW := float32(0.15) // half-width

	// Visible anchor block at the top — static, no gravity
	anchor := physx.CreateRigidStatic(physics, 0, 8, 0, 0, 0, 0, 1)
	if anchor == nil {
		fmt.Println("ERROR: failed to create anchor")
		return
	}
	anchorShape := physx.CreateBoxShape(physics, 0.3, 0.3, 0.3, mat, true)
	if anchorShape != nil {
		anchor.AttachShape(anchorShape)
	}
	anchor.SetActorFlags(physx.ActorFlagVisualization)
	scene.AddActor(anchor)

	links := make([]*physx.ActorHandle, 0, numLinks)
	joints := make([]*physx.JointHandle, 0, numLinks)

	prevActor := anchor
	for i := 0; i < numLinks; i++ {
		// Position each link below the previous
		y := float32(8) - float32(i+1)*linkHalfH*2.1
		box := physx.CreateDynamicBox(physics, 0, y, 0, linkHalfH, linkHalfW, linkHalfW, mat, 10.0)
		if box == nil {
			continue
		}
		box.SetLinearDamping(0.1)
		box.SetAngularDamping(0.1)
		box.SetActorFlags(physx.ActorFlagVisualization) // required for vis to work
		scene.AddActor(box)
		links = append(links, box)

		// Revolute joint: pivot at the top of this link, bottom of previous
		jointLocal0 := physx.NewTransform(0, -linkHalfH, 0, 0, 0, 0, 1) // bottom of upper body
		jointLocal1 := physx.NewTransform(0, linkHalfH, 0, 0, 0, 0, 1)  // top of lower body
		joint := physx.CreateRevoluteJoint(physics, prevActor, jointLocal0, box, jointLocal1)
		if joint != nil {
			joint.SetRevoluteLimit(-2.5, 2.5, 50, 5)
			joint.SetConstraintFlag(physx.JointFlagVisualization, true)
			joints = append(joints, joint)
			fmt.Printf("  Link %d at y=%.2f\n", i+1, y)
		}
		prevActor = box
	}

	// Give the bottom link a horizontal kick to start swinging
	if len(links) > 0 {
		last := links[len(links)-1]
		last.AddForce(500, 0, 0, physx.ForceModeImpulse, true)
		fmt.Println("  Gave bottom link a 500N impulse to start swinging!")
	}

	fmt.Println("Simulating 10 seconds — watch PVD for the swinging chain...")
	dt := float32(1.0 / 60.0)

	for i := 0; i < 600; i++ {
		scene.Simulate(dt)
		if i%30 == 0 {
			// Print angles of all revolute joints
			angles := ""
			for _, j := range joints {
				angles += fmt.Sprintf("%+5.1f° ", j.GetRevoluteAngle()*57.3)
			}
			if len(links) > 0 {
				_, py, _, _, _, _, _ := links[len(links)-1].GetGlobalPose()
				fmt.Printf("  t=%.1fs  joint_angles=[%s]  tip_y=%.2f\n", float32(i)/60.0, angles, py)
			}
		}
		time.Sleep(16 * time.Millisecond)
	}
	fmt.Println("Simulation complete!")
}

// ──────────────────────────────────────────────────────────────────────────────
// Demo 4: Raycast
// ──────────────────────────────────────────────────────────────────────────────

func runRaycastDemo() {
	fmt.Println("=== Demo 4: Raycast Test ===")
	fmt.Println()

	foundation := physx.CreateFoundation()
	if foundation == nil {
		fmt.Println("ERROR: foundation")
		return
	}
	defer foundation.Release()

	physics := physx.CreatePhysics(foundation, "")
	if physics == nil {
		fmt.Println("ERROR: physics")
		return
	}
	defer physics.Release()

	scene := physx.CreateScene(physics, 2, 0, -9.81, 0)
	if scene == nil {
		fmt.Println("ERROR: scene")
		return
	}
	defer scene.Release()

	mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.3)
	defer mat.Release()

	// Ground plane
	ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
	scene.AddActor(ground)

	// Some boxes to raycast against
	for i := 0; i < 3; i++ {
		box := physx.CreateDynamicBox(physics, float32(i*3-3), 2, 0, 1, 1, 1, mat, 1.0)
		if box != nil {
			scene.AddActor(box)
			box.PutToSleep() // prevent them from falling
		}
	}

	// Step once to settle
	scene.Simulate(1.0 / 60.0)

	// Raycast downward from above
	fmt.Println("Raycasting downward from (0, 10, 0)...")
	origin := physx.NewVec3(0, 10, 0)
	direction := physx.NewVec3(0, -1, 0)
	hits := scene.Raycast(origin, direction, 100,
		physx.HitFlagPosition|physx.HitFlagNormal|physx.HitFlagDistance,
		physx.QueryFlagStatic|physx.QueryFlagDynamic, nil, 16)

	fmt.Printf("  Got %d hits:\n", len(hits))
	for i, hit := range hits {
		fmt.Printf("  Hit %d: pos=(%.3f, %.3f, %.3f) dist=%.3f face=%d\n",
			i, hit.Position.X, hit.Position.Y, hit.Position.Z, hit.Distance, hit.FaceIndex)
	}
	fmt.Println("Raycast demo complete!")
}

// ──────────────────────────────────────────────────────────────────────────────
// Demo 5: Trigger
// ──────────────────────────────────────────────────────────────────────────────

func runTriggerDemo() {
	fmt.Println("=== Demo 5: Trigger Test ===")
	fmt.Println()

	foundation := physx.CreateFoundation()
	if foundation == nil {
		fmt.Println("ERROR: foundation")
		return
	}
	defer foundation.Release()

	physics := physx.CreatePhysics(foundation, "")
	if physics == nil {
		fmt.Println("ERROR: physics")
		return
	}
	defer physics.Release()

	scene := physx.CreateScene(physics, 2, 0, -9.81, 0)
	if scene == nil {
		fmt.Println("ERROR: scene")
		return
	}
	defer scene.Release()

	mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.3)
	defer mat.Release()

	// Ground
	ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
	scene.AddActor(ground)

	// Trigger zone (static box set as trigger)
	triggerActor := physx.CreateRigidStatic(physics, 0, 2, 0, 0, 0, 0, 1)
	triggerShape := physx.CreateBoxShape(physics, 2, 2, 2, mat, true)
	triggerShape.SetAsTrigger(true)
	triggerActor.AttachShape(triggerShape)
	scene.AddActor(triggerActor)
	fmt.Println("Created trigger zone at (0, 2, 0) size=(4,4,4)")

	// Falling sphere that will pass through the trigger
	sphere := physx.CreateDynamicSphere(physics, 0, 8, 0, 0.5, mat, 1.0)
	scene.AddActor(sphere)
	fmt.Println("Created sphere at (0, 8, 0) - will fall through trigger zone")

	fmt.Println("Simulating 3 seconds...")
	dt := float32(1.0 / 60.0)

	for i := 0; i < 180; i++ {
		scene.Simulate(dt)
		if i%30 == 0 {
			_, py, _, _, _, _, _ := sphere.GetGlobalPose()
			fmt.Printf("  t=%.1fs sphere Y=%.3f\n", float32(i)/60.0, py)
		}
		time.Sleep(16 * time.Millisecond)
	}
	fmt.Println("Trigger demo complete!")
}

// ──────────────────────────────────────────────────────────────────────────────
// Demo 6: Sweep query
// ──────────────────────────────────────────────────────────────────────────────

func runSweepDemo() {
	fmt.Println("=== Demo 6: Sweep Query ===")
	fmt.Println()

	foundation := physx.CreateFoundation()
	if foundation == nil {
		fmt.Println("ERROR: foundation")
		return
	}
	defer foundation.Release()

	physics := physx.CreatePhysics(foundation, "")
	if physics == nil {
		fmt.Println("ERROR: physics")
		return
	}
	defer physics.Release()

	scene := physx.CreateScene(physics, 2, 0, -9.81, 0)
	defer scene.Release()

	mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.3)
	defer mat.Release()

	// Ground plane
	ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
	scene.AddActor(ground)

	// Place some static boxes as obstacles
	for i := 0; i < 5; i++ {
		x := float32(i*3 - 6)
		box := physx.CreateDynamicBox(physics, x, 1, 0, 0.5, 1.5, 0.5, mat, 1.0)
		if box != nil {
			box.PutToSleep()
			scene.AddActor(box)
		}
	}
	scene.Simulate(1.0 / 60.0)

	// Sweep a sphere through the scene
	geom := physx.SphereGeometry{Radius: 0.5}
	pose := physx.NewTransform(-8, 0.5, 0, 0, 0, 0, 1)
	dir := physx.NewVec3(1, 0, 0)

	fmt.Println("Sweeping sphere from x=-8 to x=+8...")
	hits := scene.Sweep(geom, physx.GeomSphere, pose, dir, 20,
		physx.HitFlagPosition|physx.HitFlagNormal|physx.HitFlagDistance,
		physx.QueryFlagStatic|physx.QueryFlagDynamic, nil, 16)

	fmt.Printf("  Got %d hits:\n", len(hits))
	for i, hit := range hits {
		fmt.Printf("  Hit %d: pos=(%.2f, %.2f, %.2f) dist=%.2f\n",
			i, hit.Position.X, hit.Position.Y, hit.Position.Z, hit.Distance)
	}
	fmt.Println("Sweep demo complete!")
}

// ──────────────────────────────────────────────────────────────────────────────
// Demo 7: Kinematic moving platform
// ──────────────────────────────────────────────────────────────────────────────

func runKinematicDemo() {
	fmt.Println("=== Demo 7: Kinematic Platform ===")
	fmt.Println()

	hostIP := getWindowsHostIP()
	foundation := physx.CreateFoundation()
	if foundation == nil {
		fmt.Println("ERROR: foundation")
		return
	}
	defer foundation.Release()

	physics := physx.CreatePhysics(foundation, hostIP)
	if physics == nil {
		fmt.Println("ERROR: physics")
		return
	}
	defer physics.Release()

	scene := physx.CreateScene(physics, 4, 0, -9.81, 0)
	defer scene.Release()
	scene.SetPVDFlags(true, true, true)

	mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.3)
	defer mat.Release()

	// Ground
	ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
	scene.AddActor(ground)

	// Kinematic platform (moves up and down)
	platform := physx.CreateDynamicBox(physics, 0, 3, 0, 2, 0.2, 1, mat, 0)
	if platform == nil {
		fmt.Println("ERROR: platform")
		return
	}
	platform.SetRigidBodyFlags(physx.RigidBodyFlagKinematic)
	scene.AddActor(platform)

	// Spheres on the platform
	for i := 0; i < 4; i++ {
		x := float32(i)*0.8 - 1.2
		sphere := physx.CreateDynamicSphere(physics, x, 5, 0, 0.3, mat, 5.0)
		if sphere != nil {
			scene.AddActor(sphere)
		}
	}

	fmt.Println("Platform moving up/down, spheres riding on it...")
	dt := float32(1.0 / 60.0)

	for i := 0; i < 600; i++ {
		// Simple up-down cycle every 2 seconds
		cycle := float32(i%120) / 60.0 // 0→2
		if cycle > 1.0 {
			cycle = 2.0 - cycle
		}
		platformY := 1.0 + cycle*4.0

		platform.SetKinematicTarget(0, platformY, 0, 0, 0, 0, 1)

		scene.Simulate(dt)
		if i%60 == 0 {
			_, py2, _, _, _, _, _ := platform.GetGlobalPose()
			fmt.Printf("  t=%.1fs  platform Y=%.2f\n", float32(i)/60.0, py2)
		}
		time.Sleep(16 * time.Millisecond)
	}
	fmt.Println("Kinematic demo complete!")
}

// ──────────────────────────────────────────────────────────────────────────────
// Demo 8: D6 Joint (configurable 6-DOF constraint)
// ──────────────────────────────────────────────────────────────────────────────

func runD6JointDemo() {
	fmt.Println("=== Demo 8: D6 Joint ===")
	fmt.Println()

	hostIP := getWindowsHostIP()
	foundation := physx.CreateFoundation()
	if foundation == nil {
		fmt.Println("ERROR: foundation")
		return
	}
	defer foundation.Release()

	physics := physx.CreatePhysics(foundation, hostIP)
	if physics == nil {
		fmt.Println("ERROR: physics")
		return
	}
	defer physics.Release()

	scene := physx.CreateScene(physics, 4, 0, -9.81, 0)
	defer scene.Release()
	scene.SetPVDFlags(true, true, true)

	// D6 joint visualization
	scene.SetVisualizationParameter(physx.VisScale, 1.0)
	scene.SetVisualizationParameter(physx.VisJointLocalFrames, 2.0)
	scene.SetVisualizationParameter(physx.VisJointLimits, 2.0)

	mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.3)
	defer mat.Release()

	// Ground
	ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
	scene.AddActor(ground)

	// Anchor (static)
	anchor := physx.CreateRigidStatic(physics, 0, 5, 0, 0, 0, 0, 1)
	anchorShape := physx.CreateBoxShape(physics, 0.3, 0.3, 0.3, mat, true)
	anchor.AttachShape(anchorShape)
	scene.AddActor(anchor)

	// Free body attached via D6 joint
	body := physx.CreateDynamicBox(physics, 0, 3, 0, 0.5, 0.3, 0.5, mat, 5.0)
	scene.AddActor(body)
	body.SetLinearDamping(0.5)
	body.SetAngularDamping(0.5)

	// D6 joint: lock all axes except twist (Y rotation) and Y translation
	anchorFrame := physx.NewTransform(0, -0.5, 0, 0, 0, 0, 1)
	bodyFrame := physx.NewTransform(0, 0.5, 0, 0, 0, 0, 1)
	d6 := physx.CreateD6Joint(physics, anchor, anchorFrame, body, bodyFrame)
	if d6 == nil {
		fmt.Println("ERROR: D6 joint creation failed")
		return
	}
	defer d6.Release()

	// Configure D6 motions:
	d6.SetD6Motion(physx.D6AxisX, physx.D6MotionLocked)     // X locked
	d6.SetD6Motion(physx.D6AxisY, physx.D6MotionFree)       // Y free (slide up/down)
	d6.SetD6Motion(physx.D6AxisZ, physx.D6MotionLocked)     // Z locked
	d6.SetD6Motion(physx.D6AxisTwist, physx.D6MotionFree)    // twist free (rotate around Y)
	d6.SetD6Motion(physx.D6AxisSwing1, physx.D6MotionLocked) // swing1 locked
	d6.SetD6Motion(physx.D6AxisSwing2, physx.D6MotionLocked) // swing2 locked

	d6.SetConstraintFlag(physx.JointFlagVisualization, true)

	// Drive: spin the body around Y axis
	drive := physx.D6JointDrive{Stiffness: 100, Damping: 10, ForceLimit: 100}
	d6.SetD6Drive(physx.D6DriveTwist, drive)
	d6.SetD6DrivePosition(0, 0, 0, 0, 0.707, 0, 0.707) // 90° rotation target

	// Give the body a push
	body.AddForce(0, 200, 50, physx.ForceModeImpulse, true)

	fmt.Println("D6 joint: Y-slide + Y-twist only. Body should slide and spin...")
	dt := float32(1.0 / 60.0)

	for i := 0; i < 600; i++ {
		scene.Simulate(dt)
		if i%60 == 0 {
			px, py, pz, qx, qy, qz, qw := body.GetGlobalPose()
			fmt.Printf("  t=%.1fs  pos=(%.2f,%.2f,%.2f) rot=(%.2f,%.2f,%.2f,%.2f)\n",
				float32(i)/60.0, px, py, pz, qx, qy, qz, qw)
		}
		time.Sleep(16 * time.Millisecond)
	}
	fmt.Println("D6 joint demo complete!")
}

// ──────────────────────────────────────────────────────────────────────────────
// Demo 9: Character Controller
// ──────────────────────────────────────────────────────────────────────────────

func runCharacterDemo() {
	fmt.Println("=== Demo 9: Character Controller ===")
	fmt.Println()

	hostIP := getWindowsHostIP()
	foundation := physx.CreateFoundation()
	if foundation == nil {
		fmt.Println("ERROR: foundation")
		return
	}
	defer foundation.Release()

	physics := physx.CreatePhysics(foundation, hostIP)
	if physics == nil {
		fmt.Println("ERROR: physics")
		return
	}
	defer physics.Release()

	scene := physx.CreateScene(physics, 4, 0, -9.81, 0)
	defer scene.Release()
	scene.SetPVDFlags(true, true, true)

	mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.3)
	defer mat.Release()

	// Ground
	ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
	scene.AddActor(ground)

	// Some obstacles to walk around/over
	for i := 0; i < 5; i++ {
		x := float32(i*3 - 6)
		box := physx.CreateDynamicBox(physics, x, 0.5, 0, 0.5, 1.0, 0.5, mat, 0)
		if box != nil {
			box.SetRigidBodyFlags(physx.RigidBodyFlagKinematic)
			scene.AddActor(box)
		}
	}

	// Ramp
	ramp := physx.CreateDynamicBox(physics, 4, 0.3, 0, 0.5, 0.3, 1.0, mat, 0)
	if ramp != nil {
		ramp.SetGlobalPose(4, 0.3, 0, 0, 0.2588, 0, 0.9659, true) // rotated 30°
		ramp.SetRigidBodyFlags(physx.RigidBodyFlagKinematic)
		scene.AddActor(ramp)
	}

	// Character controller
	mgr := physx.CreateControllerManager(scene)
	if mgr == nil {
		fmt.Println("ERROR: controller manager")
		return
	}
	defer mgr.Release()

	ctrl := mgr.CreateCapsuleController(physics, 0.4, 1.6, 0, 2, 0, mat)
	if ctrl == nil {
		fmt.Println("ERROR: controller")
		return
	}
	defer ctrl.Release()
	ctrl.SetStepOffset(0.5)
	ctrl.SetSlopeLimit(0.7)

	fmt.Println("Character walking forward with gravity...")
	dt := float32(1.0 / 60.0)

	for i := 0; i < 600; i++ {
		// Move forward, gravity pulls down
		dx := float32(2.0) // walk speed
		dy := float32(0.0)
		ctrl.Move(dx*dt, dy, 0, 0.001, dt)

		scene.Simulate(dt)
		if i%30 == 0 {
			x, y, z := ctrl.GetPosition()
			fmt.Printf("  t=%.1fs  ctrl=(%.2f, %.2f, %.2f)\n", float32(i)/60.0, x, y, z)
		}
		time.Sleep(16 * time.Millisecond)
	}
	fmt.Println("Character demo complete!")
}

// ──────────────────────────────────────────────────────────────────────────────
// Demo 10: Mesh Cooking
// ──────────────────────────────────────────────────────────────────────────────

func runCookingDemo() {
	fmt.Println("=== Demo 10: Mesh Cooking ===")
	fmt.Println()

	foundation := physx.CreateFoundation()
	if foundation == nil {
		fmt.Println("ERROR: foundation")
		return
	}
	defer foundation.Release()

	physics := physx.CreatePhysics(foundation, "")
	if physics == nil {
		fmt.Println("ERROR: physics")
		return
	}
	defer physics.Release()

	// Create cooking instance
	cooking := physx.CreateCooking()
	if cooking == nil {
		fmt.Println("ERROR: cooking")
		return
	}
	defer cooking.Release()

	// Cook a convex mesh (tetrahedron — 4 vertices)
	tetraVerts := []physx.Vec3{
		physx.NewVec3(0, 1, 0),
		physx.NewVec3(1, -0.5, 0.87),
		physx.NewVec3(-1, -0.5, 0.87),
		physx.NewVec3(0, -0.5, -0.87),
	}
	convex, errCode := cooking.CookConvexMesh(tetraVerts)
	if convex == nil {
		fmt.Printf("ERROR: cook convex failed, code=%d\n", errCode)
		return
	}
	defer convex.Release()
	fmt.Printf("Cooked convex mesh: tetrahedron (4 verts) OK\n")

	// Cook a triangle mesh (simple pyramid — 5 verts, 6 tris)
	pyramidVerts := []physx.Vec3{
		physx.NewVec3(0, 1, 0),    // apex
		physx.NewVec3(-1, 0, -1),  // base corners
		physx.NewVec3(1, 0, -1),
		physx.NewVec3(1, 0, 1),
		physx.NewVec3(-1, 0, 1),
	}
	pyramidIndices := []uint32{
		0, 1, 2, // side 1
		0, 2, 3, // side 2
		0, 3, 4, // side 3
		0, 4, 1, // side 4
		1, 3, 2, // base tri 1
		1, 4, 3, // base tri 2
	}
	triMesh, errCode := cooking.CookTriangleMesh(pyramidVerts, pyramidIndices)
	if triMesh == nil {
		fmt.Printf("ERROR: cook tri mesh failed, code=%d\n", errCode)
	} else {
		defer triMesh.Release()
		fmt.Printf("Cooked triangle mesh: pyramid (5 verts, 6 tris) OK\n")
	}

	// Create shapes from cooked meshes
	scene := physx.CreateScene(physics, 2, 0, -9.81, 0)
	defer scene.Release()

	mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.6)
	defer mat.Release()

	// Ground
	ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
	scene.AddActor(ground)

	// Drop a tetrahedron
	convexShape := physx.CreateConvexMeshShape(physics, convex, mat, true)
	if convexShape != nil {
		body := physx.CreateRigidDynamic(physics, 0, 5, 0, 0, 0, 0, 1)
		body.AttachShape(convexShape)
		body.SetMass(5)
		scene.AddActor(body)
		fmt.Println("Dropping tetrahedron from y=5...")
	}

	if triMesh != nil {
		triShape := physx.CreateTriangleMeshShape(physics, triMesh, mat, true)
		if triShape != nil {
			body := physx.CreateRigidStatic(physics, 3, 1, 0, 0, 0, 0, 1)
			body.AttachShape(triShape)
			scene.AddActor(body)
			fmt.Println("Placed pyramid mesh at (3,1,0) as static")
		}
	}

	fmt.Println("Simulating 3 seconds...")
	dt := float32(1.0 / 60.0)
	for i := 0; i < 180; i++ {
		scene.Simulate(dt)
		time.Sleep(16 * time.Millisecond)
	}
		fmt.Println("Cooking demo complete!")
	}

// ──────────────────────────────────────────────────────────────────────────────
// Demo 11: Contact Report (mirrors SnippetContactReport)
// ──────────────────────────────────────────────────────────────────────────────

func runContactDemo() {
	fmt.Println("=== Demo 11: Contact Report ===")
	fmt.Println("    Mirrors PhysX SnippetContactReport")
	fmt.Println()

	foundation := physx.CreateFoundation()
	if foundation == nil {
		fmt.Println("ERROR: foundation"); return
	}
	defer foundation.Release()

	physics := physx.CreatePhysics(foundation, "")
	if physics == nil {
		fmt.Println("ERROR: physics"); return
	}
	defer physics.Release()

	scene := physx.CreateScene(physics, 2, 0, -9.81, 0)
	defer scene.Release()

	mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.6)
	defer mat.Release()

	// ── Ground plane ──────────────────────────────────────────────────────────
	ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
	scene.AddActor(ground)

	// ── Filter shader: enable contact detection + notification ────────────────
	scene.SetFilterShader(func(attr0 uint32, fd0 *physx.FilterData, attr1 uint32, fd1 *physx.FilterData) (uint32, uint32) {
		pairFlags := uint32(physx.PairFlagSolveContact |
			physx.PairFlagDetectDiscreteContact |
			physx.PairFlagNotifyTouchFound |
			physx.PairFlagNotifyTouchPersists |
			physx.PairFlagNotifyContactPoints)
		return pairFlags, physx.FilterFlagDefault
	})

	contactCount := 0
	// ── Contact callback: print contact events ────────────────────────────────
	scene.SetContactCallback(func(header physx.ContactPairHeader, pairs []physx.ContactPair) {
		for i, cp := range pairs {
			if cp.Events&physx.PairFlagNotifyTouchFound != 0 {
				fmt.Printf("  CONTACT FOUND: pair[%d] actors=(%p,%p) pos=(%.2f,%.2f,%.2f) impulse=%.3f\n",
					i, header.Actor0, header.Actor1,
					cp.ContactPoint.X, cp.ContactPoint.Y, cp.ContactPoint.Z,
					cp.Impulse.X)
				contactCount++
			}
		}
	})

	// ── Create dynamic objects ───────────────────────────────────────────────
	// A stack of boxes that will fall and collide
	for i := 0; i < 3; i++ {
		y := float32(3 + i*2)
		box := physx.CreateDynamicBox(physics, float32(i)*0.5-0.5, y, 0.0,
			0.5, 0.5, 0.5, mat, 10.0)
		if box != nil {
			box.UpdateMassAndInertia(10.0)
			scene.AddActor(box)
		}
	}

	// A sphere that will bounce
	bouncyMat2 := physx.CreateMaterial(physics, 0.3, 0.3, 0.8)
	sphere := physx.CreateDynamicSphere(physics, 2, 6, 0, 0.5, bouncyMat2, 5.0)
	if sphere != nil {
		sphere.UpdateMassAndInertia(5.0)
		scene.AddActor(sphere)
	}

	// Bouncy sphere with separate material
	bouncyMat := physx.CreateMaterial(physics, 0.3, 0.3, 0.9)
	bouncySphere := physx.CreateDynamicSphere(physics, -2, 8, 0, 0.6, bouncyMat, 3.0)
	if bouncySphere != nil {
		bouncySphere.UpdateMassAndInertia(3.0)
		scene.AddActor(bouncySphere)
	}

	fmt.Println("Created boxes + spheres. Simulating 3 seconds...")
	fmt.Println("(Contact events will print as objects collide)")
	dt := float32(1.0 / 60.0)

	for i := 0; i < 180; i++ {
		scene.Simulate(dt)
		if i%60 == 0 {
			fmt.Printf("  t=%.1fs\n", float32(i)/60.0)
		}
		time.Sleep(16 * time.Millisecond)
	}
	fmt.Printf("Simulation complete! %d contact-found events received.\n", contactCount)
}

// ──────────────────────────────────────────────────────────────────────────────
// Demo 12: Contact Report CCD (mirrors SnippetContactReportCCD)
// ──────────────────────────────────────────────────────────────────────────────

func runContactCCDDemo() {
	fmt.Println("=== Demo 12: Contact Report CCD ===")
	fmt.Println("    Mirrors PhysX SnippetContactReportCCD")
	fmt.Println()

	foundation := physx.CreateFoundation()
	if foundation == nil {
		fmt.Println("ERROR: foundation"); return
	}
	defer foundation.Release()

	physics := physx.CreatePhysics(foundation, "")
	if physics == nil {
		fmt.Println("ERROR: physics"); return
	}
	defer physics.Release()

	scene := physx.CreateScene(physics, 2, 0, -9.81, 0)
	defer scene.Release()

	// Enable CCD at scene level
	scene.EnableCCD(true, 1)

	mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.3)
	defer mat.Release()

	// ── Ground ────────────────────────────────────────────────────────────────
	ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
	scene.AddActor(ground)

	// ── Thin wall for the bullet to hit ──────────────────────────────────────
	wall := physx.CreateRigidStatic(physics, 10, 1, 0, 0, 0, 0, 1)
	wallShape := physx.CreateBoxShape(physics, 0.1, 2, 2, mat, true)
	wall.AttachShape(wallShape)
	scene.AddActor(wall)
	fmt.Println("Created thin wall at x=10 (target)")

	// ── Filter shader with CCD flags ─────────────────────────────────────────
	contactHappened := false
	scene.SetFilterShader(func(attr0 uint32, fd0 *physx.FilterData, attr1 uint32, fd1 *physx.FilterData) (uint32, uint32) {
		pairFlags := uint32(physx.PairFlagSolveContact |
			physx.PairFlagDetectDiscreteContact |
			physx.PairFlagDetectCCDContact |
			physx.PairFlagNotifyTouchFound |
			physx.PairFlagNotifyTouchCCD)
		return pairFlags, physx.FilterFlagDefault
	})

	scene.SetContactCallback(func(header physx.ContactPairHeader, pairs []physx.ContactPair) {
		for _, cp := range pairs {
			if cp.Events&physx.PairFlagNotifyTouchCCD != 0 {
				fmt.Printf("  CCD CONTACT! pos=(%.2f,%.2f,%.2f) dist=%.3f\n",
					cp.ContactPoint.X, cp.ContactPoint.Y, cp.ContactPoint.Z,
					cp.ContactDistance)
				contactHappened = true
			} else if cp.Events&physx.PairFlagNotifyTouchFound != 0 {
				fmt.Printf("  Contact found at (%.2f,%.2f,%.2f)\n",
					cp.ContactPoint.X, cp.ContactPoint.Y, cp.ContactPoint.Z)
				contactHappened = true
			}
		}
	})

	// ── Fast-moving bullet (CCD-enabled) ─────────────────────────────────────
	bullet := physx.CreateDynamicSphere(physics, -5, 1, 0, 0.2, mat, 1.0)
	if bullet != nil {
		bullet.SetRigidBodyFlags(physx.RigidBodyFlagEnableCCD)
		bullet.SetLinearVelocity(100, 0, 0) // Very fast!
		scene.AddActor(bullet)
		fmt.Println("Fired bullet from x=-5 with vx=100 (CCD enabled)")
	}

	fmt.Println("Simulating 1 second...")
	dt := float32(1.0 / 60.0)

	for i := 0; i < 60; i++ {
		scene.Simulate(dt)
		if i%10 == 0 {
			px, _, _, _, _, _, _ := bullet.GetGlobalPose()
			fmt.Printf("  t=%.2fs bullet x=%.2f\n", float32(i)/60.0, px)
		}
		time.Sleep(16 * time.Millisecond)
	}
	if contactHappened {
		fmt.Println("CCD contact detected successfully!")
	} else {
		fmt.Println("WARNING: No CCD contact detected (bullet may have tunneled)")
	}
	fmt.Println("CCD demo complete!")
}

// ──────────────────────────────────────────────────────────────────────────────
// Demo 13: Contact Modification (mirrors SnippetContactModification)
// ──────────────────────────────────────────────────────────────────────────────

func runContactModifyDemo() {
	fmt.Println("=== Demo 13: Contact Modification ===")
	fmt.Println("    Mirrors PhysX SnippetContactModification")
	fmt.Println()

	foundation := physx.CreateFoundation()
	if foundation == nil {
		fmt.Println("ERROR: foundation"); return
	}
	defer foundation.Release()

	physics := physx.CreatePhysics(foundation, "")
	if physics == nil {
		fmt.Println("ERROR: physics"); return
	}
	defer physics.Release()

	scene := physx.CreateScene(physics, 2, 0, -9.81, 0)
	defer scene.Release()

	mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.6)
	defer mat.Release()

	// ── Ground ────────────────────────────────────────────────────────────────
	ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
	scene.AddActor(ground)

	// ── Filter shader: enable contact modification ───────────────────────────
	modifyCount := 0
	scene.SetFilterShader(func(attr0 uint32, fd0 *physx.FilterData, attr1 uint32, fd1 *physx.FilterData) (uint32, uint32) {
		pairFlags := uint32(physx.PairFlagSolveContact |
			physx.PairFlagDetectDiscreteContact |
			physx.PairFlagModifyContacts |
			physx.PairFlagNotifyTouchFound)
		return pairFlags, physx.FilterFlagDefault
	})

	// ── Contact modify callback: scale down the response ─────────────────────
	scene.SetContactModifyCallback(func(pairs []physx.ContactModifyPair) {
		for i := range pairs {
			// Scale down the inverse mass on both bodies → softer contacts
			physx.SetContactModifyInvMassScale(i, 0, 0.5)
			physx.SetContactModifyInvMassScale(i, 1, 0.5)
			modifyCount++
		}
	})

	// Also set a contact callback to see the effect
	scene.SetContactCallback(func(header physx.ContactPairHeader, pairs []physx.ContactPair) {
		for _, cp := range pairs {
			if cp.Events&physx.PairFlagNotifyTouchFound != 0 {
				fmt.Printf("  Contact: impulse=(%.3f,%.3f) pos=(%.2f,%.2f,%.2f)\n",
					cp.Impulse.X, cp.Impulse.Y,
					cp.ContactPoint.X, cp.ContactPoint.Y, cp.ContactPoint.Z)
			}
		}
	})

	// ── Create dynamic objects ───────────────────────────────────────────────
	// Heavy box falling on a light box
	heavyBox := physx.CreateDynamicBox(physics, 0, 5, 0, 1, 1, 1, mat, 100.0)
	if heavyBox != nil {
		heavyBox.UpdateMassAndInertia(100.0)
		scene.AddActor(heavyBox)
		fmt.Println("Heavy box (100 kg) at y=5")
	}

	lightBox := physx.CreateDynamicBox(physics, 0, 1.5, 0, 1, 1, 1, mat, 1.0)
	if lightBox != nil {
		lightBox.UpdateMassAndInertia(1.0)
		scene.AddActor(lightBox)
		fmt.Println("Light box (1 kg) at y=1.5")
	}

	fmt.Println("Contact modification enabled (invMassScale=0.5 → softer contacts)")
	fmt.Println("Simulating 3 seconds...")
	dt := float32(1.0 / 60.0)

	for i := 0; i < 180; i++ {
		scene.Simulate(dt)
		if i%60 == 0 {
			py1, _, _, _, _, _, _ := heavyBox.GetGlobalPose()
			py2, _, _, _, _, _, _ := lightBox.GetGlobalPose()
			fmt.Printf("  t=%.1fs heavy_y=%.2f light_y=%.2f\n",
				float32(i)/60.0, py1, py2)
		}
		time.Sleep(16 * time.Millisecond)
	}
	fmt.Printf("Simulation complete! %d contact pairs modified.\n", modifyCount)
}
