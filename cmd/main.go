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
		fmt.Println("  examples: sphere, boxes, joints, raycast, trigger")
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
	scene.SetPVDFlags(true, true, true)

	mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.3)
	defer mat.Release()

	// Ground
	ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
	scene.AddActor(ground)

	// Create a chain of boxes connected by revolute joints
	numLinks := 5
	boxSize := float32(0.5)

	// Anchor
	anchor := physx.CreateRigidStatic(physics, 0, 8, 0, 0, 0, 0, 1)
	scene.AddActor(anchor)

	var prevActor *physx.ActorHandle = anchor
	_ = physx.NewTransform(0, -boxSize*0.6, 0, 0, 0, 0, 1)

	for i := 0; i < numLinks; i++ {
		y := float32(8) - float32(i+1)*boxSize*1.2
		box := physx.CreateDynamicBox(physics, 0, y, 0, boxSize, boxSize*0.2, boxSize*0.2, mat, 1.0)
		if box == nil {
			continue
		}
		scene.AddActor(box)

		// Revolute joint between previous and this box
		jointLocal0 := physx.NewTransform(0, -boxSize*0.6, 0, 0, 0, 0, 1)
		jointLocal1 := physx.NewTransform(0, boxSize*0.6, 0, 0, 0, 0, 1)
		joint := physx.CreateRevoluteJoint(physics, prevActor, jointLocal0, box, jointLocal1)
		if joint != nil {
			joint.SetRevoluteLimit(-3.14*0.5, 3.14*0.5, 100, 10)
			fmt.Printf("  Link %d created at y=%.2f\n", i+1, y)
		}
			prevActor = box
	}

	fmt.Println("Simulating 5 seconds with pendulum chain...")
	dt := float32(1.0 / 60.0)

	for i := 0; i < 300; i++ {
		scene.Simulate(dt)
		if i%60 == 0 {
			fmt.Printf("  t=%.1fs\n", float32(i)/60.0)
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
