package main

/*
#cgo CXXFLAGS: -std=c++11 -DNDEBUG -I/home/beijian/PhysX-3.4-master/PhysX_3.4/Include -I/home/beijian/PhysX-3.4-master/PxShared/include
#cgo LDFLAGS: -L/home/beijian/PhysX-3.4-master/PhysX_3.4/Bin/linux64 -L/home/beijian/PhysX-3.4-master/PxShared/bin/linux64 -L/home/beijian/PhysX-3.4-master/PhysX_3.4/Lib/linux64 -L/home/beijian/PhysX-3.4-master/PxShared/lib/linux64
#cgo LDFLAGS: -lPhysX3Extensions -lPhysX3_x64 -lPhysX3Common_x64 -lPhysX3CharacterKinematic_x64 -lPxFoundation_x64 -lPxPvdSDK_x64
#cgo LDFLAGS: -Wl,-rpath,/home/beijian/PhysX-3.4-master/PhysX_3.4/Bin/linux64 -Wl,-rpath,/home/beijian/PhysX-3.4-master/PxShared/bin/linux64
#cgo LDFLAGS: -lstdc++ -lm -lpthread -ldl

#include <stdlib.h>
#include "bridge.h"
*/
import "C"
import (
	"fmt"
	"os/exec"
	"strings"
	"time"
	"unsafe"
)

// getWindowsHostIP returns the IP of the Windows host from within WSL2.
// In WSL2, the Windows host is the default gateway.
func getWindowsHostIP() string {
	// Method 1: parse default route gateway
	out, err := exec.Command("sh", "-c", "ip route show default 2>/dev/null | awk '{print $3}'").Output()
	if err == nil {
		ip := strings.TrimSpace(string(out))
		if ip != "" {
			return ip
		}
	}
	// Method 2: parse resolv.conf (usually works in WSL2)
	out, err = exec.Command("sh", "-c", "cat /etc/resolv.conf 2>/dev/null | grep nameserver | awk '{print $2}'").Output()
	if err == nil {
		ip := strings.TrimSpace(string(out))
		if ip != "" {
			return ip
		}
	}
	// Fallback
	return "127.0.0.1"
}

func main() {
	fmt.Println("=== Go + PhysX 3.4 + PVD Demo ===")
	fmt.Println()

	hostIP := getWindowsHostIP()
	fmt.Printf("Detected Windows host IP: %s\n", hostIP)
	fmt.Printf("Make sure PVD is running on Windows (listening on port 5425)\n")
	fmt.Println()

	// Initialize PhysX
	var cHost *C.char
	if hostIP != "" {
		cHost = C.CString(hostIP)
		defer C.free(unsafe.Pointer(cHost))
	}
	ctx := C.physx_init(cHost)
	if ctx == nil {
		fmt.Println("ERROR: Failed to initialize PhysX!")
		return
	}

	dt := C.float(1.0 / 60.0)

	// Simulate ~10 seconds with real-time pacing
	fmt.Println("Simulating (10s, watch PVD window on Windows)...")
	for i := 0; i < 600; i++ {
		C.physx_step(ctx, dt)
		y := C.physx_get_sphere_y(ctx)
		if i%60 == 0 {
			fmt.Printf("  t=%.1fs  sphere Y=%.4f\n", float32(i)/60.0, float32(y))
		}
		time.Sleep(16 * time.Millisecond)
	}

	fmt.Println()
	fmt.Println("Simulation complete!")
	C.physx_cleanup(ctx)
}
