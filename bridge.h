#ifndef PHYSX_BRIDGE_H
#define PHYSX_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to PhysX simulation context
typedef struct PhysXContext PhysXContext;

// Create and initialize PhysX.
// pvd_host: IP address of the PVD server (e.g. "127.0.0.1" or "172.23.224.1").
//           Pass NULL or empty string to disable PVD.
PhysXContext* physx_init(const char* pvd_host);

// Step the simulation forward (dt in seconds, e.g. 1/60)
void physx_step(PhysXContext* ctx, float dt);

// Get the Y position of the falling sphere (returns -1 if no sphere)
float physx_get_sphere_y(PhysXContext* ctx);

// Cleanup and release all PhysX resources
void physx_cleanup(PhysXContext* ctx);

#ifdef __cplusplus
}
#endif

#endif // PHYSX_BRIDGE_H
