#include "bridge.h"
#include "PxPhysicsAPI.h"

#include <stdio.h>

using namespace physx;

struct PhysXContext {
    PxDefaultAllocator       allocator;
    PxDefaultErrorCallback   errorCallback;

    PxFoundation*            foundation;
    PxPvd*                   pvd;
    PxPvdTransport*          pvdTransport;

    PxPhysics*               physics;
    PxDefaultCpuDispatcher*  dispatcher;
    PxScene*                 scene;
    PxMaterial*              material;
    PxRigidDynamic*          sphere;
};

PhysXContext* physx_init(const char* pvd_host)
{
    PhysXContext* ctx = new PhysXContext();
    ctx->foundation   = NULL;
    ctx->pvd          = NULL;
    ctx->pvdTransport = NULL;
    ctx->physics      = NULL;
    ctx->dispatcher   = NULL;
    ctx->scene        = NULL;
    ctx->material     = NULL;
    ctx->sphere       = NULL;

    // 1. Create foundation
    ctx->foundation = PxCreateFoundation(
        PX_FOUNDATION_VERSION,
        ctx->allocator,
        ctx->errorCallback
    );
    if (!ctx->foundation) {
        fprintf(stderr, "PxCreateFoundation failed!\n");
        delete ctx;
        return NULL;
    }

    // 2. Optionally connect PVD
    int pvd_enabled = (pvd_host && pvd_host[0] != '\0');
    if (pvd_enabled) {
        fprintf(stderr, "PVD: attempting to connect to %s:5425 ...\n", pvd_host);
        ctx->pvd = PxCreatePvd(*ctx->foundation);
        ctx->pvdTransport = PxDefaultPvdSocketTransportCreate(pvd_host, 5425, 10);
        if (ctx->pvd && ctx->pvdTransport) {
            bool connected = ctx->pvd->connect(*ctx->pvdTransport, PxPvdInstrumentationFlag::eALL);
            if (connected) {
                fprintf(stderr, "PVD: connected successfully!\n");
            } else {
                fprintf(stderr, "PVD: connect() returned false (PVD may not be running on Windows?)\n");
            }
        } else {
            fprintf(stderr, "PVD: failed to create PVD/transport (pvd=%p transport=%p)\n",
                    (void*)ctx->pvd, (void*)ctx->pvdTransport);
            if (ctx->pvd)       { ctx->pvd->release(); ctx->pvd = NULL; }
            if (ctx->pvdTransport) { ctx->pvdTransport->release(); ctx->pvdTransport = NULL; }
        }
    } else {
        fprintf(stderr, "PVD: disabled (no host provided)\n");
    }

    // 3. Create physics
    ctx->physics = PxCreatePhysics(
        PX_PHYSICS_VERSION,
        *ctx->foundation,
        PxTolerancesScale(),
        true,
        ctx->pvd   // can be NULL
    );
    if (!ctx->physics) {
        fprintf(stderr, "PxCreatePhysics failed!\n");
        if (ctx->pvdTransport) ctx->pvdTransport->release();
        if (ctx->pvd) ctx->pvd->release();
        ctx->foundation->release();
        delete ctx;
        return NULL;
    }

    // 4. Create scene
    PxSceneDesc sceneDesc(ctx->physics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
    ctx->dispatcher = PxDefaultCpuDispatcherCreate(2);
    if (!ctx->dispatcher) {
        fprintf(stderr, "PxDefaultCpuDispatcherCreate failed!\n");
        ctx->physics->release();
        if (ctx->pvdTransport) ctx->pvdTransport->release();
        if (ctx->pvd) ctx->pvd->release();
        ctx->foundation->release();
        delete ctx;
        return NULL;
    }
    sceneDesc.cpuDispatcher  = ctx->dispatcher;
    sceneDesc.filterShader   = PxDefaultSimulationFilterShader;
    ctx->scene = ctx->physics->createScene(sceneDesc);

    // 5. PVD scene flags
    if (ctx->pvd) {
        PxPvdSceneClient* pvdClient = ctx->scene->getScenePvdClient();
        if (pvdClient) {
            pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
            pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
            pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
            fprintf(stderr, "PVD: scene flags set (constraints, contacts, queries)\n");
        }
    }

    // 6. Create material
    ctx->material = ctx->physics->createMaterial(0.5f, 0.5f, 0.6f);

    // 7. Ground plane
    PxRigidStatic* groundPlane = PxCreatePlane(
        *ctx->physics,
        PxPlane(0, 1, 0, 0),
        *ctx->material
    );
    ctx->scene->addActor(*groundPlane);

    // 8. Falling sphere
    PxShape* shape = ctx->physics->createShape(
        PxSphereGeometry(1.0f),
        *ctx->material
    );
    ctx->sphere = PxCreateDynamic(
        *ctx->physics,
        PxTransform(PxVec3(0.0f, 20.0f, 0.0f)),
        PxSphereGeometry(1.0f),
        *ctx->material,
        10.0f
    );
    if (ctx->sphere) {
        ctx->sphere->setLinearVelocity(PxVec3(5.0f, 0.0f, 0.0f));
        ctx->scene->addActor(*ctx->sphere);
    }
    shape->release();

    printf("PhysX initialized successfully!\n");
    return ctx;
}

void physx_step(PhysXContext* ctx, float dt)
{
    if (!ctx || !ctx->scene) return;
    ctx->scene->simulate(dt);
    ctx->scene->fetchResults(true);
}

float physx_get_sphere_y(PhysXContext* ctx)
{
    if (!ctx || !ctx->sphere) return -1.0f;
    PxTransform t = ctx->sphere->getGlobalPose();
    return t.p.y;
}

void physx_cleanup(PhysXContext* ctx)
{
    if (!ctx) return;
    if (ctx->scene)      ctx->scene->release();
    if (ctx->dispatcher)  ctx->dispatcher->release();
    if (ctx->physics)     ctx->physics->release();
    if (ctx->pvd) {
        if (ctx->pvdTransport) {
            ctx->pvd->disconnect();
            ctx->pvdTransport->release();
        }
        ctx->pvd->release();
    }
    if (ctx->foundation)  ctx->foundation->release();
    delete ctx;
    printf("PhysX cleaned up.\n");
}
