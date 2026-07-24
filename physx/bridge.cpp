#include "bridge.h"
#include "PxPhysicsAPI.h"

#include <stdio.h>
#include <string.h>
#include <vector>

using namespace physx;

/*============================================================================
 *  STATIC HELPERS: ERROR REPORTING
 *============================================================================*/

static int    g_last_error_code = PHYSX_SUCCESS;
static char   g_last_error_msg[512] = {0};

static void set_error(int code, const char* msg) {
    g_last_error_code = code;
    if (msg) {
        strncpy(g_last_error_msg, msg, sizeof(g_last_error_msg) - 1);
        g_last_error_msg[sizeof(g_last_error_msg) - 1] = '\0';
    } else {
        g_last_error_msg[0] = '\0';
    }
}

static void clear_error() {
    g_last_error_code = PHYSX_SUCCESS;
    g_last_error_msg[0] = '\0';
}

int physx_get_last_error_code(void) { return g_last_error_code; }
const char* physx_get_last_error_message(void) { return g_last_error_msg; }

/*============================================================================
 *  STATIC HELPERS: TYPE CONVERSIONS
 *============================================================================*/

static inline PxVec3 toPxVec3(const CPxVec3& v) { return PxVec3(v.x, v.y, v.z); }
static inline CPxVec3 toCPxVec3(const PxVec3& v) { CPxVec3 r = {v.x, v.y, v.z}; return r; }

static inline PxQuat toPxQuat(const CPxQuat& q) { return PxQuat(q.x, q.y, q.z, q.w); }
static inline CPxQuat toCPxQuat(const PxQuat& q) { CPxQuat r = {q.x, q.y, q.z, q.w}; return r; }

static inline PxTransform toPxTransform(const CPxTransform& t) {
    return PxTransform(toPxVec3(t.p), toPxQuat(t.q));
}
static inline CPxTransform toCPxTransform(const PxTransform& t) {
    CPxTransform r;
    r.p = toCPxVec3(t.p);
    r.q = toCPxQuat(t.q);
    return r;
}

static inline PxFilterData toPxFilterData(const CPxFilterData& d) {
    PxFilterData fd; fd.word0 = d.word0; fd.word1 = d.word1; fd.word2 = d.word2; fd.word3 = d.word3; return fd;
}

static inline CPxBounds3 toCPxBounds3(const PxBounds3& b) {
    CPxBounds3 r = { toCPxVec3(b.minimum), toCPxVec3(b.maximum) }; return r;
}

/*============================================================================
 *  OPAQUE HANDLE STRUCTS
 *============================================================================*/

struct PxFoundationHandle_ {
    PxDefaultAllocator     allocator;
    PxDefaultErrorCallback errorCallback;
    PxFoundation*          foundation;
};

struct PxPhysicsHandle_ {
    PxPhysics*             physics;
    PxPvd*                 pvd;
    PxPvdTransport*        pvdTransport;
    PxFoundationHandle_*   foundation;  /* back-ref for cleanup */
};

struct PxSceneHandle_ {
    PxScene*               scene;
    PxDefaultCpuDispatcher* dispatcher;

    /* callbacks */
    PhysxContactCallback   contactCb;
    void*                  contactCbData;
    PhysxTriggerCallback    triggerCb;
    void*                  triggerCbData;
    PhysxSleepCallback     sleepCb;
    void*                  sleepCbData;
    PhysxAdvanceCallback   advanceCb;
    void*                  advanceCbData;
};

struct PxMaterialHandle_ {
    PxMaterial* material;
};

struct PxActorHandle_ {
    PxRigidActor* actor;
    int           actorType; /* 0=static, 1=dynamic */
};

struct PxShapeHandle_ {
    PxShape* shape;
};

struct PxJointHandle_ {
    PxJoint* joint;
};

struct PxCookingHandle_ {
    PxCooking* cooking;
};

struct PxControllerMgrHandle_ {
    PxControllerManager* mgr;
};

struct PxControllerHandle_ {
    PxController* ctrl;
};

struct PxTriangleMeshHandle_ {
    PxTriangleMesh* mesh;
};

struct PxConvexMeshHandle_ {
    PxConvexMesh* mesh;
};

/*============================================================================
 *  SECTION 1: FOUNDATION & SDK LIFECYCLE
 *============================================================================*/

PxFoundationHandle physx_create_foundation(void) {
    clear_error();
    PxFoundationHandle_* h = new PxFoundationHandle_();
    h->foundation = NULL;

    h->foundation = PxCreateFoundation(
        PX_FOUNDATION_VERSION,
        h->allocator,
        h->errorCallback
    );
    if (!h->foundation) {
        set_error(PHYSX_ERROR_GENERIC, "PxCreateFoundation failed");
        delete h;
        return NULL;
    }
    return h;
}

void physx_release_foundation(PxFoundationHandle f) {
    if (!f) return;
    if (f->foundation) f->foundation->release();
    delete f;
}

PxPhysicsHandle physx_create_physics(PxFoundationHandle foundation, const char* pvd_host) {
    clear_error();
    if (!foundation || !foundation->foundation) {
        set_error(PHYSX_ERROR_NULL_PTR, "Null foundation");
        return NULL;
    }

    PxPhysicsHandle_* h = new PxPhysicsHandle_();
    h->physics      = NULL;
    h->pvd          = NULL;
    h->pvdTransport = NULL;
    h->foundation   = foundation;

    /* Optionally connect PVD */
    int pvd_enabled = (pvd_host && pvd_host[0] != '\0');
    if (pvd_enabled) {
        fprintf(stderr, "PVD: connecting to %s:5425 ...\n", pvd_host);
        h->pvd = PxCreatePvd(*foundation->foundation);
        h->pvdTransport = PxDefaultPvdSocketTransportCreate(pvd_host, 5425, 10);
        if (h->pvd && h->pvdTransport) {
            bool connected = h->pvd->connect(*h->pvdTransport, PxPvdInstrumentationFlag::eALL);
            if (connected) {
                fprintf(stderr, "PVD: connected successfully!\n");
            } else {
                fprintf(stderr, "PVD: connect() returned false (PVD not running?)\n");
            }
        } else {
            fprintf(stderr, "PVD: failed to create PVD/transport\n");
            if (h->pvd)       { h->pvd->release(); h->pvd = NULL; }
            if (h->pvdTransport) { h->pvdTransport->release(); h->pvdTransport = NULL; }
        }
    }

    /* Create physics */
    h->physics = PxCreatePhysics(
        PX_PHYSICS_VERSION,
        *foundation->foundation,
        PxTolerancesScale(),
        true,
        h->pvd  /* can be NULL */
    );
    if (!h->physics) {
        set_error(PHYSX_ERROR_GENERIC, "PxCreatePhysics failed");
        if (h->pvdTransport) h->pvdTransport->release();
        if (h->pvd) h->pvd->release();
        delete h;
        return NULL;
    }

    /* Initialize extensions (required for joints, etc.) */
    if (!PxInitExtensions(*h->physics, h->pvd)) {
        fprintf(stderr, "WARNING: PxInitExtensions returned false\n");
    }

    return h;
}

void physx_release_physics(PxPhysicsHandle p) {
    if (!p) return;
    PxCloseExtensions();
    if (p->physics)     p->physics->release();
    if (p->pvd) {
        if (p->pvdTransport) {
            p->pvd->disconnect();
            p->pvdTransport->release();
        }
        p->pvd->release();
    }
    delete p;
}

/*============================================================================
 *  SECTION 2: SCENE
 *============================================================================*/

/* Trampoline for simulation events */
class SceneEventCallback : public PxSimulationEventCallback {
public:
    PxSceneHandle_* sceneHandle;

    void onConstraintBreak(PxConstraintInfo*, PxU32) override {}
    void onAdvance(const PxRigidBody*const* bodyBuffer, const PxTransform* poseBuffer, const PxU32 count) override {
        if (!sceneHandle || !sceneHandle->advanceCb) return;
        if (count == 0) return;
        std::vector<PxActorHandle> actors(count);
        std::vector<CPxTransform> poses(count);
        for (PxU32 i = 0; i < count; i++) {
            actors[i] = NULL; /* we don't have handles for these bodies without lookup */
            poses[i] = toCPxTransform(poseBuffer[i]);
        }
        /* For advance callback, we just pass poses. Actors are raw PxRigidBody* not easily mapped. */
        sceneHandle->advanceCb(sceneHandle->advanceCbData,
                                NULL, poses.data(), (int)count);
    }
    void onWake(PxActor** actors, PxU32 count) override {
        if (!sceneHandle || !sceneHandle->sleepCb) return;
        if (count == 0) return;
        std::vector<PxActorHandle> ah(count);
        for (PxU32 i = 0; i < count; i++) {
            ah[i] = (PxActorHandle)actors[i]; /* raw pointer as handle for callback */
        }
        sceneHandle->sleepCb(sceneHandle->sleepCbData, ah.data(), (int)count, 1);
    }
    void onSleep(PxActor** actors, PxU32 count) override {
        if (!sceneHandle || !sceneHandle->sleepCb) return;
        if (count == 0) return;
        std::vector<PxActorHandle> ah(count);
        for (PxU32 i = 0; i < count; i++) {
            ah[i] = (PxActorHandle)actors[i];
        }
        sceneHandle->sleepCb(sceneHandle->sleepCbData, ah.data(), (int)count, 0);
    }
    void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) override {
        if (!sceneHandle || !sceneHandle->contactCb) return;
        if (nbPairs == 0) return;

        CPxContactPairHeader hdr;
        hdr.actors[0] = (uint64_t)(uintptr_t)pairHeader.actors[0];
        hdr.actors[1] = (uint64_t)(uintptr_t)pairHeader.actors[1];

        std::vector<CPxContactPair> cps(nbPairs);
        for (PxU32 i = 0; i < nbPairs; i++) {
            cps[i].shapes[0] = (uint64_t)(uintptr_t)pairs[i].shapes[0];
            cps[i].shapes[1] = (uint64_t)(uintptr_t)pairs[i].shapes[1];
            cps[i].actors[0] = (uint64_t)(uintptr_t)pairHeader.actors[0];
            cps[i].actors[1] = (uint64_t)(uintptr_t)pairHeader.actors[1];

            /* Extract first contact point */
            PxContactPairPoint pt;
            PxU32 nbContacts = pairs[i].extractContacts(&pt, 1);
            if (nbContacts > 0) {
                cps[i].contactPoint   = toCPxVec3(pt.position);
                cps[i].contactNormal  = toCPxVec3(pt.normal);
                cps[i].contactDistance = pt.separation;
                cps[i].impulse[0]     = pt.impulse.x;
                cps[i].impulse[1]     = pt.impulse.y;
                cps[i].internalFaceIndex0 = pt.internalFaceIndex0;
                cps[i].internalFaceIndex1 = pt.internalFaceIndex1;
            } else {
                memset(&cps[i].contactPoint, 0, sizeof(cps[i].contactPoint));
                memset(&cps[i].contactNormal, 0, sizeof(cps[i].contactNormal));
                cps[i].contactDistance = 0;
                cps[i].impulse[0] = cps[i].impulse[1] = 0;
                cps[i].internalFaceIndex0 = cps[i].internalFaceIndex1 = 0;
            }
            cps[i].events       = (uint32_t)pairs[i].events;
            cps[i].contactCount = (uint32_t)nbContacts;
        }

        hdr.shapes[0] = 0; /* not easily available from pairHeader without traversal */
        hdr.shapes[1] = 0;
        sceneHandle->contactCb(sceneHandle->contactCbData, &hdr, cps.data(), (int)nbPairs);
    }
    void onTrigger(PxTriggerPair* pairs, PxU32 count) override {
        if (!sceneHandle || !sceneHandle->triggerCb) return;
        if (count == 0) return;
        std::vector<CPxTriggerPair> tps(count);
        for (PxU32 i = 0; i < count; i++) {
            tps[i].triggerShape = (uint64_t)(uintptr_t)pairs[i].triggerShape;
            tps[i].triggerActor = (uint64_t)(uintptr_t)pairs[i].triggerActor;
            tps[i].otherShape   = (uint64_t)(uintptr_t)pairs[i].otherShape;
            tps[i].otherActor   = (uint64_t)(uintptr_t)pairs[i].otherActor;
            tps[i].status       = (uint32_t)pairs[i].status;
            tps[i].flags        = (uint32_t)pairs[i].flags;
        }
        sceneHandle->triggerCb(sceneHandle->triggerCbData, tps.data(), (int)count);
    }
};

PxSceneHandle physx_create_scene(PxPhysicsHandle physics, int num_threads,
                                  float gravity_x, float gravity_y, float gravity_z) {
    clear_error();
    if (!physics || !physics->physics) {
        set_error(PHYSX_ERROR_NULL_PTR, "Null physics");
        return NULL;
    }

    PxSceneHandle_* h = new PxSceneHandle_();
    h->scene       = NULL;
    h->dispatcher  = NULL;
    h->contactCb   = NULL;
    h->contactCbData = NULL;
    h->triggerCb   = NULL;
    h->triggerCbData = NULL;
    h->sleepCb     = NULL;
    h->sleepCbData = NULL;
    h->advanceCb   = NULL;
    h->advanceCbData = NULL;

    PxSceneDesc sceneDesc(physics->physics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(gravity_x, gravity_y, gravity_z);

    h->dispatcher = PxDefaultCpuDispatcherCreate(num_threads > 0 ? num_threads : 2);
    if (!h->dispatcher) {
        set_error(PHYSX_ERROR_GENERIC, "PxDefaultCpuDispatcherCreate failed");
        delete h;
        return NULL;
    }
    sceneDesc.cpuDispatcher = h->dispatcher;
    sceneDesc.filterShader  = PxDefaultSimulationFilterShader;

    h->scene = physics->physics->createScene(sceneDesc);
    if (!h->scene) {
        set_error(PHYSX_ERROR_GENERIC, "createScene failed");
        h->dispatcher->release();
        delete h;
        return NULL;
    }

    /* Register default event callback */
    SceneEventCallback* cb = new SceneEventCallback();
    cb->sceneHandle = h;
    h->scene->setSimulationEventCallback(cb);

    return h;
}

void physx_release_scene(PxSceneHandle scene) {
    if (!scene) return;
    /* Remove and delete the callback */
    PxSimulationEventCallback* cb = const_cast<PxSimulationEventCallback*>(scene->scene->getSimulationEventCallback());
    scene->scene->setSimulationEventCallback(NULL);
    delete cb;
    if (scene->scene)      scene->scene->release();
    if (scene->dispatcher)  scene->dispatcher->release();
    delete scene;
}

int physx_scene_simulate(PxSceneHandle scene, float dt) {
    if (!scene || !scene->scene) return PHYSX_ERROR_NULL_PTR;
    scene->scene->simulate(dt);
    scene->scene->fetchResults(true);
    return PHYSX_SUCCESS;
}

int physx_scene_simulate_start(PxSceneHandle scene, float dt) {
    if (!scene || !scene->scene) return PHYSX_ERROR_NULL_PTR;
    scene->scene->simulate(dt);
    return PHYSX_SUCCESS;
}

int physx_scene_fetch_results(PxSceneHandle scene, int block) {
    if (!scene || !scene->scene) return PHYSX_ERROR_NULL_PTR;
    scene->scene->fetchResults(block != 0);
    return PHYSX_SUCCESS;
}

int physx_scene_set_gravity(PxSceneHandle scene, float x, float y, float z) {
    if (!scene || !scene->scene) return PHYSX_ERROR_NULL_PTR;
    scene->scene->setGravity(PxVec3(x, y, z));
    return PHYSX_SUCCESS;
}

int physx_scene_get_gravity(PxSceneHandle scene, float* x, float* y, float* z) {
    if (!scene || !scene->scene) return PHYSX_ERROR_NULL_PTR;
    PxVec3 g = scene->scene->getGravity();
    if (x) *x = g.x; if (y) *y = g.y; if (z) *z = g.z;
    return PHYSX_SUCCESS;
}

int physx_scene_add_actor(PxSceneHandle scene, PxActorHandle actor) {
    if (!scene || !scene->scene || !actor || !actor->actor) return PHYSX_ERROR_NULL_PTR;
    scene->scene->addActor(*actor->actor);
    return PHYSX_SUCCESS;
}

int physx_scene_remove_actor(PxSceneHandle scene, PxActorHandle actor) {
    if (!scene || !scene->scene || !actor || !actor->actor) return PHYSX_ERROR_NULL_PTR;
    scene->scene->removeActor(*actor->actor);
    return PHYSX_SUCCESS;
}

int physx_scene_set_pvd_flags(PxSceneHandle scene, int constraints, int contacts, int queries) {
    if (!scene || !scene->scene) return PHYSX_ERROR_NULL_PTR;
    PxPvdSceneClient* client = scene->scene->getScenePvdClient();
    if (client) {
        client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, constraints != 0);
        client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, contacts != 0);
        client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, queries != 0);
    }
    return PHYSX_SUCCESS;
}

/*============================================================================
 *  SECTION 3: MATERIALS
 *============================================================================*/

PxMaterialHandle physx_create_material(PxPhysicsHandle physics,
                                        float sf, float df, float rest) {
    clear_error();
    if (!physics || !physics->physics) { set_error(PHYSX_ERROR_NULL_PTR, "Null physics"); return NULL; }
    PxMaterialHandle_* h = new PxMaterialHandle_();
    h->material = physics->physics->createMaterial(sf, df, rest);
    if (!h->material) { set_error(PHYSX_ERROR_GENERIC, "createMaterial failed"); delete h; return NULL; }
    return h;
}

void physx_release_material(PxMaterialHandle mat) {
    if (!mat) return;
    if (mat->material) mat->material->release();
    delete mat;
}

int physx_material_set_friction(PxMaterialHandle mat, float sf, float df) {
    if (!mat || !mat->material) return PHYSX_ERROR_NULL_PTR;
    mat->material->setStaticFriction(sf);
    mat->material->setDynamicFriction(df);
    return PHYSX_SUCCESS;
}

int physx_material_get_friction(PxMaterialHandle mat, float* sf, float* df) {
    if (!mat || !mat->material) return PHYSX_ERROR_NULL_PTR;
    if (sf) *sf = mat->material->getStaticFriction();
    if (df) *df = mat->material->getDynamicFriction();
    return PHYSX_SUCCESS;
}

int physx_material_set_restitution(PxMaterialHandle mat, float r) {
    if (!mat || !mat->material) return PHYSX_ERROR_NULL_PTR;
    mat->material->setRestitution(r);
    return PHYSX_SUCCESS;
}

float physx_material_get_restitution(PxMaterialHandle mat) {
    if (!mat || !mat->material) return 0;
    return mat->material->getRestitution();
}

int physx_material_set_friction_combine_mode(PxMaterialHandle mat, int mode) {
    if (!mat || !mat->material) return PHYSX_ERROR_NULL_PTR;
    mat->material->setFrictionCombineMode((PxCombineMode::Enum)mode);
    return PHYSX_SUCCESS;
}

int physx_material_set_restitution_combine_mode(PxMaterialHandle mat, int mode) {
    if (!mat || !mat->material) return PHYSX_ERROR_NULL_PTR;
    mat->material->setRestitutionCombineMode((PxCombineMode::Enum)mode);
    return PHYSX_SUCCESS;
}

/*============================================================================
 *  SECTION 4: ACTORS
 *============================================================================*/

static PxActorHandle_* make_actor_handle(PxRigidActor* a, int type) {
    if (!a) return NULL;
    PxActorHandle_* h = new PxActorHandle_();
    h->actor     = a;
    h->actorType = type;
    return h;
}

PxActorHandle physx_create_rigid_dynamic(PxPhysicsHandle physics,
                                          float px, float py, float pz,
                                          float qx, float qy, float qz, float qw) {
    clear_error();
    if (!physics || !physics->physics) { set_error(PHYSX_ERROR_NULL_PTR, "Null physics"); return NULL; }
    PxTransform pose(PxVec3(px,py,pz), PxQuat(qx,qy,qz,qw));
    PxRigidDynamic* a = physics->physics->createRigidDynamic(pose);
    return make_actor_handle(a, 1);
}

PxActorHandle physx_create_rigid_static(PxPhysicsHandle physics,
                                         float px, float py, float pz,
                                         float qx, float qy, float qz, float qw) {
    clear_error();
    if (!physics || !physics->physics) { set_error(PHYSX_ERROR_NULL_PTR, "Null physics"); return NULL; }
    PxTransform pose(PxVec3(px,py,pz), PxQuat(qx,qy,qz,qw));
    PxRigidStatic* a = physics->physics->createRigidStatic(pose);
    return make_actor_handle(a, 0);
}

PxActorHandle physx_create_dynamic_box(PxPhysicsHandle physics,
                                        float px, float py, float pz,
                                        float hx, float hy, float hz,
                                        PxMaterialHandle mat, float density) {
    clear_error();
    if (!physics || !physics->physics || !mat || !mat->material) return NULL;
    PxRigidDynamic* a = PxCreateDynamic(*physics->physics,
        PxTransform(PxVec3(px,py,pz)),
        PxBoxGeometry(hx, hy, hz),
        *mat->material, density);
    return make_actor_handle(a, 1);
}

PxActorHandle physx_create_dynamic_sphere(PxPhysicsHandle physics,
                                           float px, float py, float pz,
                                           float radius,
                                           PxMaterialHandle mat, float density) {
    clear_error();
    if (!physics || !physics->physics || !mat || !mat->material) return NULL;
    PxRigidDynamic* a = PxCreateDynamic(*physics->physics,
        PxTransform(PxVec3(px,py,pz)),
        PxSphereGeometry(radius),
        *mat->material, density);
    return make_actor_handle(a, 1);
}

PxActorHandle physx_create_dynamic_capsule(PxPhysicsHandle physics,
                                            float px, float py, float pz,
                                            float radius, float half_height,
                                            PxMaterialHandle mat, float density) {
    clear_error();
    if (!physics || !physics->physics || !mat || !mat->material) return NULL;
    PxRigidDynamic* a = PxCreateDynamic(*physics->physics,
        PxTransform(PxVec3(px,py,pz)),
        PxCapsuleGeometry(radius, half_height),
        *mat->material, density);
    return make_actor_handle(a, 1);
}

PxActorHandle physx_create_static_plane(PxPhysicsHandle physics,
                                         float nx, float ny, float nz, float d,
                                         PxMaterialHandle mat) {
    clear_error();
    if (!physics || !physics->physics || !mat || !mat->material) return NULL;
    PxRigidStatic* a = PxCreatePlane(*physics->physics,
        PxPlane(nx, ny, nz, d), *mat->material);
    return make_actor_handle(a, 0);
}

void physx_release_actor(PxActorHandle actor) {
    if (!actor) return;
    if (actor->actor) actor->actor->release();
    delete actor;
}

int physx_actor_get_global_pose(PxActorHandle actor, float* px, float* py, float* pz,
                                 float* qx, float* qy, float* qz, float* qw) {
    if (!actor || !actor->actor) return PHYSX_ERROR_NULL_PTR;
    PxTransform t = actor->actor->getGlobalPose();
    if (px) *px = t.p.x; if (py) *py = t.p.y; if (pz) *pz = t.p.z;
    if (qx) *qx = t.q.x; if (qy) *qy = t.q.y; if (qz) *qz = t.q.z; if (qw) *qw = t.q.w;
    return PHYSX_SUCCESS;
}

int physx_actor_set_global_pose(PxActorHandle actor,
                                 float px, float py, float pz,
                                 float qx, float qy, float qz, float qw,
                                 int autowake) {
    if (!actor || !actor->actor) return PHYSX_ERROR_NULL_PTR;
    actor->actor->setGlobalPose(PxTransform(PxVec3(px,py,pz), PxQuat(qx,qy,qz,qw)), autowake != 0);
    return PHYSX_SUCCESS;
}

/* Helper: cast to PxRigidBody* if dynamic */
static PxRigidBody* toBody(PxActorHandle a) {
    if (!a || a->actorType != 1) return NULL;
    return static_cast<PxRigidBody*>(a->actor);
}

static PxRigidDynamic* toDynamic(PxActorHandle a) {
    if (!a || a->actorType != 1) return NULL;
    return static_cast<PxRigidDynamic*>(a->actor);
}

int physx_actor_get_linear_velocity(PxActorHandle actor, float* x, float* y, float* z) {
    PxRigidBody* b = toBody(actor); if (!b) return PHYSX_ERROR_INVALID_ARG;
    PxVec3 v = b->getLinearVelocity();
    if (x) *x = v.x; if (y) *y = v.y; if (z) *z = v.z;
    return PHYSX_SUCCESS;
}

int physx_actor_set_linear_velocity(PxActorHandle actor, float x, float y, float z) {
    PxRigidBody* b = toBody(actor); if (!b) return PHYSX_ERROR_INVALID_ARG;
    b->setLinearVelocity(PxVec3(x,y,z));
    return PHYSX_SUCCESS;
}

int physx_actor_get_angular_velocity(PxActorHandle actor, float* x, float* y, float* z) {
    PxRigidBody* b = toBody(actor); if (!b) return PHYSX_ERROR_INVALID_ARG;
    PxVec3 v = b->getAngularVelocity();
    if (x) *x = v.x; if (y) *y = v.y; if (z) *z = v.z;
    return PHYSX_SUCCESS;
}

int physx_actor_set_angular_velocity(PxActorHandle actor, float x, float y, float z) {
    PxRigidBody* b = toBody(actor); if (!b) return PHYSX_ERROR_INVALID_ARG;
    b->setAngularVelocity(PxVec3(x,y,z));
    return PHYSX_SUCCESS;
}

int physx_actor_add_force(PxActorHandle actor, float fx, float fy, float fz, CPxForceMode mode, int autowake) {
    PxRigidBody* b = toBody(actor); if (!b) return PHYSX_ERROR_INVALID_ARG;
    b->addForce(PxVec3(fx,fy,fz), (PxForceMode::Enum)mode, autowake != 0);
    return PHYSX_SUCCESS;
}

int physx_actor_add_torque(PxActorHandle actor, float tx, float ty, float tz, CPxForceMode mode, int autowake) {
    PxRigidBody* b = toBody(actor); if (!b) return PHYSX_ERROR_INVALID_ARG;
    b->addTorque(PxVec3(tx,ty,tz), (PxForceMode::Enum)mode, autowake != 0);
    return PHYSX_SUCCESS;
}

int physx_actor_clear_force(PxActorHandle actor, CPxForceMode mode) {
    PxRigidBody* b = toBody(actor); if (!b) return PHYSX_ERROR_INVALID_ARG;
    b->clearForce((PxForceMode::Enum)mode);
    return PHYSX_SUCCESS;
}

int physx_actor_clear_torque(PxActorHandle actor, CPxForceMode mode) {
    PxRigidBody* b = toBody(actor); if (!b) return PHYSX_ERROR_INVALID_ARG;
    b->clearTorque((PxForceMode::Enum)mode);
    return PHYSX_SUCCESS;
}

int physx_actor_set_mass(PxActorHandle actor, float mass) {
    PxRigidBody* b = toBody(actor); if (!b) return PHYSX_ERROR_INVALID_ARG;
    b->setMass(mass);
    return PHYSX_SUCCESS;
}

float physx_actor_get_mass(PxActorHandle actor) {
    PxRigidBody* b = toBody(actor); if (!b) return 0;
    return b->getMass();
}

int physx_actor_set_mass_space_inertia_tensor(PxActorHandle actor,
                                               float m00, float m01, float m02,
                                               float m10, float m11, float m12,
                                               float m20, float m21, float m22) {
    PxRigidBody* b = toBody(actor); if (!b) return PHYSX_ERROR_INVALID_ARG;
    b->setMassSpaceInertiaTensor(PxVec3(m00,m01,m02));
    return PHYSX_SUCCESS;
}

int physx_actor_is_sleeping(PxActorHandle actor) {
    PxRigidDynamic* d = toDynamic(actor); if (!d) return 0;
    return d->isSleeping() ? 1 : 0;
}

int physx_actor_wake_up(PxActorHandle actor) {
    PxRigidDynamic* d = toDynamic(actor); if (!d) return PHYSX_ERROR_INVALID_ARG;
    d->wakeUp();
    return PHYSX_SUCCESS;
}

int physx_actor_put_to_sleep(PxActorHandle actor) {
    PxRigidDynamic* d = toDynamic(actor); if (!d) return PHYSX_ERROR_INVALID_ARG;
    d->putToSleep();
    return PHYSX_SUCCESS;
}

int physx_actor_set_kinematic_target(PxActorHandle actor,
                                      float px, float py, float pz,
                                      float qx, float qy, float qz, float qw) {
    PxRigidDynamic* d = toDynamic(actor); if (!d) return PHYSX_ERROR_INVALID_ARG;
    d->setKinematicTarget(PxTransform(PxVec3(px,py,pz), PxQuat(qx,qy,qz,qw)));
    return PHYSX_SUCCESS;
}

int physx_actor_set_linear_damping(PxActorHandle actor, float d) {
    PxRigidDynamic* dyn = toDynamic(actor); if (!dyn) return PHYSX_ERROR_INVALID_ARG;
    dyn->setLinearDamping(d);
    return PHYSX_SUCCESS;
}

int physx_actor_set_angular_damping(PxActorHandle actor, float d) {
    PxRigidDynamic* dyn = toDynamic(actor); if (!dyn) return PHYSX_ERROR_INVALID_ARG;
    dyn->setAngularDamping(d);
    return PHYSX_SUCCESS;
}

float physx_actor_get_linear_damping(PxActorHandle actor) {
    PxRigidDynamic* d = toDynamic(actor); if (!d) return 0;
    return d->getLinearDamping();
}

float physx_actor_get_angular_damping(PxActorHandle actor) {
    PxRigidDynamic* d = toDynamic(actor); if (!d) return 0;
    return d->getAngularDamping();
}

int physx_actor_set_actor_flags(PxActorHandle actor, uint32_t flags) {
    if (!actor || !actor->actor) return PHYSX_ERROR_NULL_PTR;
    actor->actor->setActorFlags((PxActorFlags)flags);
    return PHYSX_SUCCESS;
}

uint32_t physx_actor_get_actor_flags(PxActorHandle actor) {
    if (!actor || !actor->actor) return 0;
    return (uint32_t)(actor->actor->getActorFlags());
}

int physx_actor_set_rigid_body_flags(PxActorHandle actor, uint32_t flags) {
    PxRigidBody* b = toBody(actor); if (!b) return PHYSX_ERROR_INVALID_ARG;
    b->setRigidBodyFlags((PxRigidBodyFlags)flags);
    return PHYSX_SUCCESS;
}

uint32_t physx_actor_get_rigid_body_flags(PxActorHandle actor) {
    PxRigidBody* b = toBody(actor); if (!b) return 0;
    return (uint32_t)(b->getRigidBodyFlags());
}

int physx_actor_set_rigid_dynamic_lock_flags(PxActorHandle actor, uint32_t flags) {
    PxRigidDynamic* d = toDynamic(actor); if (!d) return PHYSX_ERROR_INVALID_ARG;
    d->setRigidDynamicLockFlags((PxRigidDynamicLockFlags)flags);
    return PHYSX_SUCCESS;
}

uint32_t physx_actor_get_rigid_dynamic_lock_flags(PxActorHandle actor) {
    PxRigidDynamic* d = toDynamic(actor); if (!d) return 0;
    return (uint32_t)(d->getRigidDynamicLockFlags());
}

int physx_actor_get_nb_shapes(PxActorHandle actor) {
    if (!actor || !actor->actor) return 0;
    return actor->actor->getNbShapes();
}

int physx_actor_get_type(PxActorHandle actor) {
    if (!actor) return -1;
    return actor->actorType;
}

/*============================================================================
 *  SECTION 5: SHAPES
 *============================================================================*/

PxShapeHandle physx_create_shape(PxPhysicsHandle physics, const CPxBoxGeometry* geom,
                                  PxMaterialHandle mat, int is_exclusive) {
    clear_error();
    if (!physics || !physics->physics || !geom || !mat || !mat->material) { set_error(PHYSX_ERROR_NULL_PTR, "Null arg"); return NULL; }
    PxShapeHandle_* h = new PxShapeHandle_();
    h->shape = physics->physics->createShape(
        PxBoxGeometry(geom->halfExtentsX, geom->halfExtentsY, geom->halfExtentsZ),
        *mat->material, is_exclusive != 0);
    if (!h->shape) { set_error(PHYSX_ERROR_GENERIC, "createShape failed"); delete h; return NULL; }
    return h;
}

PxShapeHandle physx_create_shape_sphere(PxPhysicsHandle physics, float radius,
                                         PxMaterialHandle mat, int is_exclusive) {
    clear_error();
    if (!physics || !physics->physics || !mat || !mat->material) return NULL;
    PxShapeHandle_* h = new PxShapeHandle_();
    h->shape = physics->physics->createShape(PxSphereGeometry(radius), *mat->material, is_exclusive != 0);
    if (!h->shape) { delete h; return NULL; }
    return h;
}

PxShapeHandle physx_create_shape_capsule(PxPhysicsHandle physics, float radius, float half_height,
                                          PxMaterialHandle mat, int is_exclusive) {
    clear_error();
    if (!physics || !physics->physics || !mat || !mat->material) return NULL;
    PxShapeHandle_* h = new PxShapeHandle_();
    h->shape = physics->physics->createShape(PxCapsuleGeometry(radius, half_height), *mat->material, is_exclusive != 0);
    if (!h->shape) { delete h; return NULL; }
    return h;
}

void physx_release_shape(PxShapeHandle shape) {
    if (!shape) return;
    if (shape->shape) shape->shape->release();
    delete shape;
}

int physx_actor_attach_shape(PxActorHandle actor, PxShapeHandle shape) {
    if (!actor || !actor->actor || !shape || !shape->shape) return PHYSX_ERROR_NULL_PTR;
    actor->actor->attachShape(*shape->shape);
    return PHYSX_SUCCESS;
}

int physx_actor_detach_shape(PxActorHandle actor, PxShapeHandle shape) {
    if (!actor || !actor->actor || !shape || !shape->shape) return PHYSX_ERROR_NULL_PTR;
    actor->actor->detachShape(*shape->shape);
    return PHYSX_SUCCESS;
}

int physx_shape_set_local_pose(PxShapeHandle shape,
                                float px, float py, float pz,
                                float qx, float qy, float qz, float qw) {
    if (!shape || !shape->shape) return PHYSX_ERROR_NULL_PTR;
    shape->shape->setLocalPose(PxTransform(PxVec3(px,py,pz), PxQuat(qx,qy,qz,qw)));
    return PHYSX_SUCCESS;
}

int physx_shape_set_flags(PxShapeHandle shape, uint32_t flags) {
    if (!shape || !shape->shape) return PHYSX_ERROR_NULL_PTR;
    shape->shape->setFlags((PxShapeFlags)flags);
    return PHYSX_SUCCESS;
}

uint32_t physx_shape_get_flags(PxShapeHandle shape) {
    if (!shape || !shape->shape) return 0;
    return (uint32_t)(shape->shape->getFlags());
}

int physx_shape_set_as_trigger(PxShapeHandle shape, int is_trigger) {
    if (!shape || !shape->shape) return PHYSX_ERROR_NULL_PTR;
    if (is_trigger) {
        shape->shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
        shape->shape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);
    } else {
        shape->shape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, false);
        shape->shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
    }
    return PHYSX_SUCCESS;
}

int physx_shape_set_simulation_filter_data(PxShapeHandle shape, const CPxFilterData* data) {
    if (!shape || !shape->shape || !data) return PHYSX_ERROR_NULL_PTR;
    shape->shape->setSimulationFilterData(toPxFilterData(*data));
    return PHYSX_SUCCESS;
}

int physx_shape_set_query_filter_data(PxShapeHandle shape, const CPxFilterData* data) {
    if (!shape || !shape->shape || !data) return PHYSX_ERROR_NULL_PTR;
    shape->shape->setQueryFilterData(toPxFilterData(*data));
    return PHYSX_SUCCESS;
}

int physx_shape_set_contact_offset(PxShapeHandle shape, float offset) {
    if (!shape || !shape->shape) return PHYSX_ERROR_NULL_PTR;
    shape->shape->setContactOffset(offset);
    return PHYSX_SUCCESS;
}

float physx_shape_get_contact_offset(PxShapeHandle shape) {
    if (!shape || !shape->shape) return 0;
    return shape->shape->getContactOffset();
}

/*============================================================================
 *  SECTION 6: SCENE QUERIES
 *============================================================================*/

static PxGeometryHolder makeGeometry(const void* geometry, int geom_type) {
    PxGeometryHolder holder;
    switch (geom_type) {
        case CPxGeometryType_SPHERE: {
            const CPxSphereGeometry* s = (const CPxSphereGeometry*)geometry;
            holder = PxSphereGeometry(s->radius);
            break;
        }
        case CPxGeometryType_BOX: {
            const CPxBoxGeometry* b = (const CPxBoxGeometry*)geometry;
            holder = PxBoxGeometry(b->halfExtentsX, b->halfExtentsY, b->halfExtentsZ);
            break;
        }
        case CPxGeometryType_CAPSULE: {
            const CPxCapsuleGeometry* c = (const CPxCapsuleGeometry*)geometry;
            holder = PxCapsuleGeometry(c->radius, c->halfHeight);
            break;
        }
        case CPxGeometryType_PLANE:
            holder = PxPlaneGeometry();
            break;
        default:
            break;
    }
    return holder;
}

int physx_scene_raycast(PxSceneHandle scene,
                         const CPxVec3* origin, const CPxVec3* direction, float max_dist,
                         uint32_t hit_flags, uint32_t query_flags,
                         const CPxFilterData* filter_data,
                         CPxRaycastHit* hit_buffer, int buffer_size) {
    if (!scene || !scene->scene || !origin || !direction || !hit_buffer || buffer_size <= 0)
        return PHYSX_ERROR_INVALID_ARG;

    PxQueryFilterData fd;
    if (filter_data) fd.data = toPxFilterData(*filter_data);
    fd.flags = (PxQueryFlags)query_flags;

    PxRaycastBufferN<256> buf;
    bool hit = scene->scene->raycast(
        toPxVec3(*origin), toPxVec3(*direction), max_dist, buf,
        (PxHitFlags)hit_flags, fd);

    if (!hit && !buf.hasBlock) return 0;

    int count = 0;
    if (buf.hasBlock) {
        hit_buffer[count].faceIndex = buf.block.faceIndex;
        hit_buffer[count].position  = toCPxVec3(buf.block.position);
        hit_buffer[count].normal    = toCPxVec3(buf.block.normal);
        hit_buffer[count].distance  = buf.block.distance;
        hit_buffer[count].flags     = (uint32_t)buf.block.flags;
        hit_buffer[count].actor     = (uint64_t)(uintptr_t)buf.block.actor;
        hit_buffer[count].shape     = (uint64_t)(uintptr_t)buf.block.shape;
        count++;
    }
    for (PxU32 i = 0; i < buf.nbTouches && count < buffer_size; i++) {
        hit_buffer[count].faceIndex = buf.touches[i].faceIndex;
        hit_buffer[count].position  = toCPxVec3(buf.touches[i].position);
        hit_buffer[count].normal    = toCPxVec3(buf.touches[i].normal);
        hit_buffer[count].distance  = buf.touches[i].distance;
        hit_buffer[count].flags     = (uint32_t)buf.touches[i].flags;
        hit_buffer[count].actor     = (uint64_t)(uintptr_t)buf.touches[i].actor;
        hit_buffer[count].shape     = (uint64_t)(uintptr_t)buf.touches[i].shape;
        count++;
    }
    return count;
}

int physx_scene_sweep(PxSceneHandle scene,
                       const void* geometry, int geom_type,
                       const CPxTransform* pose, const CPxVec3* direction, float max_dist,
                       uint32_t hit_flags, uint32_t query_flags,
                       const CPxFilterData* filter_data,
                       CPxSweepHit* hit_buffer, int buffer_size) {
    if (!scene || !scene->scene || !geometry || !pose || !direction || !hit_buffer || buffer_size <= 0)
        return PHYSX_ERROR_INVALID_ARG;

    PxQueryFilterData fd;
    if (filter_data) fd.data = toPxFilterData(*filter_data);
    fd.flags = (PxQueryFlags)query_flags;

    PxGeometryHolder holder = makeGeometry(geometry, geom_type);
    PxSweepBufferN<256> buf;

    bool hit = scene->scene->sweep(
        holder.any(), toPxTransform(*pose), toPxVec3(*direction), max_dist, buf,
        (PxHitFlags)hit_flags, fd);

    if (!hit && !buf.hasBlock) return 0;

    int count = 0;
    if (buf.hasBlock) {
        hit_buffer[count].faceIndex = buf.block.faceIndex;
        hit_buffer[count].position  = toCPxVec3(buf.block.position);
        hit_buffer[count].normal    = toCPxVec3(buf.block.normal);
        hit_buffer[count].distance  = buf.block.distance;
        hit_buffer[count].flags     = (uint32_t)buf.block.flags;
        hit_buffer[count].actor     = (uint64_t)(uintptr_t)buf.block.actor;
        hit_buffer[count].shape     = (uint64_t)(uintptr_t)buf.block.shape;
        count++;
    }
    for (PxU32 i = 0; i < buf.nbTouches && count < buffer_size; i++) {
        hit_buffer[count].faceIndex = buf.touches[i].faceIndex;
        hit_buffer[count].position  = toCPxVec3(buf.touches[i].position);
        hit_buffer[count].normal    = toCPxVec3(buf.touches[i].normal);
        hit_buffer[count].distance  = buf.touches[i].distance;
        hit_buffer[count].flags     = (uint32_t)buf.touches[i].flags;
        hit_buffer[count].actor     = (uint64_t)(uintptr_t)buf.touches[i].actor;
        hit_buffer[count].shape     = (uint64_t)(uintptr_t)buf.touches[i].shape;
        count++;
    }
    return count;
}

int physx_scene_overlap(PxSceneHandle scene,
                         const void* geometry, int geom_type,
                         const CPxTransform* pose,
                         uint32_t query_flags,
                         const CPxFilterData* filter_data,
                         CPxOverlapHit* hit_buffer, int buffer_size) {
    if (!scene || !scene->scene || !geometry || !pose || !hit_buffer || buffer_size <= 0)
        return PHYSX_ERROR_INVALID_ARG;

    PxQueryFilterData fd;
    if (filter_data) fd.data = toPxFilterData(*filter_data);
    fd.flags = (PxQueryFlags)query_flags;

    PxGeometryHolder holder = makeGeometry(geometry, geom_type);
    PxOverlapBufferN<256> buf;

    bool hit = scene->scene->overlap(holder.any(), toPxTransform(*pose), buf, fd);

    int count = 0;
    if (buf.hasBlock) {
        hit_buffer[count].faceIndex = buf.block.faceIndex;
        hit_buffer[count].actor     = (uint64_t)(uintptr_t)buf.block.actor;
        hit_buffer[count].shape     = (uint64_t)(uintptr_t)buf.block.shape;
        count++;
    }
    for (PxU32 i = 0; i < buf.nbTouches && count < buffer_size; i++) {
        hit_buffer[count].faceIndex = buf.touches[i].faceIndex;
        hit_buffer[count].actor     = (uint64_t)(uintptr_t)buf.touches[i].actor;
        hit_buffer[count].shape     = (uint64_t)(uintptr_t)buf.touches[i].shape;
        count++;
    }
    return count;
}

/*============================================================================
 *  SECTION 7: JOINTS
 *============================================================================*/

PxJointHandle physx_create_fixed_joint(PxPhysicsHandle physics,
                                        PxActorHandle a0, float px0,float py0,float pz0, float qx0,float qy0,float qz0,float qw0,
                                        PxActorHandle a1, float px1,float py1,float pz1, float qx1,float qy1,float qz1,float qw1) {
    clear_error();
    if (!physics || !physics->physics || !a0 || !a0->actor || !a1 || !a1->actor) {
        set_error(PHYSX_ERROR_NULL_PTR, "Null arg"); return NULL;
    }
    PxFixedJoint* j = PxFixedJointCreate(*physics->physics,
        a0->actor, PxTransform(PxVec3(px0,py0,pz0), PxQuat(qx0,qy0,qz0,qw0)),
        a1->actor, PxTransform(PxVec3(px1,py1,pz1), PxQuat(qx1,qy1,qz1,qw1)));
    if (!j) { set_error(PHYSX_ERROR_GENERIC, "create fixed joint failed"); return NULL; }
    PxJointHandle_* h = new PxJointHandle_(); h->joint = j; return h;
}

PxJointHandle physx_create_revolute_joint(PxPhysicsHandle physics,
                                           PxActorHandle a0, float px0,float py0,float pz0, float qx0,float qy0,float qz0,float qw0,
                                           PxActorHandle a1, float px1,float py1,float pz1, float qx1,float qy1,float qz1,float qw1) {
    clear_error();
    if (!physics || !physics->physics || !a0 || !a0->actor || !a1 || !a1->actor) { set_error(PHYSX_ERROR_NULL_PTR, "Null arg"); return NULL; }
    PxRevoluteJoint* j = PxRevoluteJointCreate(*physics->physics,
        a0->actor, PxTransform(PxVec3(px0,py0,pz0), PxQuat(qx0,qy0,qz0,qw0)),
        a1->actor, PxTransform(PxVec3(px1,py1,pz1), PxQuat(qx1,qy1,qz1,qw1)));
    if (!j) { set_error(PHYSX_ERROR_GENERIC, "create revolute joint failed"); return NULL; }
    PxJointHandle_* h = new PxJointHandle_(); h->joint = j; return h;
}

PxJointHandle physx_create_spherical_joint(PxPhysicsHandle physics,
                                            PxActorHandle a0, float px0,float py0,float pz0, float qx0,float qy0,float qz0,float qw0,
                                            PxActorHandle a1, float px1,float py1,float pz1, float qx1,float qy1,float qz1,float qw1) {
    clear_error();
    if (!physics || !physics->physics || !a0 || !a0->actor || !a1 || !a1->actor) { set_error(PHYSX_ERROR_NULL_PTR, "Null arg"); return NULL; }
    PxSphericalJoint* j = PxSphericalJointCreate(*physics->physics,
        a0->actor, PxTransform(PxVec3(px0,py0,pz0), PxQuat(qx0,qy0,qz0,qw0)),
        a1->actor, PxTransform(PxVec3(px1,py1,pz1), PxQuat(qx1,qy1,qz1,qw1)));
    if (!j) { set_error(PHYSX_ERROR_GENERIC, "create spherical joint failed"); return NULL; }
    PxJointHandle_* h = new PxJointHandle_(); h->joint = j; return h;
}

PxJointHandle physx_create_prismatic_joint(PxPhysicsHandle physics,
                                            PxActorHandle a0, float px0,float py0,float pz0, float qx0,float qy0,float qz0,float qw0,
                                            PxActorHandle a1, float px1,float py1,float pz1, float qx1,float qy1,float qz1,float qw1) {
    clear_error();
    if (!physics || !physics->physics || !a0 || !a0->actor || !a1 || !a1->actor) { set_error(PHYSX_ERROR_NULL_PTR, "Null arg"); return NULL; }
    PxPrismaticJoint* j = PxPrismaticJointCreate(*physics->physics,
        a0->actor, PxTransform(PxVec3(px0,py0,pz0), PxQuat(qx0,qy0,qz0,qw0)),
        a1->actor, PxTransform(PxVec3(px1,py1,pz1), PxQuat(qx1,qy1,qz1,qw1)));
    if (!j) { set_error(PHYSX_ERROR_GENERIC, "create prismatic joint failed"); return NULL; }
    PxJointHandle_* h = new PxJointHandle_(); h->joint = j; return h;
}

PxJointHandle physx_create_distance_joint(PxPhysicsHandle physics,
                                           PxActorHandle a0, float px0,float py0,float pz0, float qx0,float qy0,float qz0,float qw0,
                                           PxActorHandle a1, float px1,float py1,float pz1, float qx1,float qy1,float qz1,float qw1) {
    clear_error();
    if (!physics || !physics->physics || !a0 || !a0->actor || !a1 || !a1->actor) { set_error(PHYSX_ERROR_NULL_PTR, "Null arg"); return NULL; }
    PxDistanceJoint* j = PxDistanceJointCreate(*physics->physics,
        a0->actor, PxTransform(PxVec3(px0,py0,pz0), PxQuat(qx0,qy0,qz0,qw0)),
        a1->actor, PxTransform(PxVec3(px1,py1,pz1), PxQuat(qx1,qy1,qz1,qw1)));
    if (!j) { set_error(PHYSX_ERROR_GENERIC, "create distance joint failed"); return NULL; }
    PxJointHandle_* h = new PxJointHandle_(); h->joint = j; return h;
}

PxJointHandle physx_create_d6_joint(PxPhysicsHandle physics,
                                     PxActorHandle a0, float px0,float py0,float pz0, float qx0,float qy0,float qz0,float qw0,
                                     PxActorHandle a1, float px1,float py1,float pz1, float qx1,float qy1,float qz1,float qw1) {
    clear_error();
    if (!physics || !physics->physics || !a0 || !a0->actor || !a1 || !a1->actor) { set_error(PHYSX_ERROR_NULL_PTR, "Null arg"); return NULL; }
    PxD6Joint* j = PxD6JointCreate(*physics->physics,
        a0->actor, PxTransform(PxVec3(px0,py0,pz0), PxQuat(qx0,qy0,qz0,qw0)),
        a1->actor, PxTransform(PxVec3(px1,py1,pz1), PxQuat(qx1,qy1,qz1,qw1)));
    if (!j) { set_error(PHYSX_ERROR_GENERIC, "create d6 joint failed"); return NULL; }
    PxJointHandle_* h = new PxJointHandle_(); h->joint = j; return h;
}

void physx_release_joint(PxJointHandle joint) {
    if (!joint) return;
    if (joint->joint) joint->joint->release();
    delete joint;
}

int physx_joint_set_break_force(PxJointHandle joint, float force, float torque) {
    if (!joint || !joint->joint) return PHYSX_ERROR_NULL_PTR;
    joint->joint->setBreakForce(force, torque);
    return PHYSX_SUCCESS;
}

int physx_joint_get_break_force(PxJointHandle joint, float* force, float* torque) {
    if (!joint || !joint->joint) return PHYSX_ERROR_NULL_PTR;
    if (force || torque) {
        PxReal f, t;
        joint->joint->getBreakForce(f, t);
        if (force) *force = f;
        if (torque) *torque = t;
    }
    return PHYSX_SUCCESS;
}

int physx_joint_set_constraint_flags(PxJointHandle joint, uint32_t flags) {
    if (!joint || !joint->joint) return PHYSX_ERROR_NULL_PTR;
    joint->joint->setConstraintFlags((PxConstraintFlags)flags);
    return PHYSX_SUCCESS;
}

int physx_joint_set_constraint_flag(PxJointHandle joint, uint32_t flag, int enabled) {
    if (!joint || !joint->joint) return PHYSX_ERROR_NULL_PTR;
    joint->joint->setConstraintFlag((PxConstraintFlag::Enum)flag, enabled != 0);
    return PHYSX_SUCCESS;
}

uint32_t physx_joint_get_constraint_flags(PxJointHandle joint) {
    if (!joint || !joint->joint) return 0;
    return (uint32_t)(joint->joint->getConstraintFlags());
}

/* Revolute joint */
int physx_revolute_joint_set_limit(PxJointHandle joint, float lower, float upper, float stiffness, float damping) {
    if (!joint || !joint->joint) return PHYSX_ERROR_NULL_PTR;
    PxRevoluteJoint* r = static_cast<PxRevoluteJoint*>(joint->joint);
    r->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
    r->setLimit(PxJointAngularLimitPair(lower, upper, PxSpring(stiffness, damping)));
    return PHYSX_SUCCESS;
}

float physx_revolute_joint_get_angle(PxJointHandle joint) {
    if (!joint || !joint->joint) return 0;
    return static_cast<PxRevoluteJoint*>(joint->joint)->getAngle();
}

float physx_revolute_joint_get_velocity(PxJointHandle joint) {
    if (!joint || !joint->joint) return 0;
    return static_cast<PxRevoluteJoint*>(joint->joint)->getVelocity();
}

int physx_revolute_joint_set_drive_velocity(PxJointHandle joint, float velocity) {
    if (!joint || !joint->joint) return PHYSX_ERROR_NULL_PTR;
    PxRevoluteJoint* r = static_cast<PxRevoluteJoint*>(joint->joint);
    r->setDriveVelocity(velocity);
    r->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
    return PHYSX_SUCCESS;
}

int physx_revolute_joint_set_drive_force_limit(PxJointHandle joint, float limit) {
    if (!joint || !joint->joint) return PHYSX_ERROR_NULL_PTR;
    static_cast<PxRevoluteJoint*>(joint->joint)->setDriveForceLimit(limit);
    return PHYSX_SUCCESS;
}

/* Spherical joint */
int physx_spherical_joint_set_limit_cone(PxJointHandle joint, float yAngle, float zAngle, float stiffness, float damping) {
    if (!joint || !joint->joint) return PHYSX_ERROR_NULL_PTR;
    PxSphericalJoint* s = static_cast<PxSphericalJoint*>(joint->joint);
    s->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
    s->setLimitCone(PxJointLimitCone(yAngle, zAngle, PxSpring(stiffness, damping)));
    return PHYSX_SUCCESS;
}

/* Prismatic joint */
int physx_prismatic_joint_set_limit(PxJointHandle joint, float lower, float upper, float stiffness, float damping) {
    if (!joint || !joint->joint) return PHYSX_ERROR_NULL_PTR;
    PxPrismaticJoint* p = static_cast<PxPrismaticJoint*>(joint->joint);
    p->setPrismaticJointFlag(PxPrismaticJointFlag::eLIMIT_ENABLED, true);
    p->setLimit(PxJointLinearLimitPair(lower, upper, PxSpring(stiffness, damping)));
    return PHYSX_SUCCESS;
}

float physx_prismatic_joint_get_position(PxJointHandle joint) {
    if (!joint || !joint->joint) return 0;
    return static_cast<PxPrismaticJoint*>(joint->joint)->getPosition();
}

float physx_prismatic_joint_get_velocity(PxJointHandle joint) {
    if (!joint || !joint->joint) return 0;
    return static_cast<PxPrismaticJoint*>(joint->joint)->getVelocity();
}

/* Distance joint */
int physx_distance_joint_set_min_distance(PxJointHandle joint, float d) {
    if (!joint || !joint->joint) return PHYSX_ERROR_NULL_PTR;
    PxDistanceJoint* dj = static_cast<PxDistanceJoint*>(joint->joint);
    dj->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, true);
    dj->setMinDistance(d);
    return PHYSX_SUCCESS;
}

int physx_distance_joint_set_max_distance(PxJointHandle joint, float d) {
    if (!joint || !joint->joint) return PHYSX_ERROR_NULL_PTR;
    PxDistanceJoint* dj = static_cast<PxDistanceJoint*>(joint->joint);
    dj->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
    dj->setMaxDistance(d);
    return PHYSX_SUCCESS;
}

int physx_distance_joint_set_spring(PxJointHandle joint, float stiffness, float damping) {
    if (!joint || !joint->joint) return PHYSX_ERROR_NULL_PTR;
    PxDistanceJoint* dj = static_cast<PxDistanceJoint*>(joint->joint);
    dj->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, true);
    dj->setStiffness(stiffness);
    dj->setDamping(damping);
    return PHYSX_SUCCESS;
}

/* D6 joint */
int physx_d6_joint_set_motion(PxJointHandle joint, CPxD6Axis axis, CPxD6Motion motion) {
    if (!joint || !joint->joint) return PHYSX_ERROR_NULL_PTR;
    static_cast<PxD6Joint*>(joint->joint)->setMotion((PxD6Axis::Enum)axis, (PxD6Motion::Enum)motion);
    return PHYSX_SUCCESS;
}

int physx_d6_joint_set_drive(PxJointHandle joint, CPxD6Drive drive, const CPxD6JointDrive* d) {
    if (!joint || !joint->joint || !d) return PHYSX_ERROR_NULL_PTR;
    PxD6JointDrive dd;
    dd.stiffness  = d->stiffness;
    dd.damping    = d->damping;
    dd.forceLimit = d->forceLimit;
    dd.flags      = (PxD6JointDriveFlags)d->flags;
    static_cast<PxD6Joint*>(joint->joint)->setDrive((PxD6Drive::Enum)drive, dd);
    return PHYSX_SUCCESS;
}

int physx_d6_joint_set_drive_position(PxJointHandle joint,
                                       float px, float py, float pz,
                                       float qx, float qy, float qz, float qw) {
    if (!joint || !joint->joint) return PHYSX_ERROR_NULL_PTR;
    static_cast<PxD6Joint*>(joint->joint)->setDrivePosition(
        PxTransform(PxVec3(px,py,pz), PxQuat(qx,qy,qz,qw)));
    return PHYSX_SUCCESS;
}

/*============================================================================
 *  SECTION 8: SIMULATION EVENT CALLBACKS
 *============================================================================*/

int physx_scene_set_contact_callback(PxSceneHandle scene, PhysxContactCallback cb, void* userdata) {
    if (!scene) return PHYSX_ERROR_NULL_PTR;
    scene->contactCb = cb;
    scene->contactCbData = userdata;
    return PHYSX_SUCCESS;
}

int physx_scene_set_trigger_callback(PxSceneHandle scene, PhysxTriggerCallback cb, void* userdata) {
    if (!scene) return PHYSX_ERROR_NULL_PTR;
    scene->triggerCb = cb;
    scene->triggerCbData = userdata;
    return PHYSX_SUCCESS;
}

int physx_scene_set_sleep_callback(PxSceneHandle scene, PhysxSleepCallback cb, void* userdata) {
    if (!scene) return PHYSX_ERROR_NULL_PTR;
    scene->sleepCb = cb;
    scene->sleepCbData = userdata;
    return PHYSX_SUCCESS;
}

int physx_scene_set_advance_callback(PxSceneHandle scene, PhysxAdvanceCallback cb, void* userdata) {
    if (!scene) return PHYSX_ERROR_NULL_PTR;
    scene->advanceCb = cb;
    scene->advanceCbData = userdata;
    return PHYSX_SUCCESS;
}

/*============================================================================
 *  SECTION 9: CHARACTER CONTROLLER
 *============================================================================*/

PxControllerMgrHandle physx_create_controller_manager(PxSceneHandle scene) {
    clear_error();
    if (!scene || !scene->scene) { set_error(PHYSX_ERROR_NULL_PTR, "Null scene"); return NULL; }
    PxControllerMgrHandle_* h = new PxControllerMgrHandle_();
    h->mgr = PxCreateControllerManager(*scene->scene);
    if (!h->mgr) { set_error(PHYSX_ERROR_GENERIC, "create controller manager failed"); delete h; return NULL; }
    return h;
}

void physx_release_controller_manager(PxControllerMgrHandle mgr) {
    if (!mgr) return;
    if (mgr->mgr) mgr->mgr->release();
    delete mgr;
}

PxControllerHandle physx_create_box_controller(PxControllerMgrHandle mgr,
                                                PxPhysicsHandle physics,
                                                float half_height, float half_side_extent,
                                                float px, float py, float pz,
                                                PxMaterialHandle mat) {
    clear_error();
    if (!mgr || !mgr->mgr || !physics || !physics->physics) { set_error(PHYSX_ERROR_NULL_PTR, "Null arg"); return NULL; }

    PxBoxControllerDesc desc;
    desc.halfHeight        = half_height;
    desc.halfSideExtent    = half_side_extent;
    desc.position          = PxExtendedVec3(px, py, pz);
    desc.material          = mat ? mat->material : NULL;
    desc.stepOffset        = 0.5f;
    desc.contactOffset     = 0.1f;
    desc.upDirection       = PxVec3(0, 1, 0);
    desc.slopeLimit        = 0.707f;
    desc.nonWalkableMode   = PxControllerNonWalkableMode::ePREVENT_CLIMBING;

    PxControllerHandle_* h = new PxControllerHandle_();
    h->ctrl = mgr->mgr->createController(desc);
    if (!h->ctrl) { set_error(PHYSX_ERROR_GENERIC, "create box controller failed"); delete h; return NULL; }
    return h;
}

PxControllerHandle physx_create_capsule_controller(PxControllerMgrHandle mgr,
                                                    PxPhysicsHandle physics,
                                                    float radius, float height,
                                                    float px, float py, float pz,
                                                    PxMaterialHandle mat) {
    clear_error();
    if (!mgr || !mgr->mgr || !physics || !physics->physics) { set_error(PHYSX_ERROR_NULL_PTR, "Null arg"); return NULL; }

    PxCapsuleControllerDesc desc;
    desc.radius            = radius;
    desc.height            = height;
    desc.position          = PxExtendedVec3(px, py, pz);
    desc.material          = mat ? mat->material : NULL;
    desc.stepOffset        = 0.5f;
    desc.contactOffset     = 0.1f;
    desc.upDirection       = PxVec3(0, 1, 0);
    desc.slopeLimit        = 0.707f;
    desc.nonWalkableMode   = PxControllerNonWalkableMode::ePREVENT_CLIMBING;

    PxControllerHandle_* h = new PxControllerHandle_();
    h->ctrl = mgr->mgr->createController(desc);
    if (!h->ctrl) { set_error(PHYSX_ERROR_GENERIC, "create capsule controller failed"); delete h; return NULL; }
    return h;
}

void physx_release_controller(PxControllerHandle ctrl) {
    if (!ctrl) return;
    if (ctrl->ctrl) ctrl->ctrl->release();
    delete ctrl;
}

PxActorHandle physx_controller_get_actor(PxControllerHandle ctrl) {
    if (!ctrl || !ctrl->ctrl) return NULL;
    PxRigidDynamic* a = ctrl->ctrl->getActor();
    if (!a) return NULL;
    PxActorHandle_* h = new PxActorHandle_();
    h->actor     = a;
    h->actorType = 1;
    return h;
}

int physx_controller_move(PxControllerHandle ctrl,
                           float dx, float dy, float dz,
                           float min_dist, float dt) {
    if (!ctrl || !ctrl->ctrl) return PHYSX_ERROR_NULL_PTR;
    PxControllerFilters filters;
    PxControllerCollisionFlags flags = ctrl->ctrl->move(PxVec3(dx,dy,dz), min_dist, dt, filters);
    return (int)(PxU32)flags;
}

int physx_controller_get_position(PxControllerHandle ctrl, float* x, float* y, float* z) {
    if (!ctrl || !ctrl->ctrl) return PHYSX_ERROR_NULL_PTR;
    PxExtendedVec3 p = ctrl->ctrl->getPosition();
    if (x) *x = (float)p.x; if (y) *y = (float)p.y; if (z) *z = (float)p.z;
    return PHYSX_SUCCESS;
}

int physx_controller_set_position(PxControllerHandle ctrl, float x, float y, float z) {
    if (!ctrl || !ctrl->ctrl) return PHYSX_ERROR_NULL_PTR;
    ctrl->ctrl->setPosition(PxExtendedVec3(x, y, z));
    return PHYSX_SUCCESS;
}

int physx_controller_get_foot_position(PxControllerHandle ctrl, float* x, float* y, float* z) {
    if (!ctrl || !ctrl->ctrl) return PHYSX_ERROR_NULL_PTR;
    PxExtendedVec3 p = ctrl->ctrl->getFootPosition();
    if (x) *x = (float)p.x; if (y) *y = (float)p.y; if (z) *z = (float)p.z;
    return PHYSX_SUCCESS;
}

int physx_controller_set_foot_position(PxControllerHandle ctrl, float x, float y, float z) {
    if (!ctrl || !ctrl->ctrl) return PHYSX_ERROR_NULL_PTR;
    ctrl->ctrl->setFootPosition(PxExtendedVec3(x, y, z));
    return PHYSX_SUCCESS;
}

int physx_controller_set_step_offset(PxControllerHandle ctrl, float offset) {
    if (!ctrl || !ctrl->ctrl) return PHYSX_ERROR_NULL_PTR;
    ctrl->ctrl->setStepOffset(offset);
    return PHYSX_SUCCESS;
}

int physx_controller_set_slope_limit(PxControllerHandle ctrl, float limit) {
    if (!ctrl || !ctrl->ctrl) return PHYSX_ERROR_NULL_PTR;
    ctrl->ctrl->setSlopeLimit(limit);
    return PHYSX_SUCCESS;
}

/*============================================================================
 *  SECTION 10: COOKING
 *============================================================================*/

PxCookingHandle physx_create_cooking(void) {
    clear_error();
    PxCookingHandle_* h = new PxCookingHandle_();
    PxCookingParams params{PxTolerancesScale()};
    h->cooking = PxCreateCooking(PX_PHYSICS_VERSION, PxGetFoundation(), params);
    if (!h->cooking) { set_error(PHYSX_ERROR_GENERIC, "create cooking failed"); delete h; return NULL; }
    return h;
}

void physx_release_cooking(PxCookingHandle cooking) {
    if (!cooking) return;
    if (cooking->cooking) cooking->cooking->release();
    delete cooking;
}

PxConvexMeshHandle physx_cook_convex_mesh(PxCookingHandle cooking,
                                           const CPxVec3* vertices, int num_vertices,
                                           int* out_error) {
    clear_error();
    if (!cooking || !cooking->cooking || !vertices || num_vertices < 3) {
        set_error(PHYSX_ERROR_INVALID_ARG, "Invalid args for cook convex mesh");
        if (out_error) *out_error = -1;
        return NULL;
    }

    PxConvexMeshDesc desc;
    desc.points.count   = (PxU32)num_vertices;
    desc.points.stride  = sizeof(CPxVec3);
    desc.points.data    = vertices;
    desc.flags          = PxConvexFlag::eCOMPUTE_CONVEX;

    PxDefaultMemoryOutputStream buf;
    PxConvexMeshCookingResult::Enum result;
    if (!cooking->cooking->cookConvexMesh(desc, buf, &result)) {
        set_error(PHYSX_ERROR_GENERIC, "cookConvexMesh failed");
        if (out_error) *out_error = (int)result;
        return NULL;
    }

    PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
    PxConvexMeshHandle_* h = new PxConvexMeshHandle_();
    h->mesh = PxGetPhysics().createConvexMesh(input);
    if (!h->mesh) {
        set_error(PHYSX_ERROR_GENERIC, "createConvexMesh failed");
        delete h;
        if (out_error) *out_error = -1;
        return NULL;
    }
    if (out_error) *out_error = 0;
    return h;
}

PxTriangleMeshHandle physx_cook_triangle_mesh(PxCookingHandle cooking,
                                               const CPxVec3* vertices, int num_vertices,
                                               const uint32_t* indices, int num_indices,
                                               int* out_error) {
    clear_error();
    if (!cooking || !cooking->cooking || !vertices || num_vertices < 3 || !indices || num_indices < 3) {
        set_error(PHYSX_ERROR_INVALID_ARG, "Invalid args for cook triangle mesh");
        if (out_error) *out_error = -1;
        return NULL;
    }

    PxTriangleMeshDesc desc;
    desc.points.count   = (PxU32)num_vertices;
    desc.points.stride  = sizeof(CPxVec3);
    desc.points.data    = vertices;
    desc.triangles.count  = (PxU32)(num_indices / 3);
    desc.triangles.stride = 3 * sizeof(uint32_t);
    desc.triangles.data   = indices;

    PxDefaultMemoryOutputStream buf;
    PxTriangleMeshCookingResult::Enum result;
    if (!cooking->cooking->cookTriangleMesh(desc, buf, &result)) {
        set_error(PHYSX_ERROR_GENERIC, "cookTriangleMesh failed");
        if (out_error) *out_error = (int)result;
        return NULL;
    }

    PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
    PxTriangleMeshHandle_* h = new PxTriangleMeshHandle_();
    h->mesh = PxGetPhysics().createTriangleMesh(input);
    if (!h->mesh) {
        set_error(PHYSX_ERROR_GENERIC, "createTriangleMesh failed");
        delete h;
        if (out_error) *out_error = -1;
        return NULL;
    }
    if (out_error) *out_error = 0;
    return h;
}

void physx_release_convex_mesh(PxConvexMeshHandle mesh) {
    if (!mesh) return;
    if (mesh->mesh) mesh->mesh->release();
    delete mesh;
}

void physx_release_triangle_mesh(PxTriangleMeshHandle mesh) {
    if (!mesh) return;
    if (mesh->mesh) mesh->mesh->release();
    delete mesh;
}

PxShapeHandle physx_create_convex_mesh_shape(PxPhysicsHandle physics,
                                              PxConvexMeshHandle mesh,
                                              PxMaterialHandle mat, int is_exclusive) {
    clear_error();
    if (!physics || !physics->physics || !mesh || !mesh->mesh || !mat || !mat->material) {
        set_error(PHYSX_ERROR_NULL_PTR, "Null arg"); return NULL;
    }
    PxShapeHandle_* h = new PxShapeHandle_();
    h->shape = physics->physics->createShape(
        PxConvexMeshGeometry(mesh->mesh), *mat->material, is_exclusive != 0);
    if (!h->shape) { delete h; return NULL; }
    return h;
}

PxShapeHandle physx_create_triangle_mesh_shape(PxPhysicsHandle physics,
                                                PxTriangleMeshHandle mesh,
                                                PxMaterialHandle mat, int is_exclusive) {
    clear_error();
    if (!physics || !physics->physics || !mesh || !mesh->mesh || !mat || !mat->material) {
        set_error(PHYSX_ERROR_NULL_PTR, "Null arg"); return NULL;
    }
    PxShapeHandle_* h = new PxShapeHandle_();
    h->shape = physics->physics->createShape(
        PxTriangleMeshGeometry(mesh->mesh), *mat->material, is_exclusive != 0);
    if (!h->shape) { delete h; return NULL; }
    return h;
}

/*============================================================================
 *  SECTION 11: VEHICLE (stub)
 *============================================================================*/

int physx_init_vehicle_sdk(PxPhysicsHandle physics) {
    if (!physics || !physics->physics) return PHYSX_ERROR_NULL_PTR;
    /* Vehicle SDK requires PxInitVehicleSDK which needs serialization registry
       For now, this is a stub - PxInitVehicleSDK is declared in PxVehicleSDK.h */
    fprintf(stderr, "Vehicle SDK: init skipped (full vehicle support needs serialization registry)\n");
    return PHYSX_SUCCESS;
}

PxVehicleHandle physx_create_vehicle_4w(PxPhysicsHandle physics, PxActorHandle chassis) {
    fprintf(stderr, "Vehicle: create_4w is a stub (full vehicle support TBD)\n");
    return NULL;
}

int physx_vehicle_update(PxVehicleHandle vehicle, float dt) {
    return PHYSX_ERROR_GENERIC;
}

int physx_vehicle_set_input(PxVehicleHandle vehicle, float t, float b, float hb, float s, int gear) {
    return PHYSX_ERROR_GENERIC;
}

void physx_release_vehicle(PxVehicleHandle vehicle) {}

int physx_close_vehicle_sdk(void) {
    return PHYSX_SUCCESS;
}

/*============================================================================
 *  SECTION 12: UTILITY
 *============================================================================*/

int physx_actor_get_world_bounds(PxActorHandle actor, CPxBounds3* bounds) {
    if (!actor || !actor->actor || !bounds) return PHYSX_ERROR_NULL_PTR;
    PxBounds3 b = actor->actor->getWorldBounds(1.0f);
    *bounds = toCPxBounds3(b);
    return PHYSX_SUCCESS;
}

int physx_scene_get_active_actors(PxSceneHandle scene, int type_flag,
                                   PxActorHandle* buffer, int buffer_size) {
    if (!scene || !scene->scene || !buffer || buffer_size <= 0) return PHYSX_ERROR_INVALID_ARG;
    PxActorTypeFlags flags = (PxActorTypeFlags)type_flag;
    PxU32 count = scene->scene->getNbActors(flags);
    if (count > (PxU32)buffer_size) count = (PxU32)buffer_size;
    scene->scene->getActors(flags, (PxActor**)buffer, count);
    return (int)count;
}
