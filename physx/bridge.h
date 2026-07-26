#ifndef PHYSX_BRIDGE_H
#define PHYSX_BRIDGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 *  ERROR CODES
 *============================================================================*/
#define PHYSX_SUCCESS            0
#define PHYSX_ERROR_GENERIC     -1
#define PHYSX_ERROR_NULL_PTR    -2
#define PHYSX_ERROR_INVALID_ARG -3

/*============================================================================
 *  C-COMPATIBLE MATH TYPES (layout-compatible with PhysX POD types)
 *============================================================================*/

typedef struct { float x, y, z; }              CPxVec3;
typedef struct { float x, y, z, w; }           CPxVec4;
typedef struct { CPxVec3 column0; CPxVec3 column1; CPxVec3 column2; } CPxMat33;
typedef struct { CPxVec4 column0; CPxVec4 column1; CPxVec4 column2; CPxVec4 column3; } CPxMat44;
typedef struct { float x, y, z, w; }           CPxQuat;
typedef struct { CPxQuat q; CPxVec3 p; }       CPxTransform;
typedef struct { CPxVec3 minimum, maximum; }    CPxBounds3;
typedef struct { CPxVec3 normal; float d; }    CPxPlane;

/*============================================================================
 *  FILTER DATA
 *============================================================================*/
typedef struct {
    uint32_t word0, word1, word2, word3;
} CPxFilterData;

/*============================================================================
 *  GEOMETRY TYPES (C-compatible POD)
 *============================================================================*/
typedef enum {
    CPxGeometryType_SPHERE = 0,
    CPxGeometryType_PLANE,
    CPxGeometryType_CAPSULE,
    CPxGeometryType_BOX,
    CPxGeometryType_CONVEXMESH,
    CPxGeometryType_TRIANGLEMESH,
    CPxGeometryType_HEIGHTFIELD
} CPxGeometryType;

typedef struct {
    CPxGeometryType type;  /* CPxGeometryType_SPHERE */
    float radius;
} CPxSphereGeometry;

typedef struct {
    CPxGeometryType type;  /* CPxGeometryType_BOX */
    float halfExtentsX, halfExtentsY, halfExtentsZ;
} CPxBoxGeometry;

typedef struct {
    CPxGeometryType type;  /* CPxGeometryType_CAPSULE */
    float radius;
    float halfHeight;
} CPxCapsuleGeometry;

typedef struct {
    CPxGeometryType type;  /* CPxGeometryType_PLANE */
} CPxPlaneGeometry;

/*============================================================================
 *  SCENE QUERY HIT TYPES
 *============================================================================*/
typedef struct {
    uint32_t  faceIndex;
    CPxVec3   position;
    CPxVec3   normal;
    float     distance;
    uint32_t  flags;    /* PxHitFlag bits */
    uint64_t  actor;    /* opaque actor handle */
    uint64_t  shape;    /* opaque shape handle */
} CPxRaycastHit;

typedef struct {
    uint32_t  faceIndex;
    CPxVec3   position;
    CPxVec3   normal;
    float     distance;
    uint32_t  flags;
    uint64_t  actor;
    uint64_t  shape;
} CPxSweepHit;

typedef struct {
    uint32_t  faceIndex;
    uint32_t  _pad;
    uint64_t  actor;
    uint64_t  shape;
} CPxOverlapHit;

/*============================================================================
 *  SIMULATION EVENT TYPES
 *============================================================================*/
typedef enum {
    CPxActorType_RIGID_STATIC  = 0,
    CPxActorType_RIGID_DYNAMIC = 1
} CPxActorType;

typedef struct {
    uint64_t triggerShape;
    uint64_t triggerActor;
    uint64_t otherShape;
    uint64_t otherActor;
    uint32_t status;   /* eTOUCH_FOUND=0, eTOUCH_LOST=1 */
    uint32_t flags;
} CPxTriggerPair;

typedef struct {
    uint64_t shapes[2];
    uint64_t actors[2];
} CPxContactPairHeader;

typedef struct {
    uint64_t shapes[2];
    uint64_t actors[2];
    CPxVec3  contactPoint;
    CPxVec3  contactNormal;
    float    contactDistance;
    float    impulse[2];
    uint32_t internalFaceIndex0;
    uint32_t internalFaceIndex1;
    uint32_t events;          /* PxPairFlag bits */
    uint32_t contactCount;
} CPxContactPair;

/* Per-contact-point data (for extractContacts) */
typedef struct {
    CPxVec3  position;
    CPxVec3  normal;
    CPxVec3  impulse;
    float    separation;
    uint32_t internalFaceIndex0;
    uint32_t internalFaceIndex1;
} CPxContactPairPoint;

/*============================================================================
 *  JOINT TYPES
 *============================================================================*/
typedef enum {
    CPxJointType_FIXED = 0,
    CPxJointType_REVOLUTE,
    CPxJointType_SPHERICAL,
    CPxJointType_PRISMATIC,
    CPxJointType_DISTANCE,
    CPxJointType_D6
} CPxJointType;

typedef enum {
    CPxD6Axis_X = 0,
    CPxD6Axis_Y,
    CPxD6Axis_Z,
    CPxD6Axis_TWIST,
    CPxD6Axis_SWING1,
    CPxD6Axis_SWING2
} CPxD6Axis;

typedef enum {
    CPxD6Motion_LOCKED = 0,
    CPxD6Motion_LIMITED,
    CPxD6Motion_FREE
} CPxD6Motion;

typedef enum {
    CPxD6Drive_X = 0,
    CPxD6Drive_Y,
    CPxD6Drive_Z,
    CPxD6Drive_SWING,
    CPxD6Drive_TWIST,
    CPxD6Drive_SLERP
} CPxD6Drive;

typedef struct {
    float stiffness;
    float damping;
    float forceLimit;
    int   flags;
} CPxD6JointDrive;

/*============================================================================
 *  FORCE MODE
 *============================================================================*/
typedef enum {
    CPxForceMode_FORCE = 0,
    CPxForceMode_IMPULSE,
    CPxForceMode_VELOCITY_CHANGE,
    CPxForceMode_ACCELERATION
} CPxForceMode;

/*============================================================================
 *  FILTER SHADER FLAGS (PxPairFlag / PxFilterFlag)
 *============================================================================*/

/* Pair flags — OR'd into *pairFlags by filter shader.
   MUST match PxPairFlag::Enum in PxFiltering.h exactly! */
typedef enum {
    CPxPairFlag_SOLVE_CONTACT                 = 1 << 0,   /* eSOLVE_CONTACT */
    CPxPairFlag_MODIFY_CONTACTS               = 1 << 1,   /* eMODIFY_CONTACTS */
    CPxPairFlag_NOTIFY_TOUCH_FOUND            = 1 << 2,   /* eNOTIFY_TOUCH_FOUND */
    CPxPairFlag_NOTIFY_TOUCH_PERSISTS         = 1 << 3,   /* eNOTIFY_TOUCH_PERSISTS */
    CPxPairFlag_NOTIFY_TOUCH_LOST             = 1 << 4,   /* eNOTIFY_TOUCH_LOST */
    CPxPairFlag_NOTIFY_TOUCH_CCD              = 1 << 5,   /* eNOTIFY_TOUCH_CCD */
    CPxPairFlag_NOTIFY_THRESHOLD_FORCE_FOUND  = 1 << 6,   /* eNOTIFY_THRESHOLD_FORCE_FOUND */
    CPxPairFlag_NOTIFY_THRESHOLD_FORCE_PERSISTS = 1 << 7, /* eNOTIFY_THRESHOLD_FORCE_PERSISTS */
    CPxPairFlag_NOTIFY_THRESHOLD_FORCE_LOST   = 1 << 8,   /* eNOTIFY_THRESHOLD_FORCE_LOST */
    CPxPairFlag_NOTIFY_CONTACT_POINTS         = 1 << 9,   /* eNOTIFY_CONTACT_POINTS */
    CPxPairFlag_DETECT_DISCRETE_CONTACT       = 1 << 10,  /* eDETECT_DISCRETE_CONTACT */
    CPxPairFlag_DETECT_CCD_CONTACT            = 1 << 11,  /* eDETECT_CCD_CONTACT */
    CPxPairFlag_PRE_SOLVER_VELOCITY           = 1 << 12,  /* ePRE_SOLVER_VELOCITY */
    CPxPairFlag_POST_SOLVER_VELOCITY          = 1 << 13,  /* ePOST_SOLVER_VELOCITY */
    CPxPairFlag_CONTACT_EVENT_POSE            = 1 << 14   /* eCONTACT_EVENT_POSE */
} CPxPairFlag;

/* Filter flags — returned by filter shader */
typedef enum {
    CPxFilterFlag_DEFAULT   = 0,
    CPxFilterFlag_KILL      = 1 << 0,  /* eKILL */
    CPxFilterFlag_SUPPRESS  = 1 << 1,  /* eSUPPRESS */
    CPxFilterFlag_CALLBACK  = 1 << 2,  /* eCALLBACK */
    CPxFilterFlag_NOTIFY    = 1 << 3   /* eNOTIFY */
} CPxFilterFlag;

/* Filter shader callback:
   - receives attributes and filter data for both shapes
   - writes desired pair flags into *pairFlags
   - returns filter flags (CPxFilterFlag bits)
*/
typedef uint32_t (*PhysxFilterShaderCallback)(
    uint32_t             attributes0,
    const CPxFilterData* filterData0,
    uint32_t             attributes1,
    const CPxFilterData* filterData1,
    uint32_t*            pairFlags,
    void*                userdata);

/*============================================================================
 *  CONTACT MODIFY TYPES
 *============================================================================*/
typedef struct {
    uint64_t     actors[2];     /* opaque actor handles */
    uint64_t     shapes[2];     /* opaque shape handles */
    CPxTransform transforms[2]; /* global poses of both actors */
} CPxContactModifyPair;

/* Called during PxContactModifyCallback::onContactModify.
   Call physx_contact_modify_set_inv_mass_scale / set_inv_inertia_scale
   to adjust the contact response of each pair. */
typedef void (*PhysxContactModifyCallback)(void* userdata,
    const CPxContactModifyPair* pairs, int nbPairs);

/*============================================================================
 *  ACTOR FLAGS
 *============================================================================*/
typedef enum {
    CPxActorFlag_VISUALIZATION     = 1 << 0,
    CPxActorFlag_DISABLE_GRAVITY   = 1 << 1,
    CPxActorFlag_SEND_SLEEP_NOTIFIES = 1 << 2,
    CPxActorFlag_DISABLE_SIMULATION  = 1 << 3
} CPxActorFlag;

typedef enum {
    CPxRigidBodyFlag_KINEMATIC                          = 1 << 0,
    CPxRigidBodyFlag_USE_KINEMATIC_TARGET_FOR_SCENE_QUERIES = 1 << 1,
    CPxRigidBodyFlag_ENABLE_CCD                         = 1 << 2,
    CPxRigidBodyFlag_ENABLE_CCD_FRICTION                = 1 << 3,
    CPxRigidBodyFlag_ENABLE_POSE_INTEGRATION_PREVIEW    = 1 << 4,
    CPxRigidBodyFlag_ENABLE_SPECULATIVE_CCD             = 1 << 5,
    CPxRigidBodyFlag_ENABLE_CCD_MAX_CONTACT_IMPULSE     = 1 << 6
} CPxRigidBodyFlag;

typedef enum {
    CPxRigidDynamicLockFlag_LINEAR_X  = 1 << 0,
    CPxRigidDynamicLockFlag_LINEAR_Y  = 1 << 1,
    CPxRigidDynamicLockFlag_LINEAR_Z  = 1 << 2,
    CPxRigidDynamicLockFlag_ANGULAR_X = 1 << 3,
    CPxRigidDynamicLockFlag_ANGULAR_Y = 1 << 4,
    CPxRigidDynamicLockFlag_ANGULAR_Z = 1 << 5
} CPxRigidDynamicLockFlag;

/*============================================================================
 *  SHAPE FLAGS
 *============================================================================*/
typedef enum {
    CPxShapeFlag_SIMULATION_SHAPE = 1 << 0,
    CPxShapeFlag_SCENE_QUERY_SHAPE = 1 << 1,
    CPxShapeFlag_TRIGGER_SHAPE     = 1 << 2,
    CPxShapeFlag_VISUALIZATION     = 1 << 3
} CPxShapeFlag;

/*============================================================================
 *  SCENE QUERY FLAGS
 *============================================================================*/
typedef enum {
    CPxHitFlag_POSITION = 1 << 0,
    CPxHitFlag_NORMAL   = 1 << 1,
    CPxHitFlag_DISTANCE = 1 << 2,
    CPxHitFlag_UV       = 1 << 3,
    CPxHitFlag_FACE_INDEX = 1 << 7
} CPxHitFlag;

typedef enum {
    CPxQueryFlag_STATIC     = 1 << 0,
    CPxQueryFlag_DYNAMIC    = 1 << 1,
    CPxQueryFlag_PREFILTER  = 1 << 2,
    CPxQueryFlag_POSTFILTER = 1 << 3,
    CPxQueryFlag_ANY_HIT    = 1 << 4,
    CPxQueryFlag_NO_BLOCK   = 1 << 5
} CPxQueryFlag;

/*============================================================================
 *  OPAQUE HANDLE TYPEDEFS
 *============================================================================*/
typedef struct PxFoundationHandle_*    PxFoundationHandle;
typedef struct PxPhysicsHandle_*       PxPhysicsHandle;
typedef struct PxSceneHandle_*         PxSceneHandle;
typedef struct PxMaterialHandle_*      PxMaterialHandle;
typedef struct PxActorHandle_*         PxActorHandle;
typedef struct PxShapeHandle_*         PxShapeHandle;
typedef struct PxJointHandle_*         PxJointHandle;
typedef struct PxCookingHandle_*       PxCookingHandle;
typedef struct PxControllerMgrHandle_* PxControllerMgrHandle;
typedef struct PxControllerHandle_*    PxControllerHandle;
typedef struct PxTriangleMeshHandle_*  PxTriangleMeshHandle;
typedef struct PxConvexMeshHandle_*    PxConvexMeshHandle;

/*============================================================================
 *  ERROR REPORTING
 *============================================================================*/
int  physx_get_last_error_code(void);
const char* physx_get_last_error_message(void);

/*============================================================================
 *  SECTION 1: FOUNDATION & SDK LIFECYCLE
 *============================================================================*/

/**
 * Create a PxFoundation. Must be the first PhysX call.
 * Returns NULL on failure; check physx_get_last_error_code().
 */
PxFoundationHandle physx_create_foundation(void);
void physx_release_foundation(PxFoundationHandle f);

/**
 * Create PxPhysics (optionally with PVD).
 * @param foundation   Foundation handle from physx_create_foundation
 * @param pvd_host     PVD server IP, or NULL/"" to disable PVD
 * @return Physics handle, or NULL on failure
 */
PxPhysicsHandle physx_create_physics(PxFoundationHandle foundation, const char* pvd_host);
void physx_release_physics(PxPhysicsHandle p);

/*============================================================================
 *  SECTION 2: SCENE
 *============================================================================*/

/**
 * Create a PxScene.
 * @param physics      Physics handle
 * @param num_threads  CPU dispatcher thread count (typically 2-4)
 * @param gravity_x/y/z  Scene gravity (e.g. 0, -9.81, 0)
 * @return Scene handle, or NULL on failure
 */
PxSceneHandle physx_create_scene(PxPhysicsHandle physics, int num_threads,
                                  float gravity_x, float gravity_y, float gravity_z);
void physx_release_scene(PxSceneHandle scene);

/** Advance simulation by dt seconds */
int physx_scene_simulate(PxSceneHandle scene, float dt);

/** Start simulation async, then fetch results */
int physx_scene_simulate_start(PxSceneHandle scene, float dt);
int physx_scene_fetch_results(PxSceneHandle scene, int block);

/** Gravity */
int physx_scene_set_gravity(PxSceneHandle scene, float x, float y, float z);
int physx_scene_get_gravity(PxSceneHandle scene, float* x, float* y, float* z);

/** Add/remove actors */
int physx_scene_add_actor(PxSceneHandle scene, PxActorHandle actor);
int physx_scene_remove_actor(PxSceneHandle scene, PxActorHandle actor);

/** PVD scene client flags */
int physx_scene_set_pvd_flags(PxSceneHandle scene, int transmit_constraints,
                               int transmit_contacts, int transmit_scenequeries);

/** PVD visualization parameters */
int physx_scene_set_vis_param(PxSceneHandle scene, int param_id, float value);

/** Register a custom filter shader callback (replaces default). Pass NULL/0 to restore default. */
int physx_scene_set_filter_shader(PxSceneHandle scene,
                                   PhysxFilterShaderCallback cb, void* userdata);

/** Enable/disable CCD at the scene level and set max passes.
    Must be called BEFORE creating CCD-enabled actors. */
int physx_scene_enable_ccd(PxSceneHandle scene, int enabled, int max_passes);

/*============================================================================
 *  SECTION 3: MATERIALS
 *============================================================================*/

PxMaterialHandle physx_create_material(PxPhysicsHandle physics,
                                        float static_friction,
                                        float dynamic_friction,
                                        float restitution);
void physx_release_material(PxMaterialHandle mat);

int physx_material_set_friction(PxMaterialHandle mat, float static_friction, float dynamic_friction);
int physx_material_get_friction(PxMaterialHandle mat, float* static_friction, float* dynamic_friction);
int physx_material_set_restitution(PxMaterialHandle mat, float restitution);
float physx_material_get_restitution(PxMaterialHandle mat);
int physx_material_set_friction_combine_mode(PxMaterialHandle mat, int mode); /* 0=AVERAGE,1=MIN,2=MULTIPLY,3=MAX */
int physx_material_set_restitution_combine_mode(PxMaterialHandle mat, int mode);

/*============================================================================
 *  SECTION 4: ACTORS
 *============================================================================*/

/** Create a dynamic rigid body */
PxActorHandle physx_create_rigid_dynamic(PxPhysicsHandle physics,
                                          float px, float py, float pz,
                                          float qx, float qy, float qz, float qw);

/** Create a static rigid body */
PxActorHandle physx_create_rigid_static(PxPhysicsHandle physics,
                                         float px, float py, float pz,
                                         float qx, float qy, float qz, float qw);

/** Quick create: dynamic + box/sphere/capsule geometry all in one */
PxActorHandle physx_create_dynamic_box(PxPhysicsHandle physics,
                                        float px, float py, float pz,
                                        float hx, float hy, float hz,
                                        PxMaterialHandle mat, float density);

PxActorHandle physx_create_dynamic_sphere(PxPhysicsHandle physics,
                                           float px, float py, float pz,
                                           float radius,
                                           PxMaterialHandle mat, float density);

PxActorHandle physx_create_dynamic_capsule(PxPhysicsHandle physics,
                                            float px, float py, float pz,
                                            float radius, float half_height,
                                            PxMaterialHandle mat, float density);

/** Create a static plane actor (infinite plane at y=0, normal up) */
PxActorHandle physx_create_static_plane(PxPhysicsHandle physics,
                                         float nx, float ny, float nz, float d,
                                         PxMaterialHandle mat);

void physx_release_actor(PxActorHandle actor);

/** Pose */
int physx_actor_get_global_pose(PxActorHandle actor, float* px, float* py, float* pz,
                                 float* qx, float* qy, float* qz, float* qw);
int physx_actor_set_global_pose(PxActorHandle actor,
                                 float px, float py, float pz,
                                 float qx, float qy, float qz, float qw,
                                 int autowake);

/** Linear velocity (dynamic only) */
int physx_actor_get_linear_velocity(PxActorHandle actor, float* x, float* y, float* z);
int physx_actor_set_linear_velocity(PxActorHandle actor, float x, float y, float z);

/** Angular velocity (dynamic only) */
int physx_actor_get_angular_velocity(PxActorHandle actor, float* x, float* y, float* z);
int physx_actor_set_angular_velocity(PxActorHandle actor, float x, float y, float z);

/** Forces (dynamic only) */
int physx_actor_add_force(PxActorHandle actor, float fx, float fy, float fz, CPxForceMode mode, int autowake);
int physx_actor_add_torque(PxActorHandle actor, float tx, float ty, float tz, CPxForceMode mode, int autowake);
int physx_actor_clear_force(PxActorHandle actor, CPxForceMode mode);
int physx_actor_clear_torque(PxActorHandle actor, CPxForceMode mode);

/** Mass properties (dynamic only) */
int physx_actor_set_mass(PxActorHandle actor, float mass);
float physx_actor_get_mass(PxActorHandle actor);
int physx_actor_set_mass_space_inertia_tensor(PxActorHandle actor,
                                               float m00, float m01, float m02,
                                               float m10, float m11, float m12,
                                               float m20, float m21, float m22);

/** Sleep (dynamic only) */
int physx_actor_is_sleeping(PxActorHandle actor);
int physx_actor_wake_up(PxActorHandle actor);
int physx_actor_put_to_sleep(PxActorHandle actor);

/** Kinematic target (dynamic + kinematic flag) */
int physx_actor_set_kinematic_target(PxActorHandle actor,
                                      float px, float py, float pz,
                                      float qx, float qy, float qz, float qw);

/** Damping */
int physx_actor_set_linear_damping(PxActorHandle actor, float damping);
int physx_actor_set_angular_damping(PxActorHandle actor, float damping);
float physx_actor_get_linear_damping(PxActorHandle actor);
float physx_actor_get_angular_damping(PxActorHandle actor);

/** Flags */
int physx_actor_set_actor_flags(PxActorHandle actor, uint32_t flags);
uint32_t physx_actor_get_actor_flags(PxActorHandle actor);
int physx_actor_set_rigid_body_flags(PxActorHandle actor, uint32_t flags);
uint32_t physx_actor_get_rigid_body_flags(PxActorHandle actor);
int physx_actor_set_rigid_dynamic_lock_flags(PxActorHandle actor, uint32_t flags);
uint32_t physx_actor_get_rigid_dynamic_lock_flags(PxActorHandle actor);

/** Get number of shapes attached */
int physx_actor_get_nb_shapes(PxActorHandle actor);

/** Get underlying actor type: 0=RIGID_STATIC, 1=RIGID_DYNAMIC */
int physx_actor_get_type(PxActorHandle actor);

/** Compute mass & inertia from geometry + density (PxRigidBodyExt::updateMassAndInertia) */
int physx_actor_update_mass_and_inertia(PxActorHandle actor, float density);

/*============================================================================
 *  SECTION 5: SHAPES
 *============================================================================*/

PxShapeHandle physx_create_shape(PxPhysicsHandle physics, const CPxBoxGeometry* geom,
                                  PxMaterialHandle mat, int is_exclusive);
PxShapeHandle physx_create_shape_sphere(PxPhysicsHandle physics, float radius,
                                         PxMaterialHandle mat, int is_exclusive);
PxShapeHandle physx_create_shape_capsule(PxPhysicsHandle physics, float radius, float half_height,
                                          PxMaterialHandle mat, int is_exclusive);

void physx_release_shape(PxShapeHandle shape);

/** Attach / detach from actor */
int physx_actor_attach_shape(PxActorHandle actor, PxShapeHandle shape);
int physx_actor_detach_shape(PxActorHandle actor, PxShapeHandle shape);

/** Shape local pose */
int physx_shape_set_local_pose(PxShapeHandle shape,
                                float px, float py, float pz,
                                float qx, float qy, float qz, float qw);

/** Shape flags */
int physx_shape_set_flags(PxShapeHandle shape, uint32_t flags);
uint32_t physx_shape_get_flags(PxShapeHandle shape);

/** Shape: set as trigger */
int physx_shape_set_as_trigger(PxShapeHandle shape, int is_trigger);

/** Shape filter data */
int physx_shape_set_simulation_filter_data(PxShapeHandle shape, const CPxFilterData* data);
int physx_shape_set_query_filter_data(PxShapeHandle shape, const CPxFilterData* data);

/** Shape contact offset */
int physx_shape_set_contact_offset(PxShapeHandle shape, float offset);
float physx_shape_get_contact_offset(PxShapeHandle shape);

/*============================================================================
 *  SECTION 6: SCENE QUERIES
 *============================================================================*/

/**
 * Raycast against scene.
 * @param origin      Ray origin
 * @param direction   Ray direction (will be normalized internally if not unit)
 * @param max_dist    Maximum distance
 * @param hit_flags   CPxHitFlag bits
 * @param query_flags CPxQueryFlag bits
 * @param filter_data Optional filter data (NULL = default)
 * @param hit_buffer  Pre-allocated buffer for hits
 * @param buffer_size Max number of hits
 * @return Number of hits (0 = nothing hit)
 */
int physx_scene_raycast(PxSceneHandle scene,
                         const CPxVec3* origin, const CPxVec3* direction, float max_dist,
                         uint32_t hit_flags, uint32_t query_flags,
                         const CPxFilterData* filter_data,
                         CPxRaycastHit* hit_buffer, int buffer_size);

/**
 * Sweep a geometry through the scene.
 * @param geometry    The geometry to sweep (by pointer to CPxBoxGeometry/CPxSphereGeometry/CPxCapsuleGeometry)
 * @param geom_type   CPxGeometryType of the geometry
 * @param pose        Initial pose of the geometry
 * @param direction   Sweep direction
 * @param max_dist    Maximum distance
 * @param hit_flags   CPxHitFlag bits
 * @param query_flags CPxQueryFlag bits
 * @param filter_data Optional filter data (NULL = default)
 * @param hit_buffer  Pre-allocated buffer
 * @param buffer_size Max number of hits
 * @return Number of hits
 */
int physx_scene_sweep(PxSceneHandle scene,
                       const void* geometry, int geom_type,
                       const CPxTransform* pose, const CPxVec3* direction, float max_dist,
                       uint32_t hit_flags, uint32_t query_flags,
                       const CPxFilterData* filter_data,
                       CPxSweepHit* hit_buffer, int buffer_size);

/**
 * Overlap test against scene.
 * @param geometry    The geometry to test
 * @param geom_type   CPxGeometryType
 * @param pose        Pose of the geometry
 * @param query_flags CPxQueryFlag bits
 * @param filter_data Optional filter data (NULL = default)
 * @param hit_buffer  Pre-allocated buffer
 * @param buffer_size Max number of overlaps
 * @return Number of overlaps
 */
int physx_scene_overlap(PxSceneHandle scene,
                         const void* geometry, int geom_type,
                         const CPxTransform* pose,
                         uint32_t query_flags,
                         const CPxFilterData* filter_data,
                         CPxOverlapHit* hit_buffer, int buffer_size);

/*============================================================================
 *  SECTION 7: JOINT TYPES
 *============================================================================*/

PxJointHandle physx_create_fixed_joint(PxPhysicsHandle physics,
                                        PxActorHandle actor0, float px0, float py0, float pz0, float qx0, float qy0, float qz0, float qw0,
                                        PxActorHandle actor1, float px1, float py1, float pz1, float qx1, float qy1, float qz1, float qw1);

PxJointHandle physx_create_revolute_joint(PxPhysicsHandle physics,
                                           PxActorHandle actor0, float px0, float py0, float pz0, float qx0, float qy0, float qz0, float qw0,
                                           PxActorHandle actor1, float px1, float py1, float pz1, float qx1, float qy1, float qz1, float qw1);

PxJointHandle physx_create_spherical_joint(PxPhysicsHandle physics,
                                            PxActorHandle actor0, float px0, float py0, float pz0, float qx0, float qy0, float qz0, float qw0,
                                            PxActorHandle actor1, float px1, float py1, float pz1, float qx1, float qy1, float qz1, float qw1);

PxJointHandle physx_create_prismatic_joint(PxPhysicsHandle physics,
                                            PxActorHandle actor0, float px0, float py0, float pz0, float qx0, float qy0, float qz0, float qw0,
                                            PxActorHandle actor1, float px1, float py1, float pz1, float qx1, float qy1, float qz1, float qw1);

PxJointHandle physx_create_distance_joint(PxPhysicsHandle physics,
                                           PxActorHandle actor0, float px0, float py0, float pz0, float qx0, float qy0, float qz0, float qw0,
                                           PxActorHandle actor1, float px1, float py1, float pz1, float qx1, float qy1, float qz1, float qw1);

PxJointHandle physx_create_d6_joint(PxPhysicsHandle physics,
                                     PxActorHandle actor0, float px0, float py0, float pz0, float qx0, float qy0, float qz0, float qw0,
                                     PxActorHandle actor1, float px1, float py1, float pz1, float qx1, float qy1, float qz1, float qw1);

void physx_release_joint(PxJointHandle joint);

/* Common joint methods */
int physx_joint_set_break_force(PxJointHandle joint, float force, float torque);
int physx_joint_get_break_force(PxJointHandle joint, float* force, float* torque);
int physx_joint_set_constraint_flags(PxJointHandle joint, uint32_t flags);
int physx_joint_set_constraint_flag(PxJointHandle joint, uint32_t flag, int enabled);
uint32_t physx_joint_get_constraint_flags(PxJointHandle joint);

/* Revolute joint */
int physx_revolute_joint_set_limit(PxJointHandle joint, float lower, float upper, float stiffness, float damping);
float physx_revolute_joint_get_angle(PxJointHandle joint);
float physx_revolute_joint_get_velocity(PxJointHandle joint);
int physx_revolute_joint_set_drive_velocity(PxJointHandle joint, float velocity);
int physx_revolute_joint_set_drive_force_limit(PxJointHandle joint, float limit);

/* Spherical joint */
int physx_spherical_joint_set_limit_cone(PxJointHandle joint, float y_angle, float z_angle, float stiffness, float damping);

/* Prismatic joint */
int physx_prismatic_joint_set_limit(PxJointHandle joint, float lower, float upper, float stiffness, float damping);
float physx_prismatic_joint_get_position(PxJointHandle joint);
float physx_prismatic_joint_get_velocity(PxJointHandle joint);

/* Distance joint */
int physx_distance_joint_set_min_distance(PxJointHandle joint, float distance);
int physx_distance_joint_set_max_distance(PxJointHandle joint, float distance);
int physx_distance_joint_set_spring(PxJointHandle joint, float stiffness, float damping);

/* D6 joint */
int physx_d6_joint_set_motion(PxJointHandle joint, CPxD6Axis axis, CPxD6Motion motion);
int physx_d6_joint_set_drive(PxJointHandle joint, CPxD6Drive drive, const CPxD6JointDrive* d);
int physx_d6_joint_set_drive_position(PxJointHandle joint,
                                       float px, float py, float pz,
                                       float qx, float qy, float qz, float qw);

/*============================================================================
 *  SECTION 8: SIMULATION EVENT CALLBACKS
 *============================================================================*/

/** Callback types */
typedef void (*PhysxContactCallback)(void* userdata,
                                      const CPxContactPairHeader* header,
                                      const CPxContactPair* pairs, int nb_pairs);

typedef void (*PhysxTriggerCallback)(void* userdata,
                                      const CPxTriggerPair* pairs, int nb_pairs);

typedef void (*PhysxSleepCallback)(void* userdata,
                                    const PxActorHandle* actors, int nb_actors,
                                    int is_waking); /* 0=sleep, 1=wake */

typedef void (*PhysxAdvanceCallback)(void* userdata,
                                      const PxActorHandle* actors,
                                      const CPxTransform* poses, int nb_actors);

/** Register callbacks on the scene. Pass NULL to unregister. */
int physx_scene_set_contact_callback(PxSceneHandle scene, PhysxContactCallback cb, void* userdata);
int physx_scene_set_trigger_callback(PxSceneHandle scene, PhysxTriggerCallback cb, void* userdata);
int physx_scene_set_sleep_callback(PxSceneHandle scene, PhysxSleepCallback cb, void* userdata);
int physx_scene_set_advance_callback(PxSceneHandle scene, PhysxAdvanceCallback cb, void* userdata);

/** Register a contact-modify callback (PxContactModifyCallback). Pass NULL to unregister. */
int physx_scene_set_contact_modify_callback(PxSceneHandle scene,
    PhysxContactModifyCallback cb, void* userdata);

/* Contact modify helpers — call these from within the contact modify callback only.
   pairIndex: index into the pairs[] array passed to the callback
   actorIndex: 0 or 1 (the two bodies in the contact pair)
   scale: new inv-mass-scale or inv-inertia-scale value */
int physx_contact_modify_set_inv_mass_scale(int pairIndex, int actorIndex, float scale);
int physx_contact_modify_set_inv_inertia_scale(int pairIndex, int actorIndex, float scale);

/** Extract all contact points from a CPxContactPair. Returns number of points extracted. */
int physx_contact_pair_extract_contacts(const CPxContactPair* pair,
    CPxContactPairPoint* buffer, int bufferSize);

/** Compute linear+angular impulse from contacts (PxRigidBodyExt). */
int physx_actor_compute_linear_angular_impulse(PxActorHandle actor,
    float* lin_x, float* lin_y, float* lin_z,
    float* ang_x, float* ang_y, float* ang_z);

/*============================================================================
 *  SECTION 9: CHARACTER CONTROLLER
 *============================================================================*/

PxControllerMgrHandle physx_create_controller_manager(PxSceneHandle scene);
void physx_release_controller_manager(PxControllerMgrHandle mgr);

PxControllerHandle physx_create_box_controller(PxControllerMgrHandle mgr,
                                                PxPhysicsHandle physics,
                                                float half_height, float half_side_extent,
                                                float px, float py, float pz,
                                                PxMaterialHandle mat);
PxControllerHandle physx_create_capsule_controller(PxControllerMgrHandle mgr,
                                                    PxPhysicsHandle physics,
                                                    float radius, float height,
                                                    float px, float py, float pz,
                                                    PxMaterialHandle mat);
void physx_release_controller(PxControllerHandle ctrl);

/** Get the underlying actor */
PxActorHandle physx_controller_get_actor(PxControllerHandle ctrl);

/** Move the controller. Returns collision flags (bitmask). */
int physx_controller_move(PxControllerHandle ctrl,
                           float dx, float dy, float dz,
                           float min_dist, float dt);

/** Get/set position */
int physx_controller_get_position(PxControllerHandle ctrl, float* x, float* y, float* z);
int physx_controller_set_position(PxControllerHandle ctrl, float x, float y, float z);
int physx_controller_get_foot_position(PxControllerHandle ctrl, float* x, float* y, float* z);
int physx_controller_set_foot_position(PxControllerHandle ctrl, float x, float y, float z);

/** Step offset */
int physx_controller_set_step_offset(PxControllerHandle ctrl, float offset);
/** Slope limit (radians) */
int physx_controller_set_slope_limit(PxControllerHandle ctrl, float limit);

/*============================================================================
 *  SECTION 10: COOKING
 *============================================================================*/

PxCookingHandle physx_create_cooking(void);
void physx_release_cooking(PxCookingHandle cooking);

PxConvexMeshHandle physx_cook_convex_mesh(PxCookingHandle cooking,
                                           const CPxVec3* vertices, int num_vertices,
                                           int* out_error);

PxTriangleMeshHandle physx_cook_triangle_mesh(PxCookingHandle cooking,
                                               const CPxVec3* vertices, int num_vertices,
                                               const uint32_t* indices, int num_indices,
                                               int* out_error);

void physx_release_convex_mesh(PxConvexMeshHandle mesh);
void physx_release_triangle_mesh(PxTriangleMeshHandle mesh);

/** Create shapes from cooked meshes */
PxShapeHandle physx_create_convex_mesh_shape(PxPhysicsHandle physics,
                                              PxConvexMeshHandle mesh,
                                              PxMaterialHandle mat, int is_exclusive);
PxShapeHandle physx_create_triangle_mesh_shape(PxPhysicsHandle physics,
                                                PxTriangleMeshHandle mesh,
                                                PxMaterialHandle mat, int is_exclusive);

/*============================================================================
 *  SECTION 11: VEHICLE (basic)
 *============================================================================*/

/** Placeholder types for vehicle handles */
typedef struct PxVehicleHandle_* PxVehicleHandle;

/** Initialize the vehicle SDK (call once after physx_create_physics) */
int physx_init_vehicle_sdk(PxPhysicsHandle physics);

/** Create a basic 4-wheel drive vehicle */
PxVehicleHandle physx_create_vehicle_4w(PxPhysicsHandle physics,
                                         PxActorHandle chassis_actor);

/** Update vehicle (call after fetch_results, before next simulate) */
int physx_vehicle_update(PxVehicleHandle vehicle, float dt);

/** Set vehicle input */
int physx_vehicle_set_input(PxVehicleHandle vehicle,
                             float throttle, float brake, float handbrake,
                             float steer, int gear);

/** Cleanup */
void physx_release_vehicle(PxVehicleHandle vehicle);
int physx_close_vehicle_sdk(void);

/*============================================================================
 *  SECTION 12: UTILITY
 *============================================================================*/

/** Get global PhysX bounds of an actor */
int physx_actor_get_world_bounds(PxActorHandle actor, CPxBounds3* bounds);

/** Get the number of active actors in the scene */
int physx_scene_get_active_actors(PxSceneHandle scene, int type_flag,
                                   PxActorHandle* buffer, int buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* PHYSX_BRIDGE_H */
