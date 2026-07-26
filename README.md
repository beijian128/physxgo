# PhysX Go — NVIDIA PhysX 3.4 cgo Bindings for Go

Go 语言对 NVIDIA PhysX 3.4 物理引擎的全面 cgo 绑定，支持 CLI 演示、Web 3D 可视化、PVD 调试。

## 项目结构

```
physx-go-demo/
├── go.mod / Makefile / config.env
├── physx/                        # PhysX Go cgo 绑定包
│   ├── bridge.h / bridge.cpp     # C/C++ 桥接层 (100+ 接口)
│   ├── physx.go                  # Foundation / Physics 生命周期
│   ├── types.go                  # Vec3, Quat, Transform, Bounds3
│   ├── actor.go                  # PxRigidDynamic / PxRigidStatic
│   ├── shape.go                  # PxShape + trigger + filter data
│   ├── material.go               # PxMaterial
│   ├── scene.go                  # PxScene + CCD + 可视化
│   ├── joint.go                  # 6 种关节 + D6
│   ├── scene_query.go            # Raycast / Sweep / Overlap
│   ├── character.go              # 角色控制器 (CCT)
│   ├── cooking.go                # 网格烹饪
│   └── callbacks.go              # Filter Shader + Contact/Trigger/Sleep/ContactModify
├── cmd/
│   ├── main.go                   # CLI 演示 (13 个 demo)
│   └── webdemo/                  # Web 3D 可视化
│       ├── main.go               # HTTP + SSE 推送 + 模拟协程
│       ├── snippets.go           # 忠实还原的 PhysX Snippet 场景
│       └── viewer.html           # Three.js 3D 渲染 (go:embed)
└── bin/
    ├── physx-demo                 # CLI 二进制
    └── physx-webdemo              # Web 可视化二进制
```

## 快速开始

### 1. 编译 PhysX SDK

```bash
cd PhysX_3.4/Source/compiler/linux64
sed -i 's/-Werror//g' Makefile.*.mk
make release -j$(nproc)
```

### 2. 配置

```bash
cp config.env.example config.env
# 编辑 config.env，设置 PHYSX_ROOT 路径
```

### 3. 构建

```bash
make build       # CLI demo
make web         # Web 可视化
```

## CLI 演示

```bash
./bin/physx-demo <name>
```

| 类别 | 命令 | 说明 |
|------|------|------|
| 物理 | `sphere` `boxes` `joints` `d6joint` `kinematic` | 刚体、关节、运动学 |
| 查询 | `raycast` `sweep` | 射线检测、扫描 |
| 碰撞 | `contact` `contact-ccd` `contact-modify` | 接触报告、CCD、接触修改 |
| 其他 | `character` `trigger` `cooking` | 角色控制器、触发器、网格烘焙 |

## Web 3D 可视化

零外部依赖，SSE 推送 + Three.js 60fps 渲染：

```bash
make web
./bin/physx-webdemo
# 浏览器打开 http://localhost:8080
```

### 3 个还原场景

| 场景 | 原版参考 | Actor 数 | 交互 |
|------|----------|----------|------|
| 📦 HelloWorld | `SnippetHelloWorld.cpp` | 277 | 🎯 发射球体 / ➕ 添加堆叠 |
| 🔗 JointChains | `SnippetJoint.cpp` | 16 | 3 条关节链，球体撞击 |
| 💥 ContactReport | `SnippetContactReport.cpp` | 14 | 红色触点可视化 |

### 架构

```
浏览器 (Three.js 60fps)         Go 服务器
┌──────────────────┐    SSE     ┌──────────────────────┐
│ EventSource      │←──20Hz ───│ broadcastSSE()       │
│ latestState      │            │ runSimulation()      │
│ render() 纯渲染   │            │ (LockOSThread)       │
│                  │            │                      │
│ switchScene()    │───POST───→│ /api/scene           │
│ fireProjectile() │───POST───→│ /api/fire            │
└──────────────────┘            └──────────────────────┘
```

## API 概览

### 生命周期 + 场景

```go
foundation := physx.CreateFoundation()
physics := physx.CreatePhysics(foundation, "") // "" = 无 PVD
scene := physx.CreateScene(physics, 2, 0, -9.81, 0)
mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.6)

sphere := physx.CreateDynamicSphere(physics, 0, 10, 0, 1.0, mat, 10.0)
ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
scene.AddActor(sphere)
scene.AddActor(ground)
```

### 碰撞回调 (Filter Shader + Contact)

```go
// 必须设置 filter shader 才能收到 contact 事件
scene.SetFilterShader(func(attr0 uint32, fd0 *physx.FilterData,
    attr1 uint32, fd1 *physx.FilterData) (uint32, uint32) {
    pf := uint32(physx.PairFlagSolveContact |
        physx.PairFlagDetectDiscreteContact |
        physx.PairFlagNotifyTouchFound)
    return pf, physx.FilterFlagDefault
})

scene.SetContactCallback(func(header physx.ContactPairHeader,
    pairs []physx.ContactPair) {
    for _, cp := range pairs {
        fmt.Printf("contact at (%.2f,%.2f,%.2f)\n",
            cp.ContactPoint.X, cp.ContactPoint.Y, cp.ContactPoint.Z)
    }
})
```

### 关节 + D6

```go
joint := physx.CreateRevoluteJoint(physics, a0, f0, a1, f1)
joint.SetRevoluteLimit(-1.5, 1.5, 100, 10)

d6 := physx.CreateD6Joint(physics, a0, f0, a1, f1)
d6.SetD6Motion(physx.D6AxisX, physx.D6MotionLocked)  // 锁定 X
d6.SetD6Motion(physx.D6AxisY, physx.D6MotionFree)    // Y 自由滑动
d6.SetD6Drive(physx.D6DriveTwist, physx.D6JointDrive{
    Stiffness: 100, Damping: 10, ForceLimit: 100})
```

### 场景查询

```go
hits := scene.Raycast(origin, dir, 100,
    physx.HitFlagPosition|physx.HitFlagDistance,
    physx.QueryFlagStatic|physx.QueryFlagDynamic, nil, 16)
```

### 角色控制器

```go
mgr := physx.CreateControllerManager(scene)
ctrl := mgr.CreateCapsuleController(physics, 0.4, 1.6, 0, 2, 0, mat)
ctrl.Move(dx, dy, dz, 0.001, dt)
```

### 网格烹饪

```go
cooking := physx.CreateCooking()
convex, _ := cooking.CookConvexMesh(vertices)
shape := physx.CreateConvexMeshShape(physics, convex, mat, true)
```

## PVD 可视化

```go
scene.SetPVDFlags(true, true, true)
scene.SetVisualizationParameter(physx.VisScale, 1.0)
scene.SetVisualizationParameter(physx.VisJointLocalFrames, 2.0)  // =21!
scene.SetVisualizationParameter(physx.VisJointLimits, 2.0)        // =22!
```

> ⚠️ `VisJointLocalFrames=21`, `VisJointLimits=22`（不是 6/7 — 枚举中间有废弃项）

## 已包装的子系统

| 子系统 | 接口数 | 说明 |
|--------|-------|------|
| Foundation | 4 | 生命周期, PVD |
| Scene | 20+ | 创建、模拟、重力、CCD、可视化 |
| Actor | 40+ | 创建、姿态、速度、力、质量、休眠、运动学 |
| Material | 10 | 摩擦、弹性、组合模式 |
| Shape | 15+ | Box/Sphere/Capsule/Mesh、触发器、过滤数据 |
| Joint | 25+ | 6 种关节+D6、限位、驱动、断裂力 |
| Scene Query | 3 | Raycast/Sweep/Overlap |
| Character | 12 | Box/Capsule 控制器 |
| Cooking | 6 | 凸包/三角网格 |
| Callbacks | 8 | FilterShader, Contact, Trigger, Sleep, ContactModify |

## 构建类型

| BUILD_TYPE | 动态库 | 用途 |
|------------|--------|------|
| `release` | `libPhysX3_x64.so` | 生产 (无调试检查) |
| `debug` | `libPhysX3DEBUG_x64.so` | PVD 关节可视化 |

```bash
BUILD_TYPE=release make web    # 生产环境
BUILD_TYPE=debug make build    # 调试 + PVD
```

## 库依赖

```
libPhysX3_x64.so
├── libPhysX3Common_x64.so
├── libPxFoundation_x64.so
├── libPxPvdSDK_x64.so
├── libPhysX3Cooking_x64.so
├── libPhysX3CharacterKinematic_x64.so
└── libPhysX3Extensions.a (static)
```
