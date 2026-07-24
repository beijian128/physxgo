# PhysX Go — NVIDIA PhysX 3.4 cgo Bindings for Go

Go 语言对 NVIDIA PhysX 3.4 物理引擎的全面 cgo 绑定，支持在 Linux/WSL2 上运行并通过 PVD 在 Windows 上进行可视化调试。

## 项目结构

```
physx-go-demo/
├── go.mod                      # Go 模块定义
├── physx/                      # PhysX Go 绑定包
│   ├── bridge.h                # C 头文件：C 兼容类型 + 100+ extern "C" 接口
│   ├── bridge.cpp              # C++ 实现：所有 PhysX 桥接函数
│   ├── physx.go                # cgo 指令，Foundation/Physics 生命周期
│   ├── types.go                # Go 类型：Vec3, Quat, Transform, Bounds3
│   ├── geometry.go             # 几何体：Box, Sphere, Capsule, Plane
│   ├── material.go             # PxMaterial 材质
│   ├── scene.go                # PxScene 场景管理/模拟
│   ├── actor.go                # PxRigidDynamic/PxRigidStatic 刚体 (30+ 方法)
│   ├── shape.go                # PxShape 形状 + 触发器
│   ├── joint.go                # 6 种关节 + D6 自由度配置
│   ├── scene_query.go          # Raycast/Sweep/Overlap 场景查询
│   ├── character.go            # 角色控制器 (CCT)
│   ├── cooking.go              # 网格烹饪 (ConvexMesh/TriangleMesh)
│   └── callbacks.go            # 模拟事件回调 + Vehicle 桩
├── cmd/
│   └── main.go                 # 5 个演示程序
└── bin/
    └── physx-demo              # 编译好的独立二进制 (~3.7 MB)
```

## 快速开始

### 1. 编译 PhysX SDK

在 PhysX 源码目录执行：

```bash
cd PhysX_3.4/Source/compiler/linux64
sed -i 's/-Werror//g' Makefile.*.mk           # 修复 GCC 13 兼容性
sed -i 's/PX_SUPPORT_PVD=0/PX_SUPPORT_PVD=1/g' Makefile.*.mk  # 启用 PVD
make release -j$(nproc)
```

### 2. 配置本地路径

```bash
cp config.env.example config.env
# 编辑 config.env，将 PHYSX_ROOT 改为你的 PhysX 源码实际路径
```

`config.env` 已被 `.gitignore` 忽略，不会提交到仓库。

### 3. 构建 Go 项目

```bash
make build
# 或者直接运行演示：
make test
```

### 4. 运行演示

```bash
# 下落球体 + PVD 可视化
make run ARGS=sphere

# 射线检测（无需 PVD）
make run ARGS=raycast

# 或直接调用二进制：
./bin/physx-demo boxes
./bin/physx-demo joints
./bin/physx-demo trigger
```

## API 概览

### 生命周期

```go
foundation := physx.CreateFoundation()
defer foundation.Release()

hostIP := getWindowsHostIP()   // WSL2 gateway IP, or "" to disable PVD
physics := physx.CreatePhysics(foundation, hostIP)
defer physics.Release()
```

### 创建场景和刚体

```go
scene := physx.CreateScene(physics, 4, 0, -9.81, 0)
defer scene.Release()

mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.6)
defer mat.Release()

// 快捷创建：动态球体
sphere := physx.CreateDynamicSphere(physics, 0, 10, 0, 1.0, mat, 10.0)
scene.AddActor(sphere)

// 地面
ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
scene.AddActor(ground)
```

### 模拟循环

```go
dt := float32(1.0 / 60.0)
for i := 0; i < 600; i++ {
    scene.Simulate(dt)
    px, py, pz, _, _, _, _ := sphere.GetGlobalPose()
    fmt.Printf("pos=(%.3f, %.3f, %.3f)\n", px, py, pz)
}
```

### 关节

```go
// 钟摆：固定关节 + 旋转关节链
anchor := physx.CreateRigidStatic(physics, 0, 5, 0, 0, 0, 0, 1)
box := physx.CreateDynamicBox(physics, 0, 4, 0, 0.5, 0.5, 0.5, mat, 1.0)

joint := physx.CreateRevoluteJoint(physics, anchor,
    physx.NewTransform(0, -0.5, 0, 0, 0, 0, 1),
    box,
    physx.NewTransform(0, 0.5, 0, 0, 0, 0, 1))
joint.SetRevoluteLimit(-1.5, 1.5, 100, 10)
```

### 射线检测

```go
origin := physx.NewVec3(0, 10, 0)
direction := physx.NewVec3(0, -1, 0)
hits := scene.Raycast(origin, direction, 100,
    physx.HitFlagPosition|physx.HitFlagDistance,
    physx.QueryFlagStatic|physx.QueryFlagDynamic, nil, 16)

for _, hit := range hits {
    fmt.Printf("pos=%v dist=%.2f\n", hit.Position, hit.Distance)
}
```

### 角色控制器

```go
mgr := physx.CreateControllerManager(scene)
defer mgr.Release()

ctrl := mgr.CreateCapsuleController(physics, 0.5, 1.8, 0, 2, 0, mat)
ctrl.SetStepOffset(0.5)
ctrl.SetSlopeLimit(0.7)

// 移动
flags := ctrl.Move(0, -9.81*dt, 0, 0.001, dt)
x, y, z := ctrl.GetPosition()
```

## 已包装的 PhysX 子系统和接口

| 子系统 | 接口数量 | 说明 |
|--------|---------|------|
| **Foundation** | 4 | 创建/释放 Foundation 和 Physics，PVD 连接 |
| **Scene** | 12 | 场景创建、模拟、重力、Actor 增删、PVD 标志 |
| **Actor** | 35+ | 动态/静态创建、姿态、速度、力/扭矩、质量、休眠、运动学目标、阻尼、锁定标志、世界包围盒 |
| **Material** | 10 | 动/静摩擦、弹性、组合模式 |
| **Shape** | 15+ | Box/Sphere/Capsule/ConvexMesh/TriangleMesh 形状、触发器、过滤数据、接触偏移 |
| **Joint** | 25+ | Fixed/Revolute/Spherical/Prismatic/Distance/D6 关节，限位、驱动、断裂力 |
| **Scene Query** | 3 | Raycast/Sweep/Overlap，可配置命中标志和查询标志 |
| **Character** | 12 | Box/Capsule 控制器管理器、移动、步高、坡度 |
| **Cooking** | 6 | 凸包网格/三角形网格烘焙，形状创建 |
| **Callbacks** | 4 | Contact/Trigger/Sleep/Advance 事件注册 |

## 系统要求

- **OS**: Ubuntu 24.04 WSL2（Windows 主机）
- **Go**: 1.22+
- **PhysX**: 3.4（从 NVIDIA GitHub 编译）
- **编译器**: GCC 13+（需要移除 `-Werror` 和修复 `GuGJKType.h`）
- **PVD**: PhysX Visual Debugger 3.4（在 Windows 上运行）

## 关键设计决策

### 不透明句柄 + 结构体包装

由于 PhysX C++ 类不能跨 cgo 边界传递，每个对象用不透明句柄包装。在 Go 侧用 struct 包装 C 指针：

```go
type ActorHandle struct{ h C.PxActorHandle }
func (a *ActorHandle) GetMass() float32 { ... }
```

### C 兼容数学类型

定义与 PhysX POD 类型布局兼容的 C 结构体，在 bridge.cpp 中做值拷贝转换：

```c
typedef struct { float x, y, z; }    CPxVec3;
typedef struct { float x, y, z, w; } CPxQuat;
typedef struct { CPxQuat q; CPxVec3 p; } CPxTransform;
```

### 从不 memset C++ 结构体

包含虚函数或非 POD 成员的 C++ 对象（如 `PxDefaultAllocator`）绝对不能 `memset`——这会破坏 vtable 指针导致段错误。始终逐个字段初始化。

### rpath 嵌入库路径

`-Wl,-rpath` 将 .so 搜索路径嵌入二进制文件，运行时无需 `LD_LIBRARY_PATH`。

## 库依赖

```
libPhysX3_x64.so              ← 主物理引擎
├── libPhysX3Common_x64.so
├── libPxFoundation_x64.so
├── libPxPvdSDK_x64.so        ← PVD 协议（可选）
├── libPhysX3Cooking_x64.so   ← 网格生成
├── libPhysX3CharacterKinematic_x64.so ← 角色控制器
└── libPhysX3Extensions.a     ← 静态库：工厂函数
```

链接顺序很重要：`-lPhysX3Extensions -lPhysX3_x64 -lPhysX3Common_x64 -lPhysX3CharacterKinematic_x64 -lPhysX3Cooking_x64 -lPxFoundation_x64 -lPxPvdSDK_x64`
