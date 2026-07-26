# PhysX Go — NVIDIA PhysX 3.4 cgo Bindings for Go

Go 语言对 NVIDIA PhysX 3.4 物理引擎的全面 cgo 绑定，支持在 Linux/WSL2 上运行并通过 PVD 在 Windows 上进行可视化调试。

## 项目结构

```
physx-go-demo/
├── go.mod                      # Go 模块定义
├── Makefile                    # 构建脚本（自动处理 release/debug 切换）
├── config.env.example          # 公开配置模板
├── config.env                  # 本地配置（gitignored）
├── physx/                      # PhysX Go 绑定包
│   ├── bridge.h                # C 头文件：C 兼容类型 + 100+ extern "C" 接口
│   ├── bridge.cpp              # C++ 实现
│   ├── physx.go                # cgo 指令，Foundation/Physics 生命周期
│   ├── types.go                # Vec3, Quat, Transform, Bounds3
│   ├── geometry.go             # Box, Sphere, Capsule, Plane
│   ├── material.go             # PxMaterial
│   ├── scene.go                # PxScene + PVD 可视化参数
│   ├── actor.go                # PxRigidDynamic/PxRigidStatic (30+ 方法)
│   ├── shape.go                # PxShape + 触发器 + 过滤数据
│   ├── joint.go                # 6 种关节 + D6
│   ├── scene_query.go          # Raycast/Sweep/Overlap
│   ├── character.go            # 角色控制器 (CCT)
│   ├── cooking.go              # 网格烹饪 (ConvexMesh/TriangleMesh)
│   └── callbacks.go            # 模拟事件回调 + Vehicle 桩
├── cmd/
│   └── main.go                 # 10 个演示程序
└── bin/
    └── physx-demo              # 编译好的独立二进制
```

## 快速开始

### 1. 编译 PhysX SDK

```bash
cd PhysX_3.4/Source/compiler/linux64
sed -i 's/-Werror//g' Makefile.*.mk
sed -i 's/PX_SUPPORT_PVD=0/PX_SUPPORT_PVD=1/g' Makefile.*.mk

# release（生产）:
make release -j$(nproc)

# debug（调试可视化）:
make debug -j$(nproc)
```

### 2. 配置本地路径

```bash
cp config.env.example config.env
```

编辑 `config.env`，设置 PhysX 源码路径和构建类型：

```ini
PHYSX_ROOT=/path/to/your/PhysX-3.4-master
BUILD_TYPE=release    # 或 debug（启用调试可视化）
```

> `config.env` 已被 `.gitignore` 忽略，不会提交到仓库。

### 3. 构建

```bash
make build
# 输出: === Building with BUILD_TYPE=debug (DEBUG) ===
```

Makefile 根据 `BUILD_TYPE` 自动选择正确的库名和编译宏：

| BUILD_TYPE | 动态库后缀 | 编译宏 | 用途 |
|------------|-----------|--------|------|
| `release` | `libPhysX3_x64.so` | `-DNDEBUG` | 生产优化 |
| `debug` | `libPhysX3DEBUG_x64.so` | `-D_DEBUG -DPX_DEBUG=1` | PVD 关节可视化 |

## 运行演示

```bash
make run ARGS=<demo-name>
make test          # 快速测试（raycast）
```

### 物理模拟

| 命令 | Demo | 说明 | PVD |
|------|------|------|-----|
| `sphere` | 下落球体 | 球从 20m 落下，触地反弹 | ✅ |
| `boxes` | 盒子堆叠 | 15 个盒子金字塔倒塌 | ✅ |
| `joints` | 钟摆链 | 4 节铰链，底部受冲量摆动 | ✅ |
| `d6joint` | D6 关节 | Y-平移+Y-扭转的 6-DOF 约束 | ✅ |
| `kinematic` | 运动平台 | 平台上下移动，球体随动 | ✅ |

### 场景查询

| 命令 | Demo | 说明 | PVD |
|------|------|------|-----|
| `raycast` | 射线检测 | 从上方射击，命中地面和障碍物 | ✗ |
| `sweep` | 扫描检测 | 球体横扫场景，返回命中列表 | ✗ |

### 其他

| 命令 | Demo | 说明 | PVD |
|------|------|------|-----|
| `character` | 角色控制器 | 胶囊体向前行走，越过障碍和斜坡 | ✅ |
| `trigger` | 触发器 | 球体穿过 trigger zone | ✗ |
| `cooking` | 网格烘焙 | 四面体+金字塔网格，放入场景 | ✗ |

## PVD 关节可视化

PhysX 默认不发送关节数据到 PVD，需要两步主动开启：

### 1. 启用数据传输

```go
scene.SetPVDFlags(true, true, true)
// → setScenePvdFlag(eTRANSMIT_CONSTRAINTS, true)
```

### 2. 开启界面渲染

```go
scene.SetVisualizationParameter(physx.VisScale, 1.0)              // 总开关
scene.SetVisualizationParameter(physx.VisJointLocalFrames, 2.0)   // 关节坐标系
scene.SetVisualizationParameter(physx.VisJointLimits, 2.0)        // 关节限位弧
```

> **注意**：`VisJointLocalFrames=21`、`VisJointLimits=22`（不是 6 和 7！PhysX 枚举中间有废弃项把值推后了）。

### 3. 使用 Debug 构建

Release 构建不含调试可视化代码，必须在 `config.env` 中设置：
```ini
BUILD_TYPE=debug
```

## API 概览

### 生命周期

```go
foundation := physx.CreateFoundation()
defer foundation.Release()

hostIP := getWindowsHostIP()   // WSL2 网关 IP，或 "" 禁用 PVD
physics := physx.CreatePhysics(foundation, hostIP)
defer physics.Release()
```

### 创建场景和刚体

```go
scene := physx.CreateScene(physics, 4, 0, -9.81, 0)
defer scene.Release()

mat := physx.CreateMaterial(physics, 0.5, 0.5, 0.6)

// 快捷创建
sphere := physx.CreateDynamicSphere(physics, 0, 10, 0, 1.0, mat, 10.0)
ground := physx.CreateStaticPlane(physics, 0, 1, 0, 0, mat)
scene.AddActor(sphere)
scene.AddActor(ground)
```

### 关节

```go
joint := physx.CreateRevoluteJoint(physics, anchor, anchorFrame, body, bodyFrame)
joint.SetRevoluteLimit(-1.5, 1.5, 100, 10)
joint.SetConstraintFlag(physx.JointFlagVisualization, true)
```

### D6 关节

```go
d6 := physx.CreateD6Joint(physics, a0, f0, a1, f1)
d6.SetD6Motion(physx.D6AxisX, physx.D6MotionLocked)
d6.SetD6Motion(physx.D6AxisY, physx.D6MotionFree)
d6.SetD6Drive(physx.D6DriveTwist, physx.D6JointDrive{Stiffness: 100, Damping: 10})
```

### 角色控制器

```go
mgr := physx.CreateControllerManager(scene)
ctrl := mgr.CreateCapsuleController(physics, 0.4, 1.6, 0, 2, 0, mat)
ctrl.SetStepOffset(0.5)
ctrl.Move(dx, dy, dz, 0.001, dt)
```

### 网格烹饪

```go
cooking := physx.CreateCooking()
convex, _ := cooking.CookConvexMesh(vertices)
shape := physx.CreateConvexMeshShape(physics, convex, mat, true)
```

### 场景查询

```go
hits := scene.Raycast(origin, dir, 100, hitFlags, queryFlags, nil, 16)
hits := scene.Sweep(geom, geomType, pose, dir, 20, hitFlags, queryFlags, nil, 16)
```

## PVD 可视化参数参考

| 常量 | 值 | 说明 |
|------|---|------|
| `VisScale` | 0 | 总开关，0=关闭所有可视化 |
| `VisBodyAxes` | 2 | 刚体坐标系 |
| `VisJointLocalFrames` | **21** | 关节局部坐标系 |
| `VisJointLimits` | **22** | 关节限位弧 |
| `VisCollisionShapes` | 13 | 碰撞形状 |

> 枚举值来自 `PxVisualizationParameter.h`。注意 6-20 被 `eDEPRECATED_BODY_JOINT_GROUPS`、contact、collision 等项占用。

## 已包装的 PhysX 子系统

| 子系统 | 接口数量 | 说明 |
|--------|---------|------|
| Foundation | 4 | Foundation/Physics 生命周期，PVD 连接 |
| Scene | 15 | 场景创建、模拟、重力、可视化参数 |
| Actor | 35+ | 动态/静态创建、姿态、速度、力、质量、休眠、运动学 |
| Material | 10 | 动/静摩擦、弹性、组合模式 |
| Shape | 15+ | Box/Sphere/Capsule/ConvexMesh/TriangleMesh、触发器、过滤数据 |
| Joint | 25+ | 6 种关节+D6，限位、驱动、断裂力、可视化标志 |
| Scene Query | 3 | Raycast/Sweep/Overlap |
| Character | 12 | Box/Capsule 控制器、移动、步高、坡度 |
| Cooking | 6 | 凸包/三角网格烘焙 |
| Callbacks | 4 | Contact/Trigger/Sleep/Advance 事件 |

## 库依赖

```
libPhysX3_x64.so              ← 主物理引擎
├── libPhysX3Common_x64.so
├── libPxFoundation_x64.so
├── libPxPvdSDK_x64.so        ← PVD 协议
├── libPhysX3Cooking_x64.so   ← 网格生成
├── libPhysX3CharacterKinematic_x64.so ← 角色控制器
└── libPhysX3Extensions.a     ← 静态库：工厂函数
```

Debug 版本库名加 `DEBUG` 后缀：`libPhysX3DEBUG_x64.so`、`libPhysX3ExtensionsDEBUG.a` 等。
