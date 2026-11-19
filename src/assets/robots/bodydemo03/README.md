# Bodydemo03 机器人模型

从机械提供的 `Bodydemo03` 导出包整理而来，本目录保存 URDF 及其引用的网格。

## 文件

```text
bodydemo03/
├── bodydemo03_constants.py   # 模型与网格加载、基础 EntityCfg、腿轮关节选择规则
├── package.xml              # 保留原包名，供 package:// 路径解析使用
├── urdf/bodydemo03.urdf      # 原始 URDF 内容，文件名简化
├── meshes/*.STL             # URDF 引用的全部 17 个网格
├── xmls/bodydemo03.xml       # 转换的机器人模型，已增加机身自由关节
├── xmls/scene_bodydemo03.xml # 地面、灯光及 home 收腿姿态
└── preview.png              # 场景 home 姿态预览
```

源包里的 ROS launch、CMake 文件、CSV、导出日志和空纹理目录不参与本模型的加载。
XML 使用 `../meshes/...` 相对路径，可连同本目录一起移动。

## 打开模型

在项目根目录运行：

```bash
conda activate unitree_rl_mjlab
python scripts/view_model.py src/assets/robots/bodydemo03/xmls/scene_bodydemo03.xml
```

默认加载 `home` 收腿姿态并暂停，便于查看。按空格开始或暂停物理仿真，
按 `R` 或退格恢复到 `home` 并暂停。

场景中的机身高度为 0.192 m，左侧 thigh/calf 为 -0.35/0.7 rad，
右侧对应关节轴向相反，使用 0.35/-0.7 rad。这是展示用初始姿态。
模型带自由机身，关节当前为被动关节；播放后会按照重力和接触继续运动。
静态预览通过暂停实现。

![Bodydemo03 home 姿态](preview.png)

## 使用 C++ simulate 查看

`simulate/config.yaml` 已切换到以下场景预览配置：

```yaml
robot: "bodydemo03"
robot_scene: "src/assets/robots/bodydemo03/xmls/scene_bodydemo03.xml"
initial_keyframe: "home"
start_paused: true
enable_unitree_sdk2_bridge: false
ros2_sensors_config: "ros2_sensors/config/bodydemo03.yaml"
```

`robot` 是机器人名称，实际加载路径由 `robot_scene` 决定。
在项目根目录编译并启动：

```bash
cmake --build simulate/build -j2
./simulate/build/unitree_mujoco
```

启动时加载 `home` 并暂停。空格开始/暂停，退格恢复 `home` 并暂停。
当前配置关闭 Unitree SDK2 桥接、ROS 传感器和弹簧吊带。

## Python 配置入口

与 Go2W 一样，可以从机器人的统一入口取得 `EntityCfg`：

```python
from src.assets.robots import get_bodydemo03_robot_cfg

robot_cfg = get_bodydemo03_robot_cfg()
```

环境中使用 `cfg.scene.entities = {"robot": robot_cfg}` 引用它。
`get_bodydemo03_spec()` 会同时加载 XML 和全部网格资源，供 mjlab 组装场景。
每次调用均返回独立配置和模型。

基础 `EntityCfg` 使用自由机身，初始高度 0.34 m、腿轮关节零位；
执行器在建立训练任务时配置。上面展示场景的 `home` 是另一个收腿姿态。

## 模型信息

- 17 个机器人刚体、16 个腿轮关节、1 个机身自由关节、17 个 STL 网格。
- 四条腿，每条腿具有 hip、thigh、calf 三个旋转关节，以及一个连续轮轴关节。
- `fl`、`fr`、`rl`、`rr` 分别代表左前、右前、左后、右后。
- 原模型中的 `*_foot_link` / `*_foot_joint` 对应轮子和轮轴，保留原名。
- URDF 质量合计：23.47056 kg。

| 部位 | 关节名称模式 | 数量 |
| --- | --- | --- |
| 髋部 | `*_hip_joint` | 4 |
| 大腿 | `*_thigh_joint` | 4 |
| 小腿 | `*_calf_joint` | 4 |
| 轮轴 | `*_foot_joint` | 4 |

## 转换与核对

转换使用项目中的 `scripts/convert_urdf.py` 和 MuJoCo 3.5.0。
脚本已处理原导出包名 `##Bodydemo03.SLDASM` 中的 `#` 字符。

需要重新转换时，可指定一个新的输出文件名：

```bash
python scripts/convert_urdf.py \
  src/assets/robots/bodydemo03/urdf/bodydemo03.urdf \
  --output src/assets/robots/bodydemo03/xmls/bodydemo03_reexport.xml
```

已核对复制前后的 URDF 和全部网格内容一致、XML 能重新加载、整体目录移动后仍能加载。
在零位及另外 10 组关节角下，独立计算的 URDF 正向运动学与 MJCF 一致，
最大位置差约 `7.9e-7 m`。各刚体质量、重心和惯量也已核对。

## 后续训练需要补充的数据

原始 URDF 的 12 个腿部关节均导出了
`lower=0, upper=0, effort=0, velocity=0`；4 个连续轮轴关节没有这些限制值。
MuJoCo 3.5.0 将这些零范围腿部关节导入为不启用位置限位。
后续训练时需要明确腿部活动范围及电机参数。

机器人 XML 在原始导入结果上增加了机身自由关节，保留原来的网格碰撞体。
展示场景启用机器人与地面的碰撞，关闭机器人自身碰撞。
建立训练任务时，再配置执行器和任务所需的接触检测。
