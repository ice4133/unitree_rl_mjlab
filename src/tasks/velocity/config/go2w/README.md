# Go2W 平地与粗糙地形速度任务

任务名：`Unitree-Go2W-Flat`、`Unitree-Go2W-Rough`。
环境参数、PPO 参数和导出适配均在本目录独立定义，
仅复用公共速度环境模板、MDP 函数、mjlab/RL 实现，以及 Go2W 自己的机器人资产。

当前两个任务均训练统一策略：根据机身坐标系的 `(vx, vy, wz)` 指令协调腿轮，
覆盖前进、后退、左右迈步横移、斜向移动、原地转向和移动中转向。
没有新增技能 ID 或单独的行走／滚动控制器；前后指令允许滚动，明显的横向指令
加入迈步引导。单凭前向速度指令不能要求策略展示固定的 Go2 纯足步态。
以下配置是训练目标，实际能力需要重新训练并回放验收，旧权重不会自动获得横移技能。

## 训练与回放

在项目根目录执行：

```bash
conda activate unitree_rl_mjlab
python scripts/train.py Unitree-Go2W-Flat --env.scene.num-envs=1024
```

默认使用 GPU 0、TensorBoard 日志、5000 次迭代，每 1000 次保存一次。
显存不足可把环境数改为 512。训练结果位于
`logs/rsl_rl/go2w_velocity/<时间>/`，包含 checkpoint 和 `policy.onnx`。

```bash
python scripts/play.py Unitree-Go2W-Flat \
  --checkpoint-file=logs/rsl_rl/go2w_velocity/<时间>/model_<迭代>.pt \
  --num-envs=1
```

## 平地任务的动作和观测约定

- 50 Hz 策略控制：物理步长 0.005 s、decimation=4。
- 前 12 个动作是 FL、FR、RL、RR 各腿 hip/thigh/calf 的位置增量。
  hip 尺度 0.125 rad，其余腿关节 0.25 rad，叠加机器人默认角度。
- 后 4 个动作是 FL、FR、RL、RR 的轮速目标，尺度 5.0 rad/s。
- 速度命令为前后 -0.5～1.0 m/s、横向最终 ±0.5 m/s、转向 ±1.0 rad/s。
  保持 10% 静止命令；其余样本按 30% 纯前后、30% 纯横移、10% 纯转向、
  30% 三轴混合采样，避免纯横移或纯转向在连续随机采样中几乎不出现。
  纯横移样本的速度绝对值至少 0.15 m/s（可用范围更小时随之缩小）。
- 横向课程从 ±0.25 开始，控制步数超过 `1000*24` 时扩至 ±0.35，
  超过 `3000*24` 时扩至 ±0.5。按每轮采样 24 步约对应第 1000、3000 轮；
  修改 `num_steps_per_env` 时应同步换算。课程在回合重置时更新范围，
  随后的指令重采样使用新范围；并行环境数不影响这些步数阈值。
  回放关闭课程，直接使用最终范围。
- Actor 观测依次为机身角速度 3、重力投影 3、速度命令 3、相位 2、
  腿关节位置 12、全部关节速度 16、上次动作 16，总计 55 维。
  轮子的累计转角不进入观测；角速度乘 0.25，关节速度乘 0.05。
- Critic 额外保留机身线速度和轮子接触信息。

`runner.py` 仅为 Go2W 适配导出元数据，记录完整的 16 个动作尺度及对应
关节名称，避免共享导出函数只记录 `joint_pos`；也修正显式轮电机 PD
增益的元数据。PPO、神经网络和 ONNX 导出过程复用 mjlab。

## 奖励与参数来源

动作/观测尺度以及腿轮分开的惩罚权重参考：

- [robot_lab Go2W 环境](https://github.com/fan-ziqi/robot_lab/blob/main/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/unitree_go2w/rough_env_cfg.py)
- [robot_lab Go2W PPO](https://github.com/fan-ziqi/robot_lab/blob/main/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/unitree_go2w/agents/rsl_rl_ppo_cfg.py)

平地任务保留腿轮分开的约束，并补回适合轮足的横移引导，权重为调参起点：

| 奖励 | 权重／处理 |
| --- | --- |
| 水平线速度跟踪 | 3.0；`z_vel_weight=0`，允许迈步所需的竖直运动 |
| 转向速度跟踪 | 1.5 |
| 单独横向速度跟踪 | 1.5；`std=0.25`，随横向指令启用 |
| 横移对角步态 | 0.5；周期 0.6 s、站立占比 0.6、偏移 `[0,0.5,0.5,0]` |
| 横移离地时间／交替支撑 | 1.0；时间目标 0.2 s，使用接触时间而非世界坐标高度 |
| 轮轴方向滑移 | -0.5；只惩罚接触轮的轴向速度平方，允许沿滚动方向运动 |
| 软着陆 | -1e-3；约束刚落地时的冲击力 |
| 腿关节加速度 | -2.5e-7 |
| 轮子角加速度 | -2.5e-9 |
| 腿电机力矩 | -2.5e-5 |
| 腿关节软位置限制 | -5.0 |
| 静止时腿部偏离默认姿态 | -2.0 |
| 动作变化率 | -0.01 |
| 姿态奖励、机身倾斜、roll/pitch 角速度、终止惩罚 | 分别为 0.5、-1.0、-0.05、-200.0 |
| 原 Go2 足端全向滑移、固定世界高度抬脚、全身角动量 | 不启用 |

`commands.py` 负责指令混合采样，`mdp.py` 负责 Go2W 专用奖励。
迈步和单独横向跟踪项按 `clamp((abs(vy)-0.1)/0.1, 0, 1)` 启用：
横向速度不超过 0.1 m/s 时关闭，达到 0.2 m/s 时完全启用。
因此静止、纯前后滚动和纯转向不会被要求按固定节律抬轮。
运动姿态容忍度为 hip 0.3、thigh 0.5、calf 0.75 rad，静止容忍度保持原值。
滑移方向使用每个轮子当前姿态旋转后的实际轮轴，兼容机身转向和髋关节倾斜。

腿部 PD、轮子 `kp=0/kd=2` 和力矩限制保留 `go2w_constants.py` 的设置。
模型接入和短步仿真检查不代表训练已收敛，后续需结合跟踪误差、跌倒率、
各项奖励贡献和轮速/力矩观察训练效果。

## 粗糙地形任务

`env_cfgs.py` 中的 `unitree_go2w_rough_env_cfg()` 与 Flat 复用 Go2W 公共配置。
Rough 保留 12 个腿位置动作和 4 个轮速动作，由地形和速度跟踪目标驱动腿轮协同，
复用横向指令下的迈步引导；前后越障不强制固定迈步节律。

- 使用公共 Rough 预设的平地、上下楼梯、上下坡、不规则地面和波浪地形。
  单级楼梯高度范围为 0～0.1 m；初始地形等级上限配置为 1，按行进距离升降难度。
  地形配置独立深拷贝，避免修改其他机器人的地形。
- 恢复机身下方 1.6×1.0 m、间距 0.1 m 的高度扫描，共 187 个射线值，
  只扫描地形。Actor 在原 55 维末尾追加扫描，变为 242 维；Critic 同样保留扫描。
- 线速度、姿态和横移奖励与 Flat 共用；静止姿态惩罚权重进一步减为 -0.5。
  保留腿轮加速度、腿力矩、动作变化和碰撞终止等约束。
- 不启用将正常轮轴平移视为滑移的 `foot_slip`，也不启用基于世界高度的
  固定抬脚奖励。轮子接触、离地时间和接触力仍供 Critic 使用。
- Flat 和 Rough 均记录 `Episode_Metrics/leg_joint_vel_l2` 和 `wheel_joint_vel_l2`，分别记录
  腿／轮关节速度平方和的每回合步均值，配合速度误差、地形等级和终止原因观察行为。
  关节有运动本身不能证明有效越障，仍需回放检查接触与机身推进。
  `Episode_Metrics/lateral_velocity_error` 同时记录横向速度绝对误差（包含各类指令）。

训练与回放：

```bash
python scripts/train.py Unitree-Go2W-Rough --env.scene.num-envs=512

python scripts/play.py Unitree-Go2W-Rough \
  --checkpoint-file=logs/rsl_rl/go2w_velocity_rough/<时间>/model_<迭代>.pt \
  --num-envs=1
```

PPO 参数复用 Flat，日志单独保存在 `logs/rsl_rl/go2w_velocity_rough/`，
避免自动恢复时混用观测维数不同的 checkpoint。显存不足可降低环境数。
回放时关闭观测噪声、外力扰动和地形课程，使用 5×5 随机地形网格。

评估时应分别观察平地／缓坡上的持续滚动，以及楼梯／不规则地面上的抬轮、落轮和推进；
统计速度误差、跌倒率、通过距离和地形等级。环境接入、短步仿真及短程 PPO 检查
只验证训练链路，不代表已训练出稳定的越障策略。

## 训练与技能验收建议

先训练 Flat 检查横移，再训练 Rough。PPO 保留三层 `(512,256,128)`，
先验证指令和奖励能否学到所需行为；必要时再单独对比更大的网络。
默认训练 5000 轮，扩展后任务若仍在改善，可用 `--agent.max-iterations=10000` 延长。
可以显式指定同观测结构的旧 checkpoint 继续训练，但应与从头训练对比，避免旧的
纯滚动行为主导探索；从头训练不加 resume 参数。续训时环境计步从头开始，课程也重新开始。

回放使用 `--viewer viser`，在 `Twist` 控件中启用手动输入，依次检查：

| 行为 | `(vx, vy, wz)` 示例 | 检查点 |
| --- | --- | --- |
| 静止 | `(0,0,0)` | 站稳、轮子不持续空转 |
| 前进／后退 | `(0.5,0,0)`、`(-0.3,0,0)` | 保持滚动推进，不为领取步态奖励反复抬轮 |
| 左／右横移 | `(0,0.3,0)`、`(0,-0.3,0)` | 朝正确侧迈步，避免四轮贴地横向拖行 |
| 斜向移动 | `(0.4,0.25,0)`、`(0.4,-0.25,0)` | 腿轮协同，跟踪两个方向 |
| 左／右转向 | `(0,0,0.5)`、`(0,0,-0.5)` | 有效转向而非机身乱摆 |
| 移动转向 | `(0.4,0.2,0.4)` | 平移和转向能同时跟踪 |

比较左右横移误差、跌倒率、实际接触与抬轮情况；Rough 还应比较越障通过率。
仅看总奖励或腿关节运动量不能证明技能学会。

## 与 C++ 部署对齐

当前 C++ 部署配置对应 **Flat 的 55 维输入**：
`deploy/robots/go2w/config/policy/velocity/v0/params/deploy.yaml`。
修改动作尺度、默认角度、观测顺序/尺度或控制周期时，应同步修改该文件。
训练后的 ONNX 已包含 Actor 的运行均值归一化。
Rough 的 242 维输入还需要部署侧提供相同定义的高度扫描，不能直接替换此处的平地策略。
独立 `simulate` 的 `scene_go2w.xml` 与 Python 任务地形分别配置；本次只增加训练／回放任务。
动作与观测接口保持原有维度和尺度。当前部署文件仍匹配原来的滚动权重、横向限幅为 0；
完成新 Flat 策略训练和仿真验收、替换 ONNX 后，再将该文件中的
`commands.base_velocity.ranges.lin_vel_y` 改为 `[-0.5, 0.5]`。

```bash
cp logs/rsl_rl/go2w_velocity/<时间>/policy.onnx \
  deploy/robots/go2w/config/policy/velocity/v0/exported/policy.onnx
```
