# Go2W 平地速度任务

任务名：`Unitree-Go2W-Flat`。环境参数、PPO 参数和导出适配均在本目录独立定义，
不导入 Go2、G1 等同级机器人配置。仅复用公共速度环境模板、MDP 函数、mjlab/RL
实现，以及 Go2W 自己的机器人资产；不修改 tracking 或其他机器人的配置。

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

## 动作和观测约定

- 50 Hz 策略控制：物理步长 0.005 s、decimation=4。
- 前 12 个动作是 FL、FR、RL、RR 各腿 hip/thigh/calf 的位置增量。
  hip 尺度 0.125 rad，其余腿关节 0.25 rad，叠加机器人默认角度。
- 后 4 个动作是 FL、FR、RL、RR 的轮速目标，尺度 5.0 rad/s。
- 速度命令为前后 -0.5～1.0 m/s、横向 0、转向 -1.0～1.0 rad/s。
  10% 环境接收静止命令；关闭 Go2 原有的横移速度课程。
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

本任务使用当前工程已有的奖励公式，权重作为迁移起点：

| 奖励 | 权重／处理 |
| --- | --- |
| 线速度跟踪 | 3.0；当前公式同时惩罚竖直速度 |
| 转向速度跟踪 | 1.5 |
| 腿关节加速度 | -2.5e-7 |
| 轮子角加速度 | -2.5e-9 |
| 腿电机力矩 | -2.5e-5 |
| 腿关节软位置限制 | -5.0 |
| 静止时腿部偏离默认姿态 | -2.0 |
| 动作变化率 | -0.01 |
| 姿态奖励、机身倾斜、roll/pitch 角速度、终止惩罚 | 分别为 1.0、-1.0、-0.05、-200.0 |
| 足端步态、抬脚、足端滑移、软着陆、全身角动量 | 移除 |

腿部 PD、轮子 `kp=0/kd=2` 和力矩限制保留 `go2w_constants.py` 的设置。
模型接入和短步仿真检查不代表训练已收敛，后续需结合跟踪误差、跌倒率、
各项奖励贡献和轮速/力矩观察训练效果。

## 与 C++ 部署对齐

部署配置：`deploy/robots/go2w/config/policy/velocity/v0/params/deploy.yaml`。
修改动作尺度、默认角度、观测顺序/尺度或控制周期时，应同步修改该文件。
训练后的 ONNX 已包含 Actor 的运行均值归一化。

```bash
cp logs/rsl_rl/go2w_velocity/<时间>/policy.onnx \
  deploy/robots/go2w/config/policy/velocity/v0/exported/policy.onnx
```
