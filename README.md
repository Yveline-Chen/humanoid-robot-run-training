# 人形机器人操场跑道训练实验

基于格物仿真平台（[Unity-RL-Playground](https://github.com/loongOpen/Unity-RL-Playground)），搭建标准操场跑道场景，使用 PPO 强化学习算法训练 3 款人形机器人自主绕跑道完成一圈。

## 项目概览

本项目是具身智能方向的入门实验，核心目标是让机器人在无人工干预的情况下，通过与仿真环境的交互自主学会"沿跑道跑步"任务。

核心任务：
- 人形机器人在弯道处需要自主转向，同时保持直立不摔倒
- 不同机器人（OpenLoong、G1、X02Lite）关节结构不同，需针对性调参
- 训练初期机器人只在起点线重置，导致训练周期过长，易过拟合

## 实验环境

| 项目 | 版本 |
|---|---|
| Unity | 2022.3.62f3 |
| ML-Agents | 3.0.0 |
| 格物仿真平台 | Unity-RL-Playground |
| 训练算法 | PPO（近端策略优化） |

## 场景设计

### 跑道场景

在 Unity 中搭建标准操场跑道，包含以下组件：

- **TrackSurface**：红色跑道地面，8条跑道
- **InnerField**：绿色内场草地
- **LaneLines**：跑道分道线
- **FinishLine**：起跑/终点线
- **WaypointGroup**：24个均匀分布在跑道上的路径点，引导机器人逆时针绕圈

### 三款人形机器人

| 机器人 | 关节数 | 道次 | laneOffset |
|---|---|---|---|
| OpenLoong（青龙） | 12 | 外圈 | +4 |
| X02Lite | 10 | 中圈 | +2 |
| G1（宇树G1） | 12 | 内圈 | 0 |

## 技术方案

### 架构设计

采用**继承扩展**的方式，在不污染原始平台代码的前提下实现跑道专用训练逻辑：

GewuAgent（格物平台原有，仅添加 useCustomReward 开关）
    └── TrackAgent（本项目核心，封装所有跑道逻辑）

`GewuAgent` 只做最小改动，所有跑道逻辑封装在 `TrackAgent` 中，原有场景的训练不受影响。

### 奖励函数设计

| 奖励项 | 数值 | 作用 |
|---|---|---|
| 存活奖励 | +0.01/步 | 鼓励机器人不摔倒 |
| 有效速度奖励 | Speed×0.25 | 奖励朝路径点方向的速度分量 |
| 弯道感知方向奖励 | bodyAlignment×0.3 | 在弯道提前转向，基于下一路径点插值计算期望朝向 |
| 偏航速度补偿 | +0.2×\|ω_y\| | 抵消 GewuAgent 原有的偏离惩罚，允许机器人转弯 |
| 到达路径点 | +3.0 | 鼓励沿跑道推进 |
| 完成一圈 | +20.0 | 完成终极目标后进入 isFinishing 阶段 |
| 越界惩罚 | -2.0 | 跑入内场草地或跑出外场时被惩罚并重置 |
| 直立奖励 | 来自GewuAgent | 保持身体直立（始终生效） |

### 关键技术点

**1. Waypoint 朝向对齐**
`WaypointManager` 在 `Awake()` 中自动让每个路径点朝向下一个，使 `targetWP.right` 在直道和弯道处都能正确指向跑道横向，确保 `laneOffset` 在弯道处不失效。

**2. 随机起点课程学习（Curriculum Learning）**
训练 OpenLoong 时，每次 episode 均从起点线重置，导致训练样本高度集中于跑道前段，训练周期长且可能过拟合。

训练 G1 和 X02Lite 时引入**随机起点策略**：机器人重置时以一定概率随机出生在已抵达过的路径点附近，朝向自动对准下一路径点。这使得神经网络能够从跑道各位置学习，显著提升了样本多样性和训练效率。

**3. 将路径点方向加入观测空间**
在 `CollectObservations` 中将前3个路径点的方向向量（机器人局部坐标系）加入观测，使神经网络能够"预见"前方弯道，提前做出转向的决策。

**4. 有效速度投影**
速度奖励使用 `Vector3.Dot(velocity, toTarget)` 计算朝目标方向的投影速度，防止机器人通过原地抖动或向后跑来骗取速度奖励。

**5. 弯道感知转向奖励**
根据当前路径点与下一路径点的夹角动态计算插值方向，弯道越大期望朝向越超前，引导机器人提前转向而非等到对准当前路径点才转向。

**6. 碰撞事件转发（TriggerForwarder）**
OpenLoong 的根 `ArticulationBody` 与 Collider 不在同一层级，导致 `OnTriggerEnter` 无法直接在 `TrackAgent` 上触发。通过给根关节动态挂载 `TriggerForwarder` 组件，将碰撞事件转发给 `TrackAgent`，解决了越界检测失效的问题。

**7. 针对 G1 的关节映射修正**
G1 的腿部关节步态前馈参数与 OpenLoong 不同，在 `OnActionReceived` 中针对 G1 单独调整了髋关节、膝关节和踝关节的目标角度与刚度，使其步态更稳定。

## 训练流程

### 环境配置

见https://github.com/loongOpen/Unity-RL-Playground

### 启动训练（每次只训练一款机器人）

### 训练 OpenLoong（机器人重置时采用固定起点）
mlagents-learn Assets/Playground/track_config.yaml --run-id=track_v1 --force

### 训练 X02Lite（机器人重置时采用随机起点）
mlagents-learn Assets/Playground/track_config.yaml --run-id=track_v2 --force

### 训练 G1（机器人重置时采用随机起点）
mlagents-learn Assets/Playground/track_config.yaml --run-id=track_v3 --force

### Tensorflow监控训练

```bash
tensorboard --logdir results --port 6006
```

### 加载模型演示

将训练好的 `.onnx` 文件拖入对应机器人的 `B Run Policy` 槽，取消勾选 `Train`，运行 Unity 即可看到三款机器人同时自主绕跑道跑步。

## 训练结果

### 各机器人训练曲线对比

本实验对三款机器人分别进行了独立训练，以下为三组训练曲线的对比分析。

| 指标 | OpenLoong（黑） | X02Lite（蓝） | G1（粉） |
|---|---|---|---|
| 最终 Cumulative Reward（Smoothed） | ~15,342 | ~6,611 | ~2,647 |
| 最终 Episode Length（Smoothed） | ~9,701步 | ~5,066步 | ~1,933步 |
| 训练步数 | 500万步 | 400万步 | 500万步 |
| 训练时长 | 约5.5小时 | 约4.8小时 | 约5.6小时 |
| 推理模型版本 | **5,000,000步** | **4,000,000步** | **4,400,000步** |

### 训练曲线分析

<img width="1925" height="1225" alt="屏幕截图 2026-06-10 055917" src="https://github.com/user-attachments/assets/576c9964-04e1-4e95-aa52-b3cf91085b5c" />

**OpenLoong（track_v1）**

表现最佳。Cumulative Reward 从 0 持续上升至约 15,000，Episode Length 最终达到约 9,701 步，说明机器人能够长时间稳定运行并多次完成绕圈。Value Loss 在波动中维持在合理范围（10~110），Policy Loss 呈现 PPO 算法典型的周期性波动特征。训练过程分为三个阶段：基础运动学习（0~1M步）、路径追踪建立（1M~2M步）、策略持续优化（2M~5M步）。

<img width="1929" height="1231" alt="屏幕截图 2026-06-11 030124" src="https://github.com/user-attachments/assets/1da803f4-ee7e-499a-82f5-7c97b606fda8" />


**X02Lite（track_v2）**

引入随机起点策略后，X02Lite 的 Cumulative Reward 最终稳定在约 6,611，Episode Length 约 5,066 步。与 OpenLoong 相比奖励绝对值较低，但曲线的上升趋势明显，在同等步数内学习效率高于 OpenLoong 的固定起点训练。Value Loss 收敛良好（最终约 38），说明随机起点策略有效提升了样本多样性。

<img width="1939" height="1165" alt="屏幕截图 2026-06-13 142735" src="https://github.com/user-attachments/assets/449439cb-287e-4cb7-bbeb-3788a520adf5" />

**G1（track_v3）**

G1 的训练曲线呈现出与另外两款机器人明显不同的特征。Cumulative Reward 仅达到约 2,647，Episode Length 约 1,933 步，性能显著低于其他两款。更值得注意的是，**Value Loss 呈持续上升趋势**（从约 10 增长至约 200），而非正常的下降或收敛，这说明 Value 网络对未来奖励的预测误差持续增大，是**策略不稳定**的典型信号。

G1 训练困难的可能原因在于其关节刚度和步态前馈参数需要更精细的调节，导致基础步态稳定性不足，神经网络难以在此基础上学习转向。鉴于此，最终选用 **4,400,000 步的中间检查点**作为推理模型，该版本在实测中表现最稳定，能够完成绕圈而不摔倒。

三组训练曲线的 Policy Loss 均表现出周期性波动，这是 PPO 算法"收集-更新"交替迭代机制的体现：每个周期对应一次经验收集与网络更新，Clip 机制限制了每次更新幅度，使训练保持稳定。Value Loss 在奖励发生跳跃时（如机器人首次完成完整一圈）会出现阶段性上升，随后随着网络适应新奖励范围而回落，OpenLoong 和 X02Lite 的曲线中均可观察到此现象。

## 项目结构

```
Assets/
├── Playground/
│   ├── GewuAgent.cs                  ← 格物平台原有
│   └── track_config.yaml             ← 本项目专用训练配置
└── Playground_run_training/
    ├── Playground_runtraining.unity  ← 跑道场景
    ├── TrackAgent.cs                 ← 跑道专用Agent（本项目核心）
    ├── WaypointManager.cs            ← 路径点管理与朝向自动对齐
    ├── openloong_run.onnx            ← OpenLoong 推理模型
    ├── x02lite_run.onnx              ← X02Lite 推理模型
    ├── G1_run.onnx                   ← G1 推理模型（4.4M步检查点）
    └── Tensorflow_data               ← 训练时Tensorflow的监测数据（JSON格式）
```

## 思考与总结

本实验的主要挑战在于**弯道转向**。解决方案是通过 `useCustomReward` 开关关闭原始的速度奖励，引入基于路径点朝向的方向奖励，并添加偏离补偿项抵消对转弯的惩罚。

**随机起点策略**是本实验中效果明显的改进之一。固定起点训练使得机器人大量样本集中在起点附近，对跑道后段的学习不足。随机起点不仅提升了样本多样性，还天然地实现了一种从易到难的课程学习——机器人只在已经抵达过的路径点中重置并继续，避免了盲目的随机初始化。

**G1 的训练困难**揭示了一个重要问题：不同机器人的形态差异（关节刚度、质量分布、自由度配置）对学习的难度有一定的影响，统一的超参数和奖励函数设计并非在对所有机器人的训练中取得相同的效果，针对特定机器人形态的调参是必要的。

## 参考资料

- [格物仿真平台 Unity-RL-Playground](https://github.com/loongOpen/Unity-RL-Playground)

