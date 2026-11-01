# ros2_turtlebot4

TurtleBot4 组 A 的 ROS 2 学习工作空间。目标是从零掌握 ROS 2 节点四件套（话题 / 服务 / 参数 / 启动文件），最终能连通真实 TurtleBot4 机器人。

## 环境

| 项目 | 版本 |
| --- | --- |
| 系统 | Ubuntu 22.04 |
| ROS 2 | Humble |
| 构建工具 | colcon |
| 可视化 | turtlesim / rviz2 |

## 目录结构

```
ros2_ws/
├── README.md
├── .gitignore                # 忽略 build/ install/ log/
└── src/
    └── my_first_nodes/       # 自建功能包
        ├── launch/
        │   └── square.launch.py
        └── my_first_nodes/
            ├── my_talker.py              # 话题发布
            ├── my_listener.py            # 话题订阅
            ├── draw_square.py            # 参数 + 闭环走正方形
            ├── add_server.py             # 服务（AddTwoInts）
            ├── turtle_to_point.py        # Pose -> PointStamped / Path
            └── turtle_point_publisher.py
```

## 构建与运行

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 话题（Topic）

```bash
ros2 run my_first_nodes my_talker        # 发布 my_chatter
ros2 run my_first_nodes my_listener      # 订阅 my_chatter
```

### 参数（Parameter）+ 启动文件（Launch）

```bash
# 一条命令同时起 turtlesim 和画方块的节点
ros2 launch my_first_nodes square.launch.py side_length:=4.0

# 运行时热改参数，不用重启节点
ros2 param set /draw_square side_length 3.0
ros2 param get /draw_square side_length
```

`draw_square` 的 5 个参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `side_length` | 2.0 | 正方形边长（米） |
| `linear_speed` | 1.0 | 直线速度 |
| `angular_speed` | 0.5 | 转向角速度 |
| `dist_tol` | 0.02 | 位移到达判据 |
| `angle_tol` | 0.03 | 角度到达判据 |

参数通过 `add_on_set_parameters_callback` 注册了回调，改完立即生效。

### 服务（Service）

```bash
ros2 run my_first_nodes add_server
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 5}"
# 返回 sum: 8
```

### 可视化数据

```bash
ros2 run my_first_nodes turtle_to_point   # /turtle1/pose -> /turtle_point, /turtle_path
```

## 学习进度

| 日期 | 内容 | 对应提交 |
| --- | --- | --- |
| Day 1 | 创建 my_first_nodes 包，talker / listener 话题通信 | `aaf55fa` |
| Day 2 | draw_square 闭环走正方形（订阅 /turtle1/pose，发布 /turtle1/cmd_vel） | `ba60a63` |
| Day 3 | 参数（5 个参数 + 回调）、square.launch.py、AddTwoInts 服务 | `13011a6` |

### 待办

- [ ] 自写 Action（与 `rotate_absolute` 对照）
- [ ] rviz2 可视化 + TF 坐标系
- [ ] 连通 TurtleBot4 真机

## 常见坑

- **改了代码没生效**：改的是 `src/` 里的源码，`colcon build --symlink-install` 后要重新 `source install/setup.bash`。
- **找不到节点**：忘了 `source install/setup.bash`，或包没编译通过。
- **Action 演示前要关掉 draw_square**：`draw_square` 以 20Hz 持续发布 `cmd_vel`，会打断 `rotate_absolute` 导致目标 ABORTED。
- **push 报 `Password authentication is not supported`**：GitHub 已禁用密码认证，改用 SSH 密钥或 Personal Access Token。

## 使用说明

本仓库是个人学习记录，所有节点均为最小可运行示例，可直接复制复用。
