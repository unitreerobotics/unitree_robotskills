# Unitree G1 描述文件（URDF 与 MJCF）

[English](README.md) | 中文

## 概述

本包包含 [Unitree G1](https://www.unitree.com/g1/) 的通用人形机器人描述文件（URDF 与 MJCF），由 [宇树科技（Unitree Robotics）](https://www.unitree.com/) 开发。

G1 机器人的 MJCF/URDF：

| MJCF/URDF 文件名               | `mode_machine` | 髋部横滚减速比           | 更新状态      | 腿部自由度 | 腰部自由度 | 手臂自由度 | 手部自由度 |
| ----------------------------- | :------------: | :----------------------: | ------------- | :-----: | :-------: | :-----: | :------: |
| `g1_23dof`                    |       1        |           14.5           | Beta          |   6*2   |     1     |   5*2   |    0     |
| `g1_29dof`                    |       2        |           14.5           | Beta          |   6*2   |     3     |   7*2   |    0     |
| `g1_29dof_with_hand`          |       2        |           14.5           | Beta          |   6*2   |     3     |   7*2   |   7*2    |
| `g1_29dof_lock_waist`         |       3        |           14.5           | Beta          |   6*2   |     1     |   7*2   |    0     |
| `g1_23dof_rev_1_0`            |       4        |           22.5           | Up-to-date    |   6*2   |     1     |   5*2   |    0     |
| `g1_29dof_rev_1_0`            |       5        |           22.5           | Up-to-date    |   6*2   |     3     |   7*2   |    0     |
| `g1_29dof_with_hand_rev_1_0`  |       5        |           22.5           | Up-to-date    |   6*2   |     3     |   7*2   |   7*2    |
| `g1_29dof_lock_waist_rev_1_0` |       6        |           22.5           | Up-to-date    |   6*2   |     1     |   7*2   |    0     |
| `g1_dual_arm`                 |       9        |           null           | Up-to-date    |    0    |     0     |   7*2   |    0     |

## 使用 [MuJoCo](https://github.com/google-deepmind/mujoco) 可视化

1. 打开 MuJoCo Viewer

   ```bash
   pip install mujoco
   python -m mujoco.viewer
   ```

2. 将 MJCF/URDF 模型文件（`g1_XXX.xml`/`g1_XXX.urdf`）拖放到 MuJoCo Viewer 中。

## 遥操作说明

`g1_body29_hand14` 修改自 [g1_29dof_with_hand_rev_1_0](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_with_hand_rev_1_0.urdf)。
