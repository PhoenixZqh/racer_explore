# msg_set

#### 公用仓库，存放各类常用消息或自定义消息数据类型，建议关联上作为算法仓库的submodule。

#### 修改msg_set后请在readme中增加修改内容、修改人，并打上对应Tag



## 更新日志

## 2023.08.02

algo_id.yaml文件，不同算法id可实时配置

```xml
AlgoID:
  MOVE_POSITION       : 11
  FOLLOW_GOAL         : 12
  SWARM_FORMATION     : 13
  ATTACK_GOAL         : 14
  BUTTER_FLY          : 15
  SEARCH_GOAL         : 16
  SWARM_EXPLORATION   : 17
  SINGLE_EXPLORATION  : 18
  SINGLE_AVOIDANCE    : 19
  AIR_DROP            : 20
  GATHER              : 22
```

## 2023.10.28

<details open>
<summary><b style="font-size: 24px;">V1.2.2（黄晓巍）</b></summary>
<ul>
<li>更新MultiGoal.msg
<li>增加MultiGoalPoint.msg
<li>删除MultiGoalOld.msg
</li></ul>
</details>


## 2023.11.17

ObjectDetection.msg中新增云台状态roll pitch yaw, 删除bbox.msg和bboxes.msg。

<details open>
<summary><b style="font-size: 24px;">V1.2.4（刘梦杰）</b></summary>
<ul>
<li>增加bbox.msg
<li>增加bboxes.msg
<li>更新DroneOdometry.msg
</li></ul>
</details>


## 2023.11.22

应后端重构需求，更新相关msg（为便于docker镜像拉取，后续统一用小写v表明版本号）

<details open>
<summary><b style="font-size: 24px;">v1.2.5（周智威）</b></summary>
<ul>
<li>更新AssignMission.msg，新增一个后端需要的字段
<li>增加GridMap.msg
<li>增加GridMapMulti.msg
<li>增加Msg.msg
<li>增加PoseTwistAngle.msg
</li></ul>
</details>


## 2023.11.27

<details open>
<summary><b style="font-size: 24px;">v1.2.6（周智威）</b></summary>
<ul>
<li>应算法需求，新增算法ID， SINGLE_AVOIDANCE    : 19
</li></ul>
</details>


## 2023.12.21

<details open>
<summary><b style="font-size: 24px;">v1.2.7（黄晓巍）</b></summary>
<ul>
<li>新增算法ID   
    AIR_DROP            : 20
    GATHER              : 22
</li></ul>
</details>
## 2024.1.2

<details open>
<summary><b style="font-size: 24px;">v1.2.8（黄晓巍）</b></summary>
<ul>
<li>删除UsmDebug.msg
<li>修改UsmDataInfo.msg
</li></ul>
</details>




## 2023.12.28

<details open>
<summary><b style="font-size: 24px;">v1.2.8（lmj）</b></summary>
<ul>
<li>DroneOdometry.msg 更新 新的死亡状态

    uint16 drone_stop=8
    drone 原地不动
</li></ul>
</details>

## 2024.1.05

<details open>
<summary><b style="font-size: 24px;">v1.2.9（lmj）</b></summary>
<ul>
<li>DroneOdometry.msg 更新 新的飞行状态

  ~~uint16 drone_stop=8~~  
  ~~drone 原地不动~~  
  uint16 unable_search=8  
  uint16 unable_track=16  
  uint16 unable_drop=32
</li></ul>
</details>


## 问题清单

1. mavlink_iusc_ctrl_mission_state_t 消息的应答信息中的 seq 是否为当时接收到消息的 seq























































