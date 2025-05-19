# 使用说明

1. 启动仿真环境注意gazebo.launch中的路径

   ```bash
   roslaunch exploration_manager gazebo.launch
   ```

2. 启动协同搜索算法, 注意脚本中的路径

   ```bash
   ./gazebo.sh
   ```

# 调试记录

## 测试记录

### 真实环境

开发中。。。

### 仿真

| 车数 | 地图尺寸 | 耗时 | 速度                     | 其他                                           |
| ---- | -------- | ---- | ------------------------ | ---------------------------------------------- |
| 1    | 20x20    | 586s | 单车速度最大控制在1m/s   | /                                              |
| 2    | 20x20    | 266s | 单车速度最大控制在1.2m/s | /                                              |
| 3    | 20x20    | 152s | 单车速度最大控制在1.2m/s | 3车可能出现节点崩溃的问题                      |
| 4    | 20x20    | 176s | 单车速度最大控制在1.2m/s | 由于前沿点更新变慢，导致路线分配不佳，效率不高 |

## 问题汇总

### 深度点云方向不对的问题

1. 深度相机位姿发布

   - 仿真中已调试正确
   - 真机时注意，位姿的箭头与小车坐标系的x轴一致
2. 根据实际偏移效果，在processDepthImage函数中添加旋转以矫正

   ```cpp
   // 添加显式旋转校正
   Eigen::AngleAxisd x_correction(M_PI / 2, Eigen::Vector3d::UnitX()); // 绕X轴90度
   Eigen::AngleAxisd y_correction(M_PI / 2, Eigen::Vector3d::UnitY()); // 绕y轴90度
   // Eigen::AngleAxisd z_correction(M_PI / 2, Eigen::Vector3d::UnitZ());  // 绕Z轴90度
   ```

### 没有前沿的问题

通过调整sdf_map的配置参数，当地图20x20时，box的尺寸选择一半：

```xml
<param name="sdf_map/resolution" value="0.1" /> 
<param name="sdf_map/map_size_x" value="$(arg map_size_x)" /> 
<param name="sdf_map/map_size_y" value="$(arg map_size_y)" /> 
<param name="sdf_map/map_size_z" value="$(arg map_size_z)" /> 
<param name="sdf_map/obstacles_inflation" value="0.199" /> 
<param name="sdf_map/local_bound_inflate" value="0.5"/>
<param name="sdf_map/local_map_margin" value="50"/>
<param name="sdf_map/ground_height" value="-0.01"/>
<param name="sdf_map/default_dist" value="0.5"/>
<param name="sdf_map/p_hit" value="0.68"/>
<param name="sdf_map/p_miss" value="0.4"/>
<param name="sdf_map/p_min" value="0.12"/>
<param name="sdf_map/p_max" value="0.99"/>
<param name="sdf_map/p_occ" value="0.70"/>
<param name="sdf_map/min_ray_length" value="0.5"/>
<param name="sdf_map/max_ray_length" value="3.5"/>
<param name="sdf_map/virtual_ceil_height" value="-10"/>
<param name="sdf_map/optimistic" value="true" type="bool"/>
<param name="sdf_map/signed_dist" value="false" type="bool"/>
<param name="sdf_map/box_min_x" value="-10" type="double"/>
<param name="sdf_map/box_min_y" value="-10" type="double"/>
<param name="sdf_map/box_min_z" value="0.01" type="double"/>
<param name="sdf_map/box_max_x" value="10" type="double"/>
<param name="sdf_map/box_max_y" value="10" type="double"/>
<param name="sdf_map/box_max_z" value="0.8" type="double"/>
<param name="sdf_map/no_drone_1" value="$(arg single_expo)" type="bool"/>
```

### 协同互相收不到消息的问题

修改话题映射：

```xml
<remap from="/swarm_expl/drone_state_send" to="/swarm_expl/drone_state" />
<remap from="/swarm_expl/drone_state_recv" to="/swarm_expl/drone_state" />
<remap from="/swarm_expl/pair_opt_send" to="/swarm_expl/pair_opt" />
<remap from="/swarm_expl/pair_opt_recv" to="/swarm_expl/pair_opt" />
<remap from="/swarm_expl/pair_opt_res_send" to="/swarm_expl/pair_opt_res" />
<remap from="/swarm_expl/pair_opt_res_recv" to="/swarm_expl/pair_opt_res" />
<remap from="/swarm_expl/grid_tour_send" to="/swarm_expl/grid_tour" />
<remap from="/swarm_expl/hgrid_send" to="/swarm_expl/hgrid" />
<remap from="/multi_map_manager/chunk_stamps_send" to="/multi_map_manager/chunk_stamps" />
<remap from="/multi_map_manager/chunk_data_send" to="/multi_map_manager/chunk_data" />
<remap from="/multi_map_manager/chunk_stamps_recv" to="/multi_map_manager/chunk_stamps" />
<remap from="/multi_map_manager/chunk_data_recv" to="/multi_map_manager/chunk_data" />
<remap from="/planning/swarm_traj_recv" to="/planning/swarm_traj" />
<remap from="/planning/swarm_traj_send" to="/planning/swarm_traj" />
```

### 迁移到无人车，找路失败的问题

> 测试发现，算法是基于无人机实现的，意味着路径是三维的，而规划出来的路线很喜欢往天空走。

1. 改进视点的生成，强制z为0

   ```cpp
   // 在planExploreMotion函数中
   for (auto &point : ed_->points_) {
       point.z() = 0.0; // 或 0.3 匹配相机高度
   }
   for (auto &avg : ed_->averages_) {
       avg.z() = 0.0;
   }
   ```

2. 改进A*算法，三维->二维

   ```cpp
   // 在Astar::search函数中
   const double fixed_z = 0.3; // 固定z值
   cur_node->position(2) = fixed_z;
   ```

### 小车会被扫描成障碍物

> 在点云生成时，根据小车的位置和yaw、小车尺寸将小车点云扣掉（临时解决方案，最好是根据动态目标来解决）：

```cpp
// 过滤其他无人机的点云
bool is_drone_point = false;
for (const auto &state : drone_states_) {
    int drone_id = state.first;
    if (drone_id == map_->mm_->drone_id_) continue;

    Eigen::Vector4d drone_state = state.second;
    Eigen::Vector3d drone_pos(drone_state[0], drone_state[1], drone_state[2]);
    double yaw = drone_state[3];

    // 计算无人机边界框（考虑 yaw 旋转）
    Eigen::Matrix3d rot;
    rot << cos(yaw), -sin(yaw), 0,
           sin(yaw), cos(yaw), 0,
           0, 0, 1;
    Eigen::Vector3d rel_pos = rot.transpose() * (pt_world - drone_pos);

    // 检查点是否在无人机边界框内
    if (rel_pos.cwiseAbs().maxCoeff() < drone_size_.maxCoeff() / 2) {
        is_drone_point = true;
        break;
    }
}
if (is_drone_point) continue;
```

### 结束搜索返回起始位置可能出现撞车

1. 增加一点idle时间
2. 增加一点两车间的碰撞距离判断

### 车协同时容易出现节点崩溃的问题

`<span style="color:red">`主要在三车测试时出现。问题分析：

1. **未初始化的指针或数据访问**

   - edt_env_ 或 edt_env_->sdf_map_ 可能未正确初始化
   - frontier_flag_ 数组可能未分配或大小不匹配
2. **索引越界**

   - toadr(first) 或 toadr(nbr) 返回的地址可能超出范围
   - 初始索引可能不在地图范围内
3. **多线程竞争**

   - 多个 FrontierFinder 实例可能同时访问共享资源
   - 日志中的 "[ERROR] Larger cost after reallocation" 暗示 sdf_map_ 可能被重新分配
4. **内存管理问题**

   - expanded 向量可能在大型边界簇中无限制增长，耗尽内存或触发重新分配，导致迭代器失效或内存溢出。

`<span style="color:green">`解决思路：

1. **检查未初始化数据**添加对 `edt_env_`, `sdf_map_`, 和 `frontier_flag_` 的检查，防止访问空指针或未初始化数据。
2. **索引范围检查**在调用 `toadr` 和 `indexToPos` 前，使用 `inmap` 检查索引是否在地图范围内。
3. **多线程保护**添加 `std::mutex frontier_mutex_`（在 `frontier_finder.h` 中声明），并使用 `std::lock_guard` 保护 `tmp_frontiers_` 的写入。
4. **内存溢出防护**
   设置 `max_expanded_size`（10000）限制 `expanded` 向量的大小，防止内存溢出。

`<span style="color:orange">`注意： 这样改之后可能会导致前沿更新变慢，还得再看看解决办法

## 注意事项

1. 测试发现障碍物的长度不能大于小车的3倍 （目前主要是控制的不够精细，后续可采用路径跟踪的方法）
2. 小车起始位置注意对应正确
3. n辆小车执行时，参数配置请选择n+1

TODO：

- [ ] 动态目标剔除
- [ ] 偶发崩溃问题
- [ ] 两车间碰撞判断， 目前只依赖位置信息， 可以加上yaw、速度、小车尺寸等已知信息
- [ ] 真车部署

# 算法原理
