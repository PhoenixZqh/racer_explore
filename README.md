# 调试记录

1. gazebo仿真环境下深度相机点云朝向不对？
   **在xacro中添加rpy， 不要在map_ros中更改**
2. 障碍物膨胀不能太大

## 节点崩溃问题

```xml
In member function ‘void fast_planner::FrontierFinder::expandFrontier(const Vector3i&)’:
/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/active_perception/src/frontier_finder.cpp:190:40: error: could not convert ‘((std::__shared_ptr_access<fast_planner::SDFMap, __gnu_cxx::_S_atomic, false, false>*)(&((std::__shared_ptr_access<fast_planner::EDTEnvironment, __gnu_cxx::_S_atomic, false, false>*)(&((fast_planner::FrontierFinder*)this)->fast_planner::FrontierFinder::edt_env_))->std::__shared_ptr_access<fast_planner::EDTEnvironment, __gnu_cxx::_S_atomic, false, false>::operator->()->fast_planner::EDTEnvironment::sdf_map_))->std::__shared_ptr_access<fast_planner::SDFMap, __gnu_cxx::_S_atomic, false, false>::operator->()->fast_planner::SDFMap::indexToPos((* & first), pos)’ from ‘void’ to ‘bool’
  190 |     if (!edt_env_->sdf_map_->indexToPos(first, pos))
      |          ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~
      |                                        |
      |                                        void
```


问题分析：



* **未初始化的指针或数据访问** ：
* edt_env_ 或 edt_env_->sdf_map_ 可能未正确初始化，导致访问空指针或未定义内存。
* frontier_flag_ 数组可能未分配或大小不匹配（尤其在 sdf_map_ 重新分配后），导致 frontier_flag_[adr] 访问越界。
* **索引越界** ：
* toadr(first) 或 toadr(nbr) 返回的地址可能超出 frontier_flag_ 的范围（frontier_flag_.size()）。
* 初始索引 first 或邻居索引 nbr 可能不在地图范围内（未通过 inmap 检查），导致 sdf_map_ 的操作（如 indexToPos）访问无效内存。
* **多线程竞争** ：
* 在多无人机系统中，多个 FrontierFinder 实例可能同时访问和修改共享资源（如 frontier_flag_ 或 tmp_frontiers_），导致数据竞争和内存损坏。
* 日志中的 "[ERROR] Larger cost after reallocation" 暗示 sdf_map_ 可能被重新分配，导致 frontier_flag_ 的地址映射失效。
* **内存管理问题** ：
* expanded 向量可能在大型边界簇中无限制增长，耗尽内存或触发重新分配，导致迭代器失效或内存溢出。
* indexToPos 的实现可能在某些边界条件下（如无效索引）未正确处理，间接导致 pos 未初始化或包含非法值。
* **崩溃的具体位置** ：
* 段错误 [0x7efda8157b2f] 表明程序试图访问无效内存地址，可能发生在：

  * frontier_flag_[adr] = 1;（地址越界）。
  * edt_env_->sdf_map_->indexToPos(first, pos);（访问未初始化或无效的 sdf_map_）。
  * expanded.push_back(pos);（pos 未正确初始化）。
    ### 解决办法：

  ### 初始修复中的编译错误

  在第一次修改 expandFrontier 时，我假设 SDFMap::indexToPos 返回 bool（表示转换成功或失败），并添加了检查：

  cpp

  复制

  `<span>if</span><span> (!edt_env_->sdf_map_-></span><span>indexToPos</span><span>(first, pos)) { </span><span></span><span>ROS_ERROR</span><span>(</span><span>"Failed to convert initial index to position: [%d, %d, %d]"</span><span>, first.</span><span>x</span><span>(), first.</span><span>y</span><span>(), first.</span><span>z</span><span>()); </span><span></span><span>return</span><span>; </span>}`

  但你后续提供的编译错误表明，indexToPos 实际返回 void，导致类型不匹配：

  text

  复制

  `<span>/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/active_perception/src/frontier_finder.cpp:190:40: error: could not convert ... from ‘void’ to ‘bool’</span>`

  这说明 indexToPos 不提供成功/失败状态，代码需要调整以适应这一接口，同时保持鲁棒性。

  ### 最终解决方案

  在最终修复的 expandFrontier 函数中，我针对上述问题进行了以下改进，确保解决崩溃并提高鲁棒性：


  1. **初始化检查** ：

  * 添加对 edt_env_, sdf_map_, 和 frontier_flag_ 的检查，防止访问空指针或未初始化数据。
    cpp

    复制

    `<span>if</span><span> (!edt_env_ || !edt_env_->sdf_map_) { </span><span></span><span>ROS_ERROR</span><span>(</span><span>"SDF map is not initialized in expandFrontier"</span><span>); </span><span></span><span>return</span><span>; </span>} <span></span><span>if</span><span> (frontier_flag_.</span><span>empty</span><span>()) { </span><span></span><span>ROS_ERROR</span><span>(</span><span>"frontier_flag_ is not initialized"</span><span>); </span><span></span><span>return</span><span>; </span>}`
  * **效果** ：避免因未初始化的 sdf_map_ 或 frontier_flag_ 导致的段错误。

  1. **索引验证** ：

  * 在调用 toadr 和 indexToPos 前，使用 inmap 检查索引是否在地图范围内。
    cpp

    复制

    `<span>if</span><span> (!</span><span>inmap</span><span>(first)) { </span><span></span><span>ROS_ERROR</span><span>(</span><span>"Initial index out of map: [%d, %d, %d]"</span><span>, first.</span><span>x</span><span>(), first.</span><span>y</span><span>(), first.</span><span>z</span><span>()); </span><span></span><span>return</span><span>; </span>}`
  * 验证 toadr 返回的地址是否在 frontier_flag_ 范围内。
    cpp

    复制

    `<span>int</span><span> adr = </span><span>toadr</span><span>(first); </span><span></span><span>if</span><span> (adr < </span><span>0</span><span> || adr >= </span><span>static_cast</span><span><</span><span>int</span><span>>(frontier_flag_.</span><span>size</span><span>())) { </span><span></span><span>ROS_ERROR</span><span>(</span><span>"Invalid address for initial index: %d, flag_size: %zu"</span><span>, adr, frontier_flag_.</span><span>size</span><span>()); </span><span></span><span>return</span><span>; </span>}`
  * 对邻居索引 nbr 应用相同的检查。
  * **效果** ：防止因无效索引或地址越界（如 frontier_flag_[adr]）导致的崩溃。

  1. **处理 indexToPos 的 void 返回** ：

  * 移除对 indexToPos 返回值的 bool 检查，直接调用并在调用前初始化 pos 为零向量。
    cpp

    复制

    `<span>pos.</span><span>setZero</span><span>(); </span><span>// 初始化 pos 以防转换失败</span><span> </span><span>edt_env_->sdf_map_-></span><span>indexToPos</span><span>(first, pos); </span><span>// indexToPos 返回 void，直接调用</span>`
  * 对邻居索引的转换同样初始化 pos。
  * **效果** ：避免因 pos 未初始化或 indexToPos 失败导致的非法内存访问，同时适配 void 返回类型。

  1. **线程安全** ：

  * 添加 std::mutex frontier_mutex_（在 frontier_finder.h 中声明），并使用 std::lock_guard 保护 tmp_frontiers_ 的写入。
    cpp

    复制

    `<span>{ </span><span></span><span>std::lock_guard<std::mutex> </span><span>lock</span><span>(frontier_mutex_)</span><span>; </span><span>    tmp_frontiers_.</span><span>push_back</span><span>(frontier); </span>}`
  * **效果** ：在多无人机场景下，防止多个线程同时修改 tmp_frontiers_ 导致的数据竞争和内存损坏。注意：frontier_flag_ 的访问未加锁，因其为 char 数组，单字节操作通常安全，但高并发场景需进一步验证。

  1. **资源限制** ：

  * 设置 max_expanded_size（10000）限制 expanded 向量的大小，防止内存溢出。
    cpp

    复制

    `<span>const</span><span></span><span>size_t</span><span> max_expanded_size = </span><span>10000</span><span>; </span><span></span><span>if</span><span> (expanded.</span><span>size</span><span>() >= max_expanded_size) { </span><span></span><span>ROS_WARN</span><span>(</span><span>"Expanded frontier too large, aborting"</span><span>); </span><span></span><span>break</span><span>; </span>}`
  * **效果** ：避免因边界簇过大导致的内存耗尽或向量重新分配问题。

  1. **日志增强** ：

  * 添加详细的错误和警告日志，记录无效索引、地址或初始化失败，便于调试。
    cpp

    复制

    `<span>ROS_ERROR</span><span>(</span><span>"Initial index out of map: [%d, %d, %d]"</span><span>, first.</span><span>x</span><span>(), first.</span><span>y</span><span>(), first.</span><span>z</span><span>()); </span><span></span><span>ROS_WARN</span><span>(</span><span>"Neighbor index out of map: [%d, %d, %d]"</span><span>, nbr.</span><span>x</span><span>(), nbr.</span><span>y</span><span>(), nbr.</span><span>z</span><span>());</span>`
  * **效果** ：帮助定位崩溃的具体原因（如无效索引或越界访问）。

  1. **地图重新分配处理** ：

  * 针对日志中的 "[ERROR] Larger cost after reallocation"，建议在 searchFrontiers 开始时重置 frontier_flag_，确保其与 sdf_map_ 的体素数量一致。
    cpp

    复制

    `<span>void</span><span></span><span>FrontierFinder::resetFrontierFlag</span><span>()</span><span></span><span>{ </span><span></span><span>if</span><span> (edt_env_ && edt_env_->sdf_map_) { </span><span>        frontier_flag_.</span><span>clear</span><span>(); </span><span>        frontier_flag_.</span><span>resize</span><span>(edt_env_->sdf_map_-></span><span>getVoxelNum</span><span>(), </span><span>0</span><span>); </span><span>        std::</span><span>fill</span><span>(frontier_flag_.</span><span>begin</span><span>(), frontier_flag_.</span><span>end</span><span>(), </span><span>0</span><span>); </span>    } }`
  * 在 searchFrontiers 开头调用：resetFrontierFlag();
  * **效果** ：防止因 sdf_map_ 重新分配导致 frontier_flag_ 大小不匹配，从而避免地址越界。
