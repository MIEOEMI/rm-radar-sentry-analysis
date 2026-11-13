# tdt-nav-kit 前端精读：YAstar / KinodynamicAstar / main

## ① 模块与数据流

`yastar.*`：二维栅格 A* + 多层位掩码 + 人工势场代价；`kinodynamicAstar.*` 继承 YAstar 复用地图/掩码/碰撞接口，搜索改为四维状态；`MinimumSnapOsqp/*` 为后端，仅在 main 调用点出现。主链 main.cpp:94-167：灰度图 → `YAstar(w,h,0.05,0,0)`（94）→ `setMap(w,h,map.data)`（95，u_char* 重载 yastar.cpp:404）→ `setMaskNumLayers(4)`（96）→ `setCostField(1,0.5/(0.1+x))`+`initCostMap()`（109-110）→ `search`（115）→ `simplifyPath`+`simplifyPathHypot`（133-134）→ `MinimumSnap::solve`（167）。内部统一 Eigen 行主序 occMap/costMap/nodeMap（yastar.hpp:298-303），像素↔米只靠 `mapping`/`originPos`（cpp:883-891）；掩码独立，`isInObstacle` 取或（cpp:936-943）。两个 `setMap` 语义相反：u_char* 版 `x==0` 判障（cpp:411），int8_t 版 `x!=0` 判障（cpp:425），main 用前者。更麻烦的是 main 自己的采样器按 `map.at<uchar>(...)<128` 判障（main.cpp:479/482），把 1..127 的灰边当障碍，而 `setMap` 只把 `==0` 当障碍（cpp:411-413），同一张 PNG 在两处得到不同占据集。

## ② 关键实现

普通 A*（cpp:802-880）：节点 `Node{cost,estim,parent,speedx,speedy,closed}`（hpp:269-283），`speedx/speedy` 全程无读写（hpp:273 只是设想）。入口只查起终点栅格是否越界（818/822），**不查起点是否在障碍内**，也不查终点；起点节点 cost=0、estim 为像素欧氏距离（826），入堆后按扁平索引 `size_t` 排序，比较器持 nodeMap 引用（hpp:286-295，cpp:813）。代价 `cost += costMap(ny,nx)*costWeight*(i<4?1:1.414)`（864），对角取常数 1.414 而非两格代价均值；能进入的前提取 `costMap<inf`（859），而障碍格在代价场里被置 inf（769/782），所以障碍实际由代价场而非 occMap 拦截。邻接固定 8 邻域（854），不查对角穿越的两个正交格，会贴角穿缝。弹出即判终点（834），沿 parent 回填后 reverse（836-850）；弹出时置 `closed`（852）。

化简：DP 分裂条件为点到弦距离 >threshold 或弦穿障（195-237，209 用 `lineInObsticle`）；`simplifyPathHypot` 向后拉直至碰撞（246-276）；`densifyPath` 用 Bresenham 回填（278-305）。注意 `Tools::simplifyPathDP`（318-344）是另一套只按距离、不做碰撞检查的实现，与成员版语义不同。

Kinodynamic（cpp:126-224）：状态 `[x,y,vx,vy]`（hpp:15），键 = 栅格 + `round(v/velResolution)`（15-20）。控制 `ax,ay∈[-4,4]` 乘 `maxAcc/4`（194-196）共 81 组，时长 `maxTau*step/3`（step=1..3，197-199），每节点 243 后继；`initial` 为真（首节点且初速或初加速度非零）时只沿 `startAcc` 展开 20 个子步（193-199），时长仍按 `maxTau*step/20` 缩放（默认即 0.1s 一档、总跨度 2s），加速度不可选；静止起步则放开为 81 组 × 3 档。转移为常加速度积分（32-37），代价 `(|a|²+timeWeight)*duration`（204）。碰撞按半格密度采样三次曲线（55-59），逐点 `pointInObstacle` 且相邻采样点再走 `lineInObsticle`（64）。解析扩展 `computeShotTraj`（89-101）由两端位置/速度解三次多项式，`checkTrajectory`（39-70）查端点加速度、端点速度与速度内部极值（47-52）。启发 `estimateHeuristic`（103-124）对 ∫a²+timeWeight·t 求导得四次方程（107-110），取正根与下界 `max(1e-3,max|dp|/(0.5maxVel))`（111）的较优者，乘 `heuristicWeight=8`（162/217）。终点门是 `cwiseAbs().maxCoeff()<=12*mapping` 的方框（172），连接失败继续扩展（注释 171）；成功后回溯父链、逐段以 `coef=[p_parent,v_parent,a/2,0]` 采样（179-187），即用二次多项式重放每段的常加速度积分。去重 `Key→下标`，更优才替换（205-206），弹出以 `expanded[key]!=index` 丢弃陈旧项（169）；节点数达 `maxNodes` 立即整体失败返回（209-212）。

## ②b 地图表示与局限

地图只有一层 `TMatrix<float> occMap`，由 8 位灰度图逐像素映射（cpp:411-413），尺寸即图片尺寸 560×300（README.md:94），`mapping` 固定 0.05 m，覆盖 28×15 m。没有膨胀层、没有未知区/通行性分级、没有高度与坡度；`occThs=0.5` 只在 occMap 存浮点概率时才有意义，而 `setMap` 只写 0/1（cpp:411-413、425-431），该阈值形同虚设。安全裕度完全由代价场承担，而代价场只在 `initCostMap` 时按当时的掩码算一次（cpp:760-800）。由此三条硬限制：(1) 5 cm 分辨率决定最小可表达间隙，窄于两格的缝隙会被对角穿越判成可通；(2) 所有碰撞都是点查询（cpp:185-193、kinodynamicAstar.cpp:22-29），车体尺寸只能靠调大 decay 半径间接体现，无法表达与朝向相关的矩形足迹；(3) 地图静态，PNG 读入后不再更新，动态障碍只能走 Mask；掩码确实会进距离场（cpp:664 的 SDF 初始化与 754 的 `getMap` 都调 `isInObstacle`，后者含掩码），所以雷达新障碍也有势场缓冲区，前提是 `initCostMap` 在掩码更新之后重跑（见 ④ 的重算说明）。另外 `occMap` 用 `float` 而源图只有 8 位，精度用不上，代价是每格 4 字节的缓存压力。

## ③ 可配置项与依赖

YAstar：`mapping`（hpp:304，默认 0.05）、`originPos`（305）、`occThs`（309，0.5）、`costWeight`（306，构造赋 1.0，cpp:368）、`setCostField(半径,decay)`（hpp:331-348）、`setMaskNumLayers`（cpp:438，容量 64×layers 位，cpp:84）、`setMaskMapTimed(duration=1.0, maxCount=16)`（hpp:134）。hpp:80 称 `setMapping` 会自动重建代价场，实际只赋值（cpp:384-386）。K 的 `Config`（hpp:17-26）：maxVel/maxAcc 4、maxTau 2.0、velResolution 0.5、timeWeight 1、heuristicWeight 8、sampleTime 0.1、maxNodes 10000；main 把 maxNodes 抬到 150000、sampleTime 降到 0.01（main.cpp:400-401）。依赖 OpenCV（CMakeLists 无开关，必选）、Eigen3、OSQP+OsqpEigen（.gitmodules 两子模块，scripts/setup.sh 编译），`-O2 -Wall`；CMake 未设 `CMAKE_CXX_STANDARD`，代码却用 `__has_include`/`std::span`/`<ranges>`（hpp:15、cpp:7/681），隐式依赖编译器默认标准。OpenCV 分支由 `OPENCV_ALL_HPP` 选择（hpp:68/350）；本机无 OpenCV 头文件，未验证该宏是否由 `opencv.hpp` 定义，存疑。

main 的前后端衔接全部靠手写常量：A* 侧 `setCostField(1.f, 0.5/(0.1+x))`（main.cpp:109）算上 `+1` 基础代价后，d→0 时约 6、一栅格（0.05 m）处约 4.3；化简阈值 `0.1f`（133）；后端 `setOrder(6)`、`setMaxDx(3)`、`setDt(0.1)`、`setTL(10)`（145-149），即 6 未知数五次多项式、10 s 迭代上限；`setCollisionCheckIter(6)`、`setMaxSpeed(3.f)`、`setMaxAcc(1.f)`、`setMaxCorridorRange(2.5f)`、`setCorridorShrink(0.f)`（157-161）。注意前端 A* 的 `costWeight` 未显式设置，用构造默认 1.0（cpp:368），而 `maxVel/maxAcc=4` 的 Kastar 默认值（hpp:18-19）与后端 3/1 不一致：两套动力学上限在同一进程里共存，谁生效取决于走哪条路径。`testKastar` 直接把 A* 的 `lineInObsticle` 拿来复检自己轨迹（main.cpp:417），复用同一碰撞口径，等于自证，不能发现两套模型共同的盲区。

## ④ 代码质量与风险

硬编码：`0.05`/`20.0` 散落 main.cpp:74/83/90/139/179/435/444/470；世界↔图像 Y 翻转手写 `map.rows-1-wy*20`（81-91、404-405、469-474），与 `mapping` 无关联，改分辨率即错；560×300 只在 README.md:94，代码靠 imread 推出。

边界：`lineInObsticle`（157）与 `search`（818/822）用 `>` 而非 `>=`，`x==cols` 能过检查并写 `nodeMap(starty,startx)`（827）越界；`Mask::masked` 明确不安全检查（hpp:41），`isInObstacle` 也不查界（936-943），安全责任全在调用方；`densifyPath` 对空路径 `size()-1` 下溢（281）。

数值：`distanceFromLine`（142-149）投影落段外开根号可出 NaN，使 `max_element` 失效；`cubic` 用 `D==0` 浮点判等（278）、`acos(R/sqrt(-Q³))` 可能越界成 NaN（284）；`quartic` 的 D/E 同样可能 NaN，仅 `isnan` 过滤（251-258）；`quartic` 取 `ys.front()`（235）未判空——目前 `cubic` 三个分支都至少返回一个元素（273-288），故不触发，但属隐式耦合。

失效分支：`search` 失败返回 `{start}` 而非空（820/824/879），与 hpp:232 注释矛盾，main.cpp:128 的 `path.empty()` 永不成立，失败静默流入后端并被当作单点路径化简。`getSDF` 的 cv 分支 `dst=dst*mapping`（560）是 Mat 重新绑定而非原地缩放，`op` 留下像素距离，与 `setCostField` 的米制口径不符（静态推断，需实测）。

重算开销：`search` 加锁后立刻把 `reseted`/`resetedCostMap` 置 false（810-811），于是每次查询都全图重建 nodeMap（947）并重跑 SDF 代价场（806-807、760-800）；main 的显式 `initCostMap`（110）只在首次生效。掩码因此总能生效（这一点与静态阅读的直觉相反），但重规划频率一高，SDF 就是主开销，`initCostMap` 的 `sparse` 分支选型（762）实际决定性能。

`setCostField` 的半径参数在默认路径下不生效：sparse 分支直接对整幅 SDF 施加 `decayFunction(dist)`（762-775），完全不查 `biasMask`，只有 `sparse=false` 才用按半径预计算的偏移表（777-799）。main 传的 `1.f` 半径（main.cpp:109）因此被忽略，衰减作用域是全图而非 1 m——远障也会被加价，与注释所称的膨胀半径语义不符（hpp:178-182）。

并发与正确性：`search` 持 `mapLocker`（809），但 `pointInObsticle`/`lineInObsticle`/`isInObstacle` 不加锁（185-193、936），与 `setMap` 并发即竞争；`getSDF` 锁外读 occMap（763）。比较器读可变 nodeMap（hpp:292-294），入堆后原地降 cost 破坏堆序，无 decrease-key、无弹出过期判断，仅 `closed` 剪枝（861），可能提前闭合致次优；h 为像素距离而 g 为代价单位（826/864/869），`costWeight<1` 时 h 不可采纳。

其他：`Mask::setShape` 在 `reset()` 清空 layerTimes 后再遍历它（cpp:22-25），循环恒空；`pushMask` 的 `laSize+=64`（95）未使用；`setMaskMapTimed` 负 id 复用层名簿记混乱（451-506），硬限 64 层（483）；main.cpp:202 用 `map.cols/3.0f` 校验米制坐标（阈值 186m，等于没校验）；`waitKey(0)`（198/222/385/451）使 1000 次随机测试（476）阻塞；`mt19937 gen(time(0))`（465）不可复现。

## ⑤ 选型结论

可直接摘用：`Mask` 多层位掩码（每格每层一 uint64，cpp:60-121）适合雷达临时障碍重投影；`transformPos` 四重载（883-907）与 `getLength`（307）；Felzenszwalb 精确 EDT 回退（565-745，线性、不依赖 OpenCV）；`setCostField` 的预计算偏移+按权重排序（hpp:331-348）。

必须改：`>=` 边界（157/818/822）、对角拐角检查（854-864）、失败返回空（879）、堆的陈旧项/降键（hpp:286-295+cpp:813）、`getSDF` cv 分支原地缩放（560）、无锁读取（936）、起点障碍检查（818）。上实体车还需补足迹：`pointInObstacle`（22-29）与 A* 均为点模型，代价场只软性推离，无半径/外接框。

换方案条件在代码里：K 的 `maxNodes` 打满即直接返回失败（209-212），而 main 的全场用例把上限从默认 10000 抬到 150000（main.cpp:400），这一改动本身说明作者判断默认容量不够覆盖 28×15 m（是作者设定，不是实测节点数）。`heuristicWeight=8`（hpp:23）配 12 格终点门（172）说明靠强贪心压时间；障碍密集需绕远时贪心先耗尽预算。`maxTau=2.0` 且仅 3 档时长（197-199），单段最远 8m，细长通道时间分辨率不足。控制为各轴独立加速度（194-196），模型是双积分点质量，非差速/阿克曼，换车模要重写扩展与键。doc/Astar.md:24-28 已承认八邻域在欧氏度量下非最优，:84-90 自述的混合策略（A* 全局 + 4m 外交 Kastar）与代码一致，也解释其自承"全局约 2 秒"。结论：前端 A*+DP 化简修完边界与拐角可用；Kastar 只适合 ≤10m 局部重规划；要覆盖全场或要运动学，须换 state lattice/带运动学模型的前端，或按 Astar.md:84 把 Kastar 限制在 4m 窗口。
