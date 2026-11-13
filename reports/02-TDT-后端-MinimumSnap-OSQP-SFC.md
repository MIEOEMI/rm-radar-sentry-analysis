# tdt-nav-kit / MinimumSnapOsqp 逐文件分析

前提：本机无 Eigen 与 osqp（编 sfcSquare.cpp 直接报 `Eigen/Dense` 缺失），故无编译/运行数据；凡性能数字均引自作者 doc/MinimumSnap.md:391-401 的自测表，未复现。行号为文件绝对行号。

## ① 模块与数据流

构建只产出一个可执行 `play`，四个 cpp 全量编入，无 library target、无 install 规则（CMakeLists.txt:19-31），所以"组件"只能源码级拷贝。核心类型两个：`PointPair = std::array<float,4>`，语义 (xmin,ymin,xmax,ymax)（sfcSquare.hpp:21）；`Map` 是 RowMajor 的 `unsigned char` 动态矩阵，约定 0 = 不可通行（sfcSquare.hpp:22,43）。

链路：前端出 `vector<Vector2f>` → `setPath` 逐点建 Item，并把末点强制 `useVxvy=true`、速度置零（minimumSnap.hpp:70-84），即终点速度硬约束为 0 → `solve()`（cpp:217）依次做输入校验(222-235)、单点短路(237-243)、时间分配(245-255)、backend 分派(257-278) → 两条后端汇入同一模板 `iterate()`（cpp:109-170）：求解 → lineDecoder 采样 → lineInObsticle 逐段判碰 → update 改约束 → `SolveOutput{path,corridor,iter,time,success}`（hpp:179-185）。

SFC 只服务 OSQPCorridor：首轮 `getCorridor` 整条生成，且 `corridorInitialized` 只允许进一次（cpp:928-936），此后不重建，只对碰撞段 shrink 或给新插点单独 `getBound`（cpp:955-990）；Close 完全不建走廊，只保留碰撞迭代（cpp:1006-1027）。走廊是轴对齐盒，`postProcess` 把它拆成 x 区间、y 区间两组（cpp:507-514），而 Q 按段分块、x/y 无交叉项，所以拆成两个独立 QP 求解是精确的，不是近似。

## ② 关键实现

阶数与约束。`order` 是系数个数而非阶数（hpp:226 默认 6，main.cpp:145 设 6），`maxdx=3`（hpp:227，main.cpp:146）。generateQ 只在 `i,l ≥ maxdx` 的块上非零（cpp:329-352），即最小化三阶导平方积分；order=2·maxdx 本身自洽，所以这份代码实际做的是 minimum jerk。`continuity = min(_maxdx, order-1) = 3`（cpp:362），getdnx 只产 0..2 阶行（cpp:385-396），只保证位置/速度/加速度连续——被最小化的 jerk 在连接点允许跳变；全仓库没有一处 maxdx=4，名字叫 Minimum Snap 但没有 snap 项。

A 行数 = 2 + n + 1 + 3(n-1) + (n-1) = 5n-1，未知数 6n（cpp:364-370，n 为段数），欠定 n+1 维。行序：起点速度、起点位置、各段末点位置、终点速度、连续性、途经点速度；low/up 靠 `endVelocityRow=numItems+1`、`interiorVelocityRow=A.rows()-(numItems-2)` 手工对位（cpp:810-836、622-645），我按 n=numItems-1 复算，两处都落在 generateA 的写入行上。归一化做了两层：连续性行逐行除自身范数（cpp:420-424），osqpExecute 再对每行连同上下界整体缩放（cpp:848-856），Q 除以最大绝对值（cpp:859，只改目标倍数、不改最优点）。

时间分配。`trapezoidalTimeAllocation`（cpp:281-303）逐段：dist < 0.5v²/a 用 2√(dist/a)，否则 (dist-distThs)/v + 2√(distThs/a)。只看段长、不看相邻段夹角，cpp:291 注释"起到转弯减速作用"与实现不符；hpp:223 建议 v>40、a>10，main.cpp:158-159 却用 3 和 1。

三种 backend 是否真启用。Close 走四参 `_solve`，遇非退化手动走廊直接打印并返回失败（cpp:262-271）；OSQPPath 与 OSQPCorridor 共用一个分支，唯一差别是把 maxCorridorRange 置 0（cpp:274），自动走廊退化为点、等价于等式约束（手动走廊仍生效）；`autoBackend` 硬编码返回 OSQPCorridor（cpp:172-175）。main 只调用 OSQPCorridor(163) 与 Close(309)，OSQPPath 在整个仓库没有调用点（仅 cpp:273-274 与 doc/Usage.md:746 提及），属未验证分支。main.cpp:163 行尾注释写"fallback=闭式求解"，但 cpp:257-278 没有任何回退逻辑。

OSQP 是否每次重建：是，且每轴一次。`OsqpEigen::Solver` 在 axis 循环内部新建（cpp:870），每次都 setHessianMatrix/setGradient/setLinearConstraintsMatrix/setLowerBound/setUpperBound/initSolver（cpp:880-887）；全仓库搜不到 updateHessianMatrix、updateBounds 或 warm start。osqpExecute 每轮迭代调一次（cpp:946），main 的 iter=6 意味着最多 6 轮 × 2 轴 = 12 次完整 setup+求解，无一次复用上次的解或矩阵结构。

SFC 方形走廊。getCorridor 只给中间点求框，首尾退化为点（sfcSquare.cpp:91-115）；`_getBoundSquare2` 从种子格逐圈外扩，上限 `maxRangeInt/2` 圈（:366），某方向受阻后改往反方向扩以维持近方形（:371-454），直到上下或左右都夹住才停（:456-458）；main 的 2.5m/0.05 = 50 格 → 最多 25 圈，每圈检查四边四角，总成本 O(k²)。碰撞侧每段固定采 11 点（cpp:567，与段长、dt 无关），判碰用 DDA（cpp:732-761），非严格模式允许一条采样线上出现 ≤2 个障碍格（cpp:737-746）。

迭代策略。`update` 按轮次奇偶交替（cpp:955-990）：偶数轮 insertMidpoints 在碰撞段中点插 Item、把该段时长对半分（cpp:72-106），新点用 getBound 求框；奇数轮只把碰撞段及其后一段的自动走廊按当前边长的一半 shrink（cpp:972-988）。maxIter=6 时 update 只在 round 0..4 被调用，即最多 3 轮插点、2 轮收缩。Close 的奇数轮是 `times[i] *= 0.8`（cpp:1023）。

## ③ 可配置项与依赖

SolveInput 侧：collisionIteration 默认 2（hpp:171）、maxCorridorRange 默认 float max（hpp:167）、corridorShrink 默认 0（hpp:168）、maxSpeed=5/maxAcc=2（hpp:169-170；hpp:44-45 明示只影响时间分配）、normTime=false、backend 默认 Close（hpp:172）、逐点 useVxvy/corridor/autoCorridor（hpp:50-56）。MinimumSnap 侧：order/maxdx/dt=0.1/timeLimit=1.0s（hpp:226-232）、strictCollision=false（hpp:233）、simplifyDPThs=0.1（hpp:230）。main 把 TL 提到 10s、iter 提到 6（main.cpp:149,157）。

死配置单独点出：`setDt` 只被 `evaluateEquation` 使用（cpp:544-550），而该函数无调用点，所以采样密度永远 11 点/段、dt 完全无效；`simplifyDPThs` 无引用；`setSaveDistance`（sfcSquare.cpp:51）全仓库无调用；`_dilate` 的 par 分支两个 if 代码完全相同、omp 行被注释（:499-508），并行开关是空壳。

依赖：Eigen3、osqp、OsqpEigen（CMakeLists.txt:12-14）；OpenCV 也被 REQUIRED（:11），与 README"OpenCV 仅示例需要"冲突——该 CMakeLists 本身是示例构建，尚可接受，接成库须去掉。C++ 标准根本没设（:4-5 注释掉），而 sfcSquare.cpp:3 include `<ranges>`、:564 用 `std::clamp`（C++17 起），靠 g++ 默认 gnu++17 侥幸通过；且缺 `<algorithm>/<cmath>/<limits>` 直接 include，靠 Eigen/OpenCV 传递进来。

子模块没有随仓库分发。.gitmodules 声明 3rd/osqp 与 3rd/osqp-eigen（:1-6），但 `git ls-tree HEAD` 只有 `3rd/.gitkeep`，无任何 160000 gitlink，`3rd/` 实测 4.0K；.gitignore 的 `3rd/*` 与之吻合，推测被忽略后从未提交。后果是硬性的：`git submodule update --init` 是空操作，setup.sh:31-34 的存在性检查必然 exit 1，一键脚本在这份仓库上跑不通。即便手工补齐，setup.sh 把依赖装到 `3rd/install`（:6,42,50），CMakeLists 也没把该前缀写进 CMAKE_PREFIX_PATH。

## ④ 代码质量与风险

1. 欠定，唯一性靠未验证前提。5n-1 条约束对 6n 个未知数，零空间至少 n+1 维；Q 只覆盖 ≥3 阶系数与 b1（cpp:339/351 的 0.01），b0、b2 零代价。解通常仍唯一，因为动 b0/b2 会破坏位置与连续性约束，但代码既没证明也没检查。信号在 closeSolver：对约化 Hessian 用 `completeOrthogonalDecomposition` 而非 Cholesky/LDLT（cpp:692），等于承认 reducedQ 可能不满秩。一旦某段退化（重复点、T 极小），OSQP（ADMM 路径）与闭式（kernel+COD 取一个解，cpp:683-694）可能选出不同解——这正是 main.cpp:309 双后端对比测试的存在理由；触发概率我无法实测。
2. 归一化没消掉量级。归一化块乘 T^(1-2maxdx) = T^-5（cpp:333），T 小时该块权重可达 1e10，与"压到 [0,1] 改善条件数"的初衷部分抵消；0.01/T² 的 b1 惩罚（cpp:339）相对 T^-5 可忽略。
3. 坐标约定三处不一致。lineInObsticle 用 `(p-origin)/mapping`（cpp:711-714），isOutside 也用减（sfcSquare.cpp:40-42），但 `_getBound/_getBoundSquare2` 用 `(x+origin)/mapping` 且输出 `left*mapping-origin`（:145-146、:228-231、:356-357），valueAt 同样用加（:46-47）。origin 非零时三套约定互相矛盾，main 用 (0,0) 把问题掩盖了。
4. 越界读。`_getBoundSquare2` 直接 `map(top-1,i)`、`map(bottom+1,i)` 不判边界（sfcSquare.cpp:359、:366-378），兜底靠 setSaveDistance 先把四周填黑，而该函数无调用点；Release 下 Eigen 不做断言。
5. 判碰偏松、success 语义弱。固定 11 点采样 + 容忍 ≤2 个障碍格 + strictCollision 默认 false 且 main 从不打开，长段必漏检；达 iter 上限时 iterate 仍返回最后一条轨迹，success 由 `bad.empty()` 决定（cpp:159-163），所以 success=true 只代表"最后一次采样没检出碰撞"。
6. 失败静默。多处 `return {}`（cpp:220、232、254、268、278）只有两处打印，调用方拿到空 path 而非错误码；OSQP 未收敛也只打印状态码后返回空（cpp:888-894）。
7. 插点可能把求解钉死。insertMidpoints 先给新点退化走廊 `{x,y,x,y}` 且 autoCorridor=false（cpp:90-91），随后 update 用 getBound 覆盖并改回 true（cpp:962-969）；若中点落在障碍格或地图外，getBound 返回四个 NaN（sfcSquare.cpp:72-75）或退化点（:359-361），前者让 osqpExecute 的 finite 校验直接失败（cpp:799-802），后者把等式约束锁在障碍内，后续迭代无法脱困。
8. 时间边界。重复路点 dist=0 → 时长 0（cpp:295-296），validTimes 要求每段 >0（cpp:66-68），整次求解失败且路径无去重保护；Close 奇数轮 `times[i]*=0.8`（cpp:1023）只缩不重分配，总时长单调下降。
9. 无速度/加速度硬约束。maxSpeed/maxAcc 只进初始时间分配（hpp:44-45），QP 里没有对应不等式，轨迹可以超限；doc/Usage.md:1210 亦承认"不提供加速度硬约束"。速度约束只作用于被指定 useVxvy 的少数点。
10. 线程安全有缺口。mapMutex 只护 setMap 与 lineInObsticle（cpp:200、703），`_solve` 里的 getCorridor/getBound（cpp:929、963）直接读 sfc.map，与 setMap 并发即数据竞争。
11. 死代码。evaluateEquation（cpp:516）、initXY（:765）、_getBound/_getBoundSquare/_getBoundSquareInplace/_removeRedundant（sfcSquare.cpp:144/296/235/513）、deprecated 单轴 closeSolver（:585）均无调用点。

长尾的代码级成因（按影响排序）：(a) 每轮每轴重建 solver 并 initSolver，无 warm start，setup 次数随段数与轴数线性增长（cpp:870-887）；(b) tol 1e-8 + polish + maxIter 20000（cpp:872-875），在 T^-5 量级差异的 Q 上磨到 1e-8，正是 0.1s 单轴上限（cpp:876）被触发的场景，且触发后以"返回空"收场而非降级；(c) A/Q 每轮先 dense 构造再 sparseView（cpp:806、858-861），dense A 是 (5n-1)×6n，n 大时这一步本身吃掉可观时间；(d) 每轮全量重采样所有段而非只重采碰撞段（cpp:137-156）；(e) timeLimit 检查放在整轮采样之后（cpp:161），单轮成本压不下来。作者自报 1000 次统计中 OSQPCorridor 平均 1.2–6.4ms、最大 179–204ms（doc/MinimumSnap.md:391-401），均值与最大值差两个数量级的形态与 (a)(b) 吻合——但那是作者数据，未复现。

## ⑤ 选型结论

可直接摘用：generateQ/generateA 的积分公式与两层行归一化（cpp:317-435、848-856）逐项核对自洽，order=2·maxdx 的取法可直接搬；endVelocityRow/interiorVelocityRow 行号映射正确；trapezoidalTimeAllocation 作为初值分配够用；lineInObsticle"先判浮点再转整数"的 DDA（cpp:709-730）可直接复用，能挡住 NaN→int 的未定义行为；SolveInput/Item 把走廊、速度、autoCorridor 分层，接口形状值得照抄；x/y 拆轴在轴对齐盒约束下精确。

必须改：① 补唯一性的显式保证（continuity 提到 2·maxdx-1 即 5，或对 reducedQ 判秩并在奇异时退化为最小范数解），否则换机器/换 OSQP 版本轨迹就会变；② origin 符号统一并给非零 origin 加测试；③ 方框扩张加边界钳制，或强制在 setMap 后调 setSaveDistance；④ 采样密度按段长与栅格分辨率自适应，而不是固定 11 点；⑤ getBound 返回 NaN 时不要直接进求解器，应回退为退化点或判定失败上报；⑥ 加真正的 backend 回退链并返回错误码；⑦ CMake 补 CXX_STANDARD 17、补直接 include、把 3rd/install 写进 CMAKE_PREFIX_PATH；⑧ 对外发布必须把 osqp/osqp-eigen 以 gitlink 提交，否则 setup.sh 必挂。

触发换方案的条件（代码里的体现）：段数增大使 dense A 的 6n 列成为瓶颈（cpp:860-861 必须先建 dense 才能 sparseView），或 12 次 initSolver 把单次 solve 推过 100ms（cpp:876 的 setTimeLimit(0.1) 就是作者自划的线），就该换成复用 Hessian 结构 + updateBounds 的增量式 QP，或直接上梯度后端。作者本人在 doc/MinimumSnap.md:405 也承认这套线性 QP"基本已经到头"：障碍进不了约束、时间进不了损失，是方法的结构上限而非实现缺陷。判据因此明确：需求里一旦出现"必须硬保证无碰撞"或"必须满足加速度上限"，就不该在此继续打补丁；反之若只要在稀疏障碍场地给出平滑可跟踪的 2D 轨迹，改完 ①②③④⑤ 后性价比仍然很高——纯 QP、无非线性迭代、单次求解毫秒级，这类实现并不多。
