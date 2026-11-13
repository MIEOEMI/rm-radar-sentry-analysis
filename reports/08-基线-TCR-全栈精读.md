# TCR-RM2026 雷达站源码精读报告：无线电最小接入与上游许可歧义

## 一、范围、证据约定与必须澄清的上游混淆

目标仓库实存于 `tdt-repos/TCR-RM2026-Radar-OpenSource/`（含 `.git`），本报告逐文件读的是发布树源码，不是 README 内容的转述；每条结论后以 `文件:行号` 标注可核对位置。为便于标注，约定路径缩写：`R/`=`radar/CombatRadarSdr/`，`V/`=`ground_vision/`，`D/`=`docs/`，`U/`=`T-DT-Algorithm-2026_RM_sdr/`。部分大文件只做了分段精读，未逐行通读的区间只引已核实部分并在此声明：`V/main.py`（744行，读185–560）、`V/main_event_loop.py`（542行，读55–95/259–278/480–505）、`R/apps/jam_rx_app.py`（1412行，读76–100/320–360/700–770/1150–1290）、`R/apps/match_rx_gui.py`（1145行，读405–435/600–670）、`R/server_comm.py`（507行全读）、`V/driver/radio_bridge.py`（667行，读1–460）、`V/position_fusion.py`（509行，读1–120）、`V/driver/referee/info_wave_bundle.py`（1110行，读1–60）、`V/driver/referee/referee_comm.py`（892行，读695–716）。上述区间之外的实现细节不作断言，以免以推测充当证据。

上游关系必须先辨析清楚，否则会误判许可。TCR 自称的无线电上游是**南京理工大学江阴校区 Combat 战队 CombatRadarSdr2026**（`NOTICE.md:38-39`、`CITATION.md:22-26`、`R/README_UPSTREAM.md:3`）；同一父目录下的 `U/` 是**另一家**上游——东北大学 T-DT 战队、MIT 许可（`U/LICENSE:2-4`、`U/README.md:5,10`），其 `U/gr-robomaster/COPYING:1-2` 为 GPL-3.0。二者不是同一工程，绝不能把 T-DT 的 MIT 当作 Combat 部分的授权依据（`LICENSES/README.md:14`）。可核对的一个技术交集是访问码同源：`R/protocol.py:3-4` 的 `0x2F6F4C74B914492E` 与 `0x16E8D377151C712D`，与 `U/README.md:79-80` 逐字相同，说明双方遵循同一份官方 2-GFSK 规范，但这不构成代码派生关系的证据，也不改变各自的许可状态。

## 二、稳定主线的代码真相：不是三次波形，而是等级驱动换 profile

"一级干扰波→二级干扰波→信息波"在源码里并不是三套并行的波形处理，而是**同一块 AD9361、同一接收通道按干扰等级切换频点配置**。裁判系统 `0x020E` 下发的等级由 `V/driver/referee/referee_comm.py:699-702` 解析进 `JamLevelState`（`V/driver/referee/jam_level_state.py:37-52` 只接受 1..3，并维护 fresh、revision、game_epoch 三个状态量，断线时 `invalidate` 保留旧值但标记不可用），地面端再以 `0xFF+level+0xFE` 三字节包下发给无线电（`V/driver/radio_bridge.py:383-394`、`R/server_comm.py:296-300`）。接收侧常量 `INFO_MODE_LEVEL=3`（`R/apps/jam_rx_app.py:80`），`should_use_info_mode` 的默认分支即 `level>=3`（`:330-335`），`get_receiver_profile` 据此在 `TEAM_LEVEL_TO_JAM_PROFILE`（红蓝各一、二、三级共六项）与 `TEAM_TO_INFO_PROFILE` 之间二选一（`:83-94,338-348`，具体频点见 `R/radio_profiles.py:21-34`）；GUI 把等级 `>=3` 显示为信息波、`>=2` 显示二级、否则一级（`R/apps/match_rx_gui.py:626-637`）。也就是说，Lv.1 与 Lv.2 分别用 `JAM_PROFILES` 解 `0x0A06` 干扰密钥，Lv.3 才切到 `INFO_PROFILES` 解 `0x0A01~0x0A05`，这就是 `D/assets/source/radio_state_machine.dot:10-21` 那张状态机的代码落点。比赛入口被固定为 `--parse-policy default`（`R/apps/match_rx_gui.py:427-434`），`R/start.sh:34-45` 的菜单 1 指向 `match_rx_gui.py`。`CHANGES_FROM_UPSTREAM.md:68` 与 `D/ARCHITECTURE.md:68` 都明确双波 FIR 分路与三板融合**不属于**稳定主线、未经过实战验证，因此下文所有"最小接入"结论都以此单板单通道前提为准。

## 三、空口到裁判发送的完整数据通路

进程边界很清楚：无线电进程独占一块 AD9361 与 TCP server，地面视觉进程是 TCP client（`D/ARCHITECTURE.md:9-16`）；server 绑定 `127.0.0.1:5000`（`R/server_comm.py:58-59,116-119`）。注意 `R/server_comm.py:4` 的注释残留上游词 "gangke_drone client"，真实对端是 ground_vision，读代码时勿被误导（`D/ARCHITECTURE.md:11`）。空口帧沿用裁判帧格式 `A5+len(LE)+seq+CRC8+cmd(LE)+data+CRC16(LE)`（`D/ARCHITECTURE.md:28-31`、`R/protocol.py:51-62`），各命令载荷长度见 `R/protocol.py:18-24`（`0x0A01` 24字节、`0x0A06` 6字节等）。发送侧 `R/server_comm.py:37-45` 的 `SEND_PRIORITY` 把密钥排在最前、坐标次之、辅助信息最后；`queue_latest_command_data`（`:416-457`）按命令类型覆盖而不是无界 FIFO，`_client_gen` 连接代次隔离（`:178-204,215-237`）保证旧连接线程不会关闭或污染新连接——这三点共同构成"最新值优先、旧会话不污染"的设计目标。客户端对称实现在 `V/driver/radio_bridge.py`：`_parse_buffer`（`:408-445`）做 SOF 重同步、CRC8、长度、CRC16 四层校验，`_connect_loop`（`:286-334`）自动重连并递增 generation，`send_jam_level` 与 `send_team_to_sdr`（`:383-404`）发送阵营与等级控制包。地面融合发生在 `V/main_event_loop.py:259-278`，即裁判坐标帧发出前的最后一刻调用 `resolve_with_enemy_aerial`；视觉侧仅在 `:497` 调用一次 `update_vision_tracks(self.tracks)`。**这"一进一出"两行，是视觉链路与无线电链路仅有的两个接触点**，也是判断哪些文件可以完全不动的关键依据。

## 四、最小接入：必须改动或新增的文件

**A 类，无线电侧整包 drop-in，逻辑几乎不必改**：直接引入 `R/` 全目录。`CHANGES_FROM_UPSTREAM.md:32` 证实 `R/phy.py`、`R/protocol_jam_key.py` 与上游逐字节相同，可原样搬运；其余 `protocol.py`、`radio_profiles.py`、`server_comm.py`、`parser/gnuradio_frame_parser.py`、`parser/info_wave_fast_decoder.py`、`launch/message_value_generate.py`、`apps/jam_rx_app.py`、`apps/match_rx_gui.py`、`start.sh` 构成完整收发与解码闭环。真正需要按本机硬件改的只有 `R/radio_profiles.py:21-34` 的频点、带宽、灵敏度与增益，以及 `R/apps/match_rx_gui.py:427-434` 的 `--rx-uri/--rx-gain-db/--team/--server-ip`；`D/RADIO_SAFETY.md:10` 明确要求"不复制他队参数直接发射"，`D/ARCHITECTURE.md:107` 也把该 profile 列为调参点。依赖很轻，仅 `numpy/pyadi-iio/pylibiio` 三项（`R/requirements.txt:5-7`）。

**B 类，地面侧三个可原样嵌入任意 Python 栈的纯标准库模块（本次最重要的结论）**：`V/driver/radio_bridge.py`、`V/position_fusion.py`、`V/driver/referee/jam_level_state.py` 的 import 全部是 `socket/struct/threading/time/dataclasses/typing/collections/enum/math` 一类标准库（`V/driver/radio_bridge.py:12-20`、`V/position_fusion.py:10-18`、`V/driver/referee/jam_level_state.py:3-6`），**不依赖 ROS2、numpy、cv2、海康 MVS，也不 import 任何视觉模块**。它们分别就是 TCP 桥、最新值坐标仲裁、等级新鲜度状态的全部实现，因此对一个已有成熟视觉工程的队伍来说，可以整文件拷贝而不必改动其内部。

**C 类，宿主侧必须改的四处接线**：其一，主入口按 `V/main.py:196-202,266-402` 接线——实例化 `RadioBridgeServer`（`:267-269`）、注册五个回调（`:398-402`）、启动 `_sdr_monitor` 同步线程（`:406-412`），并按"先 team 后 level"的顺序下发控制（`:494-550,514-538`，防止 Lv.3 用旧阵营的信息波频点）。其二，裁判层必须暴露 `0x020E` 等级与阵营，TCR 的落点是 `V/driver/referee/referee_comm.py:699-711,411-416,428`、`serial_comm.py:378`、`serial_protocol.py:69`，用户既有裁判栈需提供等价的等级快照与 `fresh` 判定。其三，`V/config/params.yaml` 增 `position_fusion:`（`:96-101`，含 0.5s 新鲜窗、10s 兜底、0.5s 视觉 watchdog）与 `info_wave_inter_robot:`（`:103-110`）两段配置。其四，打包裁判坐标消息处改调 `resolve_with_enemy_aerial`（`V/main_event_loop.py:267-278`）。**必须强调 `V/main.py` 本身不可照搬**：它 `import rclpy/cv2/driver.hik_camera/interface.core/utils.config`（`V/main.py:98-114`），属于 HKUST 方案加 ROS2 的体系，只能借用其接线逻辑在自有主入口里重写。

**D 类，可选但耦合较强**：`V/driver/referee/info_wave_bundle.py` 负责非坐标信息经 `0x0301/0x02A1` 向我方机器人转发（`CHANGES_FROM_HKUST.md:18`、`V/main.py:268-286`），它依赖 numpy 与 `.serial_comm/.serial_protocol`（`:14-22`），须连同 TCR 裁判串口层一起移植或自行替换；若只做"信息波坐标进裁判"的最小闭环，可先不接。

## 五、可以完全不碰的文件及其边界证据

在"视觉识别与相机链路不动"前提下，以下整类可以零改动。其一，`V/model/` 全部（含 `pfa_yolov5/` 整套 YOLOv5、`yolov12/`、装甲板与数字分类器）、`V/tracker/`、`transform/`、`field/`、`driver/hik_camera/` 与 `MvImport/`、`interface/core.py`、`utils/`。依据即第三节的 `V/main_event_loop.py:497` 与 `:267`：视觉结果只以 `self.tracks` 单点喂入融合缓冲，检测、跟踪、相机、标定内部不存在任何 radio 符号，替换视觉实现也不会波及无线电。其二，`counter_drone/` 全目录与 `V/driver/defense_counter_serial.py`，这是 A7 次数链路、独立进程甚至独立计算平台（`D/ARCHITECTURE.md:74-87`、`D/DEPLOYMENT.md:73-108`），与无线电没有调用关系。其三，`R/` 内的实验件：`coordinate_fusion.py`、`experiments/`、`apps/aux_info_gui.py`、`full_duplex_gui.py`、`rx_only_gui.py`、`tx_only_gui.py`、`radio_monitor.py`、`match_control_simulator.py` 以及 `0x0A07` 兼容槽（`R/protocol.py:13-15`），默认被 `TCR_ENABLE_EXPERIMENTAL_THREE_BOARD` 开关隐藏（`R/README.md:5`、`R/apps/README.md:16-18`），稳定主线不加载。其四，全部 `D/assets/`、`tests/`、`.github/`、`CHANGELOG.md`、`CITATION*`、`LICENSE*`、场地素材与 CI 脚本。**前提失效警告**：若改走双波或三板主线，则第三类不再成立（`D/ARCHITECTURE.md:89-91`），本报告不建议这样做。

## 六、上游许可歧义：具体落在哪些文件

歧义的本质不是"没写清是不是 MIT"，而是**上游没有许可证却被整包再分发，且仓库内部对是否已解决给出两种口径**。

根 `LICENSE:5-12` 声明 Apache-2.0 只覆盖范以恒、徐琰欢有权授权的 TCR 原创贡献与修改，并明确"不 relicense 上游或第三方内容"。而 `R/` 目录下**没有任何** LICENSE、COPYING 或 NOTICE 文件（`find R -iname "*LICENSE*"` 结果为空），全目录唯一的 SPDX 头是 TCR 自己新写的 `R/coordinate_fusion.py:1`（Apache-2.0）。也就是说，无线电整包既不受根 Apache 覆盖，自身也没有授权文本。

`NOTICE.md:49` 自述："截至 2026-08-07，上游 GitHub 仓库页面未见独立 `LICENSE` 文件……不会把 Combat 上游部分标注为 MIT，也不会用 TCR 原创代码许可证覆盖上游内容。若上游后续补充许可证或署名要求，本仓库应同步更新。"`LICENSES/README.md:14` 把该部分记为"上游公开发布，当前未见独立 `LICENSE`"，`:23` 的推荐决策流程进一步把它降格为"按署名和修改声明管理"。上游 README 正文只有"开源……仅供参考"、免责声明与"遵守国家法律法规"（`R/README_UPSTREAM.md:5-6`），**没有任何授权条款**，因此按默认著作权理解，这部分代码的再分发授权是悬空的。

**最直接的矛盾落点**在 `R/README.md:16`：该模块自己的说明仍写着"正式公开前必须解决上游代码许可歧义，具体见 `../../LICENSES/README.md`"，即自认尚未解决；而 `NOTICE.md:49`、`LICENSES/README.md:14,23,26` 已经按"战队发布决定"把它当作已处置，只保留来源链接、原 README、致谢与逐文件修改说明。同一发布包内"必须解决"与"已按决定处理"两种口径并存，这正是许可歧义的所在文件与行号。

再分发规模有明确佐证：`CHANGES_FROM_UPSTREAM.md:11` 记录"13 个同路径文件保持不变、12 个同路径源码/文档发生修改"，`:32` 列明未改者包含 `R/phy.py`、`R/protocol_jam_key.py`、原 README、包初始化文件与依赖清单，即纯上游代码以无授权状态进入公开仓库。`R/README_UPSTREAM.md:1` 只是把原 README 改名归档，`CITATION.md:22-26` 也只是引用来源，二者都不构成授权。审计链同样有缺口：`THIRD_PARTY_NOTICES.md:7-19` 的"已确认来源"表逐行覆盖 HKUST、pfa_yolov5、Ultralytics 配置、MvImport、数据集、图片与场地 PLY，**却没有 `R/` 的任何一行**；`tools/license_audit.sh:22-32` 的 `require_text` 逐项校验 LICENSE、NOTICE、CHANGES_FROM_HKUST、counter_drone 等，但**不检查** `CHANGES_FROM_UPSTREAM.md`、`R/README_UPSTREAM.md` 或 radar 目录，`:74-77` 只防止 README 把混合许可仓库误标为单一许可。因此这个歧义既没有清单条目，也没有 CI 门禁兜底。

**不确定项，禁止外推**：本次只能读到仓库内的声明，无法联网核实 `https://github.com/qianchuan-wys/CombatRadarSdr2026` 当前是否已经补上 LICENSE；`NOTICE.md:49` 的"未见"是 2026-08-07 的快照。若上游后来补了许可证，`R/README.md:16` 与 `NOTICE.md:49` 必须同步重写并补齐 `THIRD_PARTY_NOTICES.md` 条目；若仍未补，则按默认著作权，该部分的公开再分发授权始终悬空——`LICENSES/README.md:28` 自己也声明"本文件是工程合规清单，不构成法律意见"。

## 七、面向"只加无线电"的直接结论与风险

最小改动面可归纳为：1 个整包 `R/`、3 个纯标准库 glue 模块、宿主侧 3 处接线（主入口、`0x020E` 等级暴露、`params.yaml` 两段配置）加 1 处坐标打包调用；视觉检测、相机、跟踪、标定、`counter_drone`、三板实验、素材与文档共 200 余文件可零改动。两个易踩的坑：`R/server_comm.py:4` 的 "gangke_drone" 是上游残留命名（真实对端见 `D/ARCHITECTURE.md:11`）；`V/main.py:109-114` 的 ROS2 与海康依赖不可随 `radio_bridge.py` 一起拷入。联调顺序按 `D/DEPLOYMENT.md:110-118`：先不接天线跑单元测试与菜单 6 模拟器，再确认 `127.0.0.1:5000` 监听、阵营与等级同步，构造帧验证 `0x0A01~0x0A06`，最后才按 `D/RADIO_SAFETY.md:5-12` 做极低功率短距离发射。