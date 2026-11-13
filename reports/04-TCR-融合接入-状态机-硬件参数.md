# TCR-RM2026 雷达源码精读（m4b）

读法：逐个通读 radio_bridge.py(667 行)、position_fusion.py(509)、config/params.yaml(123)、server_comm.py(507)、apps/match_rx_gui.py(1145)、apps/jam_rx_app.py(1412)、docs/ARCHITECTURE.md(116)；为把参数落到定义处，另读 radio_profiles.py、protocol.py、ground_vision/main.py、main_event_loop.py 的相关段。行号均为文件内绝对行；读不到或无法确认处文中直说。

未读范围：parser/info_wave_fast_decoder.py 与 parser/gnuradio_frame_parser.py 的内部实现没有逐行通读，凡涉及这两处算法细节的判断只依据调用点、参数与返回值，不作实现断言；coordinate_fusion.py 的三板选源逻辑同样只看了接口。

## ① 模块与数据流

两个进程，互不 import，靠 TCP 127.0.0.1:5000 串联（server_comm.py:58-59 与 radio_bridge.py:129 各自硬编码同一默认值）。

A 端 ground_vision/main.py：独占工业相机、裁判串口、TCP client、融合缓冲，并持有最终裁判帧组装权（main_event_loop.py:259-314）。B 端 jam_rx_app.py：由 match_rx_gui.py:427-445 以 subprocess 拉起（带 --parent-pid 做孤儿看门狗，jam_rx_app.py:107-118），独占 AD9361 与 TCP server，`listen(1)`（server_comm.py:119），新连接直接顶掉旧连接（215-226）。

上行（无线电→视觉）：rx.rx() 取 IQ（jam_rx_app.py:1073）→ 按 state.rx_mode 分流，jam 模式走 fm_demod + slice_packet_candidates（1129-1141），info 模式默认走 FastInformationWaveDecoder（1239-1276）→ publish_info_frame（722-871）→ partition_coordinate_payload 剔除越界车（775）→ queue_latest_command_data（763-767 或 823-827）→ TCP → radio_bridge._parse_buffer（408-445）→ on_info_positions_event → main.py:312-342 → position_fusion.update_info_wave → 裁判发送线程在真正写帧前调 resolve（main_event_loop.py:267）。

下行（视觉→无线电）：裁判 0x020E 等级（referee_comm.py:701-711）→ _sdr_monitor 先发阵营再发等级（main.py:517-538，注释 514-516 说明顺序原因）→ _send_frame 发 3 字节 FF/FE、FD/FC（radio_bridge.py:383-404）→ _parse_jam_level（server_comm.py:270-307）→ pending_level（jam_rx_app.py:694-698）→ 主循环换 profile。

链路不对称：上行是带 CRC8+CRC16 的裁判帧，下行是无校验的三字节裸包，只靠首尾魔数定界加值域过滤（296-307）。

## ② 关键实现

latest-value 队列：`_pending_send` 为 dict[cmd_id → (data, source_seq, generation)]（server_comm.py:76），同 cmd 新帧直接覆盖旧帧并计数 send_queue_replaced（433-441、454-456）。_send_loop（151-172）每轮 wait_for 0.25 s，再按 SEND_PRIORITY（37-45）扫字典取第一个命中者弹出即发；33-36 注释明确「每发一帧就重估顺序」，新解出的密钥不会压在已排队的辅助帧后。槽位数上限是 cmd 种类数（≤7），内存不增长，真正背压点是同步 sendall（405）与 conn 的 0.5 s 超时（213）。代价是无老化：0A06 每轮都有新帧时，0A02–0A05 长期轮不到。另注意 update_command_data 根本不接受 0A07（343-364），而 SEND_PRIORITY 把 0A07 排在第二位，该槽位在当前代码里没有生产者。

seq 语义：只有 source_seq is None 才用本机 seq_by_cmd 自增（390-393）；jam_rx_app 入队时一律透传空口 seq（763-767、823-827、1213-1217），下游因此会看到重复或跳变 seq，且各 cmd 的 seq 空间互不同步。

generation 断连隔离：server 侧 _client_gen 单调递增（217），新连接在 _client_state_lock 内一次完成「发布新 conn + 递增代次 + 清空 pending」（215-226），注释（221-224）说明这是让生产者不可能把新代次数据塞进被清空的缝隙。_disconnect_client 用 expected_conn + expected_generation 双条件校验（178-204）；send_command_data 在 _socket_send_lock 内二次校验 conn 与 gen（397-404）；queue_latest_command_data 入队后再验，不符则按 (data, seq, gen) 值相等精确回滚自己那一条（442-453）。client 侧同构：_generation（radio_bridge.py:143）、_is_current（261-268）、_disconnect(expected_sock, expected_generation)（220-255）、_send_frame 失败即按代次断开（372-379）。回调事件都带 connection_generation（85-110），上层据此丢弃旧代次回调（main.py:296-302、314-317），并在连/断时都清空信息波缓存（347-349、371-373）。

融合与新鲜度：双缓冲，_info_by_name（无线电，按车名，逐样本带 received_at）与 _vision_by_class（视觉，10 类，整表替换，缺失类被清空，position_fusion.py:266-269）。update_info_wave 只认 5 个地面车名加 enemy_air（181），逐车做 finite 与场地边界校验（192-202），非法车不擦除旧样本（163-165）；同帧共享一个 revision（214-223），但允许 received_at_by_name 给逐车不同年龄（204-209）。

resolve（278-404）优先级链：INFO_CURRENT / INFO_PREVIOUS（age ≤ info_fresh_timeout_s，revision 与上次输出相同则降为 PREVIOUS，328-339）→ GROUND（视觉 age ≤ vision_watchdog_s 且 ground 非空，350-367）→ INFO_STALE（age ≤ info_stale_timeout_s，369-382）→ BLIND_GUESS（384-395）→ INVALID。顺序含义：过期无线电排在新鲜视觉之后，但仍排在新鲜盲猜之前——最坏情况下一条 10 s 前的信息波坐标会盖住当前帧的盲猜点。阵营隔离靠样本内 enemy_class_offset，与当前 offset 不符直接弃用（310-316、429-433）；`_last_output_info_revision` 的键是 (enemy_class_offset, info_name)（328-331），因此换阵营后 PREVIOUS 判定从 -1 重新起算，不会把对方阵营的旧 revision 当成「已发过」。空中目标只有信息波一条来源，无视觉兜底（406-478）。

阈值位置：params.yaml:97-101 是唯一部署真值（fresh 0.5 / stale 10.0 / watchdog 0.5 / diag 1.0），由 main.py:211-216 读入注入；position_fusion.py:86 与 main_event_loop.py:83 的 0.2 只是构造默认值，运行时被 yaml 的 0.5 覆盖。谁直接 new PositionFusionBuffer() 而不传参，拿到的就是 0.2。

单板「一级→二级→信息波」状态机：判定在 should_use_info_mode（jam_rx_app.py:330-335）与 get_receiver_profile（338-348），执行在主循环 1013-1057——pending_level ≠ state.level 时重算 profile，若 rx_mode/频率/带宽有变则重置 stream、live、cycle_filter、off_lock、fast_info_decoder 并重调 configure_receiver（1044-1057）。default 策略 level≥3 才进信息波；onekey_then_info 下 level≥2 即锁死不回退（333-334、624-630）。关键事实：代码里没有任何「一级解码成功才升级」的判定，升级完全由地面端下发的等级驱动，jam_rx_app 只把等级写进 pending_level（694-698）。ARCHITECTURE.md:63-64 的「确认一级成功后再切二级」是流程描述而非代码强制；match_rx_gui.py:55-60、311-325 的 STAGE_LABELS/_set_stage 纯属显示。

adi.Pluto 射频参数：实例化在 jam_rx_app.py:878（OSError 重连路径 1084），射频设置集中在 configure_receiver（438-453）七行：sample_rate(447)、rx_lo(448)、rx_rf_bandwidth(449)、gain_control_mode_chan0(450)、rx_hardwaregain_chan0(451)、rx_enabled_channels=[0](452)、rx_buffer_size(453)。数值来源三处：采样率默认 1 MHz（565）、缓冲 65536（569）、增益 50 dB（568）、AGC 默认 manual（606-607）；中心频率与带宽来自 radio_profiles.py 的 RadioProfile（8-14）与表（21-34），INFO red1 = 433.2 MHz / 540 kHz，JAM red1 = 432.2 MHz / 940 kHz。match_rx_gui.py 只额外传 --rx-buffer-size 65536、--rx-gain-db 50、--rx-agc-mode slow_attack（419-434），不传采样率。解码器参数写死在构造处（665-683）：correlation_threshold 0.45、max_access_errors 24、tail_samples 12000、iq_lowpass_hz 270 kHz、max_header_errors 2。

时延硬门槛：estimated_age = 缓冲时长 + 处理耗时，超过 INFO_MAX_ESTIMATED_AGE_SECONDS = 0.20 就整批丢帧（104、1245-1266），是全链路唯一不可配置的时延预算。

## ③ 可配置项与依赖

yaml 可控：position_fusion 三个超时与诊断间隔（97-101）、info_wave_inter_robot 的 5 Hz/1 Hz、stale 0.5 s、radio_age_guard 200 ms（104-110）、defense_link 全套（82-89）、referee 串口与波特率（76-79）、相机 K/dist（112-117，与 transform 段 50-53 重复）、场地 mesh 与关键点（67-74）、recording（119-123）。

写死在源码：INFO_MAX_ESTIMATED_AGE_SECONDS=0.20（jam_rx_app.py:104）、解码器五参数（665-683）、场地 2800/1500（三处）、SEND_PRIORITY（server_comm.py:37-45）、重连退避与队列上限（match_rx_gui.py:42-52）。

依赖：pyadi-iio、numpy（jam_rx_app.py:25-29 显式 import numpy._core 等私有子模块，实际把版本钉在 numpy 2.x）、tkinter、可选 iio；adi 缺失直接 return 2（556-558）。radio_bridge 与 position_fusion 只用标准库，可脱离 SDR 单测。

## ④ 代码质量与风险

- 常量多处复制：场地边界见 radio_bridge.py:70-71、position_fusion.py:79-80、jam_rx_app.py:95-98；EXPECTED_COMMAND_LENGTHS 见 radio_bridge.py:73-81 与 server_comm.py:23-31，protocol.py:18-24 的 INFO_CMD_LEN 是第三份且不含 0A06/0A07。改帧长必漏改。
- 数值稳定性：jam_rx_app.py:20-21 全局 np.seterr(all='ignore') 并 SIG_IGN 掉 SIGFPE，DSP 除零与溢出被静默；功率仅 +1e-15 兜底（1122）。反过来 position_fusion 是全仓检查最严的模块：_finite_time 拒非有限时间戳（113-117），_metres_to_cm 先校验浮点再转 int，注释（139-141）专门解释不能先 int 再校验（int(-0.1)==0、int(2800.9)==2800）。
- 未验证/死分支：legacy 信息波路径（1288-1353）只有 RADAR_INFO_DECODER=legacy 才走（615）；0A07 解析存在（radio_bridge.py:607-653）但无生产者，ARCHITECTURE.md:91 自认未达实战验证；双波 FIR 只做过离线录包（ARCHITECTURE.md:68）。radio_bridge.py:484-486 那个「已知 cmd 但长度不符仍 return True」的分支，在 431-435 已拦掉的前提下不可达，属防御性死代码。
- 回调持锁：on_connected 在 _connection_transition_lock 内被调用（radio_bridge.py:300-315），回调里做慢操作会阻塞 _disconnect；main.py:347-368 的回调又跨模块调 info_wave_controller.update_context，锁序跨文件。
- 无锁统计：frames_received / errors_crc8 等在接收线程自增、get_stats 无锁读（655-667），仅影响诊断。
- 发送丢帧语义：_send_loop 先 pop 再发（166-172），send 失败只返 False 不重入队，该帧静默丢失；latest-value 下可接受，若依赖 0A06 必达需自补。
- 解析复杂度：_parse_buffer 每轮 buf.find 重新分配 bytes 且假帧只删 1 字节（415-444），畸形流下最坏 O(n²)；_parse_jam_level 同理（272-307）。
- 无 seq 单调校验：_decode_0a01 只验长度与坐标范围（594-605），重放帧会重复刷新 revision，把 INFO_CURRENT 再续命一次。
- 文档漂移：ARCHITECTURE.md:63 的成功判定在代码中不存在；position_fusion.py:416 docstring 写 200 ms，与部署值 0.5 s 冲突。
- match_rx_gui.py:694 用 cid.endswith(cid_key.replace(...)) 决定数据树归类，cmd 命名一变就静默走 else 分支。

## ⑤ 选型结论

可直接摘用：1) server_comm.py 的 latest-value + 优先级 + generation 三件套（76、151-172、178-204、215-226、397-404、433-453），覆盖写、发送前二次校验、入队后按值回滚，逻辑闭合，是最值得整体搬走的部分；2) radio_bridge.py 帧解析骨架（408-445）——CRC8 定位帧头、长度表定界、CRC16 确认、假帧只丢 1 字节重同步，加带代次与逐车 received_at 的事件对象（85-110）；3) position_fusion.py 全文，纯逻辑无 IO、clock 可注入，能用假时钟完整单测；4) protocol.py:27-70 的 CRC 与帧构造。

必须改：SEND_PRIORITY（server_comm.py:37-45）按业务重排并加老化/最大等待，否则低优先级饿死；透传 source_seq（jam_rx_app.py:763-767）会产生重复 seq，对端严格时改服务端自增；三份重复常量收敛到一处；融合超时默认值 0.2（position_fusion.py:86）与部署 0.5（yaml:98）不一致，接入时强制只读 yaml；「一级/二级成功」判定要自补，若要求解出密钥才算成功，需在 1168-1235 的密钥匹配处加计数回传；INFO_MAX_ESTIMATED_AGE_SECONDS=0.20（104）应提到配置，否则换采样率或缓冲长度就误丢帧。

最小接入（改 5 个文件）：radio_profiles.py（频率/带宽/接入码）、jam_rx_app.py（configure_receiver 实参与等级来源）、server_comm.py（优先级表，可选）、radio_bridge.py（host/port 与坐标解码）、params.yaml（三个超时）。position_fusion.py 与 protocol.py 原样可用。完全不用碰：match_rx_gui.py（有自有启动器时）、full_duplex_gui.py、counter_drone/*、experiments/*、docs/*。

换方案的触发条件（代码内有硬体现）：要双波同解——get_receiver_profile 一次只返一个 RadioProfile（338-348），必须重写射频层，调参无用；要三板融合——update_command_data 不收 0A07（343-364），得先加入队接口；空口帧长非 24 字节——radio_bridge.py:431-435 直接丢帧，需同改长度表与 _decode_0a01；要求端到端 < 200 ms——硬编码 0.20（104）即天花板，必须改代码而非改配置。
