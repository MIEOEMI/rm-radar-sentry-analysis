# CombatRadarSdr 源码精读（M3b）

## ① 模块与数据流

phy.py 是纯 GFSK 数学工具，不是硬件采集：全文无 adi/socket/文件 IO（grep 实测无 adi），只有 4 个无状态函数 `bytes_to_bits_msb`(:6)、`gaussian_taps`(:11)、`packet_to_iq`(:20)、`fm_demod`(:37)，收发共用同一套。真正调 pyadi-iio 的全在 apps/：`jam_rx_app.py:32` import adi、`:878` `adi.Pluto(rx_uri)`、`:1073` `rx.rx()`、`:438-453` 设 rx_lo/rf_bandwidth/gain_control_mode_chan0/rx_buffer_size；`tx_app.py:17,147`、`jam_tx_app.py:19,177`、`full_duplex_gui.py:166` 同构。必读清单里没有采集文件，采集代码全部在清单外。

接收分派在 `jam_rx_app.py`：`:1129-1141` JAM 模式走 legacy（`fm_demod`+`slice_packet_candidates`）；`:1239-1276` info 模式且 `RADAR_INFO_DECODER=aggressive`（`:615-622`）走 `FastInformationWaveDecoder.process`；`:1288-1302` 保留 legacy 供 `RADAR_INFO_DECODER=legacy` 回滚。频点由 `get_receiver_profile`(:338-348) 按 team/level/parse_policy 查 `radio_profiles.py:21-34`，level 变化在 `:1013-1057` 触发重配并 reset 解码器。

三级流：15B 空包 → `_CoordinateReassembler` 拼帧 → `decode_cmd` 解字段 → `publish_info_frame`(:722-871) 越界过滤/端序纠错/TCP 上报。实测 0A01 帧 33B=3 空包、0A05 帧 50B=4 空包、0A06 帧 15B=1 空包。

## ② 关键实现

**帧**（protocol.py）：SOF=0xA5(:5)；头=SOF+len(LE u16)+seq+CRC8(:51-57)，CRC8 只覆盖前 4 字节，**不覆盖 cmd_id**；体=cmd_id(LE u16)+data；尾=CRC16(LE)(:59-62)。`build_air_packet`(:65-70) 强校验 15B，输出 8B 大端访问码+2×大端 u16(=15)+15B=27B/216bit。cmd_id 与载荷：0A01=24B 六车 x,y 各 u16 LE；0A02=12B 六血量；0A03=10B 五弹药；0A04=8B 经济+占点 u32；0A05=41B 五组 7B 增益+姿态+五状态；0A06=6B ASCII 密钥（protocol.py:18-24）。完整编码侧只有 `message_value_generate.py:134-185`。

**CRC 实测**：`crc8_maxim` check("123456789")=0x0B（非 MAXIM-DOW 的 0xA1）；`crc16_ibm` check=0x6F91（即 MCRF4XX，非 X-25 的 0x906E）。两端同源自洽，但按函数名猜算法去对接外部实现必错。

**同步**：64bit 访问码+32bit 固定头 0x000F000F 拼 96bit ±1 模板(:500-518)；INFO 与 JAM 访问码汉明距实测 26bit。

**软判决**(:559-612)：相关峰符号定极性 → 用 96bit 已知段最小二乘估 scale 与门限 `known_mean-scale*template_mean`(:574-581) → 全帧硬判 → `payload_soft=(x-threshold)/scale`(:600) 留软值供聚簇合并。容限 `max_access_errors` 与 `max_header_errors` 两道闸，再用 `info_jam_access_margin`(:586-592) 拒“更像 JAM”的候选。

**47 相搜索**(:700-725)：低通→鉴频→匹配滤波后 `phase in range(sps)` 各抽符号做一次归一化互相关（去中位数直流+滑窗能量归一，:90-110），取超阈且间隔≥8 符号的峰(:113-136)；按 start_sample 聚簇(:661-676)，`soft_combine_candidates` 份按 `corr**power` 加权平均软值重判(:631-659)。合并后仍按 `start_sample` 去重，距最近 512 个已收包 ≤sps 的直接丢弃(:731-739)。

**SPS/灵敏度**：sps=47、bt=0.35、fs=1M → 21276.6bps，216bit 空包 10.15ms。`sensitivity` 是 rad/sample：`phase=cumsum(sensitivity*shaped)`(:32)，1.5628→248.7kHz 频偏、Carson 518.7kHz 正好填满 info 的 540kHz；jam 2.8194→448.7/918.7kHz vs 940kHz。这些数只在 fs=1M 下成立。`fm_demod`(:37-40) 每块减均值，等价 AC 耦合，丢掉绝对频偏，因此全链路没有 AFC。

**拆粘包**(:332-469)：bytearray 累积，`gap>maximum_air_gap_samples` 或 `gap<0` 清缓冲(:357-371)，上限 16384；find(0xA5)→CRC8→`data_len` 必须等于 `INFO_COMMAND_LENGTHS` 期望(:403-407)→不足 break 等包→CRC16 失败**只丢 1 字节**(:429-438)，这是抗假 SOF 的关键。

**CRC 修复**(:171-230，生产关闭)：预计算 208 条 syndrome delta（192 数据位+16 CRC 位），1bit 查表、2bit 用 `syndrome^delta` 配对；`_choose_repair`(:242-295) 按 (合法车数,-回退数,-翻转位,-连续性) 字典序选优，禁止越界修复覆盖原合法车。

**legacy 路径**（gnuradio_frame_parser.py，仍在 JAM 与回滚态生效）：`slice_packet_candidates`(:394-468) 先用 `mean|x|` 挑最佳相位(:408-417)，再在 ±refine_span 内穷举 offset，硬判后交给 `parse_air_packets`；后者**只比对 64bit 访问码与两个长度字段 l1==l2==15 就标 valid(:361-389)，完全不做 CRC**。硬判失败才降级到软互相关：`soft_correlate`(:162-187) 去中位数+滑窗能量归一，`find_soft_sync`(:190-229) 取 abs 峰，`soft_try_decode_packets`(:268-326) 在峰位 ±3 符号内搜帧头，阈值只在 base_idx 估一次。offset 之间用 `evaluate_offset_candidate`(:532-565) 克隆缓冲区试解、按 (0A01/0A05 命中数, 帧数, 模糊分, 追加包数) 排序选优，`OffsetLock`(:594-629) 用 hold_buffers=8 抑制抖动。

**JAM 另一套**：`extract_jam_0a06_frames`(:364-399) 在 30B 双包滑窗逐字节找 0xA5，验 len==6/CRC8/cmd==0A06/CRC16；`jam_confidence` 用 0.45 访问码距+0.40 CRC+0.15 ASCII 字符集加权(:409-413)，默认门槛 0.85。

## ③ 可配置项与依赖

依赖仅 3 条(requirements.txt:5-7)：numpy 2.2.5、pyadi-iio 0.0.20、pylibiio 0.25。兼容补丁：`gnuradio_frame_parser.py:10-12` 注入 `np.int`；`jam_rx_app.py:20-29` 关浮点报错并预热 `numpy._core.*`。

`FastInformationWaveDecoder` 暴露 15 个参数(:475-492)，生产实参见 `jam_rx_app.py:665-683`：corr_threshold=0.45、max_access_errors=24、tail_samples=12000（内部强制 ≥(216+8)*47=10528，:498）、maximum_air_gap_ms=0（禁重置）、max_repair_bits=0、emit_invalid_coordinates=False、iq_lowpass_hz=270k、info_jam_access_margin=2、max_header_errors=2、soft_combine_candidates=3、cluster_rank="sync"。

频点 `radio_profiles.py:21-34`：info 433.2/433.92MHz、BW540k、tx−50dB；jam 六档 432.2/432.5/432.8、434.92/434.62/434.32MHz，BW 940/860/250k。CLI 见 `jam_rx_app.py:560-607`。

## ④ 代码质量与风险

1. `gnuradio_frame_parser.py:405` 归一化写 `sensitivity*1e6`，把采样率写死 1MHz；改 --sample-rate 会让 legacy 判决整体失准，而 fast 路径完全不看 sensitivity，两条路径对同一参数敏感度不一致。
2. `FIELD_MAX_X/Y=2800/1500`(:37-38) 与 `INFO_FIELD_BOUNDS`(jam_rx_app.py:95-98) 重复；`INFO_COMMAND_LENGTHS`(:39-45) 复制 protocol 的 `INFO_CMD_LEN`(:18-24) 而非 import；15/216/96 多处各写一遍；`ROBOT_NAMES`(:47-54) 与 gnuradio:90-97 重复，而顺序即字节序，改一处不报错只静默错位。
3. `np.convolve(complex64,float64)` 实测提升为 **complex128**(:692 每块一次)，内存/算力翻倍；47 相×96 抽头是纯直接卷积，无 FFT。
4. 容限自相矛盾：`max_access_errors=24`(37.5%) 实际被 `margin=2` 压到约 12bit——真 INFO 访问码错 k 位时 jam 距最坏为 26-k，判据 26-2k≥2 → k≤12；反向 JAM 码需错≥14 位才会被误判成 INFO。24 这个值基本惰性，调大无收益，两侧边界只隔 1bit。
5. 门限用 96bit 已知段估一次套 120bit 载荷；`scale<=1e-12` 直接丢包(:578)，弱信号下等价静默丢帧，且无独立计数暴露。
6. `packet_to_iq` 每包相位从 ~0 重新 cumsum，包间无相位连续/无保护间隔（tx_app.py:175 直接 concatenate），边界产生一次鉴频毛刺；`amplitude*2048`(:33) 的标定来源代码里没有说明。
7. `protocol_jam_key.py` **全仓零 import**（仅自身 docstring 与 README 提及），死代码；且生成 7B 密钥(:51-53)，接收端写死 6B(jam_rx_app.py:76、:376)，7B 帧 16B 超单空包 15B 容量，根本不兼容。同文件 `_generate_periodic_key` 用 `time.time()` 初始化(:72) 却与传入 sim_time 比较(:100)，首次判断必错；TIME_BASED(:110-144) 名不副实。
8. legacy 的 `valid` 不含 CRC(gnuradio:387)，`extract_frames` 只拦 `data_len>256`(:507)、不校验 cmd_id 白名单：假 SOF 若恰好凑出合法 CRC8（1/256）就能注入最长 256B 的伪帧并吞掉后续真帧。
9. `StrictInfoCycleFilter.accept` 恒 True(:50-55)，`--strict-cycle` 是空开关；`COORDINATE_CRC_DELTAS` 在 import 时算 208 次 CRC16(:187)，即使生产 max_repair_bits=0 也照算。
10. `LiveInfoState.update` 标注 `->None` 却 `return changed`(:642-650)；`decode_cmd_0a05` 只回 hex(:125-126)，41B 结构接收端从未解析。
11. 测试 `assertLess(processing_ms,450.0)`(test:82) 是墙钟断言，慢机假失败；全部用例跑合成 IQ，无真实录包回归（`experiments/results/*.json` 引用的 c64 未随包分发）。
12. 0A01 端序“自动纠正”(jam_rx_app.py:737-752) 只在越界时单向切到 big、不切回，一次误判会让后续所有帧按大端解释。

## ⑤ 选型结论

可直接摘用：protocol.py 帧格式与 CRC(:51-62) 自洽且有测试覆盖，当协议字典抄；`info_wave_fast_decoder.py` 的 47 相搜索+归一化相关+已知段自适应门限(:559-612、:700-725) 不依赖灵敏度/增益标定，是这套代码最值钱的部分；`_CoordinateReassembler` 的“CRC16 失败只丢 1 字节+cmd_id 强校验长度”重同步(:383-438) 明显强于 legacy 的 `data_len<=256`；`radio_profiles.py` 参数表直接复用；0A05 的 41B 布局只能从 `message_value_generate.py:172-185` 反推。

必须改：`sensitivity*1e6` 改 `sensitivity*sample_rate`；低通 taps 转 complex64 消除 complex128 提升；`max_access_errors` 降到 12 左右或与 margin 解耦，别留惰性旋钮；legacy 的 `valid` 补 CRC 或直接下线；删 `protocol_jam_key.py` 或按 6B 重写；`StrictInfoCycleFilter` 要么实现要么删；0A01 端序判定要可回切。

换方案触发条件（代码里的体现）：`correlation_peak` 长期贴 0.45 且 `best_access_errors` 贴 12，说明门限与容错在硬扛噪声，该换相干解调/前导设计而非继续调参——`jam_rx_app.py:1286` 已把 correlation_peak 打进告警，就是给这个判断用的；`estimated_age_seconds > INFO_MAX_ESTIMATED_AGE_SECONDS`(0.20s，:104、:1258) 频繁触发说明 `processing_ms` 吃掉实时预算，47 相全搜算力模型不成立，需降相位或改 FFT 相关；缓冲长期触顶 16384(:369) 而 `gap_resets` 不涨，说明“永不清缓冲”假设(maximum_air_gap_ms=0)失效。

实测：`test_info_wave_fast_decoder.py` 9 项全通过（1.888s），覆盖 47 相、任意 buffer 边界、非固定包间隔、0A05=41B、1/2bit 修复、越界不覆盖合法车、边界包含性。未跑真实录包与 AD9361 硬件。
