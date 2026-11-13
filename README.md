# RM 雷达与哨兵开源方案源码分析

对两支 RoboMaster 战队的公开开源代码做逐文件精读与选型论证，供本队备赛参考。

- **东北大学 T-DT 战队** — [tdt-nav-kit](https://github.com/T-DT-Algorithm-2026/tdt-nav-kit)（RM2026 哨兵导航前后端，MIT）
- **厦门大学嘉庚学院 TCR 战队** — [TCR-RM2026-Radar-OpenSource](https://github.com/coach114514/TCR-RM2026-Radar-OpenSource)（RM2026 雷达站，mixed license）

## 内容

| 文件 | 内容 |
|---|---|
| [00-综合报告](reports/00-综合报告.md) | 主题对齐后的结论、模型间分歧复核、最终建议 |
| 01 | TDT 前端 `YAstar` / `KinodynamicAstar` / `main` |
| 02 | TDT 后端 `MinimumSnap` / OSQP / SFC |
| 03 | TCR 协议、访问码、软判决解码 |
| 04 | TCR 融合接入、状态机、硬件参数 |
| 05 | 许可证链与合规复核 |
| 06 | 选型落点与交叉挑错 |
| 07–08 | 基线全栈精读（TDT / TCR） |

## 方法

各组独立逐文件精读，结论强制标注 `文件:行号`，不复述 README。按主题对齐后，一致项采信、分歧项回源码复核（见综合报告第 5 节）。

参与模型：`DeepSeek-V4.1-Flash`（4 组）、`GLM-5.3`（1 组）、`Qwen3.8-Max`（1 组交叉参考）、`hy4free`（2 组基线）。

## 边界声明

- 本仓库只含**分析文本**，不含被分析项目的任何源码。引用均为短句 + 行号定位，用于说明问题所在位置。
- 所有性能数字（耗时、准确率、比赛成绩）均引自上游仓库 README / 技术报告，**未在本机复现**；唯一带本地复现的是第 03 篇中的 CRC check 值与 9 项单元测试（本机重跑通过）。
- 关于许可证的表述基于被分析仓库内的声明文本（`NOTICE.md` 等，快照日期见原文），**未联网核实上游仓库当前是否已补 LICENSE**，不构成法律意见。
- 分析中的批评性结论仅针对代码实现，不针对战队或个人。上游战队公开开源值得尊重，本文用于队内技术选型。

## 许可证

本文分析文本采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)。
