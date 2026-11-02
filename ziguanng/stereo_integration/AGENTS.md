## 2026-10-27 当前回归入口（优先于历史完成状态）

21:07 更新：`diag_renderer_legacy_band_v1` 已于21:04:04通过，raw0/raw1各30720像素（双目合计），170998 checks，33份输入SHA经主审复验。不要重跑此图带。仅 `diag_renderer_legacy_pnr_v1` 完整PDS仍在运行：runner44952起20:57:05.1739471+08、front41368起20:57:05.8554265+08；真实compile参数1/0/1/0。147源仅将renderer替换为临时旧caa9bc源，其余36c源码和6输入不变；正式源码未改。先收集该唯一构建，不重复启动。诊断恢复只缩小嫌疑，不能认定流水协议错误。

36c11b97 实板用户报告左画面整屏异常，LEFTCMP 两开关 OFF 原图仍异常、冻结后错误静止；右正常。旧 3aed438d 画面用户报告正常，但不是完整 P3 验收。状态 FAIL_USER_REPORTED / ROOT_CAUSE_PENDING；36c 暂不推荐演示。先读 runs/20261027_204011_83c2a350/field_handoff_v1/README.md、runs/20261027_204011_83c2a350/FIELD_STATUS_36c11b97.md 和 runs/20261027_204011_83c2a350/regression_delta_review_v1/conclusion.md，再以 state/events.jsonl 和两个任务的实时 status.json 为准。renderer 单变量回退的 PNR 与独立短图带验证已启动，先收集，不重复 PNR 或仿真；源码/活动任务输入冻结。旧 renderer 若恢复画面，仅支持 renderer 改动或布局嫌疑，不证明协议错误；未恢复再评估 controller 单变量回退。摄像头相位模型仅条件性线索，不据此改采样沿或标定；已封存包不改，诊断 bit 不发布为修复版。P3 完整通过前不 P4。其余预算、租约、保护和协作规则继续有效。

## 最新交付入口（2026-10-27 19:59；优先于以下历史）36c 候选包已由主代理核对包内333项SHA、ZIP全部CRC、147源与6输入绑定，并发布到 `D:/topic2_workspace/fpga_download`：`Top_cmp_controls_36c11b97.sbit`、`CMP_candidate_36c11b97.zip`、同名前缀 README/检查清单。旧文件保留。ZIP SHA为 `c6b145a62aefb9a1e65c6dbf0ec5e79568898c263788bc182d7a2631eb2155e5`，bit仍为已验收36c完整SHA；包内261项成熟基线仅四授权变化，四文件补丁双向应用和字节回退证据齐全。包来源 `runs/20261027_183144_e900c78c/candidate_package_v1`。后续不要重复包装/PNR/全帧/已完成STA。优先实板记录和有新依据的物理结构核验。整体时序仍失败、14物理FF唯一性INCONCLUSIVE、全部板验NOT_TESTED，P4未启动。PDS界面读取授权超时，未进入窗口操作；不要把等待授权当成验收。该等待使旧租约到期，主代理停止旧token写入并取得新租约收尾，仍以原轮20:01总截止为限，不借重取租约扩展预算。单一事实源仍为events.jsonl。## 最新执行入口（2026-10-27 15:40 +08；以最新事件为准）

本节优先于历史内容；`state/events.jsonl` 是唯一事实源，本节是快照，先读最新事件、checkpoint 和登记任务并核实际 status。当前 run `20261027_151911_388cc37d`。

- 正式全帧 `runs/20261027_132928_a9f38d92/pixel_reset_render_paced_full_v1/status.json` 于 14:55:13 passed，2157156 checks、3397577 cycles、Errors 0；raw0/raw1 各 921600（每项双目合计），31 native + 2 bench + 4 常量绑定 passed，147 源和 6 输入 unchanged。模型带宽不代表板上实测 FPS。
- 原 Top `runs/20261027_132928_a9f38d92/reset_render_cmp1_full_v1/status.json` 完成，commands_pass、bitgen、输入不变为 true，实际参数 1/0/1/0，High Voltage；bit SHA `36c11b97f1266ce3f0202e21ead4c67f4610742a8991f741cf2ceb94a80405cb`，DB SHA `94e975eb6e75f3d9eed2305e3b51338227a0f4cdd2883df6c208d5f73c93c456`，status SHA `64f0a6e34795c50c51915aba2c7520873ee920e14cdddadfba769447fdde4bfb`，整体 timing=false。旧两个 pending job 已收集，不重复 PNR/full。
- `runs/20261027_151911_388cc37d/reset_render_sta_v2/summary_v2.json` 已完成 `passed_scoped_collection`；主审独立重解析 12 份报告 34382 paths，核对报告 SHA/filter/DB/bit/执行身份及完整 paths 对象。own 负 0、mature 100、vendor 727，unique 827/occurrences 827、unknown 0；整体 timing_pass=false。
- `runs/20261027_151911_388cc37d/reset_release_paths_v1` 原生报告 SHA `bd6a28ed77111dc6a2a70f267583d43f9c3e4e86a55ceb2a2d5130778bc51b23`；主审复验 DB、from/to 过滤，56 paths 覆盖左右各 0→1..7 的四 corner/type，均同 core clock sequential、无负、min +0.184ns。每眼共同 firststage Q2 可证；D 端 Location 为共享 site/Mx 资源输入且重复，并非唯一 FF ID：14 个物理 FF 互异/未合并仍 INCONCLUSIVE，不能由逻辑名称断言物理 14，也不能由重复 M 输入断言合并。先查现成 PDS GUI/实际映射网表或其他有证据的方法补结构，不盲重跑 probe。
- 目前无活跃原生任务；正式全帧、构建和全部 STA 已完成。除非具体新失败或源码变化，不重复全帧、PNR 或已充分的 8 项测试。后续先读 checkpoint 和闭合证据，准备当前 bit `36c11b97f1266ce3f0202e21ead4c67f4610742a8991f741cf2ceb94a80405cb` 的源绑定候选交付包，包含成熟边界审计、完整回退、全部 STA 与结构限制、像素、板验清单和真实 SHA；用户指定下载位置 `D:/topic2_workspace/fpga_download`，新文件保留旧回退。本轮未发布新包或下载。
- `pds_netlist_probe_v1` 和 `netprobe_v2` 原生进程均 exit 0 但未进入 Tcl、无完成标记、无网表；原输入未变，停止重试；不能据此断言 gen_netlist 不支持 PNR。复位副本结构仍待物理证据。正式 controller `8be93`、renderer `3e1c52`、cache `4cb` 不变，活动源冻结。source patch 和原下载保留；未发布新下载或包。新包需绑定新 bit/DB/像素/全部 STA/源审计/回退。
- P3 尚缺整体时序与实板 8 CMP 组合、四类功能、实测 FPS、延迟、10/30 分钟稳定性及标准图卡；P3 全通过前不进入 P4。

每轮读最新事件与任务，取得自己的 run_id/token；写前 check，先 step_begin 后 step_end，15 分钟租约、至少每 3 分钟 heartbeat，写 checkpoint 和 run_finish。按该轮 run_start 返回的 80 分钟开发、90 分钟总预算；仅本轮 15:19 开始、16:39 开发截止、16:49 结束截止，未来轮不得沿用。额度可读且低于 10% 提前收尾，不可读按预算。后台任务保留并登记。成熟/vendor 不改；Top/PDS 仅限授权必要参数、接线和清单，不复制 Top，不接废弃 coordinator，不运行 update_source_list.py。最多两个子智能体且不递归委派：Explorer gpt-6-luna/high，Worker gpt-6.1-terra/high，Complex Worker gpt-6.1-sol/high；Reviewer 仅必要时使用，继承当前父模型与档位，不传模型/档位覆盖，不必每次创建。主代理决策和最终验收。旧任务 2 保持 PAUSED；只对重要进展、失败或需用户操作通知，其余安静；P3/P4/P5 全部通过后按官方流程暂停续做。

## 最新执行入口（2026-10-27 08:51 +08；本轮事件截至 1433）

本节优先于下方较早的交接和执行入口；下方内容保留作历史记录。本轮工作目录 `D:\虚拟C盘\ziguanng\stereo_integration`，run 为 `20261027_083010_8f0b5035`，事件日志 `state/events.jsonl` 是唯一事实源。

- **旧压力 job 已被主审接受为测试体完成。** 本轮收集记录在 `runs/20261027_083010_8f0b5035/pressure_collected_v1/collected_status.json`，授权/接受记录是事件 1402。raw 0 与 raw 1 各 921600 像素（每种 raw 为双目合计），final 检查 4627023、Errors 0、源/标定/compiled 绑定通过；原 wrapper timeout 与 native exit 未知照实保留。这不是 package passed。禁止再运行旧压力 collector 把已提升的当前源码与旧任务比较、触发误判或回退。
- **正式 cache 已提升且保持冻结。** 唯一正式改动为 `rtl/cache_request_slice_v3.sv`，前 SHA `71911af16b23db5e63a34b74711726b91dfaa6a536ebb01c3c3db2c0b22362eb`、正式 SHA `4cb51b9d3506ffcaaeed8e368de29289a26598024551582f75a775824bd32c64`。before、v4 manifest、原补丁、校验补丁和提升证据在 `runs/20261027_083010_8f0b5035/promotion_v1/`；`promotion_evidence.json` 记录旧构建 147 源中仅 cache 与新源不同、6 个输入不变。依据事件 1416 和该证据，不因旧 build 源哈希不匹配回退。
- **唯一新原 Top PDS job 尚在运行。** `runs/20261027_083010_8f0b5035/cache_ring_cmp1_full_v1/status.json` 登记 runner 42700（08:34:09.023 +08）、native tool 41936（08:34:09.742 +08）；当前 `state=running`。native 日志确认参数 1/0/1/0，synthesis 成功并保存 `synthesize/Top_syn.adf`；尚不能写 PNR、完整时序或 bitgen 通过。以后以磁盘 status 和新事件为准，不启动重复 PDS。
- **唯一正式 paced 双目全帧也在运行。** `runs/20261027_083010_8f0b5035/pixel_cache_ring_paced_full_v1/status.json` 当前 `running`；runner 43832（08:35:23.366 +08）、tool 41668（08:35:31.998 +08）、kernel 24304（08:35:32.639 +08）。输入是正式 cache SHA `4cb51b9d...32c64`、默认 formal engine/controller/renderer；命令启用 `--height 720 --display-hz 60 --camera-hz 30 --profile --timeout 7200`，校准 `runs/20261021_195529_57401425/calibration_v2`，crop 证书 `runs/20261022_055435_029654e1/crop_certificate_v1/certificate.json`，输出及启动清单均在本轮 job 目录。当前 load model 是 640×720 平均带宽 60/30Hz 场景（status 中有效模型约 60.021/30.037Hz），**不是实测 FPS、FIFO/曝光证明或实板结果**。job 输入冻结；不要复制重跑。
- 旧 bit `91744401...c2839f2`、DB `71932eb5...396fe8f` 的 12-scope STA 只适用于旧版本。不得借给新源/新 DB。新的 `scripts/collect_scoped_sta_v2.py` 已由主审独立 8/8 合成测试通过（事件 1432），不代表新设计时序通过。仅待新 build 完成、DB 无 lock 后，由主审独立计算 DB/bit SHA，再显式传 `--run-id --token --build --output --db-sha256 --bit-sha256`；先跑 `owned_views`，再按预算完成 12 scope。`--resume` 只用于相同 run/output。其他轮已有报告只读核对；用 `report_existing_timing.py` 补缺项并在同 DB 汇总，不重复已完成 native report。
- P3 尚缺新 paced full 和新 DB 的有效证据，也缺实板 8 组合、四类功能、帧率、延迟、10/30 分钟稳定运行及标准图卡。P3 全部通过前不能开始 P4 测长。新 STA 工具或模型场景都不能代替这些验收。

每轮先读 events/checkpoint/job，再获取本轮自己的 `run_id`/`token`；写入前 `check`，步骤先 `step_begin` 后结果，JSON 必须实际解析，events 为唯一事实源。租约 15 分钟、至少每 3 分钟 heartbeat；总预算 90 分钟（80 开发、10 收尾），按每轮 `run_start` 返回的 `development_until` 与 `finish_until` 执行；本 run 的具体截止只适用于 run `20261027_083010_8f0b5035`，其他轮不得沿用。额度可读且剩余低于 10% 时提前收尾，额度不可读仍按固定预算。超时后台 job 保留并登记，收尾时给 checkpoint；不得为等候而停止、重启或改动其输入。源码和活动 job 输入冻结；成熟/vendor 不改，Top/PDS 仅限用户授权的必要参数/接线/清单，不复制 Top；不接废弃 coordinator、不运行 `update_source_list.py`。代码任务用 Luna/high，主代理审查，最多两个子智能体且不递归委派。旧六小时任务 2 保持 PAUSED；只有重要进展、失败或需用户操作时通知，其余安静。主代理可在授权范围内官方管理现有 fpga automation，不创建重复任务。验收未全通过前不暂停任务，全部完成后按官方流程暂停续做。

## 最新执行入口（2026-10-27 04:33；高于下方历史）

- 唯一旧全帧 `pixel_corrected_raw_full_v1` status=`runs/20261027_032818_ab0b24da/pixel_corrected_raw_full_v1/status.json`，是无 `load_scenario` 的 unpaced behavioral pressure。formal SHA=`71911af16b23db5e63a34b74711726b91dfaa6a536ebb01c3c3db2c0b22362eb` 冻结；runner 35712 在3600s wrapper timeout_running后退出，front/tool 36924、kernel 14636仍活。front启动03:33:43.079，kernel启动03:33:43.1068784+08（原status无kernel启动时间）。`completion_parent_probe_v1`=`still_running/bodyfalse/exitunknown`，collector 11项测试经主审复验通过。无正式提升或新PNR/full。
- 先只读执行 `scripts/collect_stereo_shared_completion.py --source-status original --output runs/<新run_id>/pressure_collected_v1`。不重启、不覆盖原status、不伪造exit 0；still_running/process_state_unknown继续冻结。只按证据报告 test-body 完成/通过、源/标定/compiled绑定、Errors=0、左右各921600像素两case；collector不等于package passed。主代理验收后才可触发Luna按有效v4 manifest对自有cache单文件byte-exact提升；旧错误v3/provenance不可用。新full需paced 60/30、`--timeout 7200`及新完整PDS，不复用旧STA。旧12-scope汇总完整路径=`runs/20261027_032818_ab0b24da/retimed_sta_v1/summary_v3.json`，只适用旧bit/DB。
- 原始数据/封存DB/证据只读；活动job输入冻结。成熟/vendor保持原样；Top/PDS仅限用户已授权的必要参数/实例接线/源码清单，禁止复制维护另一Top；当前任务仅自有cache单文件并须主审授权。P3实板通过后才P4；旧六小时任务2 PAUSED。

每轮读 `state/events.jsonl`、checkpoint 和 job，以 `start` 获取自己的 `run_id`/`token`；写入前 `check`，步骤先意图后结果。租约15分钟/3分钟心跳，预算90分钟（80开发+10收尾）；额度可读且<10%时收尾，查询不可读仍按固定预算；结束写 checkpoint 和 `run_finish`。长任务登记唯一目录/PID/启动时间；最多2个子智能体，代码交 Luna/high，主代理决策验收。只通知重要进展、失败或需用户操作，其余quiet；全部验收后官方暂停续做。主智能体可在用户已授权范围内用官方工具管理现有 fpga 任务，不创建重复任务。

## 最新交接状态（2026-10-27；优先于下方历史）

- `builder_cmp1_full_v1`上一轮完整命令通过，但整体timing_failed；bit=`91744401f4505ccdb9f3d3be1ab25455af817d164ddce03f34bcde094c2839f2`，唯一PNR DB=`71932eb5dde4b05af888f895b4d1d96200e9e267730cad89a1646a211396fe8f`。12项定向STA仅绑定该旧bit，v3逐项身份/解析来源经主审核验。own cache/request-buffer仍4条负setup（最差−0.115ns）；记录“限域收集通过”，不得写整机时序通过或实板通过。
- 隔离候选`production_candidate_v2` SHA=`4cb51b9d3506ffcaaeed8e368de29289a26598024551582f75a775824bd32c64`已获条件性提升审查；formal目前仍SHA=`71911af16b23db5e63a34b74711726b91dfaa6a536ebb01c3c3db2c0b22362eb`，没有拷贝应用。640×24带测试累计61,440像素及20,091周期差分已通过；8ns组件壳无恢复/移除摘要并含未约束端口，只算组件证据。
- 唯一旧正式源全帧`pixel_corrected_raw_full_v1`的runner 35712已在3600s wrapper timeout_running后退出，front/tool 36924与kernel 14636仍运行；不是paced验收。先只读以`collect_stereo_shared_completion.py --source-status original --output runs/<新run_id>/pressure_collected_v1`收集。只有主代理独立验收test body完成及输入绑定后，才可明确触发单文件byte-exact提升，再开展新源码全帧和新整机验证。
- promotion manifest/最新事件是源身份依据；之后旧构建与新formal SHA不同，不得借此回退formal。旧STA不借给新源或新bit。CMP保持冻结；不接入废弃coordinator、不运行update_source_list；P4仍在P3实板通过后。输出摘要见`runs/20261027_032818_ab0b24da/retimed_sta_v1/summary_v3.json`。

执行规则：每轮检查事件、进度与后台任务，先取新start租约；写入前check，先登记step意图，events.jsonl为唯一事实源。租约15分钟且至少3分钟续约；80分钟开发+10分钟收尾，额度低于10%收尾。长任务用唯一目录登记PID/启动时间并先收集；同时最多2个子智能体，代码交指定Luna，主代理负责决策/验收。正式源、原始数据与封存DB只读。P3未实板验收前不进P4；旧六小时任务2保持PAUSED。

## 最新交接状态（2026-10-26 22:46；优先于下文历史）

- Claude交接的CMP RTL、仿真、ModelSim候选构建/bitgen和12份定向STA均已完成并收集。CMP设计已冻结：不得重写，勿接入废弃`rtl/cmp_pair_coordinator.sv`，勿运行`update_source_list.py`。
- CMP bit SHA=`3aed438d0edcbb257e9c9a2441edf9f7f15ffc87a41c1b0794c64fddaa4e04f7`；12份STA已确认completed、filter_applied、输入DB不变且报告SHA正确，唯一DB SHA=`c52a69c93c9174e7e72089e4a2bb7c79535502b4c642cb42b3c30629077571f1`。CMP逻辑无负路径。自有`imu_ray_homography` builder另有6条setup负路径，最差-0.239ns；先收集当前修复的运行状态与源码SHA，随后按checkpoint决定是否继续修该自有逻辑，成熟模块保持不动。
- PC离线去畸变测评工具A和自有测试已实现，17项测试及主审细节待最终确认；摘要要求、命令和指标解释见`docs/离线去畸变测评工具.md`。最新reference-only演示在`runs/20261026_215544_d1bf1db4/quality_demo_reference_only_v1/report_same_raw_regions_v1`，只验证方法，不是RTL/板上证明。
- 工具验收要求见本轮任务交接：640x720与左右眼/calibration/output_K、ray/crop严格绑定；记录同一scene_id及图像、标定、元数据和bit/RTL运行状态指纹。v1的ray矩阵仅支持inv(output_K)，其他旋转ray模型明确拒绝；校准文件缺少eye字段时从source文本核眼别。前后棋盘ID需由标定raw→output映射检查，发现翻转/歧义显示NA且不重排；输出native与换回输入像素尺度两套几何结果。黑色有效内容不得自动屏蔽。RGB565截断保留OpenCV BGR顺序，展示参考图必须与比较数组相同，无效热图区域灰显。HTML提供摘要表和折叠完整来源JSON，CSV拆分PSNR/MAE/max及三分区x/y RMS/max/count/NA。硬件源缺少匹配run-status证据时标`DECLARED_ONLY`，不能仅凭64位哈希称已绑定。错误尺寸、错眼、K/crop错配应拒绝，不能自动配准抬高PSNR。
- 独立左眼RTL核心全帧`quality_rtl_capture_v1`于22:40:38完成，ModelSim `ALL_PASS_QUALITY_CAPTURE pixels=460800 cycles=854693 core_cycles=854172`、Errors 0；21份RTL/bench源SHA一致，启动前capture脚本SHA=82a6bc2ad3ea5e05b5cf6e2c09d6bf5151db5637a11190ca153fe6f9ded805b4，raw/config/calibration/metadata/manifest/output SHA均绑定于status和source_binding_audit，source_unchanged=true。报告在`runs/20261026_215544_d1bf1db4/quality_rtl_capture_v1/report_rtl_core_left_v1`，source_binding_status=VERIFIED。主审独立复验了21项源码/聚合哈希、actual PNG/dump/metadata/status绑定及17项测试，并确认数值和静态报告结构在本轮范围内通过。仅为左眼undistort core+行为DDR adapter，不代表完整CMP或实板；报告HTML未在浏览器渲染，PNG和结构已目检。FPGA-core输出对OpenCV参考PSNR=37.9746dB、MAE=1.4114、max=8；角点直线度same-scale x/y RMS=.2267/.0796px，raw=.2004/.0666px，故校正未改善此留出样本，原样保留；edge为NA（0角点）。Probe v2仅7200像素，明确diagnostic_only。
- builder自有模块修复已完成功能和组件internal复核；下轮先收集运行状态与源码SHA。完整整机`builder_cmp1_full_v1` PID 38284/tool 42496于22:22:33启动并冻结源码，下轮先收集，不重复起PNR。12份旧STA仅适用于旧CMP bit，不能转借。PC离线测评及核心仿真capture范围按上述已通过；整机PNR和实板仍未验收。CMP RTL已完成，不重写或回归重做，不登记废弃coordinator。

## 历史：CMP实现已由Claude接手（2026-10-26，用户在Claude线程选择“我接手实现”）
CMP RTL、测试与约束已完成并写入：controller/layer/Top/PDS（授权例外）、rtl/cmp_pair_gate.v（新）、frame_view_render_retimed.v（view_step_x_q16/MIN_STEP_X）、stereo_algorithm_view.v（透传+job_idle）、sim/tb_cmp_ui|tb_cmp_pair_gate|tb_cmp_render.sv、scripts/test_cmp_ui.py（四个bench）、test_renderer_candidate.py、test_view_control.py。设计、证据、修改前快照、changes.patch、前后SHA和补充约束都在runs/20261026_172609_43ff93e4/claude_cmp_v1（说明.md、cmp_controls_cdc.fdc、上板检查清单_CMP.md）。
**不要再实现或重写CMP，不要接入GPT未完成的rtl/cmp_pair_coordinator.sv（已被cmp_pair_gate.v取代，不在PDS清单；不要运行update_source_list.py把它加进去）。** 续做只做：设备上ModelSim回归→teammate审计→新CMP1候选PNR/bitgen（额外加入cmp_controls_cdc.fdc）→Top_cmp_controls.sbit另存→实板清单。若回归失败，先记录复现与日志，交主审判断，不擅自改设计语义。
CMP检查点：cmp_rtl_done（已完成）、cmp_modelsim_regression、cmp_teammate_audit、cmp1_pnr_build、cmp1_bit_saved、cmp_board_8_combos。

## 历史CMP授权与优先级（已完成）
本节记录的CMP实现授权已完成并冻结；当前优先级以本文最前的2026-10-26 22:34交接状态和PLAN.md最新入口为准。

历史CMP实现授权（截至2026-10-26）仅包含原UI模块stereo_ui_controller.v、stereo_ui_layer.v，以及必要的原Top参数实例接线、PDS源码清单；该一次性授权已经完成，不是后续任务的长期文件范围。实现时原Top参数统一ENABLE_CMP_CONTROLS默认0；构建工具通过--cmp-controls启用CMP1，并从原生日志核实实际值。其他成熟底层严格禁止改动，禁止复制Top；保存原指纹和patch。CMP1构建/像素/打包身份必须显式绑定，旧CMP0结果保持兼容但不可充当CMP1证据。相对STA、像素、板上范围须分别说明。

自动化使用官方automation API管理；本轮更新请求和原始快照仅作审计材料。预算恢复、租约、标定和阶段规则延续下文。

## 智能体模型分工（用户指定）

当前主智能体保留本聊天当前模型，负责思考、任务拆解、只读检查、测试验收、证据核对及用户沟通；不得更改账户全局模型，也不得把定时主任务改成 Luna。涉及代码编写或修改（包括相关脚本与测试实现）时，交由明确指定 `model=gpt-6-luna` 的子智能体完成；主智能体独立审查并按需验证，发现问题交回 Luna 修正。

子智能体遵守与主智能体相同的工程边界、租约和日志规则，不得各自抢租约，也不得同时修改同一文件。若工具不支持指定 Luna，明确记录限制，不静默更换模型或声称已委派；主智能体可继续不依赖代码编写的只读分析和验收。

## 历史状态快照（2026-10-26，正式闭环修复已应用；CMP新授权见本文首节）

先读最新events/checkpoint和runs/20261026_110037_28f46c9d。**正式renderer/engine已应用闭环修复，PDS已登记cache_request_slice_v3.sv，不要再拷贝旧候选或重跑prepare_core。** Top本轮未改、未复制。formal_promotion_v1.json证明当前146份HDL按顺序内容SHA等于已完成PNR候选。候选基路径是runs/20261023_052640_7653313d/closure_candidate_full_pnr_v1；其中旧PDS项目指纹因合法提升已变化，不能拿其当前旧工程列表哈希失败就回退源码，也不篡改旧status。实际PNR仍是原Top+隔离自有模块的历史构建，正式源码等价证据与新正式Map复核分开记录。

新板测候选是本轮board_candidate_v2，实际bitSHA=eac8e042993f43625b5618bc5b9443bce5b31d0d2cbf999c380b4e9abc4729f4。自有六范围定向STA无负路径，最紧setup+0.001/hold+0.100/recovery+0.903ns。仍有60条成熟/vendor已报告负路径和154IO复核，relative保持REQUIRES_REVIEW，不能写全STA/实板通过。候选全640x720校正/raw各921600像素、2156860检查已收集通过；31相关RTL及4标定常量绑定，见full_pixel_binding_v1.json，不再重复同一全帧。

**SCBV已从代码+板卡资料解决，不再问用户。** 原队友107条gen_bit_stream都默认None、原位流也有Bitstream-2001。随附硬件手册PDF6明确100H/200H共用说明，PDF15配置图直接标PG2L200H，SCBV/AB15接VCC_CFG；官方UG040012表2-1与本机PDS enum对应High Voltage（不是数字3.3）。实际PNR L4空、L5有28个3.3V端口，L4/L5都用时设置无效的例外不适用。原Top/PDS旧配置保持，集成执行器使用--scbv "High Voltage"。scbv_high_bitgen_v2已从同一PNR重新gen成功，告警消失，数据库/源不变。v1空流程Flow-0044失败保留；gen_bit_stream必须加官方-force_to_run才能从新空工作区的已完成PNR库单独生成。不要用3.3/猜参数，不重新跑一小时PNR来改这个选项。

用户回复“不能看他的代码吗”后已按上述证据解决。只读JTAG扫描也已尝试：本机127.0.0.1:65420服务器连接/版本查询成功，但cableList为空，cfg_scan_chain报JtagServer-0306/Openconfigcable失败。没有assign/program/Flash/eFuse操作。HW003缺板卡供电/下载器或驱动可访问连接，具体物理原因未确认。无连接变化或新板上数据时，不反复跑同一扫描/重编译；P4不开始。

formal_closure_map_v1 已在2026-10-26 11:46:07完成并收集：正式原Top/PDS的Compile/Synth/Map通过，实际1/0/0；146源和5构建输入当前SHA全部重核一致。资源APM489/740、FF95215/319600、LUT61640/159800、DRM266/415与已布线候选一致。见formal_map_collection_v1.json。没有待收集后台任务，不再等待2368/15692/35432或重复起编译。此Map不含PNR/STA/bitgen，不能替代历史PNR或实板验收。

赛题MTF/SFR已补离线诊断：sfr_matlab_v2 / sfr_report_v2。MATLAB R2024b官方measureSharpness，8个高斯理论用例误差最大0.419%，零角/低对比度拒绝；3对留出中pair0001两眼x法线16对边、pair0015两眼y法线12对，共56ROI，pair0003两眼无合适边缘拒绝数值。x/y不等方单位不混合、不换毫米；校正后的MTF下降如实保留。编码亮度/OECF、焦点、软纸、局部放大率及参考模型/实板范围明确，QUALITY001仍须平整标准图卡和真实板上帧。v1元数据struct/UTF8错误已修，v2源码封存在sfr_source_v2；Python缺Matplotlib，使用MATLAB原生绘图，不安装依赖。

70项Python门禁通过，默认formal共享仿真compile-only已补root SV和retimed renderer依赖；realcore/pose/source-list工具同样补SV，避免漏模块。新增工具run_existing_bitgen和package的--bitgen-status严格绑定同PNR/实际命令/SCBV/新bit哈希，--pixel-status拒绝旧版或图带。最新模块/报表/报告不可借给旧版本宣称通过。

下一步：等待可用PG2L200H板卡/下载器连接或新的真实板上资料；连接变化后先只读核对器件，再按本轮board_candidate_v2/上板检查清单验证四功能、实际帧率/端到端延迟、10/30分钟稳定性和标准图卡，并绑定实际bitSHA。没有新连接/数据时只记录缺失条件并结束，不能重复同一扫描、全帧、Map或PNR。formal_map_collection_v1.json和包内formal_map_status.json已经封存，旧任务均已收集。独立焦距/横向仅fx-cx减半、格距181/7、对焦仍unknown、原UI CMP右侧二值化的既有结论保持。旧六小时2保持暂停；所有验收完成前不暂停fpga。

以下为历史入口，发生冲突时以最新事件、当前源指纹和本段证据为准。

## 最新续做入口（2026-10-23清晨，本轮中间快照）

先读最新events/checkpoint及runs/20261023_052640_7653313d。上一轮full_cache_reset_alg1_aux0_v1和reset_shared_full_v1均已收集完成，不再等待。正式双眼640x720校正/raw各921600像素、4626919检查通过；旧正式复位的真实PNR定向时序已转正，TIM004只关闭对应自有复位问题，IO/MTBF/板上仍未验收。

**正式renderer/engine尚未替换。本轮新候选在closure_candidate_v1，勿边跑工具边覆盖。** 旧正式cache本地RAM反馈已正，但tag-hit ready到sampler/map CE仍-0.232ns；另有affine终点到burst decode -0.563ns、DDR读valid到自有pixel -0.149ns，共29自有负路径。全局bounded报告漏掉它们。必须用本轮relative_review_v2聚合六份同DB定向报告；单独relative_review_v1不可用于打包审查。已生成diagnostic_package_v1，明确FAILED_OWN_TIMING，SCBV未设置、板上/30fps未验收。

候选增加每帧affine终点寄存、DDR beat寄存（rawvalid立即撤请求保留）、146bit两槽cache请求寄存ready缓冲。成熟渲染20用例共57600字=921600像素等效对照通过；两眼640x24校正/raw共61440像素/170875检查通过；slice9913接受=9872输出+41reset/flush丢弃、II1通过。8ns组件renderer setup+2.211/hold+.103ns；真实sampler+map+cache链setup+.721/hold+.116ns。组件不代替整机或全高候选测试。v1探针inside保留字失败、slice v1采样delta错误均仅测试外壳，正确结果是v2，失败证据保留。

目前后台任务必须先收集：
1. closure_candidate_full_pnr_v1：本轮原Top完整1/0/0 Compile/Synth/Map/PNR/STA/bitgen，runner14508/PDS9452；146设计源/5输入。通过--engine/--renderer仅替换自有模块，加request slice，不复制Top、不改原PDS。仅原FDC+上轮cdc_reset_candidate_v4+vendor_ddr_candidate_v2；不用旧cell-wide reset。
2. shared_full_profile_v1：本轮正式旧版本唯一640x720计数实验，runner25632/ModelSim7356。它仍冻结32源；未结束不再启动任何同类全高测试，不把当前正式文件提前改成候选。
3. paced_60_30_band_v1：本轮候选24行图带，显示60/相机30平均带宽情境，仅测试工具限速、RTL不变。收集状态后登记完成，不作为全帧/板上FPS。

工具身份、时间和hash看各status.json、closure_evidence_v1.json及事件，PID需要同时核对启动时间/目录。长工具输出允许独立完成，不能覆盖目录。

P3性能：旧/当前已收集全帧在连续竞争行为DDR下校正4522683周期=36.181464ms，模型27.638fps，不能当板上上限。现有压力bench显示和相机无间隔发请求，非固定30fps流量。新增--display-hz/--camera-hz成对参数只改变生成bench；60/30情境实际略高60.021/30.037Hz，125MHz周期按16字/4字burst量化，无FIFO/VS/曝光模型。先用图带核对再在唯一全帧结束后作必要全高情境。离线cache地址模型与两眼图带profile计数完全一致；全高预测缺失27183/26486，其中重复仅12/16，扩容收益很小，不盲目增加cache。完整计数尚待原生结果核对。

54项Python门禁回归及队友边界审计通过，仅Top/PDS两个获准基线文件有差异。对焦配置不能证明过去镜头机械位置，FOCUS001仍capture_focus_unverified。P4不启动，旧六小时2仍暂停。下一轮先收集两个长任务，候选原Top定向STA及全高候选像素通过后才考虑正式应用；一旦应用新增request slice需加入原PDS，不维护第二Top。

以下为历史入口；当前事实以最新事件和本段对应证据为准。

## 最新续做入口（2026-10-23凌晨，事件708快照）

先读最新events/checkpoint和runs/20261023_002811_6054d8a5/本轮修复断点.md。**正式cache预读及按域复位已应用，不要再次拷贝补丁或回退REV001/002。** cache正确SHA=a73fa353fa91d3b5bb08a8157409d1d32f28e1001c4106d8038ac3eea4c85687。用cache_candidate/manifest.raw_bytes.json，初版manifest.json预写出SHA错误保留。原Top仅四个自有用途邮箱替换，PDS新增自有stereo_event_cdc，成熟模块未改。

已通过cache逐周期/独立oracle/epoch回卷/II1、邮箱6相位含旧协议差分、相机6组停钟/reset、帧租约、170875检查图带及19项审查门禁测试。reset_top_binding_v2已经完整Compile/Synth，实际1/1/0，输入未变，补充对象0错误/78原告警。camera各只D0，20组40个reset C/P，级间D/CLK保留。v3失败不要用；经小组件和真实Top确认v4字面request_sync*/D[0]，内部regexp失败原因未知不猜。

当前仅两项真实后台：full_cache_reset_alg1_aux0_v1（原Top完整新1/0/0构建，runner10536/PDS12864）及reset_shared_full_v1（runner25664/ModelSim29744）。源/输入保持冻结。先检查每项status.json、PID/启动时间/目录，收集原生命令最终结果与哈希，不重复启动，不覆盖输出；准备task.json不是另一个新任务。辅助图带profile_shared_band_v1已通过，不再等待。

新完整流程原FDC + 本轮cdc_reset_candidate_v4 + 旧vendor_ddr_candidate_v2；**不加旧cell-wide REV001 reset FDC**，不用Map单独恢复DDR。同步器仅逻辑C/P异步断言例外，不能切D/CLK或本域恢复/移除。新PNR若bounded报告缺自有端点，追加目标报告，不能据缺席宣称收敛；report_existing_timing现在支持组件/Top报告名，但只接受已完成、无lock的新PNR数据库。

新打包门禁对自有same-clock/reset/cross/unknown负路径都保留FAILED_OWN_TIMING；原诊断位流仍是旧失败版本，不能打包成新修复版。TIM003/004尚待新整机STA，实物/30fps尚无通过证据。P4不启动。performance_budget_v1及snapshot_cadence_v1仅模型诊断，15Hz不是实测；测试grant直连request未模拟真实VS。--profile新观测工具经图带像素回归，必要时等当前全帧结束再查全高性能，勿重复旧无效任务。

下面为旧轮入口，当前状态以本段和最新事件为准。

## 最新续做入口（2026-10-22 晚间）

先读events最后checkpoint和runs/20261022_192855_fc368b41/修复断点.md。本轮完整1/0/0构建已结束，20:38:26真实bitgen成功；没有待收集后台。诊断包在同轮diagnostic_package_v2，SHA与源码/构建输入均核验，但相对基线FAILED_OWN_TIMING。TIM-003自有缓存标签RAM反馈-1.083ns，TIM-004自有发布邮箱复位-4.854ns，不能按厂商基线忽略或当P3通过。下一轮先对pair_cache_select_v2的RAM输出→hit/ready→RAM CE/地址反馈做独立流水化/差分/8ns时序，再修按域复位，最后才重新完整PNR；不要直接重复一小时全流程。正式缓存/复位RTL本轮未改。旧full_alg1_aux0_bitgen_v2已中断且无bit，旧running状态不再等待。

使用scripts/run_pds_board_v2.py的--extra-constraint及--bitgen；原run_pds_board.py仍是历史入口，不必为了本次构建再应用旧extra_constraints.patch。完整构建后可用package_board_candidate.py校验参数/输入/完成标记并生成P3测试包；STA、相对基线、实板/FPS仍分开记录。

外审关于“二值化失效”不完整：最终成熟stereo_ui_layer仍将CMP右半二值化；本轮真实UI回归11520像素通过。CMP只能检几何，LIGHT彩色效果在STEREO检查。队友UI不改。上板检查清单.md包含实际按键/菜单及未完成项。

原队友目录prj/report_timing/Top.rtr确认为10月21日15:05的PG2L200H报告，native编译没有新开关；它与后来0/0集成Top报告不是同一基线。报告/bit指纹见本轮baseline_identity.json，编译期完整源码和用户实板所用文件身份尚无独立证明。原bitgen还有SCBV未设置告警，新bit生成时须保留并检查，不能凭原版也有就当配置电压已核对。复位同步器现有v2约束为cell范围，会同时排除其级间D时序；相对审查须披露，不能冒充全STA签核。

## 外部审查（2026-10-22，Claude，用户要求）

先读 [外部审查_Claude_20261022.md](外部审查_Claude_20261022.md)。该轮修改了2个自有RTL（display_read_descriptor_bridge.v、ddr_aux_read_mux.v）和2个bench（tb_aux_mux.sv、tb_cdc_phase.sv），已登记为REV-001/REV-002并有回归证据和原文件备份（runs/20261022_125232_699f019d/originals），不是工程出错，不要回退。队友10_15_2文件未被该轮改动。整机STA全过不再作为上板前门槛，改用文中“相对基线”标准；下一目标是生成比特流交用户上板。

# 双目集成续做规则

先读PLAN.md和state/events.jsonl（唯一事实源），再读当前进度和问题视图。使用scripts/run_state.py取得自己的run_id/token；15分钟租约，至少每3分钟续约。每个修改/工具启动前先写step_begin，完成后step_end。丢失租约立即停止，不能使用其他轮次的令牌。
固定每轮90分钟，前80开发/后10收尾；额度可查且低于10%提前结束，查不到正常继续。所有生成物留本目录，临时工具独立目录，不能写旧工作区。不要执行旧环境pth或旧规划控制脚本。
不复制维护Top.v。通用边界是原Top参数/实例接线和PDS源码清单；CMP本轮明确例外另可修改stereo_ui_controller.v与stereo_ui_layer.v。其他成熟模块禁止修改。P3实板赛题验收前不开发测长功能。原始照片只读。保留失败日志和对应源码指纹。仿真/PDS/实板分开报告。

PDS验收先查compile/run.log中的实际ENABLE_ALGORITHM/ENABLE_AUX_READ及实例，不能把请求参数或add_design宏当作已生效证据。PDS -defined_parameters 使用交替名称/值列表（不是NAME=value）。真正启用整机必须重新综合/PnR，首轮旧PDS实际关闭两开关。审核同时读取.rtr与工具日志中的未约束/忽略约束警告；定向报告若STA-3016对象未找到、过滤被忽略，必须判为诊断失败，不得标通过。长工具运行期间禁止修改其仍待读取/复核的设计输入。ModelSim已经编译并封存指纹的旧快照可以独立跑完，但源文件若改变，其输出只能标旧快照证据，不能证明修改后版本；旧任务仍运行时不重复启动同类全帧任务。

原生report_timing的input_db_file只接受PNR阶段，不能用Top_syn.adf反复尝试。生成器内重复的endmodule文本不能作宽补丁锚点，修改后先对自有Python脚本做AST解析再启动工具。双视图单帧测试须分别在各自ready后撤销source_valid，避免快的一眼启动第二帧导致计数误判。旧全帧仿真快照若仍在运行先收集，不重复启动，也不批量终止进程。

组合方案已在20261022_104801_3ed91b5d按指纹正式应用；先读最新checkpoint和实施与验证报告，不要再次执行旧候选拷贝。runs/20261022_055435_029654e1/候选应用说明.md仅保留为历史指纹和试验来源。生成候选时用read_text统一内存换行，但SHA仍取原始bytes；不因CRLF匹配失败放宽唯一锚点。坐标数组用mapped_x/mapped_y等名称，不能覆盖标量图带偏移。组件时序探针须显式验证实际USE_AFFINE=1；单组件内部Setup/Hold/Pulse结论不能冒充整机IO/复位签核，完整整机审计仍要求十类摘要及工具警告检查。原生进度文件只有$fflush，不等同events的fsync；旧快照没有进度文件时不能据stdout未刷新判定卡死。

输入驱动必须在采样沿记住valid&&ready的接受结果，不能在另一沿依据已变化的ready判断是否重复/保持旧输入。新工具将bench复制到独立任务目录后编译并保存指纹。PDS原生pnr支持-input_db_file从已完成Top_map.adf续做；run_mapped_pnr.py强制检查Map完成/实际1/1/0参数/源及DB锁状态。Fabric shell无set_max_fanout命令，不能猜用；综合有fanout_guide，优化选项以原生帮助为准。

重要更正（ENV-008）：该Map续做未通过DDR整机。combined_mapped_pnr_v1/v2/v3均Place-0006，原FDC登记和原生PCF复制仍不足以恢复完整工程物理上下文；小组件的成功不能推广。禁止重复这条失败路径，后续使用run_pds_board.py完整Compile→Map→PNR。20261022_055435轮的fallback因80分钟预算保护未启动，后续状态见下段。Public-4024锁定忽略的优化旗不能算生效，失败目录均保留。

最新更正：完整fallback已在20261022_104801_3ed91b5d启动并进入布线，先收集，不能重复启动。add_constraint后对旧PNR库report_timing并未覆盖DB里的原约束（ENV-009）；禁止按退出0宣称新约束通过。DDR最终报告中的自动推导时钟名在综合检查时还不存在，应使用经实际Top匹配的显式声明候选。候选匹配与原FDC物理告警分别记录，原FDC的78项告警为36项share pin与42项DRIVE/SLEW建议；不能把局部匹配通过称为全局签核。

build_tool_candidate/extra_constraints.patch尚未应用，必须等仍使用原run_pds_board.py的任务结束、check所有权并核对base SHA后再应用；新完整流程保留原FDC，附加约束必须纳入输入哈希。P3前不启用测长。原Top辅助请求目前默认不发起，板上主动辅助读尚未验收；不能只按ENABLE_AUX_READ=1宣称该硬件流量已测过。
