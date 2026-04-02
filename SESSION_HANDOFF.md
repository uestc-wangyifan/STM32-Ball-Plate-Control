# STM32 Ball Plate Control — 当前会话交接摘要

## 1. 项目路径
- Windows 工程目录：
  `D:\EmbedDev\01_Workspace\stm32_project\Ball_Control_V1`
- 当前实际编译/烧录的还是这个原工程目录
- 只是当前 Git 分支已经切到：
  `codex/session-1`

## 2. 当前 Git 状态
- 当前分支：`codex/session-1`
- 本地有备份分支：
  `backup/pre-codex-20260401`
- 本地有标签：
  `pre-codex-20260401`
- 本地有 bundle 备份：
  `/mnt/d/EmbedDev/01_Workspace/stm32_project/STM32-Ball-Plate-Control-pre-codex.bundle`

## 3. 已完成的重要提交
最近关键提交如下：

- `d2940ed` fix: close rx_flag race window in UART parse path
- `23802f2` feat: enforce center-before-target flow for task 5
- `cd3a4bd` feat: make task 6 configurable and >10cm
- `f52c66b` fix: ensure safe centering on timeout/lost
- `d93f143` chore: add project codex config

## 4. 这次已经改过的内容
### 已处理
1. **失联/超时后的安全行为**
   - 修了 `Core/Src/main.c`
   - 超时后不再提前 return
   - 可以真正落到舵机归中逻辑
   - LOST / invalid 计数阈值由 `>` 改为 `>=`

2. **Task 6**
   - 现在不是固定 100mm 了
   - 改成“两次点击定义 A/B 点”
   - 长度必须 **严格大于 10cm**
   - Task 6 实际运行使用当前配置的 A/B 点

3. **Task 5**
   - 增加了“先中心平衡，再接受裁判目标”的流程约束

4. **rx_flag 竞争窗口**
   - 针对 `main.c` 里串口解析路径做了最小补丁
   - 目的：避免同一帧被重复处理

## 5. 当前阶段明确不处理
- **Task 7 暂不处理**
- 原因：当前机械结构暂时不支持
- 后续 AI 不要把 Task 7 继续列为当前第一优先级

## 6. 当前仍可能存在、但暂未继续处理的问题
这些不是本轮重点，但可能以后还要看：

- Python 发送节拍仍然依赖主循环，存在抖动风险
- T 帧仍不是强一致协议，仍可能存在目标不同步风险
- 协议层没有 ACK / 重发 / 帧序号 / 重放保护
- 文档（例如 CLAUDE.md）可能仍过期
- 没有做完整工业级串口高压测试

## 7. 今天结束时的建议状态
- 直接基于 `codex/session-1` 分支编译、烧录、上板测试
- 编译时仍然是打开原来的工程目录
- 不是去打开某个单独源码文件烧录
- 只要当前目录分支是 `codex/session-1`，编出来的就是修改后的版本

## 8. 下次上来先做什么
先执行：

```bash
git branch --show-current
git log --oneline -n 5
git status
#rules
你现在只基于当前最新仓库状态工作，不要沿用历史旧结论。
当前阶段：

Task 7 暂不处理，因为机械结构不支持
已经做过 4 个修改：
失联/超时安全归中
Task 6 改成可指定且严格大于 10cm
Task 5 增加先中心平衡再接受目标
rx_flag 竞争窗口最小补丁
请先重新读取当前代码，再判断：
哪些问题已经不用管
哪些问题还值得继续修
不要长篇复读旧审计
不要重构
只接受最小补丁
不允许修改 .ioc / .ld / openocd / Makefile
不允许新建或删除文件
不允许 git push / git reset / git clean
11. Codex 项目配置

仓库里已经加了 .codex/config.toml，目的是减少频繁确认。

当前配置思路：

approval_policy = "never"
sandbox_mode = "workspace-write"
network_access = false
12. 备注
这轮主要目标不是“工业级完美”，而是先把训练题里最值得的软件问题收掉
题目里最相关的已处理重点是：Task 5、Task 6、失联安全

你也可以用更短的方式保存成这句：

```text
当前分支是 codex/session-1，已经修了失联安全、Task 6、Task 5、rx_flag 竞争窗口；Task 7 先不管。