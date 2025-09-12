# ACT vs SmolVLA 推理速度基准测试

该目录包含用于比较 ACT、SmolVLA（以及可选的 Diffusion）策略推理速度的脚本。

## 脚本说明

### 1. `simple_benchmark.py` - 简单基准测试
使用合成数据测试策略的推理速度。

**特点：**
- 使用合成数据，不依赖真实数据集
- 运行快速，适合快速验证
- 无需下载大型数据集
- 支持“稳态（steady-state）”基准，更贴近实时控制使用方式

**使用方法：**
```bash
python simple_benchmark.py
```

## 输出说明

脚本会打印：

1. **设备信息**：使用的计算设备（CPU/GPU）
2. **策略加载状态**：模型是否成功创建
3. **推理时间统计（稳态）**：
   - Mean Time：所有调用的平均延迟
   - Median Time：所有调用的中位延迟
   - Min/Max Time：最小/最大单次延迟
4. **速度比较**：ACT 相对于 SmolVLA（以及 Diffusion）的速度对比
5. **成功次数**：成功推理的调用次数
6. **稳态细节**：
   - Step Mean (ms)：“普通步骤”延迟（从已生成的动作队列中弹出一个动作）
   - Refill Mean (ms)：“补充步骤”延迟（当动作队列为空时，策略需要生成一批新动作）
   - Step/Refill Count：统计两类步骤的出现次数

为什么要区分 Step 和 Refill？
- 很多策略内部会缓存一小段未来动作。在稳态控制中，大多数步只是弹出一个动作（开销小）；每隔一段时间需要生成一批新动作（开销大）。同时报告两者，可以同时看到“典型控制周期延迟”和“最坏情况尖峰”。

## 示例输出（节选）

```
Using device: mps
Creating policies...
✓ ACT policy created
✓ SmolVLA policy created
✓ Diffusion policy created

======================================================================
INFERENCE SPEED BENCHMARK RESULTS
======================================================================
Metric               ACT (ms)     SmolVLA (ms) Diffusion (ms)   ACT/SmolVLA  Diff/SmolVLA
----------------------------------------------------------------------
Mean Time            0.30         14.75        418.24           0.02x       28.36x
Median Time          0.30         2.58         2.03             0.11x       0.79x
Min Time             0.29         2.50         1.74             0.11x       0.70x
Max Time             0.31         601.12       3478.52          0.00x       5.79x
----------------------------------------------------------------------
Successful Runs      50           50           50
Step Mean (ms)       0.30ms       2.78ms       3.29ms
Refill Mean (ms)     -            601.12ms     3461.20ms
Step/Refill Count    50/-         49/1         44/6
```

解读建议
- 以 Median/Step Mean 作为每个控制周期的典型延迟参考。
- Refill Mean 与 Max Time 反映需要“补充动作队列”时的最坏情况。该指标也反应了模型进行预测时的速度有多快（或者有多慢）

## 注意事项

1. **首次运行**：第一次运行可能会下载预训练模型和数据集，耗时较长
2. **内存要求**：SmolVLA 与 Diffusion 体量较大，需要更多 GPU 显存
3. **设备选择**：尽量使用 CUDA GPU 以获得更具代表性的结果
4. **数据兼容**：不同策略可能需要不同输入格式，脚本已做自动处理

### 快速诊断

运行下述命令验证策略创建与基本推理：

```bash
python verify_benchmark_setup.py
```

## 依赖要求

安装依赖：
```bash
pip install torch torchvision
pip install lerobot
pip install datasets
pip install transformers
```

SmolVLA 额外依赖：
```bash
pip install -e ".[smolvla]"
```
