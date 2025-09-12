# ACT vs SmolVLA Inference Speed Benchmark

This directory contains scripts to compare the inference speed of the ACT, SmolVLA, and (optionally) Diffusion policies.

## Scripts

### 1. `simple_benchmark.py` - Simple Benchmark
Uses synthetic data to test the inference speed of policies.

**Highlights:**
- Uses synthetic data; no real dataset required
- Fast to run; suitable for quick checks
- No large dataset downloads needed
- Includes steady-state benchmarking to reflect real-time control usage

**Usage:**
```bash
python simple_benchmark.py
```

## Output

The script prints:

1. **Device info**: The compute device used (CPU/GPU)
2. **Policy loading status**: Whether models load successfully
3. **Inference time statistics** (steady-state):
   - Mean Time: average per-call latency across all steps
   - Median Time: median per-call latency
   - Min/Max Time: per-call extremes
4. **Speed comparison**: ACT speed relative to SmolVLA (and Diffusion if present)
5. **Success rate**: Number of successful inference calls
6. **Steady-state details**:
   - Step Mean (ms): typical per-step latency when dequeuing a pre-generated action
   - Refill Mean (ms): latency when the policy must generate a new action chunk
   - Step/Refill Count: how many of each occurred during the run

Why Step vs Refill?
- Many policies internally cache a small queue of future actions. In steady-state control, most steps pop a single action (cheap). Periodically, the policy generates a new chunk (expensive). Reporting both reveals typical cycle time and worst-case spikes.

## Example Output (abridged)

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
Mean Time            0.53         14.72        446.89           0.04        x 30.36       x
Median Time          0.28         2.58         2.03             0.11        x 0.79        x
Min Time             0.27         2.36         1.75             0.12        x 0.74        x
Max Time             31.07        633.19       3763.59          0.05        x 5.94        x
----------------------------------------------------------------------
Successful Runs      500          500          500             
Step Mean (ms)       0.53ms       2.75ms       2.24ms          
Refill Mean (ms)     -            601.19ms     3531.22ms       
Step/Refill Count    500/-            490/10           437/63              

======================================================================
SUMMARY
======================================================================
ACT vs SmolVLA: faster (27.84x)
Diffusion vs SmolVLA: slower (30.36x)
ACT mean inference time: 0.53ms
SmolVLA mean inference time: 14.72ms
Diffusion mean inference time: 446.89ms
```

Interpretation tips
- Use Median/Step Mean as a proxy for per-control-cycle latency.
- Refill Mean and Max Time indicate worst-case spikes when generating a new action chunk. This metric will tell you how fast(or slow) the model runs.

## Notes

1. **First run**: The first run may download pretrained models and datasets; this can take time
2. **Memory requirements**: SmolVLA and Diffusion are larger and need more GPU memory
3. **Device selection**: Use CUDA GPU if available for more representative results
4. **Dataset compatibility**: Policies may require different input formats; the script handles this automatically

### Quick Sanity Check

Run the following to verify policy creation and basic inference:

```bash
python verify_benchmark_setup.py
```

## Requirements

Install dependencies:
```bash
pip install torch torchvision
pip install lerobot
pip install datasets
pip install transformers
```

For SmolVLA extras:
```bash
pip install -e ".[smolvla]"
```
