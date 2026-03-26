# 无模型评分规则说明

## 1. 文档目的

本文档详细说明当前无模型评分器的评分规则。

对应脚本：

- `C:\Users\mizhiaishang\codex_runs\auv_0325\cnp_psi_model\csv_quality_scorer_nomodel.py`

这个评分器的目标不是调用已有训练模型后再判断，而是：

- 只看 CSV 本身
- 不使用 checkpoint
- 直接判断这段数据对当前 `yitr -> r` 建模任务是否友好


## 2. 当前任务背景

当前评分器面向的任务是：

- 输入：`yitr`
- 输出：`r`

也就是判断一段 CSV 中：

- 舵角 `yitr`
- 航向角速度 `r`

这两条信号是否具有足够的建模价值。


## 3. 支持的输入格式

评分器支持两类 CSV。

### 3.1 `norm_traj_data_*.csv`

这是模型直接使用的归一化数据。

脚本会：

- 读取 `r`
- 读取 `yitr`
- 按 `norm_config.py` 反归一化为真实物理量


### 3.2 `traj_data_*.csv`

这是原始轨迹数据。

脚本会：

- 读取 `r`
- 读取 `yitr`
- 自动去掉前 `50` 行暖机段


## 4. 基础工具函数

### 4.1 `clip01`

```text
clip01(x) = 将 x 截断到 [0, 1]
```

作用：

- 保证所有子分数最终都在 `0~1`


### 4.2 `scale_score`

```text
scale_score(value, low, high) =
clip((value - low) / (high - low), 0, 1)
```

含义：

- 当 `value <= low` 时，得分为 `0`
- 当 `value >= high` 时，得分为 `1`
- 中间线性变化


## 5. 原始统计特征

评分器先从 CSV 中提取以下原始特征。

### 5.1 输入激励相关

- `yitr_std_deg`
  `yitr` 的标准差，单位 `deg`
- `yitr_range_deg`
  `yitr` 的极差，单位 `deg`


### 5.2 输出响应相关

- `r_std_deg_s`
  `r` 的标准差，单位 `deg/s`
- `r_range_deg_s`
  `r` 的极差，单位 `deg/s`


### 5.3 周期和耦合相关

- `estimated_cycles`
  估计有效周期数
- `max_lagged_corr`
  `yitr` 与 `r` 在允许时滞下的最大相关系数
- `best_corr_lag`
  达到最大相关系数时的时滞


### 5.4 噪声和异常相关

- `yitr_high_freq_ratio`
  `yitr` 高频能量占比
- `r_high_freq_ratio`
  `r` 高频能量占比
- `yitr_spike_ratio`
  `yitr` 尖峰比例
- `r_spike_ratio`
  `r` 尖峰比例
- `rudder_saturation_ratio`
  `|yitr_deg| > 35` 的比例


## 6. 4 个核心子分数

当前版本已经移除了：

- `row_score`
- `cycle_score`

真正参与主评分的只有 4 项。


### 6.1 `excitation_score`

表示输入激励强不强。

公式：

```text
excitation_score =
0.5 * scale(yitr_std_deg,   0.4,  2.5) +
0.5 * scale(yitr_range_deg, 1.5, 10.0)
```

解释：

- `yitr` 的波动太小，会低分
- `yitr` 的整体摆动范围太小，也会低分


### 6.2 `response_score`

表示输出响应明显不明显。

公式：

```text
response_score =
0.5 * scale(r_std_deg_s,   0.25, 1.0) +
0.5 * scale(r_range_deg_s, 1.0,  4.0)
```

解释：

- `r` 响应太弱，低分
- `r` 动态范围太小，低分


### 6.3 `coupling_score`

表示 `yitr` 和 `r` 的耦合关系清不清楚。

公式：

```text
coupling_score = scale(max_lagged_corr, 0.30, 0.75)
```

解释：

- 低于 `0.30` 时认为耦合很弱
- 到 `0.75` 及以上时认为耦合较清楚

说明：

- 这项在旧版本里过于严格，后来已放宽


### 6.4 `cleanliness_score`

表示数据干不干净、平不平滑。

公式：

```text
cleanliness_score =
(
  1 - clip(yitr_high_freq_ratio / 0.35) +
  1 - clip(r_high_freq_ratio / 0.35) +
  1 - clip(yitr_spike_ratio / 0.05) +
  1 - clip(r_spike_ratio / 0.05) +
  1 - clip(rudder_saturation_ratio / 0.10)
) / 5
```

解释：

- 高频噪声太多会扣分
- 尖峰太多会扣分
- 饱和太严重会扣分


## 7. 基础分 `base_score`

当前基础分公式是：

```text
base_score = 100 * (
  0.30 * excitation_score +
  0.35 * response_score +
  0.15 * coupling_score +
  0.20 * cleanliness_score
)
```

### 7.1 权重含义

- `response_score` 权重最高：`0.35`
- `excitation_score` 次高：`0.30`
- `cleanliness_score` 中等：`0.20`
- `coupling_score` 最低：`0.15`

### 7.2 设计原因

当前这组权重的想法是：

- 真实模型对 `response` 和 `excitation` 更敏感
- `coupling` 重要，但不能再像早期版本那样过度主导
- `cleanliness` 是稳定的辅助项


## 8. 温和提分项 `uplift_bonus`

为了让无模型评分整体更接近真实模型分数分布，加入了一个补偿项：

```text
uplift_bonus = 100 * (
  0.08 * excitation_score +
  0.06 * response_score +
  0.04 * coupling_score +
  0.02 * cleanliness_score
)
```

作用：

- 不是替代基础分
- 而是在基础分上做一个温和上调
- 用来修正“无模型评分整体偏低”的问题

然后得到：

```text
pre_penalty_score = min(100, base_score + uplift_bonus)
```


## 9. 两个惩罚项

为了防止某些异常数据被误判为高分，保留了两个惩罚项。


### 9.1 `sharp_signal_penalty`

先定义：

```text
total_spike = yitr_spike_ratio + r_spike_ratio
```

如果：

```text
best_corr_lag <= 4
```

则：

```text
sharp_signal_penalty = scale(total_spike, 0.18, 0.30)
```

否则为 `0`。

含义：

- 输入输出几乎同步
- 同时尖峰很多
- 这更像突变信号或方波边缘
- 因此需要惩罚


### 9.2 `excessive_cycle_penalty`

如果：

```text
best_corr_lag <= 4
```

则：

```text
excessive_cycle_penalty = scale(estimated_cycles, 12.0, 20.0)
```

否则为 `0`。

含义：

- 时滞很小
- 周期又非常密
- 可能更像高频过密信号
- 因此需要惩罚


### 9.3 惩罚后得分

```text
penalty_adjusted_score =
pre_penalty_score *
(1 - 0.30 * sharp_signal_penalty) *
(1 - 0.22 * excessive_cycle_penalty)
```


## 10. 对比度校准

这是目前最新一版里最关键的“贴近真实模型分数”的步骤。

用户的目标是：

- 低分要更低
- 高分要更高
- 中间分适度调整

因此加了这一层：

```text
contrast_pivot = 45
contrast_gain = 1.18
```

最终总分：

```text
total_score =
clip(
  45 + 1.18 * (penalty_adjusted_score - 45),
  0,
  100
)
```

### 10.1 含义

- 以 `45` 为中轴
- 高于 `45` 的分数会被放大
- 低于 `45` 的分数会被压低
- 因此能实现“低的更低，高的更高”

### 10.2 作用

这一步不是简单整体提分，而是增强分布的对比度，使无模型分数更像真实模型分数的梯度。


## 11. 最终标签规则

```text
if total_score >= 80:
    label = "good"
elif total_score >= 60:
    label = "usable"
else:
    label = "poor"
```

同时：

```text
is_good_data = (total_score >= 80)
```


## 12. 文字原因 `reason`

评分器还会自动生成理由说明。

规则如下：

- `excitation_score < 0.4`
  输出：`输入舵角激励偏弱，信息量不足`
- `response_score < 0.4`
  输出：`输出响应幅值偏小，系统特征不明显`
- `coupling_score < 0.4`
  输出：`yitr 与 r 的耦合关系偏弱`
- `cleanliness_score < 0.4`
  输出：`高频噪声、尖峰或饱和现象偏多`

如果以上都不低，则输出：

- `激励、响应、耦合度和数据洁净度都较好`


## 13. 当前哪些量会真正影响总分

当前会直接影响总分的量有：

- `excitation_score`
- `response_score`
- `coupling_score`
- `cleanliness_score`
- `uplift_bonus`
- `sharp_signal_penalty`
- `excessive_cycle_penalty`
- `contrast calibration`


## 14. 当前哪些量只用于分析，不直接进入主评分

这些量会输出，但不再直接进入主分：

- `rows`
- `estimated_cycles`
- `best_corr_lag`
- 各种原始统计量

其中：

- `estimated_cycles` 虽然不再参与主分
- 但仍然参与 `excessive_cycle_penalty`


## 15. 汇总表中相关字段含义

在批量输出的 `summary.csv` 里，与无模型评分直接相关的字段包括：

- `nomodel_score`
  最终无模型总分
- `nomodel_base_score`
  核心 4 项加权得到的基础分
- `nomodel_uplift_bonus`
  温和提分项
- `nomodel_pre_contrast_score`
  惩罚后、对比度拉伸前的分数
- `nomodel_excitation_score`
  输入激励子分数
- `nomodel_response_score`
  输出响应子分数
- `nomodel_coupling_score`
  输入输出耦合子分数
- `nomodel_cleanliness_score`
  数据洁净度子分数
- `nomodel_sharp_signal_penalty`
  尖锐突变惩罚
- `nomodel_excessive_cycle_penalty`
  高频过密惩罚


## 16. 如何解释一条数据为什么分低

建议按以下顺序看：

1. `nomodel_response_score`
2. `nomodel_excitation_score`
3. `nomodel_coupling_score`
4. `nomodel_cleanliness_score`
5. `nomodel_reason`

如果这几项本身不低，但总分还是被压下来了，再看：

6. `nomodel_sharp_signal_penalty`
7. `nomodel_excessive_cycle_penalty`


## 17. 一句话总结

当前版本的无模型评分器，本质上是：

- 用 `excitation / response / coupling / cleanliness` 四个核心质量分做主评分
- 再加一个 `uplift_bonus` 解决整体偏低
- 再加两个惩罚项防止异常信号误判
- 最后通过一层 `contrast calibration` 实现“低分更低，高分更高”，让无模型评分分布更贴近真实模型评分。
