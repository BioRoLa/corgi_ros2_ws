# ESEKF 里程計估測失敗 — Debug 紀錄

## 1. 問題描述

原本使用 **Information Filter + 平均速度** 的腿部里程計可以正確估測機器人速度與位置，但移植到 **ES-EKF (Error-State Extended Kalman Filter) + 瞬時速度** 後無法正確估測。

### 主要改動項目

| 項目 | 舊系統 (legacy) | 新系統 (ESEKF) |
|------|----------------|---------------|
| **狀態表示** | `[v₁..vⱼ, ba]` 滑動視窗，維度 6J | `[p, v, q, ba, bw, bv]` 名義狀態 19 維 + 誤差狀態 18 維 |
| **運動學觀測** | 平均速度：J 步位移 / (J·dt) | 瞬時速度：`PointVelocity(v=0, ω)` 逐步更新 |
| **觸地判斷** | KLD (Kullback-Leibler Divergence) 排除 | GMO (General Momentum Observer) + Schmitt Trigger |
| **估測架構** | Information Filter (批次更新) | ES-EKF (序列更新 predict→update→inject) |

---

## 2. Debug 策略：逐步排查

由於四個改動同時引入，需要逐一排除。以下按照從底層到高層的順序排查：

```
運動學計算 → 觸地判斷 → 狀態表示 → 估測架構
     ↓
  ✅ 第一步（本次）
```

---

## 3. 第一步：運動學計算方式驗證

### 3.1 實驗設計

使用 `single_leg_contact_theta_beta.csv` 測試資料：
- **條件**：單足、單 cycle、全程觸地
- **資料**：每個時刻的 `theta, beta` 與對應的 ground truth `hip_vx, hip_vz`
- **Ground truth**：`vx ≈ 0.1 m/s`, `vz ≈ 0.0 m/s`
- **dt**: 0.001 s (1 kHz)

### 3.2 兩種計算方式

#### 平均速度 (Legacy)
```
displacement = travel(contact_beta arc) + compensate(rolling) + (first_pt - last_pt)
v_avg = displacement / (J × dt)
```
- `travel2()`: 累積 J 步的接觸點弧長位移
- `compensate2()`: 滾動補償（來自 theta_d 造成的 rim 旋轉）
- `first_pt - last_pt`: 接觸點位置變化修正

#### 瞬時速度 (ESEKF)
```
PointVelocity(v=0, ω, rim, alpha, inbody=true)
z_leg = -contact_velocity
```
- 基於 no-slip constraint：接觸點世界速度 = 0
- 設 v=0 得到機構速度分量，取負值得到 body velocity 觀測

### 3.3 測試結果

**比較腳本**: `kinematic_compare/compare_velocity_methods.py`

> 注意：CSV 中的 beta 需取負號以符合 body frame 慣例（與 C++ 中右側腿 negate beta 一致）。

#### 數值結果 (中間點 t=1.2s)

| 方法 | vx [m/s] | vz [m/s] |
|------|----------|----------|
| Ground Truth | +0.100000 | 0.000000 |
| 瞬時速度 | +0.097241 | +0.000068 |
| 平均速度 (J=5) | +0.102220 | -0.000000 |
| 平均速度 (J=10) | +0.102216 | -0.000000 |
| 平均速度 (J=20) | +0.102208 | -0.000000 |

#### RMSE 統計

| 方法 | RMSE_vx | RMSE_vz |
|------|---------|---------|
| 瞬時速度 | 0.00287 m/s | 0.00041 m/s |
| 平均 (J=5) | 0.00237 m/s | 0.00003 m/s |
| 平均 (J=10) | 0.00237 m/s | 0.00003 m/s |
| 平均 (J=20) | 0.00237 m/s | 0.00003 m/s |

#### 平均速度統計

| 方法 | mean vx | mean vz |
|------|---------|---------|
| Ground Truth | 0.100000 | 0.000000 |
| 瞬時速度 | 0.097307 | 0.000024 |
| 平均 (J=5) | 0.102096 | -0.000004 |

### 3.4 關鍵發現

1. **兩種方法的精度都很高**：RMSE 都在 ~3 mm/s 以內。

2. **瞬時 vs 平均對比**：
   - 平均速度更平滑（vz noise 小一個數量級）
   - 瞬時速度略有波動（RMSE_vx 2.87 vs 2.37 mm/s，RMSE_vz 0.41 vs 0.03 mm/s）
   - 但差異在噪聲範圍內，不足以導致 ESEKF 發散

3. **瞬時速度偏低約 2.7%，平均速度偏高約 2.1%**：
   - 瞬時速度 mean = 0.0973，GT = 0.1（偏低）
   - 平均速度 mean = 0.1021，GT = 0.1（偏高）
   - 可能是有限差分計算 theta_d, beta_d 的誤差

4. **平均速度對 J 不敏感**：因為 ground truth 近乎常數，J=5,10,20 結果一致。

### 3.5 結論

> **運動學計算方式的改變（平均 → 瞬時）本身不是 ESEKF 失敗的根本原因。**
> 
> 瞬時速度的計算在數學上是正確的，與平均速度的精度差異在噪聲範圍內。
> ESEKF 估測失敗的原因應在其他改動中。

---

## 4. 可能的問題根源與後續排查方向

### 4.1 觀測噪聲特性差異（高優先級 ⭐⭐⭐）

**問題**：瞬時速度比平均速度 noisier（特別是 vz 增加 ~10 倍），若 ESEKF 的觀測噪聲 `R_leg` 設定過小，會過度信任噪聲觀測。

**現狀**：
- `noise_.sigma_leg = 1e-3` (ESEKF)
- Legacy: `dM = (0.01, 0.01, 0.3, 0.3)`, `Q = I × 2.5e-7`

**排查**：
- 比較兩系統的觀測噪聲設定
- 嘗試增大 `sigma_leg` 觀測噪聲
- 檢查 process noise 與 measurement noise 的比例是否合理

### 4.2 觸地狀態判斷（高優先級 ⭐⭐⭐）

**問題**：GMO + Schmitt Trigger 可能比 KLD 產生更多錯誤的觸地判斷。

**排查**：
- 記錄觸地狀態並與 GT / KLD 結果比較
- 錯誤的觸地判斷會注入完全錯誤的速度觀測，可能導致 ESEKF 發散
- Schmitt Trigger 的閾值（`RM_THRESHOLD`, `BETA_THRESHOLD`）需要校正

### 4.3 Bias 狀態初始化與收斂（中優先級 ⭐⭐）

**問題**：ESEKF 新增了 `bw` (gyro bias) 和 `bv` (velocity bias)，這些偏置的初始協方差和隨機遊走噪聲如果設定不當，可能導致偏置發散。

**現狀**：
- `sigma_bw = 1e-5`, `sigma_bv = 1e-6`
- 初始 P 對 bias 為 `1e-4`

**排查**：
- 監控 `ba, bw, bv` 的收斂行為
- 確認偏置是否合理收斂或持續增長

### 4.4 ESEKF 數值穩定性（中優先級 ⭐⭐）

**問題**：
- `S.inverse()` 可能不穩定 → 考慮使用 Cholesky 分解
- Error state inject 後未做完整的協方差 reset（目前跳過 G matrix）
- 協方差矩陣可能漸漸失去對稱正定性

**排查**：
- 監控 P 矩陣的特徵值
- 加入對稱化 `P = (P + P^T) / 2`
- 實作完整的 G matrix reset

### 4.5 IMU 預測步驟中的重力補償（中優先級 ⭐⭐）

**問題**：ESEKF predict 中使用 `g_body = R_body^T * g_world` 在 body frame 補償重力。如果 IMU 方向不正確或初始化有誤，會導致加速度偏差累積。

**排查**：
- 確認初始 quaternion 是否正確
- 比較 body frame 下的 a_corrected 與期望值

### 4.6 Body frame vs World frame 速度定義（低優先級 ⭐）

**問題**：Legacy 系統在 body frame 估計速度，ESEKF 也在 body frame 估計速度，但 predict 步驟涉及座標轉換。如果 velocity 的 frame 不一致，會導致 position 積分錯誤。

**排查**：
- 確認 `v` 在 predict 和 update 中的 frame 定義一致
- 確認 `p += R * v * dt` 是否對應正確的轉換

---

## 5. 第二步：觸地判斷驗證 (Schmitt Trigger Tuning)

### 5.1 實驗設計

使用 `output_data/walk_3m_01m.csv` 錄製資料（行走 0.1 m/s 前進 3m）：
- **資料**：31105 筆 (31.1s, 1kHz)，有效欄位為 motor state, imu, sim_pos
- **Force / dst 欄位為空**，無 ground truth 接觸力
- **驗證方式**：以 sim_pos 有限差分求 GT 速度，比較不同觸地判斷下的腿部里程計速度估測 RMSE

### 5.2 Disturbance Observer 產出

使用 `offline_test` 產生 disturbance 估測 (`walk_3m_01m_result.csv`)：
- **觀察者架構**：General Momentum Observer (GMO), cutoff=15Hz, γ=0.91
- **輸出**: 12 維 disturbance vector → 每腿取 `rm` (法向力) 和 `beta` (扭矩)
- **Index 對應**: beta_a=4, rm_a=5, beta_b=6, rm_b=7, beta_c=8, rm_c=9, beta_d=10, rm_d=11

#### 穩態訊號統計（跳過前 500 筆 transient）

| 訊號 | 平均 |rm| | std |rm| | max |rm| | 平均 |β| | std |β| | max |β| |
|------|---------|---------|---------|---------|---------|---------|
| Leg A (LF) | 57.2 | 41.6 | 138.8 | 1.52 | 1.55 | 7.35 |
| Leg B (RF) | 57.8 | 42.2 | 139.3 | 1.64 | 1.79 | 8.17 |

> **觀察**：rm 值持續 >25（行走時四足幾乎總在接觸），beta 值較小（mean ~1.5, max ~8）。

### 5.3 Schmitt Trigger 測試結果

**測試腳本**: `src/TEST/contact_detection/tune_schmitt_trigger.py`

#### 8 種閾值配置比較

| Config | 邏輯 | rm_H | rm_L | β_H | β_L | RMSE_vx | 平均觸地腿數 | Coverage | 各腿觸地率 |
|--------|------|------|------|-----|-----|---------|-------------|----------|-----------|
| **Current OR (rm=25,β=10)** | OR | 25 | 15 | 10 | 1 | **4.5 mm/s** | 2.91 | 100% | 70/71/75/75% |
| AND rm=50,β=2 | AND | 50 | 25 | 2 | 0.5 | 4.7 mm/s | 2.39 | 100% | 59/57/61/62% |
| AND rm=40,β=2 | AND | 40 | 20 | 2 | 0.5 | 5.3 mm/s | 2.45 | 100% | 59/61/61/63% |
| AND rm=25,β=2 | AND | 25 | 15 | 2 | 0.5 | 5.3 mm/s | 2.47 | 100% | 59/61/62/64% |
| AND rm=30,β=1.5 | AND | 30 | 15 | 1.5 | 0.3 | 6.3 mm/s | 2.77 | 100% | 70/72/67/68% |
| AND rm=25,β=1 | AND | 25 | 15 | 1 | 0.3 | 6.3 mm/s | 3.06 | 100% | 81/75/75/75% |
| **Current AND (rm=25,β=10)** | AND | 25 | 15 | 10 | 1 | **68.5 mm/s** | 0.58 | 57.8% | 0/0/0/58% |
| rm_only (rm=25) | OR | 25 | 15 | ∞ | 0 | **148.8 mm/s** | 4.00 | 100% | 100/100/100/100% |

### 5.4 關鍵發現

1. **現行 OR 邏輯 (rm=25,β=10) 在行走資料上表現最佳** (RMSE 4.5 mm/s)
   - 因為 rm >> 25 所以 OR 條件幾乎總被 rm 滿足
   - β threshold (10) 高於大部分 beta 值，但因為用 OR 所以不影響
   - 實際行為等同於 rm-only trigger（但有 β<1 的 deactivation 條件保護）

2. **AND 邏輯需要大幅降低 β threshold**：
   - `β_HIGH=10`：失效（68.5 mm/s RMSE），因 beta 幾乎不超過 10，只有 leg d 偶爾觸發
   - `β_HIGH=2`：可用（4.7~5.3 mm/s RMSE），觸地率 ~60%，更 selective
   - `β_HIGH=1`：過於寬鬆，幾乎所有時刻都觸發 (75~81%)

3. **純 rm 判斷不可行**：rm 在行走時恆 >25，導致全部 4 足永遠判為觸地
   - 空中腿的錯誤速度觀測 → RMSE 148.8 mm/s

4. **AND rm=50,β=2 是最佳 AND 配置**：
   - RMSE 4.7 mm/s（僅比 OR 差 0.2 mm/s）
   - 更 selective：平均 2.39 腿觸地（vs OR 的 2.91）
   - rm 閾值提高到 50 可以更好地排除 swing phase

### 5.5 結論與建議

> **現行 OR 邏輯在穩定行走下表現良好**，觸地判斷本身不是 ESEKF 失敗的直接原因。
>
> 若要切換為 AND 邏輯（更 robust against false positive），建議：
> - `CONTACT_BETA_THRESHOLD_HIGH` 降至 **2.0**（從 10）
> - `CONTACT_BETA_THRESHOLD_LOW` 降至 **0.5**（從 1）
> - `CONTACT_RM_THRESHOLD_HIGH` 可提高至 **40~50** 增加 selectivity
>
> 但優先排查其他問題（觀測噪聲/ESEKF 數值穩定性），因為觸地判斷表現尚可。

---

## 6. 建議的 Debug 順序

```
Step 1 ✅  運動學計算驗證 → 通過
Step 2 ✅  觸地判斷驗證 → 觸地判斷表現尚可，非主因

Step 3 →  觀測噪聲調參
          - 增大 sigma_leg（目前 1e-3 可能太小）
          - 比較不同噪聲參數下的收斂行為
          - 注意：瞬時速度 noise 比平均速度高 ~10x

Step 4 →  ESEKF 隔離測試
          - 用固定的（已知正確的）觸地狀態 + 運動學觀測
          - 僅測試 predict → update → inject 流程
          - 監控 P, dx, bias 的行為

Step 5 →  ESEKF 數值穩定性
          - 加入 P 對稱化
          - 實作 G matrix covariance reset
          - 使用 LLT 分解替代 S.inverse()
```

---

## 7. 相關檔案

| 檔案 | 說明 |
|------|------|
| `src/TEST/kinematic_compare/compare_velocity_methods.py` | 平均/瞬時速度比較腳本 |
| `src/TEST/kinematic_compare/velocity_comparison.png` | 速度比較圖表 |
| `src/TEST/kinematic_compare/velocity_error.png` | 估測誤差圖表 |
| `src/TEST/contact_detection/tune_schmitt_trigger.py` | Schmitt Trigger 離線調校腳本 |
| `src/TEST/contact_detection/disturbance_signals.png` | 4 腿 disturbance 訊號圖 |
| `src/TEST/contact_detection/contact_detection_comparison.png` | 8 種閾值觸地判斷比較圖 |
| `src/TEST/contact_detection/velocity_estimation_comparison.png` | 前 4 名配置速度估測圖 |
| `src/TEST/contact_detection/rm_beta_scatter.png` | |rm| vs |β| 散佈圖（閾值區域視覺化） |
| `output_data/walk_3m_01m.csv` | 行走 3m 錄製資料 |
| `output_data/walk_3m_01m_result.csv` | disturbance observer 離線產出 |
| `src/es_ekf/ESEKF.cpp` | ES-EKF 實作 |
| `src/corgi_leg_odom.cpp` | ROS2 腿部里程計節點 |
| `legacy/src/KLD_estimation/InformationFilter.cpp` | 舊 Information Filter |
| `legacy/src/corgi_odometry/corgi_odometry.cpp` | 舊里程計主程式 |
| `src/kinematic/Leg.cpp` | 腿部運動學模型 |
| `include/kinematic/ContactMap.hpp` | 接觸點映射 |
| `include/Config.hpp` | 閾值與參數設定 |
