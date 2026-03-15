# VNCLio 公式推导

> VNC = 法向一致性约束（Visual Normal Consistency）—— 一种旋转约束，在标准点到面 IEKF 基础上增加扫描-地图法向对齐残差。

## 1. 状态定义

15 自由度 IMU 状态 $\mathbf{x}$：

$$
\mathbf{x} = \begin{bmatrix} \mathbf{R} \\ \mathbf{p} \\ \mathbf{v} \\ \mathbf{b}_g \\ \mathbf{b}_a \end{bmatrix} \in SO(3) \times \mathbb{R}^{12}
$$

其中 $\mathbf{R} \in SO(3)$ 为旋转矩阵，$\mathbf{p}, \mathbf{v}, \mathbf{b}_g, \mathbf{b}_a \in \mathbb{R}^3$ 分别为位置、速度、陀螺仪偏置、加速度计偏置。

### 1.1 状态加法 Boxplus（右扰动）

状态更新采用 $SO(3)$ 上的**右扰动**：

$$
\mathbf{x} \boxplus \boldsymbol{\delta} :
\begin{cases}
\mathbf{R} \leftarrow \mathbf{R} \cdot \mathrm{Exp}(\delta\boldsymbol{\theta}) \\
\mathbf{p} \leftarrow \mathbf{p} + \delta\mathbf{p} \\
\mathbf{v} \leftarrow \mathbf{v} + \delta\mathbf{v} \\
\mathbf{b}_g \leftarrow \mathbf{b}_g + \delta\mathbf{b}_g \\
\mathbf{b}_a \leftarrow \mathbf{b}_a + \delta\mathbf{b}_a
\end{cases}
$$

其中 $\boldsymbol{\delta} = [\delta\boldsymbol{\theta}^\top, \delta\mathbf{p}^\top, \delta\mathbf{v}^\top, \delta\mathbf{b}_g^\top, \delta\mathbf{b}_a^\top]^\top \in \mathbb{R}^{15}$。

### 1.2 状态差分 Boxminus

$$
\mathbf{x}_a \boxminus \mathbf{x}_b = \begin{bmatrix} \mathrm{Log}(\mathbf{R}_b^\top \mathbf{R}_a) \\ \mathbf{p}_a - \mathbf{p}_b \\ \mathbf{v}_a - \mathbf{v}_b \\ \mathbf{b}_{g,a} - \mathbf{b}_{g,b} \\ \mathbf{b}_{a,a} - \mathbf{b}_{a,b} \end{bmatrix}
$$

> **约定**：$\boxplus$ 和 $\boxminus$ 均使用右扰动。所有雅可比矩阵必须与此约定保持一致。

---

## 2. 点到面残差

给定 body 系下的点 $\mathbf{q}$，其世界坐标为：

$$
\mathbf{w} = \mathbf{R} \mathbf{q} + \mathbf{p}
$$

对地图平面 $(\mathbf{n}, \mathbf{c})$ 的点到面残差：

$$
r_{\text{p2p}} = \mathbf{n}^\top (\mathbf{w} - \mathbf{c})
$$

### 2.1 点到面雅可比矩阵

施加右扰动 $\mathbf{R} \leftarrow \mathbf{R} \cdot \mathrm{Exp}(\delta\boldsymbol{\theta})$：

$$
\mathbf{w}(\delta\boldsymbol{\theta}, \delta\mathbf{p}) = \mathbf{R} \mathrm{Exp}(\delta\boldsymbol{\theta}) \mathbf{q} + \mathbf{p} + \delta\mathbf{p}
\approx \mathbf{R}(\mathbf{I} + [\delta\boldsymbol{\theta}]_\times)\mathbf{q} + \mathbf{p} + \delta\mathbf{p}
$$

$$
= \mathbf{w} + \mathbf{R}[\delta\boldsymbol{\theta}]_\times \mathbf{q} + \delta\mathbf{p}
= \mathbf{w} - \mathbf{R}[\mathbf{q}]_\times \delta\boldsymbol{\theta} + \delta\mathbf{p}
$$

因此：

$$
\frac{\partial r_{\text{p2p}}}{\partial \delta\boldsymbol{\theta}} = -\mathbf{n}^\top \mathbf{R} [\mathbf{q}]_\times = (\mathbf{R}^\top \mathbf{n})^\top [\mathbf{q}]_\times = ([\mathbf{q}]_\times \mathbf{R}^\top \mathbf{n})^\top
$$

$$
\frac{\partial r_{\text{p2p}}}{\partial \delta\mathbf{p}} = \mathbf{n}^\top
$$

> **代码对应**（`VINASlam.cpp:436`）：`jac.head(3) = hat(q) * R^T * n`，`jac.tail(3) = n` —— 与右扰动一致。

### 2.2 信息权重

$$
R_{\text{inv}} = \frac{1}{0.0005 + \sigma_d}
$$

其中 $\sigma_d$ 是体素搜索中点到面匹配的不确定性。

---

## 3. VNC 法向一致性残差

### 3.1 扫描平面提取

对**当前扫描**（body 系）中每个八叉树叶节点体素，通过特征分解拟合平面。特征值 $\lambda_{\min} \leq \lambda_{\text{mid}} \leq \lambda_{\max}$ 表征平面性：

$$
\text{quality} = 1 - \frac{\lambda_{\min}}{\lambda_{\min} + \lambda_{\text{mid}} + \lambda_{\max}}
$$

$$
\sigma_n = \sqrt{\frac{\lambda_{\min}}{\lambda_{\min} + \lambda_{\text{mid}} + \lambda_{\max}}}
$$

滤波条件：
- $\lambda_{\text{mid}} > 10^{-12}$（非退化）
- $\lambda_{\min} / \lambda_{\text{mid}} \leq 0.12$（薄而平的结构）
- $\text{quality} > 0.5$

每个通过的平面生成一个 `ScanPlaneInfo`：$(\mathbf{c}^b, \mathbf{n}^b, q, \sigma_n)$ —— 中心、法向、质量、不确定性，均在 body 系下。

### 3.2 扫描-地图法向匹配

对每个扫描平面，变换到世界系：

$$
\mathbf{c}^w = \mathbf{R} \mathbf{c}^b + \mathbf{p}, \qquad
\mathbf{n}^w_{\text{scan}} = \frac{\mathbf{R} \mathbf{n}^b}{\|\mathbf{R} \mathbf{n}^b\|}
$$

通过 27 邻域 `matchVoxelMap` 搜索找到最近的地图平面 $\mathbf{n}_{\text{map}}$。应用法向一致性过滤：

$$
|\mathbf{n}^w_{\text{scan}} \cdot \mathbf{n}_{\text{map}}| > 0.7 \quad (\text{夹角} > 45° \text{时拒绝})
$$

> 取绝对值使残差具有**符号不变性**：方向相反的法向被视为平行。这在物理上是正确的，因为平面法向仅在符号意义上有定义。

### 3.3 VNC 残差

定义 $\mathbf{n}_{\text{map}}$ 零空间上的**投影矩阵**：

$$
\mathbf{S} = \mathbf{I}_3 - \mathbf{n}_{\text{map}} \mathbf{n}_{\text{map}}^\top
$$

$\mathbf{S}$ 是秩为 2 的投影矩阵，提取任意向量垂直于 $\mathbf{n}_{\text{map}}$ 的分量。

三维 VNC 残差：

$$
\mathbf{r}_{\text{vnc}} = \mathbf{S} \, \mathbf{n}^w_{\text{scan}}
$$

**几何解释**：当 $\mathbf{n}^w_{\text{scan}} \parallel \mathbf{n}_{\text{map}}$ 时，残差为零（法向对齐）。残差捕捉的是切向错位量。由于 $\mathbf{S}$ 的秩为 2，每对 VNC 配对提供**2 个独立的旋转约束**（绕 $\mathbf{n}_{\text{map}}$ 方向的旋转对单对法向不可观）。

### 3.4 VNC 雅可比矩阵（右扰动）

施加右扰动 $\mathbf{R} \leftarrow \mathbf{R} \cdot \mathrm{Exp}(\delta\boldsymbol{\theta})$：

$$
\mathbf{n}^w_{\text{scan}}(\delta\boldsymbol{\theta}) = \mathbf{R} \, \mathrm{Exp}(\delta\boldsymbol{\theta}) \, \mathbf{n}^b
\approx \mathbf{R}(\mathbf{I} + [\delta\boldsymbol{\theta}]_\times) \mathbf{n}^b
$$

$$
= \mathbf{R}\mathbf{n}^b + \mathbf{R}[\delta\boldsymbol{\theta}]_\times \mathbf{n}^b
= \mathbf{n}^w_{\text{scan}} + \mathbf{R}([\delta\boldsymbol{\theta}]_\times \mathbf{n}^b)
$$

利用恒等式 $[\mathbf{a}]_\times \mathbf{b} = -[\mathbf{b}]_\times \mathbf{a}$：

$$
= \mathbf{n}^w_{\text{scan}} - \mathbf{R} [\mathbf{n}^b]_\times \, \delta\boldsymbol{\theta}
$$

因此：

$$
\boxed{\frac{\partial \mathbf{r}_{\text{vnc}}}{\partial \delta\boldsymbol{\theta}} = -\mathbf{S} \, \mathbf{R} \, [\mathbf{n}^b]_\times}
$$

$$
\frac{\partial \mathbf{r}_{\text{vnc}}}{\partial \delta\mathbf{p}} = \mathbf{0}_{3 \times 3} \quad \text{（法向与平移无关）}
$$

完整雅可比矩阵 $\mathbf{J} \in \mathbb{R}^{3 \times 6}$：

$$
\mathbf{J} = \begin{bmatrix} -\mathbf{S} \mathbf{R} [\mathbf{n}^b]_\times & \mathbf{0}_{3\times 3} \end{bmatrix}
$$

> **左扰动与右扰动的区别**：左扰动给出 $\mathbf{J}_\theta = -\mathbf{S}[\mathbf{n}^w_{\text{scan}}]_\times$。利用恒等式 $[\mathbf{R}\mathbf{v}]_\times = \mathbf{R}[\mathbf{v}]_\times \mathbf{R}^\top$ 可以验证：
> $$-\mathbf{S}[\mathbf{R}\mathbf{n}^b]_\times = -\mathbf{S}\mathbf{R}[\mathbf{n}^b]_\times \mathbf{R}^\top \neq -\mathbf{S}\mathbf{R}[\mathbf{n}^b]_\times$$
> 左扰动雅可比多了一个尾随的 $\mathbf{R}^\top$。在使用右扰动的系统中使用左扰动雅可比会将梯度旋转到错误的坐标系。

### 3.5 VNC 权重

$$
w_{\text{vnc}} = \frac{\alpha \cdot q}{\sigma_n^2 + 0.01}
$$

其中 $\alpha = 0.1$ 为 `VNC_ALPHA`，$q$ 为扫描平面质量，$\sigma_n$ 为法向估计不确定性。质量越高（越平坦）、不确定性越低的平面贡献越大。

---

## 4. IEKF 法方程

两种残差共同贡献到一个共享的 $6 \times 6$ 法方程（旋转 + 平移）：

$$
\mathbf{H}^\top \mathbf{H} = \sum_{i \in \text{p2p}} R_{\text{inv},i} \, \mathbf{j}_i \mathbf{j}_i^\top + \sum_{k \in \text{vnc}} w_k \, \mathbf{J}_k^\top \mathbf{J}_k
$$

$$
\mathbf{H}^\top \mathbf{z} = -\sum_{i \in \text{p2p}} R_{\text{inv},i} \, \mathbf{j}_i \, r_i - \sum_{k \in \text{vnc}} w_k \, \mathbf{J}_k^\top \mathbf{r}_k
$$

其中 $\mathbf{j}_i \in \mathbb{R}^6$ 为点到面雅可比，$\mathbf{J}_k \in \mathbb{R}^{3\times 6}$ 为 VNC 雅可比。

### 4.1 IEKF 更新

利用预测协方差 $\mathbf{P}^-$ 和传播先验 $\mathbf{x}_{\text{prop}}$：

$$
\mathbf{K}^{-1} = (\mathbf{H}^\top \mathbf{H} + (\mathbf{P}^-)^{-1})^{-1}
$$

$$
\mathbf{G} = \mathbf{K}^{-1} \mathbf{H}^\top \mathbf{H}
$$

$$
\boldsymbol{\delta} = \mathbf{K}^{-1} \mathbf{H}^\top \mathbf{z} + (\mathbf{x}_{\text{prop}} \boxminus \mathbf{x}_{\text{curr}}) - \mathbf{G} (\mathbf{x}_{\text{prop}} \boxminus \mathbf{x}_{\text{curr}})_{[0:6]}
$$

$$
\mathbf{x}_{\text{curr}} \leftarrow \mathbf{x}_{\text{curr}} \boxplus \boldsymbol{\delta}
$$

### 4.2 收敛判据

$$
\|\delta\boldsymbol{\theta}\| < \epsilon_R \quad \text{且} \quad \|\delta\mathbf{p}\| < \epsilon_t
$$

默认值：$\epsilon_R = 0.1°$，$\epsilon_t = 0.1 \text{ cm}$（来自 `constants.hpp`）。

### 4.3 协方差更新

收敛后：

$$
\mathbf{P}^+ = (\mathbf{I} - \mathbf{G}) \mathbf{P}^-
$$

---

## 5. LioStateEstimation 算法流程

`LioStateEstimation(PVecPtr pptr, bool use_vnc)` 是整个 LIO 估计器的核心函数。`use_vnc=false` 时退化为标准 IEKF，`use_vnc=true` 时即 VNCLio。以下描述完整的算法流程。

### 5.1 输入

- `pptr`：当前帧点云（body 系），每个点 $(\mathbf{q}_i, \boldsymbol{\Sigma}_i^b)$ 带 $3\times3$ 协方差
- `use_vnc`：是否启用 VNC 法向一致性约束
- 全局状态 `x_curr`：当前 IEKF 状态估计（IMU 传播后的先验）
- 全局地图 `surf_map`：`unordered_map<VOXEL_LOC, OctoTree*>` 体素地图

### 5.2 初始化

保存传播先验用于 IEKF 校正：

$$
\mathbf{x}_{\text{prop}} = \mathbf{x}_{\text{curr}}
$$

计算先验协方差逆（用于 IEKF 的先验约束）：

$$
(\mathbf{P}^-)^{-1} = (\mathbf{x}_{\text{curr}}.\text{cov})^{-1}
$$

### 5.3 VNC 预处理（仅 `use_vnc=true`）

1. 将 body 系点云做一次体素化 → `scan_voxels`
2. 对每个体素做平面拟合 `fit_scan_plane()`
3. 递归遍历八叉树 `collectScanPlanes()` 提取合格的扫描平面（见 §3.1 滤波条件）
4. 得到 `scan_planes` 列表，每个元素 $(\mathbf{c}^b, \mathbf{n}^b, q, \sigma_n)$

### 5.4 IEKF 迭代（最多 `num_max_iter = 4` 次）

每次迭代重新构建法方程 $\mathbf{H}^\top\mathbf{H}$ 和 $\mathbf{H}^\top\mathbf{z}$（$6\times6$ 和 $6\times1$）。

#### 5.4.1 点协方差传播到世界系

从当前状态协方差中提取旋转和平移不确定性：

$$
\boldsymbol{\Sigma}_R = \mathbf{P}^-_{[0:3,\,0:3]}, \qquad
\boldsymbol{\Sigma}_t = \mathbf{P}^-_{[3:6,\,3:6]}
$$

对每个点 $\mathbf{q}_i$（body 系），传播到世界系的不确定性：

$$
\mathbf{w}_i = \mathbf{R} \mathbf{q}_i + \mathbf{p}
$$

$$
\boldsymbol{\Sigma}_i^w = \underbrace{\mathbf{R} \boldsymbol{\Sigma}_i^b \mathbf{R}^\top}_{\text{观测噪声}} + \underbrace{[\mathbf{q}_i]_\times \boldsymbol{\Sigma}_R [\mathbf{q}_i]_\times^\top}_{\text{旋转不确定性}} + \underbrace{\boldsymbol{\Sigma}_t}_{\text{平移不确定性}}
$$

> **代码对应**（`VINASlam.cpp:403-404`）：`var_world = R * pv.var * R^T + phat * rot_var * phat^T + tsl_var`

**推导**：世界点 $\mathbf{w} = \mathbf{R}\mathbf{q} + \mathbf{p}$ 对 $(\mathbf{q}, \delta\boldsymbol{\theta}, \delta\mathbf{p})$ 的一阶近似为 $\delta\mathbf{w} \approx \mathbf{R}\delta\mathbf{q} - \mathbf{R}[\mathbf{q}]_\times\delta\boldsymbol{\theta} + \delta\mathbf{p}$。右扰动下 $-\mathbf{R}[\mathbf{q}]_\times$ 简化为 $[\mathbf{q}]_\times$ 作用于旋转协方差时的效果（因为 $\boldsymbol{\Sigma}_R$ 在 body 系下定义），代码直接使用 `phat * rot_var * phat^T`。

#### 5.4.2 体素地图匹配

对每个世界系点 $\mathbf{w}_i$，执行两级查找：

1. **快速路径**：若上一次匹配的 `octos[i]` 仍包含该点（`inside(w)`），在同一八叉树节点内 `match()`
2. **慢速路径**：否则在 `surf_map` 哈希表中重新查找 → `match(surf_map, w, ...)`

匹配输出：最近平面 $(\mathbf{n}, \mathbf{c})$ 和匹配不确定性 $\sigma_d$（取决于 $\boldsymbol{\Sigma}_i^w$ 在法向方向的投影）。

#### 5.4.3 点到面法方程累加

对每个匹配成功的点（详见 §2）：

$$
\mathbf{H}^\top\mathbf{H} \mathrel{+}= R_{\text{inv},i} \, \mathbf{j}_i \mathbf{j}_i^\top, \qquad
\mathbf{H}^\top\mathbf{z} \mathrel{-}= R_{\text{inv},i} \, \mathbf{j}_i \, r_i
$$

同时累加法向外积矩阵（用于退化检测）：

$$
\mathbf{N} \mathrel{+}= \mathbf{n}_i \mathbf{n}_i^\top
$$

#### 5.4.4 VNC 法方程累加（仅 `use_vnc=true`）

对每个合格的扫描平面-地图平面配对（详见 §3）：

$$
\mathbf{H}^\top\mathbf{H} \mathrel{+}= w_k \, \mathbf{J}_k^\top \mathbf{J}_k, \qquad
\mathbf{H}^\top\mathbf{z} \mathrel{-}= w_k \, \mathbf{J}_k^\top \mathbf{r}_k
$$

#### 5.4.5 状态更新

将 $6\times6$ 法方程嵌入 $15\times15$ 系统（见 §4.1）：

$$
\mathbf{K}^{-1} = \left(\begin{bmatrix} \mathbf{H}^\top\mathbf{H} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} \end{bmatrix} + (\mathbf{P}^-)^{-1}\right)^{-1}
$$

$$
\mathbf{G} = \mathbf{K}^{-1}_{[\cdot,\,0:6]} \, \mathbf{H}^\top\mathbf{H}
$$

$$
\boldsymbol{\eta} = \mathbf{x}_{\text{prop}} \boxminus \mathbf{x}_{\text{curr}} \in \mathbb{R}^{15}
$$

$$
\boldsymbol{\delta} = \mathbf{K}^{-1}_{[\cdot,\,0:6]} \, \mathbf{H}^\top\mathbf{z} + \boldsymbol{\eta} - \mathbf{G} \, \boldsymbol{\eta}_{[0:6]}
$$

$$
\mathbf{x}_{\text{curr}} \leftarrow \mathbf{x}_{\text{curr}} \boxplus \boldsymbol{\delta}
$$

> **解释**：$\boldsymbol{\eta}$ 项是 IEKF 特有的——它补偿当前线性化点 $\mathbf{x}_{\text{curr}}$ 和传播先验 $\mathbf{x}_{\text{prop}}$ 之间的差异。随着迭代收敛，$\boldsymbol{\eta} \to 0$。

#### 5.4.6 收敛与重匹配策略

```
converged = (‖δθ‖ < 0.1°) and (‖δp‖ < 0.1 cm)
```

重匹配逻辑（`rematch_num`）：
1. 第一次收敛或倒数第二次迭代 → 标记需要重匹配
2. 重匹配次数 ≥ 2 或最后一次迭代 → 更新协方差、退出循环

这允许在收敛后用更新后的状态重新匹配一次，提高线性化精度。

#### 5.4.7 协方差更新

仅在最终退出时执行：

$$
\mathbf{P}^+ = (\mathbf{I}_{15} - \mathbf{G}) \, \mathbf{P}^-
$$

### 5.5 失败恢复

如果迭代中检测到数值异常（NaN/Inf），回滚到传播先验：

$$
\mathbf{x}_{\text{curr}} \leftarrow \mathbf{x}_{\text{prop}}
$$

返回 `false` 表示估计失败。

### 5.6 退化检测

使用累积的法向外积矩阵 $\mathbf{N} = \sum_i \mathbf{n}_i \mathbf{n}_i^\top$ 检测几何退化：

$$
\lambda_{\min}(\mathbf{N}) \geq \tau_{\text{plane}} \quad (\tau_{\text{plane}} = 14.0)
$$

$\mathbf{N}$ 是一个 $3\times3$ 对称矩阵，其特征值反映法向分布的散度：
- 三个大特征值 → 法向分布在三个方向都有足够约束
- 最小特征值接近 0 → 法向集中在某个平面内，存在平移退化方向

> 此退化检测仅基于点到面法向；VNC 贡献不计入 $\mathbf{N}$（它约束的是旋转而非平移）。

### 5.7 VNCLio 入口

`VNCLio(PVecPtr pptr)` 是 `LioStateEstimation(pptr, true)` 的简单包装：

```cpp
bool VINA_SLAM::VNCLio(PVecPtr pptr) {
  return LioStateEstimation(pptr, true);
}
```

---

## 6. 可观性分析

| 约束类型 | 秩 | 可观方向 |
|---|---|---|
| 单个点到面 | 1 | 沿平面法向的平移 |
| 单对 VNC 法向 | 2 | 绕垂直于 $\mathbf{n}_{\text{map}}$ 的两个轴的旋转 |
| 点到面 + VNC | 3 + 2 = 5（每对） | 接近完整位姿（仅绕 $\mathbf{n}_{\text{map}}$ 的旋转对该单对仍不可观） |

VNC 在**平面环境**（走廊、隧道）中尤其有价值——在这些环境中，点到面残差退化（大量法向指向同一方向，使得绕该方向的旋转不可观）。VNC 增加了显式的法向对齐约束来部分补偿这一问题。

---

## 7. 附录：Hat Map 恒等式

**Hat Map**（反对称矩阵映射）$[\cdot]_\times : \mathbb{R}^3 \to \mathfrak{so}(3)$ 满足：

$$
[\mathbf{R}\mathbf{v}]_\times = \mathbf{R} [\mathbf{v}]_\times \mathbf{R}^\top
$$

该恒等式是左右扰动雅可比转换的关键。它解释了为什么左扰动形式 $-\mathbf{S}[\mathbf{n}^w]_\times$ 展开为 $-\mathbf{S}\mathbf{R}[\mathbf{n}^b]_\times \mathbf{R}^\top$，与右扰动形式 $-\mathbf{S}\mathbf{R}[\mathbf{n}^b]_\times$ 相差一个尾随的 $\mathbf{R}^\top$。
