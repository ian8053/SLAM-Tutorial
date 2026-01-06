SLAM Course 01 - Introduction to Robot Mapping 筆記
=====================================================


1. 六個核心術語 (Terms)
-----------------------

State Estimation (狀態估計)
  - 估計世界的狀態（機器人位置或地標位置）
  - 因為感測器有噪聲，無法完美知道，只能「估計」

Localization (定位)
  - 狀態估計的應用，專門估計「機器人在哪裡」
  - Pose = 位置(x,y) + 方向(θ)

Mapping (建圖)
  - 假設知道機器人在哪，估計環境長什麼樣子

SLAM (Simultaneous Localization and Mapping)
  - 同時估計機器人位置和環境地圖

Navigation (導航)
  - 機器人自主決定從 A 走到 B
  - 需要依賴 SLAM

Motion Planning (運動規劃)
  - 規劃達到目標的最佳運動序列


2. Navigation vs Motion Planning 差異
-------------------------------------

Navigation：
  - 通常指「移動機器人」在 x, y, θ 空間中從一個位置移動到另一個位置
  - 例：「機器人從 A 點走到 B 點」

Motion Planning：
  - 更廣泛，可以是機械手臂規劃如何從一個姿態到另一個姿態
  - 例：「我的手臂怎麼穿過這個洞？」（不叫 Navigation）

大多數 Navigation 系統會使用 Motion Planning 演算法（如 A*）


3. Localization 和 Mapping 的「同時性問題」
------------------------------------------

什麼情況下可以分開做？
  - 純 Localization：需要已知地圖（如神告訴你地圖長什麼樣）
  - 純 Mapping：需要已知位置（完美知道機器人在哪）

現實中的問題：
  兩者都不知道！
  如果沒有外部的「神級」感測器告訴你位置或地圖，你一定要做 SLAM

例外情況：
  - 機器人在軌道上移動（完美知道位置）→ 可以只做 Mapping
  - 有非常精確的預建地圖 → 可以只做 Localization


4. Localization Example：修正 (Correction) 的意義
-------------------------------------------------

情境描述：
  1. 機器人從起點出發，走到新位置
  2. 因為輪子打滑等因素，機器人以為自己左轉，但實際上右轉（drift）
  3. 機器人觀測到 landmark，說：「根據我的位置估計，landmark 應該在 1 米外」
  4. 但根據已知地圖，landmark 應該只有 50 公分遠
  5. 修正：既然地圖是對的，那我的位置估計一定有誤，把自己「拉回」正確位置

這就是 correction step：用觀測來修正位置估計


5. Mapping Example vs SLAM Example 比較
---------------------------------------

| 比較項目 | Mapping | SLAM |
|----------|---------|------|
| 知道位置？ | 是（已知） | 不知道 |
| 知道地圖？ | 要估計 | 要估計 |
| 結果準確度 | 較準確（只有感測器噪聲） | 誤差更大（位置+感測器雙重誤差） |


6. 雞生蛋問題 (Chicken and Egg Problem)
---------------------------------------

- 需要地圖才能定位
- 需要好的位置估計才能建好地圖
- 它們相互依賴，必須同時解決

這就是為什麼需要 SLAM！


7. SLAM 的數學定義
------------------

變數定義：

| 符號 | 名稱 | 說明 | 範圍 |
|------|------|------|------|
| u | Controls/Odometry | 控制指令，如「前進1米」 | u1 到 ut |
| z | Observations | 觀測資料，如雷射掃描、相機影像 | z1 到 zt |
| m | Map | 環境地圖 | - |
| x | Path/Poses | 機器人軌跡 | x0 到 xt |

核心公式：

  p(x0:t, m | z1:t, u1:t)

解讀：給定所有觀測 z 和控制 u，求路徑 x 和地圖 m 的機率分布

為什麼 x 從 0 開始，z 和 u 從 1 開始？
  - x0 用來定義座標系原點（參考框架）
  - 如果有 3 個 poses (x0, x1, x2)，中間只能執行 2 個控制指令 (u1, u2)
  - z 從 1 開始是習慣，理論上 z0 也可以有

z 是 landmark 嗎？
  - z 是觀測資料（laser scan、camera image）
  - m 才是地標位置或環境模型
  - 觀測 z 取決於：機器人位置 x 和 地圖 m


8. Graphical Model 圖形模型
---------------------------

![Graphical Model](https://github.com/user-attachments/assets/6e45fd1d-de07-43c3-a315-c1b379a6543b)


箭頭的意義：
  - 箭頭 = 影響 (influences)
  - x(t-1) --> xt：前一個位置影響下一個位置
  - ut --> xt：控制指令影響新位置
  - xt, m --> zt：位置和地圖一起決定觀測結果

節點顏色：
  - 灰色圓圈 = 已知（observed）
  - 白色圓圈 = 未知（要估計）


9. Full SLAM vs Online SLAM
---------------------------

| 比較項目 | Full SLAM | Online SLAM |
|----------|-----------|-------------|
| 估計什麼 | 整條軌跡 x0:t + 地圖 m | 只有當前位置 xt + 地圖 m |
| 公式 | p(x0:t, m \| z, u) | p(xt, m \| z, u) |
| 應用 | 事後處理、精確地圖 | 即時導航、決策 |
| Graph表示 | 所有 x 都是未知（白色） | 只有 xt 是未知 |


10. 積分的意義 (Marginalization) - 詳細解釋
-------------------------------------------

Online SLAM 公式：

  p(xt, m | z, u) = ∫∫∫...∫ p(x0:t, m | z, u) dx0 dx1 ... dx(t-1)

什麼是邊緣化 (Marginalization)？

  如果有聯合機率 p(A, B)，想只要 p(A)：

  p(A) = ∫ p(A, B) dB

  意思是：把 B 的所有可能值都「積掉」，剩下 A

直覺解釋：

  想像你有一張表格，記錄了「身高」和「體重」的聯合分布：

  | 身高\體重 | 50kg | 60kg | 70kg | 總和 |
  |-----------|------|------|------|------|
  | 160cm | 0.1 | 0.15 | 0.05 | 0.3 |
  | 170cm | 0.05 | 0.2 | 0.15 | 0.4 |
  | 180cm | 0.05 | 0.1 | 0.15 | 0.3 |

  如果你只想知道「身高的分布」（不管體重），你就把每一行的機率加起來：
  - P(身高=160) = 0.3
  - P(身高=170) = 0.4
  - P(身高=180) = 0.3

  這個「加起來」的操作就是邊緣化，連續情況下就是積分。

對應到 SLAM：

  - Full SLAM 知道：x0, x1, x2, ..., xt（整條路徑）
  - Online SLAM 只關心：xt（現在位置）

  要從 Full SLAM 得到 Online SLAM，就要把過去的位置 x0 到 x(t-1) 都積掉：

  p(xt, m | z, u) = ∫∫∫...∫ p(x0, x1, ..., xt, m | z, u) dx0 dx1 ... dx(t-1)

這不是「回推」，而是「遺忘」：

  - 不是回推過去位置讓它更精確
  - 而是說「我不在乎過去在哪，只關心現在在哪」
  - 把過去位置的所有可能性都考慮進來，然後「總結」成現在的狀態

實際做法 - 遞迴計算：

  不是一次算所有積分，而是每一步只算一個：

  1. t=1 時：從 x0 算出 x1 的分布，然後「忘掉」x0
  2. t=2 時：用 x1 的分布，算出 x2 的分布，然後「忘掉」x1
  3. ...依此類推

  這就是為什麼叫「Online」—— 可以即時、遞迴地計算！

比喻：

  Full SLAM 的回答：「如果今天晴天，明天下雨機率 20%；如果今天陰天，明天下雨機率 60%」
  Online SLAM 的回答：「不管今天怎樣，明天下雨機率是 35%」（這是積分後的結果）

  第二種回答更直接可用，這就是 Online SLAM 的價值。


11. 四個模型
------------

Motion Model（運動模型）
  - 給定前一個位置和控制指令，新位置的機率分布
  - 公式：p(xt | x(t-1), ut)

Observation Model（觀測模型）
  - 給定位置和地圖，會觀測到什麼的機率分布
  - 公式：p(zt | xt, m)

Odometry Model（里程計模型）
  - Motion Model 的一種，用輪子編碼器估計移動量
  - 用 δrot1, δtrans, δrot2 描述

Odometry vs Control Command 的差別：
  - Control: 「前進1米」（指令）
  - Odometry: 「實際走了 99.9 公分」（編碼器回饋）

  Odometry 更準確，所以通常用 Odometry Model 作為 Motion Model。


12. 為什麼假設高斯分布？
------------------------

原因：
  - 感測器噪聲和運動誤差通常近似高斯分布
  - 高斯分布數學上好處理（Kalman Filter）
  - 如果誤差很小，高斯是合理的近似

非高斯情況：
  - 可能是「香蕉形」分布（banana shape）
  - 可能是多模態（multimodal）
  - 這時需要用 Particle Filter 或 Graph-based 方法


13. 三種主要方法
----------------

Kalman Filter
  - 假設：高斯分布
  - 特點：數學優雅、計算高效

Particle Filter
  - 假設：任意分布（多模態）
  - 特點：可以處理非高斯、複雜情況

Graph-based
  - 假設：通常高斯，但可放寬
  - 特點：更好處理異常值、可結合不同感測器


推薦閱讀
--------

- Handbook on Robotics - SLAM 章節 (Section 1 & 2)
- Probabilistic Robotics - Chapter 5 & 6 (Motion Model & Observation Model)
- Source:
    -Video: SLAM-Course - 01 - Introduction to Robot Mapping (2013/14; Cyrill Stachniss)
    -URL: https://www.youtube.com/watch?v=wVsfCnyt5jA
    -Lecturer: Cyrill Stachniss (University of Freiburg)
