# ORB-SLAM3 Code Review

深入分析 ORB-SLAM3 三大核心線程的原始碼實現。

## 目錄結構

```
orbslam3_code_review/
├── README.md                    # 本文件
├── 01_System_Overview.md        # 系統架構總覽
├── 02_Tracking.md               # Tracking 線程詳解
├── 03_LocalMapping.md           # LocalMapping 線程詳解
└── 04_LoopClosing.md            # LoopClosing 線程詳解
```

## 三線程架構

```
                    ┌─────────────────────────────────────────────────────────┐
                    │                      ORB-SLAM3                          │
                    └─────────────────────────────────────────────────────────┘
                                            │
            ┌───────────────────────────────┼───────────────────────────────┐
            │                               │                               │
            ▼                               ▼                               ▼
    ┌───────────────┐               ┌───────────────┐               ┌───────────────┐
    │   Tracking    │──KeyFrame────▶│ LocalMapping  │──KeyFrame────▶│ LoopClosing   │
    │               │               │               │               │               │
    │ • 特徵提取    │               │ • 局部 BA     │               │ • 回環檢測    │
    │ • 位姿估計    │               │ • 地圖點剔除  │               │ • 位姿圖優化  │
    │ • 關鍵幀決策  │               │ • 新地圖點    │               │ • 地圖合併    │
    └───────────────┘               └───────────────┘               └───────────────┘
           │                               │                               │
           │                               │                               │
           └───────────────────────────────┴───────────────────────────────┘
                                           │
                                           ▼
                                    ┌─────────────┐
                                    │    Atlas    │
                                    │  (多地圖)   │
                                    └─────────────┘
```

## 線程間通訊

ORB-SLAM3 **不使用** ROS Topic/Service，而是直接通過**指針**傳遞：

```cpp
// System.cc 中設置線程關係
mpTracker->SetLocalMapper(mpLocalMapper);
mpTracker->SetLoopClosing(mpLoopCloser);
mpLocalMapper->SetTracker(mpTracker);
mpLocalMapper->SetLoopCloser(mpLoopCloser);
```

使用 `std::mutex` 保護共享數據，避免競爭條件。

## 對應原始碼

| 模塊 | 頭文件 | 源文件 |
|------|--------|--------|
| System | `include/System.h` | `src/System.cc` |
| Tracking | `include/Tracking.h` | `src/Tracking.cc` |
| LocalMapping | `include/LocalMapping.h` | `src/LocalMapping.cc` |
| LoopClosing | `include/LoopClosing.h` | `src/LoopClosing.cc` |

## 參考資源

- [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- ORB-SLAM3 論文：Campos et al., IEEE T-RO 2021
- 《視覺SLAM十四講》高翔
