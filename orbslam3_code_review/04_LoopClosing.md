# ORB-SLAM3 LoopClosing 線程詳解

## 一、LoopClosing 概述

LoopClosing 是**子線程**，負責檢測回環並校正累積誤差。ORB-SLAM3 還支援多地圖合併。

```
LocalMapping ─ KeyFrame ─▶ LoopClosing ─▶ 回環校正 / 地圖合併
                                              ↓
                                         全局 BA
```

## 二、Run() 主循環

```cpp
// LoopClosing.cc
void LoopClosing::Run() {
    mbFinished = false;

    while (1) {
        // 1. 檢查是否有新關鍵幀
        if (CheckNewKeyFrames()) {
            // 1.1 檢測回環候選
            if (DetectLoop()) {
                // 1.2 計算 Sim3（相似變換）
                if (ComputeSim3()) {
                    // 1.3 執行回環校正
                    CorrectLoop();
                }
            }

            // 1.4 檢測地圖合併候選
            if (DetectMergeCandidate()) {
                if (ComputeSim3Merge()) {
                    MergeLocal();
                }
            }
        }

        // 2. 執行全局 BA（如果需要）
        if (mbRunningGBA) {
            // 等待 GBA 完成
            while (isRunningGBA()) {
                usleep(5000);
            }
        }

        // 3. 檢查是否結束
        if (CheckFinish())
            break;

        usleep(5000);
    }

    SetFinish();
}
```

### 流程圖

```
┌──────────────────────────────────────────────────────────────────┐
│                      LoopClosing 主循環                          │
└──────────────────────────────────────────────────────────────────┘
                                │
                                ▼
                    ┌───────────────────────┐
                    │  CheckNewKeyFrames()  │
                    └───────────────────────┘
                                │
                    ┌───────────┴───────────┐
                    │ Yes                   │ No
                    ▼                       ▼
        ┌─────────────────────┐     ┌─────────────┐
        │   DetectLoop()      │     │   Sleep     │
        └─────────────────────┘     └─────────────┘
                    │
            ┌───────┴───────┐
            │ Found         │ Not Found
            ▼               │
    ┌───────────────┐       │
    │ ComputeSim3() │       │
    └───────────────┘       │
            │               │
            ▼               │
    ┌───────────────┐       │
    │ CorrectLoop() │       │
    └───────────────┘       │
            │               │
            └───────┬───────┘
                    │
                    ▼
        ┌─────────────────────┐
        │ DetectMergeCandidate│
        └─────────────────────┘
                    │
            ┌───────┴───────┐
            │ Found         │ Not Found
            ▼               │
    ┌───────────────┐       │
    │  MergeLocal() │       │
    └───────────────┘       │
                            │
                            ▼
                    ┌───────────────┐
                    │   Global BA   │
                    └───────────────┘
```

## 三、DetectLoop（回環檢測）

```cpp
bool LoopClosing::DetectLoop() {
    // 1. 取出新關鍵幀
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
    }

    // 2. 如果地圖關鍵幀太少，不檢測
    if (mpCurrentKF->GetMap()->KeyFramesInMap() < 10)
        return false;

    // 3. 避免太頻繁的回環（距離上次回環需要超過 10 幀）
    if (mpCurrentKF->mnId < mnLastLoopKFid + 10)
        return false;

    // 4. 獲取與當前幀共視的關鍵幀（排除這些作為回環候選）
    vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();

    // 5. 計算當前幀的 BoW 分數基準
    const DBoW2::BowVector& CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1.0f;

    for (KeyFrame* pKF : vpConnectedKeyFrames) {
        if (pKF->isBad())
            continue;

        const DBoW2::BowVector& BowVec = pKF->mBowVec;
        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

        if (score < minScore)
            minScore = score;
    }

    // 6. 在關鍵幀數據庫中搜索回環候選
    vector<KeyFrame*> vpCandidateKFs =
        mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

    if (vpCandidateKFs.empty())
        return false;

    // 7. 連續性檢測（候選幀需要連續出現 3 次）
    mvpEnoughConsistentCandidates.clear();

    for (KeyFrame* pCandidateKF : vpCandidateKFs) {
        // 檢查這個候選是否與之前的候選有連續性
        bool bConsistentForSomeGroup = false;

        for (auto& prevGroup : mvConsistentGroups) {
            bool bConsistent = false;
            for (KeyFrame* pPrevKF : prevGroup.first) {
                if (pCandidateKF->IsConnectedTo(pPrevKF)) {
                    bConsistent = true;
                    break;
                }
            }

            if (bConsistent) {
                bConsistentForSomeGroup = true;
                prevGroup.second++;  // 連續計數 +1

                if (prevGroup.second >= 3) {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                }
            }
        }

        // 如果這是新候選，創建新的一致性組
        if (!bConsistentForSomeGroup) {
            set<KeyFrame*> newGroup;
            newGroup.insert(pCandidateKF);
            mvConsistentGroups.push_back(make_pair(newGroup, 0));
        }
    }

    return !mvpEnoughConsistentCandidates.empty();
}
```

### BoW 回環檢測原理

```
當前幀 BoW 向量: [0.1, 0.05, 0.3, 0.02, ...]
                           ↓
              關鍵幀數據庫（倒排索引）
                           ↓
    ┌─────────────────────────────────────┐
    │ Word 1: KF3, KF15, KF42, ...        │
    │ Word 2: KF7, KF23, ...              │
    │ Word 3: KF3, KF42, KF100, ...       │
    │ ...                                  │
    └─────────────────────────────────────┘
                           ↓
              相似度排序 + 連續性檢測
                           ↓
                    回環候選幀
```

### 連續性檢測

```
時間 t:   候選 = [KF100]          連續計數 = 1
時間 t+1: 候選 = [KF101, KF100]   連續計數 = 2  (KF101 與 KF100 共視)
時間 t+2: 候選 = [KF102]          連續計數 = 3  (KF102 與 KF101 共視)
                                      ↓
                              確認為回環候選！
```

## 四、ComputeSim3（計算相似變換）

```cpp
bool LoopClosing::ComputeSim3() {
    // 1. 對每個候選幀進行 ORB 匹配
    ORBmatcher matcher(0.75, true);

    for (KeyFrame* pKF : mvpEnoughConsistentCandidates) {
        // 1.1 BoW 匹配
        vector<MapPoint*> vpMapPointMatches;
        int nmatches = matcher.SearchByBoW(mpCurrentKF, pKF, vpMapPointMatches);

        if (nmatches < 20)
            continue;

        // 1.2 使用 RANSAC 求解 Sim3
        Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF, pKF, vpMapPointMatches, mbFixScale);

        pSolver->SetRansacParameters(0.99, 20, 300);

        int nInliers;
        bool bNoMore;
        cv::Mat Scm;  // 相似變換 (scale, rotation, translation)

        while (!pSolver->iterate(5, bNoMore, vbInliers, nInliers, Scm)) {
            if (bNoMore)
                break;
        }

        if (nInliers >= 20) {
            // 找到有效的 Sim3
            g2o::Sim3 gScm(Converter::toMatrix3d(Scm.rowRange(0, 3).colRange(0, 3)),
                           Converter::toVector3d(Scm.rowRange(0, 3).col(3)),
                           pSolver->GetEstimatedScale());

            // 1.3 通過投影搜索更多匹配
            int nTotalMatches = matcher.SearchBySim3(mpCurrentKF, pKF, gScm, vpMapPointMatches);

            if (nTotalMatches >= 40) {
                // 1.4 優化 Sim3
                Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

                mpMatchedKF = pKF;
                mg2oScw = gScm;
                mvpCurrentMatchedPoints = vpMapPointMatches;

                return true;
            }
        }
    }

    return false;
}
```

### Sim3 vs SE3

| 變換類型 | 自由度 | 參數 | 用途 |
|----------|--------|------|------|
| SE3 | 6 | R, t | 雙目/RGB-D（尺度已知）|
| Sim3 | 7 | s, R, t | 單目（尺度未知）|

```
Sim3 變換：
p' = s * R * p + t

其中：
- s: 尺度因子
- R: 3x3 旋轉矩陣
- t: 平移向量
```

## 五、CorrectLoop（回環校正）

```cpp
void LoopClosing::CorrectLoop() {
    // 1. 停止 LocalMapping
    mpLocalMapper->RequestStop();
    while (!mpLocalMapper->isStopped()) {
        usleep(1000);
    }

    // 2. 獲取當前幀的共視關鍵幀
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    // 3. 計算校正後的位姿
    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;

    // 當前幀的校正位姿
    g2o::Sim3 g2oCorrectedScw = mg2oScw;
    CorrectedSim3[mpCurrentKF] = g2oCorrectedScw;

    // 當前幀的原始位姿
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();
    g2o::Sim3 g2oScw(Converter::toMatrix3d(Twc.rowRange(0, 3).colRange(0, 3)),
                     Converter::toVector3d(Twc.rowRange(0, 3).col(3)), 1.0);
    NonCorrectedSim3[mpCurrentKF] = g2oScw;

    // 4. 傳播校正到共視關鍵幀
    for (KeyFrame* pKFi : mvpCurrentConnectedKFs) {
        if (pKFi != mpCurrentKF) {
            cv::Mat Tiw = pKFi->GetPose();
            cv::Mat Tic = Tiw * Twc;  // 相對變換

            g2o::Sim3 g2oSic(Converter::toMatrix3d(Tic.rowRange(0, 3).colRange(0, 3)),
                             Converter::toVector3d(Tic.rowRange(0, 3).col(3)), 1.0);

            // 校正後的位姿 = 相對變換 × 當前幀校正位姿
            g2o::Sim3 g2oCorrectedSiw = g2oSic * g2oCorrectedScw;
            CorrectedSim3[pKFi] = g2oCorrectedSiw;

            // 原始位姿
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Tiw.rowRange(0, 3).colRange(0, 3)),
                             Converter::toVector3d(Tiw.rowRange(0, 3).col(3)), 1.0);
            NonCorrectedSim3[pKFi] = g2oSiw;
        }
    }

    // 5. 校正地圖點位置
    for (KeyFrame* pKFi : mvpCurrentConnectedKFs) {
        vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();

        for (MapPoint* pMPi : vpMPs) {
            if (pMPi && !pMPi->isBad()) {
                if (pMPi->mnCorrectedByKF != mpCurrentKF->mnId) {
                    // 獲取校正前後的位姿
                    g2o::Sim3 g2oCorrectedSwi = CorrectedSim3[pKFi].inverse();
                    g2o::Sim3 g2oSwi = NonCorrectedSim3[pKFi].inverse();

                    // 校正地圖點位置
                    cv::Mat P3Dw = pMPi->GetWorldPos();
                    Eigen::Vector3d eigP3Dw = Converter::toVector3d(P3Dw);
                    Eigen::Vector3d eigCorrectedP3Dw =
                        g2oCorrectedSwi.map(g2oSwi.inverse().map(eigP3Dw));
                    pMPi->SetWorldPos(Converter::toCvMat(eigCorrectedP3Dw));

                    pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                }
            }
        }
    }

    // 6. 更新關鍵幀位姿
    for (KeyFrame* pKFi : mvpCurrentConnectedKFs) {
        g2o::Sim3 g2oCorrectedSiw = CorrectedSim3[pKFi];
        // 提取 SE3 部分（忽略尺度）
        Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
        double s = g2oCorrectedSiw.scale();
        eigt = eigt / s;  // 歸一化

        cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);
        pKFi->SetPose(Tiw);
    }

    // 7. 融合重複的地圖點
    SearchAndFuse(CorrectedSim3);

    // 8. 更新連接關係
    for (KeyFrame* pKFi : mvpCurrentConnectedKFs) {
        pKFi->UpdateConnections();
    }

    // 9. Essential Graph 優化（位姿圖優化）
    Optimizer::OptimizeEssentialGraph(
        mpCurrentKF->GetMap(),
        mpMatchedKF,
        mpCurrentKF,
        NonCorrectedSim3,
        CorrectedSim3,
        LoopConnections,
        mbFixScale
    );

    // 10. 恢復 LocalMapping
    mpLocalMapper->Release();

    // 11. 啟動全局 BA
    mbRunningGBA = true;
    mptGlobalBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this);

    mnLastLoopKFid = mpCurrentKF->mnId;
}
```

### 回環校正示意圖

```
校正前：
    KF1 ── KF2 ── KF3 ── ... ── KF100
     │                           │
     │       累積漂移              │
     │                           │
     └─────────X─────────────────┘
              應該閉合但沒有

校正後：
    KF1 ── KF2 ── KF3 ── ... ── KF100
     │                           │
     │        校正位姿            │
     │                           │
     └───────────────────────────┘
              正確閉合
```

## 六、Essential Graph 優化

```cpp
void Optimizer::OptimizeEssentialGraph(
    Map* pMap,
    KeyFrame* pLoopKF,
    KeyFrame* pCurKF,
    const KeyFrameAndPose& NonCorrectedSim3,
    const KeyFrameAndPose& CorrectedSim3,
    const map<KeyFrame*, set<KeyFrame*>>& LoopConnections,
    const bool bFixScale
) {
    // 1. 構建 g2o 優化器
    g2o::SparseOptimizer optimizer;
    // ...

    // 2. 添加所有關鍵幀作為頂點
    for (KeyFrame* pKF : vpKFs) {
        if (pKF->isBad())
            continue;

        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        // 如果在校正列表中，使用校正後的位姿
        if (CorrectedSim3.count(pKF)) {
            VSim3->setEstimate(CorrectedSim3.at(pKF));
        } else {
            // 否則使用原始位姿
            VSim3->setEstimate(g2oSiw);
        }

        // 回環幀固定
        if (pKF == pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(pKF->mnId);
        optimizer.addVertex(VSim3);
    }

    // 3. 添加邊
    // 3.1 Spanning Tree 邊（父子關係）
    for (KeyFrame* pKF : vpKFs) {
        KeyFrame* pParentKF = pKF->GetParent();
        if (pParentKF) {
            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(0, optimizer.vertex(pKF->mnId));
            e->setVertex(1, optimizer.vertex(pParentKF->mnId));
            // 設置相對變換作為測量
            e->setMeasurement(Sji);
            optimizer.addEdge(e);
        }
    }

    // 3.2 回環邊
    for (KeyFrame* pKF : LoopConnections) {
        for (KeyFrame* pKFn : LoopConnections[pKF]) {
            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(0, optimizer.vertex(pKF->mnId));
            e->setVertex(1, optimizer.vertex(pKFn->mnId));
            e->setMeasurement(Snm);
            optimizer.addEdge(e);
        }
    }

    // 3.3 強共視邊（共視 > 100 個點）
    for (KeyFrame* pKF : vpKFs) {
        vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(100);
        for (KeyFrame* pKFn : vpConnectedKFs) {
            // 添加共視邊
        }
    }

    // 4. 執行優化
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    // 5. 更新所有關鍵幀和地圖點
    for (KeyFrame* pKF : vpKFs) {
        g2o::VertexSim3Expmap* VSim3 = ...;
        g2o::Sim3 CorrectedSiw = VSim3->estimate();
        pKF->SetPose(Tiw);
    }

    // 更新地圖點
    for (MapPoint* pMP : vpMPs) {
        // ...
    }
}
```

### Essential Graph 結構

```
       ┌────────────────────────────────────────────┐
       │              Essential Graph               │
       ├────────────────────────────────────────────┤
       │                                            │
       │    Spanning Tree     +    Loop Edges       │
       │    (最小連接)              (回環約束)        │
       │                                            │
       │     KF1                                    │
       │      │                                     │
       │     KF2 ── KF5                             │
       │      │      │                              │
       │     KF3    KF6 ────────┐                   │
       │      │      │          │ (回環邊)          │
       │     KF4    KF7         │                   │
       │             │          │                   │
       │            KF8 ────────┘                   │
       │                                            │
       └────────────────────────────────────────────┘
```

## 七、Global BA（全局光束法平差）

```cpp
void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF) {
    cout << "Starting Global Bundle Adjustment" << endl;

    // 1. 執行全局 BA
    Optimizer::GlobalBundleAdjustment(
        mpCurrentKF->GetMap(),  // 當前地圖
        10,                      // 迭代次數
        &mbStopGBA,             // 停止標誌
        nLoopKF,                // 回環 KF ID
        false                   // 不使用 IMU
    );

    // 2. 更新結果
    if (!mbStopGBA) {
        // 停止 LocalMapping
        mpLocalMapper->RequestStop();

        while (!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished()) {
            usleep(1000);
        }

        // 更新所有關鍵幀和地圖點
        cout << "Global BA finished, updating map..." << endl;

        // 傳播校正
        // ...

        // 恢復 LocalMapping
        mpLocalMapper->Release();
    }

    mbFinishedGBA = true;
    mbRunningGBA = false;
}
```

### Global BA vs Local BA

| 項目 | Local BA | Global BA |
|------|----------|-----------|
| 範圍 | 局部共視區域 | 整個地圖 |
| 時機 | 每個新關鍵幀 | 回環校正後 |
| 耗時 | 數十毫秒 | 數秒到數分鐘 |
| 線程 | LocalMapping | 獨立線程 |

## 八、地圖合併（Map Merging）

ORB-SLAM3 支援多地圖管理，當檢測到兩個地圖可以合併時：

```cpp
void LoopClosing::MergeLocal() {
    // 1. 獲取當前地圖和合併地圖
    Map* pCurrentMap = mpCurrentKF->GetMap();
    Map* pMergeMap = mpMergeMatchedKF->GetMap();

    // 2. 計算兩個地圖之間的變換
    g2o::Sim3 gSmw = mg2oMergeScw;  // 合併變換

    // 3. 變換當前地圖的關鍵幀和地圖點
    for (KeyFrame* pKFi : pCurrentMap->GetAllKeyFrames()) {
        // 更新位姿
        g2o::Sim3 g2oSiw = g2oScw * relativeSim3;
        // ...
    }

    // 4. 融合地圖點
    SearchAndFuse(CorrectedSim3, vpCheckFuseMapPoint);

    // 5. 將當前地圖的元素移動到合併地圖
    for (KeyFrame* pKFi : pCurrentMap->GetAllKeyFrames()) {
        pMergeMap->AddKeyFrame(pKFi);
        pKFi->UpdateMap(pMergeMap);
    }

    for (MapPoint* pMPi : pCurrentMap->GetAllMapPoints()) {
        pMergeMap->AddMapPoint(pMPi);
        pMPi->UpdateMap(pMergeMap);
    }

    // 6. 刪除舊地圖
    mpAtlas->SetMapBad(pCurrentMap);

    // 7. Essential Graph 優化
    Optimizer::OptimizeEssentialGraph(...);

    // 8. 全局 BA
    mbRunningGBA = true;
    mptGlobalBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this);
}
```

### 多地圖場景

```
場景 1: 機器人進入新區域（tracking lost）
    → 創建新地圖

場景 2: 機器人回到之前區域
    → 檢測到回環
    → 合併兩個地圖

┌──────────────────────────────────────────────────────────┐
│                         Atlas                            │
├──────────────────────────────────────────────────────────┤
│                                                          │
│   Map 1 (Active)    Map 2         Map 3                  │
│   ┌─────────┐      ┌─────────┐   ┌─────────┐             │
│   │ KF1-50  │      │ KF51-80 │   │ KF81-   │             │
│   │ MP1-500 │      │ MP-     │   │ MP-     │             │
│   └─────────┘      └─────────┘   └─────────┘             │
│        │                │                                │
│        └────合併────────┘                                │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

## 九、線程同步

### 與其他線程的協調

```cpp
// 停止 LocalMapping
void LoopClosing::CorrectLoop() {
    mpLocalMapper->RequestStop();
    while (!mpLocalMapper->isStopped()) {
        usleep(1000);
    }

    // ... 執行回環校正 ...

    mpLocalMapper->Release();
}

// 中斷 Global BA
void LoopClosing::RequestReset() {
    mbStopGBA = true;

    while (mbRunningGBA) {
        usleep(1000);
    }
}
```

### Mutex 保護

```cpp
class LoopClosing {
private:
    std::mutex mMutexLoopQueue;    // 保護關鍵幀隊列
    std::mutex mMutexGBA;          // 保護 GBA 狀態
    std::mutex mMutexFinish;       // 保護完成標誌

public:
    void InsertKeyFrame(KeyFrame* pKF) {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mlpLoopKeyFrameQueue.push_back(pKF);
    }
};
```

## 十、性能與調優

### 關鍵參數

| 參數 | 典型值 | 說明 |
|------|--------|------|
| minScore | 0.01 | BoW 相似度閾值 |
| mnCovisibilityConsistencyTh | 3 | 連續性檢測閾值 |
| 回環間隔 | 10 幀 | 避免頻繁回環 |
| GBA 迭代次數 | 10-20 | 全局 BA 迭代 |

### 常見問題

| 問題 | 可能原因 | 解決方向 |
|------|----------|----------|
| 回環檢測失敗 | BoW 閾值太高 | 降低 minScore |
| 誤檢測（假回環） | 場景重複性高 | 增加連續性要求 |
| GBA 太慢 | 地圖太大 | 分段優化 |
| 校正後漂移 | Sim3 計算不準 | 增加匹配點數 |
