# ORB-SLAM3 LocalMapping 線程詳解

## 一、LocalMapping 概述

LocalMapping 是**子線程**，負責處理新關鍵幀、創建新地圖點、執行局部 BA 優化。

```
Tracking 插入關鍵幀 → LocalMapping 處理 → 優化後的局部地圖
                                              ↓
                                        LoopClosing
```

## 二、Run() 主循環

```cpp
// LocalMapping.cc
void LocalMapping::Run() {
    mbFinished = false;

    while (1) {
        // 1. 告訴 Tracking 不要插入新關鍵幀
        SetAcceptKeyFrames(false);

        // 2. 檢查隊列中是否有新關鍵幀
        if (CheckNewKeyFrames()) {
            // 2.1 處理新關鍵幀
            ProcessNewKeyFrame();

            // 2.2 剔除不好的地圖點
            MapPointCulling();

            // 2.3 創建新地圖點（通過三角化）
            CreateNewMapPoints();

            // 2.4 如果隊列空了，搜索更多匹配
            if (!CheckNewKeyFrames()) {
                SearchInNeighbors();
            }

            // 2.5 執行局部 BA
            if (!CheckNewKeyFrames()) {
                if (mpAtlas->GetCurrentMap()->KeyFramesInMap() > 2) {
                    Optimizer::LocalBundleAdjustment(
                        mpCurrentKeyFrame,
                        &mbAbortBA,
                        mpAtlas->GetCurrentMap()
                    );
                }

                // 2.6 剔除冗餘關鍵幀
                KeyFrameCulling();
            }

            // 2.7 將關鍵幀傳給 LoopClosing
            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }

        // 3. 允許 Tracking 插入新關鍵幀
        SetAcceptKeyFrames(true);

        // 4. 等待新任務
        if (CheckFinish())
            break;

        usleep(3000);  // 3ms
    }

    SetFinish();
}
```

### 流程圖

```
┌──────────────────────────────────────────────────────────────────┐
│                      LocalMapping 主循環                          │
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
        │ ProcessNewKeyFrame  │     │   Sleep     │
        └─────────────────────┘     └─────────────┘
                    │
                    ▼
        ┌─────────────────────┐
        │  MapPointCulling    │
        └─────────────────────┘
                    │
                    ▼
        ┌─────────────────────┐
        │ CreateNewMapPoints  │
        └─────────────────────┘
                    │
                    ▼
        ┌─────────────────────┐
        │ SearchInNeighbors   │
        └─────────────────────┘
                    │
                    ▼
        ┌─────────────────────┐
        │   Local BA          │
        └─────────────────────┘
                    │
                    ▼
        ┌─────────────────────┐
        │  KeyFrameCulling    │
        └─────────────────────┘
                    │
                    ▼
        ┌─────────────────────┐
        │ → LoopClosing       │
        └─────────────────────┘
```

## 三、ProcessNewKeyFrame（處理新關鍵幀）

```cpp
void LocalMapping::ProcessNewKeyFrame() {
    // 1. 從隊列取出關鍵幀
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // 2. 計算 BoW（用於回環檢測）
    mpCurrentKeyFrame->ComputeBoW();

    // 3. 將 Tracking 中關聯的地圖點與關鍵幀建立雙向連接
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for (size_t i = 0; i < vpMapPointMatches.size(); i++) {
        MapPoint* pMP = vpMapPointMatches[i];
        if (pMP && !pMP->isBad()) {
            if (!pMP->IsInKeyFrame(mpCurrentKeyFrame)) {
                // 地圖點添加對這個關鍵幀的觀測
                pMP->AddObservation(mpCurrentKeyFrame, i);
                // 更新地圖點的平均觀測方向和深度
                pMP->UpdateNormalAndDepth();
                // 更新描述子（取中位數）
                pMP->ComputeDistinctiveDescriptors();
            }
        }
    }

    // 4. 更新關鍵幀之間的連接關係（共視圖）
    mpCurrentKeyFrame->UpdateConnections();

    // 5. 將關鍵幀插入地圖
    mpAtlas->AddKeyFrame(mpCurrentKeyFrame);
}
```

### 共視圖（Covisibility Graph）

```cpp
void KeyFrame::UpdateConnections() {
    // 統計與其他關鍵幀共享的地圖點數量
    map<KeyFrame*, int> KFcounter;

    for (MapPoint* pMP : mvpMapPoints) {
        if (pMP && !pMP->isBad()) {
            map<KeyFrame*, size_t> observations = pMP->GetObservations();
            for (auto& obs : observations) {
                if (obs.first->mnId != mnId)
                    KFcounter[obs.first]++;
            }
        }
    }

    // 共享點數 > 15 的建立連接
    vector<pair<int, KeyFrame*>> vPairs;
    for (auto& kf : KFcounter) {
        if (kf.second >= 15) {
            vPairs.push_back(make_pair(kf.second, kf.first));
            kf.first->AddConnection(this, kf.second);
        }
    }

    // 按共享點數排序
    sort(vPairs.begin(), vPairs.end());

    // 儲存排序後的連接
    mConnectedKeyFrameWeights = KFcounter;
    mvpOrderedConnectedKeyFrames = ...;
}
```

## 四、MapPointCulling（地圖點剔除）

```cpp
void LocalMapping::MapPointCulling() {
    list<MapPoint*>::iterator it = mlpRecentAddedMapPoints.begin();

    while (it != mlpRecentAddedMapPoints.end()) {
        MapPoint* pMP = *it;

        if (pMP->isBad()) {
            // 已經被標記為壞點
            it = mlpRecentAddedMapPoints.erase(it);
        }
        else if (pMP->GetFoundRatio() < 0.25f) {
            // 條件 1：被找到的比例太低
            // FoundRatio = 被跟蹤到的次數 / 應該被看到的次數
            pMP->SetBadFlag();
            it = mlpRecentAddedMapPoints.erase(it);
        }
        else if (mnCurrentKFId - pMP->mnFirstKFid >= 2 &&
                 pMP->Observations() <= 2) {
            // 條件 2：創建超過 2 幀，但觀測數 <= 2
            pMP->SetBadFlag();
            it = mlpRecentAddedMapPoints.erase(it);
        }
        else if (mnCurrentKFId - pMP->mnFirstKFid >= 3) {
            // 條件 3：超過 3 幀，認為已經穩定
            it = mlpRecentAddedMapPoints.erase(it);
        }
        else {
            it++;
        }
    }
}
```

### 剔除標準

| 條件 | 說明 |
|------|------|
| FoundRatio < 25% | 地圖點經常無法被跟蹤到 |
| 觀測數 <= 2 且 > 2 幀 | 只被少數關鍵幀看到 |
| > 3 幀 | 已經穩定，從候選列表移除 |

## 五、CreateNewMapPoints（創建新地圖點）

```cpp
void LocalMapping::CreateNewMapPoints() {
    // 1. 獲取共視程度最高的 nn 個關鍵幀
    int nn = 10;  // 單目
    if (mbMonocular)
        nn = 20;

    vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6, false);

    // 當前幀的相機參數
    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    // 2. 遍歷每個鄰居關鍵幀
    for (KeyFrame* pKF2 : vpNeighKFs) {
        // 2.1 檢查基線長度
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2 - Ow1;
        const float baseline = cv::norm(vBaseline);

        // 基線太短，跳過
        if (baseline < pKF2->mb)  // mb = baseline * fx
            continue;

        // 2.2 計算基礎矩陣
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame, pKF2);

        // 2.3 通過極線約束匹配特徵點
        vector<pair<size_t, size_t>> vMatchedIndices;
        matcher.SearchForTriangulation(
            mpCurrentKeyFrame, pKF2, F12, vMatchedIndices, false
        );

        // 2.4 對每對匹配進行三角化
        for (auto& match : vMatchedIndices) {
            const cv::KeyPoint& kp1 = mpCurrentKeyFrame->mvKeysUn[match.first];
            const cv::KeyPoint& kp2 = pKF2->mvKeysUn[match.second];

            // 三角化
            cv::Mat x3D;
            Triangulate(kp1, kp2, P1, P2, x3D);

            // 檢查深度為正
            float z1 = Rcw1.row(2).dot(x3D) + tcw1.at<float>(2);
            if (z1 <= 0)
                continue;

            // 檢查重投影誤差
            // ...

            // 創建地圖點
            MapPoint* pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpAtlas->GetCurrentMap());

            pMP->AddObservation(mpCurrentKeyFrame, match.first);
            pMP->AddObservation(pKF2, match.second);

            mpCurrentKeyFrame->AddMapPoint(pMP, match.first);
            pKF2->AddMapPoint(pMP, match.second);

            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();

            mpAtlas->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);
        }
    }
}
```

### 三角化示意圖

```
         O1 ─────────────── O2
        /  \               /  \
       /    \             /    \
      /      \           /      \
     /        \         /        \
    ▼          ▼       ▼          ▼
   p1          P(x,y,z)          p2
```

- O1, O2：兩個相機中心
- p1, p2：特徵點在各自影像上的投影
- P：三角化得到的 3D 點

## 六、SearchInNeighbors（鄰居搜索融合）

```cpp
void LocalMapping::SearchInNeighbors() {
    // 1. 獲取二級共視關鍵幀
    int nn = 10;
    vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    vector<KeyFrame*> vpTargetKFs;
    for (KeyFrame* pKFi : vpNeighKFs) {
        if (!pKFi->isBad() && pKFi->mnFuseTargetForKF != mpCurrentKeyFrame->mnId) {
            vpTargetKFs.push_back(pKFi);
            pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

            // 加入二級鄰居
            vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
            for (KeyFrame* pKFi2 : vpSecondNeighKFs) {
                if (!pKFi2->isBad() &&
                    pKFi2->mnFuseTargetForKF != mpCurrentKeyFrame->mnId &&
                    pKFi2->mnId != mpCurrentKeyFrame->mnId) {
                    vpTargetKFs.push_back(pKFi2);
                }
            }
        }
    }

    // 2. 將當前幀的地圖點投影到鄰居幀，尋找融合
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for (KeyFrame* pKFi : vpTargetKFs) {
        matcher.Fuse(pKFi, vpMapPointMatches);
    }

    // 3. 將鄰居幀的地圖點投影到當前幀
    vector<MapPoint*> vpFuseCandidates;
    for (KeyFrame* pKFi : vpTargetKFs) {
        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();
        for (MapPoint* pMP : vpMapPointsKFi) {
            if (pMP && !pMP->isBad()) {
                vpFuseCandidates.push_back(pMP);
            }
        }
    }

    matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates);

    // 4. 更新地圖點描述子和連接關係
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for (MapPoint* pMP : vpMapPointMatches) {
        if (pMP && !pMP->isBad()) {
            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();
        }
    }

    mpCurrentKeyFrame->UpdateConnections();
}
```

## 七、Local BA（局部光束法平差）

```cpp
void Optimizer::LocalBundleAdjustment(
    KeyFrame* pKF, bool* pbStopFlag, Map* pMap
) {
    // 1. 定義局部關鍵幀
    list<KeyFrame*> lLocalKeyFrames;
    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    // 加入共視關鍵幀
    vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for (KeyFrame* pKFi : vNeighKFs) {
        pKFi->mnBALocalForKF = pKF->mnId;
        lLocalKeyFrames.push_back(pKFi);
    }

    // 2. 定義局部地圖點
    list<MapPoint*> lLocalMapPoints;
    for (KeyFrame* pKFi : lLocalKeyFrames) {
        for (MapPoint* pMP : pKFi->GetMapPointMatches()) {
            if (pMP && !pMP->isBad()) {
                if (pMP->mnBALocalForKF != pKF->mnId) {
                    lLocalMapPoints.push_back(pMP);
                    pMP->mnBALocalForKF = pKF->mnId;
                }
            }
        }
    }

    // 3. 定義固定關鍵幀（只觀測但不優化）
    list<KeyFrame*> lFixedCameras;
    for (MapPoint* pMP : lLocalMapPoints) {
        map<KeyFrame*, size_t> observations = pMP->GetObservations();
        for (auto& obs : observations) {
            KeyFrame* pKFi = obs.first;
            if (pKFi->mnBALocalForKF != pKF->mnId &&
                pKFi->mnBAFixedForKF != pKF->mnId) {
                pKFi->mnBAFixedForKF = pKF->mnId;
                lFixedCameras.push_back(pKFi);
            }
        }
    }

    // 4. 構建 g2o 優化器
    g2o::SparseOptimizer optimizer;
    // ... 設置求解器

    // 5. 添加頂點（位姿和地圖點）
    for (KeyFrame* pKFi : lLocalKeyFrames) {
        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == 0);  // 第一幀固定
        optimizer.addVertex(vSE3);
    }

    for (MapPoint* pMP : lLocalMapPoints) {
        g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        vPoint->setId(pMP->mnId + maxKFid + 1);
        vPoint->setMarginalized(true);  // 邊緣化
        optimizer.addVertex(vPoint);
    }

    // 6. 添加邊（重投影誤差）
    for (MapPoint* pMP : lLocalMapPoints) {
        for (auto& obs : pMP->GetObservations()) {
            KeyFrame* pKFi = obs.first;
            // 創建邊
            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
            e->setVertex(0, optimizer.vertex(pMP->mnId + maxKFid + 1));
            e->setVertex(1, optimizer.vertex(pKFi->mnId));
            e->setMeasurement(obs_point);
            e->setInformation(Matrix2d::Identity());
            e->setRobustKernel(new g2o::RobustKernelHuber);
            optimizer.addEdge(e);
        }
    }

    // 7. 執行優化
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // 8. 檢查外點
    // ...

    // 9. 再次優化（排除外點）
    optimizer.optimize(10);

    // 10. 更新關鍵幀位姿和地圖點位置
    for (KeyFrame* pKFi : lLocalKeyFrames) {
        g2o::VertexSE3Expmap* vSE3 = ...;
        pKFi->SetPose(Converter::toCvMat(vSE3->estimate()));
    }

    for (MapPoint* pMP : lLocalMapPoints) {
        g2o::VertexPointXYZ* vPoint = ...;
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
    }
}
```

### Local BA 結構

```
┌─────────────────────────────────────────────────────────────┐
│                      Local BA 範圍                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   ┌─────────────────────────────────────────────────┐       │
│   │           Local KeyFrames (優化)                 │       │
│   │    KF1 ─── KF2 ─── KF3 ─── KF4 ─── KF5         │       │
│   │     │       │       │       │       │          │       │
│   │     ▼       ▼       ▼       ▼       ▼          │       │
│   │    MP1     MP2     MP3     MP4     MP5         │       │
│   │           Local MapPoints (優化)                │       │
│   └─────────────────────────────────────────────────┘       │
│                          │                                   │
│                          ▼                                   │
│   ┌─────────────────────────────────────────────────┐       │
│   │        Fixed KeyFrames (只提供約束)              │       │
│   │              KF_old ─── KF_older                │       │
│   └─────────────────────────────────────────────────┘       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## 八、KeyFrameCulling（關鍵幀剔除）

```cpp
void LocalMapping::KeyFrameCulling() {
    // 獲取當前幀的共視關鍵幀
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for (KeyFrame* pKF : vpLocalKeyFrames) {
        if (pKF->mnId == 0)  // 不剔除第一幀
            continue;

        // 統計該關鍵幀的地圖點被多少其他關鍵幀觀測到
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs = nObs;
        int nRedundantObservations = 0;
        int nMPs = 0;

        for (MapPoint* pMP : vpMapPoints) {
            if (pMP && !pMP->isBad()) {
                nMPs++;

                if (pMP->Observations() > thObs) {
                    // 這個地圖點被超過 thObs 個關鍵幀觀測到
                    const int scaleLevel = pKF->mvKeysUn[i].octave;
                    const map<KeyFrame*, size_t> observations = pMP->GetObservations();

                    int nObs = 0;
                    for (auto& obs : observations) {
                        KeyFrame* pKFi = obs.first;
                        if (pKFi == pKF)
                            continue;

                        const int scaleLeveli = pKFi->mvKeysUn[obs.second].octave;

                        // 在相同或更好的尺度觀測到
                        if (scaleLeveli <= scaleLevel + 1) {
                            nObs++;
                            if (nObs >= thObs)
                                break;
                        }
                    }

                    if (nObs >= thObs)
                        nRedundantObservations++;
                }
            }
        }

        // 如果 90% 的地圖點都能被其他關鍵幀觀測到，則剔除這個關鍵幀
        if (nRedundantObservations > 0.9 * nMPs) {
            pKF->SetBadFlag();
        }
    }
}
```

### 剔除邏輯

```
地圖點 MP1 被 KF1, KF2, KF3 觀測到
地圖點 MP2 被 KF1, KF2, KF4 觀測到
...

如果 KF1 的 90% 地圖點都被其他 >= 3 個關鍵幀觀測到
→ KF1 是冗餘的，可以剔除
```

## 九、IMU 預積分優化

```cpp
void LocalMapping::Run() {
    // ...

    // IMU 模式下的特殊處理
    if (mpTracker->mSensor == System::IMU_STEREO ||
        mpTracker->mSensor == System::IMU_MONOCULAR) {

        // 初始化 IMU
        if (!mpCurrentKeyFrame->GetMap()->isImuInitialized()) {
            if (mbMonocular)
                InitializeIMU(1e2, 1e10, true);
            else
                InitializeIMU(1e2, 1e10, true);
        }

        // IMU + 視覺聯合優化
        if (mpCurrentKeyFrame->GetMap()->isImuInitialized()) {
            Optimizer::LocalInertialBA(
                mpCurrentKeyFrame,
                &mbAbortBA,
                mpCurrentKeyFrame->GetMap()
            );
        }
    }
}
```

## 十、線程同步機制

### Mutex 保護

```cpp
class LocalMapping {
private:
    std::mutex mMutexNewKFs;       // 保護新關鍵幀隊列
    std::mutex mMutexStop;         // 保護停止標誌
    std::mutex mMutexAccept;       // 保護接受關鍵幀標誌
    std::mutex mMutexFinish;       // 保護完成標誌

public:
    void InsertKeyFrame(KeyFrame* pKF) {
        unique_lock<mutex> lock(mMutexNewKFs);
        mlNewKeyFrames.push_back(pKF);
    }

    bool CheckNewKeyFrames() {
        unique_lock<mutex> lock(mMutexNewKFs);
        return !mlNewKeyFrames.empty();
    }
};
```

### 與 Tracking 的協調

```cpp
// Tracking 檢查是否可以插入關鍵幀
bool Tracking::NeedNewKeyFrame() {
    // ...
    if (mpLocalMapper->isStopped() ||
        mpLocalMapper->stopRequested()) {
        return false;
    }

    // LocalMapping 隊列太長時不插入
    if (mpLocalMapper->KeyframesInQueue() > 3)
        return false;
}
```

## 十一、性能考慮

| 操作 | 時間複雜度 | 說明 |
|------|------------|------|
| ProcessNewKeyFrame | O(N) | N = 地圖點數 |
| CreateNewMapPoints | O(K×M) | K = 鄰居數, M = 特徵點數 |
| Local BA | O(N×M) | 迭代優化 |
| KeyFrameCulling | O(K×N) | K = 共視幀數 |

### 優化策略

1. **設置 `mbAbortBA`**：當有新關鍵幀時中斷 BA
2. **限制局部範圍**：只優化共視關鍵幀
3. **邊緣化**：地圖點設為可邊緣化，加速求解
