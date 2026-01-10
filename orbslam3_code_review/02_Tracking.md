# ORB-SLAM3 Tracking 線程詳解

## 一、Tracking 概述

Tracking 是 ORB-SLAM3 的**主線程**，負責處理每一幀輸入影像，估計相機位姿。

```
輸入影像 → 特徵提取 → 位姿估計 → 關鍵幀決策 → 輸出位姿
                           ↓
                    LocalMapping（如果是關鍵幀）
```

## 二、主要狀態機

```cpp
// Tracking.h
enum eTrackingState {
    SYSTEM_NOT_READY = -1,
    NO_IMAGES_YET = 0,
    NOT_INITIALIZED = 1,
    OK = 2,
    RECENTLY_LOST = 3,
    LOST = 4,
    OK_KLT = 5
};
```

### 狀態轉換圖

```
                    ┌─────────────────────────────────────────┐
                    │                                         │
                    ▼                                         │
┌──────────────┐   初始化成功   ┌─────┐   丟失少量幀   ┌─────────────┐
│NOT_INITIALIZED│────────────▶│ OK  │──────────────▶│RECENTLY_LOST│
└──────────────┘              └─────┘               └─────────────┘
                                 ▲                        │
                                 │                        │ 持續丟失
                             重定位成功                    ▼
                                 │                   ┌─────┐
                                 └───────────────────│LOST │
                                                     └─────┘
```

## 三、Track() 主函數流程

```cpp
// Tracking.cc
void Tracking::Track() {
    // 1. 檢查狀態
    if (mState == NO_IMAGES_YET) {
        mState = NOT_INITIALIZED;
    }

    // 2. 未初始化 → 執行初始化
    if (mState == NOT_INITIALIZED) {
        if (mSensor == System::STEREO || mSensor == System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();
        return;
    }

    // 3. 已初始化 → 執行跟蹤
    bool bOK;

    if (mState == OK) {
        // 3.1 有運動模型 → 用運動模型預測
        if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2) {
            bOK = TrackReferenceKeyFrame();
        } else {
            bOK = TrackWithMotionModel();
            if (!bOK)
                bOK = TrackReferenceKeyFrame();
        }
    } else {
        // 3.2 丟失狀態 → 重定位
        bOK = Relocalization();
    }

    // 4. 跟蹤局部地圖
    if (bOK)
        bOK = TrackLocalMap();

    // 5. 更新狀態
    if (bOK)
        mState = OK;
    else
        mState = LOST;

    // 6. 決定是否創建關鍵幀
    if (NeedNewKeyFrame())
        CreateNewKeyFrame();

    // 7. 計算速度（用於運動模型）
    if (bOK) {
        mVelocity = mCurrentFrame.mTcw * mLastFrame.mTcw.inv();
    }
}
```

## 四、三種跟蹤策略

### 4.1 TrackWithMotionModel（運動模型跟蹤）

**適用**：相機連續運動，速度變化不大

```cpp
bool Tracking::TrackWithMotionModel() {
    // 1. 用速度模型預測當前位姿
    mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);

    // 2. 投影上一幀的地圖點到當前幀
    int nmatches = SearchByProjection(mCurrentFrame, mLastFrame, th);

    // 3. 優化位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // 4. 剔除外點
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (mCurrentFrame.mvbOutlier[i]) {
                // 標記為外點的地圖點
                mCurrentFrame.mvpMapPoints[i] = nullptr;
            } else {
                nmatchesMap++;
            }
        }
    }

    return nmatchesMap >= 10;  // 至少需要 10 個匹配
}
```

**流程圖**：
```
上一幀位姿 + 速度 → 預測當前位姿 → 投影搜索 → 位姿優化 → 剔除外點
```

### 4.2 TrackReferenceKeyFrame（參考關鍵幀跟蹤）

**適用**：沒有運動模型、運動模型失敗

```cpp
bool Tracking::TrackReferenceKeyFrame() {
    // 1. 計算當前幀的 BoW
    mCurrentFrame.ComputeBoW();

    // 2. 用 BoW 進行特徵匹配
    ORBmatcher matcher(0.7, true);
    vector<MapPoint*> vpMapPointMatches;
    int nmatches = matcher.SearchByBoW(
        mpReferenceKF,      // 參考關鍵幀
        mCurrentFrame,      // 當前幀
        vpMapPointMatches   // 匹配結果
    );

    if (nmatches < 15)
        return false;

    // 3. 設置初始位姿（用參考關鍵幀的位姿）
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    // 4. 優化位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // 5. 剔除外點，統計有效匹配數
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (mCurrentFrame.mvbOutlier[i]) {
                mCurrentFrame.mvpMapPoints[i] = nullptr;
            } else {
                nmatchesMap++;
            }
        }
    }

    return nmatchesMap >= 10;
}
```

**BoW（詞袋模型）用途**：
- 將特徵描述子轉換為「視覺單詞」
- 加速特徵匹配（只比較相同 word 的特徵）
- 用於回環檢測和重定位

### 4.3 Relocalization（重定位）

**適用**：跟蹤丟失，需要從地圖中找回位置

```cpp
bool Tracking::Relocalization() {
    // 1. 計算當前幀的 BoW
    mCurrentFrame.ComputeBoW();

    // 2. 用 BoW 在關鍵幀數據庫中搜索候選幀
    vector<KeyFrame*> vpCandidateKFs =
        mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if (vpCandidateKFs.empty())
        return false;

    // 3. 對每個候選幀嘗試匹配
    for (KeyFrame* pKF : vpCandidateKFs) {
        // 3.1 BoW 匹配
        int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);

        if (nmatches < 15)
            continue;

        // 3.2 PnP RANSAC 求解位姿
        PnPsolver* pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
        cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

        if (!Tcw.empty()) {
            mCurrentFrame.SetPose(Tcw);

            // 3.3 位姿優化
            int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

            if (nGood >= 50)
                return true;

            // 3.4 如果不夠好，嘗試投影更多點
            int nadditional = SearchByProjection(...);
            if (nGood + nadditional >= 50)
                return true;
        }
    }

    return false;
}
```

**重定位流程**：
```
BoW 搜索候選幀 → 特徵匹配 → PnP RANSAC → 位姿優化 → 投影驗證
```

## 五、TrackLocalMap（局部地圖跟蹤）

這是精化位姿的關鍵步驟：

```cpp
bool Tracking::TrackLocalMap() {
    // 1. 更新局部地圖（局部關鍵幀 + 局部地圖點）
    UpdateLocalMap();

    // 2. 搜索局部地圖點中的更多匹配
    SearchLocalPoints();

    // 3. 再次優化位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // 4. 統計內點數
    int mnMatchesInliers = 0;
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (!mCurrentFrame.mvbOutlier[i]) {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                mnMatchesInliers++;
            }
        }
    }

    // 5. 根據內點數決定是否成功
    if (mnMatchesInliers < 30)
        return false;
    else
        return true;
}
```

### 局部地圖的定義

```cpp
void Tracking::UpdateLocalMap() {
    // 局部關鍵幀 =
    //   1. 與當前幀共享地圖點的關鍵幀
    //   2. 這些關鍵幀的鄰居

    // 局部地圖點 =
    //   局部關鍵幀觀測到的所有地圖點
}
```

## 六、關鍵幀決策 NeedNewKeyFrame()

```cpp
bool Tracking::NeedNewKeyFrame() {
    // 條件 1：LocalMapping 空閒
    if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    // 條件 2：距離上次關鍵幀足夠遠
    const int nKFs = mpAtlas->GetCurrentMap()->KeyFramesInMap();
    if (mCurrentFrame.mnId < mnLastKeyFrameId + mMaxFrames && nKFs > mMaxFrames)
        return false;

    // 條件 3：當前幀跟蹤到足夠多的地圖點
    int nMinObs = 3;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // 條件 4：跟蹤質量下降
    bool bNeedToInsert = false;

    // 4.1 跟蹤的地圖點數量 < 參考關鍵幀的 90%
    if (mnMatchesInliers < nRefMatches * 0.9)
        bNeedToInsert = true;

    // 4.2 跟蹤的地圖點數量 < 參考關鍵幀的 75%（更寬鬆）
    if (mnMatchesInliers < nRefMatches * 0.75) {
        if (mbOnlyTracking)  // 純定位模式不需要
            return false;
        bNeedToInsert = true;
    }

    // 條件 5：IMU 模式下的特殊處理
    // ...

    return bNeedToInsert;
}
```

### 關鍵幀選取原則

| 條件 | 說明 |
|------|------|
| LocalMapping 空閒 | 避免隊列堆積 |
| 時間間隔 | 不能太頻繁 |
| 視覺變化 | 跟蹤質量下降時需要新關鍵幀 |
| 地圖覆蓋 | 確保地圖完整性 |

## 七、CreateNewKeyFrame（創建關鍵幀）

```cpp
void Tracking::CreateNewKeyFrame() {
    // 1. 創建新關鍵幀
    KeyFrame* pKF = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

    // 2. 設為參考關鍵幀
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    // 3. 對於雙目/RGB-D：創建新地圖點
    if (mSensor != System::MONOCULAR) {
        mCurrentFrame.UpdatePoseMatrices();

        // 將有深度的特徵點創建為地圖點
        vector<pair<float, int>> vDepthIdx;
        for (int i = 0; i < mCurrentFrame.N; i++) {
            float z = mCurrentFrame.mvDepth[i];
            if (z > 0) {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        sort(vDepthIdx.begin(), vDepthIdx.end());

        // 創建地圖點（近的優先）
        int nPoints = 0;
        for (auto& p : vDepthIdx) {
            if (mCurrentFrame.mvpMapPoints[p.second] == nullptr) {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(p.second);
                MapPoint* pNewMP = new MapPoint(x3D, pKF, mpAtlas->GetCurrentMap());
                pNewMP->AddObservation(pKF, p.second);
                pKF->AddMapPoint(pNewMP, p.second);
                mCurrentFrame.mvpMapPoints[p.second] = pNewMP;
                mpAtlas->AddMapPoint(pNewMP);
                nPoints++;
            }

            if (nPoints >= 100)  // 最多創建 100 個
                break;
        }
    }

    // 4. 插入到 LocalMapping 隊列
    mpLocalMapper->InsertKeyFrame(pKF);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}
```

## 八、IMU 積分（慣性模式）

```cpp
void Tracking::PreintegrateIMU() {
    // 1. 獲取兩幀之間的 IMU 數據
    while (vImuMeas[i].t < mCurrentFrame.mTimeStamp) {
        // 2. IMU 預積分
        mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(
            vImuMeas[i].a,  // 加速度
            vImuMeas[i].w,  // 角速度
            dt              // 時間間隔
        );
    }
}

void Tracking::PredictStateIMU() {
    // 用 IMU 預積分預測當前位姿
    const cv::Mat Rwb = ...;  // 旋轉
    const cv::Mat twb = ...;  // 平移
    const cv::Mat Vwb = ...;  // 速度

    mCurrentFrame.SetImuPoseVelocity(Rwb, twb, Vwb);
}
```

## 九、數據結構總覽

### Frame（當前幀）

```cpp
class Frame {
public:
    // 特徵點
    std::vector<cv::KeyPoint> mvKeys;       // 原始特徵點
    std::vector<cv::KeyPoint> mvKeysUn;     // 去畸變後
    cv::Mat mDescriptors;                    // 描述子

    // 地圖點關聯
    std::vector<MapPoint*> mvpMapPoints;    // 每個特徵點對應的地圖點
    std::vector<bool> mvbOutlier;           // 外點標記

    // 位姿
    cv::Mat mTcw;                           // 相機位姿（world to camera）

    // 深度（雙目/RGB-D）
    std::vector<float> mvDepth;             // 特徵點深度

    // BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;
};
```

## 十、調試技巧

### 常見問題排查

| 問題 | 可能原因 | 排查方向 |
|------|----------|----------|
| 初始化失敗 | 特徵點太少、視差不足 | 檢查 ORB 參數、運動幅度 |
| 跟蹤經常丟失 | 運動太快、場景變化大 | 調整閾值、增加特徵數 |
| 漂移嚴重 | 尺度不準、回環失敗 | 檢查 IMU 標定、回環檢測 |

### 關鍵日誌

```cpp
// Tracking.cc 中的輸出
cout << "Track: " << mnMatchesInliers << " inliers" << endl;
cout << "New KeyFrame created" << endl;
cout << "Relocalization" << endl;
```
