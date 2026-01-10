# ORB-SLAM3 系統架構總覽

## 一、檔案結構

```
ORB_SLAM3/
├── examples/                    # main 函數入口（依傳感器分類）
│   ├── Monocular/               # 單目相機
│   ├── Stereo/                  # 雙目相機
│   ├── RGB-D/                   # RGB-D 相機
│   ├── Monocular-Inertial/      # 單目 + IMU
│   ├── Stereo-Inertial/         # 雙目 + IMU
│   └── ROS/                     # ROS 節點
├── include/                     # 頭文件 (.h)
├── src/                         # 源文件 (.cc)
├── Thirdparty/                  # 第三方庫
│   ├── DBoW2/                   # 詞袋模型（回環/重定位）
│   ├── Sophus/                  # 李代數（SE3 位姿表示）
│   └── g2o/                     # 圖優化（BA）
└── Vocabulary/                  # ORB 詞典文件
```

## 二、System 構造函數

```cpp
// System.h
class System {
public:
    System(const string& strVocFile,      // 詞典路徑
           const string& strSettingsFile, // 配置文件（相機內參等）
           const eSensor sensor,          // 傳感器類型
           const bool bUseViewer = true,  // 是否顯示
           const int initFr = 0,          // 初始幀 ID
           const string& strSequence = string());
};
```

### 傳感器類型 (eSensor)

```cpp
enum eSensor {
    MONOCULAR = 0,      // 單目
    STEREO = 1,         // 雙目
    RGBD = 2,           // RGB-D
    IMU_MONOCULAR = 3,  // 單目 + IMU
    IMU_STEREO = 4,     // 雙目 + IMU
    IMU_RGBD = 5        // RGB-D + IMU
};
```

## 三、初始化流程 (12 步驟)

```cpp
// System.cc 構造函數
System::System(...) {
    // 1. 檢查傳感器類型有效性
    if (mSensor == MONOCULAR)
        cout << "Monocular" << endl;
    // ...

    // 2. 讀取配置文件
    cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);

    // 3. 載入 ORB 詞典
    mpVocabulary = new ORBVocabulary();
    mpVocabulary->loadFromTextFile(strVocFile);

    // 4. 創建關鍵幀數據庫
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    // 5. 創建 Atlas（多地圖管理）
    mpAtlas = new Atlas(0);

    // 6. 創建 FrameDrawer 和 MapDrawer
    mpFrameDrawer = new FrameDrawer(mpAtlas);
    mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile);

    // 7. 創建 Tracking（主線程中運行）
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor);

    // 8. 創建 LocalMapping 線程
    mpLocalMapper = new LocalMapping(this, mpAtlas, mSensor==MONOCULAR || mSensor==IMU_MONOCULAR);
    mptLocalMapping = new thread(&LocalMapping::Run, mpLocalMapper);

    // 9. 創建 LoopClosing 線程
    mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary,
                                    mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR);
    mptLoopClosing = new thread(&LoopClosing::Run, mpLoopCloser);

    // 10. 創建 Viewer 線程
    if (bUseViewer) {
        mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
    }

    // 11. 設置線程間指針（互相通訊）
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);
    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);
    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}
```

## 四、線程創建語法解析

```cpp
// 創建新線程執行 LocalMapping::Run
mptLocalMapping = new thread(&LocalMapping::Run, mpLocalMapper);
//                           ↑                   ↑
//                           成員函數指針         對象指針（this）

// 等同於讓 mpLocalMapper 這個對象執行 Run() 函數
// mpLocalMapper->Run(); ← 但這是在新線程中執行
```

**為什麼需要傳對象指針？**
- 成員函數需要 `this` 指針才能存取成員變數
- `&LocalMapping::Run` 只是函數地址，不知道是哪個對象的
- `mpLocalMapper` 告訴線程「執行這個對象的 Run()」

## 五、線程間通訊機制

### 不是 ROS 的 Topic/Service！

ORB-SLAM3 使用**直接指針調用**：

```cpp
class Tracking {
private:
    LocalMapping* mpLocalMapper;  // 指向 LocalMapping 的指針

public:
    void SetLocalMapper(LocalMapping* pLocalMapper) {
        mpLocalMapper = pLocalMapper;
    }

    void CreateNewKeyFrame() {
        KeyFrame* pKF = new KeyFrame(...);
        // 直接調用 LocalMapping 的方法！
        mpLocalMapper->InsertKeyFrame(pKF);
    }
};
```

### Mutex 保護共享數據

```cpp
// LocalMapping.cc
void LocalMapping::InsertKeyFrame(KeyFrame* pKF) {
    unique_lock<mutex> lock(mMutexNewKFs);  // 上鎖
    mlNewKeyFrames.push_back(pKF);          // 安全寫入
}   // 函數結束自動解鎖

bool LocalMapping::CheckNewKeyFrames() {
    unique_lock<mutex> lock(mMutexNewKFs);  // 上鎖
    return !mlNewKeyFrames.empty();         // 安全讀取
}
```

## 六、數據流向

```
┌──────────────────────────────────────────────────────────────────────┐
│                              數據流                                   │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│   Image/IMU ──▶ Tracking ──KeyFrame──▶ LocalMapping ──▶ LoopClosing │
│                    │                        │                │       │
│                    │                        ▼                ▼       │
│                    │                   Local BA         Loop BA      │
│                    │                        │                │       │
│                    ▼                        ▼                ▼       │
│              Current Pose              MapPoints           Merged    │
│                    │                        │              Map       │
│                    └────────────────────────┴────────────────┘       │
│                                             │                        │
│                                             ▼                        │
│                                          Atlas                       │
│                                       (多地圖管理)                    │
└──────────────────────────────────────────────────────────────────────┘
```

## 七、核心類別關係

```
System
  ├── Atlas（多地圖管理）
  │     └── Map（單個地圖）
  │           ├── KeyFrame（關鍵幀）
  │           └── MapPoint（地圖點）
  │
  ├── Tracking（主線程）
  │     └── Frame（當前幀）
  │
  ├── LocalMapping（子線程）
  │     └── 局部 BA 優化
  │
  ├── LoopClosing（子線程）
  │     └── 回環檢測 + 地圖合併
  │
  └── Viewer（可視化線程）
```

## 八、配置文件結構

```yaml
# EuRoC.yaml 範例
%YAML:1.0

# 相機內參
Camera.fx: 458.654
Camera.fy: 457.296
Camera.cx: 367.215
Camera.cy: 248.375

# 畸變參數
Camera.k1: -0.28340811
Camera.k2: 0.07395907
Camera.p1: 0.00019359
Camera.p2: 1.76187114e-05

# ORB 特徵參數
ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8

# IMU 參數（如果有）
IMU.NoiseGyro: 1.7e-4
IMU.NoiseAcc: 2.0e-3
```

## 九、主要頭文件速查

| 頭文件 | 功能 | 重要成員 |
|--------|------|----------|
| `System.h` | 系統入口 | `TrackStereo()`, `TrackRGBD()`, `TrackMonocular()` |
| `Tracking.h` | 跟蹤 | `Track()`, `CreateNewKeyFrame()` |
| `LocalMapping.h` | 局部建圖 | `Run()`, `ProcessNewKeyFrame()` |
| `LoopClosing.h` | 回環 | `Run()`, `DetectLoop()`, `CorrectLoop()` |
| `Atlas.h` | 多地圖 | `GetCurrentMap()`, `CreateNewMap()` |
| `Frame.h` | 幀 | `ExtractORB()`, `ComputeBoW()` |
| `KeyFrame.h` | 關鍵幀 | `GetPose()`, `GetConnectedKeyFrames()` |
| `MapPoint.h` | 地圖點 | `GetWorldPos()`, `GetObservations()` |
