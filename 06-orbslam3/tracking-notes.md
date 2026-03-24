# ORB-SLAM3 Code Review 問題筆記

## System.h 部分

### Atlas 和 ORBVocabulary 的差異
- Atlas 和 ORBVocabulary 這兩個哪裡不一樣？

### settings.h 的用途
- settings.h 那邊是作什麼的？

### class System 的 public 設定
```cpp
0 mono
1 stereo
2 rgbd
3 imu_mono
4 imu_stereo
5 imu_rgbd
```

### 為什麼有兩個 public:？
- 為什麼下面又有一個 `public:`？

### EIGEN_MAKE_ALIGNED_OPERATOR_NEW
- 那個 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 是什麼東西？
- 還有一個可視化線程

### TrackStereo 函數
```cpp
Sophus::SE3f TrackStereo(
    const cv::Mat &imLeft,
    const cv::Mat &imRight,
    const double &timestamp,
    const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(),
    string filename=""
);
```

**問題：**
- 那個 `Sophus::SE3f TrackStereo` 是李群裡面的雙目 tracking 線程？要返回相機位姿？
- `const cv::Mat &imLeft` 這邊是從 cv 裡面取出 mat 的函數，然後對左右的影像做計算？
- 後面有一個 timestamp
- 還有一個 IMU 的點雲的矩陣？那個應該是預積分吧？預積分規定它的矩陣要是 `IMU::Point` 輸出的內容？
- 最後那個 `string filename` 是要幹麻？為什麼填空？
- `&imLeft` 這邊是 imLeft 已經有一個指標然後去取裡面的資料？
- 這邊是有各種上面設定是哪種 sensor 的輸入規定參數？

---

## 線程與數據類型定義

> 先把要用的線程、數據類型、變量要定義出來

**問題：**
- 這邊是說 tracking 裡面有什麼 class 可以調用？還是什麼 void 可以調用？
- 它現在是在說 .h 還是 .cpp？

### 主要功能函數

| 函數 | 說明 |
|------|------|
| `ActivateLocalizationMode` | 激活局部定位 mode |
| `DeactivateLocalizationMode` | 關閉定位模式 |
| `MapChange` | 地圖改變 |
| `Reset` | 重製系統、重製 Atlas？？ |
| `Shutdown` | 所有線程返回 bool？ |
| `SaveTrajectoryTUM` | 存相機旋轉平移的位姿 |
| `SaveKeyFrameTrajectoryTUM` | 關鍵幀的位姿（關鍵幀本質上會有誤差？均方誤差？他在講什麼） |

**問題：**
- 那個 EuRoC 軌跡幹麻還要特別設定存起來？
- 為什麼要保存不同的位姿？全部用同一個 void 存不行？
- 純視覺的可以用那個 `SaveTrajectoryKITTI` 數據集

---

## 變量定義

### eSensor mSensor
```cpp
eSensor mSensor;
```
- 這是什麼東西？應該說這是對感測器定義型別？
- `eSensor` 是哪來的還是什麼？

### KeyFrameDatabase 指標
```cpp
KeyFrameDatabase* mpKeyFrameDatabase;
```
- 這邊是智慧指標？還是什麼？
- 為什麼宣告指標沒有 `new` 之類的？

### Atlas
- Atlas 存的就是關鍵幀跟地圖點，形式是指針？那是什麼東西？

### LocalMapping 指標
```cpp
LocalMapping* mpLocalMapper;
```
- 那個 `LocalMapping* mpLocalMapper;` 是什麼鬼？
- 它說指向線程是什麼意思？

---

## Tracking 描述

> Tracking 的部份是說接收 frame 然後計算 pose，also 也選擇關鍵幀，製造一些新的 mappoints，然後 performs 那個定位如果它 tracking 失敗的話。這個是在對 tracking 的描述？

### LocalMapping 註解
```cpp
LocalMapping* mpLocalMapper;
// 管理 local map 跟使用 BA 調整
```

**問題：**
- 這邊是 System 在控制 thread 流程嗎？就是那個 public 的部份？

---

## 各線程說明

### LoopClosing（閉環）
```cpp
LoopClosing* mpLoopCloser;
```
- 對每個新的關鍵幀進行搜尋
- 如果有環回的話就做位姿圖優化
- 在新的線程做全局的 BA
- 主要功能是建圖

### FrameDrawer / MapDrawer
```cpp
FrameDrawer* mpFrameDrawer;
MapDrawer* mpMapDrawer;
```
- 這邊是不是原本線程以外的功能？就是畫地圖、然後畫關鍵幀？
- 但是這個對建立 3D 地圖好像很重要？

### 線程指標
```cpp
std::thread* mptLocalMapping;
std::thread* mptLoopClosing;
std::thread* mptViewer;
```
- 這邊說那個 **Tracking 才是主線程**，其他都是為這個服務的
- 所以只要 tracking 處理的好，基本上後面吃的效能就會很低？

### Tracking State
- 它那個 tracking state 是要幹麻？

---

## System.cc 部分

### 構造函數
```cpp
System::System(
    const string &strVocFile,
    const string &strSettingsFile,
    const eSensor sensor,
    const bool bUseViewer,
    const int initFr,
    const string &strSequence
):
    mSensor(sensor),
    mpViewer(static_cast<Viewer*>(NULL)),
    mbReset(false),
    mbResetActiveMap(false),
    mbActivateLocalizationMode(false),
    mbDeactivateLocalizationMode(false),
    mbShutDown(false)
{
```

**問題：**
- 這個是在幹麻？剛剛那個 public 可以輸入的變數？
- 初始化 KF 的 id？
- `static_cast<Viewer*>(NULL)` 這是一開始初始化那個 viewer 的指標先指向 null？
- 這邊是先定義初始化的狀態？

---

## 第一步：判斷 Sensor 類型

> 先用邏輯判斷是什麼 sensor 資料進來

### .h vs .cc 的關係
- `.h` 和 `.cc` 又有什麼不同？
- `.cc` 跟 `.cpp` 應該是一樣的吧？
- 重點是他們是什麼關係？

---

## 第二步：讀取配置文件

```cpp
cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
```

**問題：**
- 這邊是對 cv 的函式庫做調用？就是那個 OpenCV？
- 裡面有些可以用而且先寫好的腳本？我可以這樣理解嗎？
- 要用 `FileStorage` 函式庫先調用出來，在 SLAM 這邊給它一個新的函數名字叫做 `fsSettings`？
- 裡面需要可以塞兩個變數分別是 string 跟 cv 裡面的 read 功能？

### cerr
- `cerr` 是什麼鬼？

### FileNode
- `FileNode` 是什麼鬼？
- 那個多地圖管理功能的「加載 Atlas 標識符」這是什麼意思？

```cpp
bool loadedAtlas = false;
```

### mStrLoadAtlasFromFile.empty()
- `mStrLoadAtlasFromFile.empty()` 這邊到底是什麼鬼？
- 應該說這個應該是會回傳是不是空？還是它本來就有 `empty` 這個函數？

---

## 實例化 ORB 辭典對象

```cpp
mpVocabulary = new ORBVocabulary();
```
- 這邊是新增一個指標？？？還是什麼？

### 檢查辭典加載
```cpp
bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
```
- 看一下辭典有沒有返回成功
- 這邊是說那個取址嗎？取 `mpVocabulary` 裡面的某個變數
- 這個變數輸入 `strVocFile` 然後看看有什麼結果？

### 創建關鍵幀數據庫
```cpp
mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
```
- 這個才是正常的指標創建方式對吧？
