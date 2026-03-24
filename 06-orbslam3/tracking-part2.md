# ORB-SLAM3 Tracking 線程教學筆記 (Part 2)

> 整理自 Bilibili ORB-SLAM3 課程講解 + Claude 問答整理
>
> 本筆記延續 Part 1，深入探討 C++ 語法細節與 ORB-SLAM3 代碼實現

---

## 目錄

1. [Sophus 李群庫詳解](#一sophus-李群庫詳解)
2. [引用傳遞深入理解](#二引用傳遞深入理解)
3. [指標宣告與動態分配](#三指標宣告與動態分配)
4. [智慧指標 vs 普通指標](#四智慧指標-vs-普通指標)
5. [構造函數與初始化列表](#五構造函數與初始化列表)
6. [互斥鎖與多線程同步](#六互斥鎖與多線程同步)
7. [指標操作符詳解](#七指標操作符詳解-new--)
8. [線程間資料共享](#八線程間資料共享)
9. [IMU 預積分代碼解析](#九imu-預積分代碼解析)
10. [OpenCV 圖像處理](#十opencv-圖像處理)
11. [深度圖轉換](#十一深度圖轉換)
12. [Atlas 初始化](#十二atlas-初始化)

---

# 一、Sophus 李群庫詳解

## 1.1 Sophus::SE3f 是什麼？

```cpp
Sophus::SE3f TrackStereo(...);
```

**解析：**

| 組成 | 說明 |
|------|------|
| `Sophus::` | Sophus 函式庫的命名空間 |
| `SE3` | Special Euclidean Group 3D（特殊歐氏群） |
| `f` | float（單精度浮點數） |

### SE3 的數學意義

SE(3) 表示**剛體變換**，包含：
- **旋轉**（Rotation）：3x3 旋轉矩陣
- **平移**（Translation）：3D 向量

```
SE(3) = [ R  t ]    R: 3x3 旋轉矩陣
        [ 0  1 ]    t: 3x1 平移向量
```

### 為什麼用 SE3？

```cpp
// 返回相機位姿（Camera Pose）
Sophus::SE3f Tcw = TrackStereo(...);
// Tcw = Transform from camera to world
// 包含相機的位置和朝向
```

**要點：**
- `Sophus::SE3f` 是調用 Sophus 庫中的 `SE3` 類別
- `f` 指定資料格式為 `float`（也有 `SE3d` 使用 `double`）
- 返回值包含相機的完整位姿資訊

---

# 二、引用傳遞深入理解

## 2.1 const cv::Mat &imLeft 解析

```cpp
Sophus::SE3f TrackStereo(
    const cv::Mat &imLeft,    // 左眼圖像
    const cv::Mat &imRight,   // 右眼圖像
    const double &timestamp,  // 時間戳
    ...
);
```

### 逐一拆解

| 關鍵字 | 含義 |
|--------|------|
| `const` | 函數內部**不能修改**這個參數 |
| `cv::Mat` | OpenCV 的矩陣類型，用於存儲圖像 |
| `&` | **引用傳遞**，不是取址！ |
| `imLeft` | 參數名稱 |

## 2.2 引用傳遞 vs 取址

**關鍵區別：`&` 的位置決定含義！**

```cpp
// 位置 1：在類型後面 → 引用傳遞
void func(const cv::Mat &img);  // img 是傳入變數的「別名」

// 位置 2：在變數前面 → 取址
int x = 10;
int* ptr = &x;  // &x 取得 x 的記憶體地址
```

### 引用傳遞的本質

```cpp
void processImage(const cv::Mat &img) {
    // img 是原始圖像的「別名」
    // 不會複製整張圖像
    // 可以讀取 img，但不能修改（因為 const）
}
```

## 2.3 為什麼用引用傳遞？

### 效率問題

```cpp
// 方式 1：值傳遞（複製整張圖像）
void func1(cv::Mat img);  // ❌ 複製 1920x1080 的圖像很慢！

// 方式 2：引用傳遞（共享記憶體）
void func2(const cv::Mat &img);  // ✅ 只傳遞「別名」，不複製
```

### 是 Shared Memory 嗎？

**是的！** 引用傳遞可以視為一種**共享記憶體**的方式：
- 函數內部和外部都指向**同一塊記憶體區域**
- 節省記憶體空間
- 提高性能

### const 的作用

```cpp
const cv::Mat &img;  // 只能讀取，不能修改
```

- 可以**讀取** `img` 的內容
- **不能修改** `img` 的內容
- 如果嘗試修改，編譯器會**報錯**

**結論：** `const cv::Mat &imLeft` 意思是「傳入左眼圖像的引用，可以讀取但不能修改」

---

# 三、指標宣告與動態分配

## 3.1 指標宣告不需要 new

```cpp
// 在 .h 檔案中：
KeyFrameDatabase* mpKeyFrameDatabase;  // 只是宣告指標
LocalMapping* mpLocalMapper;           // 只是宣告指標
Atlas* mpAtlas;                        // 只是宣告指標
```

**為什麼沒有 `new`？**

| 階段 | 語法 | 說明 |
|------|------|------|
| 宣告 | `Type* ptr;` | 只告訴編譯器「有一個指標」 |
| 初始化 | `ptr = new Type();` | **真正分配記憶體** |

### 完整流程

```cpp
// Step 1：在 .h 中宣告（告訴編譯器有這個指標）
KeyFrameDatabase* mpKeyFrameDatabase;

// Step 2：在 .cc 中初始化（真正分配記憶體）
mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
```

### 常見誤解

```cpp
// ❌ 錯誤理解：宣告時就要 new
KeyFrameDatabase* mpKeyFrameDatabase = new KeyFrameDatabase();  // 不一定！

// ✅ 正確：可以先宣告，之後再初始化
KeyFrameDatabase* mpKeyFrameDatabase;  // 宣告
// ... 在構造函數中 ...
mpKeyFrameDatabase = new KeyFrameDatabase(...);  // 初始化
```

---

# 四、智慧指標 vs 普通指標

## 4.1 普通指標的問題

```cpp
// 普通指標需要手動管理記憶體
KeyFrameDatabase* mpKeyFrameDatabase = new KeyFrameDatabase();

// 必須手動釋放，否則記憶體洩漏！
delete mpKeyFrameDatabase;  // 容易忘記！
```

## 4.2 智慧指標自動管理記憶體

```cpp
// 使用 unique_ptr
std::unique_ptr<KeyFrameDatabase> mpKeyFrameDatabase;
mpKeyFrameDatabase = std::make_unique<KeyFrameDatabase>();

// 離開作用域時自動釋放，不用 delete！
```

### 比較表

| 特性 | 普通指標 | 智慧指標 |
|------|----------|----------|
| 記憶體管理 | 手動 `delete` | 自動釋放 |
| 記憶體洩漏風險 | 高 | 低 |
| 使用複雜度 | 簡單但危險 | 稍複雜但安全 |

## 4.3 智慧指標的兩個步驟

```cpp
// Step 1：宣告智慧指標
std::unique_ptr<KeyFrameDatabase> mpKeyFrameDatabase;

// Step 2：動態分配記憶體
mpKeyFrameDatabase = std::make_unique<KeyFrameDatabase>();

// 兩步都必須做！只宣告不分配會指向空
```

## 4.4 智慧指標的種類

| 類型 | 說明 | 使用場景 |
|------|------|----------|
| `std::unique_ptr` | 獨佔所有權 | 單一擁有者 |
| `std::shared_ptr` | 共享所有權 | 多個擁有者 |
| `std::weak_ptr` | 弱引用 | 避免循環引用 |

```cpp
// unique_ptr：只有一個擁有者
std::unique_ptr<Atlas> atlas = std::make_unique<Atlas>(0);

// shared_ptr：多個擁有者
std::shared_ptr<Map> map = std::make_shared<Map>();
auto map2 = map;  // 現在有兩個指標指向同一個 Map
```

---

# 五、構造函數與初始化列表

## 5.1 什麼是構造函數？

**構造函數（Constructor）= 建構子**

```cpp
class MyClass {
public:
    // 構造函數：名稱與類名相同，沒有返回類型
    MyClass(int value) : myValue(value) {}

private:
    int myValue;
};
```

### 構造函數的特點

1. **名稱與類名相同**
2. **沒有返回類型**（連 void 都沒有）
3. **創建物件時自動調用**

```cpp
MyClass obj(10);  // 創建物件時自動調用 MyClass(10)
```

## 5.2 初始化列表

```cpp
System::System(const string &strVocFile, ...)
    : mSensor(sensor),                      // ← 這是初始化列表
      mpViewer(static_cast<Viewer*>(NULL)),
      mbReset(false),
      mbResetActiveMap(false)
{
    // 構造函數主體
}
```

### 初始化列表語法

```cpp
ClassName::ClassName(params)
    : member1(value1),   // 冒號後面是初始化列表
      member2(value2),
      member3(value3)
{
    // 構造函數主體
}
```

## 5.3 `mSensor(sensor)` 是什麼意思？

```cpp
System::System(const eSensor sensor)
    : mSensor(sensor)  // 用 sensor 參數初始化 mSensor 成員變數
{
}
```

**相當於：**

```cpp
mSensor = sensor;  // 但效率更低！
```

## 5.4 為什麼用初始化列表？

### 效率差異

```cpp
// 方式 1：在函數主體中賦值（效率低）
System::System(const eSensor sensor) {
    mSensor = sensor;  // 先創建 mSensor，再賦值
}

// 方式 2：初始化列表（效率高）
System::System(const eSensor sensor)
    : mSensor(sensor)  // 直接用 sensor 初始化 mSensor
{
}
```

**差異原因：**
- 方式 1：先創建 `mSensor`（調用默認構造函數），再賦值
- 方式 2：直接用 `sensor` 創建 `mSensor`，**省去一次構造**

### 必須使用初始化列表的情況

```cpp
class Example {
    const int constValue;      // const 成員
    int& refValue;             // 引用成員
    const OtherClass obj;      // 沒有默認構造函數的類

    // 這些必須用初始化列表！
    Example(int v, int& r, int o)
        : constValue(v),       // const 必須初始化
          refValue(r),         // 引用必須初始化
          obj(o)               // 沒有默認構造函數必須初始化
    {}
};
```

## 5.5 `static_cast<Viewer*>(NULL)` 是什麼？

```cpp
mpViewer(static_cast<Viewer*>(NULL))
```

- `NULL` 是空指標（值為 0）
- `static_cast<Viewer*>(...)` 把 NULL 轉換成 `Viewer*` 類型
- 整句話：把 `mpViewer` 初始化為空指標

**等同於（C++11 以後）：**

```cpp
mpViewer(nullptr)
```

---

# 六、互斥鎖與多線程同步

## 6.1 mutex 是什麼？

**mutex = mutual exclusion（互斥）**

```cpp
std::mutex mMutexMode;   // 保護「模式」相關的資源
std::mutex mMutexState;  // 保護「狀態」相關的資源
```

### 為什麼需要互斥鎖？

多線程同時訪問同一資源會導致**競爭條件（Race Condition）**：

```cpp
// 危險！兩個線程同時修改 counter
void thread1() { counter++; }
void thread2() { counter++; }
// 結果可能是 1 而不是 2！
```

## 6.2 unique_lock 詳解

```cpp
unique_lock<mutex> lock(mMutexMode);   // lock 是變數名稱
unique_lock<mutex> lock2(mMutexState); // lock2 是另一個變數名稱
```

### unique_lock 的工作原理

```cpp
void func() {
    unique_lock<mutex> lock(mMutexMode);
    // ↑ 創建 lock 時自動「上鎖」

    // 這裡操作受保護的資源
    // 其他線程無法同時進入這段代碼

}  // ← 離開作用域時自動「解鎖」
```

### RAII 設計模式

**RAII = Resource Acquisition Is Initialization**
- 資源獲取即初始化
- 創建物件時獲取資源（上鎖）
- 銷毀物件時釋放資源（解鎖）

## 6.3 不同的 mutex 保護不同的資源

```cpp
std::mutex mMutexMode;   // 保護模式相關（如定位/建圖模式）
std::mutex mMutexState;  // 保護狀態相關（如追蹤狀態）
std::mutex mMutexReset;  // 保護重置相關
```

### lock vs lock2 的區別

```cpp
// lock 和 lock2 只是變數名稱！
// 重要的是它們鎖住不同的 mutex

unique_lock<mutex> lock(mMutexMode);   // 鎖住 mMutexMode
unique_lock<mutex> lock2(mMutexState); // 鎖住 mMutexState
```

**為什麼用不同的鎖？**
- 避免**死鎖**：如果都用同一把鎖，可能互相等待
- 提高**並行性**：操作不相關的資源可以同時進行

## 6.4 mutex 的類型

| 類型 | 說明 | 使用場景 |
|------|------|----------|
| `std::mutex` | 基本互斥鎖 | 一般情況 |
| `std::shared_mutex` | 讀寫鎖 | 讀多寫少 |
| `std::recursive_mutex` | 可重入鎖 | 同一線程多次上鎖 |
| `std::timed_mutex` | 帶超時 | 需要超時機制 |

### shared_mutex 的特點

```cpp
std::shared_mutex mtx;

// 寫操作：獨佔
unique_lock<shared_mutex> writeLock(mtx);
// 此時其他線程不能讀也不能寫

// 讀操作：共享
shared_lock<shared_mutex> readLock(mtx);
// 多個線程可以同時讀，但不能寫
```

**寫入時會阻塞讀取：** 當有線程在寫入時，其他線程的讀取會被暫停，直到寫入完成。

---

# 七、指標操作符詳解 (new, ->, *)

## 7.1 new：動態分配記憶體

```cpp
mpVocabulary = new ORBVocabulary();
```

| 部分 | 說明 |
|------|------|
| `mpVocabulary` | 指標變數 |
| `new` | 動態分配記憶體 |
| `ORBVocabulary()` | 創建物件 |
| `=` | 把物件地址存入指標 |

### 記憶體分配過程

```
1. new ORBVocabulary() 在堆上分配記憶體
2. 創建 ORBVocabulary 物件
3. 返回物件的地址
4. 把地址存入 mpVocabulary
```

## 7.2 ->：透過指標訪問成員

```cpp
bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
```

### -> 的含義

```cpp
// 這兩行是等價的
mpVocabulary->loadFromTextFile(strVocFile);
(*mpVocabulary).loadFromTextFile(strVocFile);
```

| 語法 | 說明 |
|------|------|
| `ptr->member` | 透過指標訪問成員（簡潔） |
| `(*ptr).member` | 先解引用，再訪問成員 |

## 7.3 *：解引用

```cpp
mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
```

### * 在這裡的含義

```cpp
*mpVocabulary  // 解引用：取得 mpVocabulary 指向的「實際物件」
```

| 符號 | 含義 |
|------|------|
| `mpVocabulary` | 指標（存放地址） |
| `*mpVocabulary` | 解引用（取得物件本身） |

### 為什麼要解引用？

```cpp
// KeyFrameDatabase 的構造函數期望接收「物件」而非「指標」
KeyFrameDatabase(const ORBVocabulary& voc);  // 接收引用

// 所以要解引用
new KeyFrameDatabase(*mpVocabulary);  // *mpVocabulary 是物件
```

## 7.4 如何判斷是新增類還是指標？

**看變數宣告！**

```cpp
// 在 .h 中
ORBVocabulary* mpVocabulary;  // * 表示這是指標

// 在 .cc 中
mpVocabulary = new ORBVocabulary();  // new 返回指標
```

### 判斷方法

| 宣告形式 | 類型 |
|----------|------|
| `ClassName* varName;` | 指標 |
| `ClassName varName;` | 物件 |

---

# 八、線程間資料共享

## 8.1 設置線程間引用

```cpp
// Tracking 設置對其他線程的引用
mpTracker->SetLocalMapper(mpLocalMapper);
mpTracker->SetLoopClosing(mpLoopCloser);

// LocalMapping 設置對其他線程的引用
mpLocalMapper->SetTracker(mpTracker);
mpLocalMapper->SetLoopCloser(mpLoopCloser);

// LoopClosing 設置對其他線程的引用
mpLoopCloser->SetTracker(mpTracker);
mpLoopCloser->SetLocalMapper(mpLocalMapper);
```

## 8.2 為什麼需要互相引用？

```
┌─────────────┐
│   Tracking  │←─────────────────────────┐
│  (主線程)   │                          │
└──────┬──────┘                          │
       │ 發送關鍵幀                      │
       ▼                                 │
┌──────────────┐      通知優化結果       │
│ LocalMapping │─────────────────────────┘
│   (子線程)   │
└──────┬───────┘
       │ 發送關鍵幀
       ▼
┌──────────────┐
│ LoopClosing  │
│   (子線程)   │
└──────────────┘
```

### 互相引用的作用

| 線程 | 需要引用 | 原因 |
|------|----------|------|
| Tracking | LocalMapping | 發送關鍵幀給 LocalMapping |
| Tracking | LoopClosing | 接收閉環校正結果 |
| LocalMapping | Tracking | 通知優化完成 |
| LocalMapping | LoopClosing | 發送關鍵幀給 LoopClosing |
| LoopClosing | Tracking | 發送閉環校正結果 |
| LoopClosing | LocalMapping | 請求暫停建圖 |

## 8.3 「線程的對象」是什麼意思？

```cpp
Tracking* mpTracker;     // 指向 Tracking 類實例的指標
LocalMapping* mpLocalMapper;  // 指向 LocalMapping 類實例的指標
```

**解釋：**
- `mpTracker` 是一個**指標**
- 它指向 `Tracking` 類的一個**實例（物件）**
- 這個實例封裝了 Tracking 線程的所有數據和行為

### 為什麼用指標？

```cpp
// 指標允許多個地方引用同一個物件
mpLocalMapper->SetTracker(mpTracker);
// LocalMapping 現在可以通過 mpTracker 調用 Tracking 的方法
```

---

# 九、IMU 預積分代碼解析

## 9.1 從 IMU 佇列取數據

```cpp
if(m->t < mCurrentFrame.mpPrevFrame->mTimeStamp - mImuPer) {
    mlQueueImuData.pop_front();  // 太舊的數據，丟棄
}
else if(m->t < mCurrentFrame.mTimeStamp - mImuPer) {
    mvImuFromLastFrame.push_back(*m);  // 在時間範圍內，保存
    mlQueueImuData.pop_front();         // 從佇列移除
}
else {
    mvImuFromLastFrame.push_back(*m);  // 保存
    break;                              // 超過當前幀時間，停止
}
```

## 9.2 逐行解析

### `m->t` 是什麼？

```cpp
m->t  // m 是指向 IMU 數據的迭代器/指標
      // t 是 IMU 數據的時間戳
      // m->t 取得這筆 IMU 數據的時間
```

### 時間關係圖

```
上一幀時間              當前幀時間
    |                      |
    v                      v
----[======================]-----> 時間軸
     ↑                    ↑
     B - C                B' - C

其中：
A = m->t（IMU 數據的時間戳）
B = mCurrentFrame.mpPrevFrame->mTimeStamp（上一幀時間）
B' = mCurrentFrame.mTimeStamp（當前幀時間）
C = mImuPer（IMU 採樣週期）
```

### 三種情況

| 條件 | 含義 | 操作 |
|------|------|------|
| `A < B - C` | IMU 數據太舊 | 丟棄 (`pop_front`) |
| `B - C ≤ A < B' - C` | 在有效範圍內 | 保存並移除 |
| `A ≥ B' - C` | 超過當前幀 | 保存並停止 |

## 9.3 關鍵變數說明

| 變數 | 類型 | 說明 |
|------|------|------|
| `m` | 迭代器 | 指向當前 IMU 數據 |
| `m->t` | double | IMU 時間戳 |
| `mImuPer` | double | IMU 採樣週期（如 100Hz → 0.01s） |
| `mlQueueImuData` | deque | IMU 原始數據佇列 |
| `mvImuFromLastFrame` | vector | 兩幀之間的 IMU 數據 |

## 9.4 mTimeStamp 存放什麼？

```cpp
mTimeStamp  // 時間戳，通常是 double 類型
            // 表示從某個參考時間點開始經過的秒數
            // 例如：1.234567（秒）
```

---

# 十、OpenCV 圖像處理

## 10.1 cvtColor 顏色轉換

```cpp
if(mImGray.channels() == 3) {
    if(mbRGB) {
        cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
        cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
    } else {
        cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
        cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
    }
}
```

### cvtColor 函數

```cpp
cvtColor(輸入圖像, 輸出圖像, 轉換類型);
```

### 圖像通道數

| 通道數 | 含義 | 例子 |
|--------|------|------|
| 1 | 灰度圖 | 每個像素只有亮度值 |
| 3 | 彩色圖（RGB/BGR） | 每個像素有 R, G, B 三個值 |
| 4 | 帶透明度（RGBA/BGRA） | 每個像素有 R, G, B, A 四個值 |

### 為什麼轉成灰度圖？

1. **ORB 特徵提取只需要亮度資訊**
2. **計算更快**
3. **記憶體更小**

### RGB vs BGR

| 格式 | 順序 | 來源 |
|------|------|------|
| RGB | 紅、綠、藍 | 一般標準 |
| BGR | 藍、綠、紅 | OpenCV 默認（歷史原因） |

## 10.2 Verbose 除錯訊息

```cpp
Verbose::PrintMess("non prev frame", Verbose::VERBOSITY_NORMAL);
```

- `Verbose` 是用於輸出除錯訊息的工具類
- `VERBOSITY_NORMAL` 是訊息等級
- 可以控制輸出多少訊息，避免過多訊息干擾

---

# 十一、深度圖轉換

## 11.1 深度縮放代碼

```cpp
if((fabs(mDepthMapFactor - 1.0f) > 1e-5) && imDepth.type() != CV_32F)
    imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);
```

### 逐部分解析

| 部分 | 說明 |
|------|------|
| `fabs(mDepthMapFactor - 1.0f) > 1e-5` | 檢查深度因子是否 ≠ 1 |
| `imDepth.type() != CV_32F` | 檢查是否不是 32 位浮點數 |
| `imDepth.convertTo(...)` | 轉換格式並縮放 |

### 為什麼需要深度縮放？

有些深度相機輸出的深度值需要縮放：

```
原始深度值: 5000
縮放因子: 0.001
實際深度: 5000 × 0.001 = 5 米
```

### mDepthMapFactor 的含義

```cpp
mDepthMapFactor  // 深度縮放因子
                 // 例如 Kinect v2: 0.001（毫米轉米）
                 // 如果已經是米，則為 1.0
```

---

# 十二、Atlas 初始化

## 12.1 new Atlas(0) 解析

```cpp
mpAtlas = new Atlas(0);
```

### 為什麼參數是 0？

```cpp
Atlas(0)  // 0 是初始地圖的 ID
          // 第一張地圖的編號是 0
```

### 這是指標嗎？

**是的！**

```cpp
Atlas* mpAtlas;           // 宣告：mpAtlas 是指向 Atlas 的指標
mpAtlas = new Atlas(0);   // 初始化：創建 Atlas 物件，把地址存入 mpAtlas
```

## 12.2 namespace 命名空間

```cpp
namespace ORB_SLAM3 {
    class System { ... };
    class Tracking { ... };
    // ...
}
```

### 為什麼用 namespace？

```cpp
// 避免名稱衝突
ORB_SLAM3::System    // ORB-SLAM3 的 System 類
MyProject::System    // 我的專案的 System 類（不衝突）
```

---

# 附錄：常見問題速查

## .h vs .cc/.cpp 的差異

| 檔案類型 | 內容 | 用途 |
|----------|------|------|
| `.h` | 宣告（類、函數原型） | 介面定義 |
| `.cc`/`.cpp` | 實現（函數主體） | 具體程式碼 |

**`.cc` 和 `.cpp` 完全相同**，只是命名慣例不同。

## mStrLoadAtlasFromFile.empty()

```cpp
mStrLoadAtlasFromFile.empty()  // 返回 true 如果字串為空
                                // 這是 std::string 的成員函數
```

## 為什麼保存不同格式的軌跡？

| 格式 | 數據集 | 用途 |
|------|--------|------|
| TUM | TUM RGB-D | 評估 RGB-D SLAM |
| EuRoC | EuRoC MAV | 評估 VI-SLAM |
| KITTI | KITTI | 評估車載 SLAM |

不同的評估工具需要不同的格式，方便與其他算法比較。

## cerr 是什麼？

```cpp
cerr << "Error message" << endl;  // 標準錯誤輸出
cout << "Normal message" << endl; // 標準輸出

// cerr 用於輸出錯誤訊息，可以重定向到錯誤日誌
```

---

# 總結

本筆記涵蓋了 ORB-SLAM3 代碼中常見的 C++ 語法和設計模式：

1. **Sophus 李群庫**：用於表示剛體變換（SE3）
2. **引用傳遞**：高效且安全的參數傳遞方式
3. **指標與動態分配**：`new`、`->`、`*` 的正確使用
4. **智慧指標**：自動記憶體管理
5. **構造函數**：初始化列表的效率優勢
6. **互斥鎖**：多線程同步的關鍵
7. **線程間通信**：透過指標共享資料

掌握這些基礎後，就能更順利地閱讀 ORB-SLAM3 的源代碼！
