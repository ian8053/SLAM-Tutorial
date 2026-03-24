# ORB-SLAM3 Tracking 線程教學筆記 (Part 1)

> 整理自 Bilibili ORB-SLAM3 課程講解 + 個人學習筆記
>
> 本筆記按照**學習順序**組織，從基礎 C++ 語法到系統架構

---

## 目錄

1. [C++ 基礎語法](#一c-基礎語法)
2. [ORB-SLAM3 命名慣例](#二orb-slam3-命名慣例)
3. [系統初始化流程](#三系統初始化-systemcc)
4. [傳感器輸入模塊](#四傳感器輸入模塊)
5. [Tracking 線程流程](#五tracking-線程主要流程)
6. [線程間通信機制](#六線程間通信機制)
7. [關鍵數據結構](#七關鍵數據結構)
8. [IMU 預積分](#八imu-預積分)
9. [Atlas 多地圖系統](#九atlas-多地圖系統)
10. [Sophus SE3 李代數](#十sophus-se3李代數)
11. [實用調試技巧](#十一實用調試技巧)

---

# 一、C++ 基礎語法

> 這些語法在 ORB-SLAM3 程式碼中頻繁出現，必須先理解！

---

## 1.1 `&` 符號：引用 vs 取址

**同一個符號 `&`，位置不同，意思完全不同！**

### 三種用法

```cpp
// 1. & 在型別後面 → 引用（別名）
const string& str;           // str 是某個字串的別名

// 2. & 在變數前面 → 取變數地址
int x = 10;
int* p = &x;                 // &x 取得 x 的位址

// 3. & 在函數前面 → 取函數地址（用於創建線程）
&LocalMapping::Run           // 取得 Run 函數的位址
```

### 拆解 `const string& str`

```cpp
const string& str
  ↑      ↑    ↑
  │      │    └── 變數名
  │      └── & 在型別後面 = 引用（別名）
  └── const = 常數，不能改
```

### 為什麼要用 `const string&`？

想像你有一本很厚的書（= 一個很長的字串）：

| 寫法 | 比喻 | 效果 |
|-----|------|------|
| `string str` | 影印整本書給對方 | 浪費時間和記憶體 |
| `string& str` | 把書借給對方（可以畫） | 快，但對方可以改 |
| `const string& str` | 把書借給對方（只能看） | **又快又安全！** |

### `&` 的位置（空格）不影響意思

```cpp
const string& str    // & 靠近型別
const string &str    // & 靠近變數名
const string & str   // & 前後都有空格
```

這三個**意思完全一樣**，只是寫法風格不同。C++ 不在意空格放哪裡！

---

## 1.2 `::` 符號和 enum

### `::` 是什麼？

`::` = 「裡面的」，用來指定「這個東西屬於哪裡」

```cpp
System::RGBD      // System 裡面的 RGBD
std::string       // std 裡面的 string
cv::Mat           // cv (OpenCV) 裡面的 Mat
```

### enum 是什麼？

**enum = 給數字取名字，讓程式碼更好讀！**

```cpp
// System.h 裡面的定義
class System {
public:
    enum eSensor {
        MONOCULAR = 0,      // 0 叫做 MONOCULAR
        STEREO = 1,         // 1 叫做 STEREO
        RGBD = 2,           // 2 叫做 RGBD
        IMU_MONOCULAR = 3,
        IMU_STEREO = 4,
        IMU_RGBD = 5
    };
};

// 使用時
System SLAM(..., System::RGBD, ...);
//            ↑       ↑
//         class名   裡面的 enum 值
```

### 為什麼用 enum？

```cpp
// 這兩行效果一樣
if (sensor == 2) { ... }            // 看不懂 2 是什麼
if (sensor == System::RGBD) { ... } // 一看就知道是 RGBD
```

**enum 就是讓程式碼更好讀，僅此而已！**

---

## 1.3 `->` 符號：指標呼叫成員

### `->` 是什麼？

```cpp
mpAtlas->CreateNewMap()
   ↑   ↑      ↑
   │   │      └── 呼叫這個函數
   │   └── 用指標存取成員（等於「的」）
   └── 指標變數（指向一個 Atlas 對象）
```

### 遙控器比喻

```
mpAtlas = 遙控器（指標）
Atlas 對象 = 電視機（真正的東西）

mpAtlas->CreateNewMap()
= 用遙控器叫電視機「創建新地圖」
= 電視機執行這個動作

不是遙控器在創建地圖，是電視機在做！
```

### `.` vs `->`

```cpp
// 普通對象用「.」
Atlas atlas;
atlas.CreateNewMap();

// 指標用「->」
Atlas* mpAtlas = new Atlas();
mpAtlas->CreateNewMap();

// 其實 -> 等於 (*指標).
mpAtlas->CreateNewMap()   // 簡寫
(*mpAtlas).CreateNewMap() // 完整寫法（一樣意思）
```

**簡單記：**
```
看到 ->  左邊一定是指標
看到 .   左邊一定是普通對象
```

**指標只是「找到對象的方式」，真正執行函數的是對象本身！**

---

## 1.4 `new` 關鍵字：回傳指標

### `new` 回傳的是指標

```cpp
LocalMapping* mpLocalMapper = new LocalMapping();
      ↑                        ↑
   指標型別                  new 回傳指標
```

`new LocalMapping()` 做兩件事：
1. 在記憶體創建一個 `LocalMapping` 對象
2. **回傳那個對象的位址（指標）**

### 對比兩種創建方式

```cpp
// 方法 A：直接創建對象（棧分配，函數結束就銷毀）
LocalMapping mapper;           // mapper 是對象本身
mapper.mInitFr = 10;           // 用 .

// 方法 B：用 new 創建（堆分配，需手動 delete）
LocalMapping* mpMapper = new LocalMapping();  // mpMapper 是指標
mpMapper->mInitFr = 10;                       // 用 ->
```

### 為什麼用 new？

- 對象需要跨函數存活
- 需要通過指針傳遞給其他線程
- 動態決定創建時機

### 指標指向「整個對象」，`->` 存取裡面的成員

```cpp
class LocalMapping {
public:
    int mInitFr;           // 成員變數 1
    float mThFarPoints;    // 成員變數 2
    bool mbStopped;        // 成員變數 3
};

LocalMapping* mpLocalMapper = new LocalMapping();
```

```
mpLocalMapper（指標）──指向──> LocalMapping 對象
                              ┌─────────────────┐
                              │ mInitFr = ?     │
                              │ mThFarPoints = ?│
                              │ mbStopped = ?   │
                              └─────────────────┘
```

```cpp
mpLocalMapper->mInitFr = 10;        // 修改裡面的 mInitFr
mpLocalMapper->mThFarPoints = 5.0;  // 修改裡面的 mThFarPoints
```

### 比喻

```
mpLocalMapper = 你家地址
LocalMapping 對象 = 你家
mInitFr = 你家的客廳
mThFarPoints = 你家的廚房

mpLocalMapper->mInitFr = 去你家，進客廳
mpLocalMapper->mThFarPoints = 去你家，進廚房

同一個地址（家），不同房間！
```

---

## 1.5 `size_t` 型態

**Q: 不是都用 int、float 嗎？size_t 是什麼？**

```cpp
size_t i = 0;   // 其實就是某種 unsigned int
int i = 0;      // 一般整數（可正可負）
```

| 型態 | 可負數？ | 用途 |
|------|----------|------|
| `int` | ✓ 可以 | 一般數字 (-100, 0, 50) |
| `unsigned int` | ✗ 不行 | 只能 0 或正數 |
| `size_t` | ✗ 不行 | **專門給「大小/索引」用** |

**Q: 為什麼不直接寫 `for(int i = 0; i < v.size(); i++)`？**

可以寫，但會有編譯警告：
```
warning: comparison between signed and unsigned integer expressions
警告：有符號和無符號整數之間的比較
```

原因：
```cpp
int i = 0;       // 有符號（可以是 -1, -2...）
v.size()         // 回傳 size_t（無符號）

i < v.size()     // 拿「可負數」跟「不可負數」比較 → 警告！
```

**結論：**
- 用 `int` 大部分時候能跑，但不嚴謹
- 用 `size_t` 沒警告，更正規
- `size_t` = 「size type」= 專門表示大小/索引的型態

---

## 1.6 函數定義的基本格式

```cpp
回傳型態 函數名(參數) {
    // 做事
    return 回傳值;
}
```

**最簡單的例子：**
```cpp
int add(int a, int b) {
    return a + b;
}
// ↑    ↑
// 回傳型態  函數名
// 中間有空格！
```

**稍微複雜的回傳型態：**
```cpp
cv::Mat getImage() {
    return 某張圖;
}
// ↑        ↑
// 回傳型態   函數名
// 空格分隔！
```

**加上「這是誰的函數」（在 .cc 檔案實作時）：**
```cpp
cv::Mat System::getImage() {
    return 某張圖;
}
// ↑       ↑        ↑
// 回傳型態  類別名::   函數名
```

**ORB-SLAM3 的例子：**
```cpp
Sophus::SE3f System::TrackMonocular(...) {
    return 位姿;
}
```

拆解：
```
Sophus::SE3f    →  回傳型態（Sophus 的 SE3f 位姿）
System::        →  這是 System 類別的
TrackMonocular  →  函數名
```

**為什麼有空格？**
```cpp
int          add()             // int 和 add 中間有空格
Sophus::SE3f TrackMonocular()  // SE3f 和 TrackMonocular 中間有空格

// 這是同一件事！只是回傳型態比較長（Sophus::SE3f），看起來像兩個東西
// 其實就是「回傳型態 + 空格 + 函數名」的固定格式
```

---

## 1.7 `cv::Mat` 和矩陣大小

**Q: `cv::Mat Tcw` 是宣告一個 4x4 矩陣嗎？**

不是！`cv::Mat` 只是型態，還沒指定大小：

```cpp
cv::Mat Tcw;         // 宣告一個矩陣變數，還不知道大小
cv::Mat image;       // 可以叫任何名字
cv::Mat whatever;    // cv::Mat 可以是任何大小
```

**Q: 那為什麼說 Tcw 是 4x4？**

是 `GrabImageStereo()` 這個**函數回傳 4x4**，不是宣告時決定的：

```cpp
cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp, filename);
//                           ↑              ↑       ↑        ↑         ↑
//                        函數名          輸入    輸入      輸入      輸入
//  ↑
// 輸出（回傳值）= 4x4 矩陣
```

**Q: 參數 imLeft, imRight... 決定大小嗎？**

不是！參數是「輸入」，大小是函數內部決定的：

| 參數 | 用途 | 決定輸出大小？ |
|------|------|---------------|
| `imLeft` | 左相機影像 | ✗ |
| `imRight` | 右相機影像 | ✗ |
| `timestamp` | 時間戳 | ✗ |
| `filename` | 檔名 | ✗ |

**類比：烤蛋糕**
```
烤蛋糕(麵粉, 雞蛋, 糖, 牛奶)  →  回傳：蛋糕

蛋糕的形狀是「烤蛋糕」這個函數決定的，不是麵粉決定的！
```

---

## 1.8 Mutex 互斥鎖

### 問題：多線程同時修改資料會出錯

```cpp
// 假設沒有鎖
int count = 0;

// 線程 A                    // 線程 B
count = count + 1;           count = count + 1;
// 讀 count = 0              // 讀 count = 0（同時！）
// 算 0 + 1 = 1              // 算 0 + 1 = 1
// 寫 count = 1              // 寫 count = 1

// 結果：count = 1（應該是 2！）
```

### 解法：互斥鎖（Mutex）

```cpp
mutex mMutex;  // 定義一把鎖

// 線程 A                         // 線程 B
unique_lock<mutex> lock(mMutex);  // 上鎖 ← B 卡在這裡等
count = count + 1;                // ...等待中...
// lock 離開作用域，自動解鎖       // ...等待中...
}                                 // A 解鎖了！
                                  unique_lock<mutex> lock(mMutex); // B 上鎖
                                  count = count + 1;
                                  // 結果：count = 2 ✓
```

### 廁所比喻

```
mutex = 廁所門鎖
unique_lock = 進去鎖門

線程 A：進廁所，鎖門，上廁所
線程 B：想進去，門鎖著，只能等
線程 A：上完了，開門出來
線程 B：現在可以進去了，鎖門，上廁所
```

### 語法拆解

```cpp
unique_lock<mutex> lock(mMutexMode);
     ↑       ↑     ↑       ↑
     │       │     │       └── 要鎖哪把鎖（之前定義好的）
     │       │     └── 這個鎖物件的變數名（隨便取）
     │       └── 鎖的類型（mutex = 互斥鎖）
     └── 鎖的包裝器（unique = 獨佔）
```

### 定義在哪？

```cpp
// 類別裡先定義鎖
class System {
    mutex mMutexMode;   // 定義一把鎖，名字叫 mMutexMode
    mutex mMutexReset;  // 另一把鎖
    mutex mMutexState;  // 又一把鎖
};

// 使用時
unique_lock<mutex> lock(mMutexMode);  // 上鎖
```

### unique 是什麼意思？

```
unique = 唯一、獨佔

unique_lock = 這把鎖只有我一個人能拿
             其他人想拿就要等我放下
             不能複製、不能共享
```

### 為什麼不用引用/指標解決？

```
& 和 * = 存取方式（怎麼找到資料）
mutex = 存取順序（誰先誰後）

兩個是不同層面的問題！
```

引用/指標都可以**同時讀寫**，不會阻止另一個線程：

```cpp
// 沒有鎖的情況
// Tracking（讀）              // LocalMapping（寫）
for (auto& p : map.points)     map.points.push_back(newPoint);
    // 正在遍歷...              // 同時加入新點！
    // 容器大小改變了！
    // 迭代器失效 → 程式崩潰！
```

**引用/指標不能解決「同時存取」的問題，mutex 才能！**

---

## 1.9 線程創建語法

```cpp
// 創建新線程執行 LocalMapping::Run 函數
mptLocalMapping = new thread(&LocalMapping::Run, mpLocalMapper);
//                           ↑                   ↑
//                           函數指針             對象指針（告訴函數是哪個對象的）
```

**為什麼需要兩個參數？**
- `&LocalMapping::Run` - 成員函數需要知道「執行哪個函數」
- `mpLocalMapper` - 也需要知道「是哪個對象的函數」（因為成員函數需要 this 指針）

### `&` 取函數地址

```cpp
new thread(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);
           ↑                               ↑
      函數的地址                         用哪個對象執行
```

`&` 在函數前面 = 取得函數的記憶體地址，讓 thread 知道要執行哪個函數。

---

## 1.10 預設值的寫法

```cpp
const string& strsequence = std::string()
              ↑                   ↑
          這是引用              這是預設值（空字串）
```

- `const string&` → 引用（跟之前一樣）
- `= std::string()` → 預設值是空字串

```cpp
// 呼叫方式 1：有傳參數
System("MH01");    // strsequence = "MH01"

// 呼叫方式 2：沒傳參數（用預設值）
System();          // strsequence = ""（空字串）
```

`std::string()` = 呼叫 string 建構函數，產生空字串 `""`

---

## 1.11 `this` 指標

### 問題：同一個類別可以創建很多對象

```cpp
class Dog {
public:
    string name;

    void introduce() {
        cout << "我是 " << name << endl;
    }
};

// 創建兩隻狗
Dog dog1;
dog1.name = "小白";

Dog dog2;
dog2.name = "小黑";
```

### 問題來了：`introduce()` 怎麼知道是誰在叫它？

```cpp
dog1.introduce();  // 應該印「我是小白」
dog2.introduce();  // 應該印「我是小黑」
```

同一個函數，怎麼知道要用 `dog1` 的 name 還是 `dog2` 的 name？

### 答案：`this` 自動指向「呼叫我的那個對象」

```cpp
class Dog {
public:
    string name;

    void introduce() {
        // this = 誰呼叫我，就指向誰
        cout << "我是 " << this->name << endl;
        //              ↑
        //         其實可以省略
    }
};
```

```cpp
dog1.introduce();
// 此時 this = &dog1（指向 dog1）
// this->name = dog1.name = "小白"

dog2.introduce();
// 此時 this = &dog2（指向 dog2）
// this->name = dog2.name = "小黑"
```

### 比喻

```
你有兩支手機（dog1 和 dog2）
都裝了同一個 App（introduce 函數）

打開 dog1 的 App → App 知道「我在 dog1 裡面」
打開 dog2 的 App → App 知道「我在 dog2 裡面」

this = 「我現在在哪支手機裡」
```

### ORB-SLAM3 的例子

```cpp
class Tracking {
    int count_;

    void setCounter(int count) {
        this->count_ = count;
        //  ↑
        // 「我這個對象」的 count_
    }
};

Tracking tracker1;
Tracking tracker2;

tracker1.setCounter(10);  // this = &tracker1, 設定 tracker1 的 count_ = 10
tracker2.setCounter(20);  // this = &tracker2, 設定 tracker2 的 count_ = 20
```

### 什麼時候要寫 `this->`？

大部分時候可以省略：

```cpp
void setCounter(int count) {
    count_ = count;        // 不寫 this-> 也可以
    this->count_ = count;  // 寫了也一樣
}
```

**只有當參數名和成員變數同名時才需要：**

```cpp
void setCounter(int count_) {  // 參數也叫 count_！
    count_ = count_;           // 錯！左右都是參數
    this->count_ = count_;     // 對！左邊是成員變數，右邊是參數
}
```

**簡單記：`this` = 「我自己」，指向當前正在執行這個函數的對象。**

---

## 1.12 class vs void

```cpp
// class = 定義類別（藍圖），包含數據和函數
class Tracking {
    int count_;          // 數據
    void Run();          // 函數
};

// void = 函數的回傳類型，表示不回傳值
void someFunction() { ... }
```

`new Tracking(...)` 創建對象後，需要呼叫 `Run()` 才會真正開始執行。

---

## 1.13 `cv::FileStorage` 配置文件讀取

```cpp
cv::FileStorage fs("config.yaml", cv::FileStorage::READ);

// 讀取參數
float fx = fs["Camera.fx"];
float fy = fs["Camera.fy"];

// 對應 YAML 文件：
// Camera.fx: 458.654
// Camera.fy: 457.296
```

就是打開 YAML 文件，然後用 `["參數名"]` 讀取數值。不是繼承。

---

# 二、ORB-SLAM3 命名慣例

```
v = vector（陣列）
m = member（成員變數）
p = pointer（指標）
b = bool（布爾）
n = number/count（數量）
```

### 例子

```cpp
vImuMeas   = vector of IMU Measurements（IMU 測量值陣列）
mvKeys     = member vector Keys（成員變數，特徵點陣列）
mpMap      = member pointer Map（成員變數，地圖指標）
mbStopped  = member bool Stopped（成員變數，是否停止）
mnMatches  = member number Matches（成員變數，匹配數量）
```

### vImuMeas 拆解

```
v + Imu + Meas
↑    ↑     ↑
v = vector（陣列）
Imu = IMU
Meas = Measurements（測量值，不是平均 average！）
```

---

# 三、系統初始化 (System.cc)

## 3.1 核心檔案結構

```
ORB-SLAM3/
├── examples/          # main 函數所在（按傳感器分類）
│   ├── Monocular/     # 單目
│   ├── Stereo/        # 雙目
│   ├── RGB-D/         # RGBD
│   └── *-Inertial/    # 與 IMU 結合的版本
├── include/           # .h 頭文件
├── src/               # .cc 源文件
├── Thirdparty/        # 第三方庫
│   ├── DBoW2/         # 詞袋模型（回環檢測、重定位）
│   ├── Sophus/        # 李代數庫（SE3 位姿表示）
│   └── g2o/           # 圖優化（BA、位姿優化）
└── Vocabulary/        # ORB 詞典文件
```

## 3.2 相關頭文件

| 頭文件 | 功能 |
|-------|------|
| `System.h` | 系統入口，初始化所有模塊 |
| `Tracking.h` | 跟蹤線程，位姿估計 |
| `LocalMapping.h` | 局部建圖線程，優化局部地圖 |
| `LoopClosing.h` | 回環檢測 + 地圖合併 |
| `Atlas.h` | 多地圖管理 |
| `Frame.h` | 普通幀數據結構 |
| `KeyFrame.h` | 關鍵幀數據結構 |
| `MapPoint.h` | 地圖點數據結構 |
| `ImuTypes.h` | IMU 數據類型和預積分 |
| `ORBVocabulary.h` | 詞袋模型 |
| `FrameDrawer.h` | 畫面繪製 |
| `Viewer.h` | 可視化界面 |

## 3.3 System 構造函數參數

```cpp
System(const string& strVocFile,      // 詞典路徑
       const string& strSettingsFile, // 配置文件路徑（相機內參等）
       const eSensor sensor,          // 傳感器類型（6種）
       const bool bUseViewer,         // 是否顯示可視化
       const int initFr,              // 初始幀 ID（通常為 0）
       const string& strSequence);    // 序列名
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

**注意：** eSensor 需要手動指定，系統不會自動判斷。在 main 函數中根據你選擇的程式來決定。

## 3.4 ImuTypes.h 內容

```cpp
// IMU 單筆測量數據
class Point {
    Eigen::Vector3f a;   // 加速度
    Eigen::Vector3f w;   // 角速度
    double t;            // 時間戳
};

// 預積分類別
class Preintegrated {
    Eigen::Vector3f dP;  // 累積位移
    Eigen::Vector3f dV;  // 累積速度
    Eigen::Matrix3f dR;  // 累積旋轉

    void IntegrateNewMeasurement(...);  // 把新 IMU 數據積進來
};
```

所以 ImuTypes.h = IMU 數據格式 + 預積分實作

## 3.5 初始化 12 步驟

| 步驟 | 功能 | 關鍵代碼 |
|-----|------|---------|
| 1 | 檢查傳感器類型 | `if(sensor == MONOCULAR)...` |
| 2 | 讀取配置文件 | `cv::FileStorage` |
| 3 | 實例化 ORB 詞典 | `mpVocabulary = new ORBVocabulary()` |
| 4 | 創建關鍵幀數據庫 | `mpKeyFrameDatabase = new KeyFrameDatabase()` |
| 5 | 構建 Atlas 多地圖系統 | `mpAtlas = new Atlas()` |
| 6 | IMU 初始化設置 | `if(mSensor == IMU_*)...` |
| 7 | 創建 FrameDrawer 和 MapDrawer | `new FrameDrawer()`, `new MapDrawer()` |
| 8 | **創建 Tracking 線程** | `mpTracker = new Tracking(...)` |
| 9 | **創建 LocalMapping 線程** | `new thread(&LocalMapping::Run, mpLocalMapper)` |
| 10 | **創建 LoopClosing 線程** | `new thread(&LoopClosing::Run, mpLoopCloser)` |
| 11 | 創建可視化線程 | `new thread(&Viewer::Run, mpViewer)` |
| 12 | 設置線程間指針 | 讓各線程能互相通信 |

### 重要 Q&A

**Q: 初始化時為什麼會有 ORB 辭典？不是開始有畫面才會開始處理 ORB 特徵嗎？**

**A:** 初始化時是「加載」預訓練好的辭典，不是「創建」。ORBvoc.txt 是離線訓練好的視覺詞彙表，系統啟動時先讀進記憶體，之後處理畫面時才用它來匹配特徵。順序：先載入辭典 → 再處理畫面。

**Q: 創建關鍵幀實例畫時是已經在做 tracking 了嗎？**

**A:** 還沒做 tracking！這只是創建一個空的數據結構（書櫃），真正的 tracking 在第 8 步之後才開始。

**Q: 初始化 vs 加載有什麼不一樣？**

**A:**
- **初始化** = `new Tracking(...)` 創建空對象，分配記憶體
- **加載** = `loadFromTextFile(...)` 從檔案讀入數據

例如：先 `new ORBVocabulary()` 創建空辭典，再 `loadFromTextFile()` 把內容讀進去。

---

## 3.6 初始化虛擬碼解說

```cpp
// 1. 檢查傳感器類型
if(mSensor == MONOCULAR)
    cout << "MONOCULAR" << endl;

// 2. 讀取配置文件
cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
// 就是打開 YAML 文件讀取相機參數

// 3. 創建 ORB 詞典
mpVocabulary = new ORBVocabulary();
bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
// 用 mpVocabulary 指針呼叫 loadFromTextFile() 函數
// 把檔案內容讀進這個辭典對象裡

// 4. 創建關鍵幀數據庫
mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
// *mpVocabulary 是解引用（指針指向的對象本身）
// 不是取址！KeyFrameDatabase 建構函數需要對象

// 5. 創建 Atlas
mpAtlas = new Atlas(0);  // 0 表示 initKfId = 0

// 6. 如果有 IMU，設定 IMU 模式
if(mSensor == IMU_STEREO || mSensor == IMU_MONOCULAR)
    mpAtlas->SetInertialSensor();  // 告訴地圖「這次有 IMU 數據」
    mbIsInertial = true;

// 7. 創建繪製器
mpFrameDrawer = new FrameDrawer(mpAtlas);
mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile);

// 8. 創建 Tracking
mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer,
                         mpMapDrawer, mpAtlas, mpKeyFrameDatabase,
                         strSettingsFile, mSensor, strSequence);
// this 是 System 對象的指針，讓 Tracking 能回頭呼叫 System 的函數

// 9. 創建 LocalMapping 線程
mpLocalMapper = new LocalMapping(...);
mptLocalMapping = new thread(&LocalMapping::Run, mpLocalMapper);
// 這樣 LocalMapping::Run() 就會在新線程中執行

// 修改成員變數
mpLocalMapper->mInitFr = initFr;
mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];

// 10. 創建 LoopClosing 線程
mpLoopCloser = new LoopClosing(...);
mptLoopClosing = new thread(&LoopClosing::Run, mpLoopCloser);

// 11. 如果需要可視化
if(bUseViewer) {
    mpViewer = new Viewer(...);
    mptViewer = new thread(&Viewer::Run, mpViewer);
    // Viewer::Run() 是無限迴圈，不斷重繪畫面

    mpTracker->SetViewer(mpViewer);  // 讓 Tracker 知道 Viewer 的指針
    mpLoopCloser->mpViewer = mpViewer;
    mpViewer->both = mpFrameDrawer->both;
    // both 是控制是否同時顯示左右相機圖像的布爾變數
}

// 12. 設置線程間指針
mpTracker->SetLocalMapper(mpLocalMapper);
mpTracker->SetLoopClosing(mpLoopCloser);
mpLocalMapper->SetTracker(mpTracker);
mpLocalMapper->SetLoopCloser(mpLoopCloser);
mpLoopCloser->SetTracker(mpTracker);
mpLoopCloser->SetLocalMapper(mpLocalMapper);
```

### `mpViewer->both` 是什麼？

```cpp
// FrameDrawer.h
bool both;  // 布爾變數

// FrameDrawer.cc
if(both){
    mvCurrentKeysRight = pTracker->mCurrentFrame.mvKeysRight;  // 右相機特徵點
    pTracker->mImRight.copyTo(mImRight);                       // 右相機圖像
    N = mvCurrentKeys.size() + mvCurrentKeysRight.size();       // 左+右的特徵點數量
}
```

白話：
- `both = true` → 雙目模式，同時顯示左右相機
- `both = false` → 只顯示左相機（或單目）

---

# 四、傳感器輸入模塊

## 4.1 三種 Track 函數

system.cc 裡面定義了三種追蹤函數：

```cpp
// 單目（+ IMU）
Sophus::SE3f TrackMonocular(
    const cv::Mat &im,                    // 圖像
    const double &timestamp,              // 時間戳
    const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(),  // IMU 數據（選填）
    string filename = ""                  // 檔名（選填）
);

// 雙目（+ IMU）
Sophus::SE3f TrackStereo(
    const cv::Mat &imLeft,                // 左相機圖像
    const cv::Mat &imRight,               // 右相機圖像
    const double &timestamp,
    const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(),
    string filename = ""
);

// RGB-D（+ IMU）
Sophus::SE3f TrackRGBD(
    const cv::Mat &im,                    // RGB 圖像
    const cv::Mat &depthmap,              // 深度圖
    const double &timestamp,
    const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(),
    string filename = ""
);
```

### 回傳值 `Sophus::SE3f`

- `SE3` = Special Euclidean group 3D（三維剛體變換）
- `f` = float（單精度浮點數）
- 表示 4x4 變換矩陣（旋轉+平移），共 6 自由度
- 這不是定義傳感器，是函數的**回傳類型**（回傳相機位姿）

### 參數說明

| 參數 | 說明 |
|------|------|
| `cv::Mat &im` | 圖像（引用，不複製） |
| `double &timestamp` | 時間戳（引用） |
| `vector<IMU::Point>& vImuMeas` | IMU 測量數據陣列（**還沒預積分**，是原始數據），預設空陣列 |
| `string filename` | 用於保存軌跡時的檔名，預設空字串 |

## 4.2 Track 函數內部流程

```cpp
// 1. 檢查傳感器類型
if(mSensor != STEREO && mSensor != IMU_STEREO) exit(-1);
// exit(-1) = 終止整個程式！傳感器類型不對就結束

// 2. 模式切換（上鎖）
unique_lock<mutex> lock(mMutexMode);

// 3. 如果切換到定位模式
if(mbActivateLocalizationMode) {
    mpLocalMapper->RequestStop();  // 告訴 LocalMapping：「停下來！」

    while(!mpLocalMapper->isStopped()) {
        usleep(1000);  // 等待 1000 微秒 = 0.001 秒（不是 1 秒！）
    }

    mpTracker->InformOnlyTracking(true);  // 告訴 Tracker「只做定位，不建圖」
    mbActivateLocalizationMode = false;   // 重置標誌
}

// 4. 如果恢復正常模式
if(mbDeactivateLocalizationMode) {
    mpTracker->InformOnlyTracking(false);  // 恢復正常
    mpLocalMapper->Release();               // 讓 LocalMapping 繼續運行
    mbDeactivateLocalizationMode = false;
}

// 5. 重置檢查
unique_lock<mutex> lock2(mMutexReset);
if(mbReset) {
    mpTracker->Reset();       // 清空所有地圖、關鍵幀，重新開始
    mbReset = false;
    mbResetActiveMap = false;
} else if(mbResetActiveMap) {
    mpTracker->ResetActiveMap();  // 只重置當前地圖
    mbResetActiveMap = false;
}

// 6. 如果有 IMU，載入 IMU 數據
if(mSensor == IMU_STEREO) {
    for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++) {
        mpTracker->GrabImuData(vImuMeas[i_imu]);
        // 把兩幀之間所有的 IMU 數據都送進 Tracker
    }
}

// 7. 計算位姿
cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp, filename);
// Tcw = Transform from camera to world = 相機位姿（4x4 矩陣）

// 8. 更新狀態
unique_lock<mutex> lock3(mMutexState);
mTrackingState = mpTracker->mState;
mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
```

---

# 五、Tracking 線程主要流程

## 5.1 Track() 函數流程圖

```
┌─────────────────────────────────────┐
│           開始 Track()              │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  1. 判斷 Track() 是否需要等待       │
│  2. 檢查 IMU 數據質量               │
│  3. 檢查輸入數據                    │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  4. 是否已初始化？                  │
│     ├── NO → 初始化（單目/雙目）    │
│     └── YES → 繼續                  │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  5. 位姿估計（三種方式擇一）        │
│     ├── TrackWithMotionModel        │ ← 用上一幀預測
│     ├── TrackReferenceKeyFrame      │ ← 用參考關鍵幀
│     └── Relocalization              │ ← 重定位（詞袋匹配）
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  6. TrackLocalMap                   │ ← 局部地圖跟蹤
│     優化當前幀位姿                  │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  7. 更新 tracking 狀態              │
│    （OK / LOST / RECENTLY_LOST）    │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  8. 更新可視化窗口內容              │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  9. 是否需要插入新關鍵幀？          │
│     條件：                          │
│     1. 幀間 tracking 狀態 OK        │
│     2. 系統需要插入新的關鍵幀       │
│     3. RECENTLY_LOST 必須在 IMU 下  │
│     └── YES → 創建 KeyFrame         │
│              → 送給 LocalMapping    │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  10. 重置系統（如果需要）           │
│      非 IMU 下丟失幀數 >= 5         │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  11. 更新 lastFrame                 │
│      保存當前幀姿態信息             │
└─────────────────────────────────────┘
```

## 5.2 三種跟蹤方式

| 方法 | 使用時機 | 原理 |
|-----|---------|------|
| TrackWithMotionModel | 正常跟蹤 | 假設勻速運動，用上一幀位姿預測當前幀 |
| TrackReferenceKeyFrame | MotionModel 失敗 | 與最近的關鍵幀進行特徵匹配 |
| Relocalization | 完全丟失 | 用詞袋在所有關鍵幀中搜索匹配 |

## 5.3 TrackLocalMap（局部地圖跟蹤）

**這是在 Tracking 線程內完成的，不是 LocalMapping！**

流程：
1. 幀間 Tracking（用上一幀估計初始位姿）
2. **TrackLocalMap**（在附近的地圖點中搜索更多匹配）
   - 從共視關鍵幀找更多地圖點
   - 投影到當前幀搜索匹配
   - 用所有匹配點優化位姿

論文裡有提到：「Track Local Map」章節。

```cpp
bool Tracking::TrackLocalMap() {
    // 1. 更新局部地圖（選擇附近的關鍵幀和地圖點）
    UpdateLocalMap();

    // 2. 在局部地圖中搜索更多匹配點
    SearchLocalPoints();

    // 3. 用所有匹配點優化當前幀位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // 4. 統計內點數量，判斷跟蹤是否成功
    return mnMatchesInliers >= 30;
}
```

## 5.4 Tracking 狀態

```cpp
- OK         → 正常繼續
- RECENTLY_LOST → 短暫丟失，嘗試用 IMU 預測（如果有 IMU）
- LOST       → 完全丟失，嘗試重定位（Relocalization），或創建新地圖（Atlas）
```

## 5.5 初始化相關

### 初始化可能失敗的原因

1. **單目初始化**需要兩幀之間有足夠視差（相機要移動）
   - 對著靜止場景不動 → 初始化失敗
   - 移動太快/太慢 → 初始化失敗
   - 特徵點太少 → 初始化失敗

2. **雙目/RGBD**相對容易，但也可能：
   - 場景太暗/太亮
   - 特徵點不夠
   - 深度值無效

3. 不一定是 TF 問題，TF 錯誤通常是初始化成功後的「定位漂移」

調試方向：
- 確保場景有豐富紋理
- 單目要確保相機有平移移動
- 檢查相機參數（焦距、畸變）

## 5.6 論文 vs 程式碼的差異

**Q: 流程跟論文不太一樣？**

**A:** 論文描述核心演算法，程式碼要處理工程問題。

差異原因：
1. 論文假設系統已初始化，實際程式要處理「還沒初始化」的情況
2. 論文不討論「丟失後怎麼辦」，程式要有 LOST/RECENTLY_LOST 處理
3. 論文不提 IMU，ORB-SLAM3 加了 IMU 相關邏輯
4. 論文不管多地圖，Atlas 是工程擴展

---

# 六、線程間通信機制

## 6.1 不是 ROS2 的 Topic/Service！

ORB-SLAM3 線程通信是**直接傳指針**：

```cpp
// System.cc 中設置線程間關係
mpTracker->SetLocalMapper(mpLocalMapper);
mpTracker->SetLoopClosing(mpLoopCloser);
mpLocalMapper->SetTracker(mpTracker);
mpLocalMapper->SetLoopCloser(mpLoopCloser);
mpLoopCloser->SetTracker(mpTracker);
mpLoopCloser->SetLocalMapper(mpLocalMapper);
```

## 6.2 SetLocalMapper 做的事

```cpp
// System.cc（初始化時）
mpTracker->SetLocalMapper(mpLocalMapper);
// 「嘿 Tracker，這是 LocalMapper 的指標，你存起來」

// Tracking.cc 裡面
void SetLocalMapper(LocalMapping* pLM) {
    mpLocalMapper_ = pLM;  // 存到自己的成員變數
}

// 之後 Tracking.cc 裡面就可以用了
mpLocalMapper_->InsertKeyFrame(pKF);  // 現在知道去哪找了！
```

### 那些函數實際做什麼？

```cpp
// LocalMapping.cc 裡面

void InsertKeyFrame(KeyFrame* pKF) {
    mlNewKeyFrames.push_back(pKF);  // 把關鍵幀加到清單裡
}

void RequestStop() {
    mbStopRequested = true;  // 設一個 flag = true
}

void Release() {
    mbStopped = false;  // 設一個 flag = false
}
```

就是呼叫函數，修改那個對象裡面的變數！

## 6.3 流程圖

```
System.cc 知道 LocalMapper 在哪
Tracking.cc 不知道
        ↓
SetLocalMapper 把指標「複製」給 Tracking
        ↓
現在 Tracking 也知道了，可以自己呼叫
```

### Tracking 傳關鍵幀給 LocalMapping

```
時間 →

Tracking 線程：    [處理幀1] [處理幀2] [處理幀3] [處理幀4] ...
                              ↓ 發現幀2是關鍵幀
                     InsertKeyFrame(pKF)
                              ↓
LocalMapping 線程：          [收到] [優化地圖點] [刪冗餘] ...
```

**因為是不同的 .cc 檔案（不同的類別），變數不共享，要手動傳過去！**

## 6.4 數據流向

```
         ┌────────────────────────────────────┐
         │                                    │
         ▼                                    │
    Tracking ──────────> LocalMapping ────────┼──> LoopClosing
         ▲                    │               │        │
         │                    │               │        │
         └────────────────────┘               │        │
         │                                    │        │
         └────────────────────────────────────┴────────┘
                     (位姿更新通知)
```

每個線程都存著其他線程的指針，需要時直接呼叫對方的函數。
這比 ROS2 pub/sub 快，因為是同一個進程內的直接函數呼叫，不需要序列化/反序列化。

## 6.5 Flag-based 線程通信設計

### 問題：多線程不能直接呼叫對方的函數

```
錯誤做法：
外部線程直接呼叫 mpTracker->Reset();
↓
Tracking 正在跑一半，突然被打斷
↓
程式崩潰！
```

### 解決：用 flag（旗標）

```cpp
// 正確做法：
// 1. 外部線程：設一個標記
mbReset = true;

// 2. Tracking 每次迴圈開頭檢查：
if(mbReset) {
    Reset();           // 自己執行重置
    mbReset = false;   // 清除標記
}
```

### 流程圖

```
外部（用戶按「重置」按鈕）
          ↓
    mbReset = true     ← 只是設一個 flag，很快
          ↓
Tracking 線程（自己的節奏）
          ↓
    發現 mbReset == true
          ↓
    自己執行 Reset()   ← 在安全的時機執行
          ↓
    mbReset = false    ← 清除 flag
```

### 餐廳點餐比喻

```
錯誤：客人直接衝進廚房自己炒菜（危險！）

正確：
1. 客人寫點餐單（設 flag）
2. 廚師空閒時看單子（檢查 flag）
3. 廚師自己做菜（執行操作）
4. 廚師劃掉單子（清除 flag）
```

**這樣設計 = 外部只負責「請求」，Tracking 自己決定「何時執行」，避免衝突！**

---

# 七、關鍵數據結構

## 7.1 Frame（普通幀）

```cpp
class Frame {
    // 時間戳
    double mTimeStamp;

    // ORB 特徵
    vector<cv::KeyPoint> mvKeys;     // 關鍵點
    cv::Mat mDescriptors;            // 描述子

    // 相機位姿 (Tcw = Transform from world to camera)
    Sophus::SE3f mTcw;

    // 地圖點關聯
    vector<MapPoint*> mvpMapPoints;

    // 詞袋向量（用於重定位）
    DBoW2::BowVector mBowVec;
};
```

## 7.2 KeyFrame（關鍵幀）

繼承 Frame，額外包含：
- 與其他關鍵幀的連接關係（Covisibility Graph）
- 生成樹關係（Spanning Tree）
- 回環邊

## 7.3 MapPoint（地圖點）

```cpp
class MapPoint {
    Eigen::Vector3f mWorldPos;           // 3D 世界坐標
    map<KeyFrame*, size_t> mObservations; // 哪些關鍵幀觀測到這個點
    cv::Mat mDescriptor;                  // 代表性描述子
};
```

---

# 八、IMU 預積分

## 8.1 為什麼需要預積分？

IMU 頻率很高（200-1000Hz），相機頻率低（30Hz）：
- 兩幀相機之間有很多 IMU 數據
- 如果每次優化都處理所有 IMU 數據 → 太慢

## 8.2 預積分原理

```
相機幀1 ─────────────────────────── 相機幀2
    │                                   │
    └── IMU1 ── IMU2 ── ... ── IMUn ───┘
              預積分成一個約束
```

把多個 IMU 測量**壓縮**成一個相對運動約束：
- 相對位移 Δp
- 相對速度 Δv
- 相對旋轉 ΔR

## 8.3 相關代碼

```cpp
// ImuTypes.h
class Preintegrated {
    Eigen::Vector3f dP;  // 預積分位移
    Eigen::Vector3f dV;  // 預積分速度
    Eigen::Matrix3f dR;  // 預積分旋轉

    void IntegrateNewMeasurement(const Eigen::Vector3f &acceleration,
                                  const Eigen::Vector3f &angVel,
                                  const float dt);
};
```

---

# 九、Atlas 多地圖系統

## 9.1 為什麼需要 Atlas？

傳統 ORB-SLAM2 問題：
- 對著白牆（無特徵）→ 跟蹤丟失
- 必須手動移動相機找到好的特徵點

Atlas 解決方案：
- 丟失時創建**新地圖**繼續運行
- 找到回環時**合併地圖**

「手動改自動」是指：ORB-SLAM2 跟蹤丟失時需要手動移動相機找特徵點，ORB-SLAM3 的 Atlas 可以自動創建新地圖繼續運行。

## 9.2 結構

```cpp
class Atlas {
    list<Map*> mspMaps;       // 所有地圖
    Map* mpCurrentMap;        // 當前活躍地圖

    Map* GetCurrentMap();
    void CreateNewMap();
    void MergeAtlas(Map* pMergedMap);
};
```

---

# 十、Sophus SE3（李代數）

## 10.1 為什麼用 SE3？

位姿 = 旋轉(R) + 平移(t)

問題：旋轉矩陣有約束（正交、行列式=1），難以優化

解決：用**李代數** se3（6維向量），無約束，方便優化

```cpp
#include <sophus/se3.hpp>

// SE3 表示 4x4 變換矩陣
Sophus::SE3f Tcw;

// 獲取旋轉和平移
Eigen::Matrix3f R = Tcw.rotationMatrix();
Eigen::Vector3f t = Tcw.translation();

// 李代數（6維向量：前3維旋轉，後3維平移）
Eigen::Matrix<float, 6, 1> se3 = Tcw.log();
```

---

# 十一、實用調試技巧

## 11.1 Lost 後持續失敗怎麼辦？

**常見原因和解法：**

| 原因 | 解法 |
|------|------|
| 快速運動 / 運動模糊 | 放慢移動、提高 FPS、縮短快門 |
| 特徵點太少（白牆、地板） | 調高 nFeatures、貼貼紙增加紋理 |
| 光線突變 | 避免對著窗戶/燈光、用自動曝光 |
| 相機標定不準 | 重新標定內參 |
| IMU 沒初始化好 | 開始時靜止 2-3 秒 |

**yaml 可調參數：**
```yaml
ORBextractor.nFeatures: 1500    # 增加特徵數（預設 1000）
ORBextractor.scaleFactor: 1.2   # 金字塔縮放因子
ORBextractor.nLevels: 12        # 金字塔層數（增加=更魯棒）
```

**優先級：**
1. 放慢移動 ← 最有效！
2. 增加 nFeatures
3. 提高 FPS
4. 重新標定

**實戰建議：Lost 時先原地不動或緩慢移動，讓系統重定位成功，再繼續。**

---

# 十二、參考資源

- [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- Bilibili ORB-SLAM3 代碼講解系列
- 《視覺SLAM十四講》
- ORB-SLAM3 論文：IEEE T-RO 2021
