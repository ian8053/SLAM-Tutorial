# ORB-SLAM3 Tracking 線程詳解

> 整理自 Bilibili ORB-SLAM3 課程講解 + 個人學習筆記

---

## 一、ORB-SLAM3 系統初始化 (system.cc)

### 核心檔案結構

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

### System 構造函數參數
初始化
system.cc裡面有
tacking.h
localmapping.h
loopclosing.h

framedrawer.h
atlas.h多地圖系統
keyframedatabase.h keyframe數據格式
orbvocabulary.h orb辭典
viewer.h 運行的時候看到的界面
imutypes.h imu數據格式（這是預積分的媽？）

  對！ ImuTypes.h 裡面定義了：                                                                                                                    
                                                                                                                                                  
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


  所以 ImuTypes.h = IMU 數據格式 + 預積分實作

system.h裡面
system(
const string &strvocfile,(這個是詞袋的用處 另外一方面這是對這個指標進行取址嗎？)


不是取址！ 這是「引用」(reference)：

  // 取址：用在變數前面，取得記憶體位址
  string s = "hello";
  string* p = &s;      // p 存的是 s 的位址

  // 引用：用在型別後面，是「別名」
  void func(const string& str) {  // str 是傳進來那個變數的別名
      // str 直接就是原本的字串，不是複製
  }

  簡單記：
  - & 在型別後面 → 引用（reference）
  - & 在變數前面 → 取址（address-of）

### `&` 的位置（空格）不影響意思

```cpp
const string& str    // & 靠近型別
const string &str    // & 靠近變數名
const string & str   // & 前後都有空格
```

這三個**意思完全一樣**，只是寫法風格不同。C++ 不在意空格放哪裡！

### 預設值的寫法

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

## 【重要】ORB-SLAM3 的命名慣例

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

## 【重要】C++ 的 `&` 符號：引用 vs 取址

**同一個符號 `&`，位置不同，意思完全不同！**

### 1. 引用（Reference）：`&` 在型別後面

```cpp
// 宣告時：& 跟著型別 → 這是引用（別名）
string& ref = s;           // ref 是 s 的別名
const string& str = s;     // str 是 s 的唯讀別名
```

### 2. 取址（Address-of）：`&` 在變數前面

```cpp
// 使用時：& 放變數前 → 取得位址
string s = "hello";
string* ptr = &s;          // &s 取得 s 的位址，存到指標 ptr
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

所以 `const string& strVocFile` 意思是：
> 「把詞典路徑**借給我看**就好，不用複製一份給我，而且我保證**不會改它**」

---

## 【重要】C++ 的 `::` 符號和 enum

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

## 【重要】C++ 的 `->` 符號：指標呼叫成員

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

### 程式碼範例

```cpp
Atlas* mpAtlas = new Atlas();   // mpAtlas 指向一個 Atlas 對象
mpAtlas->CreateNewMap();        // 叫那個 Atlas 去做 CreateNewMap()
```

```
mpAtlas（指標）──指向──> Atlas 對象
                           ↓
                    執行 CreateNewMap()
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

**指標只是「找到對象的方式」，真正執行函數的是對象本身！**

---

## 【重要】線程間傳指標：SetLocalMapper 的用途

### 問題：不同檔案的變數不共享

```
System.cc 有 mpLocalMapper 這個變數
Tracking.cc 沒有！

所以要先告訴 Tracking：「LocalMapper 的指標在這」
```

### SetLocalMapper 做的事

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

### 流程圖

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

---

const string &strsettingsfile,(這是各種配置文件的路徑)
const esnsor sensor,(定義一個esensor傳感器的類型 代表六種的傳感器輸入 單目 雙目 rgbd 以及這三種跟imu的結合 所以這邊會自動判定？還是說是用auto然後判斷資料 接著就用判別式判斷 傳感器類型然後繼承多型傳感器回到system?)

**→ 解答：不會自動判斷！是手動指定的。** 在 main 函數裡寫死，例如 `System SLAM(..., System::RGBD, ...)`。然後 System.cc 用 if-else 判斷走哪條路。沒有 auto，沒有多型繼承。
const bool buserviewer = true,(是否使用可視化界面 如果是false的話就是單單輸出一個路徑的結果)
const int initfr =0;(初始畫的真 設定為零)
const string &strsequence = std::string();（再跟跟蹤現成跟局部現成裡面 那個關鍵真看成一個訓練？這是什麼意思？）

**→ 解答：strsequence 是序列名稱**，用來標記這次運行的數據集名稱（例如 "MH01", "V101"）。不是訓練的意思，是 sequence（序列）= 一段連續的數據。
)

system模塊

system主要完成以下照順序工作

1.檢查傳感器初始畫類型（判斷六種）
2.檢查傳感器配置文件 也就是相機的內參 像是xy焦距 光心的平移cyfxfy那些 strsettingsfile函數
3.加仔orb辭典 通過最後一針重定位 如果需要重定位就需要orb辭典 （初始畫為什麼會有orb辭典？這個不是開始有畫面才會開始處裡orb特徵媽？）

**→ 解答：初始化時是「加載」預訓練好的辭典，不是「創建」。** ORBvoc.txt 是離線訓練好的視覺詞彙表，系統啟動時先讀進記憶體，之後處理畫面時才用它來匹配特徵。順序：先載入辭典 → 再處理畫面。
4. 創見關鍵真實例畫 這邊是已經再做tracking了媽？

**→ 解答：還沒做 tracking！** 這只是創建一個空的數據結構（書櫃），真正的 tracking 在第 8 步之後才開始。

5.創見altas多地圖系統 這邊都是叫出多型媽？就是實力畫那些函數到底要怎麼用？初始畫在程式上的意義是什麼？ 手動改成自動式什麼意思

**→ 解答：這不是多型！** `new Atlas(0)` 就是普通的創建對象：
- 分配記憶體
- 呼叫建構函數
- 回傳指針
使用方式：`mpAtlas->CreateNewMap();`

「手動改自動」是指：ORB-SLAM2 跟蹤丟失時需要手動移動相機找特徵點，ORB-SLAM3 的 Atlas 可以自動創建新地圖繼續運行。
6.imu初始畫 如果有imu要對它歸零？
7.利用altas畫關鍵真然後要畫地圖
8.初始畫tacking跟加載有什麼不一樣？
9.初始畫localmapping跟加載有什麼不一樣？
10.初始畫loopclosing跟加載有什麼不一樣？
11.初始畫用戶可是畫線程跟加載有什麼不一樣？

**→ 解答（8-11）：初始化 vs 加載**
- **初始化** = `new Tracking(...)` 創建空對象，分配記憶體
- **加載** = `loadFromTextFile(...)` 從檔案讀入數據
例如：先 `new ORBVocabulary()` 創建空辭典，再 `loadFromTextFile()` 把內容讀進去。

12. 891011把這些實力畫的對象指針傳入需要用到的現成方便數據共享 這邊是在說資料都放到記憶體裡面然後給其他ros2 node做share memory？

**→ 解答：不是 ROS2 的 shared memory！** 這是同一個進程內的指針傳遞：
```cpp
mpTracker->SetLocalMapper(mpLocalMapper);  // 把 LocalMapping 的指針存到 Tracking 裡
mpLocalMapper->InsertKeyFrame(pKF);         // 之後直接呼叫對方的函數
```
比 ROS2 pub/sub 快很多，因為是直接函數呼叫，不需要序列化。



所以上面步驟的虛擬碼是
system(strvocfile,strsettingsfile,sensor,buseviewer,inifr)
 if(msensor==MONOCULAR)
 cout<<”MONOCULAR”<<endl;

cv::filestorage fssettings(strsettingfile.c_str().cv::filestorage::READ);//opencv xml文件讀取（這邊完全看不懂這個是在幹麻 是說有文件路徑？然後繼承讀取的類別？）

**→ 解答：這是 OpenCV 的配置文件讀取器**
```cpp
cv::FileStorage fs(檔案路徑, cv::FileStorage::READ);
float fx = fs["Camera.fx"];  // 讀取相機焦距
```
就是打開 YAML 文件，然後用 `["參數名"]` 讀取數值。不是繼承。
mpvocabulary = new orbvocabulary();（這邊絕對是多行新增一個辭典了八？還是繼承辭典的類型裡面的public？）

**→ 解答：對，是 new 創建新對象。** 不是繼承！就是普通的 `new` 分配記憶體。

bool bvocload = mpvocabulary ->loadfromtextfile(strvocfile);（這邊是把這個新增多型的類別指向剛剛載入的檔案？這樣是什麼意思）

**→ 解答：這是呼叫成員函數。** `mpvocabulary->loadFromTextFile()` = 用 mpvocabulary 指針呼叫 loadFromTextFile() 函數，把檔案內容讀進這個辭典對象裡。

mpkeyframedatabase = new keyframedatabase(*mpvocabulary);(這邊就是取址那個剛剛創見的辭典然後又新增一個多型繼承的關鍵真資料庫？)

**→ 解答：`*mpVocabulary` 是解引用，不是取址！**
- `mpVocabulary` = 指針
- `*mpVocabulary` = 指針指向的對象本身
KeyFrameDatabase 建構函數需要對象，所以用 `*` 解引用。不是多型繼承！

mpatlas = new atlas(0)(0表示initkfid)就是創見一個atlas的繼承多型？

**→ 解答：不是多型！** 就是 `new Atlas(0)` 創建 Atlas 對象，參數 0 是初始關鍵幀 ID。
if(msensor==IMU_STEREO || msensor==IMU_MONOCULAR)
 mpatlas - >setinertialsensor(); 就是說如果imu那邊回傳型別了之後這邊atlas就會指定資料型態？

**→ 解答：這是設定 IMU 模式。** 如果傳感器有 IMU，就呼叫 `setInertialSensor()` 告訴地圖「這次有 IMU 數據」。

Void map::setinertialsensor(){ 所以是從map裡面叫出setinertialsensor的class
 unique_lock<mutex> lock(mMutexmap); 然後創見一個lock的字典？這是幹麻用的？命名為lock

**→ 解答：這是互斥鎖！** 不是字典！
```cpp
unique_lock<mutex> lock(mMutexMap);  // 進入就上鎖
// 這段只有一個線程能執行
// 離開作用域自動解鎖
```
防止多個線程同時修改 mMap 造成數據競爭。像廁所門鎖，一次只能一個人進。

---

## 【重要】Mutex 互斥鎖詳解

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

### 程式碼對照

```cpp
{
    unique_lock<mutex> lock(mMutexMap);  // 進廁所，鎖門

    mMap.update();  // 上廁所（修改資料）

}  // 離開作用域 = 開門出來（自動解鎖）
```

### ORB-SLAM3 為什麼要用？

```
Tracking 線程想讀地圖
LocalMapping 線程想寫地圖
          ↓
    同時存取 = 出錯！
          ↓
    用 mutex 保護，一次只能一個
```

### 會不會慢？為什麼不用引用/指標解決？

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

## 【重要】`new` 回傳指標 + `->` 存取成員

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
// 方法 A：直接創建對象
LocalMapping mapper;           // mapper 是對象本身
mapper.mInitFr = 10;           // 用 .

// 方法 B：用 new 創建（回傳指標）
LocalMapping* mpMapper = new LocalMapping();  // mpMapper 是指標
mpMapper->mInitFr = 10;                       // 用 ->
```

### `.` vs `->` 簡單記

```
看到 ->  左邊一定是指標
看到 .   左邊一定是普通對象
```

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

### `&` 取函數地址（用於創建線程）

```cpp
new thread(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);
           ↑                               ↑
      函數的地址                         用哪個對象執行
```

`&` 在函數前面 = 取得函數的記憶體地址，讓 thread 知道要執行哪個函數。

### `&` 的三種用法總結

```cpp
// 1. & 在型別後面 → 引用
const string& str;

// 2. & 在變數前面 → 取變數地址
int x = 10;
int* p = &x;

// 3. & 在函數前面 → 取函數地址
&LocalMapping::Run
```

---

 mbisinertial = true;(如果有imu就設定維true 什麼視覺imu的函數進行調用？)

**→ 解答：只是設定一個布爾變數。** 後面的代碼會檢查 `mbIsInertial`，如果是 true 就執行視覺+IMU 融合的邏輯。

}
mpframedrawer = new framedrawer(mpatlas);
mpmapdrawer = new mapdrawer(mpatlas,strsettingsfils);這邊就是在用atlas創見drawer 包含frame map的繪製（這邊是在說繪製的class媽？）

**→ 解答：對，FrameDrawer 和 MapDrawer 是繪製類別。** 負責把幀和地圖畫到視窗上。

都準備好之後就是初始畫tracking
mptracker = new tracking(this, mpvocabulary, mpframedrawer, mpmapdrawer, mpatlas, mpkeyframedatabase, strsettingsfile, msensor, strsequence)
就是把剛剛那些創見的參數都丟到tracking這個class就是等於輸入這些參數 那tracking這個class就可以開始啟動？class跟void有什麼不一樣？

**→ 解答：class vs void**
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

 Mplocalmapper = new localmapping(this, mpatlas, msensor==MONOCULAR || msensor==IMU_MONOCULAR , msensor==IMU_MONOCULAR || msensor==IMU_STEREO, strsequence);localmapping裡面要輸入的參數除了要this？this不是一個自己的指標媽？要這個幹麻？然後還要atlas這個地圖管理系統還要看哪一種sensor的型態要輸入近來

**→ 解答：this 是 System 對象的指針。** LocalMapping 需要能回頭呼叫 System 的函數，所以把 System 的指針傳進去。

mplocalmapping = mew thread(&ORB_SLAM3::localmapping::Run,mplocalmapper);然後這邊是從orb裡面的localmapping載入run？跟剛剛心創立的localmapper 這兩個參數輸入到一個新的thread之後就是localmapping的線成了？

**→ 解答：對！這就是創建線程。**
```cpp
new thread(&LocalMapping::Run, mpLocalMapper);
           ↑                    ↑
           執行這個函數          用這個對象（作為 this）
```
這樣 LocalMapping::Run() 就會在新線程中執行。

Mplocalmapper - > minitfr = initfr;
mplocalmapper - >mthfarpoints = fssettings["thfarpoints"]; 然後這邊是把localmapper指標指向的直去做修改？為什麼可以指到很多不同的位址？

**→ 解答：這不是指到不同位址！** `mpLocalMapper` 始終指向同一個 LocalMapping 對象。`->` 是存取該對象的**不同成員變數**：
```cpp
mpLocalMapper->mInitFr = ...;      // 修改成員變數 mInitFr
mpLocalMapper->mThFarPoints = ...; // 修改成員變數 mThFarPoints
```
就像 `dog.name` 和 `dog.age` 是同一隻狗的不同屬性。

Mploopcloser = new loopclosing(mpatlas, mpkeyframedatabase, mpvocabulary, msensor != monocular);//msensor != monocular
mploopclosing = new thread(&ORB_SLAM3::loopclosing::run,mploopcloser);
這邊一樣就是對loopcloser做出使畫判別 然後創見一個thread之後裡面引用orbslam裡面的loopclosing裡面的run參數還有剛剛心創見的mploopcloser？

**→ 解答：對！** 跟 LocalMapping 一樣，創建 LoopClosing 對象，然後開一個新線程執行它的 Run() 函數。

建好之後要可視畫
if(buseviewer){
 mpviewer = new viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile); 這邊是對可是化作一個實例參數輸入？

**→ 解答：對，創建 Viewer 對象。** 把需要的繪製器和設定傳進去。

 mptViewer = new thread(&Viewer::Run, mpViewer);這邊是在對新的thread載入 所以要輸入參數 輸入的參數是viewer裡面的run（這裡面是在幹麻？）還有剛剛實力畫的mpviewer

**→ 解答：Viewer::Run() 是無限迴圈。** 不斷重繪畫面，顯示當前幀和地圖。

 mpTracker - > SetViewer(mpViewer);然後這邊就把實力畫的tracker指到setviewer仔入mpviewer參數的函數的值？這是什麼意思？

**→ 解答：讓 Tracker 知道 Viewer 的指針。** 之後 Tracker 可以通知 Viewer 更新畫面。

 mpLoopCloser - > mpViewer = mpViewer;這邊是不是打錯還是什麼意思？

**→ 解答：沒打錯！** 這是把**右邊的** `mpViewer`（System 的成員）賦值給**左邊的** `mpLoopCloser->mpViewer`（LoopCloser 的成員）。兩個同名但屬於不同對象。

 mpViewer - > both = mpFrameDrawer - > both;這邊是什麼意思both的部份？怎麼突然出現了？

**→ 解答：`both` 是控制是否同時顯示左右相機圖像的布爾變數。**

查看 ORB_SLAM3 原始碼：
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

所以 `mpViewer->both = mpFrameDrawer->both` 是把設定同步到 Viewer，讓兩邊顯示方式一致。

}

//Tracking需要用到mpLocalMapper和閉環mpLoopCloser
mpTracker - > SetLocalMapper(mpLocalMapper);
mpTracker - > SetLoopClosing(mpLoopCloser); 這邊是把tracking指向localmap跟loopclosing輸出？就是說因為tracking需要這些參數來修正？所以這邊就是再做share memory就是說指向他們輸出的數值 然後這邊因為是不是tracker裡面有很多參數可以輸入 這邊那就是把 tracker裡面的參數指到需要用的參數？

**→ 解答：不是 ROS2 shared memory！** 這是同進程內的指針傳遞：
```cpp
class Tracking {
    LocalMapping* mpLocalMapper;  // 存對方的指針

    void SetLocalMapper(LocalMapping* pLM) {
        mpLocalMapper = pLM;  // 保存指針
    }

    void InsertKeyFrame(KeyFrame* pKF) {
        mpLocalMapper->InsertKeyFrame(pKF);  // 直接呼叫對方的函數！
    }
};
```

//LocalMapper 需要用到Tracking和閉環mpLoopCloser
mpLocalMapper - > SetTracker(mpTracker);
mpLocalMapper - > SetLoopCloser(mpLoopCloser);

//mpLoopCloser需要用到Tracking和LocalMapper
mpLoopCloser - > SetTracker(mpTracker);
mpLoopCloser - > SetLocalMapper(mpLocalMapper);

所以這邊就是互相丟資料但是這邊指過去不會很奇怪媽？互相指來指去的 這邊詳細說明一下

**→ 解答：不奇怪！這是常見的設計模式。**

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
```

每個線程都存著其他線程的指針，需要時直接呼叫對方的函數。
這比 ROS2 pub/sub 快，因為是同一個進程內的直接函數呼叫，不需要序列化/反序列化。



傳感器輸入模塊 主要是判斷傳感器
system.cc裡面
單目,單-imu
雙目,雙-imu
rgbd,rgbd-imu

這邊Sophus::SE3f這個是什麼函數（李代數庫？相機的位姿旋轉平移）？裡面是不是就是定義六種傳感器？

**→ 解答：Sophus::SE3f 是李群表示位姿。**
- `SE3` = Special Euclidean group 3D（三維剛體變換）
- `f` = float（單精度浮點數）
- 表示 4x4 變換矩陣（旋轉+平移），共 6 自由度
- 這不是定義傳感器，是函數的**回傳類型**（回傳相機位姿）
單目,單-imu
Sophus::SE3f TrackMonocular(
const cv::Mat &im, 這邊是從cv裡面進入mat函式然後拿出指標im裡面的值？

**→ 解答：`cv::Mat` 是 OpenCV 的圖像/矩陣類別。** `&im` 是引用（不是取值），意思是「把圖像傳進來，不要複製」。

const duble &timestamp, 這邊是宣告timestamp這個指標裡面的值是duble

**→ 解答：這是引用，不是指標！** `const double& timestamp` = 時間戳的引用。

const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), 這邊是宣告一個陣列是imu裡面的point 然後這是取出 vImuMeas裡面的值（這裡面看起來是已經預積分處裡好了imu？） 這句還是不是很懂

**→ 解答：**
```cpp
const vector<IMU::Point>& vImuMeas = vector<IMU::Point>()
      ↑                   ↑           ↑
      IMU數據點的陣列       變數名       預設值（空陣列）
```
- `vector<IMU::Point>` = IMU 測量點的陣列（**還沒預積分**，是原始數據）
- `= vector<IMU::Point>()` = 預設值是空陣列（如果不傳就用空的）
- 白話：「有 IMU 數據就傳進來，沒有就用空的」

string filename=""這邊給它一個空的檔案名稱？不然是什麼意思？

**→ 解答：預設值是空字串。** 用於保存軌跡時的檔名，如果不指定就不保存。

); 
雙目,雙-imu
Sophus::SE3f TrackStereo(
const cv::Mat &imLeft, 這個是仔入mat裡面的左相機？
const cv::Mat &imRight, 這個是仔入mat裡面的右相機？

**→ 解答：對！** `imLeft` 和 `imRight` 是左右相機的圖像。

const double &timestamp,這邊是宣告timestamp這個指標裡面的值是duble
const vector<IMU::Point>& vImuMeas = vector<IMU::Point>();跟上面一樣看不懂
string filename = ""這邊給它一個空的檔案名稱？不然是什麼意思？

**→ 解答：跟上面 TrackMonocular 一樣。** 時間戳 + IMU 數據（選填）+ 檔名（選填）

);
rgbd,rgbd-imu
Sophus::SE3f TrackRGBD(
const cv::Mat &im, image的縮寫？

**→ 解答：對，`im` = image 縮寫。** 這是 RGB 彩色圖像。

const cv::Mat &depthmap,所以這是仔入深度資料？

**→ 解答：對！** depthmap 是深度圖，每個像素存的是該點到相機的距離（通常單位是公尺或毫米）。

const double &timestamp,這邊是宣告timestamp這個指標裡面的值是duble型？

**→ 解答：這是引用，不是指標！** `const double& timestamp` = 時間戳的引用，避免複製 double 值。

const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(),跟上面一樣看不懂

**→ 解答：跟 TrackMonocular 一樣。** IMU 測量數據陣列，預設是空陣列。

string filename = ""
);



傳感器輸入模塊

完成初始畫之後system.cc(.h?)的Sophus::SE3f system::TrackMonocular || TrackStereo || TrackRGBD裡面
1.檢查slam傳感器類型
2.檢查是否打開或關閉定位模式
3.檢查重製狀態 也就是重是track線程或是activate map
4.如果使用imu傳感器 需要加載imu數據
5.track線程計算當前真位姿參數
6.更新狀態參數和數據

//TrackStereo
if(mSensor != STEREO && mSensor != IMU_STEREO) exit(-1); 這邊exit是要去哪裡？

**→ 解答：`exit(-1)` = 終止整個程式！**
- `exit(0)` = 正常結束
- `exit(-1)` 或 `exit(1)` = 錯誤結束
這表示：如果傳感器類型不對，就直接結束程式。

//TrackRGBD
if(mSensor != RGBD)exit(-1);


//TrackMonocular
if(mSensor != MONOCULAR && mSensor != IMU_MONOCULAR)exit(-1);
這邊是什麼意思 是去看輸入的資料是不是指定的格式媽？

**→ 解答：對！這是類型檢查。** 如果你呼叫 `TrackMonocular()`，但系統配置的是 STEREO，就會報錯退出。防止用錯函數。


unique_lock<mutex>lock(mMutexMode);這邊是弄了一個lock?這是幹麻的？就是上面那邊還是沒弄懂

**→ 解答：這是互斥鎖，防止多線程同時修改同一變數。**
```cpp
unique_lock<mutex> lock(mMutexMode);  // 這行執行時「上鎖」
// 這段代碼只有一個線程能進來
// 其他線程會卡在上面那行等待
// 離開作用域 } 時「自動解鎖」
```
就像廁所鎖：進去鎖門，出來才開門，一次只能一人使用。

if(mbActivatelLocalizationMode){
 //如果是定位模式 請求localmapper線程停止建圖
 mpLocalMapper - > RequestStop(); 這是什麼意思mplocalmapper指向stop的內容然後執行媽？

**→ 解答：呼叫 LocalMapping 的 RequestStop() 函數。** `->` 是用指針呼叫成員函數。告訴 LocalMapping 線程：「停下來！」

 //wait until local mapping has effectively stopped
 while(!mpLocalMapper - > isStopped()){
 usleep(1000) 這邊是只要沒有指到停止的指標內容就幹麻？usleep是等待一秒鐘？

**→ 解答：等待直到 LocalMapping 真的停了。**
- `isStopped()` 回傳 true/false
- `usleep(1000)` = 等待 1000 微秒 = **0.001 秒**（不是 1 秒！）
- 持續檢查，直到對方確實停止

}
 //Track線程模式切換為只做tracking
 mpTracker - > informOnlyTracking(true);所以指到只做tracking的指標內容的話

**→ 解答：告訴 Tracker「只做定位，不要建圖」。** 參數 true = 只做 tracking。

 //將默認定位狀態置為false 默認同時做tracking和建圖
 mbActivateLocalizationMode = false;就設定要做tracking？

**→ 解答：重置標誌位。** 這個 flag 是「請求切換到定位模式」的意思，已經執行完切換了，所以設回 false。

}
 //如果關閉定位模式 則打開同時定位和建圖
if(mbDeactivateLocalicationMode)如果這個是true就執行以下事情
 mpTracker - > informOnlyTracking(false); 只做tracking就不要做了

**→ 解答：告訴 Tracker「恢復正常模式，同時定位+建圖」。** 參數 false = 不是只做 tracking。

 mpLocalMapper - > Release();然後localmapper就指向釋放的內容然後釋放記憶體？

**→ 解答：Release() 不是釋放記憶體！** 是「釋放」LocalMapping 線程，讓它繼續運行（之前被 RequestStop 暫停了）。

 mbDeactivateLocalizationMode = false; 所以就設定定位模式維false？

**→ 解答：對，重置標誌位。** 已經處理完「關閉定位模式」的請求了。

}

unique_lock<mutex>lock(mMutexReset);
if(mbReset){如果要重製
 mpTracker - >Reset(); 就指到重置的指標內容？？

**→ 解答：呼叫 Tracker 的 Reset() 函數。** 這會清空所有地圖、關鍵幀，重新開始。用於系統完全迷路時。

 mbReset = false;
 mbResetActiveMap = false;
}else if(mbResetActiveMap){
 mpTracker - > ResetActiveMap();什麼意思？這邊都是指到指標內容去執行這個記憶體指標的內容？

**→ 解答：只重置「當前地圖」，不是全部重置。**
- `Reset()` = 清空所有東西
- `ResetActiveMap()` = 只清空當前活躍的地圖（Atlas 中可能有多張地圖）

是的，`->` 都是用指針呼叫對方的成員函數。

 mbResetActiveMap = false;
}

用atlas重定位功能 這段是在幹麻？解釋一下我是說架構系統設計的部份

**→ 解答：這段是「模式切換」的狀態機設計。**

整體架構：
```
外部請求（用戶按鍵/API呼叫）
     ↓
設定 flag（mbReset, mbActivateLocalizationMode 等）
     ↓
TrackXXX() 開頭檢查這些 flag
     ↓
執行對應操作（重置/切換模式）
     ↓
清除 flag
```

為什麼這樣設計？因為多線程：
- 外部線程設定 flag = 「請求」
- Tracking 線程檢查 flag = 「執行」
- 這樣不會有線程衝突


//TrackStereo
if(mSensor == system:IMU_STEREO)
 for(size_t i_imu = 0;i_imu < vImuMeas.size();i_ime++)  // size_t i_imu這是兩個變數？還是什麼型態？size_t型態的i_imu變數？

**→ 解答：`size_t i_imu` 是一個變數宣告。**
- `size_t` = 型態（無符號整數，用於大小/索引）
- `i_imu` = 變數名
就是「宣告一個 size_t 類型的變數叫 i_imu」。

  mpTracker - > GrabImuData(vImeMeas[i_imu]); GrabImuData 這次什麼東西

**→ 解答：把 IMU 數據餵給 Tracker。**
- `GrabImuData()` = 「抓取」一筆 IMU 測量
- `vImuMeas[i_imu]` = 第 i_imu 筆 IMU 數據
- 這個 for 迴圈會把兩幀之間所有的 IMU 數據都送進 Tracker

//TrackMonocular
if(mSensor == system::IMU_MONOCULAR)
 for(size_t i_imu = 0; i_imu < vImuMeas.size();i_imu++)
  mpTracker - > GrablmuData(vImuMeas[i_imu]);

**→ 解答：跟上面一樣，只是這是單目+IMU 的版本。**





下邊就是在計算當前位姿？所以那個 Grablmage就是已經是寫好的封裝 只是等待是stereo＼grbd＼monocular來做繼承並且多型？

**→ 解答：不是多型/繼承！** 這只是三個不同的函數：
- `GrabImageStereo()` - 處理雙目圖像
- `GrabImageRGBD()` - 處理 RGB-D 圖像
- `GrabImageMonocular()` - 處理單目圖像
根據你在 main 中選的傳感器類型，呼叫對應的函數。

//TrackStereo
cv::Mat Tcw = mpTracker - > GrablmageStereo(imLeft, imRight, timestamp, filename);

tcw transforme 透過 GrablmageStereo獲取相機位姿 所以這邊tcw是一個新的函數媽？就是作用是取得相機位姿？

**→ 解答：`Tcw` 是變數，不是函數！**
- `Tcw` = Transform from camera to world = 相機位姿
- `cv::Mat Tcw` = 宣告一個 4x4 矩陣變數
- `= mpTracker->GrabImageStereo(...)` = 呼叫函數，回傳值存到 Tcw

所以這行意思是：「呼叫 GrabImageStereo 處理圖像，把算出的相機位姿存到 Tcw 裡」

//TrackRGBD
cv::Mat Tcw = mpTracker - > GrablmageRGBD(im, depthmap, timestamp, filename);


//TrasckMonocular
cv::Mat Tcw = mpTracker - > GrablmageMonocular(im, timestamp, filename);

GrablmageStereo
GrablmageRGBD
GrablmageMonocular 位於tracking.cc中



unique_lock<mutex>lock2(mMutexState);
mTrackingState = mpTracker - > mState;
mTrackedMapPoints = mpTracker - > mCurrentFrame.mvpMapPpints;
mTrackedKeyPointsUn = mpTracker - > mCurrentFrame.mvKevsUn;

把tracking輸出做一個更新 所以世紀算完的指標內容重新寫到原本的變數裡面？

**→ 解答：對！這是從 Tracker 讀取結果，更新 System 的成員變數。**
```cpp
mTrackingState = mpTracker->mState;              // 狀態（OK/LOST 等）
mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;    // 追蹤到的地圖點
mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;      // 特徵點（去畸變）
```
這樣外部呼叫者可以透過 System 查詢這些結果。




tracking線程子模塊

system.cc裡面的Sophus::SE3f System::TrackMonocular/TrackStereo/TrackRGBD

SE3f System這個為什麼可以有空格

**→ 解答：這是「回傳型態 + 函數名」的語法。**
```cpp
Sophus::SE3f System::TrackMonocular(...)
↑            ↑       ↑
回傳型態      類別名   函數名
```
意思是：System 類別的 TrackMonocular 函數，回傳 SE3f 型態的位姿。


1.跟數輸入的數據生成一個frame
2.根據frame的信息調用tacke（）函數的GrabImageMonocular/Stereo/RGBD進行位姿估計

track()
1.判斷track（）是否需要等待
2.檢查imu數據質量
3.檢查輸入數據
4.進行真間tracking
5.是否已經初始畫
 (a)yes
  6.不同模式幀間tracking（定位模式或是定位加建圖 調用trackingwithmotionmodel或是trackreferencekeyframe實現）
  7.局部地圖tracking 這邊局部地圖tracking是什麼意思？理論中好像沒有提到？事先直接送給localmap做追蹤特徵點媽？還是怎樣

**→ 解答：局部地圖 Tracking 是在 Tracking 線程內完成的，不是 LocalMapping！**

流程：
1. 幀間 Tracking（用上一幀估計初始位姿）
2. **TrackLocalMap**（在附近的地圖點中搜索更多匹配）
   - 從共視關鍵幀找更多地圖點
   - 投影到當前幀搜索匹配
   - 用所有匹配點優化位姿

論文裡有提到：「Track Local Map」章節。

  8.更新tracking狀態 （ok＼lost＼recenty_lost)這邊如果失敗會怎麼做？

**→ 解答：**
- `OK` → 正常繼續
- `RECENTLY_LOST` → 短暫丟失，嘗試用 IMU 預測（如果有 IMU）
- `LOST` → 完全丟失，嘗試重定位（Relocalization），或創建新地圖（Atlas）
  9.更新可視化窗口內容 更新可是畫窗口當前幀的姿態參數
  10.插入新的關鍵幀 （必須同時滿足1.幀間tracking狀態ok 2.系統需要插入新的關鍵幀3.mstate==RECENTLY_LOST必須是在imu傳感器環境下）
  11.重製系統 如果在非imu傳感器環境下丟失真數大於等無五
  12.更新lastframe
 這邊跟理論好像有點不一樣的流程 為什麼有這種差異？是因為遇到什麼問題所以要這樣設計？

**→ 解答：論文描述核心演算法，程式碼要處理工程問題。**

差異原因：
1. 論文假設系統已初始化，實際程式要處理「還沒初始化」的情況
2. 論文不討論「丟失後怎麼辦」，程式要有 LOST/RECENTLY_LOST 處理
3. 論文不提 IMU，ORB-SLAM3 加了 IMU 相關邏輯
4. 論文不管多地圖，Atlas 是工程擴展

 (b)no
 1.進行初始化（mSensor == STEREO||RGBD||IMU_STEREO）分成1雙目初始化2單目初始化
 2.判斷初始畫狀態成功 分成1更新mnFirstFrameid 2 退出track（）

這邊使為什麼要判斷有沒有初始畫？因為這是一個很大的系統媽？既然沒有初始畫成功藥劑入初始畫之後還會初始畫狀態失敗？這邊是在說還沒辦法在畫面中抓到orb特徵點媽？也就是說這邊要回去調整相機速度？或是tf數有沒有轉換正確？

**→ 解答：初始化可能失敗的原因：**

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

最後保存當前幀姿態信息





                                                                                                                                                                    


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

### 初始化 12 步驟

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

---

## 二、C++ 語法解答

### Q1: `const string& strVocFile` 是什麼意思？

```cpp
const string& strVocFile
      ↑       ↑
      │       └── 引用（不複製，直接用原本的）
      └── 常量（不能修改）
```

**對比：**
- `string strVocFile` - 複製一份（慢，佔記憶體）
- `string& strVocFile` - 引用原本的（快）
- `const string& strVocFile` - 引用但不能改（安全又快）✓ 最常用

### Q2: `cv::FileStorage` 是什麼？

OpenCV 提供的配置文件讀取工具，支援 YAML 和 XML 格式：

```cpp
cv::FileStorage fs("config.yaml", cv::FileStorage::READ);

// 讀取參數
float fx = fs["Camera.fx"];
float fy = fs["Camera.fy"];

// 對應 YAML 文件：
// Camera.fx: 458.654
// Camera.fy: 457.296
```

### Q3: `new` 關鍵字

```cpp
// 堆分配（需手動管理生命週期，用完要 delete）
Tracking* mpTracker = new Tracking(...);

// 棧分配（自動管理，函數結束就銷毀）
Tracking tracker(...);
```

**為什麼用 new？**
- 對象需要跨函數存活
- 需要通過指針傳遞給其他線程
- 動態決定創建時機

### Q4: 成員函數 vs 普通函數

```cpp
// 成員函數：屬於某個 class，可以存取成員變數
class Tracking {
public:
    void Run();  // 成員函數
private:
    int count_;  // 成員變數
};

// 普通函數：獨立存在
void someFunction();
```

### Q5: `this` 指標

```cpp
class Tracking {
public:
    void setCounter(int count) {
        this->count_ = count;  // this 指向當前對象
        // 等同於：count_ = count;
    }
private:
    int count_;
};
```

### Q6: 線程創建語法

```cpp
// 創建新線程執行 LocalMapping::Run 函數
mptLocalMapping = new thread(&LocalMapping::Run, mpLocalMapper);
//                           ↑                   ↑
//                           函數指針             對象指針（告訴函數是哪個對象的）
```

**為什麼需要兩個參數？**
- `&LocalMapping::Run` - 成員函數需要知道「執行哪個函數」
- `mpLocalMapper` - 也需要知道「是哪個對象的函數」（因為成員函數需要 this 指針）

### Q7: Mutex 互斥鎖

```cpp
std::mutex mMutexMap;  // 定義一個鎖

void updateMap() {
    unique_lock<mutex> lock(mMutexMap);  // 上鎖
    // 只有一個線程能執行這段代碼
    mMap.update();
    // 函數結束自動解鎖
}
```

**為什麼需要？** 多線程同時修改同一數據會出錯：
- Tracking 在寫入 KeyFrame
- LocalMapping 也在讀取 KeyFrame
- → 數據競爭，程式崩潰

---

## 三、線程間通信機制

### 不是 ROS2 的 Topic/Service！

ORB-SLAM3 線程通信是**直接傳指針**：

```cpp
// System.cc 中設置線程間關係
mpTracker->SetLocalMapper(mpLocalMapper);
mpTracker->SetLoopClosing(mpLoopCloser);
mpLocalMapper->SetTracker(mpTracker);
mpLocalMapper->SetLoopCloser(mpLoopCloser);
```

**原理：**
```cpp
class Tracking {
private:
    LocalMapping* mpLocalMapper;  // 儲存指向 LocalMapping 的指針

public:
    void SetLocalMapper(LocalMapping* pLocalMapper) {
        mpLocalMapper = pLocalMapper;  // 保存指針
    }

    void InsertKeyFrame(KeyFrame* pKF) {
        mpLocalMapper->InsertKeyFrame(pKF);  // 直接調用！
    }
};
```

### 數據流向

```
Tracking ──KeyFrame──> LocalMapping ──KeyFrame──> LoopClosing
    ↑                       │                         │
    └───────────────────────┴─────────────────────────┘
                     (位姿更新通知)
```

---

## 四、IMU 預積分 (IMU Pre-integration)

### 為什麼需要預積分？

IMU 頻率很高（200-1000Hz），相機頻率低（30Hz）：
- 兩幀相機之間有很多 IMU 數據
- 如果每次優化都處理所有 IMU 數據 → 太慢

### 預積分原理

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

### 相關代碼

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

## 五、Tracking 線程主要流程

### Track() 函數流程

```
┌─────────────────────────────────────┐
│           開始 Track()              │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  1. 檢查 IMU 數據是否正常           │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  2. 是否已初始化？                  │
│     ├── NO → 初始化（單目/雙目）    │
│     └── YES → 繼續                  │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  3. 位姿估計（三種方式擇一）        │
│     ├── TrackWithMotionModel        │ ← 用上一幀預測
│     ├── TrackReferenceKeyFrame      │ ← 用參考關鍵幀
│     └── Relocalization              │ ← 重定位（詞袋匹配）
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  4. TrackLocalMap                   │ ← 局部地圖跟蹤
│     優化當前幀位姿                  │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  5. 是否需要插入新關鍵幀？          │
│     └── YES → 創建 KeyFrame         │
│              → 送給 LocalMapping    │
└──────────────┬──────────────────────┘
               ▼
┌─────────────────────────────────────┐
│  6. 更新狀態，準備下一幀            │
└─────────────────────────────────────┘
```

### 三種跟蹤方式

| 方法 | 使用時機 | 原理 |
|-----|---------|------|
| TrackWithMotionModel | 正常跟蹤 | 假設勻速運動，用上一幀位姿預測當前幀 |
| TrackReferenceKeyFrame | MotionModel 失敗 | 與最近的關鍵幀進行特徵匹配 |
| Relocalization | 完全丟失 | 用詞袋在所有關鍵幀中搜索匹配 |

### TrackLocalMap（局部地圖跟蹤）

這是論文中 "Track Local Map" 的實現：

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

---

## 六、關鍵數據結構

### Frame（普通幀）

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

### KeyFrame（關鍵幀）

繼承 Frame，額外包含：
- 與其他關鍵幀的連接關係（Covisibility Graph）
- 生成樹關係（Spanning Tree）
- 回環邊

### MapPoint（地圖點）

```cpp
class MapPoint {
    Eigen::Vector3f mWorldPos;           // 3D 世界坐標
    map<KeyFrame*, size_t> mObservations; // 哪些關鍵幀觀測到這個點
    cv::Mat mDescriptor;                  // 代表性描述子
};
```

---

## 七、Atlas 多地圖系統

### 為什麼需要 Atlas？

傳統 ORB-SLAM2 問題：
- 對著白牆（無特徵）→ 跟蹤丟失
- 必須手動移動相機找到好的特徵點

Atlas 解決方案：
- 丟失時創建**新地圖**繼續運行
- 找到回環時**合併地圖**

### 結構

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

## 八、Sophus SE3（李代數）

### 為什麼用 SE3？

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

## 九、重要頭文件總結

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

---

## 十、參考資源

- [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- Bilibili ORB-SLAM3 代碼講解系列
- 《視覺SLAM十四講》
- ORB-SLAM3 論文：IEEE T-RO 2021
