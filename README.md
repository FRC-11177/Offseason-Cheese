# 季後賽機器人 - 起司

    這是(會動的)機器人的程式碼

# 目錄

- 機構資訊
    - [底盤](#底盤)
    - [Intake](#intake)
    - [Indexer](#indexer)
    - [Shooter](#shooter)

- 協作規範
    - [GitHub篇](#github-篇)
    - [NetworkTables篇](#networktables-篇)
    - [子系統篇](#子系統篇)
 
- 附錄
    - [依賴函式庫](#依賴函式庫)

# 機構資訊

## 底盤

### 基礎資訊


- 設計大小： 00x00x00 cm/ 00x00x00 in.
- 周長： 00 cm / 00 in.
- 總重： 00 kg / 00 lbs.
- 齒輪比： SDS MK5n R2

| 馬達名稱    |     Tuner 名    | CAN ID | PDH 接的洞 |
| :-------    |  :-------------- | :-------:| :--:|
| 左前動力馬達 | FrontLeftDrive | 11 | -1 | 
| 左前轉向馬達 | FrontLeftSteer | 12| -1 | 
| 右前動力馬達 | FrontRightDrive | 21| -1 | 
| 右前轉向馬達 | FrontRightDrive | 22 | -1 | 
| 左後動力馬達 | BackLeftDrive | 31 | -1 | 
| 左後轉向馬達 | BackLeftSteer | 32 | -1 | 
| 右後動力馬達 | BackRightDrive | 41 | -1 | 
| 右後轉向馬達 | BackRightSteer | 42 | -1 | 

| 零件名稱    |     Tuner 名      | CAN ID | 氣動供電接的洞 |
| :-------   |  :-------------- | :-------:| :--:|
| 左前編碼器   | FrontLeftEncoder | 10 | -1 |
| 右前編碼器   | FrontLeftEncoder | 10 | -1 |
| 左後編碼器   | FrontLeftEncoder | 10 | -1 |
| 右後編碼器   | FrontLeftEncoder | 10 | -1 |

| 馬達名稱 / 類別 | 控制方法 |
| --- | ---|
| 動力馬達 | MotionMagicVelocityVoltage | 
| 轉向馬達 | MotionMagicTorqueCurrentFOC |

| 模塊名稱 | x 方向 | y 方向 |
| --- | :--: | :--:|
| 模心距 | 552.5mm | 552.5mm |
| 左前 | - | + |
| 右前 | + | + |
| 左後 | - | - |
| 右後 | + | - |

| 編碼器偏移量 | 數值(已反轉) |
| -- | --|
| 左前 | 0 rot |
| 右前 | 0 rot |
| 左後 | 0 rot |
| 右後 | 0 rot |


## Intake

- 皮帶比： 1:1

| 馬達名稱    |     Tuner 名    | CAN ID | PDH 接的洞 |
| :-------    |  :-------------- | :-------:| :--:|
| 抬升馬達 | (NEO Vortex) | 51 | -1 | 
| 左轉輪馬達 | LeftRollMotor | 52| -1 | 
| 右轉輪馬達 | RightRollMotor | 53| -1 | 


| 馬達名稱 / 類別 | 控制方法 |
| --- | ---|
| 抬升馬達 | MAXMotionPosition |
| 左轉輪馬達 | Follow(RightRollMotor) |
| 右轉輪馬達 | MotionMagicVelocityVoltage |

## Indexer

- 自動化跟Shooter綁在一起；不對外提供控制API
- Index 馬達 0.6倍跟隨，傳輸馬達0.7倍跟隨

| 馬達名稱    |     Tuner 名    | CAN ID | PDH 接的洞 |
| :-------    |  :-------------- | :-------:| :--:|
| Index馬達 | IndexMotor | 54 | -1 | 
| 左傳輸馬達 | LeftPassMotor | 55 | -1 |
| 右傳輸馬達 | RIghtPassMotor | 56 | -1 |


| 馬達名稱 / 類別 | 控制方法 |
| --- | ---|
| Index馬達 | MotionMagicVelocityVoltage |
| 左傳輸馬達 | Follow(IndexMotor) |
| 右傳輸馬達 | Follow(LeftPassMotor) |

## Shooter

| 馬達名稱    |     Tuner 名    | CAN ID | PDH 接的洞 |
| :-------    |  :-------------- | :-------:| :--:|
| 左射擊馬達 | LeftShootMotor | 57 | -1 | 
| 右射擊馬達 | RightShootMotor | 58 | -1 |


| 馬達名稱 / 類別 | 控制方法 |
| --- | ---|
| 左射擊馬達 | Follow(RightShootMotor) |
| 右射擊馬達 | MotionMagicVelocityVoltage |

# 協作規範

## GitHub 篇

- 每一個機構請獨立一個branch，寫完之後申請PR等大部分人看過之後再一起討論要怎麼merge。
- 請先確認程式沒有問題(模擬器不會閃退，機器程式傳上去之後不會一直重啟)之後再把程式上傳上GitHub，不要沒事添別人的麻煩。
- 請先pull main branch的內容再進行更改，不然PR沒辦法merge自行負責。
- 所有branch都要從main裡面開，如果是用CLI開branch的請注意這一點。
- 記得常常看電子郵件，如果有人PR發了沒發現到時候不要哭夭
- 看過別人的PR如果沒有留留言的話記得在討論串裡面講你已經看過了。

## NetworkTables 篇

- 請全部用`DogLog`推資料
- 資料格式請遵循 `{子系統}/{資料名稱}` (eg. `Drivetrain/CurrentSpeeds`) 
- 請確保你的資料結構是乾淨的，不要污染別人的眼睛。

## 子系統篇

- 請遵循以下檔案結構，並且將所有指令都集中在檔案內部，所有資料（除了共用資料）請只生活在自己的資料夾

    ```text
    .
    ├── RobotContainer.java
    ├── Main.java
    └── subsystems/
        └── ExampleSubsystem/
            ├── ...
            └── ExampleSubsystem.java <- 這個名字要跟資料夾的名稱一樣
    ```

- 在`ExampleSubsystem.java`中，請遵守以下結構

    ```java

    public class ExampleSubsystem implements Subsystem{
        public Variable SomeVariables;
        private ExampleSubsystem inst; //這個是防呆

        private ExampleSubsystem(){ //這邊請記得設private
            ...
            register();
        }

        /**
         *對於資料類型的函數，請使用設計過的資料類如Pose2d, ChassisSpeeds，如果是是純量資料的話，請套上單位系統如(Distance, Angle, LinearVelocity, AngularVelocity等)，避免回傳double，不然沒有人知道你的資料是幹嘛的，還容易出錯。
        */
        public SomeData getData(){
            ...
        }

        /**
          *  請寫好你的函數的註解，方便別人要用的時候不用看你的程式看半個小時還看不懂在幹嘛，最好可以附實例片段。
          *  接受資料如果不是用設計過的資料類請使用有單位的資料做輸入，不然沒有人知道你的程式需要什麼會幹什麼事。
          *  @params var 理論上如果你有乖乖用自動選字的話他會自己跳出來，這邊就放你的程式需要的資料的功能
          * @return 這個函數在幹嘛 

        */
        public Command someCommand(SomeVariables var){
            ...
        }

        // 可以直接複製貼上改自己的類別名稱
        public static ExampleSubsystem getInstance(){
            inst = inst == null ? new ExampleSubsystem() : inst;
            return inst;
        }
    }

    ```

    這樣做的好處就是外部的類別在使用你的class的時候都會抓到的是同一個人。

    ```java
    public ExampleSubsystem subsystem = ExampleSubsystem.getInstance();
    ```

- 註解不要講幹話：如果函數的輸入變數名稱不能一眼看出來他在幹嘛的話，請在註解說明他是在幹嘛的。註解中應該要寫使用這個指令有哪些注意事項，或是他有套哪些自動化指令等等。
    - 關於如何寫好註解可以參考看看[這篇文章](https://blog.kyomind.tw/python-craftsman-01/)

- 善用 `@Deprecated` 標籤：如果你的函數名字變了，或是輸入、輸出的東西變了，不要把原本的函數刪掉，應該新增一個新的函數，然後把原本的函數deprecated掉，等到一次merge到main之後再一次刪
    - 詳細內容請參考 [這篇文章](https://sites.google.com/site/javahuide9/brief/annotation/deprecated)

- 如果有寫自定義類別或方法覺得通用性夠高或是放在自己的code裡面會很影響觀感的話可以放到utils的資料夾。

# 附錄
## 依賴函式庫

| 名稱 | 用途 |
|-- | --- |
| WPILibNewCommands | 內建函式庫|
| Phoenix 6 | TalonFX API |
| REVLib | SparkFlex API |
| DogLog | 資料收集、記錄|
| PhotonLib | 視覺處理|
| PathPlannerLib | 自動路徑規劃 |

