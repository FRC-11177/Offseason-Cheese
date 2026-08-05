package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.drivetrain.Constant.TunerSwerveDrivetrain;

/**
 * 機器人 Swerve 底盤控制子系統 (單例模式)。
 * 負責底盤運動控制、里程計 (Odometry) 計算、視覺定位融合以及 SysId 馬達參數校正。
 */
public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {
    
    private static Drivetrain inst; // 單例模式防呆實例

    private static final double kSimLoopPeriod = 0.004; // 4 毫秒
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* 藍色聯盟視角的正前方為 0 度（朝向紅色聯盟牆面） */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* 紅色聯盟視角的正前方為 180 度（朝向藍色聯盟牆面） */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* 記錄是否曾經套用過操作員視角設定 */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* 在 SysId 參數校正期間所採用的 Swerve 控制請求 */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* 校正平移動作的 SysId 流程。用於計算驅動馬達的 PID 增益。 */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // 使用預設斜率 (1 V/s)
            Volts.of(4), // 將動態步階電壓降至 4 V 以防止壓降
            null,        // 使用預設超時 (10 秒)
            // 使用 SignalLogger 類別記錄狀態
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* 校正轉向動作的 SysId 流程。用於計算轉向馬達的 PID 增益。 */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // 使用預設斜率 (1 V/s)
            Volts.of(7), // 使用 7 V 的動態步階電壓
            null,        // 使用預設超時 (10 秒)
            // 使用 SignalLogger 類別記錄狀態
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * 校正旋轉動作的 SysId 流程。
     * 用於計算 FieldCentricFacingAngle 車頭朝向控制器的 PID 增益。
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* 單位為 弧度/秒²，但 SysId 軟體僅支援 "伏特/秒" 標籤 */
            Volts.of(Math.PI / 6).per(Second),
            /* 單位為 弧度/秒，但 SysId 軟體僅支援 "伏特" 標籤 */
            Volts.of(Math.PI),
            null, // 使用預設超時 (10 秒)
            // 使用 SignalLogger 類別記錄狀態
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output 實際上為 弧度/秒，但 SysId 僅支援以 "伏特" 表示 */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* 記錄請求的輸出數值給 SysId 使用 */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* 當前準備執行的 SysId 測試流程 */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * 私有建構子：初始化硬體設備與 SwerveDrivetrain。
     * 請透過 {@link #getInstance()} 取得單例對象。
     */
    private Drivetrain() {
        super(
            Constant.DrivetrainConstants,
            Constant.FrontLeft,
            Constant.FrontRight,
            Constant.BackLeft,
            Constant.BackRight
        );

        if (Utils.isSimulation()) {
            startSimThread();
        }
        
        // 向 CommandScheduler 註冊此子系統
        register();
    }

    /**
     * 取得底盤目前的場地絕對姿態 (Pose2d)。
     * 
     * @return {@link Pose2d} 包含機器人的 X, Y 位置與朝向角度
     */
    public Pose2d getPose() {
        // ⭕ 正確：透過 getPigeon2() 方法存取 Pigeon 2 物件
        return getPigeon2() != null ? getState().Pose : new Pose2d();
    }


    /**
     * 取得底盤目前的航向角 (Angle)。
     * 
     * @return {@link Angle} 代表底盤目前的旋轉角度
     */
    public Angle getHeading() {
        return Degrees.of(getPigeon2().getYaw().getValueAsDouble());
    }

    /**
     * 取得底盤目前的機器人相對速度 (ChassisSpeeds)。
     * 
     * @return {@link ChassisSpeeds} 包含 vx, vy (m/s) 與 omega (rad/s)
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    /**
     * 取得特定時間點的姿態採樣數據。
     * 
     * @param timestamp 時間戳記 (例如從 Vision 傳入的測量時間)
     * @return {@link Optional}<{@link Pose2d}> 回傳該時間點估算的 Pose2d（若緩衝區無資料則為 empty）
     */
    public Optional<Pose2d> getSampledPoseAt(Time timestamp) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestamp.in(Seconds)));
    }

    /**
     * 將視覺辨識系統產生的姿態更新加入卡爾曼濾波器 (Kalman Filter) 進行融合與校正。
     * 
     * <pre>{@code
     * Pose2d visionPose = new Pose2d(2.5, 3.1, Rotation2d.fromDegrees(45));
     * drivetrain.addVisionMeasurement(visionPose, Seconds.of(Timer.getFPGATimestamp()));
     * }</pre>
     * 
     * @param visionRobotPoseMeters 攝影機測量到的機器人姿態 (Pose2d)
     * @param timestamp 該次視覺測量發生的時間戳記 (Time)
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, Time timestamp) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestamp.in(Seconds)));
    }

    /**
     * 將帶有標準差（可信度）的視覺辨識姿態加入卡爾曼濾波器進行融合。
     * 
     * @param visionRobotPoseMeters 攝影機測量到的機器人姿態 (Pose2d)
     * @param timestamp 該次視覺測量發生的時間戳記 (Time)
     * @param visionMeasurementStdDevs 視覺資料的標準差矩陣 [x, y, theta]ᵀ (單位：公尺與弧度)
     */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        Time timestamp,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(
            visionRobotPoseMeters, 
            Utils.fpgaToCurrentTime(timestamp.in(Seconds)), 
            visionMeasurementStdDevs
        );
    }

    /**
     * 套用指定之 Swerve 控制請求並持續執行的指令。
     * 
     * <pre>{@code
     * SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
     * Command driveCmd = drivetrain.applyRequest(() -> driveRequest.withVelocityX(2.0));
     * }</pre>
     * 
     * @param request 提供 {@link SwerveRequest} 控制請求的 Supplier
     * @return 執行該控制請求的 {@link Command}
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    /**
     * 執行 SysId 準靜態測試 (Quasistatic Test)。
     * 
     * @param direction 測試方向 ({@link SysIdRoutine.Direction})
     * @return 執行 SysId 準靜態測試的 {@link Command}
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * 執行 SysId 動態測試 (Dynamic Test)。
     * 
     * @param direction 測試方向 ({@link SysIdRoutine.Direction})
     * @return 執行 SysId 動態測試的 {@link Command}
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * 定期嘗試套用操作員視角。
         * 若尚未套用過視角，無論 DriverStation 狀態為何皆會強制套用。
         * 這能確保機器人程式在中途重啟時能正確修正操控視角。
         * 否則，僅在 DriverStation 處於 Disabled 停用狀態時進行檢查與更新，以防測試期間駕駛感受突然改變。
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* 以更高速率執行模擬線程，使 PID 控制行為在模擬中更接近真實狀況 */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* 使用測得的時間差，並從 WPILib 讀取目前的電池電壓 */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * 取得底盤控制子系統單例 (Singleton Instance)。
     * 
     * @return 全域唯一的 {@link Drivetrain} 實例
     */
    public static Drivetrain getInstance() {
        inst = inst == null ? new Drivetrain() : inst;
        return inst;
    }
}
