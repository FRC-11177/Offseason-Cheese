package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

// 由 2026 Tuner X Swerve 專案產生器自動生成
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class Constant {
    // 兩組 PID 增益皆需根據個別機器人進行校正調校。

    // 轉向馬達使用指定的控制類型，輸出類型由 SwerveModuleConstants.SteerMotorClosedLoopOutput 設定
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(1.48).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    // 使用閉迴路控制時，驅動馬達使用指定於 SwerveModuleConstants.DriveMotorClosedLoopOutput 的輸出類型
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);

    // 轉向馬達採用的閉迴路輸出類型；這會影響轉向馬達的 PID/FF 增益
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // 驅動馬達採用的閉迴路輸出類型；這會影響驅動馬達的 PID/FF 增益
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // 驅動馬達採用的馬達類型
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // 轉向馬達採用的馬達類型
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // 轉向馬達採用的遠端感測器回授類型；
    // 未訂閱 Pro 版授權時，Fused*/Sync* 會自動回退為 Remote*
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // 輪子開始打滑時的定子電流限制 (Stator Current)；需根據機器人實體進行調校
    private static final Current kSlipCurrent = Amps.of(120);

    // 驅動馬達、轉向馬達與方位角編碼器的初始設定檔；此處不可為 null。
    // 部分設定檔會被覆蓋，詳情請查閱 `with*InitialConfigs()` API 文件。
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // 預設電源電流限制為 70 A，但可進一步調低以避免壓降 (Brownout)。
                // 電源電流限制值可以大於斷路器 (Breaker) 的額定電流。
                .withSupplyCurrentLimit(Amps.of(70))
                .withSupplyCurrentLimitEnable(true)
        );
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve 的方位轉向不需要太大的扭力輸出，因此可以設定較低的定子電流限制以防止壓降，且不影響效能。
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Pigeon 2 設定檔；保持 null 即可跳過套用 Pigeon 2 設定
    private static final Pigeon2Configuration pigeonConfigs = null;

    // 設備所連接的 CAN 總線；所有 Swerve 設備必須在同一個 CAN 總線上
    public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");

    // 在 12 V 輸出電壓下測得的機器人速度 (m/s)；
    // 這不是機器人期望的最大速度 - 最大速度設定請參考 RobotContainer 中的 MaxSpeed；
    // 此數值需根據個別機器人實體調校
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4);

    // 方位角每旋轉 1 圈導致驅動馬達轉動的 kCoupleRatio 圈數 (齒輪耦合比)；
    // 需根據機器人實體調校
    private static final double kCoupleRatio = 0;

    private static final double kDriveGearRatio = 6.75;
    private static final double kSteerGearRatio = 15.42857142857143;
    private static final Distance kWheelRadius = Inches.of(2);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 1;

    // 以下設定僅用於電腦模擬 (Simulation)
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.035);
    // 用於克服摩擦力所需的模擬電壓
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // 左前輪 (Front Left)
    private static final int kFrontLeftDriveMotorId = 7;
    private static final int kFrontLeftSteerMotorId = 8;
    private static final int kFrontLeftEncoderId = 12;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.496582);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(13.5);
    private static final Distance kFrontLeftYPos = Inches.of(13.5);

    // 右前輪 (Front Right)
    private static final int kFrontRightDriveMotorId = 5;
    private static final int kFrontRightSteerMotorId = 6;
    private static final int kFrontRightEncoderId = 11;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.066895);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(13.5);
    private static final Distance kFrontRightYPos = Inches.of(-13.5);

    // 左後輪 (Back Left)
    private static final int kBackLeftDriveMotorId = 1;
    private static final int kBackLeftSteerMotorId = 2;
    private static final int kBackLeftEncoderId = 9;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.106201);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-13.5);
    private static final Distance kBackLeftYPos = Inches.of(13.5);

    // 右後輪 (Back Right)
    private static final int kBackRightDriveMotorId = 3;
    private static final int kBackRightSteerMotorId = 4;
    private static final int kBackRightEncoderId = 10;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.264648);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-13.5);
    private static final Distance kBackRightYPos = Inches.of(-13.5);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );

    /**
     * 利用 CTR Electronics Phoenix 6 API 與指定硬體設備建構的 Swerve 驅動類別。
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }
}
