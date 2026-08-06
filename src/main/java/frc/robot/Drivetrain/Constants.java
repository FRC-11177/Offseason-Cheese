package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public class Constants {
    public static CANBus bus = new CANBus("rio");
    public static Slot0Configs DrivePID = new Slot0Configs()
        .withKP(0).withKD(0)
        .withKS(0).withKV(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    public static Slot0Configs SteerPID = new Slot0Configs()
        .withKP(0).withKD(0)
        .withKS(0).withKD(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    //控制是不是用磁場導向(FOC)控制
    public static ClosedLoopOutputType DriveOutputType = ClosedLoopOutputType.Voltage;
    public static ClosedLoopOutputType SteerOutputType = ClosedLoopOutputType.Voltage;

    //馬達(嚴格來講應該是控制器)的種類
    public static DriveMotorArrangement DriveMotor = DriveMotorArrangement.TalonFX_Integrated;
    public static SteerMotorArrangement SteerMotor = SteerMotorArrangement.TalonFX_Integrated;
    public static SteerFeedbackType SteerFeedback = SteerFeedbackType.RemoteCANcoder; //沒有用鈔能力所以用Remote, 如果有鈔能力的話建議用Fused

    //開的馬達他會開始滑的電流
    public static Current SlipCurrent = Amps.of(120); //TODO: 要記得調
     
    public static TalonFXConfiguration DriveConfig = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(70)) //參考CTRE官方文件Improving Performance with Current Limits
                .withSupplyCurrentLimitEnable(true)
        );
    public static TalonFXConfiguration SteerConfig = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(70)) //同上
                .withStatorCurrentLimitEnable(true)
        );
    public static CANcoderConfiguration EncoderConfig = new CANcoderConfiguration(); //把設定刷回去原廠設定
    public static Pigeon2Configuration GyroConfig = null; //因為陀螺儀通常不會用程式設定，所以就用null防止原本有的值被刷掉

    public static Distance WheelRadius = Inches.of(2);
    public static LinearVelocity MaxVelocity = WheelRadius.times(2).times(Math.PI).times(100/Constants.DriveGearRatio).per(Second);
    public static LinearVelocity MaxDriveVelocity = MetersPerSecond.of(4);
    public static AngularVelocity MaxDriveOmega = RotationsPerSecond.of(1.5);
    public static final double DriveGearRatio = 1.0/(14.0/54*32/25*15/30);
    public static final double SteerGearRatio = 287.0/11;
    public static final double CoupleGearRatio = 54.0/14;

    //.模擬器在用的東西不用理他
    public static MomentOfInertia SimulationInertia = KilogramSquareMeters.of(0.01);
    public static Voltage SimulationFrictionVoltage = Volts.of(0.2);

    public static SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
        .withCANBusName(bus.getName())
        .withPigeon2Id(0)
        .withPigeon2Configs(GyroConfig);

    public static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ModuleFactory =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(DriveGearRatio)
            .withSteerMotorGearRatio(SteerGearRatio)
            .withCouplingGearRatio(CoupleGearRatio)
            .withWheelRadius(WheelRadius)
            .withSteerMotorGains(SteerPID)
            .withDriveMotorGains(DrivePID)
            .withSteerMotorClosedLoopOutput(SteerOutputType)
            .withDriveMotorClosedLoopOutput(DriveOutputType)
            .withSlipCurrent(SlipCurrent)
            .withSpeedAt12Volts(MaxVelocity)
            .withDriveMotorType(DriveMotor)
            .withSteerMotorType(SteerMotor)
            .withFeedbackSource(SteerFeedback)
            .withDriveMotorInitialConfigs(DriveConfig)
            .withSteerMotorInitialConfigs(SteerConfig)
            .withEncoderInitialConfigs(EncoderConfig)
            .withSteerInertia(SimulationInertia)
            .withDriveInertia(SimulationInertia)
            .withSteerFrictionVoltage(SimulationFrictionVoltage)
            .withDriveFrictionVoltage(SimulationFrictionVoltage);

    public static double WheelOffset = Centimeters.of(55).div(2).in(Meters);

    public static List<ModuleConfig> modules = List.of(
        new ModuleConfig( //FL
            7, 
            8, 
            12, 
            Rotations.of(0), 
            new Translation2d(-WheelOffset, WheelOffset), 
            false),
        new ModuleConfig( //FR
            5, 
            6, 
            11, 
            Rotations.of(0), 
            new Translation2d(WheelOffset, WheelOffset), 
            true),
        new ModuleConfig( //BL
            1, 
            2, 
            9, 
            Rotations.of(0), 
            new Translation2d(-WheelOffset, -WheelOffset), 
            false),
        new ModuleConfig( //BR
            3, 
            4, 
            10, 
            Rotations.of(0), 
            new Translation2d(WheelOffset, -WheelOffset), 
            true)
    );

    public record ModuleConfig(
        int DriveID, int SteerID, int EncoderID,
        Angle offset,
        Translation2d place, boolean isRight
    ) {
        public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> toModuleConstants(){
            return Constants.ModuleFactory.createModuleConstants(
                SteerID, 
                DriveID, 
                EncoderID, 
                offset, 
                place.getMeasureX(), place.getMeasureY(), 
                isRight, 
                true, 
                false);
        }
    }
}
