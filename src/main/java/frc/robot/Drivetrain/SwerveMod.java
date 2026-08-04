package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Drivetrain.Constants.ModuleConfig;

public class SwerveMod {
    public TalonFX DriveMotor, SteerMotor;
    public CANcoder Encoder;

    public MotionMagicVelocityVoltage DrivePID;
    public MotionMagicExpoVoltage SteerPID;

    private TalonFXConfiguration DriveConfig, SteerConfig;
    private CANcoderConfiguration EncoderConfig;

    public SwerveMod(ModuleConfig config){
        DriveMotor = new TalonFX(config.DriveID());
        SteerMotor = new TalonFX(config.SteerID());
        Encoder = new CANcoder(config.EncoderID());

        DrivePID = new MotionMagicVelocityVoltage(0);
        SteerPID = new MotionMagicExpoVoltage(0);

        DriveConfig = new TalonFXConfiguration();
        SteerConfig = new TalonFXConfiguration();
        EncoderConfig = new CANcoderConfiguration();

        DriveConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(config.isRight() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive);
        DriveConfig.withSlot0(Constants.DrivePID);
        DriveConfig.withMotionMagic(Constants.DriveMagic);
        DriveConfig.CurrentLimits
            .withStatorCurrentLimit(Constants.SlipCurrent)
            .withSupplyCurrentLimit(Amps.of(70)) //source:https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true); 
        DriveConfig.Feedback
            .withSensorToMechanismRatio(Constants.DriveGearRatio);
        
        SteerConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        SteerConfig.withSlot0(Constants.SteerPID);
        SteerConfig.withMotionMagic(Constants.SteerMagic);
        SteerConfig.CurrentLimits
            .withStatorCurrentLimit(Constants.SteerCurrent)
            .withStatorCurrentLimitEnable(true);
        SteerConfig.Feedback
            .withRemoteCANcoder(Encoder)
            .withSensorToMechanismRatio(Constants.SteerGearRatio);
        SteerConfig.ClosedLoopGeneral
            .withContinuousWrap(true);

        EncoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withMagnetOffset(config.Offset());

        DriveMotor.getConfigurator().apply(DriveConfig);
        SteerMotor.getConfigurator().apply(SteerConfig);
        Encoder.getConfigurator().apply(EncoderConfig);
    }


    /**
     * 取得目前向量模塊的狀態
     *
     * <p>此函數會回傳目前模組的輪速與轉向角度，其中輪速會先進行
     * coupling gear ratio 補償，以消除轉向馬達運動對驅動編碼器造成的額外旋轉。
     *
     * @return 目前向量模塊的狀態，用 {@link SwerveModuleState 表示}
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Constants.WheelCirc.per(Seconds)
            .times(
                DriveMotor.getVelocity().getValue()
                    .minus(
                        SteerMotor.getVelocity().getValue().times(Constants.CouplingRatio)
                    ).in(RotationsPerSecond)
            ),
            new Rotation2d(SteerMotor.getPosition().getValue())
        );
    }

    /**
     * 取得目前 Swerve 模組的位置狀態。
     *
     * <p>此方法會回傳模組目前的輪子行進距離以及轉向角度。
     * 在計算輪子距離時，會套用 coupling gear ratio 補償，
     * 移除因轉向機構旋轉所造成的驅動軸額外旋轉量。
     *
     * @return 包含目前輪子行進距離與轉向角度的{@link SwerveModulePosition}
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Constants.WheelCirc
            .times(
                DriveMotor.getPosition().getValue()
                    .minus(
                        SteerMotor.getPosition().getValue().times(Constants.CouplingRatio)
                    ).in(Rotations)
            ),
            new Rotation2d(SteerMotor.getPosition().getValue())
        );
    }

    /**
     * 讓向量模塊轉到目標的狀態
     * @param state {@link SwerveModuleState} 目標狀態(可不最佳化)
     */
    public void setState(SwerveModuleState state){
        state.optimize(getState().angle);

        DriveMotor.setControl(DrivePID.withVelocity(RotationsPerSecond.of(state.speedMetersPerSecond/Constants.WheelCirc.in(Meters))));
        SteerMotor.setControl(SteerPID.withPosition(state.angle.getMeasure()));
    }

    /**
     * 記錄用函數，記得在底盤裡面串
     */
    public void log(){
        DogLog.log("Drivetrain/Modules/%d/DriveTorque".formatted(Encoder.getDeviceID()), DriveMotor.getStatorCurrent().getValue().in(Amps) * DriveMotor.getMotorKT().getValue().in(NewtonMeters.per(Amps)), NewtonMeters);
    }
}
