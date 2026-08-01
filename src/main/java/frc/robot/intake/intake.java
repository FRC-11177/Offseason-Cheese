package frc.robot.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class intake implements Subsystem {
    public SparkFlex UpMotor;
    public TalonFX TurnMotor;
    
    public RelativeEncoder UpEncoder; // 編碼器
    public SparkClosedLoopController UpPID; //閉環控制器

    public SparkMaxConfig UpConfig;
    public TalonFXConfiguration TurnConfig;

    public DutyCycleOut TurnPID;
    private static intake inst;
    
    private intake(){
        UpMotor = new SparkFlex(constant.UpMotorID, MotorType.kBrushless);
        TurnMotor = new TalonFX(constant.TurnMotorID);

        UpEncoder = UpMotor.getEncoder() ;
        UpPID = UpMotor.getClosedLoopController();

        UpConfig = new SparkMaxConfig();
        TurnConfig = new TalonFXConfiguration();

        TurnPID = new DutyCycleOut(0);
        
        UpConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(40);
        UpConfig.encoder
            .positionConversionFactor(1/constant.UpGearatio)
            .velocityConversionFactor(1/constant.UpGearatio/60);
        UpConfig.apply(constant.UpPID);
        UpConfig.apply(constant.UpLimit);

        TurnConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);

        TurnConfig.Feedback
            .withSensorToMechanismRatio(constant.TurnGearatio);
        TurnMotor.getConfigurator().apply(TurnConfig);
        UpMotor.configure(UpConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);   
    
        register();
        setState(new SwerveModuleState(0, getState().angle));
    }

    /**
     * 取得目前intake狀態，用 SwerveModuleState 簡化，速度是輪子的轉速(m/s),角度是intake的轉角(Rotation2d)
     * @return intake的狀態
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            TurnMotor.getVelocity().getValue().in(RotationsPerSecond)*constant.TurnCirc,
            Rotation2d.fromRotations(UpEncoder.getPosition())
        );
    }
    /**
     * 當執行setState，把機構轉到設定的狀態
     * @param target 目標的狀態
     * @return 轉的的函數
     */

    public Command setState(SwerveModuleState target){
        return run(() -> {
            UpPID.setSetpoint(target.angle.getRotations(), ControlType.kMAXMotionPositionControl);
            TurnMotor.setControl(TurnPID.withOutput(target.speedMetersPerSecond/constant.TurnMaxVelocity));
        }).until(() -> getState().angle.getMeasure().isNear(target.angle.getMeasure(), 0.05));
    }

    public Command shake(){
        return setState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)))
                .andThen(setState(new SwerveModuleState(0, Rotation2d.fromDegrees(50))))
                .repeatedly();
    }

    @Override
    public void periodic(){
        DogLog.log("Intake/state", getState());
    }

    public static intake getInstance(){
        inst = inst == null ? inst : new intake();
        return inst;

    }

}
