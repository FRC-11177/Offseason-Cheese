package frc.robot.passer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.shooter.Shooter;

public class Passer implements Subsystem{
    
    public TalonFX PassLeftMotor,PassRightMotor;
    public TalonFXConfiguration PassLeftConfig,PassRightConfig;
    public MotionMagicVelocityVoltage PassPID;


    public static Passer inst;

    private Passer(){
        //左
        PassLeftMotor = new TalonFX(constant.PassLeftMotor);
        PassLeftConfig = new TalonFXConfiguration();
        PassPID = new MotionMagicVelocityVoltage(0);
    
        PassLeftConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);
        PassLeftConfig.Feedback
            .withSensorToMechanismRatio(constant.PassGearatio);
        PassLeftConfig.withSlot0(constant.PassPID);

        PassLeftMotor.getConfigurator().apply(PassLeftConfig);
     
        //右
        PassRightMotor = new TalonFX(constant.PassRightMotor);
        PassRightConfig = new TalonFXConfiguration();

        PassRightConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        PassRightConfig.Feedback
            .withSensorToMechanismRatio(constant.PassGearatio);
        PassRightConfig.withSlot0(constant.PassPID);

        PassRightMotor.getConfigurator().apply(PassRightConfig);

        register();
    }
}
