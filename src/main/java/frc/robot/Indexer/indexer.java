package frc.robot.Indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class indexer implements Subsystem{
    public TalonFX IndexMotor ;
    public TalonFXConfiguration IndexConfig;
    public MotionMagicVelocityVoltage IndexPID;
    public static indexer inst;

    private indexer(){
        IndexMotor =  new TalonFX(constant.IndexID);
        IndexConfig = new TalonFXConfiguration();
        IndexPID = new MotionMagicVelocityVoltage(0);

        IndexConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);

        IndexConfig.Feedback
            .withSensorToMechanismRatio(constant.IndexGearatio);
        IndexMotor.getConfigurator().apply(IndexConfig);

        register();

    } 
    
    public static indexer getInstance(){
        inst = inst == null ? inst : new indexer();
        return inst;
    }
}
