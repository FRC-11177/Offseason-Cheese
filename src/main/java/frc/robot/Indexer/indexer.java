package frc.robot.Indexer;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.shooter.Shooter;

public class Indexer implements Subsystem{
    public TalonFX IndexMotor ;
    public TalonFXConfiguration IndexConfig;
    public MotionMagicVelocityVoltage IndexPID;

    public static Indexer inst;

    /**
     * 跟Shooter自動綁定在一起，無公開指令
     */
    private Indexer(){
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
        setDefaultCommand(setState(Shooter.getInstance().targetSpeed.times(0.6))
        .onlyIf(() -> Shooter.getInstance().targetSpeed.in(MetersPerSecond) != 1));
    } 

    /**
     * 取得目前indexer的轉速（用LinearVelocity表示）
     * 
    */
    public LinearVelocity getState(){
        return MetersPerSecond.of(IndexMotor.getVelocity().getValue().in(RotationsPerSecond)*constant.IndexCirc);
    }
    
    private Command setState (LinearVelocity target){
        return runEnd(
           () -> IndexMotor.setControl(IndexPID.withVelocity(RotationsPerSecond.of(target.in(MetersPerSecond)/constant.IndexCirc))),
            IndexMotor::stopMotor);
    }

    @Override
    public void periodic() {
        DogLog.log("Indexer/Velocity", getState());
    }

    public static Indexer getInstance(){
        inst = inst == null ? inst : new Indexer();
        return inst;
    }
}
