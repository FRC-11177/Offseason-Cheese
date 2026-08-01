package frc.robot.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.intake.intake;

public class Shooter implements Subsystem{
    public TalonFX ShootMotor;
    public TalonFXConfiguration ShootConfig;
    public MotionMagicVelocityVoltage ShootPID;
    
    public static Shooter inst;

    public Shooter(){
        ShootMotor = new TalonFX(constant.ShootMotor);
        ShootConfig = new TalonFXConfiguration();
        ShootPID = new MotionMagicVelocityVoltage(0);

        ShootConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        ShootConfig.Feedback
            .withSensorToMechanismRatio(constant.ShootGearatio);
        ShootConfig.withSlot0(constant.ShootPID);
        ShootConfig.withMotionMagic(constant.ShootMotionMagic);

        ShootMotor.getConfigurator().apply(ShootConfig);
        /**
         *  寫設定，寫getState setState(要吃linearvelocity)
         * 
         */
        register();
    }
    /**
     * 取得當下的線速度
     * @return 取得速度值轉換成每秒轉＊輪周（單位：每秒公尺）
     */
    public LinearVelocity getState(){
        return MetersPerSecond.of(ShootMotor.getVelocity().getValue().in(RotationsPerSecond)*constant.ShootCirc);

    }
    

    public Command setState(LinearVelocity target){
        return run(
            () -> {
                ShootMotor.setControl(ShootPID.withVelocity(RotationsPerSecond.of(target.in(MetersPerSecond)/constant.ShootCirc)));

             }
        );
    }

    public static Shooter getInstance(){
        inst = inst == null ? new Shooter() : inst;
        return inst;
    }
}