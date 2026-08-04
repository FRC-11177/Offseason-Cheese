package frc.robot.Shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Shooter implements Subsystem{
    public TalonFX LeftShoot, RightShoot;

    public MotionMagicVelocityVoltage LeftPID;
    public Follower RightPID;

    private TalonFXConfiguration LeftConfig, RightConfig;

    private static Shooter inst;

    private Shooter(){
        LeftShoot = new TalonFX(Constants.MotorID[0]);
        RightShoot = new TalonFX(Constants.MotorID[1]);
        LeftConfig = new TalonFXConfiguration();
        RightConfig = new TalonFXConfiguration();

        LeftPID = new MotionMagicVelocityVoltage(0);
        RightPID = new Follower(LeftShoot.getDeviceID(), MotorAlignmentValue.Opposed);

        LeftConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive);
        LeftConfig.withSlot0(Constants.ShootPID);
        RightConfig.withMotionMagic(Constants.ShootMagic);
        LeftConfig.CurrentLimits
            .withStatorCurrentLimit(Constants.CurrentLimit)
            .withStatorCurrentLimitEnable(true);

        setDefaultCommand(setVelocity(MetersPerSecond.of(1)));
        register();
    }

    /**
     * @deprecated 這個函數因為自解釋性的關係已經被棄用了，請轉向使用{@link #getVelocity()}
     * @return 目前shooter的速度
     */
    @Deprecated(forRemoval = true)
    public LinearVelocity getState(){
        return MetersPerSecond.of(LeftShoot.getVelocity().getValue().in(RotationsPerSecond) * Constants.WheelCirc.in(Meters));
    }

    /**
     * 取得目前shooter的速度
     * @return 目前shooter的速度
     */
    public LinearVelocity getVelocity(){
        return  MetersPerSecond.of(LeftShoot.getVelocity().getValue().in(RotationsPerSecond) * Constants.WheelCirc.in(Meters));
    }

    /**
     * 取得shooter的目標速度
     * @return {@link LinearVelocity} shooter 的目標速度
     */
    public LinearVelocity getTargetVelocity(){
        return MetersPerSecond.of(LeftPID.getVelocityMeasure().in(RotationsPerSecond) * Constants.WheelCirc.in(Meters));
    }

    public Command setVelocity(LinearVelocity vel){
        return run(() -> {
            LeftShoot.setControl(LeftPID.withVelocity(RotationsPerSecond.of(vel.in(MetersPerSecond)/Constants.WheelCirc.in(Meters))));
            RightShoot.setControl(RightPID);
        }).until(() -> getTargetVelocity().isNear(getVelocity(), 0.05));
    }

    public static Shooter getInstance(){
        inst = inst == null ? new Shooter() : inst;
        return inst;
    }
    
}