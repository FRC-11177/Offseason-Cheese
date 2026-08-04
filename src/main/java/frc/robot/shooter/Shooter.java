package frc.robot.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.security.Key;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule;

import dev.doglog.DogLog;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Drivetain.Drivetrain;
import frc.robot.intake.intake;
import frc.robot.shooter.constant.FieldPlace;

public class Shooter implements Subsystem{
    public TalonFX ShootMotor;
    public TalonFXConfiguration ShootConfig;
    public MotionMagicVelocityVoltage ShootPID;
    public LinearVelocity IdleVelocity = MetersPerSecond.of(1);
    public LinearVelocity targetSpeed = IdleVelocity;

    private final double g = 9.807;
    public static Shooter inst;

    private Shooter(){
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

        setDefaultCommand(setState(IdleVelocity));

        
        
    }
    /**
     * 取得當下的線速度
     * @return 取得速度值轉換成每秒轉＊輪周（單位：每秒公尺）
     */
    public LinearVelocity getState(){
        return MetersPerSecond.of(ShootMotor.getVelocity().getValue().in(RotationsPerSecond)*constant.ShootCirc);

    }
    
/**
 * 設定目標速度
 * @param target 目標（線速度）
 * @return 執行
 * 設定控制 -> 從PID讀取速度（每秒轉之每秒公尺目標除以輪周）
 */
    public Command setState(LinearVelocity target){
        return run(
            () -> {
                ShootMotor.setControl(ShootPID.withVelocity(RotationsPerSecond.of(target.in(MetersPerSecond)/constant.ShootCirc)));
             }
        );
    }

    public Command shoot(){
        return setState(MetersPerSecond.of(
            Math.sqrt(
                g*getDistanceToHub()*getDistanceToHub()/
                2*constant.PitchAngle.getCos()*constant.PitchAngle.getCos()*(getDistanceToHub()*constant.PitchAngle.getTan()-)

            )
        ));

    }

    private double getDistanceToHub(){
        return PhotonUtils.getDistanceToPose(Drivetrain.getInstance().getPose3d().plus(constant.place).toPose2d(), FieldPlace.HUB.getPose2d());
    }
    @Override
    public void periodic(){
        targetSpeed = MetersPerSecond.of(ShootPID.getVelocityMeasure().in(RotationsPerSecond)*constant.ShootCirc);
        DogLog.log("Shooter/CurrentVelocity", getState());
        DogLog.log("Shooter/TargetVelocity", targetSpeed);
    }

    public static Shooter getInstance(){
        inst = inst == null ? new Shooter() : inst;
        return inst;
    }
}