package frc.robot.drivetrain;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem{
    private static Drivetrain inst;

    private Drivetrain(){
        super(TalonFX::new, TalonFX::new, CANcoder::new, 
        Constants.DrivetrainConstants, Constants.modules.stream().<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>map(c -> c.toModuleConstants()).toArray(SwerveModuleConstants<?,?,?>[]::new));
    }

    public ChassisSpeeds getSpeeds(){
        return getState().Speeds;
    }

    public Pose2d getPose(){
        return this.getState().Pose;
    }

    public Pose3d getPose3d(){
        return new Pose3d(getPose());
    }
    
    /**
     * 底盤操作的原函數
     * @param req 向量底盤的控制請求，可以用 {@link SwerveRequest} 的自動選字看看;
     * @return 開的函數
     */
    public Command drive(Supplier<SwerveRequest> req){
        return run(() -> setControl(req.get()));
    }

    public static Drivetrain getInstance(){
        inst = inst == null ? new Drivetrain() : inst;
        return inst;
    }
}
