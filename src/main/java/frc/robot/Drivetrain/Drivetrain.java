package frc.robot.Drivetrain;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Drivetrain implements Subsystem{
    public List<SwerveMod> modules;
    public Pigeon2 gyro;
    public SwerveDrivePoseEstimator3d PoseEstimator;
    
    private static Drivetrain inst;

    private Drivetrain(){
        modules = Constants.ModuleConfigs.stream().map(SwerveMod::new).toList();
        gyro = new Pigeon2(0);
        PoseEstimator = new SwerveDrivePoseEstimator3d(Constants.kinematics, gyro.getRotation3d(), getPositions(), new Pose3d());

        register();
    }

    /**
     * 取得所有向量模塊的位置
     * @return 所有模塊目前的位置，用 {@link SwerveModulePosition} 表示
     */
    public SwerveModulePosition[] getPositions(){
        return modules.stream().map(SwerveMod::getPosition).toArray(SwerveModulePosition[]::new);
    }

    /**
     * 取得所有向量模塊的狀態
     * @return 所有模塊目前的狀態，用 {@link SwerveModuleState} 表示
     */
    public SwerveModuleState[] getStates(){
        return modules.stream().map(SwerveMod::getState).toArray(SwerveModuleState[]::new);
    }

    /**
     * 建立給driver用的開的函數
     *
     * @param vx x方向速度 {@link LinearVelocity} 
     * @param vy y方向速度 {@link LinearVelocity}
     * @param omega z軸自轉速度 {@link AngularVelocity}
     * @param isRobotCentric 讓使用者可以傳入是不是用機器中心控制
     *
     * @return 開底盤的函數
     */
    public Command drive(
        Supplier<LinearVelocity> vx,
        Supplier<LinearVelocity> vy,
        Supplier<AngularVelocity> omega,
        Supplier<Boolean> isRobotCentric
    ){
        return drive(
            isRobotCentric.get() ? 
            ChassisSpeeds.discretize(new ChassisSpeeds(vx.get(),vy.get(), omega.get()), 0.2) :
            ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(vx.get(), vy.get(),omega.get()), gyro.getRotation2d()), 0.2)
        );
    }

    /**
     * <strong>注意：</strong>不建議使用這個函數，如果要給driver用的話，可以使用 {@link #drive(Supplier, Supplier, Supplier, Supplier)}
     * @param spds 機器中心的速度
     * @return 開底盤的指令
     */
    public Command drive(ChassisSpeeds spds){
        return run(() -> setStates(Constants.kinematics.toSwerveModuleStates(spds)));
    }

    /**
     * Internal function to access swerve module
     * @param states
     */
    private void setStates(SwerveModuleState[] states){
        IntStream.range(0, 4)
            .forEach(i -> modules.get(i).setState(states[i]));
    }

    public Pose2d getPose(){
        return PoseEstimator.getEstimatedPosition().toPose2d();
    }

    @Override
    public void periodic(){
        PoseEstimator.update(gyro.getRotation3d(), getPositions());
        modules.stream().forEach(SwerveMod::log);
    }

    public static Drivetrain getInstance(){
        inst = inst == null ? new Drivetrain() : inst;
        return inst;
    }
}
