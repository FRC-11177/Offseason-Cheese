package frc.robot.shooter;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class constant {
    public static final int ShootMotor = 55;
    public static final double ShootGearatio = 1;
    public static final double ShootCirc = Inches.of(2.25).times(Math.PI).in(Meters);
    public static final Slot0Configs ShootPID = new Slot0Configs()
         .withKS(0)
         .withKG(0)
         .withKP(0)
         .withKD(0);
    public static final MotionMagicConfigs ShootMotionMagic = new MotionMagicConfigs()
        .withMotionMagicAcceleration(0)
        .withMotionMagicJerk(0);

    /**
     * 確切三維待確認
     * Transfor3d ( x , y , z ,rotation 180/0 標記方向用)
     */
    public static Transform3d ShootPlace = new Transform3d(
        Centimeter.of(0),Centimeter.of(50),Centimeter.of(0), new Rotation3d(Rotation2d.kZero)
    );


    public static final Transform3d place = new Transform3d(
        Centimeter.of(0),
        Centimeter.of(0),
        Centimeters.of(0),
        Rotation3d.kZero
    );

    public static final Rotation2d PitchAngle = Rotation2d.fromDegrees(18);
    //射出角度
    //public static final Distance ShooterToHub;
 

    /**
     * 設定場地
     *
     */
    public enum FieldPlace{
        HUB;
        /**
         * 取得固定3D座標
         * @return 紅隊/藍隊 的絕對座標
         */
        public Pose3d getPose3d(){
            return switch (this) {
                case HUB -> 
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 
                    new Pose3d(
                        Meters.of(15),
                        Meters.of(4),
                        Meters.of(3),
                        Rotation3d.kZero
                    ) : 
                    new Pose3d();
                
            };
        }

        public Pose2d getPose2d(){
            return getPose3d().toPose2d();
        }
    }

}
