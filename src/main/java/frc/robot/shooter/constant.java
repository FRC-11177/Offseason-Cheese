package frc.robot.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

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

}
