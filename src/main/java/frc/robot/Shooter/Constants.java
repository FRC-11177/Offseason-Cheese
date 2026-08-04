package frc.robot.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    public static final int[] MotorID = {51,52};
    public static final double GearRatio = 1;
    public static final Distance WheelCirc = Centimeters.of(3).times(Math.PI);

    public static Slot0Configs ShootPID = new Slot0Configs()
        .withKP(0).withKD(0)
        .withKS(0).withKV(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    public static MotionMagicConfigs ShootMagic = new MotionMagicConfigs()
        .withMotionMagicAcceleration(0)
        .withMotionMagicJerk(0);
    public static Current CurrentLimit = Amps.of(60);


}
