package frc.robot.passer;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

public class constant {
    public static final int PassRightMotor = 24 ,PassLeftMotor = 23 ;
    public static final double PassGearatio = 2.25;
    public static final double PassCirc = Inches.of(2.25).times(Math.PI).in(Meters);
    public static final Slot0Configs PassPID = new Slot0Configs()
         .withKS(0.22)
         .withKV(0.279)
         .withKP(1.2)
         .withKD(0);


}
