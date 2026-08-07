package frc.robot.indexer;


import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

public class constant {
    public static final int IndexID = 31;
    public static final double IndexGearatio = 40.0/16;
    public static final double IndexCirc = Inches.of(2.25).times(Math.PI).in(Meters);;

    public static final Slot0Configs IndexPID = new Slot0Configs()
        .withKS(0.35)
        .withKP(0.1)
        .withKV(12.4/100*constant.IndexGearatio);

    public static MotionMagicConfigs indexMotionMagic = new MotionMagicConfigs()
        .withMotionMagicAcceleration(25)
        .withMotionMagicJerk(180);
    
    

}
