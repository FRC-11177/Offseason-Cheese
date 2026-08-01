package frc.robot.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class constant {
    public static final int UpMotorID = 10;
    public static final double UpGearatio = 1 ;
    
    public static final Angle Upoffset = Degrees.of(0);
    public static final FeedForwardConfig UpFF = new FeedForwardConfig()
        .svag(0, 0, 0, 0);
    public static final MAXMotionConfig UpMotion = new MAXMotionConfig()
        .cruiseVelocity(0)
        .maxAcceleration(0);
    public static final ClosedLoopConfig UpPID = new ClosedLoopConfig()
        .pid(0, 0, 0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .apply(UpFF).apply(UpMotion);
    public static final SoftLimitConfig UpLimit = new SoftLimitConfig()
        .forwardSoftLimit(0)
        .reverseSoftLimit(0)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);
    public static final int TurnMotorID = 20;
    public static final double TurnGearatio = 1;
    public static final double TurnCirc = Inches.of(2.25).times(Math.PI).in(Meters);
    public static final double TurnMaxVelocity = 100/constant.TurnGearatio*TurnCirc;

}
