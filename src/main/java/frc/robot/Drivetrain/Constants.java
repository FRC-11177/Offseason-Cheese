package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
    public static Slot0Configs DrivePID = new Slot0Configs()
        .withKP(0).withKD(0)
        .withKS(0).withKV(0).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    public static Slot0Configs SteerPID = new Slot0Configs()
        .withKP(0).withKD(0)
        .withKS(0).withKV(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    public static MotionMagicConfigs DriveMagic = new MotionMagicConfigs()
        .withMotionMagicAcceleration(0)
        .withMotionMagicJerk(0);
    public static MotionMagicConfigs SteerMagic = new MotionMagicConfigs()
        .withMotionMagicExpo_kV(1000)
        .withMotionMagicExpo_kA(1000);
    
    public static final Current SlipCurrent = Amps.of(120);
    public static final Current SteerCurrent = Amps.of(60);
    public static final Distance WheelCirc = Inches.of(4).times(Math.PI);
    public static final Distance WheelRadius = Inches.of(2);
    public static final double DriveGearRatio = 1.0/(14.0/54*32/25*15/30);
    public static final double SteerGearRatio =  287/11;
    public static final double CouplingRatio = 1.0/(54/14.0);

    public static final LinearVelocity MaxDriveVelocity = MetersPerSecond.of(5);
    public static final AngularVelocity MaxOmega = RotationsPerSecond.of(1.5);

    public static final double WheelOffset = Centimeters.of(55).in(Meters);

    public static List<ModuleConfig> ModuleConfigs = List.of(
        new ModuleConfig(
            7, 
            8, 
            10, 
            Rotations.of(0), 
            false, 
            new Translation2d(-WheelOffset, WheelOffset)),

        new ModuleConfig(
            5, 
            6, 
            2, 
            Rotations.of(0), 
            true, 
            new Translation2d(WheelOffset, WheelOffset)),
        new ModuleConfig(
            1, 
            2, 
            3, 
            Rotations.of(0), 
            false, 
            new Translation2d(-WheelOffset, -WheelOffset)),
        new ModuleConfig(
            3, 
            4, 
            4, 
            Rotations.of(0), 
            true, 
            new Translation2d(WheelOffset, -WheelOffset))
    );

    public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(ModuleConfigs.stream().map(ModuleConfig::place).toArray(Translation2d[]::new));

    public record ModuleConfig(
        int DriveID,
        int SteerID,
        int EncoderID,
        Angle Offset,
        boolean isRight,
        Translation2d place
    ){

    }
}
