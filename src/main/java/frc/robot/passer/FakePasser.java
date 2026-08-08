package frc.robot.passer;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class FakePasser implements Subsystem{
    public TalonFX LeftPassMotor, RightPassMotor;
    
    public MotionMagicVelocityVoltage LeftPID;
    public Follower RightPID;

    private TalonFXConfiguration LeftConfig, RightConfig;

    public FakePasser(){
        LeftPassMotor = new TalonFX(0);
        RightPassMotor = new TalonFX(0);

        LeftPID = new MotionMagicVelocityVoltage(0);
        RightPID = new Follower(LeftPassMotor.getDeviceID(), MotorAlignmentValue.Opposed);

        LeftConfig = new TalonFXConfiguration();
        RightConfig = new TalonFXConfiguration();

        LeftConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast);

        LeftPassMotor.getConfigurator().apply(LeftConfig);

    }

    public void set(){
        LeftPassMotor.setControl(LeftPID.withVelocity(RadiansPerSecond.of(10)));
        RightPassMotor.setControl(RightPID);
    }
}
