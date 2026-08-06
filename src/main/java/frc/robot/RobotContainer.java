// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Drivetrain.Constants;
import frc.robot.Drivetrain.Drivetrain;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  public CommandXboxController controller = new CommandXboxController(0);
  public SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
    .withDesaturateWheelSpeeds(true)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.Position);

  public RobotContainer() {
    drivetrain.setDefaultCommand(drivetrain.drive(() -> driveRequest
      .withVelocityX(Constants.MaxDriveVelocity.times(controller.getLeftX()))
      .withVelocityY(Constants.MaxDriveVelocity.times(controller.getLeftY()))
      .withRotationalRate(Constants.MaxDriveOmega.times(controller.getRightX()))
      ));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
