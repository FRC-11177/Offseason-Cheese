// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Shooter.Shooter;
import frc.robot.Drivetrain.Constants;

public class RobotContainer {
  Drivetrain drivetrain = Drivetrain.getInstance();
  Shooter shooter = Shooter.getInstance();
  CommandXboxController controller = new CommandXboxController(0);
  boolean isRobotCentric = false;
  public RobotContainer() {
    drivetrain.setDefaultCommand(drivetrain.drive(
      () -> Constants.MaxDriveVelocity.times(controller.getLeftX()),
      () -> Constants.MaxDriveVelocity.times(controller.getRightX()),
      () -> Constants.MaxOmega.times(controller.getRightX()),
      () -> false
    ));
    configureBindings();
  }

  private void configureBindings() {
    controller.a().onTrue(Commands.runOnce(() -> isRobotCentric = !isRobotCentric));
    controller.b().onTrue(shooter.setVelocity(MetersPerSecond.of(114514)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
