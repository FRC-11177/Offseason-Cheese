// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.indexer.Indexer;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;

public class RobotContainer {
  public Intake intake = Intake.getInstance();
  public Shooter shooter = Shooter.getInstance();
  public Indexer indexer = Indexer.getInstance();
   public CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    intake.setDefaultCommand(intake.setState(new SwerveModuleState()));
    configureBindings();
  }

  private void configureBindings() {
    /**
     * intake 按下後旋轉
     */
      controller.a().whileTrue(command);


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
