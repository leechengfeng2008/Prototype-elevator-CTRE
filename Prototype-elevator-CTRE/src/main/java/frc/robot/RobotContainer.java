// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Subsystem.elevatorSubsystem;

public class RobotContainer {
  private final elevatorSubsystem elevator = new elevatorSubsystem();
  private final CommandXboxController elevController = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    elevController.a().whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    elevController.b().whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    elevController.x().whileTrue(elevator.sysIdDymamic(SysIdRoutine.Direction.kForward));
    elevController.y().whileTrue(elevator.sysIdDymamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
