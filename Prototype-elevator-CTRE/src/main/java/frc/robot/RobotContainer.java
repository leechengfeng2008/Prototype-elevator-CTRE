// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Subsystem.elevatorSubsystem;

public class RobotContainer {
  private final elevatorSubsystem elevator = new elevatorSubsystem();
  private final CommandXboxController testController = new CommandXboxController(1);
  private final CommandXboxController elevatorController = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    elevatorController.a().whileTrue(new InstantCommand(()->elevator.setElevatorTargetPosition(40/9)));
    elevatorController.b().whileTrue(new InstantCommand(()->elevator.setElevatorTargetPosition(2/9)));
    elevatorController.x().whileTrue(new InstantCommand(()->elevator.setElevatorTargetPosition(80/9)));
    elevatorController.y().whileTrue(new InstantCommand(()->elevator.setElevatorTargetPosition(120/9)));
    // elevatorController.leftBumper().whileTrue(new InstantCommand(()->elevator.setElevatorTargetPosition(0)));                       


    testController.a().whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    testController.b().whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    testController.x().whileTrue(elevator.sysIdDymamic(SysIdRoutine.Direction.kForward));
    testController.y().whileTrue(elevator.sysIdDymamic(SysIdRoutine.Direction.kReverse));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
