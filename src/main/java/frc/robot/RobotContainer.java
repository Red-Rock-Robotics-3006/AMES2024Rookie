// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystem.Door;
import frc.robot.subsystem.Elevator;

public class RobotContainer {
  private final Door m_door = new Door();
  private final Elevator m_elevator = new Elevator();

  private final CommandXboxController driverController = new CommandXboxController(0);


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    this.driverController.povUp()
      .onTrue(this.m_elevator.topCommand());
    this.driverController.povDown()
      .onTrue(this.m_elevator.bottomCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}