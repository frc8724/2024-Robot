// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SimpleFalconSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class SwerveTurnWheelTo extends InstantCommand {
  double m_rad;

  public SwerveTurnWheelTo(double rad) {
    addRequirements(RobotContainer.m_robotDrive);
    m_rad = rad;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_robotDrive.setWheelsAt(m_rad);
  }
}
