// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Targeting;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class TargetingSet extends InstantCommand {
  /** Creates a new TargetingSet. */
  double percent;

  public TargetingSet(double d) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(RobotContainer.m_targets);
    percent = d;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // RobotContainer.m_climber.setInTicks(percent);

  }
}
