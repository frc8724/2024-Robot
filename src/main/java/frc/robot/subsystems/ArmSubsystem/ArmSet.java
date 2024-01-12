// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class ArmSet extends InstantCommand {
  /** Creates a new ArmSet. */
  double percent;
  public ArmSet(double d) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
    percent = d;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  RobotContainer.m_arm.set(percent);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
}