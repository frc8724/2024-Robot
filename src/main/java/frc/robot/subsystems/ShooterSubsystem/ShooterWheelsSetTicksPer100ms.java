// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterWheelsSetTicksPer100ms extends InstantCommand {
  double m_ticksPer100ms;

  public ShooterWheelsSetTicksPer100ms(double t) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ticksPer100ms = t;
    addRequirements(RobotContainer.m_wheels);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_wheels.setShooterSpeed(m_ticksPer100ms);
  }
}
