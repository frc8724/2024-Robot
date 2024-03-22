// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;

public class ClimberAtPosition extends Command {
  double tolerance;

  /** Creates a new ClimberAtPosition. */
  public ClimberAtPosition() {
    this(ClimberSubsystem.POSITION_SLOP);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ClimberAtPosition(double t) {
    this.tolerance = t;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      RobotContainer.m_climber.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_climber.isAtPosition(tolerance);
  }
}
