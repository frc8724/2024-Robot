// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmIsAtPosition extends Command {
  /** Creates a new CheckingPosition. */
  double tolerance;

  public ArmIsAtPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    this(ArmSubsystem.POSITION_SLOP);
  }

   public ArmIsAtPosition(double t) {
    this.tolerance = t;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
