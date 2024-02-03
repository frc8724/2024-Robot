// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LimeLight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class CenterOnTag extends Command {
  public CenterOnTag() {
    addRequirements(RobotContainer.m_robotDrive);
    addRequirements(RobotContainer.m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.m_limelight.getTv() == 1)
    {
        while (RobotContainer.m_limelight.getTx() != 0) {
            double lateralOffset = RobotContainer.m_limelight.getTx();
            double magnitude = -(lateralOffset / Math.abs(lateralOffset));
            RobotContainer.m_robotDrive.drive(0.0, 0.2 * magnitude, 0.0, false);
        }

        // while (condition) {
            
        // }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.m_robotDrive.drive(0.0, 0.0, 0.0, false);
  }

  double getDistance(double x, double y) {
    return Math.sqrt(x * x + y * y);
  }

  // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     Pose2d p = RobotContainer.m_robotDrive.getPose();
//     double dist = getDistance(p.getX(), p.getY());

//     return dist >= m_distance;
//   }
}
