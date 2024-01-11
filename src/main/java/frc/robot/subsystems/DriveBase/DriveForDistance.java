// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DriveForDistance extends Command {
  double m_x;
  double m_y;
  double m_rot;
  double m_distance;

  public DriveForDistance(double x, double y, double rot, double distance) {
    m_x = x;
    m_y = y;
    m_rot = rot;
    m_distance = distance;
    addRequirements(RobotContainer.m_robotDrive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d pose = new Pose2d();
    RobotContainer.m_robotDrive.resetOdometry(pose);
    RobotContainer.m_robotDrive.drive(m_x, m_y, m_rot, false);
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
  @Override
  public boolean isFinished() {
    Pose2d p = RobotContainer.m_robotDrive.getPose();
    double dist = getDistance(p.getX(), p.getY());

    return dist >= m_distance;
  }
}
