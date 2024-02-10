// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LimeLight;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
        Pose2d currentPose2d = RobotContainer.m_robotDrive.getPose();

        double x_offset = RobotContainer.m_limelight.getTx();
        double y_offset = RobotContainer.m_limelight.getTy();
        double a_offset = RobotContainer.m_limelight.getTa();

        double new_y = currentPose2d.getY() + y_offset;
        double new_x = currentPose2d.getX() + x_offset;
        double new_a = currentPose2d.getRotation().getDegrees() + a_offset;

        TrajectoryConfig config =
        new TrajectoryConfig(
                0.2, //Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                0.2) //Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.DriveConstants.kDriveKinematics);

        var interiorWaypoints = new ArrayList<Translation2d>();

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          RobotContainer.m_robotDrive.getPose(),
          interiorWaypoints,
          new Pose2d(new_x, new_y, new Rotation2d(new_a)),
          config
        );


        // while (RobotContainer.m_limelight.getTx() > 0.2) {
        //     double lateralOffset = RobotContainer.m_limelight.getTx();
        //     double magnitude = -(lateralOffset / Math.abs(lateralOffset));
        //     RobotContainer.m_robotDrive.drive(0.0, 0.2 * magnitude, 0.0, false);
        // }

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
