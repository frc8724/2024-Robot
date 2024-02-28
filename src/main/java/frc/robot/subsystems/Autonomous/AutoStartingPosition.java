// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SystemArmZero;
import frc.robot.subsystems.DriveBase.DriveForDistance;
import frc.robot.subsystems.DriveBase.DriveZeroGyro;
import frc.robot.subsystems.DriveBase.DriveZeroWheels;
import frc.robot.subsystems.DriveBase.DrivebaseResetEncoders;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStartingPosition extends SequentialCommandGroup {
  /** Creates a new AutoStartingPosition. */
  public AutoStartingPosition(double startingAngle) {
    addCommands(
        new SequentialCommandGroup(
            new DriveZeroWheels(),
            new DriveZeroGyro(0.0),
            new WaitCommand(0.1),
            new DrivebaseResetEncoders(),
            new InstantCommand(() -> RobotContainer.m_robotDrive.drive(0.2, 0, 0, true), RobotContainer.m_robotDrive),
            new WaitCommand(0.1),
            new DriveZeroWheels(),
            new WaitCommand(0.1),
            new DriveZeroWheels(),
            new DriveZeroGyro(0.0),
            new WaitCommand(0.1),
            new DrivebaseResetEncoders(),
            new InstantCommand(() -> RobotContainer.m_robotDrive.drive(-0.2, 0, 0, true), RobotContainer.m_robotDrive),
            new WaitCommand(0.1),
            new DriveZeroWheels(),
            new DriveZeroGyro(-startingAngle)));
  }
}
