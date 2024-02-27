// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SystemArmZero;
import frc.robot.subsystems.DriveBase.DriveForDistance;
import frc.robot.subsystems.DriveBase.DriveZeroWheels;
import frc.robot.subsystems.ShooterSubsystem.ShootNote;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootandDriveOut extends SequentialCommandGroup {
  /** Creates a new AutoDriveOut. */
  public AutoShootandDriveOut() {
    addCommands(
        new AutoStartingPosition(30),
        // new ShootNote(2600),
        new DriveForDistance(1.0, -30.0, 0.0, 1.0));

  }
}
