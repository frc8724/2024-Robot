// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem.ArmIsAtPosition;
import frc.robot.subsystems.ArmSubsystem.ArmSet;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.DriveBase.DriveForDistance;
import frc.robot.subsystems.ShooterSubsystem.ShootShort;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStartLongShootDriveOut extends SequentialCommandGroup {
  /** Creates a new AutoStartLeftShootDriveOut. */
  public AutoStartLongShootDriveOut(double alliance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AutoStartingPosition(60.0 * alliance),
        new ArmSet(ArmSubsystem.SHORT_SHOT),
        new ArmIsAtPosition(ArmSubsystem.POSITION_SLOP),
        new ShootShort(),
        new DriveForDistance(2.0, 55 * alliance, 0, 2.0),
        new ArmSet(ArmSubsystem.NOTE_INTAKE),
        new DriveForDistance(2.0, 55 * alliance, 0, 3.0),
        new DriveForDistance(2.0, 2.0, 0, 0, 2.5, 4),
        new DriveForDistance(0, 0, 0, 0));
  }
}
