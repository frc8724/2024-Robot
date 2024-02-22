// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SystemArmZero;
import frc.robot.subsystems.DriveBase.DriveForDistance;
import frc.robot.subsystems.IntakeRollers.IntakeRollersPickupSet;
import frc.robot.subsystems.ShooterSubsystem.ShootNote;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveandShootandPickupX2 extends SequentialCommandGroup {
  /** Creates a new AutoDriveandShootandPickupX2. */
  public AutoDriveandShootandPickupX2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SystemArmZero(),
        new DriveForDistance(0.0, 0.2, 0.0, 100.0),
        new ShootNote(2600),
        new IntakeRollersPickupSet(),

        new DriveForDistance(0.0, 0.2, 0.0, 100.0),
        new ShootNote(2600),
        new IntakeRollersPickupSet());
  }
}
