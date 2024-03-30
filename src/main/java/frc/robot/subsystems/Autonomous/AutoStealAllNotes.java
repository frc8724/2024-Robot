// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveBase.DriveAtConstantRotation;
import frc.robot.subsystems.DriveBase.DriveForDistance;
import frc.robot.subsystems.IntakeRollers.IntakeSequenceOn;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStealAllNotes extends SequentialCommandGroup {
  /** Creates a new AutoStealAllNotes. */
  public AutoStealAllNotes(double alliance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AutoStartingPosition(0),
        new DriveForDistance(0.5, 4.0, 0 * alliance, 0, .5),
        new DriveForDistance(4.0, 0 * alliance, 0, 7.5),
        new DriveForDistance(4.0, 0.01, 0 * alliance, 0, .5),
        new DriveForDistance(-0.1, 0, 0, 0.01),
        new WaitCommand(0.2),
        // new DriveForDistance(2.25, -90, 0, 4)
        new DriveAtConstantRotation(2.25, 1.0, -85 * alliance, 3.0, 7)
    // new ParallelCommandGroup(
    // new IntakeSequenceOn(),
    // new DriveAtConstantRotation(2.25, 0.75, -90 * alliance, 3.0, 1))
    );

  }
}
