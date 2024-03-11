// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem.ArmIsAtPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNotePost extends SequentialCommandGroup {
  /** Creates a new ShootNote. */
  public ShootNotePost(double shooterSpeed) {
    addCommands(
        // move the note to the shooter
        new ShooterMagSet(1.0),
        // wait for the note to leave
        new WaitCommand(0.5),
        // turn off the mag and shooter
        new ShooterMagSet(0.0),
        new ShooterWheelsSet(0.0));
  }
}
