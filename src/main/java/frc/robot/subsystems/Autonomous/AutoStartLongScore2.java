// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem.ArmIsAtPosition;
import frc.robot.subsystems.ArmSubsystem.ArmSet;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.DriveBase.DriveForDistance;
import frc.robot.subsystems.DriveBase.DriveStop;
import frc.robot.subsystems.IntakeRollers.IntakeRollersSet;
import frc.robot.subsystems.ShooterSubsystem.ShootNote;
import frc.robot.subsystems.ShooterSubsystem.ShootShort;
import frc.robot.subsystems.ShooterSubsystem.ShooterMagSet;
import frc.robot.subsystems.ShooterSubsystem.ShooterWheelsSet;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStartLongScore2 extends SequentialCommandGroup {
  /** Creates a new AutoStartLeftScore2. */
  public AutoStartLongScore2(double alliance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AutoStartingPosition(60.0 * alliance),
        new ArmSet(ArmSubsystem.SHORT_SHOT),
        new ArmIsAtPosition(ArmSubsystem.POSITION_SLOP),
        new ShootShort(),
        // move the arm
        new ArmSet(ArmSubsystem.NOTE_INTAKE),
        // intake the note
        new IntakeRollersSet(0.5),
        new ShooterMagSet(0.25),
        new ShooterWheelsSet(-0.1),
        // drive to get the note
        new DriveForDistance(1.7, 50 * alliance, 0, 0.8),
        new DriveForDistance(1.7, 20 * alliance, 0, 1.0),
        // drive back to the speaker
        new DriveForDistance(-1.7, 20.0 * alliance, 60 * alliance, 1.0),
        new DriveForDistance(-1.7, 50 * alliance, 60 * alliance, 0.8),
        // new DriveForDistance(0.1, 45 * alliance, 60 * alliance, 0.01), // stop
        new DriveStop(),

        new IntakeRollersSet(0.0),
        new ShooterMagSet(0.0),
        new ShooterWheelsSet(0.0),
        new ArmSet(ArmSubsystem.SHORT_SHOT),
        new ArmIsAtPosition(ArmSubsystem.POSITION_SLOP),
        new ShootShort());

  }
}
