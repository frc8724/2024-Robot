// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem.ArmSet;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.DriveBase.DriveForDistance;
import frc.robot.subsystems.IntakeRollers.IntakeRollersSet;
import frc.robot.subsystems.ShooterSubsystem.ShootNote;
import frc.robot.subsystems.ShooterSubsystem.ShootNotePost;
import frc.robot.subsystems.ShooterSubsystem.ShootNotePre;
import frc.robot.subsystems.ShooterSubsystem.ShootShort;
import frc.robot.subsystems.ShooterSubsystem.ShooterMagSet;
import frc.robot.subsystems.ShooterSubsystem.ShooterWheelsSet;
import frc.robot.subsystems.System.SystemIntakeNote;
import frc.robot.subsystems.System.SystemIntakeOff;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AuthDriveMidNote3AndBack extends SequentialCommandGroup {
  /** Creates a new AuthDriveMidNote3AndBack. */
  public AuthDriveMidNote3AndBack() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new AutoStartingPosition(0.0),

        // drive forward
        new DriveForDistance(4, 0, 0, 2.0),
        // slide to the right
        new DriveForDistance(4, 45, 0, 2.2),
        // drive to the midline
        new DriveForDistance(4, 0, 0, 2.5),
        new ParallelCommandGroup(
            new SystemIntakeNote(),
            new DriveForDistance(2, 0, 0, 1.6)),

        new ParallelRaceGroup(
            new DriveForDistance(-0.1, 0, 0, 0.0),
            new WaitCommand(0.1)),
        new ParallelCommandGroup(
            new DriveForDistance(-2, 0, 0, 1.8),
            new ArmSet(ArmSubsystem.ZERO_POSITION),
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                new IntakeRollersSet(0.0),
                new ShooterMagSet(0.0),
                new ShooterWheelsSet(0.0))),
        // drive back under the stage
        new DriveForDistance(-4, 0, 0, 2.5),
        // slide to the left
        new DriveForDistance(-4, 45, 0, 2.2),
        //warm up shooter
        new ParallelCommandGroup(
            new ShootNotePre(5000),
        // drive back to the subwoofer
        new DriveForDistance(-4, 0, 0, 2.0)),
    
        // STOP! new ParallelRaceGroup(

        new DriveForDistance(-0.1, 0, 0, 0.0),
        new WaitCommand(0.1),
        // new WaitCommand((0.1))

        new ShootNotePost(5000));
  }
}
