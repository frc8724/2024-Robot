// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SystemArmZero;
import frc.robot.subsystems.ArmSubsystem.ArmIsAtPosition;
import frc.robot.subsystems.ArmSubsystem.ArmSet;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.DriveBase.DriveForDistance;
import frc.robot.subsystems.IntakeRollers.IntakeRollersPickupSet;
import frc.robot.subsystems.IntakeRollers.IntakeRollersSet;
import frc.robot.subsystems.ShooterSubsystem.ShootShort;
import frc.robot.subsystems.ShooterSubsystem.ShootNote;
import frc.robot.subsystems.ShooterSubsystem.ShootNotePost;
import frc.robot.subsystems.ShooterSubsystem.ShootNotePre;
import frc.robot.subsystems.ShooterSubsystem.ShooterMagSet;
import frc.robot.subsystems.ShooterSubsystem.ShooterWheelsSet;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootAndDrivex2 extends SequentialCommandGroup {
  /** Creates a new AutoDriveandShootandPickupX2. */
  public AutoShootAndDrivex2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // start flat against the speaker
        new AutoStartingPosition(0.0),
        // shoot the note close
        new ShootShort(),
        // move the arm
        new ArmSet(ArmSubsystem.NOTE_INTAKE),

        // drive forward
        new DriveForDistance(2.5, 0.0, 0.0, 0.5),

        // make sure the arm is at position
        new ArmIsAtPosition(ArmSubsystem.POSITION_SLOP),  
        // drive to get the note

      new ParallelCommandGroup(
        new IntakeRollersSet(0.5),
        new ShooterMagSet(0.25),
        new ShooterWheelsSet(-0.1),
    //start driving
        new DriveForDistance(2.5, 0.0, 0.0, 0.5)),
    //finish driving
        new DriveForDistance(2.5, 0.0, 0.0, 0.5),

        

//drive back
        new DriveForDistance(-2.5, 0.0, 0.0, 0.7),

        new ParallelCommandGroup(
            new DriveForDistance(-2.5, 0.0, 0.0, 1.0),
            new SequentialCommandGroup(
                new IntakeRollersSet(0),
                new ShooterMagSet(0.0),
                new ShooterWheelsSet(0.0),
                // put the arm to the shooting position
                new ArmSet(ArmSubsystem.ZERO_POSITION),
                new ShootNotePre(4500))),
        // stop
        new DriveForDistance(0., 0.0, 0.0, 0.0),
        // make sure the arm is at position.
        new ArmIsAtPosition(ArmSubsystem.POSITION_SLOP),
        new WaitCommand(0.5),
        // shoot the note
        new ShootNotePost(4500));
  }
}
