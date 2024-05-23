// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem.ArmIsAtPosition;
import frc.robot.subsystems.ArmSubsystem.ArmSet;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.DriveBase.DriveForDistance;
import frc.robot.subsystems.DriveBase.DriveStop;
import frc.robot.subsystems.IntakeRollers.IntakeRollersSet;
import frc.robot.subsystems.ShooterSubsystem.ShootShort;
import frc.robot.subsystems.ShooterSubsystem.ShootNotePost;
import frc.robot.subsystems.ShooterSubsystem.ShootNotePre;
import frc.robot.subsystems.ShooterSubsystem.ShooterMagSet;
import frc.robot.subsystems.ShooterSubsystem.ShooterWheelAtSpeed;
import frc.robot.subsystems.ShooterSubsystem.ShooterWheelsSet;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootAndDrivex2 extends SequentialCommandGroup {
    final static double TOP_SPEED = 4.0;

    /** Creates a new AutoDriveandShootandPickupX2. */
    public AutoShootAndDrivex2() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                // start flat against the speaker
                new AutoStartingPosition(0.0),
                // shoot the note close
                new ShootShort(8000),
                // move the arm
                new ArmSet(ArmSubsystem.NOTE_INTAKE + 3000),

                // drive forward
                new ParallelCommandGroup(
                        new ArmIsAtPosition(ArmSubsystem.POSITION_SLOP),
                        new SequentialCommandGroup(
                                new DriveForDistance(.1, TOP_SPEED, 0.0, 0.0, 0.1),
                                new DriveForDistance(TOP_SPEED, 0.0, 0.0, 0.4))),

                // make sure the arm is at position
                // drive to get the note
                new ParallelCommandGroup(
                        new IntakeRollersSet(0.5),
                        new ShooterMagSet(0.25),
                        new ShooterWheelsSet(-0.1),
                        // start driving
                        new DriveForDistance(TOP_SPEED, 0.0, 0.0, 0.5)),
                // finish driving
                new DriveForDistance(TOP_SPEED, 0.0, 0.0, 0.4),
                new DriveForDistance(TOP_SPEED, .1, 0.0, 0.0, 0.1),

                // drive back
                new DriveForDistance(-.1, -TOP_SPEED, 0.0, 0.0, 0.1),
                new DriveForDistance(-TOP_SPEED, 0.0, 0.0, 0.5),

                new ParallelCommandGroup(
                        // new DriveForDistance(-TOP_SPEED, 0.0, 0.0, .4),
                        new DriveForDistance(-TOP_SPEED, -.01, 0.0, 0.0, 0.1),
                        new SequentialCommandGroup(
                                new IntakeRollersSet(0),
                                new ShooterMagSet(0.0),
                                new ShooterWheelsSet(0.0),
                                // put the arm to the shooting position
                                new ArmSet(ArmSubsystem.LONG_SHOT + 6000),
                                new ShootNotePre(4500))),

                // new DriveForDistance(-TOP_SPEED, -.1, 0.0, 0.0, .1),
                // stop
                new DriveStop(),
                // new DriveForDistance(0.1, 0., 0.0, 0.1),
                // make sure the arm is at position.
                new ArmIsAtPosition(ArmSubsystem.POSITION_SLOP),
                // wait until it is at speed or 2 seconds
                new ParallelRaceGroup(
                        new ShooterWheelAtSpeed(),
                        new WaitCommand(2.0)),
                // shoot the note
                new ShootNotePost(4500),

                new ArmSet(ArmSubsystem.ZERO_POSITION));

    }
}
