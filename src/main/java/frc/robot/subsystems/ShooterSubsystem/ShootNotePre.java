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
public class ShootNotePre extends SequentialCommandGroup {
  /** Creates a new ShootNote. */
  public ShootNotePre(double shooterSpeed) {
    addCommands(
        // set mag to backwards so note is at position

        new ShooterMagSet(-0.07),
        new ShooterWheelsSet(-0.1),
        new WaitCommand(0.2),

        new ShooterMagSet(0.0),
        new ShooterWheelsSet(0.0),
        new WaitCommand(0.2),

        // turn on the shooter wheels
        new ShooterWheelsSetTicksPer100ms(shooterSpeed));
  }
}
