// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.System;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem.ArmIsAtPosition;
import frc.robot.subsystems.ArmSubsystem.ArmSet;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.IntakeRollers.IntakeRollersSet;
import frc.robot.subsystems.ShooterSubsystem.ShooterMagSet;
import frc.robot.subsystems.ShooterSubsystem.ShooterWheelBrake;
import frc.robot.subsystems.ShooterSubsystem.ShooterWheelsSet;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemIntakeNote extends SequentialCommandGroup {
  /** Creates a new SystemIntakeNote. */
  public SystemIntakeNote() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ArmSet(ArmSubsystem.NOTE_INTAKE),
        new ArmIsAtPosition(ArmSubsystem.POSITION_SLOP),
        new IntakeRollersSet(0.45),
        new ShooterMagSet(0.5),
        new ShooterWheelsSet(0.0),
        new ShooterWheelBrake());
  }
}
