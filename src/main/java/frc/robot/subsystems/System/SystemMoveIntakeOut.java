// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.System;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem.ArmSet;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemMoveIntakeOut extends SequentialCommandGroup {
  /** Creates a new SystemMoveIntakeOut. */
  public SystemMoveIntakeOut() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SelectCommand< Boolean >(
        Map.ofEntries(
            Map.entry(false, new ArmSet(ArmSubsystem.STOW)),
            Map.entry(true, new WaitCommand(0.0))),
        () -> RobotContainer.m_arm.getCurrentPositionInTicks()> ArmSubsystem.FLOOR_PICKUP_BACK));

  }
}
