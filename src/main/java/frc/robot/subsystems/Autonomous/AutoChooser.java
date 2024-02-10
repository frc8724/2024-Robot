// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoChooser extends SubsystemBase {
  /** Creates a new AutoChooser. */
  public AutoChooser() {
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public void addAuto(Command cmd) {
    String name = cmd.getClass().getSimpleName();
    autoChooser.addOption(name, cmd);
  }

  public Command getAutoCommand() {
    return autoChooser.getSelected();

  }

  public static Command AutoDriveOut(AutoChooser mAuto) {
    return null;
  }
}
