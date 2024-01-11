// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveBase;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveEncoder extends SubsystemBase {

  AnalogInput m_analogInput;

  /** Creates a new SwerveEncoder. */
  public SwerveEncoder(int channel) {
    m_analogInput = new AnalogInput(channel);
  }

  public int get() {
    return m_analogInput.getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("mag encoder " + m_analogInput.getChannel(), m_analogInput.getValue());
  }
}
