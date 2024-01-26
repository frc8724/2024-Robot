// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollers extends SubsystemBase {
//  CANSparkMax m_motor = new CANSparkMax(Constants.MotorIDs.INTAKE_ROLLERS, CANSparkMaxLowLevel.MotorType.kBrushless);

  /** Creates a new IntakeRollers. */
  public IntakeRollers() {
    //m_motor.setSmartCurrentLimit(999);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double d) {
   // m_motor.set(d);
  }

}
