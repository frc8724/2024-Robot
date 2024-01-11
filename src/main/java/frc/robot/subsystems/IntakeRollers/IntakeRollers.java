// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeRollers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {
  CANSparkMax topRollerMotor = new CANSparkMax(61, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax bottomRollerMotor = new CANSparkMax(60, CANSparkMaxLowLevel.MotorType.kBrushless);

  /** Creates a new IntakeRollers. */
  public IntakeRollers() {
    topRollerMotor.setSmartCurrentLimit(999);
    bottomRollerMotor.setSmartCurrentLimit(999);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double d) {
    topRollerMotor.set(d);
    bottomRollerMotor.set(d);
  }

}
