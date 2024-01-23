// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterMag extends SubsystemBase {
  TalonFX m_motor = new TalonFX(Constants.MotorIDs.MAGAZINE_LEFT);
  TalonFX m_motorFollower = new TalonFX(Constants.MotorIDs.MAGAZINE_RIGHT);

  /** Creates a new ShooterMag. */
  public ShooterMag() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double d) {
  
  }
}
