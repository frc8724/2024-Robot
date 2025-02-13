// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.IMayhemCANSparkMax;

public class ShooterMag extends SubsystemBase {
  IMayhemCANSparkMax leftMotor;
  IMayhemCANSparkMax rightMotor;

  /** Creates a new ShooterMag. */
  public ShooterMag(IMayhemCANSparkMax left, IMayhemCANSparkMax right) {
    leftMotor = left;
    rightMotor = right;

    leftMotor.setInverted(false);
    rightMotor.setInverted(false);

    setPowerVbus(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** set vbus percent power */
  public void setPowerVbus(double d) {
    leftMotor.setVBusPower(d);
    rightMotor.setVBusPower(d);
  }
}
