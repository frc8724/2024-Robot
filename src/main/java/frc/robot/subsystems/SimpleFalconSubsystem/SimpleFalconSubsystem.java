// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SimpleFalconSubsystem;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class SimpleFalconSubsystem extends SubsystemBase {
  private TalonFX motor;
  private String name;

  private final double Drive1rotationTicks = 13257.0;
  final double WheelDiameterMeters = 0.102;
  // final double TalonFXEncoderTicks = 2048;

  final double Turning1RotationTicks = 21674.0;

  /** Creates a new SimpleFalconSubsystem. */
  public SimpleFalconSubsystem(String name, int id, boolean invert) {
    motor = new TalonFX(id);
    motor.setInverted(invert);
    this.name = name;
    motor.setSelectedSensorPosition(0);
  }

  double m_set;

  public void set(double percent) {
    motor.set(TalonFXControlMode.PercentOutput, percent);
    m_set = percent;
  }

  public double getRotationRadians() {
    return motor.getSelectedSensorPosition() / Turning1RotationTicks * Math.PI;
  }

  /**
   * get meters per second of the drive wheel
   */
  public double getRotationalVelocity() {
    var ticksPerSecond = motor.getSelectedSensorVelocity() / 0.100;
    var metersPerSecond = ticksPerSecond / Drive1rotationTicks * Math.PI * WheelDiameterMeters;
    return metersPerSecond;
  }

  // distance in meters
  public double getDistance() {
    return motor.getSelectedSensorPosition() / Drive1rotationTicks * WheelDiameterMeters * Math.PI;
  }

  public void reset() {
    set(0.0);
    motor.setSelectedSensorPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber(this.name + " velocity",
    // motor.getSelectedSensorVelocity());
    // SmartDashboard.putNumber(this.name + " position",
    // motor.getSelectedSensorPosition());
    // SmartDashboard.putNumber(this.name + " rads", this.getRotationRadians());
    // SmartDashboard.putNumber(this.name + " m_set", m_set);

  }
}
