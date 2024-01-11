// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SimpleFalconSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class SwerveDriveFalcon extends SubsystemBase {
  private TalonFX motor;
  private String name;

  private final double Drive1rotationTicks = 13824.0;
  final double WheelDiameterMeters = 0.102;
  final double WheelCircumferenceMeters = WheelDiameterMeters * Math.PI;

  /** Creates a new SimpleFalconSubsystem. */
  public SwerveDriveFalcon(String name, int id, boolean invert) {
    motor = new TalonFX(id);
    motor.setInverted(invert);
    this.name = name;
    motor.setSelectedSensorPosition(0);
    motor.config_kP(0, 0.05);
    motor.config_kI(0, 0.0);
    motor.config_kD(0, 0.0);
    motor.config_kF(0, 0.0);

    motor.configNominalOutputForward(0.0);
    motor.configNominalOutputReverse(0.0);
    motor.configPeakOutputForward(+12.0);
    motor.configPeakOutputReverse(-12.0);
    motor.configNeutralDeadband(0.0);

    motor.setNeutralMode(NeutralMode.Coast);
  }

  double m_set;

  double convertMpsToTicksPer100ms(double mps) {
    return mps * Drive1rotationTicks / WheelCircumferenceMeters / 10.0;
  }

  /**
   * 
   * @param value meters per second
   */
  public void set(double value) {
    double ticksPer100ms = convertMpsToTicksPer100ms(value);
    motor.set(TalonFXControlMode.Velocity, ticksPer100ms);
    m_set = value;
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
    // // This method will be called once per scheduler run
    // SmartDashboard.putNumber(this.name + " velocity",
    // motor.getSelectedSensorVelocity());
    // SmartDashboard.putNumber(this.name + " position",
    // motor.getSelectedSensorPosition());
    // SmartDashboard.putNumber(this.name + " m_set", m_set);

  }
}
