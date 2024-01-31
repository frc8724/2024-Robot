// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SimpleFalconSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionDutyCycle;

public class SwerveDriveFalcon extends SubsystemBase {
  // private TalonFX motor;
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
    
    motor.getConfigurator().apply(new TalonFXConfiguration());
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP =0.5;
    slot0Configs.kP =0.0;
    slot0Configs.kP =0.5;
    slot0Configs.kP =0.0;
    motor.getConfigurator().apply(slot0Configs, 0.050);
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
    motor.set( ticksPer100ms);
    m_set = value;
  }

  /**
   * get meters per second of the drive wheel
   */
  public double getRotationalVelocity() {
    var ticksPerSecond = motor.getVelocity().getValueAsDouble()/ 0.100;
    var metersPerSecond = (double)ticksPerSecond / Drive1rotationTicks * Math.PI * WheelDiameterMeters;
    return metersPerSecond;
  }

  // distance in meters
  public double getDistance() {
    return motor.getPosition().getValueAsDouble() / Drive1rotationTicks * WheelDiameterMeters * Math.PI;
  }

  public void reset() {
    set(0.0);
    motor.setPosition(0.0);
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
