// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SimpleFalconSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;

public class SwerveDriveKraken extends SubsystemBase {


  private TalonFX motor;
  private String name;

  private final double Drive1rotationTicks = 13824.0;
  final double WheelDiameterMeters = 0.102;
  final double WheelCircumferenceMeters = WheelDiameterMeters * Math.PI;

  StatusSignal<Double> velocitySupplier;
  StatusSignal<Double> motorPosition;
  int motorID;

  /** Creates a new SimpleFalconSubsystem. */
  public SwerveDriveKraken(String name, int id, boolean invert) {
    motor = new TalonFX(id);
    motor.setInverted(invert);
    this.motorID = id;
    this.name = name;
    var talonFXConfigs = new TalonFXConfiguration();

    // class member variable
    final VoltageOut m_request = new VoltageOut(0);
    // main robot code, command 12 V output
    motor.setControl(m_request.withOutput(12.0));

    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.05;
    slot0Configs.kP = 0.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;

    // apply gains, 50 ms total timeout
    motor.getConfigurator().apply(slot0Configs, 0.050);

    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    velocitySupplier = motor.getVelocity();
    motorPosition = motor.getPosition();

    set(0.0);

  
  }

  double m_set;

  // double convertMpsToTicksPer100ms(double mps) {
  // return mps * Drive1rotationTicks / WheelCircumferenceMeters / 10.0;
  // }

  // covert meters per second to rotations per second
  double wheelRadiusMeters = 0.0508; // meters

  double convertMpsToRps(double mps) {
    return mps / (2 * wheelRadiusMeters * Math.PI);
  }

  /**
   * 
   * @param value meters per second
   */
  public void set(double value) {
    double rps = convertMpsToRps(value);
    motor.setControl(new VelocityDutyCycle(rps));
    m_set = value;
  }

  final private static double DRIVE_MOTOR_TO_WHEEL_RATIO = 6.75;

  /**
   * get meters per second of the drive wheel
   */
  public double getRotationalVelocity() {
    var motorSpeed = velocitySupplier.getValueAsDouble();
    var metersPerSecond = motorSpeed / DRIVE_MOTOR_TO_WHEEL_RATIO * Math.PI * WheelDiameterMeters;
    return metersPerSecond;
  }

  // distance in meters
  public double getDistance() {
    return motor.getPosition().getValueAsDouble() / DRIVE_MOTOR_TO_WHEEL_RATIO * WheelDiameterMeters * Math.PI * 1.15;
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
    // if (motorID == 1) {
    // // System.out.println(motor.getRotorPosition());
    // System.out.println(
    // "Position =" + motor.getPosition().getValueAsDouble() * WheelDiameterMeters *
    // Math.PI);

    // }
  }
}
