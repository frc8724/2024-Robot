// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberSubsystem;

import frc.robot.motors.MayhemTalonFX;
import frc.robot.motors.MayhemTalonFX.CurrentLimit;

import org.opencv.dnn.Image2BlobParams;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motors.IMayhemTalonFX;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {

  // static final double TICKS_PER_INCH = 3442;

  public static final double POSITION_SLOP = 1000.0;
  static final double CLOSED_LOOP_RAMP_RATE = 1.0; // todo: lower this value

  IMayhemTalonFX leftMotor;
  IMayhemTalonFX rightMotor;

  /** Creates a new Arm. */
  public ClimberSubsystem(IMayhemTalonFX left, IMayhemTalonFX right) {
    leftMotor = left;
    rightMotor = right;

    configTalon(leftMotor);
    configTalon(rightMotor);
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    zero();
    setPower(0.0);

  }

  private void configTalon(IMayhemTalonFX talon) {
    talon.setNeutralMode(NeutralMode.Brake);

    talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    talon.config_kP(0, 0.15);
    talon.config_kI(0, 0.0);
    talon.config_kD(0, 50.0);
    talon.config_kF(0, 0.0);

    talon.configMotionCruiseVelocity(60000); // measured velocity of ~100K at 85%; set cruise to that
    talon.configMotionAcceleration(1 * 40000); // acceleration of 2x velocity allows cruise to be attained in 1
                                               // second
    // second

    talon.configAllowableClosedloopError(0, POSITION_SLOP, 0);
    talon.configClosedloopRamp(CLOSED_LOOP_RAMP_RATE); // specify minimum time for neutral to full in seconds
  }

  boolean manualMode = false;

  @Override
  public void periodic() {

    // if (!manualMode && isMovingIn()) {
    // zero();
    // }

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Position", getCurrentPosition());
    SmartDashboard.putNumber("Arm Target", getTargetPosition());
    // SmartDashboard.putNumber("Arm Error", talon.getClosedLoopError());
    // SmartDashboard.putNumber("Arm Error 2", Math.abs(getCurrentPosition() -
    // getTargetPosition()));
    // SmartDashboard.putBoolean("Arm At Position", isAtPosition());
  }

  public double getCurrentPosition() {
    return leftMotor.getSelectedSensorPosition();
  }

  public double getCurrentPositionInTicks() {
    return getCurrentPosition();
  }

  public double getTargetPosition() {
    if (leftMotor.getControlMode() == ControlMode.Position) {
      return leftMotor.getClosedLoopTarget();
    }
    return 0.0;
  }

  boolean isMovingIn() {
    return getTargetPosition() + 5000 < getCurrentPosition();
  }

  double m_targetPosition;

  public void setInTicks(double p) {
    m_targetPosition = p;
    manualMode = false;
    leftMotor.set(ControlMode.MotionMagic, p);
  }

  public boolean isAtPosition(double tolerance) {
    return Math.abs(getCurrentPosition() - m_targetPosition) < 5 *
        tolerance;
  }

  public void stop() {
    manualMode = true;
    leftMotor.set(ControlMode.PercentOutput, 0.0);
  }

  // Set the arm to horizontal and then call zero().
  public void zero() {
    // DriverStation.reportWarning("Arm: zero", false);
    leftMotor.setSelectedSensorPosition(0.0);
    leftMotor.set(ControlMode.PercentOutput, 0.0);

    rightMotor.setSelectedSensorPosition(0.0);
    rightMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void setPower(double d) {
    manualMode = true;
    leftMotor.set(ControlMode.PercentOutput, d);
    rightMotor.set(ControlMode.PercentOutput, d);
  }

  public void setLeftPower(double d) {
    manualMode = true;
    leftMotor.set(ControlMode.PercentOutput, d);
  }

  public void setRightPower(double d) {
    manualMode = true;
    rightMotor.set(ControlMode.PercentOutput, d);
  }

}
