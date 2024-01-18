// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.MayhemTalonFX;
import frc.robot.subsystems.MayhemTalonFX.CurrentLimit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {

  // static final double TICKS_PER_INCH = 3442;
  public static final double[] LEVEL_X_SCORE = { 0.0, 2000.0, 45000.0, 119000.0 };
  public static final double[] LEVEL_X_SCORE_Cube = { 0.0, 2000.0, 32000.0, 80000.0 };

  public static final double HUMAN_PLAYER_STATION = 20000.0;
  public static final double ALMOST_STOW = 500.0;
  public static final double FLOOR_PICKUP = 35000.0;
  public static final double FLOOR_PICKUP_BACK = 36700;

  public static final double POSITION_SLOP = 1000.0;
  static final double CLOSED_LOOP_RAMP_RATE = 1.0; // todo: lower this value

  private final MayhemTalonFX talon = new MayhemTalonFX(SwerveConstants.DriveConstants.ARM_FALCON, CurrentLimit.HIGH_CURRENT);

  /** Creates a new Arm. */
  public ClimberSubsystem() {
    configTalon(talon);
    zero();
  }

  private void configTalon(TalonFX talon) {
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

    if ( !manualMode && isMovingIn()) {
      zero();
    }

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Position", getCurrentPosition());
    SmartDashboard.putNumber("Arm Target", getTargetPosition());
    // SmartDashboard.putNumber("Arm Error", talon.getClosedLoopError());
    // SmartDashboard.putNumber("Arm Error 2", Math.abs(getCurrentPosition() -
    // getTargetPosition()));
    // SmartDashboard.putBoolean("Arm At Position", isAtPosition());
  }

 

  public double getCurrentPosition() {
    return talon.getSelectedSensorPosition();
  }

  public double getCurrentPositionInTicks() {
    return getCurrentPosition();
  }

  public double getTargetPosition() {
    if (talon.getControlMode() == ControlMode.Position) {
      return talon.getClosedLoopTarget();
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
    talon.set(ControlMode.MotionMagic, p);
  }

  public boolean isAtPosition(double tolerance) {
    return Math.abs(getCurrentPosition() - m_targetPosition) < 5 *
        tolerance;
  }

  public void stop() {
    manualMode = true;
    talon.set(ControlMode.PercentOutput, 0.0);
  }

  // Set the arm to horizontal and then call zero().
  public void zero() {
    // DriverStation.reportWarning("Arm: zero", false);
    talon.setSelectedSensorPosition(0.0);
    talon.set(ControlMode.PercentOutput, 0.0);
  }

  public void setPower(double d) {
    manualMode = true;
    talon.set(ControlMode.PercentOutput, d);
  }

  public void setAutoPower(double d) {
    manualMode = false;
    talon.set(ControlMode.PercentOutput, d);
  }
}

