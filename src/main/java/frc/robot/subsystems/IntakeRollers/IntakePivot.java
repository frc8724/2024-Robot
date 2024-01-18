// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeRollers;

import java.lang.annotation.Target;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;

public class IntakePivot extends SubsystemBase {
  public static final double[] LEVEL_X_PRESCORE = { 0.0, 2000.0, 75000.0, 82500.0 };
  public static final double[] LEVEL_X_SCORE = { 0.0, 2000.0, 53000.0, 74000.0 };
  public static final double[] LEVEL_X_SCORE_CUBE = { 0.0, 2000.0, 57000.0, 76000.0 };

  public static final double HUMAN_PLAYER_STATION = 74000.0;
  public static final double STOW = 9000.0;
  public static final double FLOOR_PICKUP = 17000;
  public static final double CONE_STOW = 14000;
  public static final double SCORE_TOLERANCE = 40000;
  public static final double STRAIGHT_UP = 100000;
  public static final double HUMAN_PLAYER_STATION_BACK = 190000;
  public static final double BACK_ISH = 140000;
  public static final double FLOOR_PICKUP_BACK = 250000;

  public static final double POSITION_SLOP = 1000.0;

  final double kWheelP = 0.09; // 0.015;
  final double kWheelI = 0.000;
  final double kWheelD = 0.000;
  final double kWheelF = 0.000;

  private final TalonFX leftTalon = new TalonFX(SwerveConstants.DriveConstants.LEFT_SHOULDER_FALCON);
  private final TalonFX rightTalon = new TalonFX(SwerveConstants.DriveConstants.RIGHT_SHOULDER_FALCON);
  private static final double CLOSED_LOOP_RAMP_RATE = 0.01; // time from neutral to full in seconds

  /** Creates a new Shoulder. */
  public IntakePivot() {
    leftTalon.configFactoryDefault();
    rightTalon.configFactoryDefault();

    configTalon(leftTalon);
    configTalon(rightTalon);

    leftTalon.follow(rightTalon);
    leftTalon.setInverted(true);
    rightTalon.setInverted(false);

    leftTalon.setSensorPhase(false);
    rightTalon.setSensorPhase(false);

    configureDriveTalon(rightTalon);
    // configureDriveTalon(leftTalon);

    zero();
  }

  private void configTalon(TalonFX talon) {
    talon.setNeutralMode(NeutralMode.Brake);
  }

  // Ideas to tune the Shoulder.
  // 0. Zero the Shoulder so that 0 degrees is horizontal.
  // 1. Rotate the Shoulder to horizontal (0 degrees). Note the talon ticks.
  // 2. Rotate the Shoulder to vertical (90 degrees). Calculate the
  // TICKS_PER_DEGREE.
  // 3. Command the Shoulder to go to 0.
  // 4. Update wheelF until it is stable at 0 degrees.

  private void configureDriveTalon(final TalonFX talon) {

    final int slot = 0;
    final int timeout = 100;

    talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, slot, timeout);
    talon.setSensorPhase(false);

    talon.config_kP(slot, kWheelP, timeout);
    talon.config_kI(slot, kWheelI, timeout);
    talon.config_kD(slot, kWheelD, timeout);
    talon.config_kF(slot, kWheelF, timeout);

    talon.configPeakOutputForward(1.0);
    talon.configPeakOutputReverse(-1.0);
    talon.configNominalOutputForward(0.0);
    talon.configNominalOutputReverse(0.0);

    talon.configClosedloopRamp(CLOSED_LOOP_RAMP_RATE); // specify minimum time for neutral to full in seconds
    talon.selectProfileSlot(slot, timeout);
    talon.configForwardSoftLimitEnable(false);
    talon.configReverseSoftLimitEnable(false);
    talon.configAllowableClosedloopError(slot, 50, timeout);

    talon.configClosedLoopPeakOutput(slot, 1.0);

    talon.configMotionCruiseVelocity(40000); // measured velocity of ~100K at 85%; set cruise to that
    talon.configMotionAcceleration(30000); // acceleration of 2x velocity allows cruise to be attained in 1 second
                                           // second
    talon.set(TalonFXControlMode.Position, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shoulder Current Ticks",
        rightTalon.getSelectedSensorPosition());
    if (rightTalon.getControlMode() != ControlMode.PercentOutput) {
      SmartDashboard.putNumber("Shoulder Target Ticks",
          rightTalon.getClosedLoopTarget());
    }

    // wheelP = SmartDashboard.getNumber("Shoulder P", kWheelP);
    // SmartDashboard.putNumber("Shoulder P", kWheelP);
    // // wheelI = SmartDashboard.getNumber("Shoulder I", kWheelI);
    // SmartDashboard.putNumber("Shoulder I", kWheelI);
    // // wheelD = SmartDashboard.getNumber("Shoulder D", kWheelD);
    // SmartDashboard.putNumber("Shoulder D", kWheelD);
    // // wheelF = SmartDashboard.getNumber("Shoulder F", kWheelF);
    // SmartDashboard.putNumber("Shoulder F", kWheelF);

    SmartDashboard.putBoolean("Shoulder at Position", isAtPosition());

    // SmartDashboard.putNumber("Shoulder error", rightTalon.getClosedLoopError());

    // SmartDashboard.putNumber("Shoulder Motor %",
    // rightTalon.getMotorOutputPercent());
  }

  double TargetPositionTicks;

  public void setAngleInTicks(double ticks) {
    TargetPositionTicks = ticks;
    rightTalon.set(ControlMode.MotionMagic, ticks, DemandType.ArbitraryFeedForward, 0.05);
  }

  public double getCurrentPositionInTicks() {
    return rightTalon.getSelectedSensorPosition();
  }

  public double getTargetPositionTicks() {
    if (rightTalon.getControlMode() != ControlMode.PercentOutput) {
      return rightTalon.getClosedLoopTarget();
    }
    return 0.0;
  }

  private boolean isAtPosition() {
    return isAtPosition(POSITION_SLOP);
  }

  public boolean isAtPosition(double tolerance) {
    return Math.abs(getCurrentPositionInTicks() - TargetPositionTicks) < tolerance;
  }

  public void stop() {
    setAngleInTicks(getCurrentPositionInTicks());
  }

  // Set the arm to horizontal and then call zero().
  public void zero() {
    // DriverStation.reportWarning("Shoulder: zero", false);
    rightTalon.setSelectedSensorPosition(0.0);
    rightTalon.set(TalonFXControlMode.Position, 0.0);

  }

  public void setPower(double power) {
    rightTalon.set(TalonFXControlMode.PercentOutput, power);
  }
}

