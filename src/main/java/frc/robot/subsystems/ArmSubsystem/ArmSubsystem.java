// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.IMayhemTalonFX;

public class ArmSubsystem extends SubsystemBase {

  public static final double ZERO_POSITION = -210000;
  public static final double NOTE_INTAKE = -160000;

  public static final double POSITION_SLOP = 10000.0;

  final double kWheelP = 0.02; // 0.015;
  final double kWheelI = 0.000;
  final double kWheelD = 0.000;
  final double kWheelF = 0.000;

  private static final double CLOSED_LOOP_RAMP_RATE = 0.01; // time from neutral to full in seconds

  double TargetPositionTicks;

  IMayhemTalonFX leftMotor;
  IMayhemTalonFX rightMotor;

  /** Creates a new Shoulder. */
  public ArmSubsystem(IMayhemTalonFX left, IMayhemTalonFX right) {
    leftMotor = left;
    rightMotor = right;
    // leftMotor.configFactoryDefault();
    // rightMotor.configFactoryDefault();

    configTalon(leftMotor);
    // configTalon(rightMotor);

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);

    leftMotor.setSensorPhase(false);
    rightMotor.setSensorPhase(false);
      
    rightMotor.follow(leftMotor);

    // configureDriveTalon(rightMotor);
    configureDriveTalon(leftMotor);

    setZeroArm();
  }

  private void configTalon(IMayhemTalonFX talon) {
    talon.setNeutralMode(NeutralMode.Coast);
  }

  // Ideas to tune the Shoulder.
  // 0. Zero the Shoulder so that 0 degrees is horizontal.
  // 1. Rotate the Shoulder to horizontal (0 degrees). Note the talon ticks.
  // 2. Rotate the Shoulder to vertical (90 degrees). Calculate the
  // TICKS_PER_DEGREE.
  // 3. Command the Shoulder to go to 0.
  // 4. Update wheelF until it is stable at 0 degrees.

  private void configureDriveTalon(final IMayhemTalonFX talon) {

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
    talon.configAllowableClosedloopError(slot, 5000, timeout);

    talon.configClosedLoopPeakOutput(slot, 0.5);

    talon.configMotionCruiseVelocity(40000); // measured velocity of ~100K at 85%; set cruise to that
    talon.configMotionAcceleration(30000); // acceleration of 2x velocity allows cruise to be attained in 1 second
                                           // second
    // talon.set(TalonFXControlMode.Position, 0.0);
    talon.set(ControlMode.Position, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     SmartDashboard.putNumber("Shoulder left Ticks",
      leftMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Shoulder right Ticks",
      rightMotor.getSelectedSensorPosition());
    // if (rightTalon.getControlMode() != ControlMode.PercentOutput) {
    //   SmartDashboard.putNumber("Shoulder Target Ticks",
    //       rightTalon.getClosedLoopTarget());
    // }

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

  public void moveArm(double outputPercentage) {
    SmartDashboard.putNumber("Controller X input", outputPercentage);
  }

  public void setAngleInTicks(double ticks) {
    TargetPositionTicks = ticks;
    // rightTalon.set(ControlMode.MotionMagic, ticks, DemandType.ArbitraryFeedForward, 0.05);
    leftMotor.set(ControlMode.Position, ticks);
  }

  public double getCurrentPositionInTicks() {
    return leftMotor.getSelectedSensorPosition();
  }

  public double getTargetPositionTicks() {
    if (leftMotor.getControlMode() != ControlMode.PercentOutput) {
      return leftMotor.getClosedLoopTarget();
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

  
  public void setZeroArm(){
    leftMotor.setSelectedSensorPosition(ZERO_POSITION);   
    rightMotor.setSelectedSensorPosition(ZERO_POSITION);
  }

  public void setPower(double power) {;
    leftMotor.set(ControlMode.PercentOutput, power);   
    // rightMotor.set(ControlMode.PercentOutput, power);
  }

  public boolean isAbove(double x){
     return getCurrentPositionInTicks()>x;
  }
}

