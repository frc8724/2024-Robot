// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SimpleFalconSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

public class SwerveTurningFalcon extends SubsystemBase {
  private TalonFX motor;
  private String name;
  static final double MOTOR_TICKS_PER_ROTATION = 2048.0;
  static  final double MOTOR_RATIO_TO_WHEEL = 150.0 / 7.0;
  public static final double MOTOR_TICKS_PER_WHEEL_ROTATION = MOTOR_TICKS_PER_ROTATION *
      MOTOR_RATIO_TO_WHEEL;

  private PositionVoltage m_angleSetter = new PositionVoltage(0);

  /** Creates a new SimpleFalconSubsystem. */
  public SwerveTurningFalcon(String name, int id, boolean invert) {
    motor = new TalonFX(id);
    motor.setInverted(invert);
    this.name = name;
    
    motor.getConfigurator().apply(new TalonFXConfiguration());
    //motor.setPosition(0);

    // motor.config_kP(0, 0.5);
    // motor.config_kI(0, 0.0);
    // motor.config_kD(0, 0.5);
    // motor.config_kF(0, 0.0);
    // var slot0Configs = new Slot0Configs();
    // slot0Configs.kP =0.5;
    // slot0Configs.kI =0.0;
    // slot0Configs.kD =0.5;
    //slot0Configs.kF =0.0;

    

    //motor.getConfigurator().apply(slot0Configs, 0.050);



    // motor.configNominalOutputForward(0.0);
    // motor.configNominalOutputReverse(0.0);
    // motor.configPeakOutputForward(+12.0);
    // motor.configPeakOutputReverse(-12.0);
    // motor.configNeutralDeadband(0.0);
    // motor.setNeutralMode(NeutralMode.Coast);
    // Timer.delay(0.1);

    var talonfxConfig = new TalonFXConfiguration();
    talonfxConfig.Slot0.kS = 0.0;
    talonfxConfig.Slot0.kV = 0.0;
    talonfxConfig.Slot0.kP = 5.0;
    talonfxConfig.Slot0.kI = 0.0;
    talonfxConfig.Slot0.kD = 0.1; // was 0.05
    talonfxConfig.ClosedLoopGeneral.ContinuousWrap = true;
    talonfxConfig.Voltage.PeakForwardVoltage = 8;
    talonfxConfig.Voltage.PeakReverseVoltage = -8;
    talonfxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonfxConfig.TorqueCurrent.PeakForwardTorqueCurrent = 100.0;
    talonfxConfig.TorqueCurrent.PeakReverseTorqueCurrent = 100.0;
    //talonfxConfig.MotorOutput.Inverted = invert;
    motor.getConfigurator().apply(talonfxConfig);
    Timer.delay(0.5);
    // motor.setPosition(10);
    
    motor.setPosition(0);

    System.out.println("Turning Init");
  }

  double m_set;

  double convertRadiansToTicks(double rads) {
    return rads * MOTOR_TICKS_PER_WHEEL_ROTATION / (2 * Math.PI);

    // double currentTicks = motor.getSelectedSensorPosition() %
    // MOTOR_TICKS_PER_WHEEL_ROTATION;
    // double desiredTicks = rads * MOTOR_TICKS_PER_WHEEL_ROTATION / (2 * Math.PI);

    // if( currentTicks - desiredTicks > MOTOR_TICKS_PER_WHEEL_ROTATION / 2){

    // }
  }

  final double twoPi = Math.PI * 2.0;

  /**
   * Give a source radian and a target radian, return the radian difference to
   * add to the source to get to the target, possibly passing through pi/-pi.
   * 
   * @param source
   * @param target
   * @return
   */
  public double shortestRotation(double source, double target) {
    // double sourceMod = source < 0 ? twoPi + (source % twoPi) : (source % twoPi);
    // double targetMod = target < 0 ? twoPi + (target % twoPi) : (target % twoPi);

    double sourceMod = source % twoPi;
    double targetMod = target % twoPi;

    if (sourceMod < 0.0) {
      sourceMod += twoPi;
    }
    if (targetMod < 0.0) {
      targetMod += twoPi;
    }

    if (sourceMod > twoPi) {
      sourceMod -= twoPi;
    }
    if (targetMod > twoPi) {
      targetMod -= twoPi;
    }

    double rotation = targetMod - sourceMod;

    if (rotation > Math.PI) {
      return rotation - twoPi;
    } else if (rotation < -Math.PI) {
      return twoPi + rotation;
    } else {
      return rotation;
    }
  }

  private double placeInAppropriate0To2PiScope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double startingRadianNeg2Pito2Pi = scopeReference % twoPi;
    if (startingRadianNeg2Pito2Pi >= 0) {
      lowerBound = scopeReference - startingRadianNeg2Pito2Pi;
      upperBound = scopeReference + (twoPi - startingRadianNeg2Pito2Pi);
    } else {
      upperBound = scopeReference - startingRadianNeg2Pito2Pi;
      lowerBound = scopeReference - (twoPi + startingRadianNeg2Pito2Pi);
    }
    while (newAngle < lowerBound) {
      newAngle += twoPi;
    }
    while (newAngle > upperBound) {
      newAngle -= twoPi;
    }
    if (newAngle - scopeReference > Math.PI) {
      newAngle -= twoPi;
    } else if (newAngle - scopeReference < -Math.PI) {
      newAngle += twoPi;
    }
    return newAngle;
  }

  private final PositionVoltage m_voltagePosition = new PositionVoltage(
    0, 0, true, 0, 0, 
    false, false, false);
 
  /**
   * value is from -pi to +pi. In order to ensure
   * smoother rotation we check if we are crossing pi
   * and do the math to find the shortest path
   * 
   * @param value radians
   */
  public void set(double value) {
    // System.out.println("Turning: set.");
    // System.out.println("Desired rotation: " + value);
    // double currentRotation = this.getRotationRadians();
    // double rotation = this.shortestRotation(currentRotation, value);
    // double finalRotation = currentRotation + rotation;
    // double e = convertRadiansToTicks(finalRotation);
    // double s = motor.getPosition().getValueAsDouble() % MOTOR_TICKS_PER_WHEEL_ROTATION;

    // if (this.name == "frontLeftTurningMotor") {
    //   SmartDashboard.putNumber(this.name + " desired radians", value);
    //   SmartDashboard.putNumber(this.name + " current radians", currentRotation);
    //   SmartDashboard.putNumber(this.name + " rotation mod", rotation);
    //   SmartDashboard.putNumber(this.name + " shortest radians", finalRotation);
    //   SmartDashboard.putNumber(this.name + " motor position", motor.getPosition().getValueAsDouble());
    //   // System.out.println("rotation: " + rotation);
    //   // System.out.println("final rot: " + finalRotation);
    //   // System.out.println("final Tick: " + e);
    //   // System.out.println("motor curr ticks: " + motor.getSelectedSensorPosition());
    //   // System.out.println("==============================");
    // }
    // double ticks;
    // if (e - s + MOTOR_TICKS_PER_WHEEL_ROTATION < s - e) {
    //   ticks = motor.getPosition().getValueAsDouble() + e - s + MOTOR_TICKS_PER_WHEEL_ROTATION;
    // } else {
    //   ticks = motor.getPosition().getValueAsDouble() - (s - e);
      
    // }
   
    // System.out.println("Motor Ticks to set to: " + ticks);
    // motor.setControl(new PositionDutyCycle(ticks));
    // motor.set(0.1); // motors turned.
    // motor.setControl(new PositionVoltage(20.0)); // motor jiggle, but not move.
    // motor.setControl(new VelocityDutyCycle(0.00)); // loud jiggle
    // motor.setControl(new PositionVoltage(1000.0));
    // double d = motor.getPosition().getValueAsDouble();
    motor.setControl(m_voltagePosition.withPosition(value));
    SmartDashboard.putNumber("Turn Rotation", value);
    
    //motor.setPosition(ticks);
    //motor.setControl(m_angleSetter.withPosition(finalRotation));
    // m_set = e;
  }

  public double getRotationRadians() {
    double limitedSensorPosition = motor.getPosition().getValueAsDouble() % (MOTOR_TICKS_PER_WHEEL_ROTATION);
    return limitedSensorPosition / MOTOR_TICKS_PER_WHEEL_ROTATION * 2 * Math.PI;
  }

  public double getRotationTicks() {
    return motor.getPosition().getValueAsDouble();
  }

  public void reset() {
    set(0.0);
    motor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    SmartDashboard.putNumber(this.name + " position", motor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(this.name + " error", motor.getClosedLoopError().getValueAsDouble());
    // SmartDashboard.putNumber(this.name + " MOTOR_TICKS_PER_WHEEL_ROTATION",
    // MOTOR_TICKS_PER_WHEEL_ROTATION);
    // SmartDashboard.putNumber(this.name + " rads", this.getRotationRadians());
    // SmartDashboard.putNumber(this.name + " m_set", m_set);
    // SmartDashboard.putNumber("test shortestRotation 3/4*PI to -3/4*PI",
    // shortestRotation(Math.PI * 3.0 / 4.0, -Math.PI * 3.0 / 4.0)); // should be
    // +Pi/2 = 1.57

    // SmartDashboard.putNumber("test shortestRotation -3/4*PI to 3/4*PI",
    // shortestRotation(-Math.PI * 3.0 / 4.0, Math.PI * 3.0 / 4.0)); // should be
    // -Pi/2 = 1.57

  }

  public void setSensorPosition(double encoderTicks) {
    // TODO Auto-generated method stub
    motor.setPosition(encoderTicks);
    motor.setControl(new PositionDutyCycle(encoderTicks));
  }
}
