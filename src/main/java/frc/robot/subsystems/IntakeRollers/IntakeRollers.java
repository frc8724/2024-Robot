// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeRollers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.IMayhemTalonFX;

public class IntakeRollers extends SubsystemBase {
  IMayhemTalonFX topRollerMotor;
  // IMayhemTalonFX bottomRollerMotor;

  /** Creates a new IntakeRollers. */
  public IntakeRollers(IMayhemTalonFX top) {
    topRollerMotor = top;
    topRollerMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double d) {
    topRollerMotor.set(ControlMode.PercentOutput, d);
  }

}
