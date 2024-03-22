// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.motors;

import com.ctre.phoenix.motorcontrol.IFollower;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/** Add your docs here. */
public interface IMayhemCANSparkMax {
    CANSparkMax getMotor();
    void setInverted(boolean b);
    void follow(CANSparkMax m);
    void setVBusPower(double d);
}
