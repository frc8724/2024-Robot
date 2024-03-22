// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class FakeMayhemCANSparkMax implements IMayhemCANSparkMax {
    public FakeMayhemCANSparkMax(int id, MotorType type){
    }

    @Override
    public void setInverted(boolean b) {}

    @Override
    public CANSparkMax getMotor() {
        return null;
    }

    @Override
    public void follow(CANSparkMax m) {}

    @Override
    public void setVBusPower(double d) {}
}
