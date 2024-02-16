package frc.robot.motors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.revrobotics.REVLibError;

public interface IMayhemTalonFX extends IMotorControllerEnhanced {
    ErrorCode configSelectedFeedbackSensor(TalonFXFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs);

    ErrorCode configMotionCruiseVelocity(double sensorUnitsPer100ms);

    ErrorCode configMotionAcceleration(double sensorUnitsPer100msPerSec);

    ErrorCode configAllowableClosedloopError(int slotIdx, double allowableClosedLoopError, int timeoutMs);

    ErrorCode configClosedloopRamp(double secondsFromNeutralToFull);

    double getClosedLoopTarget();

    ErrorCode setSelectedSensorPosition(double sensorPos);
    // REVLibError setSmartCurrentLimit(int limit);

    double getSelectedSensorPosition();

    ErrorCode config_kP(int slotIdx, double value);

    ErrorCode config_kI(int slotIdx, double value);

    ErrorCode config_kD(int slotIdx, double value);

    ErrorCode config_kF(int slotIdx, double value);

    void configPeakOutputVoltage(double d, double e);

    void configNominalOutputVoltage(double f, double g);

    void setFeedbackDevice(FeedbackDevice integratedsensor);

    void setPosition(int zeroPositionCount) ;

    ErrorCode configPeakOutputForward(double d);

    ErrorCode configPeakOutputReverse(double d);

    ErrorCode configNominalOutputForward(double d);

    ErrorCode configNominalOutputReverse(double d);

    ErrorCode configForwardSoftLimitEnable(boolean b);

    ErrorCode configReverseSoftLimitEnable(boolean b);

    ErrorCode configClosedLoopPeakOutput(int slot, double d);
}