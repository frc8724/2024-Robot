// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.motors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import frc.robot.motors.MayhemTalonFX.CurrentLimit;

/** Add your docs here. */
public class FakeFalconFX implements IMayhemTalonFX {
    public ErrorCode configMotionCruiseVelocity(double sensorUnitsPer100ms) {
        return ErrorCode.OK;
    }

    public ErrorCode configMotionAcceleration(double sensorUnitsPer100msPerSec) {
        return ErrorCode.OK;
    }

    public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull) {
        return ErrorCode.OK;
    }

    public double getSelectedSensorPosition() {
        return 0.0;
    }

    public double getClosedLoopTarget() {
        return 0.0;
    }

    public ErrorCode setSelectedSensorPosition(double sensorPos) {
        return ErrorCode.OK;
    }

    public ErrorCode config_kP(int slotIdx, double value) {
        return ErrorCode.OK;
    }

    public ErrorCode config_kI(int slotIdx, double value) {
        return ErrorCode.OK;
    }

    public ErrorCode config_kD(int slotIdx, double value) {
        return ErrorCode.OK;
    }

    public ErrorCode config_kF(int slotIdx, double value) {
        return ErrorCode.OK;
    }

    public FakeFalconFX(int id, frc.robot.motors.MayhemTalonFX.CurrentLimit limit) {
    }

    public void set(double d) {
    }

    public void configNominalOutputVoltage(float f, float g) {
    }

    public void configPeakOutputVoltage(double d, double e) {
    }

    public void setPosition(int zeroPositionCount) {
    }

    public double getSpeed() {
        return 0.0;
    }

    public void setEncPosition(int i) {
    }

    public void setSmartCurrentLimit(double d) {
    }

    public void changeControlMode(ControlMode mode) {
    }

    public void setFeedbackDevice(FeedbackDevice feedback) {
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    // @Override
    public ErrorCode configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    public ErrorCode configSelectedFeedbackSensor(TalonFXFeedbackDevice f, int i, int j) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitCfg, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configVelocityMeasurementPeriod(SensorVelocityMeasPeriod period, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configVelocityMeasurementPeriod(VelocityMeasPeriod period, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configVelocityMeasurementWindow(int windowSize, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getOutputCurrent() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public int getStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode changeMotionControlFramePeriod(int periodMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode clearMotionProfileHasUnderrun(int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode clearMotionProfileTrajectories() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode clearStickyFaults(int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configAllowableClosedloopError(int slotIdx, double allowableCloseLoopError, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configAuxPIDPolarity(boolean invert, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configClosedLoopPeakOutput(int slotIdx, double percentOut, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int deviceID, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configForwardSoftLimitEnable(boolean enable, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configForwardSoftLimitThreshold(double forwardSensorLimit, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public int configGetCustomParam(int paramIndex, int timeoutMs) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double configGetParameter(ParamEnum paramEnum, int ordinal, int timeoutMs) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double configGetParameter(int paramEnum, int ordinal, int timeoutMs) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public ErrorCode configMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configMotionAcceleration(double sensorUnitsPer100msPerSec, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configMotionCruiseVelocity(double sensorUnitsPer100ms, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configMotionProfileTrajectoryPeriod(int baseTrajDurationMs, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configMotionSCurveStrength(int curveStrength, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configNeutralDeadband(double percentDeadband, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configNominalOutputForward(double percentOut, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configNominalOutputReverse(double percentOut, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configPeakOutputForward(double percentOut, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configPeakOutputReverse(double percentOut, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(CANCoder canCoderRef, int remoteOrdinal, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(BaseTalon talonRef, int remoteOrdinal, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource, int remoteOrdinal,
            int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int deviceID, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configReverseSoftLimitEnable(boolean enable, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configReverseSoftLimitThreshold(double reverseSensorLimit, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configSelectedFeedbackCoefficient(double coefficient, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configSetCustomParam(int newValue, int paramIndex, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configSetParameter(int param, double value, int subValue, int ordinal, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode config_IntegralZone(int slotIdx, double izone, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode config_kD(int slotIdx, double value, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode config_kF(int slotIdx, double value, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode config_kI(int slotIdx, double value, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode config_kP(int slotIdx, double value, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void enableVoltageCompensation(boolean enable) {
        // TODO Auto-generated method stub

    }

    @Override
    public double getActiveTrajectoryPosition() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getActiveTrajectoryVelocity() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public int getBaseID() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getBusVoltage() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getClosedLoopError(int pidIdx) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getClosedLoopTarget(int pidIdx) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public ControlMode getControlMode() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public int getDeviceID() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getErrorDerivative(int pidIdx) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public ErrorCode getFaults(Faults toFill) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public int getFirmwareVersion() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getIntegralAccumulator(int pidIdx) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean getInverted() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public ErrorCode getLastError() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode getMotionProfileStatus(MotionProfileStatus statusToFill) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public int getMotionProfileTopLevelBufferCount() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getMotorOutputPercent() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getMotorOutputVoltage() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getSelectedSensorPosition(int pidIdx) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getSelectedSensorVelocity(int pidIdx) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public int getStatusFramePeriod(StatusFrame frame, int timeoutMs) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public ErrorCode getStickyFaults(StickyFaults toFill) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getTemperature() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean hasResetOccurred() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean isMotionProfileTopLevelBufferFull() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void neutralOutput() {
        // TODO Auto-generated method stub

    }

    @Override
    public void overrideLimitSwitchesEnable(boolean enable) {
        // TODO Auto-generated method stub

    }

    @Override
    public void overrideSoftLimitsEnable(boolean enable) {
        // TODO Auto-generated method stub

    }

    @Override
    public void processMotionProfileBuffer() {
        // TODO Auto-generated method stub

    }

    @Override
    public ErrorCode pushMotionProfileTrajectory(TrajectoryPoint trajPt) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void selectProfileSlot(int slotIdx, int pidIdx) {
        // TODO Auto-generated method stub

    }

    @Override
    public void set(ControlMode Mode, double demand) {
        // TODO Auto-generated method stub

    }

    @Override
    public void set(ControlMode Mode, double demand0, DemandType demand1Type, double demand1) {
        // TODO Auto-generated method stub

    }

    @Override
    public ErrorCode setControlFramePeriod(ControlFrame frame, int periodMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode setIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void setInverted(boolean invert) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setInverted(InvertType invertType) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        // TODO Auto-generated method stub

    }

    @Override
    public ErrorCode setSelectedSensorPosition(double sensorPos, int pidIdx, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void setSensorPhase(boolean PhaseSensor) {
        // TODO Auto-generated method stub

    }

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrame frame, int periodMs, int timeoutMs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void follow(IMotorController masterToFollow) {
        // TODO Auto-generated method stub

    }

    @Override
    public void valueUpdated() {
        // TODO Auto-generated method stub

    }

    @Override
    public void configNominalOutputVoltage(double f, double g) {
        // TODO Auto-generated method stub

    }

    @Override
    public ErrorCode configPeakOutputForward(double d) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ErrorCode configPeakOutputReverse(double d) {
        // TODO Auto-generated method stub
        return null;

    }

    @Override
    public ErrorCode configNominalOutputForward(double d) {
        // TODO Auto-generated method stub
        return null;

        // throw new UnsupportedOperationException("Unimplemented method
        // 'configNominalOutputForward'");
    }

    @Override
    public ErrorCode configNominalOutputReverse(double d) {
        // TODO Auto-generated method stub
        return null;

        // throw new UnsupportedOperationException("Unimplemented method
        // 'configNominalOutputReverse'");
    }

    @Override
    public ErrorCode configForwardSoftLimitEnable(boolean b) {
        // TODO Auto-generated method stub
        return null;

        // throw new UnsupportedOperationException("Unimplemented method
        // 'configForwardSoftLimitEnable'");
    }

    @Override
    public ErrorCode configReverseSoftLimitEnable(boolean b) {
        // TODO Auto-generated method stub
        return null;

        // throw new UnsupportedOperationException("Unimplemented method
        // 'configReverseSoftLimitEnable'");
    }

    @Override
    public ErrorCode configClosedLoopPeakOutput(int slot, double d) {
        // TODO Auto-generated method stub
        return null;

        // throw new UnsupportedOperationException("Unimplemented method
        // 'configClosedLoopPeakOutput'");
    }
}