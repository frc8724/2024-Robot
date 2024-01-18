package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.*;

public class MayhemTalonFX extends TalonFX {

    public enum CurrentLimit {
        HIGH_CURRENT, LOW_CURRENT
    }

    ControlMode controlMode;

    public MayhemTalonFX(int deviceNumber, CurrentLimit currentLimit) {
        super(deviceNumber);

        this.configFactoryDefault();

        this.configNominalOutputForward(0.0);
        this.configNominalOutputReverse(0.0);
        this.configPeakOutputForward(1.0);
        this.configPeakOutputReverse(-1.0);

        this.configVoltageCompSaturation(12.0); // "full output" scaled to 12.0V for all modes when enabled.
        this.enableVoltageCompensation(true); // turn on voltage compensation

        this.setNeutralMode(NeutralMode.Coast);

        if (currentLimit == CurrentLimit.HIGH_CURRENT) {
            this.configSupplyCurrentLimit(
                    new SupplyCurrentLimitConfiguration(
                            true,
                            40,
                            60,
                            1.0));

        } else if (currentLimit == CurrentLimit.LOW_CURRENT) {
            this.configSupplyCurrentLimit(
                    new SupplyCurrentLimitConfiguration(
                            true,
                            30,
                            45,
                            1.0));
        }
    }

    public void changeControlMode(ControlMode mode) {
        controlMode = mode;
    }

    public void set(int deviceID) {
        this.set(controlMode, deviceID);
    }

    public void setFeedbackDevice(FeedbackDevice feedback) {
        this.configSelectedFeedbackSensor(feedback, 0, 0);
    }

    public void configNominalOutputVoltage(float f, float g) {
        this.configNominalOutputForward(f / 12.0, 0);
        this.configNominalOutputReverse(g / 12.0, 0);
    }

    public void configPeakOutputVoltage(double d, double e) {
        this.configPeakOutputForward(d / 12.0, 0);
        this.configPeakOutputReverse(e / 12.0, 0);

    }

    // public double getError() {
    // return this.getClosedLoopError(0);
    // }

    public void setPosition(int zeroPositionCount) {
        this.setSelectedSensorPosition(zeroPositionCount, 0, 0);
    }

    // public int getPosition() {
    // return this.getSelectedSensorPosition(0);
    // }

    public double getSpeed() {
        return this.getSelectedSensorVelocity(0);
    }

    public void setEncPosition(int i) {
        setPosition(i);
    }
}
