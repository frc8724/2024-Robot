package frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.IMayhemTalonFX;

public class ShooterWheels extends SubsystemBase {
    IMayhemTalonFX leftMotor;
    IMayhemTalonFX rightMotor;

    private final double TALON_TICKS_PER_REV = 2048.0;
    private final double SECONDS_PER_MINUTE = 60.0;
    private final double HUNDRED_MS_PER_SECOND = 10.0;

    public static final double CLOSE_SHOT = 1125.0;
    public static final double MAX_SPEED_RPM = 5000;
    public static final double SPEED_TOLERANCE = 250;

    public double m_targetSpeedRPM;

    public ShooterWheels(IMayhemTalonFX left, IMayhemTalonFX right) {
        leftMotor = left;
        rightMotor = right;
        left.setInverted(true);
        right.setInverted(true);
        // right.follow(left);

        configureOneWheelFalcon(left);
        configureOneWheelFalcon(right);
        ;
        setShooterSpeedVBus(0.0);
    }

    // Note: for ease of thinking, 1 RPM =~ 3.4 native units for the shooter
    double convertRpmToTicksPer100ms(double rpm) {
        return rpm / SECONDS_PER_MINUTE * TALON_TICKS_PER_REV / HUNDRED_MS_PER_SECOND;
    }

    // Note: 3.413 native units =~ 1.0 RPM for the shooter
    double convertTicksPer100msToRPM(double ticks) {
        return ticks * HUNDRED_MS_PER_SECOND / TALON_TICKS_PER_REV * SECONDS_PER_MINUTE;
    }

    private void configureOneWheelFalcon(IMayhemTalonFX shooterWheelFalcon) {
        shooterWheelFalcon.setFeedbackDevice(FeedbackDevice.IntegratedSensor);
        shooterWheelFalcon.setNeutralMode(NeutralMode.Coast);
        shooterWheelFalcon.configNominalOutputVoltage(+0.0f, 0.0);
        shooterWheelFalcon.configPeakOutputVoltage(+12.0, -12.0);
        shooterWheelFalcon.configNeutralDeadband(0.001, 0); // Config neutral deadband
        // to be the smallest possible

        // For PIDF computations, 1023 is interpreted as "full" motor output
        // Velocity is in native units of TicksPer100ms.
        // 1000rpm =~ 3413 native units.
        // P of "3.0" means that full power applied with error of 341 native units =
        // 100rpm
        // (above also means that 50% power boost applied with error of 50rpm)
        // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#calculating-velocity-feed-forward-gain-kf
        // shooterWheelFalcon.config_kP(0, 6.0, 0); // if we are 100 rpm off, then apply
        // 10% more output = 10% * 1023 / 100
        // rpm
        shooterWheelFalcon.config_kP(0, 0.0, 0);
        shooterWheelFalcon.config_kI(0, 0.0, 0);
        shooterWheelFalcon.config_kD(0, 0.0, 0); // CTRE recommends starting at 10x P-gain
        shooterWheelFalcon.config_kF(0, 1023.0 * 0.25 / 1560, 0); // at 0.25 Percent VBus, the
                                                                  // shooter is at 1560
        shooterWheelFalcon.configAllowableClosedloopError(0, 0, 0); // no "neutral" zone around target
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateDashboard();
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("Shooter Wheel RPM",
                convertTicksPer100msToRPM(leftMotor.getSelectedSensorVelocity(0)));

        SmartDashboard.putNumber("Shooter Wheel target RPM", m_targetSpeedRPM);
        // SmartDashboard.putNumber("Shooter Wheel Error",
        // m_targetSpeedRPM -
        // convertTicksPer100msToRPM(shooterWheel.getSelectedSensorVelocity(0)));

        SmartDashboard.putBoolean("shooter too slow", this.isShooterTooSlow());
        SmartDashboard.putBoolean("shooter too fast", this.isShooterTooFast());
    }

    /**
     * Set shooter to rpm speed.
     * 
     * @param ticksPer100ms
     */
    public void setShooterSpeed(double ticksPer100ms) {
        if (ticksPer100ms < 0)
            ticksPer100ms = 0;

        m_targetSpeedRPM = ticksPer100ms;
        // System.out.println("setShooterSpeed: " + ticksPer100ms);
        if (ticksPer100ms > 50) {
            leftMotor.setNeutralMode(NeutralMode.Coast);
            rightMotor.setNeutralMode(NeutralMode.Coast);
            leftMotor.set(ControlMode.Velocity, ticksPer100ms);
            rightMotor.set(ControlMode.Velocity, ticksPer100ms);
        } else {
            setShooterSpeedVBus(0);
        }
    }

    public void setBrakeMode() {
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setShooterSpeedVBus(double pos) {
        leftMotor.set(ControlMode.PercentOutput, pos);
        rightMotor.set(ControlMode.PercentOutput, pos);
    }

    public double getShooterSpeed() {
        return leftMotor.getSelectedSensorVelocity(0);
    }

    public double getShooterTargetSpeed() {
        // System.out.println("getShooterTargetSpeed: " + m_targetSpeedRPM);
        return m_targetSpeedRPM;
    }

    public boolean isShooterAtSpeed() {
        if (m_targetSpeedRPM < 100)
            return false;

        // the shooter has trouble locking into a low speed. Double the tolerance
        if (m_targetSpeedRPM < 600) {
            return Math.abs(m_targetSpeedRPM - getShooterSpeed()) < SPEED_TOLERANCE * 2;
        } else {
            return Math.abs(m_targetSpeedRPM - getShooterSpeed()) < SPEED_TOLERANCE;
        }
    }

    public boolean isShooterTooSlow() {
        if (m_targetSpeedRPM < 100)
            return true;

        if (getShooterSpeed() + SPEED_TOLERANCE < m_targetSpeedRPM) {
            return true;
        }

        return false;
    }

    public boolean isShooterTooFast() {
        if (getShooterSpeed() - SPEED_TOLERANCE > m_targetSpeedRPM) {
            return true;
        }

        return false;
    }

    public boolean isShooterWarmingUp() {
        // the shooter has trouble locking into a low speed. Double the tolerance
        return (m_targetSpeedRPM > 200);
    }

    public double getShooterSpeedVBus() {
        return leftMotor.getMotorOutputVoltage();
    }
}
