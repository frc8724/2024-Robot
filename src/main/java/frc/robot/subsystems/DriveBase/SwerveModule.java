package frc.robot.subsystems.DriveBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SimpleFalconSubsystem.SwerveDriveFalcon;
import frc.robot.subsystems.SimpleFalconSubsystem.SwerveTurningFalcon;

public class SwerveModule extends SubsystemBase {
    private final SwerveDriveFalcon m_driveMotor;
    private final SwerveTurningFalcon m_turningMotor;
    private final SwerveEncoder m_magEncoder;

    public SwerveModule(
            String driveMotorName,
            int driveMotorID,
            String turningMotorName,
            int turningMotorID,
            boolean driveReversed,
            boolean turningReversed,
            int magInput) {
        m_driveMotor = new SwerveDriveFalcon(driveMotorName, driveMotorID, driveReversed);
        m_turningMotor = new SwerveTurningFalcon(turningMotorName, turningMotorID, turningReversed);
        m_magEncoder = new SwerveEncoder(magInput);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveMotor.getRotationalVelocity(),
                new Rotation2d(m_turningMotor.getRotationRadians()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveMotor.getDistance(),
                new Rotation2d(m_turningMotor.getRotationRadians()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(m_turningMotor.getRotationRadians()));

        // Calculate the turning motor output from the turning PID controller.
        m_driveMotor.set(state.speedMetersPerSecond);
        m_turningMotor.set(state.angle.getRadians());
    }

    public static SwerveModuleState optimizeB(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngleDegrees = placeInAppropriate0To360Scope(currentAngle.getDegrees(),
                desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngleDegrees - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngleDegrees = delta > 90 ? (targetAngleDegrees -= 180) : (targetAngleDegrees += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngleDegrees));
    }

    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    final double MAG_MAX = 4096.0;
    final double WHEEL_MAX = 2048.0;

    public void zeroTurningWheel(double MagTickTarget) {
        double magTicks = m_magEncoder.get();
        double magRad = magTicks / MAG_MAX * (Math.PI * 2);
        double magTargetRad = MagTickTarget / MAG_MAX * (Math.PI * 2);

        double wheelRad = m_turningMotor.getRotationRadians();

        double diffRad = magTargetRad - magRad;

        // SmartDashboard.putNumber(this.m_magEncoder.m_analogInput.getChannel() + "
        // test magTicks", magTicks);
        // SmartDashboard.putNumber(this.m_magEncoder.m_analogInput.getChannel() + "
        // test diffRad", diffRad);
        // SmartDashboard.putNumber(this.m_magEncoder.m_analogInput.getChannel() + "
        // test magRad ", magRad);
        // SmartDashboard.putNumber(this.m_magEncoder.m_analogInput.getChannel() + "
        // test magTargetRad", magTargetRad);
        // SmartDashboard.putNumber(this.m_magEncoder.m_analogInput.getChannel() + "
        // test wheelRad ", wheelRad);

        m_turningMotor.set(wheelRad + diffRad);
    }

    public void setTurningWheel(double rad) {
        m_turningMotor.set(rad);
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_driveMotor.reset();
        m_turningMotor.reset();
    }

    @Override
    public void periodic() {

        // int magTicks = m_magEncoder.get();
        // double wheelTicks = m_turningMotor.getRotationTicks();

        // SmartDashboard.putNumber(this.m_magEncoder.m_analogInput.getChannel() + "
        // Swerve Mag ", magTicks);
        // SmartDashboard.putNumber(this.m_magEncoder.m_analogInput.getChannel() + "
        // Swerve Wheel ", wheelTicks);
        super.periodic();
    }

}
