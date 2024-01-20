package frc.robot.subsystems.DriveBase;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.Swerve;

public class DriveBaseSubsystem extends SubsystemBase {

    private final SwerveModule m_frontLeftSwerveModule = new SwerveModule(
            "frontLeftDriveMotor",
            MotorIDs.kFrontLeftDriveMotorPort,
            "frontLeftTurningMotor",
            MotorIDs.kFrontLeftTurningMotorPort,
            Swerve.kFrontLeftDriveEncoderReversed,
            Swerve.kFrontLeftTurningEncoderReversed,
            Swerve.FrontLeftMag);

    private final SwerveModule m_rearLeftSwerveModule = new SwerveModule(
            "rearLeftDriveMotor",
            MotorIDs.kRearLeftDriveMotorPort,
            "rearLeftTurningMotor",
            MotorIDs.kRearLeftTurningMotorPort,
            Swerve.kRearLeftDriveEncoderReversed,
            Swerve.kRearLeftTurningEncoderReversed,
            Swerve.RearLeftMag);

    private final SwerveModule m_frontRightSwerveModule = new SwerveModule(
            "frontRightDriveMotor",
            MotorIDs.kFrontRightDriveMotorPort,
            "frontRightTurningMotor",
            MotorIDs.kFrontRightTurningMotorPort,
            Swerve.kFrontRightDriveEncoderReversed,
            Swerve.kFrontRightTurningEncoderReversed,
            Swerve.FrontRightMag);

    private final SwerveModule m_rearRightSwerveModule = new SwerveModule(
            "rearRightDriveMotor",
            MotorIDs.kRearRightDriveMotorPort,
            "rearRightTurningMotor",
            MotorIDs.kRearRightTurningMotorPort,
            Swerve.kRearRightDriveEncoderReversed,
            Swerve.kRearRightTurningEncoderReversed,
            Swerve.RearRightMag);

    private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(22);

    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        Swerve.kDriveKinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    m_frontLeftSwerveModule.getPosition(),
                    m_frontRightSwerveModule.getPosition(),
                    m_rearLeftSwerveModule.getPosition(),
                    m_rearRightSwerveModule.getPosition()
            });

    public DriveBaseSubsystem() {
        m_gyro.configFactoryDefault();
        m_gyro.setYaw(0);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        // System.out.println("Gyro rotation: " + m_gyro.getRotation2d());
        // System.out.println("Gyro rotation 2: " + m_gyro.getYaw());

        m_odometry.update(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeftSwerveModule.getPosition(),
                        m_frontRightSwerveModule.getPosition(),
                        m_rearLeftSwerveModule.getPosition(),
                        m_rearRightSwerveModule.getPosition()
                });
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeftSwerveModule.getPosition(),
                        m_frontRightSwerveModule.getPosition(),
                        m_rearLeftSwerveModule.getPosition(),
                        m_rearRightSwerveModule.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           ngular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds roboChassisSpeeds = null;

        if (fieldRelative) {
            roboChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d());
        } else {
            roboChassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }
        // As of July 18th, the fromDiscreteSpeeds methods were added to ChassisSpeeds
        // to fix drift in complex turns
        // We would need to update to get that method
        // SwerveModuleState[] swerveModuleStates =
        // DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromDiscreteSpeeds(roboChassisSpeeds,
        // DriveConstants.kDrivePeriod));
        SwerveModuleState[] swerveModuleStates = Swerve.kDriveKinematics
                .toSwerveModuleStates(roboChassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Swerve.kMaxSpeedMetersPerSecond);
        m_frontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
        m_frontRightSwerveModule.setDesiredState(swerveModuleStates[1]);
        m_rearLeftSwerveModule.setDesiredState(swerveModuleStates[2]);
        m_rearRightSwerveModule.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve.kMaxSpeedMetersPerSecond);
        m_frontLeftSwerveModule.setDesiredState(desiredStates[0]);
        m_frontRightSwerveModule.setDesiredState(desiredStates[1]);
        m_rearLeftSwerveModule.setDesiredState(desiredStates[2]);
        m_rearRightSwerveModule.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeftSwerveModule.resetEncoders();
        m_frontRightSwerveModule.resetEncoders();
        m_rearLeftSwerveModule.resetEncoders();
        m_rearRightSwerveModule.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    public void zeroWheels() {
        m_frontLeftSwerveModule.zeroTurningWheel(Swerve.FrontLeftMagZero);
        m_frontRightSwerveModule.zeroTurningWheel((Swerve.FrontRightMagZero));
        m_rearRightSwerveModule.zeroTurningWheel(Swerve.RearRightMagZero);
        m_rearLeftSwerveModule.zeroTurningWheel(Swerve.RearLeftMagZero);
    }

    public void zeroGyro() {
        m_gyro.setYaw(0);
    }

    public void setWheelsAt(double rad) {
        m_frontLeftSwerveModule.setTurningWheel(rad);
        m_frontRightSwerveModule.setTurningWheel(rad);
        m_rearRightSwerveModule.setTurningWheel(rad);
        m_rearLeftSwerveModule.setTurningWheel(rad);

    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (Swerve.kGyroReversed ? -1.0 : 1.0);
    }
}
