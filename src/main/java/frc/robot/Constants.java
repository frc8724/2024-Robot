// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Operator {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorIDs {
    // Drive Motors
    public static final int kFrontLeftDriveMotorPort = 1; // kraken
    public static final int kRearLeftDriveMotorPort = 2; // kraken
    public static final int kFrontRightDriveMotorPort = 3; // kraken
    public static final int kRearRightDriveMotorPort = 4; // kraken
    // Drive Turing
    public static final int kFrontLeftTurningMotorPort = 5; // falcon fx
    public static final int kRearLeftTurningMotorPort = 6; // falcon fx
    public static final int kFrontRightTurningMotorPort = 7; // falcon fx
    public static final int kRearRightTurningMotorPort = 8; // falcon fx

    // Arm Pivot
    public static final int LEFT_SHOULDER_FALCON = 9; // falcons FX
    public static final int RIGHT_SHOULDER_FALCON = 10; // falcons FX

    // Magazine
    public static final int MAGAZINE_TALON_LEFT = 11; // neo 550
    public static final int MAGAZINE_TALON_RIGHT = 12; // neo 550

    // Shooter
    public static final int SHOOTER_WHEEL_LEFT = 13; // neo
    public static final int SHOOTER_WHEEL_RIGHT = 14; // neo

    // Intake Pivot
    public static final int INTAKE_PIVOT_LEFT = 15; // neo
    public static final int INTAKE_PIVOT_RIGHT = 16; // neo

    // Intake Rollers
    public static final int INTAKE_ROLLERS = 17; // neo

    // Climber
    public static final int CLIMBER_FALCON_LEFT = 18; // falcons FX
    public static final int CLIMBER_FALCON_RIGHT = 19; // falcons FX
  }

  public static class Swerve {
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    public static final int FrontLeftMag = 2;
    public static final int RearLeftMag = 0;
    public static final int FrontRightMag = 1;
    public static final int RearRightMag = 3;

    public static final int ninety_degrees_in_ticks = 0;
    public static final int one_eighty_degrees_in_ticks = 2048;

    public static final int FrontLeftMagZero = 333 + ninety_degrees_in_ticks; // 455;
    public static final int FrontRightMagZero = 678 + ninety_degrees_in_ticks;
    public static final int RearRightMagZero = 792 + ninety_degrees_in_ticks;// 333;
    public static final int RearLeftMagZero = 455 + ninety_degrees_in_ticks; // 333;// 792;

    // If you call DriveSubsystem.drive() with a different period make sure to
    // update this.
    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;

    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 1.5;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 50;

    public static final double kPModuleDriveController = 1;
  }
}
