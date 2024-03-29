// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.VisionSubsystems.Vision;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.controls.MayhemExtreme3dPro;
import frc.robot.subsystems.MayhemTalonFX;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.Autonomous.AutoChooser;
import frc.robot.subsystems.Autonomous.AutoDriveOut;
import frc.robot.subsystems.ClimberSubsystem.ClimberSubsystem;
import frc.robot.subsystems.DriveBase.DriveBaseSubsystem;
import frc.robot.subsystems.DriveBase.DriveForDistance;
import frc.robot.subsystems.DriveBase.DriveZeroGyro;
import frc.robot.subsystems.DriveBase.DriveZeroWheels;
import frc.robot.subsystems.DriveBase.DrivebaseResetEncoders;
import frc.robot.subsystems.IntakeRollers.IntakeRollers;
import frc.robot.subsystems.LimeLight.CenterOnTag;
import frc.robot.subsystems.LimeLight.LimeLightSubsystem;
import frc.robot.subsystems.MayhemTalonFX.CurrentLimit;
import frc.robot.subsystems.ShooterSubsystem.ShooterMag;
import frc.robot.subsystems.ShooterSubsystem.ShooterMagSet;
import frc.robot.subsystems.ShooterSubsystem.ShooterWheels;
import frc.robot.subsystems.ShooterSubsystem.ShooterWheelsSet;
import frc.robot.subsystems.ShooterSubsystem.ShooterWheelsSetRPM;
import frc.robot.subsystems.Targeting.Targeting;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static final IMayhemTalonFX intakeTop = new FakeFalconFX(Constants.DriveConstants.kIntakeRollerId,
      CurrentLimit.HIGH_CURRENT);
  private static final IMayhemTalonFX armLeft = new FakeFalconFX(Constants.DriveConstants.kArmLeftId,
      CurrentLimit.HIGH_CURRENT);
  private static final IMayhemTalonFX armRight = new FakeFalconFX(Constants.DriveConstants.kArmRightId,
      CurrentLimit.HIGH_CURRENT);
  private static final IMayhemTalonFX shooterLeft = new MayhemTalonFX(Constants.DriveConstants.kShooterLeftId,
      CurrentLimit.HIGH_CURRENT);
  private static final IMayhemTalonFX shooterRight = new MayhemTalonFX(Constants.DriveConstants.kShooterRightId,
      CurrentLimit.HIGH_CURRENT);
  private static final IMayhemTalonFX magLeft = new FakeFalconFX(Constants.DriveConstants.kMagLeftId,
      CurrentLimit.HIGH_CURRENT);
  private static final IMayhemTalonFX magRight = new FakeFalconFX(Constants.DriveConstants.kMagRightId,
      CurrentLimit.HIGH_CURRENT);
  private static final IMayhemTalonFX climberLeft = new FakeFalconFX(Constants.DriveConstants.kMagRightId,
      CurrentLimit.HIGH_CURRENT);
  private static final IMayhemTalonFX climberRight = new FakeFalconFX(Constants.DriveConstants.kMagRightId,
      CurrentLimit.HIGH_CURRENT);

  public static final DriveBaseSubsystem m_robotDrive = new DriveBaseSubsystem();
  public static final IntakeRollers m_rollers = new IntakeRollers(intakeTop);
  public static final ShooterMag m_mag = new ShooterMag(magLeft, magRight);
  public static final ShooterWheels m_wheels = new ShooterWheels(shooterLeft, shooterRight);
  public static final ArmSubsystem m_arm = new ArmSubsystem(armLeft, armRight);
  public static final ClimberSubsystem m_climber = new ClimberSubsystem(climberLeft, climberRight);

  public static final Targeting m_targets = new Targeting();
  private static final MayhemExtreme3dPro DriverStick = new MayhemExtreme3dPro(0);
  private static final AutoChooser m_auto = new AutoChooser();
  public static final Vision vision = new Vision(0);
  public static final LimeLightSubsystem m_limelight = new LimeLightSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -DriverStick.DeadbandAxis(MayhemExtreme3dPro.Axis.Y, 0.10)
                    * DriveConstants.kMaxSpeedMetersPerSecond,
                -DriverStick.DeadbandAxis(MayhemExtreme3dPro.Axis.X, 0.10)
                    * DriveConstants.kMaxSpeedMetersPerSecond,
                -DriverStick.DeadbandAxis(MayhemExtreme3dPro.Axis.Z, 0.20)
                    * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                true),
            m_robotDrive));

    m_auto.addAuto(new AutoDriveOut());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    DriverStick.Button(9).onTrue(
        new SequentialCommandGroup(
            new DriveZeroWheels(),
            new DriveZeroGyro(),
            new WaitCommand(1.0),
            new DrivebaseResetEncoders(),
            new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true), m_robotDrive)));

    DriverStick.Button(01).onTrue(new CenterOnTag());
    DriverStick.Button(10).onTrue(new DriveForDistance(0.0, 0.2, 0.0, 1.0));
    DriverStick.Button(11).onTrue(new DriveForDistance(0.2, 0.0, 0.0, 1.0));
    DriverStick.Button(12).onTrue(new DriveForDistance(0, 0, -.3, 1.0));
    DriverStick.Button(7).onTrue(new ShooterWheelsSet(0));
    DriverStick.Button(8).onTrue(new ShooterWheelsSetRPM(1500.0));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_auto.getAutoCommand();
  }

}
