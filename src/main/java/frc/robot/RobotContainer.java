// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.VisionSubsystems.Vision;
import frc.robot.subsystems.VisionSubsystems.Vision;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.Operator;
import frc.robot.Constants.Swerve;
import frc.robot.commands.TestSwerveTurnToPos;
import frc.robot.controls.MayhemExtreme3dPro;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.Autonomous.AutoChooser;
import frc.robot.subsystems.Autonomous.AutoDriveOut;
import frc.robot.subsystems.Autonomous.AutoStandStill;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.DriveBase.DriveBaseSubsystem;
import frc.robot.subsystems.DriveBase.DriveZeroGyro;
import frc.robot.subsystems.DriveBase.DriveZeroWheels;
import frc.robot.subsystems.DriveBase.DrivebaseResetEncoders;
import frc.robot.subsystems.Intake.IntakeRollers;
import frc.robot.subsystems.ShooterSubsystem.ShooterMag;
import frc.robot.subsystems.ShooterSubsystem.ShooterWheels;
import frc.robot.subsystems.Targeting.Targeting;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public static final DriveBaseSubsystem m_robotDrive = new DriveBaseSubsystem();
  public static final IntakeRollers m_rollers = new IntakeRollers();
  public static final ShooterMag m_mag = new ShooterMag();
  public static final ShooterWheels m_wheels = new ShooterWheels();
  public static final ArmSubsystem m_arm = new ArmSubsystem();
  public static final ClimberSubsystem m_climber = new ClimberSubsystem();
  public static final Targeting m_targets = new Targeting();
  private static final MayhemExtreme3dPro DriverStick = new MayhemExtreme3dPro(0);
  private static final AutoChooser m_auto= new AutoChooser();
  public static final Vision vision = new Vision(0);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(Operator.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

m_auto.addAuto(new AutoStandStill());
m_auto.addAuto(new AutoDriveOut());


    // Configure the trigger bindings
    configureBindings();
    // m_robotDrive.setDefaultCommand(
		// 		new RunCommand(
		// 				() -> m_robotDrive.drive(
    //           	DriverStick.DeadbandAxis(MayhemExtreme3dPro.Axis.X, 0.2)
		// 								* Swerve.kMaxSpeedMetersPerSecond,
    //              -DriverStick.DeadbandAxis(MayhemExtreme3dPro.Axis.Y, 0.20)
    //                 * Swerve.kMaxSpeedMetersPerSecond,
		// 						DriverStick.DeadbandAxis(MayhemExtreme3dPro.Axis.Z, 0.20)
		// 								* ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
		// 						true),
		// 				m_robotDrive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    		// DriverStick.Button(9).onTrue(
				// new SequentialCommandGroup(
				// 		new DriveZeroWheels(),
				// 		new DriveZeroGyro(),
				// 		new WaitCommand(1.0),
				// 		new DrivebaseResetEncoders(),
				// 		new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true), m_robotDrive)));


        DriverStick.Button(7).onTrue(new TestSwerveTurnToPos(0.0));
        DriverStick.Button(8).onTrue(new TestSwerveTurnToPos(0.5));

        DriverStick.Button(9).onTrue(new TestSwerveTurnToPos(1.0));
        DriverStick.Button(10).onTrue(new TestSwerveTurnToPos(3.0)); 
        
        m_robotDrive.setDefaultCommand(
				new RunCommand(
						() -> m_robotDrive.setWheelsAt(
              	DriverStick.DeadbandAxis(MayhemExtreme3dPro.Axis.X, 0.2)
										* 10),
						m_robotDrive));
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
