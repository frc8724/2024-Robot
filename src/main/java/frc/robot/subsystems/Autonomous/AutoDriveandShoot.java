// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SystemArmZero;
import frc.robot.subsystems.DriveBase.DriveForDistance;
import frc.robot.subsystems.ShooterSubsystem.ShootNote;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveandShoot extends SequentialCommandGroup {
  /** Creates a new AutoDriveandShoot. */
  public AutoDriveandShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SystemArmZero(),
    // add in correct distance, then add a turn command, then zero and prepare to shoot-- all with a note preloaded
    new DriveForDistance(0.0,0.2, 0.0,100.0),
    new ShootNote()
    );
    
  }
}
