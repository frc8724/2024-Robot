package frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ArmSubsystem.ArmIsAtPosition;
import frc.robot.subsystems.ArmSubsystem.ArmSet;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;

public class ShootShort extends SequentialCommandGroup {
    public ShootShort() {
        addCommands(
                new ArmSet(ArmSubsystem.ANGLE_SHOT_POSITION),
                new ArmIsAtPosition(ArmSubsystem.POSITION_SLOP),
                new ShootNote(4500.0));
    }
}
