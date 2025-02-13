package frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ArmSubsystem.ArmIsAtPosition;
import frc.robot.subsystems.ArmSubsystem.ArmSet;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;

public class ShootShort extends SequentialCommandGroup {
    public ShootShort() {
        this(0.0);
    }

    public ShootShort(double offset) {
        addCommands(
                new ArmSet(ArmSubsystem.SHORT_SHOT + offset),
                new ArmIsAtPosition(ArmSubsystem.POSITION_SLOP),
                new ShootNote(3500.0));
    }
}
