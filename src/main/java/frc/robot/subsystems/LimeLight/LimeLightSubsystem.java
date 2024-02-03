package frc.robot.subsystems.LimeLight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LimeLightSubsystem extends SubsystemBase{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv; // Whether the limelight has any valid targets (0-1)
    NetworkTableEntry tx; // Horizontal Offset from Crosshair to target (-27 degrees to 27 degrees)
    NetworkTableEntry ty; // Vertical Offset from Crosshair to target (-20.5 degrees to 20.5 degrees)
    NetworkTableEntry ta; // Target Area (0% of image to 100% of image)
    NetworkTableEntry ts; // Skew or Rotation (-90 degrees to 0 degrees)
    NetworkTableEntry tl; // The pipeline's latency contribution (ms) Add at least 11ms for image capture latency

    NetworkTableEntry tid;    // ID of the tag that we are looking at
    NetworkTableEntry tshort; // Sidelength of shortest side of the fitted boudning box (pixels)
    NetworkTableEntry tlong;  // Sidelength of the longest side of the fitted boudning box (pixels)
    NetworkTableEntry thoriz; // Horizontal sidelength of the rough bounding box (0-320 pixels)
    NetworkTableEntry tvert;  // Vertical sidelength of the rough bounding box (0-320 pixels)

    public LimeLightSubsystem() {
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ts = table.getEntry("ts");
        tl = table.getEntry("tl");
        tid = table.getEntry("tid");
        tshort = table.getEntry("tshort");
        tlong  = table.getEntry("tlong");
        thoriz = table.getEntry("thoriz");
        tvert  = table.getEntry("tvert");

       // addRequirements(RobotContainer.m_robotDrive);
    }

    public void centerOnTag() {
        if (tv.getDouble(0.0) == 1)
        {
            while (tid.getDouble(0.0) != 0) {
                double lateralOffset = tx.getDouble(0.0);
                RobotContainer.m_robotDrive.drive(0.0, 0.2, 0.0, true);
            }
        }
    }

    public Double getTv() {
        return tv.getDouble(0.0);
    }

    public Double getTx() {
        return tx.getDouble(0.0);
    }

    public Double getTy() {
        return ty.getDouble(0.0);
    }

    public Double getTs() {
        return ts.getDouble(0.0);
    }

    public Double getTa() {
        return ta.getDouble(0.0);
    }

    public Double getTid() {
        return tid.getDouble(0.0);
    }



    @Override
    public void periodic() {
        SmartDashboard.putNumber("LimelightImage?", this.getTv());
        SmartDashboard.putNumber("LimelightTagID", tid.getDouble(0.0));
        SmartDashboard.putNumber("LimelightX", tx.getDouble(0.0));
        SmartDashboard.putNumber("LimelightY", ty.getDouble(0.0));
        SmartDashboard.putNumber("LimelightRotation", ts.getDouble(0.0));
        SmartDashboard.putNumber("LimelightArea", ta.getDouble(0.0));
    }
}
