package frc.robot.controls;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class MayhemExtreme3dPro {
    Joystick m_joystick;

    public enum Axis {
        X(0), Y(1), Z(2);

        private final int value;

        private Axis(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public MayhemExtreme3dPro(int joystickPort) {
        m_joystick = new Joystick(joystickPort);
    }

    public Trigger Button(int button) {
        return new JoystickButton(m_joystick, button);
    }

    public DoubleSupplier Axis(Axis axis) {
        return () -> m_joystick.getRawAxis(axis.getValue());
    }

    public double DeadbandAxis(Axis axis, double deadband) {
        double val = m_joystick.getRawAxis(axis.getValue());

        double sign = (val > 0) ? +1.0 : -1.0;
        double mag = Math.abs(val);

        if (mag < deadband) {
            return 0;
        }

        double offset = mag - deadband;
        double scaled = offset / (1.0 - deadband);
        return sign * scaled;
    }
}
