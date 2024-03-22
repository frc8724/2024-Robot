package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class MayhemLogitechAttack3 {
    Joystick m_joystick;

    public MayhemLogitechAttack3(int joystickPort) {
        m_joystick = new Joystick(joystickPort);
    }

    public Trigger Button(int button) {
        return new JoystickButton(m_joystick, button);
    }
}
