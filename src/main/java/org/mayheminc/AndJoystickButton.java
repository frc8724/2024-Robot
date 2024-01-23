package org.mayheminc;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.*;

public class AndJoystickButton extends Trigger {
    public AndJoystickButton(GenericHID joystick1, int buttonNumber1, GenericHID joystick2, int buttonNumber2) {
        super(() -> joystick1.getRawButton(buttonNumber1) && joystick2.getRawButton(buttonNumber2));
    }
}
