/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.mayheminc.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 *
 * @author Team1519
 */
public class JoystickAxisButton extends Trigger {
    enum Direction {
        BOTH_WAYS,
        POSITIVE_ONLY,
        NEGATIVE_ONLY
    }

    private static final double AXIS_THRESHOLD = 0.2;

    // private Joystick joystick;
    // private Joystick.AxisType axis;
    // private int axisInt;
    // private Direction direction;

    static double getAxis(Joystick stick, Joystick.AxisType axis) {
        switch (axis) {
        case kX:
            return stick.getX();
        case kY:
            return stick.getY();
        case kZ:
            return stick.getZ();
        case kThrottle:
            return stick.getThrottle();
        case kTwist:
            return stick.getTwist();
        default:
            // Unreachable
            return 0.0;
        }
    }

    public JoystickAxisButton(Joystick stick, Joystick.AxisType axis) {
        this(stick, axis, Direction.BOTH_WAYS);
    }

    public JoystickAxisButton(Joystick stick, Joystick.AxisType axis, Direction direction) {
        super(() -> get(stick, axis, direction));
        // joystick = stick;
        // this.axis = axis;
        // this.direction = direction;
    }

    // public JoystickAxisButton(Joystick stick, int axis) {
    //     this(stick, axis, Direction.BOTH_WAYS);
    // }

    public JoystickAxisButton(Joystick stick, int axis, Direction direction) {
        super( () -> getInt(stick, axis, direction));
        // joystick = stick;
        // axisInt = axis;
        // this.direction = direction;
    }

    static boolean getInt(Joystick stick, int axis, Direction direction) {
        switch (direction) {
        case BOTH_WAYS:
                return Math.abs(stick.getRawAxis(axis)) > AXIS_THRESHOLD;

        case POSITIVE_ONLY:
                return stick.getRawAxis(axis) > AXIS_THRESHOLD;
           
        case NEGATIVE_ONLY:
                return stick.getRawAxis(axis) < -AXIS_THRESHOLD;
        }

        return false;
    }

    // @Override
    static boolean get(Joystick stick, Joystick.AxisType axis, Direction direction) {
        switch (direction) {
        case BOTH_WAYS:
                return Math.abs(getAxis(stick, axis)) > AXIS_THRESHOLD;

        case POSITIVE_ONLY:
                return getAxis(stick, axis) > AXIS_THRESHOLD;

        case NEGATIVE_ONLY:
                return getAxis(stick, axis) < -AXIS_THRESHOLD;
        }

        return false;
    }
}
