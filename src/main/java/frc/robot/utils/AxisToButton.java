package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class AxisToButton extends JoystickButton {
    private final GenericHID controller;

    private final int port;
    private double threshold = 0.0;

    public AxisToButton(GenericHID controller, int port, double threshold) {
        super(controller, port);
        this.controller = controller;
        this.port = port;
        this.threshold = threshold;
    }

    public AxisToButton(GenericHID controller, int port) {
        super(controller, port);
        this.controller = controller;
        this.port = port;
    }

    @Override
    public boolean getAsBoolean() {
        return controller.getRawAxis(port) >= threshold;
    }
}
