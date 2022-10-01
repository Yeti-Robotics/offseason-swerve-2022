package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AxisToButton extends Button {
    private final GenericHID controller;

    private final int port;
    private double threshold = 0.0;

    public AxisToButton(GenericHID controller, int port, double threshold) {
        this.controller = controller;
        this.port = port;
        this.threshold = threshold;
    }

    public AxisToButton(GenericHID controller, int port) {
        this.controller = controller;
        this.port = port;
    }

    @Override
    public boolean get() {
        return controller.getRawAxis(port) >= threshold;
    }
}
