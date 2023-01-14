package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.function.BooleanSupplier;

public class JoyButton extends JoystickButton {

    public JoyButton(Joystick joystick, int number) {
        super(joystick, number);
    }

    public void conditionalPressed(
        Command trueCommand,
        ActiveState trueActiveState,
        Command falseCommand,
        ActiveState falseActiveState,
        BooleanSupplier booleanSupplier) {

        CommandScheduler.getInstance().getActiveButtonLoop()
            .bind(
                new Runnable() {
                    private boolean pressedLast = getAsBoolean();

                    @Override
                    public void run() {
                        boolean pressed = getAsBoolean();

                        if (booleanSupplier.getAsBoolean()) {
                            conditionRunner(trueCommand, trueActiveState, pressed, pressedLast);
                        } else {
                            conditionRunner(falseCommand, falseActiveState, pressed, pressedLast);
                        }
                        pressedLast = pressed;
                    }
                });
    }

    private void conditionRunner(
        Command command, ActiveState activeState, boolean pressed, boolean pressedLast) {
        switch (activeState) {
            case WHILE_HELD:
                if (pressed) {
                    command.schedule();
                } else if (pressedLast) {
                    command.cancel();
                }
                break;
            default:
                if (!pressedLast && pressed) {
                    command.schedule();
                }
                break;
        }
    }

    public enum ActiveState {
        WHILE_HELD,
        WHEN_PRESSED,
    }
}
