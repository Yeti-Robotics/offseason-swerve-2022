package frc.robot.utils;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.function.BooleanSupplier;

public class JoyButton extends JoystickButton {

  public enum ActiveState {
    WHILE_HELD,
    WHEN_PRESSED,
  }

  public JoyButton(Joystick joystick, int number) {
    super(joystick, number);
  }

  public void conditionalPressed(
      Command trueCommand,
      ActiveState trueActiveState,
      Command falseCommand,
      ActiveState falseActiveState,
      BooleanSupplier booleanSupplier) {
    requireNonNullParam(trueCommand, "trueCommand", "conditionalPressed");
    requireNonNullParam(falseCommand, "falseCommand", "conditionalPressed");
    requireNonNullParam(booleanSupplier, "booleanSupplier", "conditionalPressed");

    CommandScheduler.getInstance()
        .addButton(
            new Runnable() {
              private boolean pressedLast = get();

              @Override
              public void run() {
                boolean pressed = get();

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
}
