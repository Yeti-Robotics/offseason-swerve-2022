package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot.AutoModes;
import frc.robot.RobotContainer;

public class AutoBuilder {
    private RobotContainer robotContainer;

    private AutoModes autoMode;

    private Command autoCommand;

    public Command build() {
        return autoCommand;
    }

    public void setRobotContainer(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void setAutoMode(AutoModes autoMode) {
        this.autoMode = autoMode;
    }

    private void oneBallAuto() {

    }

    private 
}
