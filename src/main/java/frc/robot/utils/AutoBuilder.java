package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.FieldConstants.CargoPositions;
import frc.robot.FieldConstants.TarmacPositions;
import frc.robot.Robot.AutoModes;
import frc.robot.RobotContainer;

public class AutoBuilder {
    private RobotContainer robotContainer;
    private AutoModes autoMode;

    private SequentialCommandGroup pathCommands;
    private SequentialCommandGroup commands;
    private AutoPath startPath;

    private final double shotDurationSec = 1.0;

    public Command build() {
        pathCommands = new SequentialCommandGroup();
        commands = new SequentialCommandGroup();
        Command autoCommand = new ParallelCommandGroup();

        switch (autoMode) {
            case ONE_BALL:
                oneBallAuto();
        }

        autoCommand.alongWith(pathCommands, commands)
                .beforeStarting(new InstantCommand(() -> startPath.getInitPose()));
        return autoCommand;
    }

    public void setRobotContainer(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void setAutoMode(AutoModes autoMode) {
        this.autoMode = autoMode;
    }

    private void oneBallAuto() {
        startPath = new AutoPath(robotContainer.drivetrainSubsystem,
                false,
                new Pose2d(TarmacPositions.tarmacVertexF, TarmacPositions.tarmacAngleA).transformBy(DriveConstants.ROBOT_CENTER),
                new Pose2d(CargoPositions.cargoB, TarmacPositions.tarmacAngleA));

        pathCommands.addCommands(startPath.getAutoPath());
    }
}
