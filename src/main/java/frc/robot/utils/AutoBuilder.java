package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Robot.AutoModes;
import frc.robot.RobotContainer;
import frc.robot.commands.AllinCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;

public class AutoBuilder {
    private RobotContainer robotContainer;
    private AutoModes autoMode;
    private SequentialCommandGroup autoCommand;
    private AutoPath startPath;

    public Command build() {
        autoCommand = new SequentialCommandGroup();

        switch (autoMode) {
            case ONE_BALL_A:
                startPath = new AutoPath(robotContainer.drivetrainSubsystem, AutoConstants.oneBallPathA);
                oneBallAutoNoPath();
                break;
            case ONE_BALL_B:
                startPath = new AutoPath(robotContainer.drivetrainSubsystem, AutoConstants.oneBallPathB);
                oneBallAutoNoPath();
                break;
            case ONE_BALL_C:
                startPath = new AutoPath(robotContainer.drivetrainSubsystem, AutoConstants.oneBallPathC);
                oneBallAutoNoPath();
                break;
            case ONE_BALL_D:
                startPath = new AutoPath(robotContainer.drivetrainSubsystem, AutoConstants.oneBallPathD);
                oneBallAutoNoPath();
                break;
            case TWO_BALL:
                twoBallAuto();
                break;
            case THREE_BALL:
                threeBallAuto();
                break;
            case FOUR_BALL:
                fourBallAuto();
                break;
            case TUNNING:
                tuningAuto();
                break;
        }

        autoCommand.beforeStarting(new InstantCommand(() -> robotContainer.drivetrainSubsystem.resetOdometer(startPath.getInitPose())));
        return autoCommand;
    }

    public void setRobotContainer(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void setAutoMode(AutoModes autoMode) {
        this.autoMode = autoMode;
    }

    private void oneBallAutoNoPath() {
        autoCommand.addCommands(
            new SequentialCommandGroup(
                new ToggleShooterCommand(ShooterMode.LIMELIGHT, robotContainer.shooterSubsystem),
                new WaitCommand(1.0),
                new AllinCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(1.5),
                new ToggleShooterCommand(ShooterMode.OFF, robotContainer.shooterSubsystem)
            ),
            startPath.getAutoPath()
        );
    }

    private void twoBallAuto() {
        startPath = new AutoPath(robotContainer.drivetrainSubsystem,
            AutoConstants.twoBallPath,
            AutoConstants.MAX_VELOCITY,
            AutoConstants.MAX_ACCEL);

        autoCommand.addCommands(
            startPath.getAutoPath().alongWith(
                new SequentialCommandGroup(
                    new ToggleIntakeCommand(robotContainer.intakeSubsystem),
                    new WaitCommand(startPath.getPathDuration() - 1.5),
                    new AllinCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(0.7),
                    new ToggleIntakeCommand(robotContainer.intakeSubsystem),
                    new WaitCommand(0.2),
                    new ToggleShooterCommand(17, robotContainer.shooterSubsystem),
                    new WaitCommand(0.4),
                    new ToggleShooterCommand(ShooterMode.LIMELIGHT, robotContainer.shooterSubsystem),
                    new WaitCommand(0.2),
                    new AllinCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(1.0)
                )
            ),
            new ToggleShooterCommand(ShooterMode.OFF, robotContainer.shooterSubsystem)
        );
    }

    private void threeBallAuto() {
        twoBallAuto();

        AutoPath twoBallToBallThree = new AutoPath(robotContainer.drivetrainSubsystem,
            AutoConstants.threeBallPath);

        autoCommand.addCommands(
            twoBallToBallThree.getAutoPath().alongWith(
                new SequentialCommandGroup(
                    new ToggleIntakeCommand(robotContainer.intakeSubsystem),
                    new WaitCommand(0.5),
                    new AllinCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(1.0),
                    new ToggleShooterCommand(17, robotContainer.shooterSubsystem),
                    new WaitCommand(0.3),
                    new ToggleShooterCommand(ShooterMode.LIMELIGHT, robotContainer.shooterSubsystem),
                    new WaitCommand(0.3),
                    new AllinCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(1.0)
                )
            ),
            new ToggleShooterCommand(ShooterMode.OFF, robotContainer.shooterSubsystem)
        );
    }

    private void fourBallAuto() {
        AutoPath ballThreeToFour = new AutoPath(robotContainer.drivetrainSubsystem,
            AutoConstants.ballThreeToTerminal
        );

        AutoPath ballFourToEnd = new AutoPath(robotContainer.drivetrainSubsystem,
            AutoConstants.fourBallPathOne
        );

        threeBallAuto();

        autoCommand.addCommands(
            ballThreeToFour.getAutoPath().alongWith(
                new SequentialCommandGroup(
                    new ToggleIntakeCommand(robotContainer.intakeSubsystem),
                    new WaitCommand(1.0),
                    new AllinCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem)
                        .withTimeout(ballThreeToFour.getPathDuration() - 1.0),
                    new ToggleIntakeCommand(robotContainer.intakeSubsystem)
                )
            ),
            ballFourToEnd.getAutoPath().alongWith(
                new SequentialCommandGroup(
                    new WaitCommand(ballFourToEnd.getPathDuration() - 0.5),
                    new ToggleShooterCommand(17, robotContainer.shooterSubsystem),
                    new WaitCommand(0.25),
                    new ToggleShooterCommand(ShooterMode.LIMELIGHT, robotContainer.shooterSubsystem),
                    new AllinCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(1.0),
                    new ToggleShooterCommand(ShooterMode.OFF, robotContainer.shooterSubsystem)
                )
            )
        );
    }

    private void fiveBallAuto() {
        AutoPath ballThreeToFour = new AutoPath(robotContainer.drivetrainSubsystem,
            AutoConstants.ballThreeToTerminal);

        AutoPath humanPlayer = new AutoPath(robotContainer.drivetrainSubsystem,
            AutoConstants.fiveBallPathOne,
            1, 3);

        AutoPath ballFiveToEnd = new AutoPath(robotContainer.drivetrainSubsystem,
            AutoConstants.fiveBallPathTwo);
    }

    private void tuningAuto() {
        startPath = new AutoPath(robotContainer.drivetrainSubsystem,
            AutoConstants.tuningAutoPath
        );

        autoCommand.addCommands(startPath.getAutoPath());
    }
}
