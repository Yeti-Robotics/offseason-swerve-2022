package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.CargoPositions;
import frc.robot.FieldConstants.TarmacPositions;
import frc.robot.Robot.AutoModes;
import frc.robot.RobotContainer;
import frc.robot.commands.AllinCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;

public class AutoBuilder {
    private final double shotDurationSec = 1.0;
    private RobotContainer robotContainer;
    private AutoModes autoMode;
    private SequentialCommandGroup autoCommand;
    private AutoPath startPath;

    public Command build() {
        autoCommand = new SequentialCommandGroup();

//        switch (autoMode) {
//            case ONE_BALL:
//                oneBallAuto();
//            case FOUR_BALL:
//                fourBallAuto();
//            case TUNNING:
//                tuningAuto();
//        }
        fourBallAuto();
        autoCommand.beforeStarting(new InstantCommand(() -> startPath.getInitPose()));
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
            new Pose2d(TarmacPositions.tarmacVertexB, TarmacPositions.tarmacAngleA).transformBy(DriveConstants.ROBOT_CENTER),
            new Pose2d(CargoPositions.cargoB, TarmacPositions.tarmacAngleA));

        autoCommand.addCommands(startPath.getAutoPath());
    }

    private void fourBallAuto() {
        startPath = new AutoPath(robotContainer.drivetrainSubsystem,
            false,
            new Pose2d(TarmacPositions.tarmacVertexE, TarmacPositions.tarmacAngleD).transformBy(DriveConstants.ROBOT_CENTER),
            new Pose2d(CargoPositions.cargoE, TarmacPositions.tarmacAngleD)
                .transformBy(
                    new Transform2d(CargoPositions.cargoE.minus(new Translation2d(0.0, -0.2)),
                        Rotation2d.fromDegrees(0.0))),
            new Pose2d(TarmacPositions.tarmacVertexE, TarmacPositions.tarmacAngleD.minus(Rotation2d.fromDegrees(10.0))).transformBy(DriveConstants.ROBOT_CENTER)
        );

        AutoPath startToBallThree = new AutoPath(robotContainer.drivetrainSubsystem,
            false,
            startPath.getEndPose(),
            new Pose2d(TarmacPositions.tarmacVertexE.minus(
                new Translation2d(1.5, 0.2)
            ), Rotation2d.fromDegrees(170)),
            new Pose2d(CargoPositions.cargoD, Rotation2d.fromDegrees(170.0))
        );

        AutoPath ballThreeTurnToGoal = new AutoPath(robotContainer.drivetrainSubsystem,
            false,
            startToBallThree.getEndPose(),
            new Pose2d(startToBallThree.getEndPose().getTranslation().plus(new Translation2d(0.1, 0.1)), new Rotation2d(
                Math.atan(startToBallThree.getEndPose().getY() / startToBallThree.getEndPose().getX())).unaryMinus())
        );

        AutoPath ballThreetoFour = new AutoPath(robotContainer.drivetrainSubsystem,
            false,
            startToBallThree.getEndPose(),
            new Pose2d(CargoPositions.terminalCargo, FieldConstants.faceTerminalRotation)
        );

        AutoPath ballFourToEnd = new AutoPath(robotContainer.drivetrainSubsystem,
            false,
            ballThreetoFour.getEndPose(),
            new Pose2d(TarmacPositions.tarmacVertexD, new Rotation2d(
                Math.atan(TarmacPositions.tarmacVertexD.getY() / TarmacPositions.tarmacVertexD.getX())).unaryMinus()
            )
        );

        autoCommand.addCommands(
            startPath.getAutoPath().deadlineWith(
                new SequentialCommandGroup(
                    new ToggleIntakeCommand(robotContainer.intakeSubsystem),
                    new WaitCommand(startPath.getPathDuration() / 2.0 - 1.0),
                    new AllinCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(1.0),
                    new ToggleIntakeCommand(robotContainer.intakeSubsystem),
                    new ToggleShooterCommand(20, robotContainer.shooterSubsystem),
                    new WaitCommand(.75),
                    new ToggleShooterCommand(ShooterMode.LIMELIGHT, robotContainer.shooterSubsystem),
                    new WaitCommand(0.5),
                    new AllinCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(1.0)
                )
            ).andThen(new ToggleShooterCommand(ShooterMode.OFF, robotContainer.shooterSubsystem)),
            startToBallThree.getAutoPath().deadlineWith(
                new SequentialCommandGroup(
                    new ToggleIntakeCommand(robotContainer.intakeSubsystem),
                    new WaitCommand(startToBallThree.getPathDuration() - 1),
                    new AllinCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(1.0)
                )
            ),
            ballThreeTurnToGoal.getAutoPath().alongWith(
                new SequentialCommandGroup(
                    new ToggleIntakeCommand(robotContainer.intakeSubsystem),
                    new ToggleShooterCommand(20, robotContainer.shooterSubsystem),
                    new WaitCommand(0.75),
                    new ToggleShooterCommand(ShooterMode.LIMELIGHT, robotContainer.shooterSubsystem),
                    new AllinCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem),
                    new ToggleShooterCommand(ShooterMode.OFF, robotContainer.shooterSubsystem))
            ),
            ballThreetoFour.getAutoPath().deadlineWith(
                new SequentialCommandGroup(
                    new ToggleIntakeCommand(robotContainer.intakeSubsystem),
                    new WaitCommand(ballThreetoFour.getPathDuration() - 2.0),
                    new ToggleIntakeCommand(robotContainer.intakeSubsystem),
                    new WaitCommand(1.0),
                    new AllinCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem).withTimeout(1.0)
                )
            ),
            ballFourToEnd.getAutoPath().deadlineWith(
                new SequentialCommandGroup(
                    new ToggleIntakeCommand(robotContainer.intakeSubsystem),
                    new WaitCommand(ballFourToEnd.getPathDuration() - 3.0),
                    new ToggleShooterCommand(20, robotContainer.shooterSubsystem),
                    new WaitCommand(0.75),
                    new ToggleShooterCommand(ShooterMode.LIMELIGHT, robotContainer.shooterSubsystem),
                    new WaitCommand(0.5),
                    new AllinCommand(robotContainer.intakeSubsystem, robotContainer.neckSubsystem)
                )
            ).andThen(new ToggleShooterCommand(ShooterMode.OFF, robotContainer.shooterSubsystem))
        );
    }

    private void tuningAuto() {
        startPath = new AutoPath(robotContainer.drivetrainSubsystem,
            false,
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(3.0, 0.0, new Rotation2d(0.0))
        );

        Command commandOne = startPath.getAutoPath();

        autoCommand.addCommands(commandOne);
    }
}
