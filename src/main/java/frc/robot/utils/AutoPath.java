package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

import java.util.Arrays;
import java.util.List;

public class AutoPath {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private Trajectory trajectory;

    private SwerveControllerCommand swerveControllerCommand;

    private final List<Pose2d> waypoints;
    private double startVel = 0.0;
    private double endVel = 0.0;
    private double maxVel = AutoConstants.MAX_VELOCITY;
    private double maxAccel = AutoConstants.MAX_ACCEL;
    private final boolean isReversed;

    public AutoPath(DrivetrainSubsystem drivetrainSubsystem,
                    double startVel, double endVel,
                    double maxVel, double maxAccel,
                    boolean isReversed, Pose2d... waypoints) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.waypoints = Arrays.asList(waypoints);
        this.startVel = startVel;
        this.endVel = endVel;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.isReversed = isReversed;

        generateAutoPathCommand();
    }

    public AutoPath(DrivetrainSubsystem drivetrainSubsystem, boolean isReversed, Pose2d... waypoints) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.waypoints = Arrays.asList(waypoints);
        this.isReversed = isReversed;

        generateAutoPathCommand();
    }

    private void generateAutoPathCommand() {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                maxVel,
                maxAccel)
                .setKinematics(drivetrainSubsystem.getKinematics())
                .setStartVelocity(startVel)
                .setEndVelocity(endVel)
                .setReversed(isReversed);

        trajectory = TrajectoryGenerator.generateTrajectory(waypoints, trajectoryConfig);

        swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                drivetrainSubsystem::getPose,
                drivetrainSubsystem.getKinematics(),
                drivetrainSubsystem.getxController(),
                drivetrainSubsystem.getyController(),
                drivetrainSubsystem.getThetaController(),
                drivetrainSubsystem::setDesiredStates,
                drivetrainSubsystem);
    }

    /**
     * DO NOT use if it is a part of a command group
     * Use getAutoPath() instead
     */
    public void runAutoPath() {
        new ScheduleCommand(swerveControllerCommand.beforeStarting(
                new InstantCommand(() -> drivetrainSubsystem.resetOdometer(getInitPose())))
        );
    }

    public Command getAutoPath() {
        return swerveControllerCommand;
    }

    public Pose2d getInitPose() {
        return trajectory.getInitialPose();
    }

    public double getPathDuration() {
        return trajectory.getTotalTimeSeconds();
    }
}
