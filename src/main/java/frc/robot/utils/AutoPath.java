package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

public class AutoPath {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private Trajectory trajectory;

    private SwerveControllerCommand swerveControllerCommand;

    private List<Pose2d> waypoints;
    private double startVel = 0.0;
    private double endVel = 0.0;
    private double maxVel = AutoConstants.MAX_VELOCITY;
    private double maxAccel = AutoConstants.MAX_ACCEL;
    private boolean isReversed = false;

    public AutoPath(DrivetrainSubsystem drivetrainSubsystem,
                    double startVel, double endVel,
                    double maxVel, double maxAccel,
                    boolean isReversed, Pose2d... waypoints) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        try {
            this.waypoints = Objects.requireNonNull(Arrays.asList(waypoints));
        } catch (NullPointerException e) {
            System.out.println("NO WAYPOINTS IN AUTOPATH CONSTRUCTOR");
            this.waypoints = new ArrayList<>();
            generateAutoPathCommand();
            return;
        }
        this.startVel = startVel;
        this.endVel = endVel;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.isReversed = isReversed;

        generateAutoPathCommand();
    }

    public AutoPath(DrivetrainSubsystem drivetrainSubsystem, boolean isReversed, Pose2d... waypoints) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        try {
            this.waypoints = Objects.requireNonNull(Arrays.asList(waypoints));
        } catch (NullPointerException e) {
            System.out.println("NO WAYPOINTS IN AUTOPATH CONSTRUCTOR");
            this.waypoints = new ArrayList<>();
            generateAutoPathCommand();
            return;
        }
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

        swerveControllerCommand = new SwerveControllerCommand(trajectory,
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
