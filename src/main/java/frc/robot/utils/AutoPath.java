package frc.robot.utils;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

import javax.swing.plaf.nimbus.State;
import java.util.Arrays;
import java.util.List;

public class AutoPath {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final List<Pose2d> waypoints;
    private final boolean isReversed;
    private Trajectory trajectory;
    private PathPlannerTrajectory pathPlannerTrajectory;
    private SwerveControllerCommand swerveControllerCommand;
    private double startVel = 0.0;
    private double endVel = 0.0;
    private double maxVel = AutoConstants.MAX_VELOCITY;
    private double maxAccel = AutoConstants.MAX_ACCEL;

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

    public AutoPath(DrivetrainSubsystem drivetrainSubsystem, String trajectoryFile) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.pathPlannerTrajectory = PathPlanner.loadPath(trajectoryFile, maxVel, maxAccel);

        waypoints = null;
        isReversed = false;
    }

    private void generateAutoPathCommand() {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            maxVel,
            maxAccel)
            .setKinematics(DriveConstants.DRIVE_KINEMATICS)
            .setStartVelocity(startVel)
            .setEndVelocity(endVel)
            .setReversed(isReversed);

        trajectory = TrajectoryGenerator.generateTrajectory(waypoints, trajectoryConfig);

        swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            drivetrainSubsystem::getPose,
            DriveConstants.DRIVE_KINEMATICS,
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

    public Pose2d getEndPose() {
        List<Trajectory.State> poses = trajectory.getStates();
        return poses.get(poses.size() - 1).poseMeters;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public double getPathDuration() {
        return trajectory.getTotalTimeSeconds();
    }
}
