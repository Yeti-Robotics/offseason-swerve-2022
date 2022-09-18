package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.List;

public class AutoPath {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private Trajectory trajectory;

    private SwerveControllerCommand swerveControllerCommand;
    private Command autoPathCommand;

    private List<Pose2d> waypoints;
    private double startVel = 0.0;
    private double endVel = 0.0;
    private double maxVel = Constants.AutoConstants.kMaxVelocity;
    private double maxAccel = Constants.AutoConstants.kMaxAccel;

    public AutoPath(DrivetrainSubsystem drivetrainSubsystem, double startVel, List<Pose2d> waypoints, double endVel, double maxVel, double maxAccel) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.waypoints = waypoints;
        this.startVel = startVel;
        this.endVel = endVel;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;

    }

    public AutoPath(DrivetrainSubsystem drivetrainSubsystem, List<Pose2d> waypoints) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.waypoints = waypoints;
    }

    private void generateAutoPathCommand() {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                maxVel,
                maxAccel)
                .setKinematics(drivetrainSubsystem.getKinematics())
                .setStartVelocity(startVel)
                .setEndVelocity(endVel);

        trajectory = TrajectoryGenerator.generateTrajectory(waypoints, trajectoryConfig);

        swerveControllerCommand = new SwerveControllerCommand(trajectory,
                drivetrainSubsystem::getPose,
                drivetrainSubsystem.getKinematics(),
                drivetrainSubsystem.getxController(),
                drivetrainSubsystem.getyController(),
                drivetrainSubsystem.getThetaController(),
                drivetrainSubsystem::setDesiredStates,
                drivetrainSubsystem);

        autoPathCommand = new SequentialCommandGroup(
                new InstantCommand(() -> drivetrainSubsystem.resetOdometer(trajectory.getInitialPose())),
                swerveControllerCommand);
    }

    public void runAutoPath() {
        new ScheduleCommand(autoPathCommand);
    }

    public Command getAutoPath() {
        return autoPathCommand;
    }

    public double getPathDuration() {
        return trajectory.getTotalTimeSeconds();
    }
}
