package frc.robot.utils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

import java.util.List;

public class AutoPath {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private PathPlannerTrajectory trajectory;
    private PPSwerveControllerCommand swerveControllerCommand;
    private double startVel = 0.0;
    private double endVel = 0.0;
    private double maxVel = AutoConstants.MAX_VELOCITY;
    private double maxAccel = AutoConstants.MAX_ACCEL;

    public AutoPath(DrivetrainSubsystem drivetrainSubsystem, String trajectoryFile) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.trajectory = PathPlanner.loadPath(trajectoryFile, maxVel, maxAccel);

        generateAutoPathCommand();
    }


    public AutoPath(DrivetrainSubsystem drivetrainSubsystem, String trajectoryFile, double maxVel, double maxAccel) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.maxVel = maxVel;
        this.maxAccel= maxAccel;
        this.trajectory = PathPlanner.loadPath(trajectoryFile, maxVel, maxAccel);

        generateAutoPathCommand();
    }

    private void generateAutoPathCommand() {

        swerveControllerCommand = new PPSwerveControllerCommand(
            trajectory,
            drivetrainSubsystem::getPose,
            DriveConstants.DRIVE_KINEMATICS,
            drivetrainSubsystem.getxController(),
            drivetrainSubsystem.getyController(),
            drivetrainSubsystem.getThetaController(),
            drivetrainSubsystem::setDesiredStates,
            true,
                drivetrainSubsystem);

    }

    public void pathToLocation(Translation2d desiredPosition, Rotation2d desiredHeading, Rotation2d desiredRotation) {
        PathPlannerTrajectory pathPlannerTrajectory = PathPlanner.generatePath(
                new PathConstraints(2, 1),
                new PathPoint(new Translation2d(drivetrainSubsystem.getPose().getX(), drivetrainSubsystem.getPose().getY()), drivetrainSubsystem.getGyroscopeHeading()),
                new PathPoint(desiredPosition, desiredHeading, desiredRotation) //
                );


    }

    public void pathToLocation(double maxVel, double maxAccel, Translation2d desiredPosition, Rotation2d desiredHeading, Rotation2d desiredRotation){
        PathPlannerTrajectory pathPlannerTrajectory = PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAccel),
                new PathPoint(new Translation2d(drivetrainSubsystem.getPose().getX(), drivetrainSubsystem.getPose().getY()), drivetrainSubsystem.getGyroscopeHeading()),
                new PathPoint(desiredPosition, desiredHeading, desiredRotation));
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
        return trajectory.getEndState().poseMeters;
    }

    public PathPlannerTrajectory getTrajectory() {
        return trajectory;
    }

    public double getPathDuration() {
        return trajectory.getTotalTimeSeconds();
    }
}
