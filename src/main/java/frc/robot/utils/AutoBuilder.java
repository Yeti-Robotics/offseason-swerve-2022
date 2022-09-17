// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Robot.AutoModes;

public class AutoBuilder {
    private RobotContainer robotContainer;
    private AutoModes autoMode;

    private ParallelCommandGroup command;
    private SequentialCommandGroup subsystemCommandGroup;
    private SequentialCommandGroup pathCommandGroup;

    private Trajectory loadTrajectoryJSON(String trajectoryJSON) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            return trajectory;
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            return null;
        }
    }

    private Trajectory loadTrajectoryPath(String trajectoryPath) {
        Trajectory trajectory = PathPlanner.loadPath(
                trajectoryPath, AutoConstants.MAX_SPEED, AutoConstants.MAX_ACCELERATION);
        return trajectory;
    }

    // for PathWeaver
    private Command runTrajectoryJSON(String trajectoryJSON) {
        Trajectory trajectory = loadTrajectoryJSON(trajectoryJSON);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
                robotContainer.drivetrainSubsystem::getPose, robotContainer.drivetrainSubsystem.getKinematics(),
                robotContainer.drivetrainSubsystem.getXController(),
                robotContainer.drivetrainSubsystem.getYController(),
                robotContainer.drivetrainSubsystem.getThetaController(),
                robotContainer.drivetrainSubsystem::setModuleStates, robotContainer.drivetrainSubsystem);

        return new SequentialCommandGroup(
                new InstantCommand(() -> robotContainer.drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> robotContainer.drivetrainSubsystem.stop()));
    }

    // for PathPlanner
    private Command runTrajectoryPath(String trajectoryPath) {
        Trajectory trajectory = loadTrajectoryPath(trajectoryPath);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
                robotContainer.drivetrainSubsystem::getPose, robotContainer.drivetrainSubsystem.getKinematics(),
                robotContainer.drivetrainSubsystem.getXController(),
                robotContainer.drivetrainSubsystem.getYController(),
                robotContainer.drivetrainSubsystem.getThetaController(),
                robotContainer.drivetrainSubsystem::setModuleStates, robotContainer.drivetrainSubsystem);

        return new SequentialCommandGroup(
                new InstantCommand(() -> robotContainer.drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> robotContainer.drivetrainSubsystem.stop()));
    }
}
