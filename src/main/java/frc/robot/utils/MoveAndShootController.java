package frc.robot.utils;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSubsystem;

public class MoveAndShootController {
    private final DrivetrainSubsystem drivetrainSubsystem;

    private ChassisSpeeds chassisSpeeds;
    private Pose2d robotPose;

    private Pose2d targetPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    private Pose2d origin = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    private double vectorMagnitude;
    private Rotation2d vectorAngle;
    private Translation2d robotToTargetVector;

    public MoveAndShootController(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    private void updateTargetLocation() {
        chassisSpeeds = drivetrainSubsystem.getChassisSpeeds();
        robotPose = drivetrainSubsystem.getPose();
        targetPose = origin.relativeTo(robotPose);

        if (robotPose.getX() > 0) {
            chassisSpeeds.vxMetersPerSecond = -chassisSpeeds.vxMetersPerSecond;
            chassisSpeeds.vyMetersPerSecond = -chassisSpeeds.vyMetersPerSecond;
        }

        vectorMagnitude = Math.sqrt(
                Math.pow(chassisSpeeds.vyMetersPerSecond, 2)
              + Math.pow(chassisSpeeds.vyMetersPerSecond, 2)
        );

        robotToTargetVector = new Translation2d(targetPose.getX(), targetPose.getY());

        vectorAngle = new Rotation2d(Math.acos(
                (chassisSpeeds.vyMetersPerSecond * targetPose.getX())
                + (chassisSpeeds.vyMetersPerSecond * robotPose.getY())
                / (vectorMagnitude * robotToTargetVector.getNorm()))
        );
    }

    public double calculateShooterSpeed() {
        if (chassisSpeeds.vyMetersPerSecond > 0.2) {
            return 0.0;
        }
        updateTargetLocation();

        return vectorMagnitude * 1.0;
    }

    public double calculateAngleOffset() {
        updateTargetLocation();



        return Math.toDegrees(Math.atan(vectorMagnitude / VisionSubsystem.getDistance()));
    }
}
