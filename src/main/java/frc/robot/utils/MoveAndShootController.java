package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSubsystem;

public class MoveAndShootController {
    private final DrivetrainSubsystem drivetrainSubsystem;

    private Pose2d targetPose;

    private ChassisSpeeds chassisSpeeds;
    private Transform2d targetOffset;
    private final Rotation2d zeroRotation = new Rotation2d(0.0);

    private double vectorMagnitude;

    public MoveAndShootController(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;

        targetPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    }

    private void updateTargetLocation() {
        chassisSpeeds = drivetrainSubsystem.getChassisSpeeds();
        targetOffset = new Transform2d(
                new Translation2d(-chassisSpeeds.vxMetersPerSecond, -chassisSpeeds.vyMetersPerSecond), zeroRotation);

        targetPose = targetPose.relativeTo(drivetrainSubsystem.getPose()).transformBy(targetOffset);

        double vectorMagnitude = Math.sqrt(Math.pow(targetPose.getX(), 2) + Math.pow(targetPose.getY(), 2));
    }

    public double calculateShooterSpeed() {
        if (targetPose.getX() > 0.2) {
            return 0.0;
        }
        updateTargetLocation();

        return vectorMagnitude * 1.0;
    }

    public double calculateAngleOffset() {
        if (targetPose.getY() > 0.2) {
            return 0.0;
        }
        updateTargetLocation();

        return Math.toDegrees(Math.atan(vectorMagnitude / VisionSubsystem.getDistance()));
    }
}
