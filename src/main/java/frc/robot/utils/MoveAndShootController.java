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

    private ChassisSpeeds chassisSpeeds;
    private Pose2d robotPose;
    private double vectorMagnitude;

    public MoveAndShootController(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    private void updateTargetLocation() {
        chassisSpeeds = drivetrainSubsystem.getChassisSpeeds();
        robotPose = drivetrainSubsystem.getPose();

        if (robotPose.getX() > 0) {
            chassisSpeeds.vxMetersPerSecond = -chassisSpeeds.vxMetersPerSecond;
            chassisSpeeds.vyMetersPerSecond = -chassisSpeeds.vyMetersPerSecond;
        }

        vectorMagnitude = Math.sqrt(
                Math.pow(chassisSpeeds.vyMetersPerSecond, 2)
              + Math.pow(chassisSpeeds.vyMetersPerSecond, 2)
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
        if (chassisSpeeds.vyMetersPerSecond > 0.2) {
            return 0.0;
        }
        updateTargetLocation();

        return Math.toDegrees(Math.atan(vectorMagnitude / VisionSubsystem.getDistance()));
    }
}
