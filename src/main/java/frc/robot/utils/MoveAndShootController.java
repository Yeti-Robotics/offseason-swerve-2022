package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class MoveAndShootController {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private boolean isControllerEnabled = false;
    private Pose2d robotPose;

    private Pose2d targetPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    private final Pose2d origin = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    private Rotation2d vectorAngle;
    private Translation2d robotToTargetVector;
    private Translation2d movementVector;

    public MoveAndShootController(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    private void updateTargetLocation() {
        ChassisSpeeds chassisSpeeds = drivetrainSubsystem.getChassisSpeeds();
        robotPose = drivetrainSubsystem.getPose();
        targetPose = origin.relativeTo(robotPose);

        robotToTargetVector = new Translation2d(targetPose.getX(), targetPose.getY());
        movementVector = new Translation2d(-chassisSpeeds.vxMetersPerSecond, -chassisSpeeds.vyMetersPerSecond);

        getAngleBetweenVectors();
    }

    private void getAngleBetweenVectors() {
        double angle =
            Math.atan(movementVector.getNorm()/ robotToTargetVector.getNorm());
//        Math.min(Math.acos(movementVector.getX() * robotToTargetVector.getX()) + (movementVector.getY() * robotToTargetVector.getY())
//                / (movementVector.getNorm() * robotToTargetVector.getNorm()) * Math.PI / 2,
//            Math.PI / 9);

        if (angle < 0.08) {
            vectorAngle = new Rotation2d(0.0);
            return;
        }

        Translation2d referenceAxis = robotToTargetVector.rotateBy(robotPose.getRotation().unaryMinus());
        Translation2d relativeVector = movementVector.rotateBy(robotPose.getRotation().unaryMinus());

        if (relativeVector.getY() < referenceAxis.getY()) {
            vectorAngle = new Rotation2d(-angle);
            return;
        }

        vectorAngle = new Rotation2d(angle);
    }

    public double calculateShooterSpeed() {
        if (!isControllerEnabled) {
            return 0;
        }
        updateTargetLocation();

        Pose2d predictedTargetPose = targetPose.transformBy(new Transform2d(movementVector, Rotation2d.fromDegrees(0)));

        return predictedTargetPose.getTranslation().getNorm() - targetPose.getTranslation().getNorm();
    }

    public double calculateDegOffset() {
        if (!isControllerEnabled) {
            return 0;
        }
        updateTargetLocation();

        return vectorAngle.getDegrees();
    }

    public double calculateRadOffset() {
        if (!isControllerEnabled) {
            return 0;
        }
        updateTargetLocation();

        return Math.toRadians(vectorAngle.getDegrees());
    }

    public boolean getIsControllerEnabled() {
        return isControllerEnabled;
    }

    public void toggleMoveAndShootController() {
        isControllerEnabled = !isControllerEnabled;
    }
}
