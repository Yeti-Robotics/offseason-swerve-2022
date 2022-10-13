package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSubsystem;
import frc.robot.utils.MoveAndShootController;

import java.util.function.DoubleSupplier;

public class FieldOrientedDrive extends CommandBase {
    private static boolean targetLock = false;
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private final ProfiledPIDController targetLockPID =
        new ProfiledPIDController(
            ShooterConstants.TARGETING_P,
            ShooterConstants.TARGETING_I,
            ShooterConstants.TARGETING_D,
            new TrapezoidProfile.Constraints(
                2 * Math.PI,
                4 * Math.PI
            )
        );
    private final MoveAndShootController moveAndShootController;

    public FieldOrientedDrive(
        DrivetrainSubsystem drivetrainSubsystem,
        MoveAndShootController moveAndShootController,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier rotationSupplier) {

        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        this.moveAndShootController = moveAndShootController;

        addRequirements(drivetrainSubsystem);
    }

    public static void toggleTargetLock() {
        FieldOrientedDrive.targetLock = !FieldOrientedDrive.targetLock;
    }

    @Override
    public void execute() {
        double xSpeed = modifyAxis(translationXSupplier.getAsDouble()) *
            DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
        double ySpeed = modifyAxis(translationYSupplier.getAsDouble()) *
            DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
        double thetaSpeed = modifyAxis(rotationSupplier.getAsDouble()) *
            DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

        if (targetLock) {
            thetaSpeed = lockToTarget();
        }

        drivetrainSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                thetaSpeed,
                drivetrainSubsystem.getPose().getRotation())
        );
    }

    public boolean isTargetLock() {
        return FieldOrientedDrive.targetLock;
    }

    private double lockToTarget() {
        double movementOffset = moveAndShootController.calculateDegOffset();
        double offset = Math.copySign(ShooterConstants.TARGET_OFFSET, movementOffset);

        return targetLockPID.calculate(
            VisionSubsystem.getX(),
            Math.toDegrees(
                Math.atan(offset / (VisionSubsystem.getDistance() + 24.0))
                    + movementOffset)
        );
    }

    private double modifyAxis(double value) {
        if (Math.abs(value) <= OIConstants.DEADBAND) {
            return 0.0;
        }

        return Math.copySign(value * value * value, value);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}