package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSubsystem;
import frc.robot.utils.MoveAndShootController;

import java.util.function.DoubleSupplier;

public class FieldOrientedDrive extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    private final SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

    private final PIDController targetLockPID;
    private final MoveAndShootController moveAndShootController;
    private boolean targetLock = false;

    public FieldOrientedDrive(DrivetrainSubsystem drivetrainSubsystem,
                              DoubleSupplier translationXSupplier,
                              DoubleSupplier translationYSupplier,
                              DoubleSupplier rotationSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        this.xLimiter = new SlewRateLimiter(DriveConstants.MAX_ACCELERATION);
        this.yLimiter = new SlewRateLimiter(DriveConstants.MAX_ACCELERATION);
        this.thetaLimiter = new SlewRateLimiter(DriveConstants.MAX_ANGULAR_ACCELERATION);

        targetLockPID = new PIDController(ShooterConstants.TARGETING_P, ShooterConstants.TARGETING_I, ShooterConstants.TARGETING_D);
        moveAndShootController = new MoveAndShootController(drivetrainSubsystem);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = translationXSupplier.getAsDouble();
        double ySpeed = -translationYSupplier.getAsDouble();
        double thetaSpeed = -rotationSupplier.getAsDouble();

        // System.out.println("Before xSpeed: " + xSpeed);
        // System.out.println("Before ySpeed: " + ySpeed);
        // System.out.println("Before thetaSpeed: " + thetaSpeed);
        
        xSpeed = xLimiter.calculate(Math.abs(xSpeed) > OIConstants.DEADBAND ? xSpeed : 0.0)
        * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
        ySpeed = yLimiter.calculate(Math.abs(ySpeed) > OIConstants.DEADBAND ? ySpeed : 0.0)
        * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
        if (targetLock) {
            thetaSpeed = lockToTarget() * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        } else {
            thetaSpeed = thetaLimiter.calculate(Math.abs(thetaSpeed) > OIConstants.DEADBAND ? thetaSpeed : 0.0)
            * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        }
        // System.out.println("xSpeed: " + xSpeed);
        // System.out.println("ySpeed: " + ySpeed);
        // System.out.println("thetaSpeed: " + thetaSpeed);
        
        drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed,
                        ySpeed,
                        thetaSpeed,
                        drivetrainSubsystem.getPose().getRotation())
        );
    }

    public void toggleTargetLock() {
        targetLock = !targetLock;
    }

    public boolean isTargetLock() {
        return targetLock;
    }

    private double lockToTargetWhileMoving() {
        return targetLockPID.calculate(
                VisionSubsystem.getX(),
                Math.toDegrees(Math.atan(ShooterConstants.TARGET_OFFSET / VisionSubsystem.getDistance())
                        + moveAndShootController.calculateAngleOffset())
        );
    }
    private double lockToTarget() {
        return targetLockPID.calculate(
                VisionSubsystem.getX(),
                Math.toDegrees(Math.atan(ShooterConstants.TARGET_OFFSET / VisionSubsystem.getDistance()))
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}