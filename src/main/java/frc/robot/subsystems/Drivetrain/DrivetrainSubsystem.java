// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.AutoConstants;
import static frc.robot.Constants.DriveConstants;

public class DrivetrainSubsystem extends SubsystemBase {
    private final AHRS gyro = new AHRS(Port.kUSB); // NavX
    private final Rotation2d flipGyro = new Rotation2d(Math.PI);

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final SwerveModule[] modules;

    private final SwerveModulePosition[] positions = new SwerveModulePosition[4];

    private final PIDController yController = new PIDController(AutoConstants.Y_CONTROLLER_P, 0.0, 0.0);
    private final PIDController xController = new PIDController(AutoConstants.X_CONTROLLER_P, 0.0, 0.0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.THETA_CONTROLLER_P,
        0.0, 0.0, AutoConstants.THETA_CONTROLLER_CONTRAINTS);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, new Rotation2d(0), positions);
    private boolean isSwerveLock;
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DrivetrainSubsystem() {
        frontLeftModule = new SwerveModule(
            DriveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            true,
            DriveConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
            DriveConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
            false,
            DriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET);

        frontRightModule = new SwerveModule(
            DriveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            false,
            DriveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
            DriveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
            false,
            DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

        backLeftModule = new SwerveModule(
            DriveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            true,
            DriveConstants.BACK_LEFT_MODULE_STEER_MOTOR,
            DriveConstants.BACK_LEFT_MODULE_STEER_ENCODER,
            false,
            DriveConstants.BACK_LEFT_MODULE_STEER_OFFSET);

        backRightModule = new SwerveModule(
            DriveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            false,
            DriveConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
            DriveConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
            false,
            DriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET);

        modules = new SwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

        positions[0] = frontLeftModule.getPosition();
        positions[1] = frontRightModule.getPosition();
        positions[2] = backLeftModule.getPosition();
        positions[4] = backRightModule.getPosition();

        thetaController.enableContinuousInput(-Math.PI, Math.PI);



        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroGyroscope();
            } catch (Exception e) {
                System.out.println("FAILED TO ZERO GYROSCOPE");
            }
        }).start();
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        gyro.reset();
    }

    public Rotation2d getGyroscopeRotation() {
        // return gyro.getRotation2d();
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getRoll(){
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    public void resetOdometer(Pose2d pose) {
        odometer.resetPosition(getGyroscopeRotation(), positions, pose);
    }

    public PIDController getxController() {
        return xController;
    }

    public PIDController getyController() {
        return yController;
    }

    public ProfiledPIDController getThetaController() {
        return thetaController;
    }

    public void setDesiredStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        if (isSwerveLock) {
            swerveLock();
            return;
        }
        this.chassisSpeeds = chassisSpeeds;
        setDesiredStates(DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds));
    }

    public SwerveModule[] getModules(){
        return modules;
    }
    private void swerveLock() {
        if (chassisSpeeds.vxMetersPerSecond > 0.5 && chassisSpeeds.vyMetersPerSecond > 0.5) {
            isSwerveLock = false;
            return;
        }

        SwerveModuleState[] desiredStates = new SwerveModuleState[4];
        desiredStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        desiredStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
        desiredStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
        desiredStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        setDesiredStates(desiredStates);
    }

    public void toggleSwerveLock() {
        isSwerveLock = !isSwerveLock;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    @Override
    public void periodic() {
        odometer.update(getGyroscopeRotation(), positions);
    }
}