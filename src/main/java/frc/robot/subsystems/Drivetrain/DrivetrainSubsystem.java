// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;

public class DrivetrainSubsystem extends SubsystemBase {
	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			// Front right
			new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
					-DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Front left
			new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
					DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back left
			new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
					DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back right
			new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
					-DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

	private final AHRS gyro = new AHRS(Port.kUSB); // NavX

	// These are our modules. We initialize them in the constructor.
	private final SwerveModule frontRightModule;
	private final SwerveModule frontLeftModule;
	private final SwerveModule backRightModule;
	private final SwerveModule backLeftModule;

	private final PIDController yController = new PIDController(AutoConstants.Y_CONTROLLER_P, 0.0, 0.0);
	private final PIDController xController = new PIDController(AutoConstants.X_CONTROLLER_P, 0.0, 0.0);
	private final ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.THETA_CONTROLLER_P, 0.0, 0.0, AutoConstants.THETA_CONTROLLER_CONTRAINTS);

	private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
	private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, new Rotation2d(0));

	public DrivetrainSubsystem() {
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

		frontRightModule = new SwerveModule(
				DriveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
				DriveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
				DriveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
				false,
				DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

		frontLeftModule = new SwerveModule(
				DriveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
				DriveConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
				DriveConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
				false,
				DriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET);

		backLeftModule = new SwerveModule(
				DriveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
				DriveConstants.BACK_LEFT_MODULE_STEER_MOTOR,
				DriveConstants.BACK_LEFT_MODULE_STEER_ENCODER,
				false,
				DriveConstants.BACK_LEFT_MODULE_STEER_OFFSET);

		backRightModule = new SwerveModule(
				DriveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
				DriveConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
				DriveConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
				false,
				DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

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

		// uncomment if switch to pigeon
		// gyro.setFusedHeading(0.0);
	}

	public Rotation2d getGyroscopeRotation() {
		return Rotation2d.fromDegrees(Math.abs(gyro.getYaw() - 180));
	}

	public Pose2d getPose() {
		return odometer.getPoseMeters();
	}

	public void resetOdometer(Pose2d pose) {
		odometer.resetPosition(pose, getGyroscopeRotation());
	}

	public SwerveDriveKinematics getKinematics() {
		return kinematics;
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
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);
		frontRightModule.setDesiredState(desiredStates[0]);
		frontLeftModule.setDesiredState(desiredStates[1]);
		backLeftModule.setDesiredState(desiredStates[2]);
		backRightModule.setDesiredState(desiredStates[3]);
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		this.chassisSpeeds = chassisSpeeds;
		setDesiredStates(kinematics.toSwerveModuleStates(chassisSpeeds));
	}

	public ChassisSpeeds getChassisSpeeds() {
		return chassisSpeeds;
	}

	@Override
	public void periodic() {
		odometer.update(getGyroscopeRotation(),
				frontRightModule.getState(), frontLeftModule.getState(),
				backLeftModule.getState(), backRightModule.getState());

		System.out.println("GYRO: " + getGyroscopeRotation());
	}
}