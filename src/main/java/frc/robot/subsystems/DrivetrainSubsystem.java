// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

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
	/**
	 * The maximum voltage that will be delivered to the drive motors.
	 * <p>
	 * This can be reduced to cap the robot's maximum speed. Typically, this is
	 * useful during initial testing of the robot.
	 */
	public static final double MAX_VOLTAGE = 12.0;

	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			// Front left
			new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
					DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Front right
			new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
					-DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back left
			new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
					DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back right
			new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
					-DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

	private final AHRS gyro = new AHRS(Port.kUSB); // NavX

	// These are our modules. We initialize them in the constructor.
	private final SwerveModule frontLeftModule;
	private SwerveModuleState frontLeftModuleState;
	private final SwerveModule frontRightModule;
	private SwerveModuleState frontRightModuleState;
	private final SwerveModule backLeftModule;
	private SwerveModuleState backLeftModuleState;
	private final SwerveModule backRightModule;
	private SwerveModuleState backRightModuleState;

	private final PIDController yController = new PIDController(AutoConstants.Y_CONTROLLER_P, 0.0, 0.0);
	private final PIDController xController = new PIDController(AutoConstants.X_CONTROLLER_P, 0.0, 0.0);
	private final ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.THETA_CONTROLLER_P, 0.0, 0.0, AutoConstants.THETA_CONTROLLER_CONTRAINTS);

	private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
	private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, new Rotation2d(0));

	public DrivetrainSubsystem() {
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

		frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
				// This parameter is optional, but will allow you to see the current state of
				// the module on the dashboard.
				tab.getLayout("Front Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(0, 0),
				// This can either be STANDARD or FAST depending on your gear configuration
				Mk4SwerveModuleHelper.GearRatio.L2,
				DriveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
				DriveConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
				DriveConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
				// This is how much the steer encoder is offset from true zero (In our case,
				// zero is facing straight forward)
				DriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET);
		frontLeftModuleState = new SwerveModuleState();

		// We will do the same for the other modules
		frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
				tab.getLayout("Front Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(2, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				DriveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
				DriveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
				DriveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
				DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);
		frontRightModuleState = new SwerveModuleState();

		backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(4, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				DriveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
				DriveConstants.BACK_LEFT_MODULE_STEER_MOTOR,
				DriveConstants.BACK_LEFT_MODULE_STEER_ENCODER,
				DriveConstants.BACK_LEFT_MODULE_STEER_OFFSET);
		backLeftModuleState = new SwerveModuleState();

		backRightModule = Mk4SwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(6, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				DriveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
				DriveConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
				DriveConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
				DriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET);
		backRightModuleState = new SwerveModuleState();

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
		gyro.zeroYaw();
		// uncomment if switch to pigeon
		// gyro.setFusedHeading(0.0);
	}

	public Rotation2d getGyroscopeRotation() {
		if (gyro.isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(gyro.getFusedHeading());
		}

		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - gyro.getYaw());

		// uncomment if switch to pigeon
		// return Rotation2d.fromDegrees(gyro.getFusedHeading());
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

	private void updateSwerveModuleStates() {
		frontLeftModuleState.speedMetersPerSecond = frontLeftModule.getDriveVelocity();
		frontLeftModuleState.angle = Rotation2d.fromDegrees(frontLeftModule.getSteerAngle());

		frontRightModuleState.speedMetersPerSecond = frontRightModule.getDriveVelocity();
		frontRightModuleState.angle = Rotation2d.fromDegrees(frontRightModule.getSteerAngle());

		backLeftModuleState.speedMetersPerSecond = backLeftModule.getDriveVelocity();
		backLeftModuleState.angle = Rotation2d.fromDegrees(backLeftModule.getSteerAngle());

		backRightModuleState.speedMetersPerSecond = backRightModule.getDriveVelocity();
		backRightModuleState.angle = Rotation2d.fromDegrees(backRightModule.getSteerAngle());
	}

	public void setDesiredStates(SwerveModuleState[] desiredStates) {
		frontLeftModule.set(desiredStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				desiredStates[0].angle.getRadians());
		frontRightModule.set(desiredStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				desiredStates[1].angle.getRadians());
		backLeftModule.set(desiredStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				desiredStates[2].angle.getRadians());
		backRightModule.set(desiredStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				desiredStates[3].angle.getRadians());
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		this.chassisSpeeds = chassisSpeeds;
	}

	@Override
	public void periodic() {
		updateSwerveModuleStates();
		odometer.update(getGyroscopeRotation(), frontLeftModuleState, frontRightModuleState, backLeftModuleState, backRightModuleState);

		SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);
		setDesiredStates(desiredStates);
	}
}