// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

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

	private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, new Rotation2d(0));

	// PID controllers for tracking trajectory
    private PIDController xController = new PIDController(AutoConstants.X_CONTROLLER_P, 0, 0);
    private PIDController yController = new PIDController(AutoConstants.Y_CONTROLLER_P, 0, 0);
    private ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.THETA_CONTROLLER_P, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);

	private final AHRS gyro = new AHRS(Port.kUSB); // NavX

	private final SwerveModule frontLeftModule;
	private final SwerveModule frontRightModule;
	private final SwerveModule backLeftModule;
	private final SwerveModule backRightModule;

	private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

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

		frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
				tab.getLayout("Front Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(2, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				DriveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
				DriveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
				DriveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
				DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

		backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(4, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				DriveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
				DriveConstants.BACK_LEFT_MODULE_STEER_MOTOR,
				DriveConstants.BACK_LEFT_MODULE_STEER_ENCODER,
				DriveConstants.BACK_LEFT_MODULE_STEER_OFFSET);

		backRightModule = Mk4SwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(6, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				DriveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
				DriveConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
				DriveConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
				DriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET);

		// PI and -PI to be the same point and automatically calculates the shortest route to the setpoint
		// i.e. more efficient rotation
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		this.chassisSpeeds = chassisSpeeds;
	}

	public void stop() {
		this.chassisSpeeds = new ChassisSpeeds();
	}

	public Pose2d getPose() {
		return odometer.getPoseMeters();
	}


	public SwerveDriveKinematics getKinematics() {
		return kinematics;
	}

	public PIDController getXController() {
		return xController;
	}

	public PIDController getYController() {
		return yController;
	}

	public ProfiledPIDController getThetaController() {
		return thetaController; 
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

	public void resetOdometry(Pose2d pose) {
		odometer.resetPosition(pose, getGyroscopeRotation());
	}
	
	public void setModuleStates(SwerveModuleState[] states) {
		frontLeftModule.set(states[0].speedMetersPerSecond / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[0].angle.getRadians());
		frontRightModule.set(states[1].speedMetersPerSecond / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[1].angle.getRadians());
		backLeftModule.set(states[2].speedMetersPerSecond / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[2].angle.getRadians());
		backRightModule.set(states[3].speedMetersPerSecond / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[3].angle.getRadians());
	}

	@Override
	public void periodic() {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
		setModuleStates(states);
	}
}