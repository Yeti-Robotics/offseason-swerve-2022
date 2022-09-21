package frc.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class FieldOrientedDrive extends CommandBase {
	private final DrivetrainSubsystem m_drivetrainSubsystem;

	private final DoubleSupplier m_translationXSupplier;
	private final DoubleSupplier m_translationYSupplier;
	private final DoubleSupplier m_rotationSupplier;

	private final SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

	public FieldOrientedDrive(DrivetrainSubsystem drivetrainSubsystem,
			DoubleSupplier translationXSupplier,
			DoubleSupplier translationYSupplier,
			DoubleSupplier rotationSupplier) {
		this.m_drivetrainSubsystem = drivetrainSubsystem;
		this.m_translationXSupplier = translationXSupplier;
		this.m_translationYSupplier = translationYSupplier;
		this.m_rotationSupplier = rotationSupplier;

		this.xLimiter = new SlewRateLimiter(DriveConstants.MAX_ACCELERATION);
		this.yLimiter = new SlewRateLimiter(DriveConstants.MAX_ACCELERATION);
		this.thetaLimiter = new SlewRateLimiter(DriveConstants.MAX_ANGULAR_ACCELERATION);

		addRequirements(drivetrainSubsystem);
	}

	@Override
	public void execute() {
		double xSpeed = m_translationXSupplier.getAsDouble();
		double ySpeed = -m_translationYSupplier.getAsDouble();
		double thetaSpeed = -m_rotationSupplier.getAsDouble();

		xSpeed = xLimiter.calculate(Math.abs(xSpeed) > OIConstants.DEADBAND ? xSpeed : 0.0)
				* DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
		xSpeed = yLimiter.calculate(Math.abs(ySpeed) > OIConstants.DEADBAND ? ySpeed : 0.0)
				* DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
		xSpeed = thetaLimiter.calculate(Math.abs(thetaSpeed) > OIConstants.DEADBAND ? thetaSpeed : 0.0)
				* DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

		m_drivetrainSubsystem.drive(
				ChassisSpeeds.fromFieldRelativeSpeeds(
						m_translationXSupplier.getAsDouble(),
						m_translationYSupplier.getAsDouble(),
						m_rotationSupplier.getAsDouble(),
						m_drivetrainSubsystem.getPose().getRotation())
		);
	}

	@Override
	public void end(boolean interrupted) {
		m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
	}
}