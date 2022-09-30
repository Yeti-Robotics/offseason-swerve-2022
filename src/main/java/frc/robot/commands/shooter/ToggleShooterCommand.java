package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;

public class ToggleShooterCommand extends InstantCommand {
	private final ShooterMode shooterMode;
	private final ShooterSubsystem shooterSubsystem;
	private double setPoint;

	public ToggleShooterCommand(ShooterMode shooterMode, ShooterSubsystem shooterSubsystem) {
		this.shooterMode = shooterMode;
		this.shooterSubsystem = shooterSubsystem;
	}

	public ToggleShooterCommand(double setPoint, ShooterSubsystem shooterSubsystem) {
		this.setPoint = setPoint;
		this.shooterMode = ShooterMode.MANUAL;
		this.shooterSubsystem = shooterSubsystem;
	}

	@Override
	public void execute() {
		if (shooterSubsystem.getShooterMode() == shooterMode) {
			shooterSubsystem.setShooterMode(ShooterMode.OFF);
		} else {
			shooterSubsystem.setShooterMode(shooterMode);
			if (shooterMode == ShooterMode.MANUAL)
				shooterSubsystem.setSetPoint(setPoint);
		}
	}
}
