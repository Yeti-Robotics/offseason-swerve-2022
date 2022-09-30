// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AllOutCommand;
import frc.robot.commands.AllinCommand;
import frc.robot.commands.drivetrain.FieldOrientedDrive;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NeckSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;
import frc.robot.utils.JoyButton;
import frc.robot.utils.JoyButton.ActiveState;
import frc.robot.utils.MoveAndShootController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	public final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
	private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(drivetrainSubsystem);
	private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	private final NeckSubsystem neckSubsystem = new NeckSubsystem();

	public final XboxController driverStationJoystick;

	public RobotContainer() {
		driverStationJoystick = new XboxController(OIConstants.DRIVER_STATION_JOY);

		// The controls are for field-oriented driving:
		// Left stick Y axis -> forward and backwards movement
		// Left stick X axis -> left and right movement
		// Right stick X axis -> rotation
		drivetrainSubsystem.setDefaultCommand(
				new FieldOrientedDrive(drivetrainSubsystem,
						this::getLeftY,
						this::getLeftX,
						this::getRightX));

		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		setButtonWhenPressed(driverStationJoystick, Button.kA.value, new ToggleIntakeCommand(intakeSubsystem));
		setButtonWhileHeld(driverStationJoystick, Button.kB.value, new IntakeInCommand(intakeSubsystem));
		setButtonWhenPressed(driverStationJoystick, Button.kLeftBumper.value, new ToggleShooterCommand(10.0, shooterSubsystem));
		setButtonWhileHeld(driverStationJoystick, Button.kRightBumper.value, new AllinCommand(intakeSubsystem, neckSubsystem));

		// setConditionalButton(2, new ToggleShooterCommand(ShooterMode.LIMELIGHT, shooterSubsystem),
		// // false currently cannot ocurr, check setConditionalButton
		// 		ActiveState.WHEN_PRESSED, new InstantCommand(), ActiveState.WHEN_PRESSED);

		// setConditionalButton(1,
		// 		new AllinCommand(intakeSubsystem, neckSubsystem), ActiveState.WHILE_HELD,
		// 		new InstantCommand(), ActiveState.WHEN_PRESSED);

		// setConditionalButton(6,
		// 		new AllOutCommand(intakeSubsystem, neckSubsystem), ActiveState.WHILE_HELD,
		// 		new InstantCommand(), ActiveState.WHEN_PRESSED);
	}

	private double getLeftY() {
		return driverStationJoystick.getLeftY();
	}

	private double getLeftX() {
		return driverStationJoystick.getLeftX();
	}

	private double getRightY() {
		return driverStationJoystick.getRightY();
	}

	private double getRightX() {
		return driverStationJoystick.getRightX();
	}

	private void setButtonWhenPressed(GenericHID genericHID, int button, CommandBase command) {
		new JoystickButton(genericHID, button).whenPressed(command);
	}

	private void setButtonWhileHeld(GenericHID genericHID, int button, CommandBase command) {
		new JoystickButton(genericHID, button).whileHeld(command);
	}

	// private void setConditionalButton(
	// 		int button,
	// 		Command trueCommand,
	// 		ActiveState trueActiveState,
	// 		Command falseCommand,
	// 		ActiveState falseActiveState) {
	// 	driverStationJoystick.
	// 			.conditionalPressed(
	// 					// currently only runs the true command
	// 					trueCommand, trueActiveState, falseCommand, falseActiveState, () -> true);
	// }
}
