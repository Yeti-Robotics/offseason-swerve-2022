// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.utils.AxisToButton;
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

	private final MoveAndShootController moveAndShootController = new MoveAndShootController(drivetrainSubsystem);
	private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(drivetrainSubsystem, moveAndShootController);
	private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	private final NeckSubsystem neckSubsystem = new NeckSubsystem();

	public final GenericHID driverStationJoystick;
	public final OIConstants.CONTROLLER controllerType = OIConstants.CONTROLLER.XBOX;

	public RobotContainer() {
		driverStationJoystick = controllerType == OIConstants.CONTROLLER.XBOX ? new XboxController(OIConstants.DRIVER_STATION_JOY)
				: new Joystick(OIConstants.DRIVER_STATION_JOY);

		// The controls are for field-oriented driving:
		// Left stick Y axis -> forward and backwards movement
		// Left stick X axis -> left and right movement
		// Right stick X axis -> rotation
		drivetrainSubsystem.setDefaultCommand(new FieldOrientedDrive(
				drivetrainSubsystem,
				moveAndShootController,
				this::getLeftY,
				this::getLeftX,
				this::getRightX)
		);

		if (controllerType == OIConstants.CONTROLLER.CUSTOM) {
			configureButtonBindings();
		} else {
			configureButtonXboxBindings();
		}
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

	}

	private void configureButtonXboxBindings() {
		setAxisWhileHeld(driverStationJoystick, Axis.kLeftTrigger.value, new AllOutCommand(intakeSubsystem, neckSubsystem));
		setAxisWhileHeld(driverStationJoystick, Axis.kRightTrigger.value, new AllinCommand(intakeSubsystem, neckSubsystem));
		setButtonWhenPressed(driverStationJoystick, Button.kRightStick.value, new ToggleIntakeCommand(intakeSubsystem));

		setButtonWhenPressed(driverStationJoystick, Button.kRightBumper.value, new ToggleShooterCommand(25, shooterSubsystem));
		setButtonWhenPressed(driverStationJoystick, Button.kLeftBumper.value, new InstantCommand(FieldOrientedDrive::toggleTargetLock));

		setButtonWhenPressed(driverStationJoystick, Button.kY.value, new InstantCommand(() -> drivetrainSubsystem.resetOdometer(
				new Pose2d(FieldConstants.launchPadB, new Rotation2d(0.0)).transformBy(DriveConstants.ROBOT_CENTER)
		)));

		setButtonWhenPressed(driverStationJoystick, Button.kB.value, new InstantCommand(drivetrainSubsystem::toggleSwerveLock));
	}

	private double getLeftY() {
		return controllerType == OIConstants.CONTROLLER.XBOX ? driverStationJoystick.getRawAxis(Axis.kLeftY.value)
				: driverStationJoystick.getRawAxis(1);
	}

	private double getLeftX() {
		return controllerType == OIConstants.CONTROLLER.XBOX ? driverStationJoystick.getRawAxis(Axis.kLeftX.value)
				: driverStationJoystick.getRawAxis(2);
	}

	private double getRightY() {
		return controllerType == OIConstants.CONTROLLER.XBOX ? driverStationJoystick.getRawAxis(Axis.kRightY.value)
				: driverStationJoystick.getRawAxis(3);
	}

	private double getRightX() {
		return controllerType == OIConstants.CONTROLLER.XBOX ? driverStationJoystick.getRawAxis(Axis.kRightX.value)
				: driverStationJoystick.getRawAxis(4);
	}

	private void setButtonWhenPressed(GenericHID genericHID, int button, CommandBase command) {
		new JoystickButton(genericHID, button).whenPressed(command);
	}

	private void setButtonWhileHeld(GenericHID genericHID, int button, CommandBase command) {
		new JoystickButton(genericHID, button).whileHeld(command);
	}

	private void setAxisWhenPressed(GenericHID genericHID, int port, CommandBase command) {
		new AxisToButton(genericHID, port, 0.75).whenPressed(command);
	}

	private void setAxisWhileHeld(GenericHID genericHID, int port, CommandBase command) {
		new AxisToButton(genericHID, port, 0.25).whileHeld(command);
	}
}
