// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drivetrain.FieldOrientedDrive;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NeckSubsystem;
import frc.robot.utils.AxisToButton;
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
    public final GenericHID controller;
    public final OIConstants.CONTROLLER controllerType = OIConstants.CONTROLLER.CUSTOM;
    public final MoveAndShootController moveAndShootController = new MoveAndShootController(drivetrainSubsystem);
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final NeckSubsystem neckSubsystem = new NeckSubsystem();

    public RobotContainer() {
        controller = controllerType == OIConstants.CONTROLLER.XBOX ? new XboxController(OIConstants.DRIVER_STATION_JOY)
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
//        setButtonWhenPressed(controller, 1, new AllinCommand(intakeSubsystem, neckSubsystem));
//        setButtonWhenPressed(controller, 6, new AllOutCommand(intakeSubsystem, neckSubsystem));
//        setButtonWhenPressed(controller, 12, new ToggleIntakeCommand(intakeSubsystem));
//
//        setButtonWhenPressed(controller, 2, new ToggleShooterCommand(ShooterSubsystem.ShooterMode.LIMELIGHT, shooterSubsystem));
//        setButtonWhenPressed(controller, 3, new ToggleShooterCommand(ShooterSubsystem.ShooterMode.LOWGOAL, shooterSubsystem));
//
//        setButtonWhenPressed(controller, 11, new InstantCommand(FieldOrientedDrive::toggleTargetLock));
//        setButtonWhenPressed(controller, 10, new InstantCommand(moveAndShootController::toggleMoveAndShootController));

        setButtonWhenPressed(controller, 12, new InstantCommand(() -> drivetrainSubsystem.resetOdometer(
            new Pose2d(FieldConstants.launchPadB, new Rotation2d(0.0)).transformBy(DriveConstants.ROBOT_CENTER)
        )));

        setButtonWhenPressed(controller, 11, new InstantCommand(drivetrainSubsystem::toggleSwerveLock));
    }

    private void configureButtonXboxBindings() {
//        setAxisWhileHeld(controller, Axis.kLeftTrigger.value, new AllOutCommand(intakeSubsystem, neckSubsystem));
//        setAxisWhileHeld(controller, Axis.kRightTrigger.value, new AllinCommand(intakeSubsystem, neckSubsystem));
//        setButtonWhenPressed(controller, Button.kLeftBumper.value, new ToggleIntakeCommand(intakeSubsystem));
//
////        setButtonWhenPressed(controller, Button.kRightBumper.value, new ToggleShooterCommand(ShooterSubsystem.ShooterMode.LIMELIGHT, shooterSubsystem));
////        setButtonWhenPressed(controller, Button.kX.value, new ToggleShooterCommand(ShooterSubsystem.ShooterMode.LOWGOAL, shooterSubsystem));
//
//        setButtonWhenPressed(controller, Button.kA.value, new InstantCommand(FieldOrientedDrive::toggleTargetLock));
//        setButtonWhenPressed(controller, Button.kBack.value, new InstantCommand(moveAndShootController::toggleMoveAndShootController));

//        setButtonWhenPressed(controller, Button.kY.value, new AutoBalancing(drivetrainSubsystem));

        setButtonWhenPressed(controller, Button.kStart.value, new InstantCommand(() -> drivetrainSubsystem.resetOdometer(
            new Pose2d(FieldConstants.launchPadB, new Rotation2d(0.0))
        )));

        setButtonWhenPressed(controller, Button.kLeftBumper.value, new InstantCommand(drivetrainSubsystem::toggleSwerveLock));
    }

    private double getLeftY() {
        return controllerType == OIConstants.CONTROLLER.XBOX ? -controller.getRawAxis(Axis.kLeftY.value)
            : -controller.getRawAxis(0);
    }

    private double getLeftX() {
        return controllerType == OIConstants.CONTROLLER.XBOX ? -controller.getRawAxis(Axis.kLeftX.value)
            : controller.getRawAxis(1);
    }

    private double getRightY() {
        return controllerType == OIConstants.CONTROLLER.XBOX ? -controller.getRawAxis(Axis.kRightY.value)
            : -controller.getRawAxis(2);
    }

    private double getRightX() {
        return controllerType == OIConstants.CONTROLLER.XBOX ? -controller.getRawAxis(Axis.kRightX.value)
            : controller.getRawAxis(3);
    }

    private void setButtonWhenPressed(GenericHID genericHID, int button, CommandBase command) {
        new JoystickButton(genericHID, button).onTrue(command);
    }

    private void setButtonWhileHeld(GenericHID genericHID, int button, CommandBase command) {
        new JoystickButton(genericHID, button).whileTrue(command);
    }

    private void setAxisWhenPressed(GenericHID genericHID, int port, CommandBase command) {
        new AxisToButton(genericHID, port, 0.25).onTrue(command);
    }

    private void setAxisWhileHeld(GenericHID genericHID, int port, CommandBase command) {
        new AxisToButton(genericHID, port, 0.25).whileTrue(command);
    }
}
