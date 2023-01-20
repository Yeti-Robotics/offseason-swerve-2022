package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Drivetrain.SwerveModule;

public class AutoBalancing extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;
    private PIDController pidController;

    public AutoBalancing(DrivetrainSubsystem drivetrainSubsystem){
        this.drivetrainSubsystem = drivetrainSubsystem;
        pidController = new PIDController(Constants.DriveConstants.DRIVE_PITCH_P,
                Constants.DriveConstants.DRIVE_PITCH_I,
                Constants.DriveConstants.DRIVE_PITCH_D,
                Constants.DriveConstants.DRIVE_PITCH_F);
        pidController.setTolerance(1);
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double translationValue = MathUtil.clamp(
                pidController.calculate(
                        drivetrainSubsystem.getPitch().getDegrees(), 0.0), -0.1, -0.1);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveModule[] swerveModules = drivetrainSubsystem.getModules();
        swerveModules[0].setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(45.0))));
        swerveModules[1].setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(315.0))));
        swerveModules[2].setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(315.0))));
        swerveModules[3].setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(45.0))));

        }

    @Override
    public boolean isFinished() {
        return false;
    }
}

