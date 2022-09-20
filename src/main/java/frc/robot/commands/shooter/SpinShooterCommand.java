// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinShooterCommand extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final double power;
  private double maxVel = 0.0;

  /** Creates a new SpinShooterCommand. */
  public SpinShooterCommand(ShooterSubsystem shooterSubsystem, double power) {
    this.shooterSubsystem = shooterSubsystem;
    this.power = power;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.shootFlywheel(power);

    double currVel = shooterSubsystem.getAverageEncoder();
    if (currVel > maxVel) {
      maxVel = currVel;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopFlywheel();
    // System.out.println("MAX VEL AT " + (power * 100) + "%: " + maxVel);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
