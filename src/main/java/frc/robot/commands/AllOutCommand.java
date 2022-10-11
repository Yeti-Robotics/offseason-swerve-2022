package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NeckSubsystem;


public class AllOutCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final NeckSubsystem neckSubsystem;

    public AllOutCommand(IntakeSubsystem intakeSubsystem, NeckSubsystem neckSubsystem) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.intakeSubsystem = intakeSubsystem;
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        neckSubsystem.moveDown();
        intakeSubsystem.rollOut();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        neckSubsystem.stopNeck();
        intakeSubsystem.stopRoll();
    }
}
