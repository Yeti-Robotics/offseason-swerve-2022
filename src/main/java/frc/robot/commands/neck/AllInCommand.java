package frc.robot.commands.neck;

import frc.robot.subsystems.NeckSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AllInCommand extends CommandBase {

    private final NeckSubsystem neckSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public AllInCommand(IntakeSubsystem intakeSubsystem, NeckSubsystem neckSubsystem) {
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
    public void execute() { //incomplete - need shooter subsystem to completely finish logic
        intakeSubsystem.rollIn();
        neckSubsystem.moveUp();
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
