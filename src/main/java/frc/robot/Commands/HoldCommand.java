package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ManipulatorSubsystem;

public class HoldCommand extends Command{
    
    private ManipulatorSubsystem manipulatorSubsystem;

    public HoldCommand(ManipulatorSubsystem manipulatorSubsystem)
    {
        this.manipulatorSubsystem = manipulatorSubsystem;

        addRequirements(manipulatorSubsystem);
    }

    @Override
    public void execute()
    {
        manipulatorSubsystem.hold();
    }

    @Override
    public void end(boolean interrupted)
    {
        manipulatorSubsystem.stop();
    }
}
