package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;

public class moveElevator extends Command
{
    private final ElevatorSubsystem m_elevator;
    private final DoubleSupplier m_speed;

    public moveElevator(DoubleSupplier speed, ElevatorSubsystem subsystem)
    {
        addRequirements(subsystem);
        m_elevator = subsystem;
        m_speed = speed;
    }

    @Override
    public void execute()
    {
        // this is where you call the subsystem!
        // m_elevator.moveElevator(m_speed.getAsDouble());
    }
}
