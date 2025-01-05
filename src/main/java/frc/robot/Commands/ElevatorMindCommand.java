package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.WristSubsystem;

public class ElevatorMindCommand extends Command
{
    private DoubleSupplier m_pov;
    private ElevatorSubsystem m_elevator;
    private WristSubsystem m_wrist;
    private BooleanSupplier m_button3pressed;
    private BooleanSupplier m_button4pressed;

    public ElevatorMindCommand(DoubleSupplier pov, BooleanSupplier button3pressed, BooleanSupplier button4pressed, ElevatorSubsystem elevator, WristSubsystem wrist)
    {
        m_pov = pov;
        m_elevator = elevator;
        m_wrist = wrist;
        m_button3pressed = button3pressed;
        m_button4pressed = button4pressed;
    }

    @Override
    public void execute()
    {
        try
        {
            switch ((int)m_pov.getAsDouble())
            {
    
                case 0: // UP
                    new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.L3, Constants.WristConstants.L3).wait();
                    break;
                case 270: // LEFT
                    new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.L2, Constants.WristConstants.L2).wait();
                    break;
                case 180: // DOWN
                    new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.PLACE_ALGAE, Constants.WristConstants.PLACE_ALGAE).wait();
                    break;
                case 90: // RIGHT
                    new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.HUMAN_PICKUP, Constants.WristConstants.HUMAN_PICKUP).wait();
                    break;
                case -1: // NOT INTERACTING
                    break;
                default:
                    break;
            }

            if (m_button3pressed.getAsBoolean())
            {
                new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.PICKUP_ALGAE_L1, Constants.WristConstants.PICKUP_ALGAE_L1).wait();
            }

            if (m_button4pressed.getAsBoolean())
            {
                new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.PICKUP_ALGAE_L2, Constants.ElevatorConstants.PICKUP_ALGAE_L2).wait();
            }
        }
        catch (InterruptedException e)
        {
            // pass
        }
    }
}
