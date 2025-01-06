package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.ManipulatorSubsystem;
import frc.robot.Subsystems.WristSubsystem;

public class MindControlCommand extends Command
{
    private DoubleSupplier m_pov;
    private ElevatorSubsystem m_elevator;
    private WristSubsystem m_wrist;
    private ManipulatorSubsystem m_manipulator;
    private BooleanSupplier m_button1pressed;
    private BooleanSupplier m_button2pressed;
    private BooleanSupplier m_button3pressed;
    private BooleanSupplier m_button4pressed;
    private BooleanSupplier m_button10pressed;

    
    public MindControlCommand(DoubleSupplier pov, BooleanSupplier button1pressed, BooleanSupplier button2pressed, BooleanSupplier button3pressed, BooleanSupplier button4pressed, BooleanSupplier button10pressed, ElevatorSubsystem elevator, WristSubsystem wrist, ManipulatorSubsystem manipulatorSubsystem)
    {
        m_pov = pov;
        m_elevator = elevator;
        m_wrist = wrist;
        m_button1pressed = button1pressed;
        m_button2pressed = button2pressed;
        m_button3pressed = button3pressed;
        m_button4pressed = button4pressed;
        m_manipulator = manipulatorSubsystem;
        m_button10pressed = button10pressed;
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

            if (m_button3pressed.getAsBoolean()) // left button on joystick
            {
                new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.PICKUP_ALGAE_L1, Constants.WristConstants.PICKUP_ALGAE_L1).wait();
            }
            
            if (m_button4pressed.getAsBoolean()) // right button on joystick
            {
                new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.PICKUP_ALGAE_L2, Constants.ElevatorConstants.PICKUP_ALGAE_L2).wait();
            }
            
            if (m_button1pressed.getAsBoolean()) // trigger
            {
                new ManipulatorCommand(m_manipulator, false).wait();
            }
            else if (m_button2pressed.getAsBoolean()) // thumb button
            {
                new ManipulatorCommand(m_manipulator, true).wait();
            }
            
            if (m_button10pressed.getAsBoolean())
            {
                new ParallelCommandGroup(new DefaultElevatorCommand(m_elevator), new DefaultWristCommand(m_wrist)).wait();
            }
        }
        catch (InterruptedException e)
        {
            // pass
        }
    }
}
