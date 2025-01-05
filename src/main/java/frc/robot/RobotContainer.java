package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.*;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;

public class RobotContainer 
{
	private final Joystick m_controller = new Joystick(Constants.DrivetrainConstants.controller_port);
	private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
	private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
	
	public RobotContainer() 
	{
		configureBindings();
	}

	private void configureBindings() 
	{
		m_drivetrain.setDefaultCommand(
			new driveArcade(() -> m_controller.getY(), () -> m_controller.getX(), m_drivetrain));
		m_elevator.setDefaultCommand(
			new moveElevator(() -> m_controller.getPOV(), m_elevator));
	}

	public Command getAutonomousCommand() 
	{
		return Commands.print("No autonomous command configured");
	}
}
