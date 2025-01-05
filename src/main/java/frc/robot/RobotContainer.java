package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.driveArcade;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer 
{
	private final Joystick m_controller = new Joystick(Constants.Drivetrain.controller_port);
	private final Drivetrain m_drivetrain = new Drivetrain();
	
	public RobotContainer() 
	{
		configureBindings();
	}

	private void configureBindings() 
	{
		m_drivetrain.setDefaultCommand(
			new driveArcade(() -> m_controller.getY(), () -> m_controller.getX(), m_drivetrain));
	}

	public Command getAutonomousCommand() 
	{
		return Commands.print("No autonomous command configured");
	}
}
