package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

public class RobotContainer 
{
	private final CommandJoystick m_controller = new CommandJoystick(Constants.DrivetrainConstants.controller_port);
	private final CommandJoystick m_testcontroller = new CommandJoystick(1);
	private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
	private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
	private final WristSubsystem m_wrist = new WristSubsystem();
	private final ManipulatorSubsystem m_manipulator = new ManipulatorSubsystem();

	public RobotContainer() 
	{
		configureBindings();
	}

	private void configureBindings() 
	{
		m_drivetrain.setDefaultCommand(
			new driveArcade(() -> m_controller.getY(), () -> m_controller.getX(), m_drivetrain));
		
		// m_elevator.setDefaultCommand(new moveElevatorCommand(() -> m_testcontroller.getY(), m_elevator));

		// m_wrist.setDefaultCommand(new moveWristCommand(() -> m_testcontroller.getX(), m_wrist));

		m_wrist.setDefaultCommand(new DefaultWristCommand(m_wrist));
		
		m_controller.povUp().whileTrue(new ElevatorToCommand(m_elevator, m_wrist, -36, -13.5));

		// m_controller.povUp().whileTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.L3, Constants.WristConstants.L3));

		// m_controller.povLeft().onTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.L2, Constants.WristConstants.L2));

		// m_controller.povDown().onTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.PLACE_ALGAE, Constants.WristConstants.PLACE_ALGAE));

		// m_controller.povRight().onTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.HUMAN_PICKUP, Constants.WristConstants.HUMAN_PICKUP));

		// m_controller.button(3).onTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.PICKUP_ALGAE_L1, Constants.WristConstants.PICKUP_ALGAE_L1));

		// m_controller.button(4).onTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.PICKUP_ALGAE_L2, Constants.ElevatorConstants.PICKUP_ALGAE_L2));

		m_controller.button(1).whileTrue(new ManipulatorCommand(m_manipulator, false));

		m_controller.button(2).whileTrue(new ManipulatorCommand(m_manipulator, true));

		// m_controller.button(10).onTrue(new ParallelCommandGroup(new DefaultElevatorCommand(m_elevator), new DefaultWristCommand(m_wrist)));
	}

	public Command getAutonomousCommand() 
	{
		return Commands.print("No autonomous command configured");
	}
}
