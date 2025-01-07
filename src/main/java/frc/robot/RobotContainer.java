package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
			new driveArcade(() -> m_controller.getY(), () -> m_controller.getTwist() / 2, m_drivetrain));
		
		// for manual testing
		// m_elevator.setDefaultCommand(new moveElevatorCommand(() -> m_testcontroller.getY(), m_elevator));

		// m_wrist.setDefaultCommand(new moveWristCommand(() -> m_controller.getX(), m_wrist));

		m_wrist.setDefaultCommand(new DefaultWristCommand(m_wrist));

		m_controller.button(5).whileTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.L3, Constants.WristConstants.L3)).onFalse(new DefaultWristCommand(m_wrist));

		m_controller.button(6).whileTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.L2, Constants.WristConstants.L2)).onFalse(new DefaultWristCommand(m_wrist));

		m_controller.button(10).whileTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.PLACE_ALGAE, Constants.WristConstants.PLACE_ALGAE)).onFalse(new DefaultWristCommand(m_wrist));

		m_controller.button(8).whileTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.HUMAN_PICKUP, Constants.WristConstants.HUMAN_PICKUP)).onFalse(new DefaultWristCommand(m_wrist));
		
		m_controller.button(3).whileTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.PICKUP_ALGAE_L1, Constants.WristConstants.PICKUP_ALGAE_L1)).onFalse(new DefaultWristCommand(m_wrist));

		m_controller.button(4).whileTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.PICKUP_ALGAE_L2, Constants.WristConstants.PICKUP_ALGAE_L2)).onFalse(new DefaultWristCommand(m_wrist));

		m_controller.button(7).whileTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.L1, Constants.WristConstants.L1)).onFalse(new DefaultWristCommand(m_wrist));
		
		// coral intake, algae shoot
		m_controller.button(1).whileTrue(new ManipulatorCommand(m_manipulator, false)).toggleOnFalse(new HoldCommand(m_manipulator, false, false));

		// algae intake, coral shoot
		m_controller.button(2).whileTrue(new ManipulatorCommand(m_manipulator, true)).toggleOnFalse(new HoldCommand(m_manipulator, false, true));

		m_controller.povUp().onTrue(new StopCommand(m_manipulator));

	}

	public Command getAutonomousCommand() 
	{
		return Commands.print("No autonomous command configured");
	}
}
