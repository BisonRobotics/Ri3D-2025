package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class drive extends Command
{
    private final Drivetrain m_drivetrain;
    private final DoubleSupplier m_leftspeed;
    private final DoubleSupplier m_rightspeed;

    public drive(DoubleSupplier leftspeed, DoubleSupplier rightspeed, Drivetrain subsystem)
    {
        addRequirements(subsystem);
        m_leftspeed = leftspeed;
        m_rightspeed = rightspeed;
        m_drivetrain = subsystem;
    }

    @Override
    public void execute()
    {
        SmartDashboard.putNumber("Left Speed", m_leftspeed.getAsDouble());
        SmartDashboard.putNumber("Right Speed", m_rightspeed.getAsDouble());
        m_drivetrain.drive(
            MathUtil.applyDeadband(
                m_leftspeed.getAsDouble(),
                Constants.Drivetrain.stickDeadband
                ), MathUtil.applyDeadband(
                    m_rightspeed.getAsDouble(),
                    Constants.Drivetrain.stickDeadband
                    ));
    }
}
