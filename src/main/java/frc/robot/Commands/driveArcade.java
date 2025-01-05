package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class driveArcade extends Command
{
    private final DrivetrainSubsystem m_drivetrain;
    private final DoubleSupplier m_leftspeed;
    private final DoubleSupplier m_rightspeed;

    public driveArcade(DoubleSupplier leftspeed, DoubleSupplier rightspeed, DrivetrainSubsystem subsystem)
    {
        addRequirements(subsystem);
        m_leftspeed = leftspeed;
        m_rightspeed = rightspeed;
        m_drivetrain = subsystem;
    }

    @Override
    public void execute()
    {
        m_drivetrain.driveArcade(
            MathUtil.applyDeadband(
                m_leftspeed.getAsDouble(),
                Constants.DrivetrainConstants.stickDeadband
                ), MathUtil.applyDeadband(
                    m_rightspeed.getAsDouble(),
                    Constants.DrivetrainConstants.stickDeadband
                    ));
    }
}
