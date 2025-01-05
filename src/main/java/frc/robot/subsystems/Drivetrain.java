package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase
{
    private SparkMax m_leftMotor;
    private SparkMax m_rightMotor;
    private DifferentialDrive m_robotDrive;

    public Drivetrain()
    {
        m_leftMotor = new SparkMax(Constants.Drivetrain.leftmotor_port, MotorType.kBrushless);
        m_rightMotor = new SparkMax(Constants.Drivetrain.rightmotor_port, MotorType.kBrushless);
        m_rightMotor.setInverted(true); // TODO: Implement SparkBaseConfig
        m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
        SendableRegistry.addChild(m_robotDrive, m_leftMotor);
        SendableRegistry.addChild(m_robotDrive, m_rightMotor);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Left Motor", m_leftMotor.get());
        SmartDashboard.putNumber("Right Motor", m_rightMotor.get());
    }

    public void driveTank(double left, double right)
    {
        m_robotDrive.tankDrive(left, right); // when left is positive: counter clockwise; when right is positive: clockwise
    }
}

