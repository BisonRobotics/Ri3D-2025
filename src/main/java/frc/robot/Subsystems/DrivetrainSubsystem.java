package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase
{
    private SparkMax m_leftMotorLeader;
    private SparkMax m_rightMotorLeader;
    private SparkMax m_leftMotorFollower;
    private SparkMax m_rightMotorFollower;
    private DifferentialDrive m_robotDrive;

    //Outreach drive speed limit
    //IMPORTANT
    //if the speed limit is removed, put the /2 back for the x-axis in line 27 of RobotContainer
    public double driveSpeedLimit = 0.35; //0.5 means 50% speed

    public DrivetrainSubsystem()
    {
        m_leftMotorLeader = new SparkMax(Constants.DrivetrainConstants.leftmotor_port, MotorType.kBrushless);
        m_rightMotorLeader = new SparkMax(Constants.DrivetrainConstants.rightmotor_port, MotorType.kBrushless);
        m_rightMotorFollower = new SparkMax(Constants.DrivetrainConstants.rightmotor_follower_port, MotorType.kBrushless);
        m_leftMotorFollower = new SparkMax(Constants.DrivetrainConstants.leftmotor_follower_port, MotorType.kBrushless);
        
        SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
        rightLeaderConfig.inverted(true);
        m_rightMotorLeader.configure(rightLeaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
        rightFollowerConfig.follow(m_rightMotorLeader);
        m_rightMotorFollower.configure(rightFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
        leftFollowerConfig.follow(m_leftMotorLeader);
        m_leftMotorFollower.configure(leftFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_robotDrive = new DifferentialDrive(m_leftMotorLeader::set, m_rightMotorLeader::set);
        SendableRegistry.addChild(m_robotDrive, m_leftMotorLeader);
        SendableRegistry.addChild(m_robotDrive, m_rightMotorLeader);
    }

    @Override
    public void periodic()
    {
        // SmartDashboard.putNumber("Left Motor", m_leftMotorLeader.get());
        // SmartDashboard.putNumber("Right Motor", m_rightMotorLeader.get());
    }

    public void driveArcade(double left, double right)
    {
        m_robotDrive.arcadeDrive(-left*driveSpeedLimit, -right*driveSpeedLimit);
    }
}

