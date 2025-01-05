package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase
{
    private SparkMax m_leader;
    private SparkMax m_follower;

    public ElevatorSubsystem()
    {
        m_leader = new SparkMax(Constants.ElevatorConstants.leaderPort, MotorType.kBrushless);
        m_follower = new SparkMax(Constants.ElevatorConstants.followerPort, MotorType.kBrushless);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(m_leader, true);
        m_follower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    @Override
    public void periodic()
    {
        
    }

    public void moveElevator(double speed)
    {
        m_leader.set(speed);
    }
}
