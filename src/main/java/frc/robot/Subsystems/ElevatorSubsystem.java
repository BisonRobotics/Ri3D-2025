package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase
{
    private SparkMax m_leader;
    private SparkMax m_follower;
    private PIDController pidController;
    private ArmFeedforward feedforward;
    private boolean inTolerance = false;

    public ElevatorSubsystem()
    {
        m_leader = new SparkMax(Constants.ElevatorConstants.leaderPort, MotorType.kBrushless);
        m_follower = new SparkMax(Constants.ElevatorConstants.followerPort, MotorType.kBrushless);

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        followerConfig.follow(m_leader, true);
        m_follower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        pidController = new PIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD);
        pidController.setTolerance(Constants.ElevatorConstants.PID_TOLERANCE);
        feedforward = new ArmFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG, Constants.ElevatorConstants.kV);
    }

    public void moveElevator(double speed)
    {
        m_leader.set(speed);
    }

    public void setPostion(double positionMeters)
    {
        inTolerance = pidController.atSetpoint();

        pidController.setSetpoint(positionMeters);
        double pidOutput = pidController.calculate(motorPostionToMeters(m_leader.getEncoder().getPosition()),positionMeters);
        double feedforwardOutput = feedforward.calculate(
            motorPostionToMeters(m_leader.getEncoder().getPosition()),
            motorPostionToMeters(m_leader.getEncoder().getVelocity()));
        double speed = pidOutput + feedforwardOutput;

        //ensure @pram speed is within -1 to 1
        speed = ( speed > 1) ? 1 :speed;
        speed = ( speed < -1) ? -1 : speed;

        // set the motor speed
        m_leader.set(speed);
    }

    public double motorPostionToMeters(double motorPosition)
    {
        return motorPosition * Constants.ElevatorConstants.MOTOR_ENCODER_POSITION_COEFFICENT;
    }
    public double getElevatorPostion()
    {
        return motorPostionToMeters(m_leader.getEncoder().getPosition());
    }

    public void stopElevator()
    {
        m_leader.stopMotor();
    }

    public void zeroElevator()
    {
        m_leader.getEncoder().setPosition(0);
    }

    public boolean getInTolerance()
    {
        return inTolerance;
    }
}
