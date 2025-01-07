package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase
{
    private SparkMax m_leader;
    private SparkMax m_follower;
    private PIDController pidController;
    private ArmFeedforward feedforward;
    private DigitalInput m_limitSwitch;
    
    private boolean inTolerance = false;

    public double kP_tune = 0.0;
    public double PID_Tolerance_tune = 0.1;
    public double e_speed_limit = 0.1;

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

        //for tuning
        pidController = new PIDController(kP_tune, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD);
        pidController.setTolerance(PID_Tolerance_tune);
        SmartDashboard.putNumber("Elevator kP", kP_tune);
        SmartDashboard.putNumber("Elevator PID Tolerance", PID_Tolerance_tune);

        //put back in after tuning
        //pidController = new PIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD);
        //pidController.setTolerance(Constants.ElevatorConstants.PID_TOLERANCE);

        feedforward = new ArmFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG, Constants.ElevatorConstants.kV);

        m_limitSwitch = new DigitalInput(Constants.ElevatorConstants.limitSwitchPort);
        
        SmartDashboard.putNumber("Elevator Speed Limit (not for position control)", e_speed_limit);
        SmartDashboard.putNumber("Elevator pidOutput", 9999);
        SmartDashboard.putNumber("Elevator feedforward", 9999);
        SmartDashboard.putNumber("Elevator speed", 9999);
        SmartDashboard.putNumber("Elevator postion (from encoder)", m_leader.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator motorPositionToMeters", motorPostionToMeters(m_leader.getEncoder().getPosition()));
    }

    public void periodic() {
        kP_tune = SmartDashboard.getNumber("Elevator kP", kP_tune);
        pidController.setP(kP_tune);
        PID_Tolerance_tune = SmartDashboard.getNumber("Elevator PID Tolerance", PID_Tolerance_tune);
        pidController.setTolerance(PID_Tolerance_tune);
    }

    public void moveElevator(double speed)
    {
        // TODO: Implement height limit
        SmartDashboard.putNumber("Elevator Position (from encoder)", m_leader.getEncoder().getPosition());

        if (m_limitSwitch.get() && speed > 0)
        {
            speed = 0;
            zeroElevator();
        }
        
        m_leader.set(speed);
    }

    public double getPosition()
    {
        return m_leader.getEncoder().getPosition();
    }
    
    public RelativeEncoder getEncoder()
    {
        return m_leader.getEncoder();
    }
    
    public void setPostion(double positionMeters)
    {
        // TODO: Implement height limit
        inTolerance = pidController.atSetpoint();

        pidController.setSetpoint(positionMeters);
        double pidOutput = pidController.calculate(motorPostionToMeters(m_leader.getEncoder().getPosition()),positionMeters);
        double feedforwardOutput = feedforward.calculate(
            motorPostionToMeters(m_leader.getEncoder().getPosition()),
            motorPostionToMeters(m_leader.getEncoder().getVelocity()));
        double speed = pidOutput + feedforwardOutput;

        SmartDashboard.putNumber("Elevator pidOutput", pidOutput);
        SmartDashboard.putNumber("Elevator feedforward", feedforwardOutput);
        SmartDashboard.putNumber("Elevator speed", speed);
        SmartDashboard.putNumber("Elevator postion (from encoder)", m_leader.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator motorPositionToMeters", motorPostionToMeters(m_leader.getEncoder().getPosition()));

        //ensure @pram speed is within -1 to 1
        speed = ( speed > 1) ? 1 :speed;
        speed = ( speed < -1) ? -1 : speed;

        // TODO: Test if positive speed is up the elevator and adjust if statement
        if (m_limitSwitch.get() && speed > 0)
        {
            speed = 0;
            zeroElevator();
        }

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
