package frc.robot.Subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase{
    private SparkMax wristMotor;
    private SparkBaseConfig wristMotorConfig;
    private PIDController wristPidController;
    private ArmFeedforward wristFeedForward;
    private DigitalInput m_limitSwitch;
    
    private boolean inTolerance = false;
    public double w_kP_tune = 0.0;
    public double w_PID_Tolerance_tune= 0.1;

    public WristSubsystem() 
    {
        wristMotor = new SparkMax(Constants.WristConstants.WRIST_MOTOR_ID, MotorType.kBrushless);

        wristMotorConfig.inverted(false);
        wristMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        //for tuning
        wristPidController = new PIDController(w_kP_tune, Constants.WristConstants.WRIST_kI, Constants.WristConstants.WRIST_kD);
        wristPidController.setTolerance(w_PID_Tolerance_tune);
        SmartDashboard.putNumber("Wrist kP", w_kP_tune);
        SmartDashboard.putNumber("Wrist PID Tolerance", w_PID_Tolerance_tune);

        //put back in after tuning
        // wristPidController = new PIDController(Constants.WristConstants.WRIST_kP, Constants.WristConstants.WRIST_kI, Constants.WristConstants.WRIST_kD);
        // wristPidController.setTolerance(Constants.WristConstants.WRIST_PID_TOLERANCE);

        wristFeedForward = new ArmFeedforward(Constants.WristConstants.WRIST_kS, Constants.WristConstants.WRIST_kG, Constants.WristConstants.WRIST_kV);

        m_limitSwitch = new DigitalInput(Constants.WristConstants.limitSwitchPort);
        SmartDashboard.putNumber("Wrist feedforward", 9999);
        SmartDashboard.putNumber("Wrist speed", 9999);
        SmartDashboard.putNumber("Wrist pidOutput", 9999);
    }

    public void periodic() {
        w_kP_tune = SmartDashboard.getNumber("Wrist kP", w_kP_tune);
        wristPidController.setP(w_kP_tune);
        w_PID_Tolerance_tune = SmartDashboard.getNumber("Wrist PID Tolerance", w_PID_Tolerance_tune);
        wristPidController.setTolerance(w_PID_Tolerance_tune);
    }

    // zero the wrist encoder
    public void zeroWrist()
    {
        wristMotor.getEncoder().setPosition(0);
    }

    // hold the wrist at a current pose using pid and feed forward. i really hope i dont need to use this
    public void setPosition(double positionRadians)
    {

        inTolerance = wristPidController.atSetpoint();

        wristPidController.setSetpoint(positionRadians);

        double wristRadians = (wristMotor.getEncoder().getPosition() * 2 * Math.PI) / 75; // convert to radians then compensate for 75:1 gear ratio
        double wristVelocityRadSec = (wristMotor.getEncoder().getVelocity() * 2 * Math.PI) / 75;

        // calculate the pid using feed forward and set the motor to maintain position
        double pidOutput = wristPidController.calculate(wristRadians, (wristPidController.getSetpoint() * 2 * Math.PI));
        double feedForward = wristFeedForward.calculate(wristRadians, wristVelocityRadSec);

        double speed = pidOutput + feedForward;
        SmartDashboard.putNumber("Wrist pidOutput", pidOutput);
        SmartDashboard.putNumber("Wrist feedforward", feedForward);
        SmartDashboard.putNumber("Wrist speed", speed);

        // ensure @param speed is within -1 to 1
        speed = (speed > 1) ? 1 : speed;
        speed = (speed < -1) ? -1 : speed;

        // TODO: Test if positive speed is up the elevator and adjust if statement
        if (m_limitSwitch.get() && speed > 0 &&
            //verify it isnt trying to go out of its limits
            wristMotor.getEncoder().getPosition() + speed < Constants.WristConstants.WRIST_LIMIT_TOP &&
            wristMotor.getEncoder().getPosition() + speed > Constants.WristConstants.WRIST_LIMIT_BOTTOM)
        {
            speed = 0;
        }

        // set the motor speed
        wristMotor.set(speed);
    }

    public double getPosition()
    {
        return wristMotor.getEncoder().getPosition();
    }

    public boolean getInTolerance()
    {
        return inTolerance;
    }

    public void stopWrist()
    {
        wristMotor.stopMotor();
    }

    public void setWristSpeed(double speed)
    {
        // ensure @param speed is within -1 to 1
        speed = (speed > 1) ? 1 : speed;
        speed = (speed < -1) ? -1 : speed;

        wristMotor.set(speed);
    }
}
