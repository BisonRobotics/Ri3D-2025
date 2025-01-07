package frc.robot.Subsystems;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase{
    private SparkMax topMotor; // Leader
    private SparkMax middleMotor; // Follower
    private SparkMax bottomMotor; // Follower

    private final double INTAKE_SPEED = Constants.ManipulatorConstants.INTAKE_SPEED;
    private final double HOLD_SPEED = Constants.ManipulatorConstants.HOLD_SPEED;
    private final double PLACE_SPEED = Constants.ManipulatorConstants.PLACE_SPEED;

    public ManipulatorSubsystem() {
        topMotor = new SparkMax(Constants.ManipulatorConstants.TOP_MANIPULATOR_MOTOR_ID, MotorType.kBrushless);
        middleMotor = new SparkMax(Constants.ManipulatorConstants.MIDDLE_MANIPULATOR_MOTOR_ID, MotorType.kBrushless);
        bottomMotor = new SparkMax(Constants.ManipulatorConstants.BOTTOM_MANIPULATOR_MOTOR_ID, MotorType.kBrushless);

        //config top motor

        SparkMaxConfig topMotorConfig = new SparkMaxConfig();
        topMotorConfig.inverted(false);
        topMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        // config middle motor

        SparkMaxConfig middleMotorConfig = new SparkMaxConfig();
        middleMotorConfig.inverted(false);
        middleMotorConfig.follow(topMotor.getDeviceId(), true);
        middleMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        // config bottom motor

        SparkMaxConfig bottomMotorConfig = new SparkMaxConfig();
        bottomMotorConfig.inverted(false);
        bottomMotorConfig.follow(topMotor.getDeviceId());
        bottomMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        topMotor.configure(topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        middleMotor.configure(middleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        bottomMotor.configure(bottomMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // intake ball, spit out pvc pipe
    public void intake() {
        topMotor.set(INTAKE_SPEED);
    }
    
    // hold gamepiece
    public void hold() {
        topMotor.set(HOLD_SPEED);
    }

    // intake pvc pipe, shoot out ball
    public void placeGamepiece() {
        topMotor.set(PLACE_SPEED);
    }

    // stop motor
    public void stop() {
        topMotor.stopMotor();
    }
}
