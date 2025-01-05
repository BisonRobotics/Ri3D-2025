package frc.robot.Subsystems;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase{
    private SparkMax topMotor; // Leader
    private SparkMax middleMotor; // Follower
    private SparkMax bottomMotor; // Follower

    private SparkBaseConfig topMotorConfig;
    private SparkBaseConfig middleMotorConfig;
    private SparkBaseConfig bottomMotorConfig;

    private final double INTAKE_SPEED = Constants.ManipulatorConstants.INTAKE_SPEED;
    private final double HOLD_SPEED = Constants.ManipulatorConstants.HOLD_SPEED;
    private final double PLACE_SPEED = Constants.ManipulatorConstants.PLACE_SPEED;

    public ManipulatorSubsystem() {
        topMotor = new SparkMax(Constants.ManipulatorConstants.TOP_MANIPULATOR_MOTOR_ID, MotorType.kBrushless);
        middleMotor = new SparkMax(Constants.ManipulatorConstants.MIDDLE_MANIPULATOR_MOTOR_ID, MotorType.kBrushless);
        bottomMotor = new SparkMax(Constants.ManipulatorConstants.BOTTOM_MANIPULATOR_MOTOR_ID, MotorType.kBrushless);

        //config top motor
        topMotorConfig.inverted(false);
        topMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        // config middle motor
        middleMotorConfig.inverted(false);
        middleMotorConfig.follow(topMotor.getDeviceId());
        middleMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        // config bottom motor
        bottomMotorConfig.inverted(false);
        bottomMotorConfig.follow(topMotor.getDeviceId());
        bottomMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        topMotor.configure(topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        middleMotor.configure(middleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        bottomMotor.configure(bottomMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // intake gamepiece
    public void intake() {
        topMotor.set(INTAKE_SPEED);
    }

    // apply a tiny bit of negative power to hold the gamepiece in place. 
    // ** Might not be needed. **
    public void hold() {
        topMotor.set(HOLD_SPEED);
    }

    // spit out gamepiece
    public void drop() {
        topMotor.set(-INTAKE_SPEED);
    }

    // place coral on pipe or spit out ball
    public void placeGamepiece() {
        topMotor.set(PLACE_SPEED);
    }

    // stop motor
    public void stop() {
        topMotor.stopMotor();
    }
}
