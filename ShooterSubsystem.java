package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * ShooterSubsystem — VERSION 2
 *
 * FIXED after practice match:
 *   - shootAuto() now uses RB mode (confirmed working at practice)
 *   - intake FORWARD, throw REVERSE = balls launch into HUB
 *   - Previously auto was using RT mode which dumped balls on the ground
 */
public class ShooterSubsystem extends SubsystemBase {

    // NOTE: Change kBrushed to kBrushless below if these are NEO motors
    private final SparkMax intakeMotor = new SparkMax(Constants.CANIds.INTAKE_MOTOR, MotorType.kBrushed);
    private final SparkMax throwMotor  = new SparkMax(Constants.CANIds.THROW_MOTOR,  MotorType.kBrushed);

    public ShooterSubsystem() {
        SparkMaxConfig mechanismConfig = new SparkMaxConfig();
        mechanismConfig.idleMode(IdleMode.kCoast);

        intakeMotor.configure(mechanismConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        throwMotor.configure(mechanismConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Motor Output", intakeMotor.get());
        SmartDashboard.putNumber("Throw Motor Output",  throwMotor.get());
    }

    // ---------------------------------------------------------------
    // TELEOP MODES — exact same as your controller mapping
    // ---------------------------------------------------------------

    /** RT — Ground to Hub: both motors forward */
    public void shootGroundToHub() {
        intakeMotor.set(Constants.Mechanism.INTAKE_GROUND_TO_HUB);
        throwMotor.set(Constants.Mechanism.THROW_GROUND_TO_HUB);
    }

    /** LT — Box to Shooting: intake reverse, throw forward */
    public void shootBoxToTarget() {
        intakeMotor.set(Constants.Mechanism.INTAKE_BOX_TO_SHOOT);
        throwMotor.set(Constants.Mechanism.THROW_BOX_TO_SHOOT);
    }

    /**
     * RB — CONFIRMED CORRECT SHOOTING MODE from practice match.
     * Intake FORWARD (+1.0), throw REVERSE (-1.0)
     * This is the mode that actually launches balls into the HUB.
     * Used for ALL auto shooting.
     */
    public void shootRB() {
        intakeMotor.set(Constants.Mechanism.INTAKE_SHOOT);
        throwMotor.set(Constants.Mechanism.THROW_SHOOT);
    }

    /** LB — Eject: both motors reverse */
    public void eject() {
        intakeMotor.set(Constants.Mechanism.INTAKE_EJECT);
        throwMotor.set(Constants.Mechanism.THROW_EJECT);
    }

    /** Auto collect: intake only, throw motor off */
    public void runIntakeOnly() {
        intakeMotor.set(Constants.Mechanism.INTAKE_COLLECT_ONLY);
        throwMotor.set(0.0);
    }

    /** Stop all mechanism motors */
    public void stop() {
        intakeMotor.set(0.0);
        throwMotor.set(0.0);
    }
}
