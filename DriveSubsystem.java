package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * DriveSubsystem
 *
 * Controls the 4 CIM drive motors (am-0255, brushed) through SparkMax controllers.
 * Uses the NavX2 MXP gyro for heading-corrected straight driving in auto.
 *
 * CIMs have NO built-in encoders, so all auto movement is TIME-based.
 * The NavX gives us heading so we can drive straight even without encoders.
 *
 * NAVX2 INSTALLATION (if not done yet):
 *   1. Power off robot completely
 *   2. Find the large 50-pin MXP port on the side of the roboRIO (labeled "MXP")
 *   3. Press NavX2 board straight down onto it — only fits one way
 *   4. Power back on — done. No wiring needed.
 */
public class DriveSubsystem extends SubsystemBase {

    // --- Hardware ---
    private final SparkMax leftFront  = new SparkMax(Constants.CANIds.LEFT_FRONT,  MotorType.kBrushed);
    private final SparkMax leftBack   = new SparkMax(Constants.CANIds.LEFT_BACK,   MotorType.kBrushed);
    private final SparkMax rightFront = new SparkMax(Constants.CANIds.RIGHT_FRONT, MotorType.kBrushed);
    private final SparkMax rightBack  = new SparkMax(Constants.CANIds.RIGHT_BACK,  MotorType.kBrushed);

    private final MotorControllerGroup leftGroup  = new MotorControllerGroup(leftFront, leftBack);
    private final MotorControllerGroup rightGroup = new MotorControllerGroup(rightFront, rightBack);
    private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

    // NavX2 plugged into MXP port on roboRIO
    private final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // Heading locked at the start of a straight-drive command
    private double lockedHeadingDegrees = 0.0;

    // Timer for the shooting rocking - Team 79
    private double rockTimer = 0;
    private double rockAmplitude = 0.7;

    public DriveSubsystem() {
        // Configure all 4 SparkMax controllers for brushed CIM motors
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kBrake); // Brake mode: robot stops instantly when power cut

        leftFront.configure(driveConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftBack.configure(driveConfig,   ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFront.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightBack.configure(driveConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Left side motors face the opposite direction physically — invert them
        leftGroup.setInverted(Constants.Drive.LEFT_INVERTED);
        rightGroup.setInverted(Constants.Drive.RIGHT_INVERTED);
    }

    @Override
    public void periodic() {
        // Push telemetry to SmartDashboard so you can monitor heading live
        SmartDashboard.putNumber("Heading (deg)", getHeadingDegrees());
        SmartDashboard.putBoolean("NavX Connected", navX.isConnected());
        SmartDashboard.putBoolean("NavX Calibrating", navX.isCalibrating());
    }

    // ---------------------------------------------------------------
    // DRIVING — used by commands
    // ---------------------------------------------------------------

    /**
     * Arcade drive — used in teleop.
     * forward: positive = forward, negative = backward
     * turn: positive = right, negative = left
     */
    public void arcadeDrive(double forward, double turn, boolean shooting) {
        if(shooting){
            drive.arcadeDrive(Math.cos(rockTimer) * -rockAmplitude, turn);
            rockTimer += 0.3;
        }
        else{
            if(rockTimer != 0) rockTimer = 0;
            drive.arcadeDrive(forward, turn);
        }
    }

    /**
     * Drive straight using NavX heading correction.
     * Call lockHeading() once before starting a straight drive,
     * then call this every loop iteration.
     *
     * @param speed Positive = forward, negative = backward
     */
    public void driveStraightCorrected(double speed) {
        double headingError  = getHeadingDegrees() - lockedHeadingDegrees;
        double correction    = Constants.Drive.kP_HEADING * headingError;
        // Flip correction sign based on direction of travel
        double turnOutput    = (speed >= 0) ? -correction : correction;
        drive.arcadeDrive(speed, turnOutput);
    }

    /**
     * Turn in place at a set speed.
     * Positive = clockwise (right), negative = counterclockwise (left)
     */
    public void turnInPlace(double turnSpeed) {
        drive.arcadeDrive(0, turnSpeed);
    }

    /** Stop all drive motors immediately */
    public void stop() {
        drive.arcadeDrive(0, 0);
    }

    // ---------------------------------------------------------------
    // HEADING (NavX)
    // ---------------------------------------------------------------

    /**
     * Returns current heading in degrees.
     * 0 = robot facing direction it faced when resetHeading() was last called.
     * Positive = turned clockwise (right), negative = counterclockwise (left).
     */
    public double getHeadingDegrees() {
        // NavX is clockwise-positive; negate to match WPILib convention
        return -navX.getAngle();
    }

    /** Snapshot the current heading — call this before a straight drive */
    public void lockHeading() {
        lockedHeadingDegrees = getHeadingDegrees();
    }

    /** Reset NavX heading to zero — called at the start of auto */
    public void resetHeading() {
        navX.reset();
        lockedHeadingDegrees = 0.0;
    }

    public boolean isNavXConnected() {
        return navX.isConnected();
    }

    public boolean isNavXCalibrating() {
        return navX.isCalibrating();
    }
}
