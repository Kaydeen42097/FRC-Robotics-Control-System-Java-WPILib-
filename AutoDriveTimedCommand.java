package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

/**
 * AutoDriveTimedCommand
 *
 * Drives the robot forward or backward for a set number of seconds,
 * using the NavX gyro to correct sideways drift and keep it straight.
 *
 * Since CIM motors have no encoders, we use time as a proxy for distance.
 * At AUTO_DRIVE_SPEED (0.45), the KitBot with 8.46:1 gearbox and 6" wheels
 * travels approximately 0.75 meters per second, so:
 *   1.0s ≈ 0.75m
 *   2.0s ≈ 1.5m
 *   3.0s ≈ 2.25m
 * These are estimates — tune by observing actual robot movement.
 */
public class AutoDriveTimedCommand extends Command {

    private final DriveSubsystem drive;
    private final double durationSeconds;
    private final double speed; // Positive = forward, negative = backward

    private final Timer timer = new Timer();

    /**
     * @param drive           The drive subsystem
     * @param durationSeconds How long to drive
     * @param speed           Speed to drive at (positive = forward, negative = backward)
     */
    public AutoDriveTimedCommand(DriveSubsystem drive, double durationSeconds, double speed) {
        this.drive = drive;
        this.durationSeconds = durationSeconds;
        this.speed = speed;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        // Lock the current heading so NavX can correct drift during the drive
        drive.lockHeading();
        String direction = speed >= 0 ? "FORWARD" : "REVERSE";
        SmartDashboard.putString("Auto Status", "DRIVING " + direction + " (" + durationSeconds + "s)");
    }

    @Override
public void execute() {
    drive.driveStraightCorrected(speed);
    SmartDashboard.putNumber("Drive Timer", timer.get());
}

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        SmartDashboard.putString("Auto Status", interrupted ? "DRIVE INTERRUPTED" : "DRIVE DONE");
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSeconds);
    }
}
