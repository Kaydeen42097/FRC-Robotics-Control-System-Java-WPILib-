package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants;

/**
 * AutoAlignToHubCommand
 *
 * Rotates the robot in place until the alliance HUB's AprilTag is centered
 * in the camera frame (within ALIGNMENT_THRESHOLD_DEGREES).
 *
 * Uses a proportional controller (P-loop):
 *   turnSpeed = kP_VISION_TURN * yawError
 *
 * The robot only turns — it does not drive forward or backward here.
 * Run AutoDriveTimedCommand BEFORE this to get close to the HUB,
 * then run this to fine-tune the aim.
 *
 * TUNING:
 *   - If the robot oscillates (turns past, turns back repeatedly): REDUCE kP_VISION_TURN
 *   - If the robot is very slow to align: INCREASE kP_VISION_TURN
 *   - If the robot never quite finishes: INCREASE ALIGNMENT_THRESHOLD_DEGREES
 *
 * FALLBACK: If no hub tag is visible, the robot slowly rotates to search.
 * After ALIGN_TIMEOUT_SECONDS, the command ends regardless so auto continues.
 */
public class AutoAlignToHubCommand extends Command {

    private final DriveSubsystem  drive;
    private final VisionSubsystem vision;
    private final Timer timer = new Timer();

    // Direction to scan if no tag visible (positive = clockwise / right)
    // If your robot consistently can't find the tag, flip this to -0.15
    private static final double SCAN_SPEED = 0.15;

    public AutoAlignToHubCommand(DriveSubsystem drive, VisionSubsystem vision) {
        this.drive  = drive;
        this.vision = vision;
        addRequirements(drive);
        // Note: Vision is read-only — we don't "require" it so it keeps updating
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        SmartDashboard.putString("Auto Status", "ALIGNING TO HUB");
    }

    @Override
    public void execute() {
        if (!vision.hasTarget() || vision.getBestHubTarget() == null) {
            // Can't see a hub tag — rotate slowly to search for one
            drive.turnInPlace(SCAN_SPEED);
            SmartDashboard.putString("Align State", "SCANNING");
            return;
        }

        double yawError   = vision.getHubTagYawDegrees();
        double turnSpeed  = Constants.Vision.kP_VISION_TURN * yawError;

        // Apply minimum power floor to overcome static friction on carpet
        // Without this the robot stalls just short of aligned
        if (Math.abs(yawError) > Constants.Vision.ALIGNMENT_THRESHOLD_DEGREES) {
            if (Math.abs(turnSpeed) < Constants.Vision.MIN_TURN_POWER) {
                turnSpeed = Math.copySign(Constants.Vision.MIN_TURN_POWER, turnSpeed);
            }
        }

        // Cap maximum turn speed — spinning too fast overshoots the target
        turnSpeed = Math.max(-Constants.Vision.MAX_TURN_POWER,
                    Math.min( Constants.Vision.MAX_TURN_POWER, turnSpeed));

        drive.turnInPlace(turnSpeed);

        SmartDashboard.putString("Align State", "TRACKING");
        SmartDashboard.putNumber("Align Yaw Error", yawError);
        SmartDashboard.putNumber("Align Turn Speed", turnSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        if (interrupted || timer.hasElapsed(Constants.AutoTiming.ALIGN_TIMEOUT_SECONDS)) {
            SmartDashboard.putString("Auto Status", "ALIGN TIMED OUT — shooting anyway");
        } else {
            SmartDashboard.putString("Auto Status", "ALIGNED TO HUB");
        }
    }

    @Override
    public boolean isFinished() {
        // Done when perfectly aimed OR we've used up our alignment time budget
        return vision.isAlignedToHub()
            || timer.hasElapsed(Constants.AutoTiming.ALIGN_TIMEOUT_SECONDS);
    }
}
