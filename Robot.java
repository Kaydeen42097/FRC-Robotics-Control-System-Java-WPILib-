package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.*;

/**
 * Robot.java — DigBot 2026 VERSION 2
 *
 * CHANGES after first practice match:
 *   1. Robot now drives BACK ~1.2m at start so camera can see HUB AprilTag
 *   2. Fixed shooting mode — now uses RB mode (intake forward, throw reverse)
 *      Previously was using RT mode which dumped balls on the ground
 *   3. Increased shoot duration to ensure all balls clear the mechanism
 *
 * =====================================================================
 * NEW AUTO SEQUENCE (20 seconds total):
 * =====================================================================
 *
 *  PHASE 1 — Drive BACK from starting line (~1.8s)
 *    Robot starts touching the ROBOT STARTING LINE facing the HUB.
 *    Drives backward ~1.2 meters so the camera can see the HUB AprilTag.
 *    The HUB tag was not visible from the starting line at practice.
 *
 *  PHASE 2 — Align to HUB using AprilTag vision (~0-2.5s)
 *    Now that we've backed up, camera can see the HUB tag.
 *    Robot rotates until perfectly aimed at the HUB.
 *
 *  PHASE 3 — Shoot preloaded 8 balls (~4.0s)
 *    Uses RB mode: intake forward, throw reverse.
 *    This is the confirmed working shooting mode from practice.
 *
 *  PHASE 4 — Settle pause (~0.3s)
 *    Brief stop so robot doesn't drive while last balls are clearing.
 *
 *  PHASE 5 — Drive forward into NEUTRAL ZONE (~2.0s)
 *    Robot drives forward past starting line into ball staging area.
 *
 *  PHASE 6 — Collect balls (~3.0s)
 *    Intake runs while driving slowly, scooping up floor balls.
 *
 *  PHASE 7 — Drive back toward HUB (~2.0s)
 *    Robot reverses back toward shooting position.
 *
 *  PHASE 8 — Realign to HUB (~0-2.5s)
 *    Vision alignment for second shot.
 *
 *  PHASE 9 — Shoot collected balls (~3.0s)
 *    Second shot using same RB mode.
 *
 * =====================================================================
 * TOTAL POTENTIAL AUTO SCORE: 8 (preload) + up to 10 (collected) = 18pts
 * =====================================================================
 */
public class Robot extends TimedRobot {

    // --- Subsystems ---
    private final DriveSubsystem   driveSubsystem   = new DriveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final VisionSubsystem  visionSubsystem  = new VisionSubsystem();

    // --- Teleop controller ---
    private final XboxController controller = new XboxController(0);

    // --- Auto ---
    private Command autoCommand;
    private final Timer autoTimer = new Timer();

    @Override
    public void robotInit() {
        SmartDashboard.putString("Robot Status", "INITIALIZING");
        SmartDashboard.putString("Auto Version", "V2 - Drive Back First");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putBoolean("NavX Ready",    !driveSubsystem.isNavXCalibrating());
        SmartDashboard.putBoolean("Camera Ready",  visionSubsystem.isCameraConnected());
    }

    // =====================================================================
    // AUTONOMOUS
    // =====================================================================

    @Override
    public void autonomousInit() {
        driveSubsystem.resetHeading();
        CommandScheduler.getInstance().cancelAll();
        autoTimer.reset();
        autoTimer.start();
        SmartDashboard.putString("Auto Status", "AUTO STARTED - V2");

        autoCommand = buildAutoSequence();
        autoCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
        if (autoTimer.get() >= (20.0 - Constants.AutoTiming.AUTO_SAFETY_CUTOFF_SECONDS)) {
            if (autoCommand != null && autoCommand.isScheduled()) {
                autoCommand.cancel();
                driveSubsystem.stop();
                shooterSubsystem.stop();
                SmartDashboard.putString("Auto Status", "AUTO COMPLETE - STOPPED");
            }
        }
    }

    /**
     * Builds the V2 auto sequence.
     *
     * KEY CHANGE: Phase 1 is now driving BACKWARD so the camera
     * can see the HUB AprilTag before we try to align and shoot.
     *
     * Time budget:
     *   Phase 1: 1.8s  (drive back)
     *   Phase 2: 2.5s  (align)
     *   Phase 3: 4.0s  (shoot preloads)
     *   Phase 4: 0.3s  (settle)
     *   Phase 5: 2.0s  (drive to neutral)
     *   Phase 6: 3.0s  (collect)
     *   Phase 7: 2.0s  (drive back)
     *   Phase 8: 2.5s  (realign)
     *   Phase 9: 3.0s  (shoot collected)
     *   Total:  ~21.1s max — safety cutoff handles any overrun
     */
    private Command buildAutoSequence() {
    return new SequentialCommandGroup(

        // Phase 1: Drive back
        new AutoDriveTimedCommand(driveSubsystem,
            Constants.AutoTiming.DRIVE_BACK_INITIAL_SECONDS,
            -Constants.Drive.AUTO_REVERSE_SPEED),

        // Phase 2: Align to HUB
        new AutoAlignToHubCommand(driveSubsystem, visionSubsystem),

        // Phase 3: Shoot
        new AutoShootCommand(shooterSubsystem,
            Constants.AutoTiming.SHOOT_PRELOAD_SECONDS)
    );
}

    // =====================================================================
    // TELEOP — fully preserved from your original code
    // =====================================================================

    @Override
    public void teleopInit() {
        if (autoCommand != null) {
            autoCommand.cancel();
        }
        driveSubsystem.stop();
        shooterSubsystem.stop();
        SmartDashboard.putString("Robot Status", "TELEOP");
    }

    @Override
    public void teleopPeriodic() {
        // --- DRIVE ---
        double forward = -controller.getLeftY();
        double turn    = -Constants.Drive.TELEOP_TURN_SCALE * controller.getRightX();
        driveSubsystem.arcadeDrive(forward, turn, controller.getRightBumper());

        // --- MECHANISM ---
        if (controller.getRightTriggerAxis() > 0.1) {
            // RT — Ground to Hub
            shooterSubsystem.shootGroundToHub();

        } else if (controller.getLeftTriggerAxis() > 0.1) {
            // LT — Box to Shooting
            shooterSubsystem.shootBoxToTarget();

        } else if (controller.getRightBumper()) {
            // RB — Confirmed working shoot mode
            shooterSubsystem.shootRB();


        } else if (controller.getLeftBumper()) {
            // LB — Eject
            shooterSubsystem.eject();

        } else {
            shooterSubsystem.stop();
        }
    }

    // =====================================================================
    // DISABLED
    // =====================================================================

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        driveSubsystem.stop();
        shooterSubsystem.stop();
        SmartDashboard.putString("Robot Status", "DISABLED");
    }
}
