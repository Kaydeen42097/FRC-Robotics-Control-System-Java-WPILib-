package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

/**
 * AutoCollectCommand
 *
 * Drives slowly forward into the NEUTRAL ZONE while running the intake motor,
 * scooping up FUEL (foam balls) staged there at the start of the match.
 *
 * Per the game manual:
 *   - ~360 FUEL are staged in the NEUTRAL ZONE per match
 *   - FUEL are spread across the neutral zone from just past the ROBOT STARTING LINE
 *     to approximately the CENTER LINE (the neutral zone is 283in = 7.19m deep)
 *   - The robot starts at the ROBOT STARTING LINE (edge of the ALLIANCE ZONE)
 *   - Driving 1.5-2m forward puts the robot into the dense ball cluster
 *
 * The robot can hold up to 10 balls — driving for COLLECT_SECONDS at a slow
 * speed while running the intake should collect a good load.
 *
 * NOTE: Drive and shooter are BOTH required so they move together as one action.
 */
public class AutoCollectCommand extends Command {

    private final DriveSubsystem  drive;
    private final ShooterSubsystem shooter;
    private final Timer timer = new Timer();

    // Drive slowly during collection so balls don't bounce away
    private static final double COLLECT_DRIVE_SPEED = 0.30;

    public AutoCollectCommand(DriveSubsystem drive, ShooterSubsystem shooter) {
        this.drive   = drive;
        this.shooter = shooter;
        addRequirements(drive, shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        drive.lockHeading();          // Keep driving straight into the ball cluster
        shooter.runIntakeOnly();      // Intake on, throw motor off
        SmartDashboard.putString("Auto Status", "COLLECTING BALLS");
    }

    @Override
    public void execute() {
        // Drive straight (NavX corrected) while intake runs
        drive.driveStraightCorrected(COLLECT_DRIVE_SPEED);
        SmartDashboard.putNumber("Collect Timer", timer.get());
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        shooter.stop();
        SmartDashboard.putString("Auto Status", interrupted ? "COLLECT INTERRUPTED" : "COLLECT DONE");
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(Constants.AutoTiming.COLLECT_SECONDS);
    }
}
