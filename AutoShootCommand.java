package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * AutoShootCommand — VERSION 2
 *
 * FIXED after practice match:
 *   Now uses shootRB() instead of shootGroundToHub()
 *   RB mode = intake FORWARD, throw REVERSE = confirmed working at practice
 */
public class AutoShootCommand extends Command {

    private final ShooterSubsystem shooter;
    private final double durationSeconds;
    private final Timer timer = new Timer();

    public AutoShootCommand(ShooterSubsystem shooter, double durationSeconds) {
        this.shooter = shooter;
        this.durationSeconds = durationSeconds;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        // FIXED: Use RB mode (intake forward, throw reverse)
        // This is the mode confirmed to launch balls into the HUB
        shooter.shootRB();
        SmartDashboard.putString("Auto Status", "SHOOTING (" + durationSeconds + "s)");
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Shoot Timer", timer.get());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        SmartDashboard.putString("Auto Status", interrupted ? "SHOOT INTERRUPTED" : "SHOOT DONE");
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSeconds);
    }
}
