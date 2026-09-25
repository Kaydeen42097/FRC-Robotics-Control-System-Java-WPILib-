package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Main.java
 *
 * Standard FRC entry point — do not modify this file.
 * All robot logic lives in Robot.java.
 */
public final class Main {
    private Main() {}

    public static void main(String[] args) {
        RobotBase.startRobot(Robot::new);
    }
}
