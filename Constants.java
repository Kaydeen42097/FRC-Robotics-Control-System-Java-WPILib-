package frc.robot;

/**
 * Constants.java
 *
 * VERSION 2 — Updated after first practice match:
 *   - Fixed shooting mode to RB (intake forward, throw REVERSE) 
 *   - Increased throw motor power
 *   - Added initial drive-back phase so camera can see AprilTag
 *   - Adjusted all timing to account for new sequence
 */
public final class Constants {

    // ---------------------------------------------------------------
    // CAN IDs
    // ---------------------------------------------------------------
    public static final class CANIds {
        public static final int INTAKE_MOTOR = 1;
        public static final int THROW_MOTOR  = 2;
        public static final int RIGHT_FRONT  = 3;
        public static final int RIGHT_BACK   = 4;
        public static final int LEFT_FRONT   = 5;
        public static final int LEFT_BACK    = 6;
    }

    // ---------------------------------------------------------------
    // DRIVE
    // ---------------------------------------------------------------
    public static final class Drive {
        public static final boolean LEFT_INVERTED  = true;
        public static final boolean RIGHT_INVERTED = false;

        public static final double TELEOP_TURN_SCALE = 0.5;

        public static final double AUTO_DRIVE_SPEED   = 0.45;
        public static final double AUTO_REVERSE_SPEED = 0.65;
        public static final double AUTO_TURN_SPEED    = 0.35;

        public static final double kP_HEADING = 0.025;
    }

    // ---------------------------------------------------------------
    // MECHANISM
    //
    // FIXED after practice match:
    //   RB mode = correct shooting mode
    //   intake FORWARD (+1.0), throw REVERSE (-1.0)
    //   Increased throw power to ensure balls reach the HUB
    // ---------------------------------------------------------------
    public static final class Mechanism {
        // RT — Ground to Hub (both forward) — used for COLLECTING only now
        public static final double INTAKE_GROUND_TO_HUB  =  1.0;
        public static final double THROW_GROUND_TO_HUB   =  1.0;

        // LT — Box to Shooting
        public static final double INTAKE_BOX_TO_SHOOT   = -0.7;
        public static final double THROW_BOX_TO_SHOOT    =  0.7;

        // RB — CORRECT AUTO SHOOTING MODE (confirmed at practice match)
        // Intake forward pulls ball in, throw motor REVERSES to launch into HUB
        // Increased throw power from -1.0 to full reverse for maximum distance
        public static final double INTAKE_SHOOT          =  0.8;
        public static final double THROW_SHOOT           = -1.0;

        // LB — Eject
        public static final double INTAKE_EJECT          = -0.7;
        public static final double THROW_EJECT           = -0.7;

        // Intake only for collecting balls off the floor
        public static final double INTAKE_COLLECT_ONLY   =  1.0;
    }

    // ---------------------------------------------------------------
    // AUTO TIMING — VERSION 2
    //
    // NEW sequence after practice match feedback:
    //
    //   Phase 1: Drive BACK 1.2m so camera can see HUB AprilTag (~1.8s)
    //   Phase 2: Align to HUB using AprilTag vision (~2.5s max)
    //   Phase 3: Shoot preloaded 8 balls (~4.0s)
    //   Phase 4: Settle pause (~0.3s)
    //   Phase 5: Drive forward into neutral zone to collect (~2.0s)
    //   Phase 6: Collect balls with intake (~3.0s)
    //   Phase 7: Drive back toward HUB (~2.0s)
    //   Phase 8: Align to HUB again (~2.5s max)
    //   Phase 9: Shoot collected balls (~3.0s)
    //   Total: ~21.1s max — safety cutoff at 19s stops anything running over
    //
    // ---------------------------------------------------------------
    public static final class AutoTiming {
        // Phase 1 — Drive back from starting line so camera can see tag
        // At 0.40 speed, ~1.2 meters takes approximately 1.8 seconds
        // If robot doesn't go back far enough, increase this value
        public static final double DRIVE_BACK_INITIAL_SECONDS = 3.5;

        // Phase 2 — Align to HUB before shooting preloads
        public static final double ALIGN_TIMEOUT_SECONDS      = 2.5;
        
        // Phase 3 — Shoot all 8 preloaded balls
        // Increased from 3.5s to 4.0s to make sure all balls clear
        public static final double SHOOT_PRELOAD_SECONDS      = 4.0;

        // Phase 4 — Brief pause after shooting
        public static final double SETTLE_SECONDS             = 0.3;

        // Phase 5 — Drive forward into neutral zone
        public static final double DRIVE_TO_NEUTRAL_SECONDS   = 2.0;

        // Phase 6 — Collect balls with intake running
        public static final double COLLECT_SECONDS            = 3.0;

        // Phase 7 — Drive back toward HUB after collecting
        public static final double DRIVE_BACK_SECONDS         = 2.0;

        // Phase 8 — Realign to HUB for second shot
        public static final double REALIGN_TIMEOUT_SECONDS    = 2.5;

        // Phase 9 — Shoot collected balls
        public static final double SHOOT_COLLECT_SECONDS      = 3.0;

        // Safety cutoff — stop everything with this many seconds left
        public static final double AUTO_SAFETY_CUTOFF_SECONDS = 1.0;
    }

    // ---------------------------------------------------------------
    // VISION
    // ---------------------------------------------------------------
    public static final class Vision {
        public static final String CAMERA_NAME = "DigBotCamera";

        public static final double CAMERA_HEIGHT_METERS = 0.45;
        public static final double CAMERA_PITCH_RADIANS = Math.toRadians(15.0);
        public static final double HUB_TAG_HEIGHT_METERS = 1.124;

        public static final double ALIGNMENT_THRESHOLD_DEGREES = 3.0;
        public static final double MIN_TURN_POWER = 0.12;
        public static final double MAX_TURN_POWER = 0.40;
        public static final double kP_VISION_TURN = 0.035;

        public static final int[] RED_HUB_TAG_IDS  = {2, 3, 4, 5, 8, 9, 10, 11};
        public static final int[] BLUE_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};

        public static final int[] RED_TRENCH_TAG_IDS  = {1, 6, 7, 12};
        public static final int[] BLUE_TRENCH_TAG_IDS = {17, 22, 23, 28};
    }
}
