package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * VisionSubsystem
 *
 * Reads AprilTag data from PhotonVision running on the Raspberry Pi 5
 * with the Logitech C270 webcam.
 * APRILTAG LAYOUT FOR REBUILT 2026:
 * =====================================================================
 *   HUB tags (44.25in = 1.124m off floor):
 *     Red  HUB: IDs  2, 3, 4, 5, 8, 9, 10, 11
 *     Blue HUB: IDs 18,19,20,21,24,25,26,27
 *   TOWER WALL tags (21.75in off floor):
 *     Red:  15, 16   |  Blue: 31, 32
 *   OUTPOST tags (21.75in off floor):
 *     Red:  13, 14   |  Blue: 29, 30
 *   TRENCH tags (35in off floor):
 *     Red:   1, 6, 7, 12   |  Blue: 17, 22, 23, 28
 */
public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;

    // Determined from DriverStation at auto start — do not hardcode
    private boolean isRedAlliance = true;

    public VisionSubsystem() {
        camera = new PhotonCamera(Constants.Vision.CAMERA_NAME);
    }

    @Override
    public void periodic() {
        // Fetch latest camera result every robot loop (~20ms)
        latestResult = camera.getLatestResult();

        // Auto-detect alliance from DriverStation each loop
        // This handles both red and blue without any manual configuration
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            isRedAlliance = (alliance.get() == Alliance.Red);
        }

        // Push vision diagnostics to SmartDashboard
        SmartDashboard.putBoolean("Camera Connected",    camera.isConnected());
        SmartDashboard.putBoolean("Has Target",          hasTarget());
        SmartDashboard.putBoolean("Has Hub Target",      getBestHubTarget() != null);
        SmartDashboard.putNumber("Hub Yaw (deg)",        getHubTagYawDegrees());
        SmartDashboard.putBoolean("Aligned to Hub",      isAlignedToHub());
        SmartDashboard.putString("Alliance",             isRedAlliance ? "RED" : "BLUE");
        SmartDashboard.putNumber("Best Tag ID",          getBestTargetId());
    }

    // ---------------------------------------------------------------
    // ALLIANCE
    // ---------------------------------------------------------------

    public boolean isRedAlliance() {
        return isRedAlliance;
    }

    /** Returns the AprilTag IDs belonging to OUR alliance's HUB */
    public int[] getOurHubTagIds() {
        return isRedAlliance
            ? Constants.Vision.RED_HUB_TAG_IDS
            : Constants.Vision.BLUE_HUB_TAG_IDS;
    }

    // ---------------------------------------------------------------
    // TARGET DETECTION
    // ---------------------------------------------------------------

    /** True if the camera can see ANY AprilTag right now */
    public boolean hasTarget() {
        return latestResult != null && latestResult.hasTargets();
    }

    /** Returns the highest-confidence visible target, or null if none */
    public PhotonTrackedTarget getBestTarget() {
        if (!hasTarget()) return null;
        return latestResult.getBestTarget();
    }

    /**
     * Searches all visible targets for one belonging to OUR alliance's HUB.
     * Returns the first match found, or null if no hub tag is visible.
     * This prevents accidentally aligning to the opponent's HUB.
     */
    public PhotonTrackedTarget getBestHubTarget() {
        if (!hasTarget()) return null;
        int[] ourIds = getOurHubTagIds();
        for (PhotonTrackedTarget target : latestResult.getTargets()) {
            for (int id : ourIds) {
                if (target.getFiducialId() == id) {
                    return target;
                }
            }
        }
        return null;
    }

    // ---------------------------------------------------------------
    // MEASUREMENTS FROM HUB TAG
    // ---------------------------------------------------------------

    /**
     * Horizontal angle to our HUB tag in degrees.
     * Negative = tag is to the LEFT  → robot should turn LEFT  (negative turn)
     * Positive = tag is to the RIGHT → robot should turn RIGHT (positive turn)
     * Returns 0.0 if no hub tag is visible.
     */
    public double getHubTagYawDegrees() {
        PhotonTrackedTarget target = getBestHubTarget();
        if (target == null) return 0.0;
        return target.getYaw();
    }

    /**
     * Estimated distance to our HUB tag in meters.
     * Uses camera height + pitch + tag height for trigonometric calculation.
     * Returns -1.0 if no hub tag is visible.
     */
    public double getHubTagDistanceMeters() {
        PhotonTrackedTarget target = getBestHubTarget();
        if (target == null) return -1.0;
        return PhotonUtils.calculateDistanceToTargetMeters(
            Constants.Vision.CAMERA_HEIGHT_METERS,
            Constants.Vision.HUB_TAG_HEIGHT_METERS,
            Constants.Vision.CAMERA_PITCH_RADIANS,
            Math.toRadians(target.getPitch())
        );
    }

    // ---------------------------------------------------------------
    // ALIGNMENT CHECKS
    // ---------------------------------------------------------------

    /**
     * True if the robot is pointed at the HUB within the alignment threshold.
     * Used by AutoAlignToHubCommand to know when to stop turning.
     */
    public boolean isAlignedToHub() {
        PhotonTrackedTarget target = getBestHubTarget();
        if (target == null) return false;
        return Math.abs(target.getYaw()) < Constants.Vision.ALIGNMENT_THRESHOLD_DEGREES;
    }

    /** The fiducial ID of the best visible tag, or -1 if none */
    public int getBestTargetId() {
        PhotonTrackedTarget t = getBestTarget();
        return (t == null) ? -1 : t.getFiducialId();
    }

    public boolean isCameraConnected() {
        return camera.isConnected();
    }
}
