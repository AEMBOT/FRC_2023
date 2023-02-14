package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple subsystem for getting information from multiple limelights
 */
public class VisionSubsystem extends SubsystemBase {
    public final Limelight[] limelights;

    /**
     * Constructor
     *
     * @param limelights list of limelights to get data from
     */
    public VisionSubsystem(Limelight[] limelights) {
        this.limelights = limelights;
    }

    /**
     * Averages the position given by each Limelight
     *
     * @return Pose2d representing robot position
     */
    public Pose3d getPose() {
        double avgX = 0;
        double avgY = 0;
        double avgZ = 0;
        double avgRotX = 0;
        double avgRotY = 0;
        double avgRotZ = 0;
        double num = 0;
        for (Limelight limelight : limelights) {
            if (limelight.visionTargetsFound()) {
                Pose3d pose = limelight.getPosition();
                avgX += pose.getX() / limelights.length;
                avgY += pose.getY() / limelights.length;
                avgZ += pose.getZ() / limelights.length;
                avgRotX += pose.getRotation().getX() / limelights.length;
                avgRotY += pose.getRotation().getY() / limelights.length;
                avgRotZ += pose.getRotation().getZ() / limelights.length;
                num++;
            }
        }
        avgX /= num;
        avgY /= num;
        avgZ /= num;
        avgRotX /= num;
        avgRotY /= num;
        avgRotZ /= num;
        return new Pose3d(new Translation3d(avgX, avgY, avgZ), new Rotation3d(avgRotX, avgRotY, avgRotZ));
    }
}
