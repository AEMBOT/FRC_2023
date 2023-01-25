package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple subsystem for getting information from multiple limelights
 */
public class VisionSubsystem extends SubsystemBase{
    public final Limelight[] limelights;

    /**
     * Constructor
     * @param limelights list of limelights to get data from
     */
    public VisionSubsystem(Limelight[] limelights){
        this.limelights = limelights;
    }

    /**
     * Averages the position given by each Limelight
     * @return Pose2d representing robot position
     */
    public Pose2d getPose(){
        double avgX = 0;
        double avgY = 0;
        double avgRot = 0;
        for(Limelight limelight: limelights){
            Pose2d pose = limelight.getPosition();
            avgX += pose.getX() / limelights.length;
            avgY += pose.getY() / limelights.length;
            avgRot += pose.getRotation().getRadians() / limelights.length;
        }
        return new Pose2d(new Translation2d(avgX, avgY), new Rotation2d(avgRot));
    }
}
