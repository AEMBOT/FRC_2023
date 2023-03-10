package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.AutoConstants.ALLIANCE;
import static frc.robot.Constants.VisionConstants.FIELD_LENGTH;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH;

/**
 * Interface for acquiring information from a singular Limelight device
 */
public class Limelight extends SubsystemBase {
    /**
     * Standard Game Piece class, displays position in camera space
     * Cannot be edited once created
     */
    public static class GamePiece {
        public final boolean isCube;
        public final double xPos;
        public final double yPos;

        /**
         * Create a gamepiece
         *
         * @param isCube A boolean, true for cubes, false for cones
         * @param xPos   The x position in pixels
         * @param yPos   The y position in pixels
         */
        private GamePiece(boolean isCube, double xPos, double yPos) {
            this.isCube = isCube;
            this.xPos = xPos + 300 - 1280 / 2;
            this.yPos = -yPos + 960 / 2;
        }

        /**
         * Converts the pixel coordinates to degrees
         *
         * @return A double list [x angle, y angle]
         */
        public double[] toDegrees() {
            return new double[]{
                    xPos * 59.6 / 320,
                    yPos * 59.6 / 320
            };
        }

        /**
         * Gets the location of the game piece relative to the robot
         *
         * @return Translation2d representing position in robot space
         */
        public Translation2d relativeLocation() {
            double distance = Math.tan(yPos) * Constants.LIMELIGHT_HEIGHT;
            double xPosition = distance * Math.cos(xPos);
            double yPosition = distance * Math.sin(xPos);
            return new Translation2d(xPosition, yPosition);
        }

        /**
         * Gets position of the game piece in world space
         *
         * @param position - Position of the bot in world space
         * @return Translation2d representing position of game piece in world space
         */
        public Translation2d absoluteLocation(Pose2d position) {
            Translation2d relativeWorldSpace = relativeLocation().rotateBy(position.getRotation());
            return position.getTranslation().plus(relativeWorldSpace);
        }
    }

    private double lastUpdate;
    private boolean accessedBefore;

    public enum Pipeline {
        APRILTAG, // ID 0
        GAMEPIECE // ID 1
    }

    private final NetworkTable limeLight;
    private Pose2d tagRelativePosition;

    /**
     * Determines whether the position data has been accessed before
     *
     * @return boolean
     */
    public boolean getDataAccessedBefore() {
        return accessedBefore;
    }

    /**
     * Returns the time of last update for the Apriltags
     *
     * @return Time of last AprilTag position update
     */
    public double getLastTimestamp() {
        return lastUpdate;
    }

    /**
     * Basic Constructor with default NetworkTable ID
     */
    public Limelight() {
        limeLight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Constructor using custom NetworkTable ID, for use with multiple Limelights.
     *
     * @param networkTableID String representing NetworkTable ID for the limelight (Default "limelight")
     */
    public Limelight(String networkTableID) {
        limeLight = NetworkTableInstance.getDefault().getTable(networkTableID);

    }

    private Pose3d position = new Pose3d(); // Position of the bot in field space

    /**
     * Returns whether an apriltag is found
     *
     * @return True if an apriltag is found, False if none found or wrong pipeline
     */
    public boolean visionTargetsFound() {
        if (getPipeline() != Pipeline.APRILTAG) { // return false if wrong pipeline
            return false;
        }
        return limeLight.getEntry("tv").getDouble(0) > 0.5; // Key tv is 1 for a vision target found and 0 for no target
    }
    /**
     * Returns the positions of any game pieces in the camera field-of-view
     * Returns in camera pixels, returns an empty set if not in the correct pipeline
     *
     * @return A list of GamePiece objects, each representing a unique game piece. Returns an empty list if using the wrong pipeline
     */
    public GamePiece[] getGamePiecePositions() {
        if (getPipeline() != Pipeline.GAMEPIECE) {
            return new GamePiece[]{}; // return empty if not detecting game pieces
        }
        double[] rawPositions = limeLight.getEntry("llpython").getDoubleArray(new double[]{}); // raw limelight data
        GamePiece[] positions = new GamePiece[rawPositions.length / 3]; // Create array
        for (int i = 0; i < rawPositions.length / 3; i++) {
            positions[i] = new GamePiece(rawPositions[3 * i] == 1, rawPositions[3 * i + 1], rawPositions[3 * i + 2]); // Every 3 values defines a gamepiece, as follows: [(1 if cube, 0 if cone), (x position), (y position)]
        }
        return positions;
    }

    /**
     * Returns the position of the bot in field space
     *
     * @return A Pose2d representing the position of the bot in field space
     */
    public Pose3d getPosition() {
        accessedBefore = true;
        return position;
    }

    /**
     * Returns the position of the camera relative to the AprilTag
     *
     * @return A Pose2d representing the position of the camera in tag space
     */
    public Pose2d getRelativePose() {
        return tagRelativePosition;
    }

    public int getMainTag() {
        if (!visionTargetsFound()) {
            return 0;
        }
        int tag = (int) limeLight.getEntry("tid").getInteger(0);
        return tag;
    }

    /**
     * A function to be called periodically, updates position from limelight
     */
    private void updatePosition() {
        if (visionTargetsFound()) {
            String ntKey;
            if (ALLIANCE == DriverStation.Alliance.Red) {
                ntKey = "botpose_wpired";
            } else {
                ntKey = "botpose_wpiblue";
            }

            double[] rawPosition = limeLight.getEntry(ntKey).getDoubleArray(new double[]{0, 0, 0, 0, 0, 0}); // Get position (botpose returns a double array, [xpos, ypos, zpos, xrot, yrot, zrot]
            NetworkTableInstance.getDefault().getTable("LimelightTesting").getEntry("rawPose").setValue(rawPosition);
            if (rawPosition.length < 6) {
                double[] newRawPosition = new double[]{0, 0, 0, 0, 0, 0};
                System.arraycopy(rawPosition, 0, newRawPosition, 0, rawPosition.length);
                rawPosition = newRawPosition;
            }
            for (int i = 3; i < rawPosition.length; i++) {
                rawPosition[i] = Units.degreesToRadians(rawPosition[i]);
            }
            position = new Pose3d(new Translation3d(rawPosition[0], rawPosition[1], rawPosition[2]), new Rotation3d(rawPosition[3], rawPosition[4], rawPosition[5])); // Convert to Pose2d for use elsewhere
            rawPosition = limeLight.getEntry("campose").getDoubleArray(new double[]{0, 0, 0, 0, 0, 0}); // Get position (botpose returns a double array, [xpos, ypos, zpos, xrot, yrot, zrot]
            NetworkTableInstance.getDefault().getTable("LimelightTesting").getEntry("rawCamTran").setValue(rawPosition);
            if (rawPosition.length < 6) {
                double[] newRawPosition = new double[]{0, 0, 0, 0, 0, 0};
                System.arraycopy(rawPosition, 0, newRawPosition, 0, rawPosition.length);
                rawPosition = newRawPosition;
            }
            rawPosition[0] = rawPosition[0] + (FIELD_LENGTH / 2);
            rawPosition[1] = rawPosition[1] + (FIELD_WIDTH / 2);
            for (int i = 3; i < rawPosition.length; i++) {
                rawPosition[i] = Units.degreesToRadians(rawPosition[i]);
            }

            tagRelativePosition = new Pose2d(new Translation2d(rawPosition[2], rawPosition[0]), new Rotation2d(rawPosition[5])); // Convert to Pose2d for use elsewhere
            if (lastUpdate != (limeLight.getEntry(ntKey).getLastChange() / 1000.0) - limeLight.getEntry("tl").getDouble(0) - limeLight.getEntry("cl").getDouble(0)) {
                lastUpdate = (limeLight.getEntry(ntKey).getLastChange() / 1000.0) - limeLight.getEntry("tl").getDouble(0) - limeLight.getEntry("cl").getDouble(0);
                accessedBefore = false;
            }

        } else {
            // Do Odometry Stuff*
        }
    }

    /**
     * Switches to a specified vision pipeline (Apriltag or Game Piece detection)
     *
     * @param pipeline The pipeline to switch to
     */
    public void setPipeline(Pipeline pipeline) {
        int pipelineID;
        switch (pipeline) { // Get numerical id from the specified pipeline
            case APRILTAG:
                pipelineID = 0;
                break;
            case GAMEPIECE:
                pipelineID = 1;
                break;
            default:
                pipelineID = -1;
        }
        limeLight.getEntry("pipeline").setNumber(pipelineID);
    }

    /**
     * Returns the pipeline the limelight is currently using
     *
     * @return A Pipeline enum representing the active pipeline
     */
    public Pipeline getPipeline() {
        int pipelineID = (int) Math.floor(limeLight.getEntry("getPipe").getDouble(0)); // Get pipeline ID
        Pipeline pipeline;
        switch (pipelineID) { // Convert it into a Pipeline object for easy use
            case 0:
                pipeline = Pipeline.APRILTAG;
                break;
            case 1:
                pipeline = Pipeline.GAMEPIECE;
                break;
            default:
                pipeline = Pipeline.APRILTAG;
        }
        return pipeline;
    }

    @Override
    /**
     * Function to be run periodically
     */
    public void periodic() {
        updatePosition(); // Update position
    }

    public void test() {
    }
}