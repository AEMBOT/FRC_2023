package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Interface for acquiring information from limelight
 */
public class Limelight extends SubsystemBase{
    /**
     * Standard Game Piece class, displays position in camera space
     * Cannot be edited once created
     */
    public static class GamePiece{
        public final boolean isCube;
        public final double xPos;
        public final double yPos;

        /**
         * Create a gamepiece
         * @param isCube A boolean, true for cubes, false for cones
         * @param xPos The x position in pixels
         * @param yPos The y position in pixels
         */
        private GamePiece(boolean isCube, double xPos, double yPos){
            this.isCube = isCube;
            this.xPos = xPos;
            this.yPos = yPos;
        }
    }
    public enum Pipeline{
        APRILTAG, // ID 0
        GAMEPIECE // ID 1
    }
    private NetworkTable limeLight;

    /**
     * Constructor
     */
    public Limelight(){
        limeLight = NetworkTableInstance.getDefault().getTable("limelight");
    }
    private Pose2d position; // Position of the bot in field space

    /**
     * Returns whether an apriltag is found
     * @return True if an apriltag is found, False if none found or wrong pipeline
     */
    public boolean visionTargetsFound(){
        if(getPipeline() != Pipeline.APRILTAG){ // return false if wrong pipeline
            return false;
        }
        return limeLight.getEntry("tv").getDouble(0) != 0; // Key tv is 1 for a vision target found and 0 for no target
    }

    /**
     * Returns the positions of any game pieces in the camera field-of-view
     * Returns in camera pixels, returns an empty set if not in the correct pipeline
     * @return A list of GamePiece objects, each representing a unique game piece. Returns an empty list if using the wrong pipeline
     */
    private GamePiece[] getGamePiecePositions(){
        if(getPipeline() != Pipeline.GAMEPIECE){
            return new GamePiece[]{}; // return empty if not detecting game pieces
        }
        double[] rawPositions = limeLight.getEntry("llpython").getDoubleArray(new double[]{}); // raw limelight data
        GamePiece[] positions = new GamePiece[rawPositions.length/3]; // Create array
        for(int i=0; i<rawPositions.length/3; i++){
            positions[i] = new GamePiece(rawPositions[3 * i] == 1, rawPositions[3 * i + 1], rawPositions[3 * i + 2]); // Every 3 values defines a gamepiece, as follows: [(1 if cube, 0 if cone), (x position), (y position)]
        }
        return positions;
    }

    /**
     * Returns the position of the bot in field space
     * @return A Pose2d representing the position of the bot in field space
     */
    public Pose2d getPosition(){
        return position;
    }

    /**
     * A function to be called periodically, updates position from limelight
     */
    private void updatePosition(){
        if(visionTargetsFound()){
            double[] rawPosition = limeLight.getEntry("botpose").getDoubleArray(new double[]{0}); // Get position (botpose returns a double array, [xpos, ypos, zpos, xrot, yrot, zrot]
            position = new Pose2d(new Translation2d(rawPosition[0], rawPosition[1]), new Rotation2d(rawPosition[5])); // Convert to Pose2d for use elsewhere
        }else{
            // Do Odometry Stuff
        }
    }

    /**
     * Switches to a specified vision pipeline (Apriltag or Game Piece detection)
     * @param pipeline The pipeline to switch to
     */
    public void setPipeline(Pipeline pipeline){
        int pipelineID;
        switch(pipeline){ // Get numerical id from the specified pipeline
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
     * @return A Pipeline enum representing the active pipeline
     */
    public Pipeline getPipeline(){
        int pipelineID = (int) Math.floor(limeLight.getEntry("getPipe").getDouble(0)); // Get pipeline ID
        Pipeline pipeline;
        switch(pipelineID){ // Convert it into a Pipeline object for easy use
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
    public void periodic(){
        updatePosition(); // Update position
    }
}
