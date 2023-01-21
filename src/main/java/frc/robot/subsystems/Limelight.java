package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase{
    public static class GamePiece{
        public final boolean isCube;
        public final double xPos;
        public final double yPos;
        private GamePiece(boolean isCube, double xPos, double yPos){
            this.isCube = isCube;
            this.xPos = xPos;
            this.yPos = yPos;
        }
    }
    public enum Pipeline{
        APRILTAG,
        GAMEPIECE
    }
    private NetworkTable limeLight;
    public Limelight(){
        limeLight = NetworkTableInstance.getDefault().getTable("limelight");
    }
    private Pose2d position;
    public boolean visionTargetsFound(){
        if(getPipeline() != Pipeline.APRILTAG){
            return false
        }
        return limeLight.getEntry("tv").getDouble(0) != 0; // Key tv is 1 for a vision target found and 0 for no target
    }
    private GamePiece[] getGamePiecePositions(){
        if(getPipeline() != Pipeline.GAMEPIECE){
            return new GamePiece[]{};
        }
        double[] rawPositions = limeLight.getEntry("llpython").getDoubleArray(new double[]{});
        GamePiece[] positions = new GamePiece[rawPositions.length/3];
        for(int i=0; i<rawPositions.length/3; i++){
            positions[i] = new GamePiece(rawPositions[3 * i] == 1, rawPositions[3 * i + 1], rawPositions[3 * i + 2]);
        }
        return positions;
    }
    public Pose2d getPosition(){
        return position;
    }
    private void updatePosition(){
        if(visionTargetsFound()){
            double[] rawPosition = limeLight.getEntry("botpose").getDoubleArray(new double[]{0});
            position = new Pose2d(new Translation2d(rawPosition[0], rawPosition[1]), new Rotation2d(rawPosition[5]));
        }else{
            // Do Odometry Stuff
        }
    }
    public void setPipeline(Pipeline pipeline){
        int pipelineID;
        switch(pipeline){
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
    public Pipeline getPipeline(){
        int pipelineID = (int) Math.floor(limeLight.getEntry("getPipe").getDouble(0));
        Pipeline pipeline;
        switch(pipelineID){
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
    public void periodic(){
        updatePosition();
    }
}
