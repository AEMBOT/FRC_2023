package frc.robot.commands.auto;

import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.Limelight;

public class DynamicPathing {
    private final Limelight m_Limelight = new Limelight();
    private final DrivebaseS m_DrivebaseS = new DrivebaseS(m_Limelight);
/* 
    @Override
    public void execute(){
        m_DrivebaseS.generateTrajectoryToPose(null, null, null)
    }*/

    
}
