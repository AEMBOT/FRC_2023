//// Copyright (c) FIRST and other WPILib contributors.
//// Open Source Software; you can modify and/or share it under the terms of
//// the WPILib BSD license file in the root directory of this project.
//
package frc.robot.commands.scoring;

import java.lang.annotation.Target;
import java.sql.Driver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.GoToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ArmSubsystem.TargetGrid;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.arm.GoToPosition;
import frc.robot.Constants.ArmConstants;

///**
// * An example command that uses an example subsystem.
// */
public class DriverAssist extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ArmSubsystem m_arm;
    private final DrivebaseS m_drivebase;
    private final Limelight m_limelight;
    private final GoToPosition m_GoToPositionPickUp;
    private final GoToPosition m_GoToPositionMid;
    private final GoToPosition m_GoToPositionHigh;

    public enum TargetHeight {
        BOTTOM,
        CENTER,
        TOP
    }

    public enum TargetPosition{
        LEFT,
        CENTER,
        RIGHT
    }

    private TargetGrid targetGrid;
    private TargetHeight targetHeight;
    private TargetPosition targetPosition;

    /**
//     * Creates a new ExampleCommand.
//     *
//     * @param subsystem The subsystem used by this command.
//     */
    //public DriverAssist(Limelight limelight, DrivebaseS drivebase, ArmSubsystem subsystem, TargetHeight TargetHeight, TargetPosition TargetPosition) {
    public DriverAssist(Limelight limelight, DrivebaseS drivebase, ArmSubsystem subsystem, TargetGrid targetgrid) {
        m_arm = subsystem;
        m_limelight = limelight;
        m_drivebase = drivebase;
        targetGrid = targetgrid;
        //targetHeight = TargetHeight;
        //targetPosition = TargetPosition;
        m_GoToPositionPickUp = new GoToPosition(m_arm, 0, ArmConstants.angleToSubstation);
        m_GoToPositionMid = new GoToPosition(m_arm, ArmConstants.extendToMid, ArmConstants.angleToDelivery);
        m_GoToPositionHigh = new GoToPosition(m_arm, ArmConstants.extendToHigh, ArmConstants.angleToDelivery);
       // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    public Pose2d goToStation(TargetGrid targetGrid){
        
       if (targetGrid == TargetGrid.COOP){
            return VisionConstants.GRID_COOP;
       }
       if (targetGrid == TargetGrid.INNER){
            return VisionConstants.GRID_INNER;
       }
       if (targetGrid == TargetGrid.OUTER){
            return VisionConstants.GRID_OUTER;
       }
       //if (targetGrid == TargetGrid.NONE){
        else{
            if (m_limelight.getMainTag() == 2 || m_limelight.getMainTag() ==7){
                return VisionConstants.GRID_COOP;
            }
            if (m_limelight.getMainTag() == 6 || m_limelight.getMainTag() == 3){
                return VisionConstants.GRID_OUTER;
            }
            if (m_limelight.getMainTag() == 8 || m_limelight.getMainTag() == 1){
                return VisionConstants.GRID_INNER;
            }
            else{
                return VisionConstants.GRID_COOP;
            } 
        }
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        /* 
        switch (targetPosition){
            //left is positive
            case LEFT:
                m_drivebase.chasePoseC(() -> VisionConstants.GRID_COOP.plus(VisionConstants.CONE_OFFSET_LEFT).plus(VisionConstants.ONE_METER_BACK.times(0.5)));
            case CENTER:
                m_drivebase.chasePoseC(() -> VisionConstants.GRID_COOP.plus(VisionConstants.ONE_METER_BACK.times(0.5)));
            case RIGHT:
                m_drivebase.chasePoseC(() -> VisionConstants.GRID_COOP.plus(VisionConstants.CONE_OFFSET_RIGHT.plus(VisionConstants.ONE_METER_BACK.times(0.5))));
        }
        switch (targetHeight){
            case TOP:
            case CENTER:
            case BOTTOM:
        }*/
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        goToStation(targetGrid);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
