// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.GoToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.Limelight;
import static frc.robot.Constants.VisionConstants.APRILTAG_LAYOUT;

/**
 * An example command that uses an example subsystem.
 */
public class HighPiecePickUpCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ArmSubsystem m_ArmSubsystem;
    private final DrivebaseS m_drivebase;
    private final Limelight m_limelight;
    private GoToPosition m_AngleToPositionFloor;
    private GoToPosition m_GoToPositionHigh; 
    private final double m_targetAngle = 1;
    private final double m_targetExtend =1;

    //command for drive to substation and pick up
    //control speed, angle + elevation


    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public HighPiecePickUpCommand(ArmSubsystem subsystem, DrivebaseS drivebase, Limelight limelight) {
        m_ArmSubsystem = subsystem;
        m_drivebase = drivebase;
        m_limelight = limelight;
        m_GoToPositionHigh =  new GoToPosition(m_ArmSubsystem,1,1);
        //m_angleMotorRotation = angleMotorRotation;
        //m_angle = angle;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    //future: add option for left or right in station
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    public double correctPoseY(){
        //returns the value to correct the robot to be inputted into translation 2d for drivebase chaseposeC
        //can run this at same time as x translation
        //TODO: change apirl tag layout to get main tag
        double currentY = APRILTAG_LAYOUT.getTagPose(4).get().getY();
        if (APRILTAG_LAYOUT.getTagPose(4).get().getY() >= 2){
            //return necessary y translation 
            return currentY - 2;
        }
        else{
            return 0;
        }
    }

    public double driveToDoubleSubstation(){
        //returns x value for translation 2d to get 
        //TODO: change apirl tag layout to get main tag
        double currentX = APRILTAG_LAYOUT.getTagPose(4).get().getX();
        if (APRILTAG_LAYOUT.getTagPose(4).get().getX() >= 5){
            return currentX - 5;
        }
        return 0;
    }

    public boolean autoCloseClamp(){
        //closes clamp when close to double substation
        //only call when the drivebase is moving
        //TODO: close while move for fast stuff
        double currentX = APRILTAG_LAYOUT.getTagPose(4).get().getX();
        if (currentX <= 5){
            return true;
        }
        return false;
    }
/* 
    autoSelector.addOption("apriltag",
                drivebaseS.chasePoseC(
                        () -> APRILTAG_LAYOUT.getTagPose(3).get().toPose2d().plus(new Transform2d(new Translation2d(2.77, 2.5), new Rotation2d(Math.PI)))
                )
        );*/

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_limelight.getMainTag() == 4 || m_limelight.getMainTag() == 5){
            //drive to double substation
            //max vel 1
            //max accel 0.3
            m_drivebase.chasePoseC(() -> 
            APRILTAG_LAYOUT.getTagPose(4).get().toPose2d()
            .plus(new Transform2d(new Translation2d(driveToDoubleSubstation(), correctPoseY()), null)));
            //move arm
            m_ArmSubsystem.setArmPosition(m_targetExtend, m_targetAngle);
            if (autoCloseClamp()){
                m_ArmSubsystem.toggleClamp();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.stopAngle();
    }

    // Returns true when the command should end.
}

