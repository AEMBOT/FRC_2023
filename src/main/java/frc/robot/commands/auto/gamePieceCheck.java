// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.GoToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Pipeline;

/**
 * An example command that uses an example subsystem.
 */
public class gamePieceCheck extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Limelight m_Limelight;
    //private final double m_angleMotorRotation;
    //private final double m_angle;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public gamePieceCheck(Limelight limelight) {
        m_Limelight = limelight;

        //m_angleMotorRotation = angleMotorRotation;
        //m_angle = angle;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // for now assuming that half and half for left or right placement of game piece
    //algorithm for later for relative to distance, which side it is on
    //alternative: press a certain button to say it is at a spot 

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //public static class GamePiece 
        //public final boolean isCube;
        //public final double xPos;
        //public final double yPos;
        m_Limelight.setPipeline(Pipeline.GAMEPIECE);
        SmartDashboard.putNumber("gamepieceauto1", m_Limelight.getGamePiecePositions()[0].xPos);
        SmartDashboard.putNumber("gamepieceauto2", m_Limelight.getGamePiecePositions()[0].yPos);
        SmartDashboard.putBoolean("gamepieceauto3", m_Limelight.getGamePiecePositions()[0].isCube);
        //m_elevator.extendClamp;
        //1280 x 960

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
}

