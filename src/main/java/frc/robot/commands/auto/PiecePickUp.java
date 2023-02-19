// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.AngleToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Limelight;

/**
 * An example command that uses an example subsystem.
 */
public class PiecePickUp extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ArmSubsystem m_elevator;
    private final double m_targetPos; //Get targetPosition from controller??
    private final Limelight m_Limelight;
    private AngleToPosition m_AngleToPositionFloor;

    //private final double m_angleMotorRotation;
    //private final double m_angle;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public PiecePickUp(ArmSubsystem subsystem, Limelight limelight,/*double angleMotorRotation, double angle, */ double targetPosition) {
        m_elevator = subsystem;
        m_targetPos = targetPosition;
        m_Limelight = limelight;

        //m_angleMotorRotation = angleMotorRotation;
        //m_angle = angle;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    PIDController pidController = new PIDController(0.2, 0.5, 0.5);

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
        m_AngleToPositionFloor = new AngleToPosition(m_elevator, ArmConstants.angleToFloor);
       
        //1280 x 960

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_elevator.stopAngle();
    }

    // Returns true when the command should end.


    @Override
    public boolean isFinished() {
        return Math.abs(m_elevator.getAnglePosition() - m_targetPos) < .05;
    }
}

