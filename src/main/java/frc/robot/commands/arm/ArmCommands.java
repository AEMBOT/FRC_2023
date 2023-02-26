package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseS;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.VisionConstants.*;

public class ArmCommands {
    public static Command HighPiecePickUpCommand(DrivebaseS m_drivebase, ArmSubsystem m_arm, int numpadPosition){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
            m_drivebase.chasePoseC(() -> 
            DOUBLE_SUBSTATION
            .plus(numpadPosition == 10? DOUBLE_SUBSTATION_OFFSET_LEFT : DOUBLE_SUBSTATION_OFFSET_RIGHT).plus(ONE_METER_BACK.times(0.5))),
            new GoToPosition(m_arm, 0.5,0.5)
            ),
            new InstantCommand(m_arm::toggleClamp, m_arm)
            );
    }

    public static Command getPlaceGamePieceCommand(DrivebaseS m_drivebase, ArmSubsystem m_arm, TargetPosition position, int numpadPosition) {
        /*
        //for driving to grid
        Pose2d targetPosition = switch (position) {
            case LEFT_GRID -> GRID_LEFT;
            case COOP_GRID, NONE -> GRID_COOP;
            case RIGHT_GRID -> GRID_RIGHT;
            case DOUBLE_SUBSTATION -> DOUBLE_SUBSTATION;
        };*/
        //for only grid itself     
        Pose2d targetPosition = m_drivebase.getPose();

        targetPosition = switch (numpadPosition) {
            case 1, 4, 7 -> targetPosition.plus(CONE_OFFSET_LEFT);
            case 2, 5, 8 -> targetPosition;
            case 3, 6, 9 -> targetPosition.plus(CONE_OFFSET_RIGHT);
            case 10 -> targetPosition.plus(DOUBLE_SUBSTATION_OFFSET_LEFT);
            case 11 -> targetPosition.plus(DOUBLE_SUBSTATION_OFFSET_RIGHT);
            default -> m_drivebase.getPose().plus(ONE_METER_BACK.times(-0.5));
        };

        Pose2d finalTargetGrid = targetPosition;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        m_drivebase.chasePoseC(
                                () -> finalTargetGrid.plus(ONE_METER_BACK.times(0.5))),
                        m_arm.getGoToPositionCommand(
                                switch (numpadPosition) {
                                    case 1, 2, 3 -> extendToFloor;
                                    case 4, 5, 6 -> extendToMid;
                                    case 7, 8, 9 -> extendToHigh;
                                    case 10, 11 -> extendToSubstation;
                                    default -> minExtendHardStop;
                                },
                                switch (numpadPosition) {
                                    case 1, 2, 3 -> angleToFloor;
                                    case 4, 5, 6, 7, 8, 9 -> angleToDelivery;
                                    case 10, 11 -> angleToSubstation;
                                    default -> maxAngleHardStop;
                                }
                        )
                )
        );
    }
    /* 
    public static Command getPlacePieceAnyOrientationCommand(DrivebaseS m_drivebase, ArmSubsystem m_arm, TargetPosition targetPosition){

    }*/
   
}