package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.VisionConstants.*;
import static java.lang.Math.abs;

public class ArmCommands {
    /* 
    public static Command HighPiecePickUpCommand(DrivebaseS m_drivebase, ArmSubsystem m_arm, int numpadPosition) {
        return new SequentialCommandGroup(
            
                new ParallelCommandGroup(
                        m_drivebase.chasePoseC(() ->
                                DOUBLE_SUBSTATION
                                        .plus(numpadPosition == 10 ? DOUBLE_SUBSTATION_OFFSET_LEFT : DOUBLE_SUBSTATION_OFFSET_RIGHT).plus(ONE_METER_BACK.times(0.5))),
                        new GoToPosition(m_arm, 0.5, 0.5)
                ),
                new InstantCommand(m_arm::toggleClamp, m_arm),
                m_arm.getGoToPositionCommand(minExtendHardStop, maxAngleHardStop)
        );
    }*/

    public static Command getHighPiecePickUpCommand(DrivebaseS m_drivebase, ArmSubsystem m_arm, IntakeSubsystem m_intake, TargetPosition targetPosition, int numpadPosition) {
        return new ParallelCommandGroup(
                new InstantCommand(m_intake::openClamp),
                getPlaceGamePieceCommand(m_drivebase, m_arm, targetPosition, numpadPosition),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> {
                            Transform2d error = m_drivebase.getPose().minus(m_drivebase.getTargetPose());
                            Transform2d tolerance = new Transform2d(
                                    new Translation2d(0.02, 0.02),
                                    Rotation2d.fromDegrees(0.5)
                            );
                            return abs(error.getRotation().getRadians()) < tolerance.getRotation().getRadians() &&
                                    abs(error.getX()) < tolerance.getX() &&
                                    abs(error.getY()) < tolerance.getY();
                        }),
                        m_intake.getIntakeAutoClampCommand(),
                        new WaitCommand(0.5),
                        m_arm.getGoToPositionCommand(minExtendHardStop, maxAngleHardStop)
                )
        );
    }

    public static Command getPlaceGamePieceCommand(DrivebaseS m_drivebase, ArmSubsystem m_arm, TargetPosition position, int numpadPosition) {
        Pose2d targetPosition = switch (position) {
            case LEFT_GRID -> GRID_LEFT;
            case COOP_GRID, NONE -> GRID_COOP;
            case RIGHT_GRID -> GRID_RIGHT;
            case DOUBLE_SUBSTATION -> DOUBLE_SUBSTATION;
        };
        targetPosition = switch (numpadPosition) {
            case 1, 4, 7 -> targetPosition.plus(CONE_OFFSET_LEFT);
            case 2, 5, 8 -> targetPosition;
            case 3, 6, 9 -> targetPosition.plus(CONE_OFFSET_RIGHT);
            case 10 -> targetPosition.plus(DOUBLE_SUBSTATION_OFFSET_LEFT);
            case 11 -> targetPosition.plus(DOUBLE_SUBSTATION_OFFSET_RIGHT);
            default -> m_drivebase.getPose().plus(ONE_METER_BACK.times(-0.5));
        };

        Pose2d finalTargetGrid = targetPosition;
        SmartDashboard.putNumber("targetPosX", finalTargetGrid.getX());
        SmartDashboard.putNumber("targetPosY", finalTargetGrid.getY());
        return new ParallelCommandGroup(
                new InstantCommand(() -> m_drivebase.setTargetPose(finalTargetGrid.plus(ONE_METER_BACK.times(
                        switch (position) {
                            case DOUBLE_SUBSTATION -> 0.27;
                            default -> 0.44;
                        }
                )))),
                m_drivebase.chasePoseC(
                        () -> finalTargetGrid.plus(ONE_METER_BACK.times(
                                switch (position) {
                                    case DOUBLE_SUBSTATION -> 0.27;
                                    default -> 0.44;
                                }
                        )),
                        switch (position) {
                            case DOUBLE_SUBSTATION -> 1.0;
                            default -> 1.5;
                        },
                        switch(position) {
                            case DOUBLE_SUBSTATION -> 0.3;
                            default -> 0.5;
                        }),
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
                            case 4, 5, 6 -> angleToMid;
                            case 7, 8, 9 -> angleToHigh;
                            case 10, 11 -> angleToSubstation;
                            default -> maxAngleHardStop;
                        }
                )
        );
    }

    public static Command getArmExtensionCommand(ArmSubsystem m_arm, int numpadPosition) {
        return m_arm.getGoToPositionCommand(
                switch (numpadPosition) {
                    case 1, 2, 3 -> extendToFloor;
                    case 4, 5, 6 -> extendToMid;
                    case 7, 8, 9 -> extendToHigh;
                    case 10, 11 -> extendToSubstation;
                    default -> minExtendHardStop;
                },
                switch (numpadPosition) {
                    case 1, 2, 3 -> angleToFloor;
                    case 4, 5, 6 -> angleToMid;
                    case 7, 8, 9 -> angleToHigh;
                    case 10, 11 -> angleToSubstation;
                    default -> maxAngleHardStop;
                }
        );
    }
    /* 
    public static Command getPlacePieceAnyOrientationCommand(DrivebaseS m_drivebase, ArmSubsystem m_arm, TargetPosition targetPosition){

    }*/

}