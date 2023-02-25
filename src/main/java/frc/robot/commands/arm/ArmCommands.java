package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseS;
import static frc.robot.Constants.VisionConstants.*;

public class ArmCommands {
    public static Command HighPiecePickUpCommand(DrivebaseS m_drivebase, ArmSubsystem m_arm, boolean left){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
            m_drivebase.chasePoseC(() -> 
            DOUBLE_SUBSTATION
            .plus(left? DOUBLE_SUBSTATION_OFFSET_LEFT : DOUBLE_SUBSTATION_OFFSET_RIGHT).plus(ONE_METER_BACK.times(0.5))),
            new GoToPosition(m_arm, 0.5,0.5)
            ),
            new InstantCommand(m_arm::toggleClamp, m_arm)
            );
    }
}