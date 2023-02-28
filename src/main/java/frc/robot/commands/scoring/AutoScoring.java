package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.Limelight;

import java.util.function.Supplier;

public class AutoScoring extends CommandBase {
    private final DrivebaseS drivetrain;
    private final Limelight limelight;
    private final ArmSubsystem arm;
    private Command moveCommand;
    private Supplier<Pose2d> targetPosition;
    private final boolean atPosition;
    private boolean finished;
    private ArmCommand armCommand;
    private final TARGET_POSITION_X target_position_x;
    private final TARGET_POSITION_Y target_position_y;

    public enum TARGET_POSITION_X {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum TARGET_POSITION_Y {
        BOTTOM,
        CENTER,
        TOP
    }

    /**
     * Creates a new ExampleCommand.
     *
     * @param drivetrain The drivetrain
     * @param limelight  The limelight to read off of
     */
    public AutoScoring(DrivebaseS drivetrain, Limelight limelight, ArmSubsystem armSubsystem, TARGET_POSITION_X targetX, TARGET_POSITION_Y targetY) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        arm = armSubsystem;
        target_position_x = targetX;
        target_position_y = targetY;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        addRequirements(limelight);
        addRequirements(arm);
        atPosition = false;
        finished = false;
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        int mainTag = limelight.getMainTag();
        Pose2d tempTarget = new Pose2d();
        switch (mainTag) {
            case 0:
                // Use Odometry Pose Estimator
                break;
            case 1:
                tempTarget = new Pose2d(0, 0, new Rotation2d(0));// Hardcoded position for the position of each Apriltag in front of the grid
                break;
            default:
                //Again, Odometry
        }
        switch (target_position_x) {
            case LEFT:
                tempTarget.transformBy(new Transform2d(new Translation2d(), new Rotation2d()));
                break;
            case CENTER:
                tempTarget.transformBy(new Transform2d(new Translation2d(1, 0), new Rotation2d()));
                break;
            case RIGHT:
                tempTarget.transformBy(new Transform2d(new Translation2d(2, 0), new Rotation2d()));
                break;
        }
        final Pose2d finalTempTarget = tempTarget;
        targetPosition = (() -> finalTempTarget);
        moveCommand = drivetrain.chasePoseC(targetPosition);
        armCommand = new ArmCommand(arm, target_position_y);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        new Trigger((() -> !this.atPosition)).onTrue(moveCommand);
        new Trigger((() -> this.atPosition)).onTrue(armCommand);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        finished = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}