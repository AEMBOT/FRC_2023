package frc.robot.commands.docking;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseS;
import io.github.oblarg.oblog.Loggable;

public class AutoPathDocking extends CommandBase{
    private final DrivebaseS m_drivebase;
    private double appliedSpeed;
    private boolean isforward;
    private double maxSpeed  = 0.25;

    public AutoPathDocking(DrivebaseS drive, Boolean isForward) {
        m_drivebase = drive;
        isforward = isForward;
        addRequirements(m_drivebase);
    }

    public void brakeRobot() {
        m_drivebase.drive(0, 0, 0, false);
    }

    public static double tilt(double roll, double pitch) {
        double cy = (Math.cos(Units.degreesToRadians(pitch)));
        double sx = (Math.sin(Units.degreesToRadians(roll)));
        double sy = (Math.sin(Units.degreesToRadians(pitch)));

        double a = Units.radiansToDegrees(Math.asin(Math.sqrt(Math.pow(sy, 2) + Math.pow(sx, 2) * Math.pow(cy, 2))));
        return a;
    }

    //public double distanceTrakcer(double )

    @Override
    public void initialize() {
        // Make sure motors are running in brake mode to avoid overshooting
        appliedSpeed = 0;
        brakeRobot();
    }

    @Override
    public void execute() {
        //+ 11 * Math.signum(m_drivebase.getRoll())
        if (isforward){
            appliedSpeed = -0.05 * (m_drivebase.getRoll())
            + 0.04 * m_drivebase.getRawGyroY();
            maxSpeed = 0.30;
        }  
        else{
            appliedSpeed  = -0.02* (m_drivebase.getRoll() )
            + 0.03 * m_drivebase.getRawGyroY();
            maxSpeed = 0.25;
        }
        if (Math.abs(appliedSpeed) > maxSpeed){;
            appliedSpeed = maxSpeed * Math.signum(appliedSpeed);
        }
        if (m_drivebase.getRoll() < 6 && m_drivebase.getRoll() > -6){
            appliedSpeed = 0;
        }
        if (tilt(m_drivebase.getRoll(), m_drivebase.getPitch()) < 5){
            appliedSpeed =0;
        }
        m_drivebase.drive(new ChassisSpeeds(appliedSpeed,0,0));
    }
    @Override
    public void end(boolean _interrupted) {

    }
}