package frc.robot.commands.docking;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseS;
import io.github.oblarg.oblog.Loggable;

public class DockingForceBalance extends CommandBase implements Loggable {

    double pitchUpper = -65;
    double pitchLower = -75;

    private final DrivebaseS m_drivebase;

    public DockingForceBalance(DrivebaseS drive) {
        m_drivebase = drive;
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
        brakeRobot();
    }

    //assuumes intial if(navx.getPitch() > pitchUpper || navx.getPitch() < pitchLower when called
    @Override
    public void execute() {
        //m_drivebase.drive(new ChassisSpeeds(-omegaToDriveSpeed(navx.getRawGyroY()), 0,0));
        //m_drivebase.drive(new ChassisSpeeds(omegaToDriveSpeed(m_drivebase.getRoll(), navx.getRawGyroY()), 0, 0));
        //turn robot orientation x forward or backward (no yaw value)
        /* 
        if  (Math.abs(navx.getYaw()) > 10){
            m_drivebase.drive(new ChassisSpeeds(0,0,0.3 * Math.signum(navx.getYaw())));
        }*/
        //double appliedSpeed  = 0.05* (m_drivebase.getRoll()) - 0.005 * navx.getRawGyroY() * Math.signum(m_drivebase.getRoll());
        //double appliedSpeed  = 0.05* (m_drivebase.getRoll()) - 0.005 * navx.getRawGyroY() * -Math.signum(m_drivebase.getRoll());
        //double appliedSpeed = 0.027 * (m_drivebase.getRoll());
        //+ 11 * Math.signum(m_drivebase.getRoll())
        //+ 0.01 * m_drivebase.getRawGyroY() * Math.signum(m_drivebase.getRoll()
        double appliedSpeed = -0.027 * (m_drivebase.getRoll() + 0.02 * m_drivebase.getRawGyroY() * Math.signum(m_drivebase.getRoll()));
        if (Math.abs(appliedSpeed) > 0.35){
            appliedSpeed = 0.35 * Math.signum(appliedSpeed);   
        }
        m_drivebase.drive(new ChassisSpeeds(appliedSpeed,0,0));
    }

    @Override
    public void end(boolean _interrupted) {
    }
}