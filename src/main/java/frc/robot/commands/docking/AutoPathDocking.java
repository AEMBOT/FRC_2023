package frc.robot.commands.docking;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivebaseS;
import io.github.oblarg.oblog.Loggable;

public class AutoPathDocking extends CommandBase implements Loggable {

    private final LinearFilter rollData = LinearFilter.movingAverage(10);
    private final LinearFilter vel = LinearFilter.movingAverage(10);
    private final AHRS navx = new AHRS(Port.kMXP);

    private final DrivebaseS m_drivebase;

    private double appliedSpeed;

    public AutoPathDocking(DrivebaseS drive) {
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
        appliedSpeed = 0;
        brakeRobot();
    }

    @Override
    public void execute() {
       // double appliedSpeed  = 0.05* (navx.getRoll() + 12)  + 0.0025 * navx.getRawGyroY();
        //double appliedSpeed  = 0.01* (navx.getRoll() + 12 * Math.signum(navx.getRoll())) + 0.0025 * navx.getRawGyroY() * Math.signum(navx.getRoll());
        //double appliedSpeed  =  0.0025 * navx.getRawGyroY() * Math.signum(navx.getRoll());
        //double appliedSpeed  = 0.01* (navx.getRoll() + 10 * Math.signum(navx.getRoll())) + 0.01 * navx.getRawGyroY() * Math.signum(navx.getRoll());
        //double appliedSpeed  = -0.03* (navx.getRoll() + 11 * Math.signum(navx.getRoll())) + 0.0025 * navx.getRawGyroY() * Math.signum(navx.getRoll());
        appliedSpeed  = -0.03* (navx.getRoll() + 11 * Math.signum(navx.getRoll()));
        if (Math.abs(appliedSpeed) > 0.25){
            appliedSpeed = 0.25 * Math.signum(appliedSpeed);   
        }
        if (navx.getRoll() < 11 && navx.getRoll() > -11){
            appliedSpeed = 0;
        }
        if (tilt(navx.getRoll(), navx.getPitch()) < 5){
            appliedSpeed =0;
        }
        m_drivebase.drive(new ChassisSpeeds(appliedSpeed,0,0));
    }
    @Override
    public boolean isFinished() {
        //double rollAvg = rollData.calculate(navx.getRoll());
        //double velAvg = vel.calculate(navx.getRawGyroY());
        return (m_drivebase.getFieldRelativeLinearSpeedsMPS().getX() > -0.3 && m_drivebase.getFieldRelativeLinearSpeedsMPS().getX() < .3) 
         && //(navx.getRoll() > -13 && navx.getRoll() < 8);
         (navx.getRoll() > -8 && navx.getRoll() < 13);
         //&& (tilt(rollAvg, velAvg) > -2);
    }
    @Override
    public void end(boolean _interrupted) {

    }
}