package frc.robot.commands.docking;

import java.util.concurrent.DelayQueue;
import java.util.concurrent.Delayed;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.Limelight;
import io.github.oblarg.oblog.Loggable;
import edu.wpi.first.hal.util.HalHandleException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.drivetrain.OperatorControlC;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class DockingForceBalance extends CommandBase implements Loggable{

    double pitchUpper = -65;
    double pitchLower = -75;


    private boolean go = true;

    private final AHRS navx = new AHRS(Port.kMXP);

    private final DrivebaseS m_drivebase;

    public DockingForceBalance(DrivebaseS drive){
        m_drivebase = drive;
        addRequirements(m_drivebase);
    }

    public void brakeRobot(){
        m_drivebase.drive(0,0, 0, false);
    }

    public static double tilt(double roll, double pitch){
        double cy = (Math.cos(Units.degreesToRadians(pitch)));
        double sx = (Math.sin(Units.degreesToRadians(roll)));
        double sy = (Math.sin(Units.degreesToRadians(pitch)));

        double a = Units.radiansToDegrees(Math.asin(Math.sqrt(Math.pow(sy,2) + Math.pow(sx,2) * Math.pow(cy,2))));
        return a;
    }

    //public double distanceTrakcer(double )

    @Override
    public void initialize() {
    // Make sure motors are running in brake mode to avoid overshooting
    brakeRobot();
  }

    public double omegaToDriveSpeed(double theta, double omega){
        double kp = 1/50; //increaese until enough to go
        //value = angle of ramp * kp, multiply estimated desired kp by around 1.5
        double kd = 1/50; //have dampening term do most work
        double retSpeed = theta * kp + omega *kd;
        double posNegRet = 1;
        if (retSpeed >= 0){
            posNegRet = 1;
        }
        if (retSpeed < 0){
            posNegRet = -1;
        }
        if (Math.abs(retSpeed) > 0.3){
            retSpeed = posNegRet * 0.3;
        }
        if (omega > -2 && omega < 2){
            retSpeed = 0;
        }
        return retSpeed;
    }


    //assuumes intial if(navx.getPitch() > pitchUpper || navx.getPitch() < pitchLower when called
    @Override
    public void execute(){
        //m_drivebase.drive(new ChassisSpeeds(-omegaToDriveSpeed(navx.getRawGyroY()), 0,0));
        m_drivebase.drive(new ChassisSpeeds(omegaToDriveSpeed(navx.getRoll(), navx.getRawGyroY()),0,0));
    }
    @Override 
    public void end(boolean _interrupted){
        /* 
        if (Math.abs(navx.getPitch()) > pitchUpper){
            m_drivebase.arcadeDrive(0,0, false);
        }*/

}
}

