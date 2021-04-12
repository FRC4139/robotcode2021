package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.SlewRateLimiter;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
//import java.lang.Math;

public class Wheels {

    public enum DriveType { TANK, ARCADE }

    private WPI_TalonFX frontLeft;
    private WPI_TalonFX backLeft;
    private WPI_TalonFX frontRight;
    private WPI_TalonFX backRight;
    private DifferentialDrive wheels;
    private boolean inverseState;
    private SlewRateLimiter leftLimiter = new SlewRateLimiter(1);
    private SlewRateLimiter righLimiter = new SlewRateLimiter(1.5);

    public Wheels(int fL, int bL, int fR, int bR) {
        //initialize motor objects
        frontLeft = new WPI_TalonFX(fL);
        backLeft = new WPI_TalonFX(bL);
        frontRight = new WPI_TalonFX(fR);
        backRight = new WPI_TalonFX(bR);
        //so motors brake when speed is 0
        frontRight.setNeutralMode(NeutralMode.Brake);
        frontLeft.setNeutralMode(NeutralMode.Brake);
        backRight.setNeutralMode(NeutralMode.Brake);
        backLeft.setNeutralMode(NeutralMode.Brake);
        
        inverseState = false;
        wheels = new DifferentialDrive(new SpeedControllerGroup(frontLeft, backLeft), new SpeedControllerGroup(frontRight, backRight));
    }

    public double getRotations(String location) { 
        if (location == "fL") return frontLeft.getSensorCollection().getIntegratedSensorPosition() / 4096;
        if (location == "bR") return backRight.getSensorCollection().getIntegratedSensorPosition() / 4096;
        return 0;

    }

    public void resetRotations() {
        resetEncoders();
    }
    // Negative speed turns wheels backwards
    public void drive(double leftSpeed, double rightSpeed) {


        //System.out.println("Setting speeds to: " + leftSpeed + " " + rightSpeed);
        leftSpeed *= 0.5;
        rightSpeed *= 0.5;
        if(!inverseState)
        {
            //if inversState is true, reverse the speeds and call drive 
            frontLeft.set(leftSpeed);
            backLeft.set(leftSpeed);
            frontRight.set(-rightSpeed);
            backRight.set(-rightSpeed);

        }
        else
        {
            //makes sure speeds are positive
            //wheels.tankDrive(Math.abs(leftSpeed), Math.abs(rightSpeed));
            frontLeft.set(-leftSpeed);
            backLeft.set(-leftSpeed);
            frontRight.set(rightSpeed);
            backRight.set(rightSpeed);
       
        }
    }

    public void diffDrive(double speed1, double speed2, DriveType dType) {
        //System.out.println("Setting speeds to: " + speed1 + " " + speed2);

        speed1 *= 0.75; 
        speed2 *= 0.6;
        speed1 = leftLimiter.calculate(speed1);
        speed2 = righLimiter.calculate(speed2);

        switch(dType) {
            case ARCADE:
                wheels.arcadeDrive(speed1, speed2); // speed scaling may need to be adjusted as we can't test in person right now
                break;
            case TANK:
                wheels.tankDrive(speed1, speed2);
                break;
        }
    }

    public void inverse()
    {
        inverseState = !inverseState;
    }       //reverses inverse everytime pressed

    public void resetEncoders() {
        frontLeft.getSensorCollection().setIntegratedSensorPosition(0, 0);
    }

    public void ShowSpeedsOnDashboard() {
        //System.out.println(new double[] {frontLeft.get(), backLeft.get(), frontRight.get(), backRight.get() });
        double[] zzz = new double[] {frontLeft.get(), backLeft.get(), frontRight.get(), backRight.get() };
        SmartDashboard.putNumber("Speeds fL", zzz[0]);
    }

	public double[] getRawSpeeds() {
		return new double[] { frontLeft.get(), backLeft.get(), frontRight.get(), backRight.get() };
    }
    
    public void setRawSpeeds(double[] speeds) {
        frontLeft.set(speeds[0]); 
        backLeft.set(speeds[1]);
        frontRight.set(speeds[2]);
        backRight.set(speeds[3]);
    }
}
