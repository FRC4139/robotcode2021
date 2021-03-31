package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
        if (location == "fL") {
            double temp = frontLeft.getSensorCollection().getIntegratedSensorPosition();
            double val = temp / 2048;
            return val;
        }
        if (location == "bR") {
            double temp = backRight.getSensorCollection().getIntegratedSensorPosition();
            double val = temp / 2048;
            return val;
            
        }
        return 0;

    }

    public void resetRotations() {
        frontLeft.setSelectedSensorPosition(0);
        backRight.setSelectedSensorPosition(0);
    }
    // Negative speed turns wheels backwards
    public void drive(double leftSpeed, double rightSpeed) {

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
        switch(dType) {
            case ARCADE:
                wheels.arcadeDrive(-speed1 * 0.5, -speed2 * 0.5); // speed scaling may need to be adjusted as we can't test in person right now
                break;
            case TANK:
                wheels.tankDrive(speed1 * 0.5, speed2 * 0.5);
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
}
