package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

//uncomment ones you need
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Wheels.DriveType;
// import edu.wpi.first.wpilibj.SPI;
// import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj.Timer;

//hgsgargrah

public class ChallengeFive {

    //put constants here
    private static double FEEDER_SPEED = -0.4;
    private static double ZERO = 0;
    private static double INTAKE_SPEED = 0.7;
    private static double SHOOTER_SPEED = 0.5;
    private DriveType driveType;
    //put variables here
    private double rightStickX;
    private double rightStickY;
    private double encoderDistance;
    private double angle;
    private double ultraSonic;
     private int inverse = 1;
    //this is the main controller class (which we have written before), which will call the update methods below. This is NOT an Xbox Controller
    private Controller controller;   

    /* 
        Instructions on how to get data from the robot:
            controller.getAngleFacing() returns a double indicating which angle (in degrees) the robot is facing
            controller.getUltraSonicReading() returns a double indicating the ultrasonic sensor reading in INCHES
          (check)  controller.getDistanceTravelled("fL") returns a double indicating the distance travelled by the front left wheel in FEET

        Instruction on how to control the robot:
            controller.setDriveSpeed(double leftSpeed, double rightSpeed) sets the speed of the right and left wheels. Only the wheels in the back will be powered. 
                                                                          The values MUST be between (0.9 and -0.9) (negative is backwards).
            controller.setIntakeSpeed(double speed) sets the speed of the intake. Values should be between (-0.9 and 0.9) 

            The shooter works in two seperate parts. 
            First, there is a big and strong wheel which actually launches the balls. This wheel must be charged up for 0.5 - 1.5 seconds before shooting.
            controller.setShooterSpeed(double speed) controls the speed of the big shooter wheel. This value should be between (0.4 and 0.9).             
            Second, there is a small, rubber wheel, which feeds the balls into the shooter.
            controller.setFeederSpeed(double speed) controls its speed. In order to feed balls, set it to -0.4 (FEEDER_SPEED). Otherwise, set it to ZERO.

    */

    //this is the xbox controller which will be plugged into the drive laptop to control the robot
    private XboxController xController;

    /* 
        The documentation for the xbox controller can be found here: https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/XboxController.html
        Example implementation can be seen in Controller.java
    
    */ 



    // Methods

    //constructor
    public ChallengeFive(Controller cIn) {
        controller = cIn; 
        xController = controller.xcontroller;
        driveType = DriveType.ARCADE;
    }


    /*
        Unfortunately, you cannot rapidly test your code since we don't have a physical robot to expirement with. 
        Please try to make sure your code works in theory. 
        If there are any ways in which it could go wrong, please write comments detailing how. 

    
        You should consider putting as much useful information as possible on the smart dashboard.
        Documentation can be found here: https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/smartdashboard/SmartDashboard.html
        For example: SmartDashboard.putNumber("ultra sonic reading", controller.getUltraSonicReading());

    */

    //this is called every 20 milliseconds during autonomous
    public void UpdateAutonomous() {

    }


    //this is called every 20 milliseconds during teleop (manually controlled by human with xboxcontroller)
    public void UpdateTeleop() {

        /* 
        
            Explain your controls here, so the driver knows what to do. 
                > Drive the robot using the left and right joysticks to control the speed of their respective wheels. 
        
        */
       
        // This is a very basic way of driving using two joysticks. Think about other ways the robot can be driven. Which would be the easiest and/or most efficient for the driver?
       // controller.setDriveSpeed(xController.getY(Hand.kLeft), xController.getY(Hand.kRight));
        
        rightStickX = xController.getX(Hand.kRight);
        rightStickY = xController.getY(Hand.kRight);
        encoderDistance = controller.getDistanceTravelled("fL"); // change encoder position accordingly
        angle = controller.getAngleFacing();
        ultraSonic = controller.getUltraSonicReading();
        SmartDashboard.putNumber("Distance travelled from start is: ", encoderDistance);
        SmartDashboard.putNumber("Current speed of the shooter: ", SHOOTER_SPEED); 
        SmartDashboard.putNumber("Angle is: ", angle);
        SmartDashboard.putNumber("Ultra Sonic Reading is: " , + ultraSonic);

      //  controller.diffDrive(inverseMultiplier * xController.getY(Hand.kLeft), inverseMultiplier * xController.getX(Hand.kLeft), driveType);
        //        break;
          //  case TANK:
            //    controller.diffDrive(inverseMultiplier * xController.getY(Hand.kLeft), inverseMultiplier * xController.getY(Hand.kRight), driveType);
        //}
//right joytick-the intake.

//drives robot with left joystick

controller.diffDrive(inverse * xController.getY(Hand.kLeft), inverse * xController.getX(Hand.kLeft), driveType);
//X button-inverse 

if(xController.getXButtonPressed())
{
    inverse = inverse * -1;
}

//intake moves up when right joystick is up
if(rightStickY > 0)
{
    controller.setIntakeSpeed(INTAKE_SPEED);
}
else
{
    controller.setIntakeSpeed(ZERO);
}
//intake moves down when right joystick is down
if(rightStickY < 0)
{
    controller.setIntakeSpeed(INTAKE_SPEED);
}
else
{
controller.setIntakeSpeed(ZERO);
}
//yyu7g
//Y button-increase to high shooter speed

    if(xController.getYButtonPressed())
        {
            SHOOTER_SPEED += 0.3;
         }
        

//B button-decrease to low shooter speed

if(xController.getBButtonPressed())
    {
        SHOOTER_SPEED -= 0.3;
    }


        //Left Trigger-Activates the feeder motor
        if(xController.getTriggerAxis(Hand.kLeft) > 0) 
            {
            controller.setFeederSpeed(FEEDER_SPEED);
             }
       else{
             controller.setFeederSpeed(ZERO);
             }
        //Right Trigger-Activates the Shooter motor
        if(xController.getTriggerAxis(Hand.kRight) > 0)
             {
             controller.setShooterSpeed(SHOOTER_SPEED);
            }
           else 
            {
            controller.setShooterSpeed(ZERO);
            }

    }



}
