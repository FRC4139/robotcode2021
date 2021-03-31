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



public class ChallengeFour {

    //put constants here
    private static double FEEDER_SPEED = -0.4;
    private static double INTAKE_SPEED = 0.85;
    private static double ZERO = 0;

    //put variables here
    private double shooterSpeed = 0.5; // incremented as we move further back from target
    private DriveType driveType;

    //this is the main controller class (which we have written before), which will call the update methods below. This is NOT an Xbox Controller
    private Controller controller;   
    private double leftStickX;
    private double leftStickY;
    private double encoderDistance; 
    private int inverseMultiplier = 1; 

    /* 
        Instructions on how to get data from the robot:
            controller.getAngleFacing() returns a double indicating which angle (in degrees) the robot is facing
            controller.getUltraSonicReading() returns a double indicating the ultrasonic sensor reading in INCHES
            controller.getDistanceTravelled("fL") returns a double indicating the distance travelled by the front left wheel in FEET

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
    public ChallengeFour(Controller cIn) {
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
                > Drive the robot using the left and right joysticks to control the speed of their respective wheels OR left joystick only
                > Rev up the shooter (big wheel) using left trigger and feed (blue wheel) using right trigger
                > Increase/decrease shooter speed using X/Y buttons
                > Start intake using left bumper, reverse intake using right bumper
                > Inverse wheels using A button
                > Toggle between tank/arcade drive using B button
        
        */

        // Get joystick directions
        leftStickX = xController.getX(Hand.kLeft);
        leftStickY = xController.getY(Hand.kLeft);
        encoderDistance = controller.getDistanceTravelled("fL"); // change encoder position accordingly

        // SmartDashboard values
        SmartDashboard.putNumber("Distance travelled from start (ft)", encoderDistance);
        SmartDashboard.putNumber("Shooter speed", shooterSpeed); 

        // Reset distance recorded by encoder using A button
        // if (xController.getAButtonPressed()) {
        //     controller.resetDistance();
        // }

        // Toggle between arcade/tank drive using B button
        if(xController.getBButtonPressed()) {
            if(driveType == DriveType.ARCADE)
                driveType = DriveType.TANK;
            else
                driveType = DriveType.ARCADE;
        }
        switch(driveType) {
            case ARCADE:
                controller.diffDrive(inverseMultiplier * xController.getY(Hand.kLeft), inverseMultiplier * xController.getX(Hand.kLeft), driveType);
                break;
            case TANK:
                controller.diffDrive(inverseMultiplier * xController.getY(Hand.kLeft), inverseMultiplier * xController.getY(Hand.kRight), driveType);
        }

        // Use X/Y buttons to increase/decrease launcher speed
        if(xController.getXButtonPressed()) {
            shooterSpeed += 0.1;
        }
        if(xController.getYButtonPressed()) {
            shooterSpeed -= 0.1;
        }
        if (xController.getAButtonPressed()) { 
            inverseMultiplier *= -1; 
        }
        // Use triggers to control shooter
        // Left trigger 
        if(xController.getTriggerAxis(Hand.kLeft) > 0) {
            controller.setShooterSpeed(shooterSpeed);
        }
        else {
            controller.setShooterSpeed(ZERO);
        }
        // Right trigger
        if(xController.getTriggerAxis(Hand.kRight) > 0) {
            controller.setFeederSpeed(FEEDER_SPEED);
        }
        else {
            controller.setFeederSpeed(ZERO);
        }

        // Use left and right bumpers to control intake
        // Left bumper
        if(xController.getBumperPressed(Hand.kLeft)) {
            controller.setIntakeSpeed(INTAKE_SPEED);
        }
        if(xController.getBumperReleased(Hand.kLeft)) {
            controller.setIntakeSpeed(ZERO);
        }
        // Right bumper (reverse intake)
        if(xController.getBumperPressed(Hand.kRight)) {
            controller.setIntakeSpeed(-1 * INTAKE_SPEED);
        }
        if(xController.getBumperReleased(Hand.kRight)) {
            controller.setIntakeSpeed(ZERO);
        }

    }



}
