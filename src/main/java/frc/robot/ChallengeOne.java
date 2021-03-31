package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.XboxController;

//uncomment ones you need
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.SPI;
// import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj.Timer;



public class ChallengeOne {

    //put constants here
    private static double FEEDER_SPEED = -0.4;
    private static double ZERO = 0;
    private static double INTAKE_SPEED = -0.85;
    

    private static final double DISTANCE_PIVOT_TO_WHEEL = 22.5 / 2;

    private static final double MAX_DRIVE_SPEED = 0.8;
    private static final double OUTER_TURN_DRIVE_SPEED = 0.6; 

    private static final double[] STRAIGHT_INTAKE_OFF = { MAX_DRIVE_SPEED, MAX_DRIVE_SPEED, 0};
    private static final double[] STRAIGHT_INTAKE_ON = { MAX_DRIVE_SPEED, MAX_DRIVE_SPEED, INTAKE_SPEED};

    private static final double TURN_RADIUS = 20.0;

    //put variables here
    private double COMPASS_OFFSET = 0; 


    //this is the main controller class (which we have written before), which will call the update methods below. This is NOT an Xbox Controller
    private Controller controller;   

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

    private ArrayList<AutonomousSegment> path = new ArrayList<AutonomousSegment>();

    private int currentSegment = 1;
    private String whichPath;



    // Methods

    //constructor
    public ChallengeOne(Controller cIn) {
        controller = cIn; 
        xController = controller.xcontroller;

        //determine which path we are on 
        if (getCompassHeading() < 1 && getCompassHeading() > -1) {
            whichPath = "RED A";
            
            /*
            Place center of front of robot at (30, my + TURN_RADIUS) at angle 0 radians
            */

            //following variables are defined by the points in the geogebra diagram
            double mx = 90 - TURN_RADIUS * Math.cos(Math.atan(4.15315));
            double my = 90 - TURN_RADIUS * Math.sin(Math.atan(4.15315));
            double ox = 150 + TURN_RADIUS * Math.cos(Math.PI + Math.atan(-2.41424));
            double oy = 60 + TURN_RADIUS * Math.sin(Math.PI + Math.atan(-2.41424));
            double distMO = Math.sqrt(Math.pow(mx-ox,2)+Math.pow((my-oy),2));

            double px = 180 + TURN_RADIUS * Math.cos(Math.atan(-1.38743));
            double py = 150 + TURN_RADIUS * Math.sin(Math.atan(-1.38743));
            double distOP = Math.sqrt(Math.pow(ox-px,2)+Math.pow((oy-py),2));

            path.add(createStraightAutonomousSegment(mx-30, 1, 0, new AutonomousSegment(false)));
            path.add(createCircularAutonomousSegment(TURN_RADIUS, Math.PI / 2 - (Math.acos(2 * TURN_RADIUS / (distMO))) - Math.atan2(oy-my, ox-mx ), 1, true, INTAKE_SPEED, path.get(path.size() - 1)));
            path.add(createStraightAutonomousSegment(Math.tan(Math.acos(2 * TURN_RADIUS / (distMO))) * TURN_RADIUS * 2, 1, 0, path.get(path.size() - 1)));
            path.add(createCircularAutonomousSegment(TURN_RADIUS, Math.PI - (  Math.acos(2 * TURN_RADIUS / (distMO)) + Math.acos(2 * TURN_RADIUS / (distOP)) + Math.atan2(oy-my, ox-mx) - Math.atan2(py-oy, px-ox) ), 1, false, INTAKE_SPEED, path.get(path.size() - 1)));
            path.add(createStraightAutonomousSegment(Math.tan(Math.acos(2 * TURN_RADIUS / (distOP))) * TURN_RADIUS * 2, 1, 0, path.get(path.size() - 1)));
            path.add(createCircularAutonomousSegment(TURN_RADIUS, Math.PI/2 - Math.acos(2 * TURN_RADIUS / (distOP)) + Math.atan2(py-oy , px - ox), 1, true, INTAKE_SPEED, path.get(path.size() - 1)));
            path.add(createStraightAutonomousSegment(330-px, 1, 0, path.get(path.size() - 1))); //increase 330 if robot is stopping too soon

        } else if (getCompassHeading() <= -15) {
            whichPath = "RED B";

            /*
            Place center of front of robot at (ax,ay) at angle (Math.asin(TURN_RADIUS/distAE) - Math.atan2(ay-ey,ex-ax)) radians
            */
            //following variables are defined by the points in the geogebra diagram
            double B1x = 30;
            double B1y = 120;

            double ex = 90 - TURN_RADIUS * Math.cos(Math.atan(1.58579));
            double ey = 120 - TURN_RADIUS * Math.sin(Math.atan(1.58579));
            double distB1e = Math.sqrt(Math.pow(B1x-ex,2)+Math.pow((B1y-ey),2));

            double ax = 30;
            double ay = B1y + TURN_RADIUS / Math.cos(Math.atan2(B1y-ey, ex-B1x)); // starting x-coord
            double distAE = Math.sqrt(Math.pow(ax-ex,2)+Math.pow((ay-ey),2));     // starting y-coord

            double gx = 150;
            double gy = 60 + TURN_RADIUS;
            double distEG = Math.sqrt(Math.pow(ex-gx,2)+Math.pow((ey-gy),2));

            double jx = 210 + TURN_RADIUS * Math.cos(Math.atan(-1.93771));
            double jy = 120 + TURN_RADIUS * Math.sin(Math.atan(-1.93771));
            double distGJ = Math.sqrt(Math.pow(jx-gx,2)+Math.pow((jy-gy),2));

            double B11x = 330;
            double B11y = 120;
            double cx = 330;
            double cy = B11y + TURN_RADIUS / Math.cos(Math.atan2(B11y-jy,B11x-jx));
            double distJC = Math.sqrt(Math.pow(jx-cx,2)+Math.pow((jy-cy),2));

            path.add(createStraightAutonomousSegment(distAE * Math.sin(Math.acos(TURN_RADIUS/distAE)), 1, 0, new AutonomousSegment(false)));
            path.add(createCircularAutonomousSegment(TURN_RADIUS, 2*Math.PI - ( Math.PI/2 + Math.atan2(ay-ey,ex-ax) + Math.acos(TURN_RADIUS / distAE) + Math.acos(2*TURN_RADIUS / distEG) + Math.atan2(gx-ex,ey-gy) ), 1, true, INTAKE_SPEED, path.get(path.size() - 1)));
            path.add(createStraightAutonomousSegment(Math.tan(Math.acos(2 * TURN_RADIUS / (distEG))) * TURN_RADIUS * 2, 1, 0, path.get(path.size() - 1)));
            path.add(createCircularAutonomousSegment(TURN_RADIUS, 2*Math.PI - ( Math.acos(2*TURN_RADIUS/distEG) + Math.atan2(gx-ex,ey-gy) + Math.atan2(jx-gx,jy-gy) + Math.acos(2*TURN_RADIUS / distGJ) ), 1, false, INTAKE_SPEED, path.get(path.size() - 1)));
            path.add(createStraightAutonomousSegment(Math.tan(Math.acos(2 * TURN_RADIUS / (distGJ))) * TURN_RADIUS * 2, 1, 0, path.get(path.size() - 1)));
            path.add(createCircularAutonomousSegment(TURN_RADIUS, 2*Math.PI - ( Math.PI/2 + Math.atan2(jx-gx,jy-gy) + Math.acos(2*TURN_RADIUS / distGJ) + Math.acos(TURN_RADIUS / distJC) + Math.atan2(cy-jy,cx-jx) ), 1, true, INTAKE_SPEED, path.get(path.size() - 1)));
            path.add(createStraightAutonomousSegment(0 + distJC * Math.sin(Math.acos(TURN_RADIUS/distJC)), 1, 0, path.get(path.size() - 1))); //increase 0 if robot is stopping too soon
            

        } else if (getCompassHeading() <= -1 && getCompassHeading() > -15) {
            whichPath = "BLUE A";
            
            /*
            Place center of front of robot at (kx,ky) at angle (Math.atan2(ny-ky, nx-kx) - Math.asin(TURN_RADIUS/distKN)) radians
            */
            //following variables are defined by the points in the geogebra diagram
            double kx = 30;     //starting x-coord
            double ky = 30;     //starting y-coord
            double nx = 180 - TURN_RADIUS * Math.cos(Math.atan(-1.38743));
            double ny = 30 - TURN_RADIUS * Math.sin(Math.atan(-1.38743));
            double distKN = Math.sqrt(Math.pow(kx-nx,2)+Math.pow((ky-ny),2));

            double zx = 210 - TURN_RADIUS * Math.cos(Math.PI + Math.atan(-2.41421));
            double zy = 120 - TURN_RADIUS * Math.sin(Math.PI + Math.atan(-2.41421));
            double distNZ = Math.sqrt(Math.pow(nx-zx,2)+Math.pow((ny-zy),2));

            double a1x = 270 + TURN_RADIUS * Math.cos(Math.atan(4.23607));
            double a1y = 90 + TURN_RADIUS * Math.sin(Math.atan(4.23607));
            double distZA1 = Math.sqrt(Math.pow(zx-a1x,2)+Math.pow((zy-a1y),2));

            path.add(createStraightAutonomousSegment(distKN * Math.sin(Math.acos(TURN_RADIUS/distKN)), 1, 0, new AutonomousSegment(false)));
            path.add(createCircularAutonomousSegment(TURN_RADIUS, Math.PI - ( Math.acos(2 * TURN_RADIUS / distNZ) - Math.atan2(zy-ny,zx-nx) + Math.acos(TURN_RADIUS/distKN) + Math.atan2(ny-ky,nx-kx) ), 1, false, INTAKE_SPEED, path.get(path.size() - 1)));
            path.add(createStraightAutonomousSegment(Math.tan(Math.acos(2 * TURN_RADIUS / (distNZ))) * TURN_RADIUS * 2, 1, 0, path.get(path.size() - 1)));
            path.add(createCircularAutonomousSegment(TURN_RADIUS, Math.PI - (  Math.acos(2 * TURN_RADIUS / (distZA1)) + Math.acos(2 * TURN_RADIUS / (distNZ)) + Math.atan2(a1y-zy, a1x-zx) - Math.atan2(zy-ny, zx-nx) ), 1, true, INTAKE_SPEED, path.get(path.size() - 1)));
            path.add(createStraightAutonomousSegment(Math.tan(Math.acos(2 * TURN_RADIUS / (distZA1))) * TURN_RADIUS * 2, 1, 0, path.get(path.size() - 1)));
            path.add(createCircularAutonomousSegment(TURN_RADIUS, Math.PI/2 - (Math.acos(2 * TURN_RADIUS / (distZA1)) + Math.atan2(a1y-zy , a1x - zx)), 1, false, INTAKE_SPEED, path.get(path.size() - 1)));
            path.add(createStraightAutonomousSegment(330-a1x, 1, 0, path.get(path.size() - 1))); //increase 330 if robot is stopping too soon
        } else {
            whichPath = "BLUE B";
            
            /*
            Place center of front of robot at (tx,ty) at angle (Math.atan2(a1y-ty,a1x-tx) - Math.asin(TURN_RADIUS/distTA1)) radians
            */
            //following variables are defined by the points in the geogebra diagram
            double D1x = 30;
            double D1y = 60;

            double a1x = 180 - TURN_RADIUS * Math.cos(Math.atan(-2.02058));
            double a1y = 60 - TURN_RADIUS * Math.sin(Math.atan(-2.02058));

            double tx = 30;                                                         //starting x-coord
            double ty = D1y - TURN_RADIUS / Math.cos(Math.atan2(a1y-D1y, a1x-D1x)); //starting y-coord
            double distTA1 = Math.sqrt(Math.pow(tx-a1x,2)+Math.pow((ty-a1y),2));

            double wx = 240;
            double wy = 120 - TURN_RADIUS;
            double distA1W = Math.sqrt(Math.pow(a1x-wx,2)+Math.pow((a1y-wy),2));

            double D10x = 300;
            double D10y = 60;
            double distWD10 = Math.sqrt(Math.pow(wx-D10x,2)+Math.pow((wy-D10y),2));

            double e1x = 330;
            double e1y = 60 + 30*Math.tan(Math.PI-(Math.asin(TURN_RADIUS/distWD10)+Math.atan2(wy-D10y,D10x-wx)));
            double distWE1 = Math.sqrt(Math.pow(wx-e1x,2)+Math.pow((wy-e1y),2));

            path.add(createStraightAutonomousSegment(distTA1 * Math.sin(Math.acos(TURN_RADIUS/distTA1)), 1, 0, new AutonomousSegment(false)));
            path.add(createCircularAutonomousSegment(TURN_RADIUS, 2*Math.PI - ( Math.PI/2 + Math.atan2(wx-a1x,wy-a1y) + Math.acos(2*TURN_RADIUS / distA1W) + Math.acos(TURN_RADIUS / distTA1) + Math.atan2(a1y-ty,a1x-tx) ), 1, false, INTAKE_SPEED, path.get(path.size() - 1)));
            path.add(createStraightAutonomousSegment(Math.tan(Math.acos(2 * TURN_RADIUS / (distA1W))) * TURN_RADIUS * 2, 1, 0, path.get(path.size() - 1)));
            path.add(createCircularAutonomousSegment(TURN_RADIUS, 2*Math.PI - ( Math.acos(2*TURN_RADIUS/distA1W) + Math.atan2(wx-a1x,wy-a1y) + Math.atan2(e1x-wx,wy-e1y) + Math.acos(TURN_RADIUS / distWE1) ), 1, true, INTAKE_SPEED, path.get(path.size() - 1)));
            path.add(createStraightAutonomousSegment(distWE1 * Math.sin(Math.acos(TURN_RADIUS/distWE1)), 1, INTAKE_SPEED, path.get(path.size() - 1)));
            

        }

        //add segments to paths here
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
        // Display useful information
        SmartDashboard.putString("Current Path", whichPath);
        SmartDashboard.putNumber("Current Segment", currentSegment);
        SmartDashboard.putNumber("Total distance travelled (in)", getDistanceTravelled());
        SmartDashboard.putNumber("Angle Facing Real (deg)", controller.getAngleFacing());
        SmartDashboard.putNumber("Angle Facing Adjusted (deg)", getAngleFacing());
        SmartDashboard.putNumber("Compass Offset", COMPASS_OFFSET);
        SmartDashboard.putNumber("Compass Heading", getCompassHeading());
        if (currentSegment == path.size()) { 
            controller.setDriveSpeed(0, 0);
            controller.setIntakeSpeed(0);
            return;
        }
        
        if (path.get(currentSegment - 1).IsSegmentComplete(getDistanceTravelled(), getAngleFacing())) {
            currentSegment++;
            if (currentSegment == path.size()) { 
                controller.setDriveSpeed(0, 0);
                controller.setIntakeSpeed(0);
                return;
            }
            path.get(currentSegment - 1).SetSpeeds(controller);
        }
    }


    //this is called every 20 milliseconds during teleop (manually controlled by human with xboxcontroller)
    public void UpdateTeleop() {
        SmartDashboard.putString("Current Path", whichPath);
        SmartDashboard.putNumber("Current Segment", currentSegment);
        SmartDashboard.putNumber("Total distance travelled (in)", getDistanceTravelled());
        SmartDashboard.putNumber("Angle Facing Real (deg)", controller.getAngleFacing());
        SmartDashboard.putNumber("Angle Facing Adjusted (deg)", getAngleFacing());
        SmartDashboard.putNumber("Compass Offset", COMPASS_OFFSET);
        SmartDashboard.putNumber("Compass Heading", getCompassHeading());

        /* 
        
            Explain your controls here, so the driver knows what to do. 
                > Drive the robot using the left and right joysticks to control the speed of their respective wheels. 
        
        */

        // This is a very basic way of driving using two joysticks. Think about other ways the robot can be driven. Which would be the easiest and/or most efficient for the driver?
        controller.setDriveSpeed(xController.getY(Hand.kLeft), xController.getY(Hand.kRight));

        if (xController.getBumper(Hand.kLeft)) { controller.calibrate(); }
        if (Math.abs(xController.getTriggerAxis(Hand.kLeft)) > 0.05) COMPASS_OFFSET += xController.getTriggerAxis(Hand.kLeft);

    }

    //radius is of center; this function accounts for distance to wheel
    //direction use 1 or -1 
    //rotation: true = cw; false = ccw;

    private AutonomousSegment createCircularAutonomousSegment(double radius, double angle, int direction, boolean rotation, double intakeSpeed, AutonomousSegment prev) {

        boolean distanceGreater = true;
        if (direction == -1) distanceGreater = false;

        

        if ((rotation && direction == 1) || (!rotation && direction == -1)) {
            //cw forward = ccw backwards = left faster
            return new AutonomousSegment((radius + DISTANCE_PIVOT_TO_WHEEL)* Math.abs(angle) * direction, 
                                        angle * -1, 
                                        prev,
                                        new double[] {OUTER_TURN_DRIVE_SPEED * direction, GetInnerTurnSpeed(radius) * direction, intakeSpeed}, 
                                        distanceGreater,
                                        !rotation); 
        } else {
            return new AutonomousSegment((radius - DISTANCE_PIVOT_TO_WHEEL) * Math.abs(angle) * direction, 
                                        angle, 
                                        prev,
                                        new double[] {GetInnerTurnSpeed(radius) * direction, OUTER_TURN_DRIVE_SPEED*direction, intakeSpeed}, 
                                        distanceGreater,
                                        !rotation); 
        }
        
    }

    private AutonomousSegment createStraightAutonomousSegment(double length, int direction, double intakeSpeed, AutonomousSegment prev) {
        boolean distanceGreater = true;
        if (direction == -1) distanceGreater = false;

        return new AutonomousSegment(length, 0, prev, new double[] { MAX_DRIVE_SPEED * direction, MAX_DRIVE_SPEED * direction, intakeSpeed}, distanceGreater, false);
    }

    private double GetInnerTurnSpeed(double CIRCLE_RADIUS) {

        return (CIRCLE_RADIUS - DISTANCE_PIVOT_TO_WHEEL) / (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL) * OUTER_TURN_DRIVE_SPEED;
    }
    
    private double getDistanceTravelled() {
        return controller.getDistanceTravelled("fL") * 12.0;
    }

    //cw is positive 
    private double getAngleFacing() {
        return Math.toRadians(controller.getAngleFacing());
    }

    private double getCompassHeading() { 
        return controller.getCompassHeading() + COMPASS_OFFSET;
    }

}
