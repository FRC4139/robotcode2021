package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.XboxController;

//uncomment ones you need
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.SPI;
// import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj.Timer;



public class ChallengeTwo {

    //put constants here
    private static final double FEEDER_SPEED = -0.4;
    private static final double ZERO = 0;

    private static final double DISTANCE_PIVOT_TO_WHEEL = 22.5 / 2;

    private static final double MAX_DRIVE_SPEED = 0.8;
    private static final double OUTER_TURN_DRIVE_SPEED = 0.6; 

    public static final boolean isSaatvikDumb = true;

    //constants for path 3 (bounce)
    private static final double R_T_ONE = 20; // we can change this
    private static final double R_T_TWO =  (1800 - Math.pow(R_T_ONE, 2)) / (60 - 2 * R_T_ONE); 
    private static final double R_T_THREE = ( 11700 - Math.pow(R_T_ONE, 2)) / ( 120 - 2 * R_T_ONE); 
    private static final double R_T_FOUR = R_T_ONE;
    private static final double R_T_FIVE = (3400 - Math.pow(R_T_ONE, 2)) / (60 - 2 * R_T_ONE);
    private static final double R_T_SIX = R_T_FIVE;
    private static final double R_T_SEVEN = R_T_ONE;
    private static final double R_T_NINE = R_T_ONE;
    private static final double R_T_TEN = (9000 - Math.pow(R_T_ONE, 2)) / (60 - 2 * R_T_ONE);
    private static final double R_T_ELEVEN = R_T_TWO;
    private static final double R_T_TWELVE = R_T_ONE;
    //------------------------------------------------------
    //put variables here
    private double CIRCLE_RADIUS = 20.0; // changed throughout path 3 code 
    private double TURN_RADIUS = 20.0; //for the sake of consistency
    private double INNER_TURN_DRIVE_SPEED = (CIRCLE_RADIUS - DISTANCE_PIVOT_TO_WHEEL) / (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL) * OUTER_TURN_DRIVE_SPEED; 

    private int segment = 1; 
    private int path = 1;
    //this is the main controller class (which we have written before), which will call the update methods below. This is NOT an Xbox Controller
    private Controller controller;   
    private ArrayList<ArrayList<AutonomousSegment>> pathSegments = new ArrayList<ArrayList<AutonomousSegment>>();
    private SendableChooser pathSelector;

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
    private boolean started = false;

    /* 
        The documentation for the xbox controller can be found here: https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/XboxController.html
        Example implementation can be seen in Controller.java
    
    */

    /* 
        Starting positions:

        Barrel Path:
        Slalom Path: 
        Bounce Path: Front right corner should be on the vertical connecting B2, D2 and should be X inches above D2. The robot should be 

    
    
    */

    // Methods

    //constructor
    public ChallengeTwo(Controller cIn, int p) {
        controller = cIn; 
        path = p;
        xController = controller.xcontroller;

        pathSelector = new SendableChooser();
        pathSelector.addOption("Path One (Barrel)", 0);
        pathSelector.addOption("Path Two (Slalom)", 1);
        pathSelector.addOption("Path Three (Bounce)", 2);

        //createCircularAutonomousSegment(double radius, double angle, int direction, boolean rotation, double intakeSpeed, AutonomousSegment prev)
        //createStraightAutonomousSegment(double length, int direction, double intakeSpeed, AutonomousSegment prev)
        if (path == 1) {
            ArrayList<AutonomousSegment> path1 = new ArrayList<AutonomousSegment>(); 

            //start robot at (60, 60 + TURN_RADIUS) facing 0 radians

            //variables defined from the geogebra diagram

            double d5x = 150;
            double d5y = 60;
            double b8x = 240;
            double b8y = 120;
            double distD5B8 = Math.sqrt(Math.pow(d5x-b8x,2) + Math.pow(d5y-b8y,2));

            double d10x = 300;
            double d10y = 60;
            double distB8D10 = Math.sqrt(Math.pow(d10x-b8x,2) + Math.pow(d10y-b8y,2));

            path1.add(createStraightAutonomousSegment(90, 1, 0, new AutonomousSegment(false)));
            path1.add(createCircularAutonomousSegment(TURN_RADIUS, 2.5 * Math.PI - Math.acos(2* TURN_RADIUS / distD5B8) - Math.atan2(b8y-d5y,b8x-d5x), 1, true, 0, path1.get(path1.size() - 1)));
            path1.add(createStraightAutonomousSegment(Math.sin(Math.acos(2* TURN_RADIUS / distD5B8)) * distD5B8, 1, 0, path1.get(path1.size() - 1)));
            path1.add(createCircularAutonomousSegment(TURN_RADIUS, 2 * Math.PI - (Math.atan2(b8y-d5y,b8x-d5x) + Math.PI/4 - Math.asin(2*TURN_RADIUS/distD5B8)) , 1, false, 0, path1.get(path1.size()-1)));
            path1.add(createStraightAutonomousSegment(distB8D10, 1, 0, path1.get(path1.size() - 1)));
            path1.add(createCircularAutonomousSegment(TURN_RADIUS, 1.25 * Math.PI , 1, false, 0, path1.get(path1.size() - 1)));
            path1.add(createStraightAutonomousSegment(240, 1, 0, path1.get(path1.size() - 1)));

            path1.add(createStraightAutonomousSegment(0, 1, 0, path1.get(path1.size()-1))); //increase 0 if robot is stopping too soon

            pathSegments.add(path1);
        } else if (path == 2) {
            ArrayList<AutonomousSegment> path2 = new ArrayList<AutonomousSegment>(); 

            //start robot at (60, 60 - TURN_RADIUS) facing 0 radians

            //variables defined from the geogebra diagram

            double alpha = Math.PI/2 - Math.acos(2 * TURN_RADIUS / 60);

            path2.add(createCircularAutonomousSegment(TURN_RADIUS, alpha, 1, false, 0, new AutonomousSegment(false)));
            path2.add(createStraightAutonomousSegment(60 * Math.cos(alpha) , 1, 0, path2.get(path2.size() - 1)));
            path2.add(createCircularAutonomousSegment(TURN_RADIUS, alpha, 1, true, 0, path2.get(path2.size()-1)));
            path2.add(createStraightAutonomousSegment(120, 1, 0, path2.get(path2.size()-1)));
            path2.add(createCircularAutonomousSegment(TURN_RADIUS, alpha, 1, true, 0, path2.get(path2.size()-1)));
            path2.add(createStraightAutonomousSegment(60 * Math.cos(alpha) , 1, 0, path2.get(path2.size() - 1)));
            path2.add(createCircularAutonomousSegment(TURN_RADIUS, Math.PI + 2*alpha, 1, false, 0, path2.get(path2.size()-1)));
            path2.add(createStraightAutonomousSegment(60 * Math.cos(alpha) , 1, 0, path2.get(path2.size() - 1)));
            path2.add(createCircularAutonomousSegment(TURN_RADIUS, alpha, 1, true, 0, path2.get(path2.size()-1)));
            path2.add(createStraightAutonomousSegment(120, 1, 0, path2.get(path2.size()-1)));
            path2.add(createCircularAutonomousSegment(TURN_RADIUS, alpha, 1, true, 0, path2.get(path2.size()-1)));
            path2.add(createStraightAutonomousSegment(60 * Math.cos(alpha) , 1, 0, path2.get(path2.size() - 1)));
            path2.add(createCircularAutonomousSegment(TURN_RADIUS, alpha, 1, false, 0, path2.get(path2.size()-1)));

            path2.add(createStraightAutonomousSegment(0, 1, 0, path2.get(path2.size()-1))); //increase 0 if robot is stopping too soon


            pathSegments.add(path2);
        } else {
            ArrayList<AutonomousSegment> path3 = new ArrayList<AutonomousSegment>(); 

            //start robot at (60, 120 - TURN_RADIUS) facing 0 radians

            //variables defined from the geogebra diagram

            double fx = 90 - (1800 - Math.pow(TURN_RADIUS,2))/(60 - 2*TURN_RADIUS);
            double fy = 150;
            double fradius = (1800 - Math.pow(TURN_RADIUS,2))/(60 - 2*TURN_RADIUS);

            double bx = 60;
            double by = 120;

            double rx = 90 + (11700-Math.pow(TURN_RADIUS,2))/(120-2*TURN_RADIUS);
            double ry = 150;
            double rradius = (11700-Math.pow(TURN_RADIUS,2))/(120-2*TURN_RADIUS);

            double ox = 150;
            double oy = 60;

            double ux = 180 - (3400-Math.pow(TURN_RADIUS,2))/(60-2*TURN_RADIUS);
            double uy = 110;
            double uradius = (3400-Math.pow(TURN_RADIUS,2))/(60-2*TURN_RADIUS);

            double wx = 180 + uradius;
            double wy = 110;
            double wradius = uradius;

            double px = 210;
            double py = 60;

            double c1x = 270 - (9000-Math.pow(TURN_RADIUS,2))/(60-2*TURN_RADIUS);
            double c1y = 150;
            double c1radius = (9000-Math.pow(TURN_RADIUS,2))/(60-2*TURN_RADIUS);

            double qx = 240;
            double qy = 60;

            path3.add(createCircularAutonomousSegment(TURN_RADIUS, Math.atan2(bx-fx,fy-by), 1, false, 0, new AutonomousSegment(false)));
            path3.add(createCircularAutonomousSegment(fradius, Math.PI/4 + Math.atan2(fy-by,bx-fx), 1, false, 0, path3.get(path3.size()-1)));

            path3.add(createCircularAutonomousSegment(rradius, Math.atan2(ry-oy,rx-ox), -1, false, 0, path3.get(path3.size()-1)));
            path3.add(createCircularAutonomousSegment(TURN_RADIUS, Math.PI - (Math.atan2(ry-oy,rx-ox) + Math.atan2(uy-oy,ox-ux)), -1, false, 0, path3.get(path3.size()-1)));
            path3.add(createCircularAutonomousSegment(uradius, Math.atan2(uy-oy,ox-ux), -1, false, 0, path3.get(path3.size()-1)));

            path3.add(createCircularAutonomousSegment(wradius, Math.atan2(wy-py,wx-px), 1, false, 0, path3.get(path3.size()-1)));
            path3.add(createCircularAutonomousSegment(TURN_RADIUS, Math.PI/2 - Math.atan2(wy-py,wx-px), 1, false, 0, path3.get(path3.size()-1)));
            path3.add(createStraightAutonomousSegment(30, 1, 0, path3.get(path3.size()-1)));
            path3.add(createCircularAutonomousSegment(TURN_RADIUS, Math.PI/2 - Math.atan2(c1y-qy,qx-c1x), 1, false, 0, path3.get(path3.size()-1)));
            path3.add(createCircularAutonomousSegment(c1radius, Math.atan2(c1y-qy,qx-c1x), 1, false, 0, path3.get(path3.size()-1)));

            //symmetric with first two segments
            path3.add(createCircularAutonomousSegment(fradius, Math.PI/4 + Math.atan2(fy-by,bx-fx), -1, false, 0, path3.get(path3.size()-1)));
            path3.add(createCircularAutonomousSegment(TURN_RADIUS, Math.atan2(bx-fx,fy-by), -1, false, 0, path3.get(path3.size()-1)));

            path3.add(createStraightAutonomousSegment(0, -1, 0, path3.get(path3.size()-1))); //increase 0 if robot is stopping too soon

            pathSegments.add(path3);
        }
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
        SmartDashboard.putNumber("Current Path", path);
        SmartDashboard.putNumber("Path Length", pathSegments.size());
        SmartDashboard.putNumber("Current Segment", segment);
        SmartDashboard.putNumber("Total distance travelled (in)", getDistanceTravelled());
        SmartDashboard.putNumber("Angle Facing Real (deg)", controller.getAngleFacing());
        SmartDashboard.putNumber("Angle Facing Adjusted (rad)", getAngleFacing());
        SmartDashboard.putNumber("Current Circle Radius (in)", CIRCLE_RADIUS);
        SmartDashboard.putNumber("Inner Turn Speed", INNER_TURN_DRIVE_SPEED);

        path = (int) pathSelector.getSelected();

        
        if (!started) { 
            pathSegments.get(path).get(0).SetSpeeds(controller);
            started = true;
        }

        if (pathSegments.get(path).get(segment - 1).IsSegmentComplete(getDistanceTravelled(), getAngleFacing())) {
            segment++;
            pathSegments.get(path).get(segment - 1).SetSpeeds(controller);
        }

        if (segment == pathSegments.size()) { 
            controller.setDriveSpeed(0, 0);
            controller.setIntakeSpeed(0);
            return;
        }



        // if (path == 1) {
        //     if (segment == 1) {
        //         SetSpeedStraight();
        //     } else if (segment == 2) {
        //         SetSpeedClockwise(); 
        //     } else if (segment == 3) {
        //         SetSpeedStraight();
        //     } else if (segment == 4) {
        //         SetSpeedCounterClockwise(); 
        //     } else if (segment == 5) {
        //         SetSpeedStraight();
        //     } else if (segment == 6) {
        //         SetSpeedCounterClockwise(); 
        //     } else if (segment == 7) {
        //         SetSpeedStraight();
        //     } else if (segment == 8) {
        //         controller.setDriveSpeed(0, 0);
        //     }
        // } else if (path == 2) {
        //     if      (segment == 1) { SetSpeedStraight(); }
        //     else if (segment == 2) { SetSpeedClockwise(); } 
        //     else if (segment == 3) { SetSpeedStraight(); }
        //     else if (segment == 4) { SetSpeedClockwise(); } 
        //     else if (segment == 5) { SetSpeedStraight(); }
        //     else if (segment == 6) { SetSpeedCounterClockwise(); } 
        //     else if (segment == 7) { SetSpeedStraight();}
        //     else if (segment == 8) { SetSpeedClockwise(); }
        //     else if (segment == 9) { SetSpeedStraight();}
        //     else if (segment == 10) { SetSpeedClockwise(); }
        //     else if (segment == 11) { SetSpeedStraight(); } 
        //     else if (segment == 12) { controller.setDriveSpeed(0, 0); }
        // } else if (path == 3) {
        //     if      (segment == 1) { UpdateTurnVariables( R_T_ONE ); SetSpeedCounterClockwise(); }
        //     else if (segment == 2) { UpdateTurnVariables( R_T_TWO ); SetSpeedCounterClockwise(); }
        //     else if (segment == 3) { UpdateTurnVariables( R_T_THREE ); SetSpeedCounterClockwise(-1); controller.setDriveSpeed(0, 0);}
        //     else if (segment == 4) { UpdateTurnVariables( R_T_FOUR ); SetSpeedCounterClockwise(-1); }
        //     else if (segment == 5) { UpdateTurnVariables( R_T_FIVE ); SetSpeedCounterClockwise(-1);}
        //     else if (segment == 6) { UpdateTurnVariables( R_T_SIX ); SetSpeedCounterClockwise();}
        //     else if (segment == 7) { UpdateTurnVariables( R_T_SEVEN ); SetSpeedCounterClockwise();}
        //     else if (segment == 8) { SetSpeedStraight();}
        //     else if (segment == 9) { UpdateTurnVariables( R_T_NINE ); SetSpeedCounterClockwise();}
        //     else if (segment == 10) {UpdateTurnVariables( R_T_TEN ); SetSpeedCounterClockwise(); }
        //     else if (segment == 11) {UpdateTurnVariables( R_T_ELEVEN ); SetSpeedCounterClockwise(-1);  }
        //     else if (segment == 12) {UpdateTurnVariables( R_T_TWELVE ); SetSpeedCounterClockwise(-1);  }
        //     else if (segment == 13) {controller.setDriveSpeed(0, 0);}
        // } else {
        //     SmartDashboard.putBoolean("SOMETHING WENT WRONG", true);
        // }
        
        // checkSegmentIncrement();
    }

    private void SetSpeedClockwise() { controller.setDriveSpeed(OUTER_TURN_DRIVE_SPEED, INNER_TURN_DRIVE_SPEED); }
    private void SetSpeedClockwise(int backwards) { controller.setDriveSpeed(backwards * INNER_TURN_DRIVE_SPEED, backwards * OUTER_TURN_DRIVE_SPEED); }

    private void SetSpeedStraight() { controller.setDriveSpeed(MAX_DRIVE_SPEED, MAX_DRIVE_SPEED); }
    private void SetSpeedStraight(int backwards) { controller.setDriveSpeed(backwards * MAX_DRIVE_SPEED, backwards * MAX_DRIVE_SPEED); }

    private void SetSpeedCounterClockwise() { controller.setDriveSpeed(INNER_TURN_DRIVE_SPEED, OUTER_TURN_DRIVE_SPEED); }
    private void SetSpeedCounterClockwise(int backwards) { controller.setDriveSpeed(backwards * OUTER_TURN_DRIVE_SPEED, backwards * INNER_TURN_DRIVE_SPEED); }

    //this is called every 20 milliseconds during teleop (manually controlled by human with xboxcontroller)
    public void UpdateTeleop() {

        SmartDashboard.putNumber("Current Path", path);
        SmartDashboard.putNumber("Current Segment", segment);
        SmartDashboard.putNumber("Total distance travelled (in)", getDistanceTravelled());
        SmartDashboard.putNumber("Angle Facing Real (deg)", controller.getAngleFacing());
        SmartDashboard.putNumber("Angle Facing Adjusted (rad)", getAngleFacing());
        path = (int) pathSelector.getSelected();
        if (xController.getAButtonPressed()) {
            controller.calibrate();
        }

    }

    private void checkSegmentIncrement() {
        if (path == 1) {
                 if (segment == 1 && getDistanceTravelled() >= getIntendedDistanceOne()) segment++;
            else if (segment == 2 && getDistanceTravelled() >= getIntendedDistanceTwo() && getAngleFacing() >= getIntendedAngleTwo()) segment++;
            else if (segment == 3 && getDistanceTravelled() >= getIntendedDistanceThree()) segment++;
            else if (segment == 4 && getDistanceTravelled() >= getIntendedDistanceFour() &&  getAngleFacing() <= getIntendedAngleFour()) segment++;
            else if (segment == 5 && getDistanceTravelled() >= getIntendedDistanceFive()) segment++;
            else if (segment == 6 && getDistanceTravelled() >= getIntendedDistanceSix() && getAngleFacing() <= getIntendedAngleSix()) segment++;
            else if (segment == 7 && getDistanceTravelled() >= getIntendedDistanceSeven()) segment++;
        } else if (path == 2) {
                 if (segment == 1 && getDistanceTravelled() >= getIntendedDistanceOne()) segment++;
            else if (segment == 2 && getDistanceTravelled() >= getIntendedDistanceTwo() && getAngleFacing() <= getIntendedAngleTwo()) segment++;
            else if (segment == 3 && getDistanceTravelled() >= getIntendedDistanceThree()) segment++;
            else if (segment == 4 && getDistanceTravelled() >= getIntendedDistanceFour() && getAngleFacing() <= getIntendedAngleFour()) segment++;
            else if (segment == 5 && getDistanceTravelled() >= getIntendedDistanceFive()) segment++;
            else if (segment == 6 && getDistanceTravelled() >= getIntendedDistanceSix() && getAngleFacing() >= getIntendedAngleSix()) segment++;
            else if (segment == 7 && getDistanceTravelled() >= getIntendedDistanceSeven()) segment++;
            else if (segment == 8 && getDistanceTravelled() >= getIntendedDistanceEight() && getAngleFacing() <= getIntendedAngleEight()) segment++;
            else if (segment == 9 && getDistanceTravelled() >= getIntendedDistanceNine()) segment++;
            else if (segment == 10 && getDistanceTravelled() >= getIntendedDistanceTen() && getAngleFacing() <= getIntendedAngleTen()) segment++;
            else if (segment == 11 && getDistanceTravelled() >= getIntendedDistanceEleven()) segment++; 
        } else if (path == 3) {
                 if (segment == 1 && getDistanceTravelled() >= getIntendedDistanceOne() && getAngleFacing() >= getIntendedAngleOne()) segment++;
            else if (segment == 2 && getDistanceTravelled() >= getIntendedDistanceTwo() && getAngleFacing() >= getIntendedAngleTwo()) segment++;
            else if (segment == 3 && getDistanceTravelled() <= getIntendedDistanceThree() && getAngleFacing() >= getIntendedAngleThree()) segment++;
            else if (segment == 4 && getDistanceTravelled() <= getIntendedDistanceFour() && getAngleFacing() >= getIntendedAngleFour()) segment++;
            else if (segment == 5 && getDistanceTravelled() <= getIntendedDistanceFive() && getAngleFacing() >= getIntendedAngleFour()) segment++;
            else if (segment == 6 && getDistanceTravelled() >= getIntendedDistanceSix() && getAngleFacing() >= getIntendedAngleSix()) segment++;
            else if (segment == 7 && getDistanceTravelled() >= getIntendedDistanceSeven() && getAngleFacing() >= getIntendedAngleSeven()) segment++;
            else if (segment == 8 && getDistanceTravelled() >= getIntendedDistanceEight()) segment++;
            else if (segment == 9 && getDistanceTravelled() >= getIntendedDistanceNine() && getAngleFacing() >= getIntendedAngleNine()) segment++;
            else if (segment == 10 && getDistanceTravelled() >= getIntendedDistanceTen() && getAngleFacing() >= getIntendedAngleTen()) segment++;
            else if (segment == 11 && getDistanceTravelled() <= getIntendedDistanceEleven() && getAngleFacing() >= getIntendedAngleEleven()) segment++; 
            else if (segment == 12 && getDistanceTravelled() <= getIntendedDistanceTwelve() && getAngleFacing() >= getIntendedAngleTwelve()) segment++; 
        }
    }


    private double getIntendedDistanceOne() {
        if (path == 1) return 90;
        if (path == 2) {
            double theta = Math.asin( (20.0 + CIRCLE_RADIUS) / 60.0  );
            return ( 60.0 * CIRCLE_RADIUS / ( 20.0 + CIRCLE_RADIUS ) * Math.cos( theta )) + ( ( 1200 ) / (20 + CIRCLE_RADIUS) ) / Math.cos(theta) + 16 * Math.tan(theta);

        } else {
            return getIntendedAngleOne() * ( CIRCLE_RADIUS - DISTANCE_PIVOT_TO_WHEEL);
        }
    }

    private double getIntendedAngleOne() {

        return Math.atan( ( -30 + (1800 - Math.pow(R_T_ONE, 2) ) / ( 60 - 2 * R_T_ONE  ) ) / 30 ); 

    }

    private double getIntendedAngleTwo() {
        if (path == 1) return (2.50 * Math.PI) - Math.acos(CIRCLE_RADIUS / (15.0 * Math.sqrt(13.0))) - Math.atan(60.0/90.0);
        if (path == 2)  {
            return  -1 * Math.asin( (20.0 + CIRCLE_RADIUS) / 60.0  );
        } else {
            return Math.PI / 2;
        }
    }

    private double getIntendedDistanceTwo() {
        if (path == 1) return getIntendedDistanceOne() + getIntendedAngleTwo() * (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL);
        if (path == 2)  {
            return getIntendedDistanceOne() + Math.asin( (20.0 + CIRCLE_RADIUS) / 60.0  ) * (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL);
        } else {
            return getIntendedDistanceOne() + (R_T_TWO - DISTANCE_PIVOT_TO_WHEEL) * (getIntendedAngleTwo() - getIntendedAngleOne()); 
        }
    }

    private double getIntendedDistanceThree() {
        if (path == 1) return getIntendedDistanceTwo() + (2.0 * Math.sqrt(2925 - Math.pow(CIRCLE_RADIUS, 2.0)));
        if (path == 2)  {
            return getIntendedDistanceTwo() + 120.0;
        } else {
            return getIntendedDistanceTwo() - (getIntendedAngleThree() - getIntendedAngleTwo()) * (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL);
        }
    }

    private double getIntendedAngleThree() {
        return getIntendedAngleTwo() + Math.atan( (90) / (R_T_THREE - 60) ) ;
    }

    private double getIntendedAngleFour() {
        if (path == 1) return 0.25 * Math.PI;
        if (path == 2)  {
            return getIntendedAngleTwo() - Math.asin( CIRCLE_RADIUS / 30.0  );
        } else {
            return getIntendedAngleThree() + (Math.PI - (getIntendedAngleThree() - getIntendedAngleTwo()) - Math.atan( 50 / (R_T_FIVE - 30)));
        }
    }

    private double getIntendedDistanceFour() {
        if (path == 1) return getIntendedDistanceThree() + (- Math.PI/4 + getIntendedAngleTwo()) * (CIRCLE_RADIUS - DISTANCE_PIVOT_TO_WHEEL);
        if (path == 2)  {
            return getIntendedDistanceThree() + Math.asin( CIRCLE_RADIUS / 30.0  ) * (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL);
        } else {
            return getIntendedDistanceThree() - (getIntendedAngleFour() - getIntendedAngleThree()) * (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL);
        }
    }

    private double getIntendedDistanceFive() {
        if (path == 1) return getIntendedDistanceFour() + 60.0 * Math.sqrt(2);
        if (path == 2)  {
            return getIntendedDistanceFour() + 2 * Math.sqrt(900.0 - Math.pow(CIRCLE_RADIUS, 2));
        } else {
            return getIntendedDistanceFour() - (getIntendedAngleFive() - getIntendedAngleFour()) * (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL);
        }
    }
    
    private double getIntendedAngleFive() {
        return Math.PI * 3 / 2;
    }

    private double getIntendedAngleSix() {
        if (path == 1) return -1 * Math.PI;
        if (path == 2)  {
            return getIntendedAngleFour() + 2 * Math.PI - 2 * Math.acos( CIRCLE_RADIUS / 30.0  );
        } else {
            return getIntendedAngleFive() + (getIntendedAngleFive() - getIntendedAngleFour());
        }
    }

    private double getIntendedDistanceSix() {
        if (path == 1) return getIntendedDistanceFive() + (1.25 * Math.PI * (CIRCLE_RADIUS - DISTANCE_PIVOT_TO_WHEEL));
        if (path == 2)  {
            return getIntendedDistanceFive() + (2 * Math.PI - 2 * Math.acos( CIRCLE_RADIUS / 30.0  )) * (CIRCLE_RADIUS - DISTANCE_PIVOT_TO_WHEEL);
        } else {
            return getIntendedDistanceFive() + (getIntendedAngleSix() - getIntendedAngleFive()) * (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL);
        }
    }

    private double getIntendedDistanceSeven() {
        //803 
        if (path == 1) return getIntendedDistanceSix() + 260; 
        if (path == 2)  {
            return getIntendedDistanceSix() + 2 * Math.sqrt(900.0 - Math.pow(CIRCLE_RADIUS, 2));
        } else {
            return getIntendedAngleSix() + (getIntendedAngleSeven() - getIntendedAngleSix()) * (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL);
        }
    }

    private double getIntendedAngleSeven() {
        return Math.PI * 2;
    }

    private double getIntendedAngleEight() {
        if (path == 2) {
            return getIntendedAngleSix()  - Math.asin( CIRCLE_RADIUS / 30.0  );
        } else {
            return 0.0;
        }
    }

    private double getIntendedDistanceEight() {
        if (path == 2)  {
            return getIntendedDistanceSeven() + Math.asin( CIRCLE_RADIUS / 30.0  ) * (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL);
        } else {
            return getIntendedDistanceSeven() + 30;
        }
    }

    private double getIntendedDistanceNine() {
        if (path == 2) {
            return getIntendedDistanceEight() + 120.0;
        } else {
            return getIntendedDistanceEight() + (getIntendedAngleNine() - getIntendedAngleSeven()) * (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL);
        }
    }

    private double getIntendedAngleNine() {
        return getIntendedAngleSeven() + (Math.PI/2 - Math.atan( (90) / (R_T_TEN - 30))); 
    }

    private double getIntendedAngleTen() {
        if (path == 2) {
            return getIntendedAngleEight()  - Math.asin( (20.0 + CIRCLE_RADIUS) / 60.0  );
        } else {
            return 5 * Math.PI / 2;
        }
    }

    private double getIntendedDistanceTen() {
        if (path == 2) {
            return getIntendedDistanceNine() + Math.asin( (20.0 + CIRCLE_RADIUS) / 60.0  ) * (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL);
        } else {
            return getIntendedDistanceNine() + (getIntendedAngleTen() - getIntendedAngleNine()) * (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL);
        }
    }

    private double getIntendedDistanceEleven() {
        if (path == 2) {
            return getIntendedDistanceTen() + getIntendedDistanceOne();
        } else {
            return getIntendedDistanceTen() - (getIntendedAngleEleven() - getIntendedAngleTen()) * (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL);
        }
    }

    private double getIntendedAngleEleven() {
        return getIntendedAngleTen() + (getIntendedAngleTwo() - getIntendedAngleOne()); 
    }

    private double getIntendedAngleTwelve() {
        return Math.PI * 3;
    }
    
    private double getIntendedDistanceTwelve() {
        return getIntendedDistanceEleven() - (getIntendedAngleEleven() - getIntendedAngleTen()) * (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL);
    }

    private double getDistanceTravelled() {
        return controller.getDistanceTravelled("fL") * 12.0;
    }

    private double getAngleFacing() {
        if (path == 1) {
            return -1 * Math.toRadians(controller.getAngleFacing());
        } else if (path == 2) {
            return Math.toRadians(controller.getAngleFacing());
        } else {
            
            return Math.toRadians(controller.getAngleFacing());
        }
        
    }

    private void UpdateTurnVariables(double newRadius) {
        CIRCLE_RADIUS = newRadius;
        INNER_TURN_DRIVE_SPEED = (CIRCLE_RADIUS - DISTANCE_PIVOT_TO_WHEEL) / (CIRCLE_RADIUS + DISTANCE_PIVOT_TO_WHEEL) * OUTER_TURN_DRIVE_SPEED;
    }

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
}
