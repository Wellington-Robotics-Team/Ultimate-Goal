package org.firstinspires.ftc.robotcontroller.internal;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//import android.media.MediaPlayer;
//import com.qualcomm.ftcrobotcontroller.R;

//This auto was made by Gabo Gang on Oct 15 2019

@Autonomous(name = "Rd-Wait and Park", group = "Rd")
public class Rd_Wait_And_Park extends LinearOpMode {

    enum Directions {
        Right,
        Left,
        Forward,
        Back
    }
    //declare motors
    private DcMotor FLM = null;
    private DcMotor FRM = null;
    private DcMotor BLM = null;
    private DcMotor BRM = null;

    //declare distance sensors
    private DistanceSensor RightDistanceSensor;
    private DistanceSensor LeftDistanceSensor;
    private DistanceSensor FrontDistanceSensor;

    //declare servos
    //private Servo DragArm = null;

    private BNO055IMU imu; //declare imu

    private Orientation lastAngles = new Orientation(); //sets the last angle to whatever the robot last had. This is just to avoid errors

    private double globalAngle; //the number of degrees the robot has turned

    //declare color sensor
    private ColorSensor FloorCS = null;
    private ColorSensor RightCS = null;

    final private double NormPower = 0.50; //The normal power to give to motors to drive
    final private double MinPower = 0.15; //slowest it should do

    //servo positions for drag arm
    //final private double DragArmDownPosition = 0.55;

    //final private double DragArmRestPosition = 0.87;
    //private MediaPlayer mediaPlayer = null;
    public void runOpMode() //when you press init
    {
        //Init
        FLM  = hardwareMap.get(DcMotor.class, "FLM"); //get the motors from the config
        FRM  = hardwareMap.get(DcMotor.class, "FRM");
        BLM  = hardwareMap.get(DcMotor.class, "BLM");
        BRM  = hardwareMap.get(DcMotor.class, "BRM");
        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //if we set the power to 0 we want the motors to stop
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //if we don't set it they will be in neutral and friction will slow it
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLM.setDirection(DcMotor.Direction.REVERSE); //reverse the motors
        BLM.setDirection(DcMotor.Direction.REVERSE);

        FloorCS = hardwareMap.get(ColorSensor.class, "FloorCS"); //get color sensor
        FloorCS.enableLed(true);
        RightCS = hardwareMap.get(ColorSensor.class, "RightCS");
        RightCS.enableLed(true);
        //get drag arm
        //DragArm = hardwareMap.servo.get("drag_arm");
        //DragArm.setDirection(Servo.Direction.FORWARD);
        //DragArm.setPosition(DragArmRestPosition);

        //get distance sensors
        RightDistanceSensor = hardwareMap.get(DistanceSensor.class, "RDS");
        LeftDistanceSensor = hardwareMap.get(DistanceSensor.class, "LDS");
        FrontDistanceSensor = hardwareMap.get(DistanceSensor.class, "FDS");

        imu = hardwareMap.get(BNO055IMU.class, "imu"); //gets the imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //makes parameters for imu
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters); //initalizes the imu
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart(); //waits for the start button

        //mediaPlayer.stop();
        //play

        //Against wall
        ResetAngle();
        sleep(25 * 1000);
        //drive to tape
        DriveToTape();
        StopRobot();

        stop();
    }
    /*
    private void GetAwayFromPlane() {
        while (RightDistanceSensor.getDistance(DistanceUnit.CM) < 20 && !isStopRequested()) {
            Drive(-MinPower, 0, 0);
        }
    }

    private void DriveForwardToWall() {
        final double DistanceOffset = 8; //calibrate
        telemetry.addData("Distance", FrontDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
        sleep(5000);
        while (FrontDistanceSensor.getDistance(DistanceUnit.CM) > 20 + DistanceOffset && !isStopRequested()) { //while more than 14 centimeters
            telemetry.addData("Distance", FrontDistanceSensor.getDistance(DistanceUnit.CM));
            Drive(NormPower, 0, 0); //straff
            telemetry.update();
        }
        StopRobot();
    }
 */
    /**
     * Runs until it gets close to wall
     * slowly gets slower
     */
    private void RushB(double endDistance, Directions Direction, double MaxPower) //moves with distance sensor. Slowly getting slower and slower
    {
        final double DistanceOffset = 2;
        endDistance = endDistance + DistanceOffset;

        DistanceSensor TheDistanceSensor = null;

        switch (Direction) {
            case Right:
                TheDistanceSensor = RightDistanceSensor;
                break;
            case Left:
                TheDistanceSensor = LeftDistanceSensor;
                break;
            case Forward:
                TheDistanceSensor = FrontDistanceSensor;
                break;
        }

        if (TheDistanceSensor == null) {
            stop();
            return;
        }

        double startDistance = TheDistanceSensor.getDistance(DistanceUnit.CM); //gets the distance to the wall
        double distanceTraveled = 0; //sets the distance traveled to 0
        double totalNeededToTravel = startDistance - endDistance; //gets the total distance the robot needs to move
        double lastDistance = startDistance; //last distance is first distance
        double thingy = totalNeededToTravel * totalNeededToTravel * totalNeededToTravel;
        double slope = -MaxPower / thingy; //gets the slope of the graph that is needed to make y = 0 when totalNeeded to travel is x

        while (TheDistanceSensor.getDistance(DistanceUnit.CM) > endDistance && !isStopRequested()) //while op mode is running and the distance to the wall is greater than the end distance
        {
            telemetry.addData("Status: ", "Rushing A");
            double currentDistance = TheDistanceSensor.getDistance(DistanceUnit.CM); //gets the current distance to the wall
            telemetry.addData("Current distance to wall: ", currentDistance);

            double deltaDistance = lastDistance - currentDistance; //change in distance is the last distance - currentDistance

            distanceTraveled += deltaDistance; //adds the change in distance to distance traveled
            lastDistance = currentDistance; //the last distance is set to the current distance
            double distanceCubed = distanceTraveled * distanceTraveled * distanceTraveled;
            double power = (   slope * distanceCubed) + MaxPower; // the power is the x value in that position
            if (power > MaxPower) power = MaxPower;
            else if (power < MinPower && power > 0) power = MinPower; //if the power is less than the min power level just set the power to the minpower level
            else if (power <= 0) power = 0; //if its 0 then set it to 0 of course
            telemetry.addData("Power", power);
            switch (Direction) {
                case Right:
                    telemetry.addData("Direction", "Right");
                    Drive(0, -power, 0);
                    break;
                case Left:
                    telemetry.addData("Direction", "Left");
                    Drive(0, power, 0);
                    break;
                case Forward:
                    telemetry.addData("Direction", "Forward");
                    Drive(power, 0, 0); //moves the robot forward with whatever the power is
                    break;
            }
            telemetry.update();
        }
    }
/*
    private void DragSkyStone() { //drag it
        //DragArm.setPosition(DragArmDownPosition); //set to down
        final double DistanceOffset = 4; //distance offset to adjust. calibrate
        while (!isStopRequested() && LeftDistanceSensor.getDistance(DistanceUnit.CM) > 34 + DistanceOffset) { //while not stopped and the distance is more than 50 centimeters
            telemetry.addData("Distance", LeftDistanceSensor.getDistance(DistanceUnit.CM));
            Drive(0, NormPower, 0); //straff
            telemetry.update();
        }
        StopRobot(); //stop moving
    }
*/
    private void ScanForSkyStone() { //scan
        while (RightCS.alpha() > 70 && !isStopRequested()) { //while FloorCS isn't detecting black
            telemetry.addData("Alpha", RightCS.alpha()); //log
            Drive(-NormPower, 0, 0); //backup
            telemetry.update();
        }
    }
/*
    private void DriveToBrick() { //go to bricks
        final double DistanceOffset = 8; //calibrate
        while (RightDistanceSensor.getDistance(DistanceUnit.CM) > 15 + DistanceOffset && !isStopRequested()) { //while more than 14 centimeters
            telemetry.addData("Distance", RightDistanceSensor.getDistance(DistanceUnit.CM));
            Drive(0, -NormPower, 0); //straff
            telemetry.update();
        }
        StopRobot();
    }
*/
    private void DriveToTape() {
        final double SCALE_FACTOR = 255; //For color sensor readings (make differences more obvious

        float hsvValues[] = {0F, 0F, 0F}; //store values

        final int RedThreshold = 50; //Anything below this number will be considered red
        final int BlueThreshold = 170; //Anything above this number will be considered blue

        Color.RGBToHSV((int) (FloorCS.red() * SCALE_FACTOR), //get readings before starting
                (int) (FloorCS.green() * SCALE_FACTOR),
                (int) (FloorCS.blue() * SCALE_FACTOR),
                hsvValues);

        while (!isStopRequested() && hsvValues[0] > RedThreshold && hsvValues[0] < BlueThreshold) { //run until the driver presses stop or its on red or blue tape
            Color.RGBToHSV((int) (FloorCS.red() * SCALE_FACTOR), //check readings again
                    (int) (FloorCS.green() * SCALE_FACTOR),
                    (int) (FloorCS.blue() * SCALE_FACTOR),
                    hsvValues);

                Drive(NormPower, 0, 0); //drive forward at the normal speed

            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
        }
        //Driver pressed stop or we are on tape
    }

    public void Drive(double forward, double sideways, double rotation) { //make a function to drive
        double correction = 0; //default correction
        if (rotation == 0) {
            if (forward != 0) {
                correction = CheckDirection(Math.abs(forward)); //if there isn't any rotation then use correction
            } else if (sideways != 0) {
                correction = CheckDirection(Math.abs(sideways)); //if there isn't any rotation then use correction
            }
        }

        telemetry.addData("Correction", correction);
        FRM.setPower((forward + sideways + rotation) + correction);
        FLM.setPower((forward - sideways - rotation) - correction);
        BRM.setPower((forward - sideways + rotation) + correction);
        BLM.setPower((forward + sideways - rotation) - correction);
    }

    private void StopRobot() {  //stop the robot
        FRM.setPower(0); //set all motors to 0 power
        FLM.setPower(0);
        BRM.setPower(0);
        BLM.setPower(0);
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double GetAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //gets the angle

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle; //deltaAngle is the current angle minus the last angle it got

        if (deltaAngle < -180) //switches it to use 0 to 360 instead of -180 to 180
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle; //adds the deltaAngle to the globalAngle

        lastAngles = angles; //lastAngle is the anlges

        return globalAngle; //returns the amount turned
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void ResetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //sets lastAngles to current angles

        globalAngle = 0; //global angle is set to 0
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double CheckDirection(double Speed) {
        double correction;

        double angle = GetAngle();  //get the total amount the angle has changed since last reset

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        double CorrectionSensitivity = 0.10 * Speed;

        correction = correction * CorrectionSensitivity;

        return correction;
    }
}