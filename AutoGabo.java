package org.firstinspires.ftc.robotcontroller.internal;

import android.graphics.Color;
import android.media.MediaPlayer;

import com.qualcomm.ftcrobotcontroller.R;
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

//This auto was made by Gabo Gang on Oct 15 2019

@Autonomous(name = "AutoGabo", group = "Auto")
public class AutoGabo extends LinearOpMode {

    //declare motors
    private DcMotor FLM = null;
    private DcMotor FRM = null;
    private DcMotor BLM = null;
    private DcMotor BRM = null;

    //declare distance sensors
    private DistanceSensor RightDistanceSensor;
    private DistanceSensor LeftDistanceSensor;

    //declare servos
    //private Servo DragArm = null;

    private BNO055IMU imu; //declare imu

    private Orientation lastAngles = new Orientation(); //sets the last angle to whatever the robot last had. This is just to avoid errors

    private double globalAngle; //the number of degrees the robot has turned

    //declare color sensor
    private ColorSensor FloorCS = null;
    private ColorSensor RightCS = null;

    private final double SCALE_FACTOR = 255; //For color sensor readings (make differences more obvious

    private float hsvValues[] = {0F, 0F, 0F}; //store values

    private final int RedThreshold = 50; //Anything below this number will be considered red
    private final int BlueThreshold = 180; //Anything above this number will be considered blue

    final private double NormPower = 0.5; //The normal power to give to motors to drive
    final private double MinPower = 0.15; //slowest it should do

    //servo positions for drag arm
    final private double DragArmRestPosition = 0.87;
    final private double DragArmDownPosition = 0.55;

    double CorrectionSensitivity = .05; //how sensitive the correction is

    MediaPlayer mediaPlayer = null;
    public void runOpMode() //when you press init
    {
        mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.march); //create media player

        mediaPlayer.start(); //play

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

        RightCS = hardwareMap.get(ColorSensor.class, "RightCS");

        //get drag arm
        //DragArm = hardwareMap.servo.get("drag_arm");
        //DragArm.setDirection(Servo.Direction.FORWARD);
        //DragArm.setPosition(DragArmRestPosition);

        //get distance sensors
        RightDistanceSensor = hardwareMap.get(DistanceSensor.class, "RDS");
        LeftDistanceSensor = hardwareMap.get(DistanceSensor.class, "LDS");

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

        mediaPlayer.stop();
        //play


        ResetAngle();
        DriveToBrick();
        ScanForSkyStone();
        DragSkyStone();

        Drive(NormPower, 0, 0);
        sleep(1000);

        DriveToTape();
    }


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

    private void ScanForSkyStone() { //scan
        CorrectionSensitivity = 0.01;
        while (RightCS.alpha() > 90 && !isStopRequested()) { //while FloorCS isn't detecting black
            telemetry.addData("Alpha", RightCS.alpha()); //log
            Drive(-MinPower, 0, 0); //backup
            telemetry.update();
        }
        StopRobot(); //stop
    }

    private void DriveToBrick() { //go to bricks
        final double DistanceOffset = 5; //calibrate
        CorrectionSensitivity = 0.05;
        while (RightDistanceSensor.getDistance(DistanceUnit.CM) > 15 + DistanceOffset && !isStopRequested()) { //while more than 14 centimeters
            telemetry.addData("Distance", RightDistanceSensor.getDistance(DistanceUnit.CM));
            Drive(0, -NormPower, 0); //straff
            telemetry.update();
        }
        StopRobot();
    }

    private void DriveToTape() {
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
        StopRobot(); //stop the robot
    }

    public void Drive(double forward, double sideways, double rotation) { //make a function to drive
        double correction = 0; //default 0
        if (rotation == 0) correction = CheckDirection(); //if there isn't any rotation then use correction

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
    private double CheckDirection()
    {
        double correction;

        double angle = GetAngle();  //get the total amount the angle has changed since last reset

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * CorrectionSensitivity;

        return correction;
    }
    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void Rotate(int degrees, double power) {
        telemetry.addData("Rotating", true); //informs
        telemetry.update();

        ResetAngle(); //sets starting angle and resets the amount turned to 0

        // GetAngle() returns + when rotating counter clockwise (left) and - when rotating clockwise (right).
        double DegreesCubed = degrees * degrees * degrees;
        double slope = -power / DegreesCubed; //gets the slope of the graph that is needed to make y = 0 when totalNeeded to travel is x

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (!isStopRequested() && GetAngle() == 0) {
                double currentAngle = GetAngle();
                double currentAngleCubed = currentAngle * currentAngle * currentAngle;
                double newPower = slope * currentAngleCubed + power; // the power is the x value in that position
                if (newPower < MinPower) newPower = MinPower;
                if (newPower <= 0) newPower = 0;
                telemetry.addData("Power: ", newPower);
                telemetry.update();
                Drive(0, 0, newPower);
            }

            while (!isStopRequested() && GetAngle() > degrees) {
                double currentAngle = GetAngle();
                double CurrentAngledCubed = currentAngle * currentAngle * currentAngle;
                double newPower = slope * CurrentAngledCubed + power; // the power is the x value in that position
                if (newPower < MinPower) newPower = MinPower;
                if (newPower <= 0) newPower = 0;
                telemetry.addData("Power: ", newPower);
                telemetry.update();
                Drive(0, 0, newPower);
            } //once it starts turning slightly more than it should.
        } else {
            // left turn.
            while (!isStopRequested() && GetAngle() < degrees) {
                double currentAngle = GetAngle();
                double CurrentAngleCubed = currentAngle * currentAngle * currentAngle;
                double newPower = slope * CurrentAngleCubed + power; // the power is the x value in that position
                if (newPower < MinPower) newPower = MinPower;
                if (newPower <= 0) newPower = 0;
                telemetry.addData("Power: ", -newPower);
                telemetry.update();
                Drive(0,0 ,-newPower);
            }
        }


        // turn the motors off.
        StopRobot();
    }
}
