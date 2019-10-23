package org.firstinspires.ftc.robotcontroller.internal;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//This auto was made by Gabo Gang on Oct 15 2019

@Autonomous(name = "AutoGabo", group = "Auto")
public class AutoGabo extends LinearOpMode {

    //declare motors
    private DcMotor FLM = null;
    private DcMotor FRM = null;
    private DcMotor BLM = null;
    private DcMotor BRM = null;

    private Servo Arm = null;

    private BNO055IMU imu; //declare imu

    private Orientation lastAngles = new Orientation(); //sets the last angle to whatever the robot last had. This is just to avoid errors

    private double globalAngle; //the number of degrees the robot has turned

    //declare color sensor
    private ColorSensor CS = null;

    private final double SCALE_FACTOR = 255; //For color sensor readings (make differences more obvious

    private float hsvValues[] = {0F, 0F, 0F}; //store values

    private final int RedThreshold = 50; //Anything below this number will be considered red
    private final int BlueThreshold = 180; //Anything above this number will be considered blue

    final private double NormPower = 0.5; //The normal power to give to motors to drive
    final private double MinPower = 0.15;

    final private double ArmRestPosition = 0;
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

        CS = hardwareMap.get(ColorSensor.class, "CS"); //get color sensor
        CS.enableLed(false); //turn off led

        Arm = hardwareMap.servo.get("Arm");
        Arm.setDirection(Servo.Direction.FORWARD);
        Arm.setPosition(ArmRestPosition);

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

        //play
        DriveToTape(); //run function (easier to read)

    }


    private void DriveToTape() {
        ResetAngle();
        CS.enableLed(true); //turn on LED
        Color.RGBToHSV((int) (CS.red() * SCALE_FACTOR), //get readings before starting
                (int) (CS.green() * SCALE_FACTOR),
                (int) (CS.blue() * SCALE_FACTOR),
                hsvValues);

        while (!isStopRequested() && hsvValues[0] > RedThreshold && hsvValues[0] < BlueThreshold) { //run until the driver presses stop or its on red or blue tape
            Color.RGBToHSV((int) (CS.red() * SCALE_FACTOR), //check readings again
                    (int) (CS.green() * SCALE_FACTOR),
                    (int) (CS.blue() * SCALE_FACTOR),
                    hsvValues);
            Drive(NormPower, 0, 0); //drive forward at the normal speed
        }
        //Driver pressed stop or we are on tape
        Stop(); //stop the robot
        CS.enableLed(false); //turn off LED
    }

    public void Drive(double forward, double sideways, double rotation) { //make a function to drive
        double correction = 0; //default 0
        if (rotation == 0) correction = CheckDirection(); //if there isn't any rotation then use correction

        telemetry.addData("Correction", correction);
        telemetry.update();
        FRM.setPower((forward + sideways + rotation) + correction);
        FLM.setPower((forward - sideways - rotation) - correction);
        BRM.setPower((forward - sideways + rotation) + correction);
        BLM.setPower((forward + sideways - rotation) - correction);
    }

    private void Stop() {  //stop the robot
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
        double gain = .10; //how sensitive the correction is

        double angle = GetAngle();  //get the total amount the angle has changed since last reset

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

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
        Stop();
    }
}
