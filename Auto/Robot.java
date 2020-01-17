package org.firstinspires.ftc.robotcontroller.internal;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class Robot {
    final private double MaxPwr = 0.75;
    final private double NormPower = 0.40; //The normal power to give to motors to drive
    final private double MinPower = 0.20; //slowest it should do

    Movement MoveToBricks = new Movement(7, 8, NormPower);
    Movement MoveToWallWithBlock = new Movement(50, 0, NormPower);
    Movement MoveToOtherSideOfField = new Movement(110,5, NormPower);
    Movement MoveToPlate = new Movement(10,0,NormPower);
    Movement MoveToWallWithPlate = new Movement(10,5,MaxPwr);
    Movement MoveToFarWall = new Movement(31, 9, NormPower);
    Movement MoveToFarPark = new Movement(70,0, NormPower);
    Movement MoveToMiddleOfPlate = new Movement(38, 5, NormPower);

    Movement.Directions DirectionToTape;
    Movement.Directions DirectionToScan;
    //declare motors
    private DcMotor FLM = null;
    private DcMotor FRM = null;
    private DcMotor BLM = null;
    private DcMotor BRM = null;

    //declare distance sensors
    private DistanceSensor RightDistanceSensor;
    private DistanceSensor LeftDistanceSensor;
    private DistanceSensor FrontDistanceSensor;
    private DistanceSensor BackDistanceSensor;

    //declare servos
    private Servo DragArm = null;

    private BNO055IMU imu; //declare imu

    private Orientation lastAngles = new Orientation(); //sets the last angle to whatever the robot last had. This is just to avoid errors

    private double globalAngle; //the number of degrees the robot has turned

    //declare color sensor
    private ColorSensor FloorCS = null;
    private ColorSensor RightCS = null;

    //servo positions for drag arm
    final private double DragArmDownPosition = 0.7;
    final private double DragArmRestPosition = 0.15;

    public abstract boolean AllowedToMove();
    public abstract void AddToTelemetry(String Tag, String Message);
    public abstract void UpdateTelemetry();
    public abstract void Sleep(int Time);

    public void Init(HardwareMap hardwareMap) {
        FLM  = hardwareMap.get(DcMotor.class, "FLM"); //get the motors from the config
        FRM  = hardwareMap.get(DcMotor.class, "FRM");
        BLM  = hardwareMap.get(DcMotor.class, "BLM");
        BRM  = hardwareMap.get(DcMotor.class, "BRM");
        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //if we set the power to 0 we want the motors to stop
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //if we don't set it they will be in neutral and friction will slow it
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRM.setDirection(DcMotor.Direction.REVERSE); //reverse the motors
        BLM.setDirection(DcMotor.Direction.REVERSE);

        FloorCS = hardwareMap.get(ColorSensor.class, "FloorCS"); //get color sensor
        FloorCS.enableLed(true);
        RightCS = hardwareMap.get(ColorSensor.class, "RightCS");
        RightCS.enableLed(true);
        //get drag arm
        DragArm = hardwareMap.servo.get("drag_arm");
        DragArm.setDirection(Servo.Direction.REVERSE);
        DragArm.setPosition(DragArmRestPosition);

        //get distance sensors
        RightDistanceSensor = hardwareMap.get(DistanceSensor.class, "RDS");
        LeftDistanceSensor = hardwareMap.get(DistanceSensor.class, "LDS");
        FrontDistanceSensor = hardwareMap.get(DistanceSensor.class, "FDS");
        BackDistanceSensor = hardwareMap.get(DistanceSensor.class, "BDS");

        imu = hardwareMap.get(BNO055IMU.class, "imu"); //gets the imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //makes parameters for imu
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters); //initalizes the imu
        while (AllowedToMove() && !imu.isGyroCalibrated()) {
            if (!AllowedToMove()) return;
        }

        ResetAngle();
    }
    public Robot(boolean BlueSide) {
        if (BlueSide) {
            MoveToBricks.SetDirection(Movement.Directions.Right);
            MoveToWallWithBlock.SetDirection(Movement.Directions.Left);
            MoveToOtherSideOfField.SetDirection(Movement.Directions.Forward);
            MoveToPlate.SetDirection(Movement.Directions.Right);
            MoveToWallWithPlate.SetDirection(Movement.Directions.Left);
            MoveToFarWall.SetDirection(Movement.Directions.Forward);
            DirectionToTape = Movement.Directions.Backwards;
            DirectionToScan = Movement.Directions.Backwards;

            MoveToFarPark.SetDirection(Movement.Directions.Left);
            MoveToMiddleOfPlate.SetDirection(Movement.Directions.Forward);
        } else {
            MoveToBricks.SetDirection(Movement.Directions.Right);
            DirectionToScan = Movement.Directions.Forward;
            MoveToWallWithBlock.SetDirection(Movement.Directions.Left);
            MoveToOtherSideOfField.SetDirection(Movement.Directions.Backwards);
            MoveToFarWall.SetDirection(Movement.Directions.Backwards);
            MoveToPlate.SetDirection(Movement.Directions.Right);
            MoveToWallWithPlate.SetDirection(Movement.Directions.Left);
            DirectionToTape = Movement.Directions.Forward;

            MoveToFarPark.SetDirection(Movement.Directions.Right);
            MoveToMiddleOfPlate.SetDirection(Movement.Directions.Backwards);


        }
    }

    void DropArm() {
        DragArm.setPosition(DragArmDownPosition);
        Sleep(500);
    }

    void RaiseArm() {
        DragArm.setPosition(DragArmRestPosition);
        Sleep(500);
    }

    /**
     * Runs until it gets close to wall
     * slowly gets slower
     */
    private boolean ShouldMove(boolean MovingTowards, double CurrentDistance, double TargetDistance) {
        if (MovingTowards) {
            return CurrentDistance > TargetDistance;
        } else {
            return CurrentDistance < TargetDistance;
        }
    }

    public void RushB(Movement Move) //moves with distance sensor. Slowly getting slower and slower
    {
        DistanceSensor TheDistanceSensor = null;

        switch (Move.getDirection()) {
            case Right:
                TheDistanceSensor = RightDistanceSensor;
                break;
            case Left:
                TheDistanceSensor = LeftDistanceSensor;
                break;
            case Forward:
                TheDistanceSensor = FrontDistanceSensor;
                break;
            case Backwards:
                TheDistanceSensor = BackDistanceSensor;
                break;
        }

        if (TheDistanceSensor == null) {
            return;
        }
        boolean MovingTowards = false;

        if (Move.GetTotalDistance() > TheDistanceSensor.getDistance(DistanceUnit.CM)) {
            Move.setPowerSign(-1);
            MovingTowards = false;
        }
        else {
            Move.setPowerSign(1);
            MovingTowards = true;
        }

        Move.setStartDistance(TheDistanceSensor.getDistance(DistanceUnit.CM));

        while (AllowedToMove() && ShouldMove(MovingTowards, TheDistanceSensor.getDistance(DistanceUnit.CM), Move.GetTotalDistance())) //while op mode is running and the distance to the wall is greater than the end distance
        {
            if (!AllowedToMove()) {
                StopRobot();
                return;
            }
            Move.UpdateDistanceTraveled(TheDistanceSensor.getDistance(DistanceUnit.CM)); //gets the current distance to the wall
            AddToTelemetry("Distance", Double.toString(TheDistanceSensor.getDistance(DistanceUnit.CM)));
            if (MovingTowards) AddToTelemetry("Direction", "True");
            else AddToTelemetry("Direction", "False");
            double power = Move.CalculatePower();

            switch (Move.getDirection()) {
                case Right:
                    Drive(0, -power, 0);
                    AddToTelemetry("Right", Double.toString(-power));
                    break;
                case Left:
                    Drive(0, power, 0);
                    AddToTelemetry("Left", Double.toString(power));
                    break;
                case Forward:
                    Drive(power, 0, 0); //moves the robot forward with whatever the power is
                    AddToTelemetry("Forward", Double.toString(power));
                    break;
                case Backwards:
                    Drive(-power,0,0);
                    AddToTelemetry("Backwards", Double.toString(-power));
            }

            UpdateTelemetry();
        }


    }

    void ScanForSkyStone(Movement.Directions Direction) { //scan
        while (AllowedToMove() && RightCS.alpha() > 710) { //while RightCS isn't detecting black 3
            if (!AllowedToMove()) {
                StopRobot();
                return;
            }
            AddToTelemetry("Alpha", Integer.toString(RightCS.alpha()));
            switch (Direction) {
                case Backwards:
                    Drive(-MinPower, 0, 0); //backup
                    break;
                case Forward:
                    Drive(MinPower, 0, 0); //backup
                    break;

            }
            UpdateTelemetry();
        }
    }

    public void DriveToTape(Movement.Directions Direction) {
        final double SCALE_FACTOR = 255; //For color sensor readings (make differences more obvious

        float hsvValues[] = {0F, 0F, 0F}; //store values

        final int RedThreshold = 50; //Anything below this number will be considered red
        final int BlueThreshold = 160; //Anything above this number will be considered blue

        Color.RGBToHSV((int) (FloorCS.red() * SCALE_FACTOR), //get readings before starting
                (int) (FloorCS.green() * SCALE_FACTOR),
                (int) (FloorCS.blue() * SCALE_FACTOR),
                hsvValues);

        while (AllowedToMove() && hsvValues[0] > RedThreshold && hsvValues[0] < BlueThreshold) { //run until the driver presses stop or its on red or blue tape
            if (!AllowedToMove()) return;
            Color.RGBToHSV((int) (FloorCS.red() * SCALE_FACTOR), //check readings again
                    (int) (FloorCS.green() * SCALE_FACTOR),
                    (int) (FloorCS.blue() * SCALE_FACTOR),
                    hsvValues);
            switch (Direction) {
                case Backwards:
                    Drive(-NormPower, 0, 0); //drive forward at the normal speed
                    break;
                case Forward:
                    Drive(0.3, 0, 0); //drive forward at the normal speed
                    break;
            }
        }
        //Driver pressed stop or we are on tape
    }

    void Drive(double forward, double sideways, double rotation) { //make a function to drive
        double correction = 0; //default correction
        if (rotation == 0) {
            if (forward != 0) {
                correction = CheckDirection(Math.abs(forward)); //if there isn't any rotation then use correction
            } else if (sideways != 0) {
                correction = CheckDirection(Math.abs(sideways)); //if there isn't any rotation then use correction
            }
        }

        FRM.setPower((forward + sideways + rotation) + correction);
        FLM.setPower((forward - sideways - rotation) - correction);
        BRM.setPower((forward - sideways + rotation) + correction);
        BLM.setPower((forward + sideways - rotation) - correction);
    }

    public void StopRobot() {  //stop the robot
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
