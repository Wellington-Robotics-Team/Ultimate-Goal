package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public abstract class Functions {

    private DcMotor FLM;
    private DcMotor FRM;
    private DcMotor BLM;
    private DcMotor BRM;
    private DcMotorEx FlyMotor;

    private DistanceSensor TheDS;
    private DistanceSensor TheSideDS;
    private DistanceSensor FDS;
    private DistanceSensor BDS;
    private DistanceSensor LDS;
   /// private DistanceSensor RDS;

    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation(); //sets the last angle to whatever the robot last had. This is just to avoid errors

    private double globalAngle; //the number of degrees the robot has turned

    public abstract boolean CanMove();
    public abstract void AddToTelemetry(String Tag, String Message);
    public abstract void UpdateTelemetry();

    public double NormPower = 0.4;


    public void Init(HardwareMap hardwareMap){

        FLM  = hardwareMap.get(DcMotor.class, "FLM"); //get the motors from the config
        FRM  = hardwareMap.get(DcMotor.class, "FRM");
        BLM  = hardwareMap.get(DcMotor.class, "BLM");
        BRM  = hardwareMap.get(DcMotor.class, "BRM");
        FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //FlyMotor = hardwareMap.get(DcMotorEx.class, "Fly");

        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //if we set the power to 0 we want the motors to stop
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //if we don't set it they will be in neutral and friction will slow it
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRM.setDirection(DcMotor.Direction.REVERSE); //reverse the motors
        BLM.setDirection(DcMotor.Direction.REVERSE);


       // RDS = hardwareMap.get(DistanceSensor .class, "RDS");
        LDS = hardwareMap.get(DistanceSensor.class, "LDS");
        FDS = hardwareMap.get(DistanceSensor.class, "FDS");
        BDS = hardwareMap.get(DistanceSensor.class, "BDS");
       /// Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)RDS;


        imu = hardwareMap.get(BNO055IMU.class, "imu"); //gets the imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //makes parameters for imu
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters); //initalizes the imu
        while (CanMove() && !imu.isGyroCalibrated()) {
            if (!CanMove()) return;
        }

        ResetAngle();
    }

    //the most basic way u can use encoders is like this : while(currentTicks < desiredTicks){move robot}
   // and then you can probably make a method that takes in inches or cm and converts it into ticks to tell the robot how far to move

    public void EncoderPID(Move Move){
        double desiredTicks = InchesToTicks(Move.ForwardDistance());
        double motorPower, P = 1;
        double error = 0;
        while (CanMove()) {
        error = desiredTicks - FLM.getCurrentPosition();
        error /= 100;

            Drive(error,0,0);
        }
        //motor.getCurrentPosition() returns current ticks that encoder on the motor has tracked
    }
    public void EncoderDrive(Move Move, double power){
        double desiredTicks = InchesToTicks(Move.ForwardDistance());
        double currentPosition = Math.abs(FLM.getCurrentPosition());
        double previousPosition = Math.abs(FLM.getCurrentPosition());
        //double desiredTicks = InchesToTicks(Move.ForwardDistance());
        if (currentPosition <= (desiredTicks + previousPosition) && CanMove()){
        Drive(power,0,0);
        AddToTelemetry("Position:", String.valueOf(currentPosition));
        currentPosition = Math.abs(FLM.getCurrentPosition());
        previousPosition = currentPosition;
        UpdateTelemetry();
        }
        else{
        Drive(0,0,0);
        }
    }
    protected double InchesToTicks(double inches){
        double tick = 31.0143327744;
        double TicksNeeded = inches * tick;
        return TicksNeeded;
    }
   // public void FlyPower(double power){
    //    double error = 0, P = 1;
      //  error = power - FlyMotor.getVelocity();
        //FlyMotor.setVelocity(P*error);
   // }


    public void PID(Move finalDist) {
        double errorForward = 0, previous_errorForward = 1, derivativeForward = 0, powerForward = 0;
        double P = 2, I = 0, D = 0;

        whichDistanceSensor(finalDist.MovingToward());
       /// TheSideDS = RDS;


        while (CanMove()) {


            errorForward = CurrentDist(TheDS) - finalDist.ForwardDistance(); // Error = Actual - Target
            derivativeForward = (errorForward - previous_errorForward) / .02;
            powerForward = P * errorForward + D * derivativeForward;
            finalDist.Power(powerForward);
            previous_errorForward = errorForward;


            AddToTelemetry("Error:", String.valueOf(errorForward));
            UpdateTelemetry();

            Drive(powerForward,0,0);
        }
    }
    public void whichDistanceSensor(Move.Direction direction){

        switch (direction){
            //case Right:
             //   TheDS = RDS;
            case Left:
                TheDS = LDS;
            case Backwards:
                TheDS = BDS;
            case Forward:
                TheDS = FDS;

        }
    }
    public double CurrentDist(DistanceSensor TheDS){
        return TheDS.getDistance(DistanceUnit.INCH);
    }

    private void ResetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //sets lastAngles to current angles

        globalAngle = 0; //global angle is set to 0
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

        FRM.setPower(-(forward + sideways + rotation) + correction);
        FLM.setPower(-(forward - sideways - rotation) - correction);
        BRM.setPower((forward - sideways + rotation) + correction);
        BLM.setPower((forward + sideways - rotation) - correction);
    }
    public void DriveTicks(double forward) { //make a function to drive
        double correction = 0; //default correction
        correction = CheckDirection(Math.abs(forward)); //if there isn't any rotation then use correction
        FRM.setPower(-forward + correction);
        FLM.setPower(-forward - correction);
        BRM.setPower(forward + correction);
        BLM.setPower(forward - correction);
    }
    private double GetAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES); //gets the angle

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle; //deltaAngle is the current angle minus the last angle it got

        if (deltaAngle < -180) //switches it to use 0 to 360 instead of -180 to 180
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle; //adds the deltaAngle to the globalAngle

        lastAngles = angles; //lastAngle is the angles

        return globalAngle; //returns the amount turned
    }
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
    public Move Path(int pathChoice, Move Move) {
        switch (pathChoice) {
            case 0: //zone A
                Move = new Move(84, 12, NormPower, org.firstinspires.ftc.teamcode.Move.Direction.Forward);
            case 1: // Zone B
                Move = new Move(108, 36, NormPower, org.firstinspires.ftc.teamcode.Move.Direction.Forward);
            case 2: //Zone C
                Move = new Move(132, 12, NormPower, org.firstinspires.ftc.teamcode.Move.Direction.Forward);


        }
        return null;
    }



}

