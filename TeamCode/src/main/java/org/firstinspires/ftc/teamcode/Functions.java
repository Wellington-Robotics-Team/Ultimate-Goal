package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import android.graphics.Color;

import java.io.OutputStream;


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

    private RevColorSensorV3 CS;


    private double globalAngle; //the number of degrees the robot has turned

    public abstract boolean CanMove();
    public abstract void AddToTelemetry(String Tag, String Message);
    public abstract void UpdateTelemetry();

    public double NormPower = 0.4;
    public Move MoveToZone;


    public void Init(HardwareMap hardwareMap){

        FLM  = hardwareMap.get(DcMotor.class, "FLM"); //get the motors from the config
        FRM  = hardwareMap.get(DcMotor.class, "FRM");
        BLM  = hardwareMap.get(DcMotor.class, "BLM");
        BRM  = hardwareMap.get(DcMotor.class, "BRM");

        FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //FlyMotor = hardwareMap.get(DcMotorEx.class, "Fly");

        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //if we set the power to 0 we want the motors to stop
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //if we don't set it they will be in neutral and friction will slow it
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLM.setDirection(DcMotor.Direction.REVERSE); //reverse the motors
        BLM.setDirection(DcMotor.Direction.REVERSE);


       /*// RDS = hardwareMap.get(DistanceSensor .class, "RDS");
        LDS = hardwareMap.get(DistanceSensor.class, "LDS");
        FDS = hardwareMap.get(DistanceSensor.class, "FDS");
        BDS = hardwareMap.get(DistanceSensor.class, "BDS");
       /// Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)RDS;
*/
        CS = hardwareMap.get(RevColorSensorV3.class,"CS");
        CS.enableLed(true);

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
    double MaxPower = .7;
    double MinPower = -MaxPower;
    public void EncoderPID(double distance, double correction){
        double desiredTicks = InchesToTicks(distance + correction);
        double motorPos = FLM.getCurrentPosition();
        double currentPosition = motorPos;
        double previousPosition = motorPos;
        double motorPower, kP = 5;
        double error = desiredTicks - motorPos;
        double MaxError = error;
        while (CanMove() && Math.abs(error) > 10) {
            motorPos = Math.abs(FLM.getCurrentPosition());
            previousPosition = currentPosition;
            currentPosition = motorPos;

            error = desiredTicks - motorPos;
            motorPower = (kP*error)/MaxError;
            if (motorPower>MaxPower) motorPower = MaxPower;
            else if (motorPower< MinPower) motorPower = MinPower;
            AddToTelemetry("error", String.valueOf(error));
            AddToTelemetry("current", String.valueOf(currentPosition));
            AddToTelemetry("Desired", String.valueOf(desiredTicks+previousPosition));
            AddToTelemetry("Speed", String.valueOf(motorPower));

            UpdateTelemetry();
            DriveTicks(motorPower);
        }
    }

    public void EncoderDrive(double distance, double correction) {
        double motorPos = FLM.getCurrentPosition();
        double desiredTicks = InchesToTicks(distance+correction);
        double currentPosition = motorPos;
        double previousPosition = motorPos;
        while (CanMove() && (currentPosition <= (desiredTicks + previousPosition))) {
            CheckEncoders();
            motorPos = Math.abs(FLM.getCurrentPosition());
            currentPosition = motorPos;
            DriveTicks(.4);
            AddToTelemetry("encoder:", String.valueOf(currentPosition));
            AddToTelemetry("Desired Position:", String.valueOf(desiredTicks + previousPosition));
            UpdateTelemetry();
        }

        DriveTicks(0);

        }
        //public void ParkOnTape(){
        //}



    protected double InchesToTicks(double inches){
        double tick = 55;
        double TicksNeeded = inches * tick;
        return TicksNeeded;
    }


    public void CheckEncoders(){
        while(CanMove()){
        double outputFL = FLM.getCurrentPosition();
        double outputFR = FRM.getCurrentPosition();
        double outputBL = BLM.getCurrentPosition();
        double outputBR = BRM.getCurrentPosition();


            AddToTelemetry("OutputFL", String.valueOf(outputFL));
            AddToTelemetry("OutputFR", String.valueOf(outputFR));
            AddToTelemetry("OutputBL", String.valueOf(outputBL));
            AddToTelemetry("OutputBR", String.valueOf(outputBR));

        UpdateTelemetry();}
    }
   // public void FlyPower(double power){
    //    double error = 0, P = 1;
      //  error = power - FlyMotor.getVelocity();
        //FlyMotor.setVelocity(P*error);
   // }



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
        FRM.setPower(forward);
        FLM.setPower(forward);
        BRM.setPower(forward);
        BLM.setPower(forward);
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
    public Move PathToZone(int pathChoice) {
        switch (pathChoice) {
            case 0: //zone A
                MoveToZone = new Move(84, 12, NormPower, org.firstinspires.ftc.teamcode.Move.Direction.Forward);
            case 1: // Zone B
                MoveToZone = new Move(108, 36, NormPower, org.firstinspires.ftc.teamcode.Move.Direction.Forward);
            case 2: //Zone C
                MoveToZone = new Move(132, 12, NormPower, org.firstinspires.ftc.teamcode.Move.Direction.Forward);


        }
        return null;
    }
    public void TeleOp(double power, Gamepad gamepad1){
        double forward = gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = gamepad1.left_stick_x;

        double FLMpower = (forward + strafe - turn)*power;
        double FRMpower = (forward - strafe + turn)*power;
        double BLMpower = (forward - strafe - turn)*power;
        double BRMpower = (forward + strafe + turn)*power;

        double correction = 0; //default correction
        if (turn == 0) {
            if (forward != 0) {
                correction = CheckDirection(Math.abs(forward)); //if there isn't any rotation then use correction
            } else if (strafe != 0) {
                correction = CheckDirection(Math.abs(strafe)); //if there isn't any rotation then use correction
            }
        }

        FLM.setPower(FLMpower - correction); //set the power to the motor
        FRM.setPower(FRMpower + correction);
        BLM.setPower(BLMpower - correction);
        BRM.setPower(BRMpower + correction);
    }
    public void TeleOp(double power, Gamepad gamepad1, Gamepad gamepad2) {
        while (CanMove()) {
            double forward = gamepad1.right_stick_y;
            double strafe = gamepad1.right_stick_x;
            double turn = gamepad1.left_stick_x;

            double FLMpower = (forward + strafe - turn) * power;
            double FRMpower = (forward - strafe + turn) * power;
            double BLMpower = (forward - strafe - turn) * power;
            double BRMpower = (forward + strafe + turn) * power;

            double correction = 0; //default correction
            if (turn == 0) {
                if (forward != 0) {
                    correction = CheckDirection(Math.abs(forward)); //if there isn't any rotation then use correction
                } else if (strafe != 0) {
                    correction = CheckDirection(Math.abs(strafe)); //if there isn't any rotation then use correction
                }
            }

            FLM.setPower(FLMpower - correction); //set the power to the motor
            FRM.setPower(FRMpower + correction);
            BLM.setPower(BLMpower - correction);
            BRM.setPower(BRMpower + correction);

        }
    }

    public void ReadColorSensor() {
        while (CanMove()) {
            int red = CS.red();
            int green = CS.green();
            int blue = CS.blue();

            AddToTelemetry("red", String.valueOf(red));
            AddToTelemetry("green", String.valueOf(green));
            AddToTelemetry("blue", String.valueOf(blue));

            UpdateTelemetry();
        }
    }

    public void ParkOnTape(){
        EncoderPID(84,0);
    }
}

