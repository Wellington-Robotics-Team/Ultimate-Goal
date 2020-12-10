package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Functions {
    Telemetry telemetry;

    private DcMotor FLM;
    private DcMotor FRM;
    private DcMotor BLM;
    private DcMotor BRM;
    
    private DistanceSensor TheDS;
    private DistanceSensor FDS;
    private DistanceSensor BDS;
    private DistanceSensor LDS;
    private DistanceSensor RDS;

    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation(); //sets the last angle to whatever the robot last had. This is just to avoid errors

    private double globalAngle; //the number of degrees the robot has turned

    boolean AllowedToMove;
    
    public double PIDMoveAndTurn(Move finalDist) {

        double error = 0, previous_error = 0, derivative = 0, power = 0;
        double P = 2, I = 0, D = 3;
        boolean CanMove = true;
        double correction = 5;
        whichDistanceSensor(finalDist.MovingToward());

        while (CanMove == true) {

            error = finalDist.Distance() - CurrentDist(); // Error = Target - Actual
            derivative = (error - previous_error) / .02;
            power = P * error + D * derivative;
            finalDist.Power(power);
            previous_error = error;

            telemetry.addData("error", error);

            if (CurrentDist() > finalDist.Distance() + correction){
                CanMove = false;
            }
            else{
                CanMove = true;
            }

        }
        return power;
    }

    

    public void whichDistanceSensor(Move.Direction direction){

        switch (direction){
            case Right:
                TheDS = RDS;
            case Left:
                TheDS = LDS;
            case Backwards:
                TheDS = BDS;
            case Forward:
                TheDS = FDS;

        }
    }
    public double CurrentDist(){
        return TheDS.getDistance(DistanceUnit.CM);
    }
    
    


    public void Init(HardwareMap hardwareMap){

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

        RDS = hardwareMap.get(DistanceSensor .class, "RDS");
        LDS = hardwareMap.get(DistanceSensor.class, "LDS");
        FDS = hardwareMap.get(DistanceSensor.class, "FDS");
        BDS = hardwareMap.get(DistanceSensor.class, "BDS");

        imu = hardwareMap.get(BNO055IMU.class, "imu"); //gets the imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //makes parameters for imu
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters); //initalizes the imu
        while (AllowedToMove && !imu.isGyroCalibrated()) {
            if (!AllowedToMove) return;
        }

        ResetAngle();
    }

    private void ResetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //sets lastAngles to current angles

        globalAngle = 0; //global angle is set to 0
    }
}

