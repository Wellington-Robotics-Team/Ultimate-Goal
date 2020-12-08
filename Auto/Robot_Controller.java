package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public abstract class Robot_Controller {
    final private double FlyWheelPower = 0.9;
    final private double MaxPwr = 0.75; //the fastest it should go
    final public static double NormPower = 0.40; //The normal power to give to motors to drive
    final private double MinPower = 0.20; //slowest it should do
    static Movement MoveToZone;
    static Movement MoveToCenterOfZone;

    public Movement Path(int pathChoice) {
        switch (pathChoice) {
            case 0: //zone A
                MoveToZone = new Movement(60, 8, NormPower, Movement.Directions.Forward);
                MoveToCenterOfZone = new Movement(12, 8, NormPower, Movement.Directions.Left);
            case 1: // Zone B
                MoveToZone = new Movement(36, 8, NormPower, Movement.Directions.Backwards);
            case 2: //Zone C
                MoveToZone = new Movement(12, 8, NormPower, Movement.Directions.Forward);
                MoveToCenterOfZone = new Movement(12, 8, NormPower, Movement.Directions.Left);


        }
        return null;
    }

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

        public abstract void UpdateTelemetry();

        public abstract void AddToTelemetry(String Tag, String Message);


    //declare color sensor
    // private ColorSensor FloorCS = null;
    // private ColorSensor RightCS = null;

    public class Drive{
        int P , I , D = 1;
        double error;
        double derivative;
        double power;
        int integral, previous_error = 0;
        DistanceSensor TheDS;
        public Drive(){
           }

        private DistanceSensor RDS;
        private DistanceSensor LDS;
        private DistanceSensor FDS;
        private DistanceSensor BDS;


        public void whichDistanceSensor(Movement.Directions direction){

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


    }
    public double PD(Movement finalDist) {

        Drive drive = new Drive();
        double error = drive.error;
        double derivative = drive.derivative;
        double power = drive.power;
        int P = drive.P;
        int D = drive.D;
        boolean CanMove = true;
        double correction = 5;

        while (CanMove) {
            error = finalDist.GetTotalDistance() - drive.CurrentDist(); // Error = Target - Actual
            drive.derivative = (error - drive.previous_error) / .02;
            drive.power = P * error + D * derivative;
            finalDist.setPower(power);

            AddToTelemetry("Error", String.valueOf(error));

            if (drive.CurrentDist() > finalDist.GetTotalDistance() + correction){
                CanMove = false;
            }
            else{
                CanMove = true;
            }

        }
        return power;
    }
    public double PDPower(Movement Move){
       double power = PD(Move);
       return power;
    }
}
