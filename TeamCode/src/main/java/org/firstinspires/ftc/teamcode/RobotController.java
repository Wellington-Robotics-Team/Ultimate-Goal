package org.firstinspires.ftc.teamcode;

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

//declaring the Movements class
class Movements {
    public enum Directions {
        Right,
        Left,
        Forward,
        Backwards;
        public Directions InverseDirection() {
            switch (this) {
                case Forward:
                    return Directions.Backwards;
                case Backwards:
                    return Directions.Forward;
                case Left:
                    return Directions.Right;
                default:
                    return Directions.Left;
            }
        }
    }

    private int Distance;
    private int Correction;
    private double distanceTraveled = 0; //sets the distance traveled to 0

    private double LastDistance = 0;

    private double MaxPower;

    private double Slope;

    private double MinPower = 0.15;

    private Directions Direction;
    Movements(int Distance, int Correction, double MaxPower, Directions Direction) {
        this.Distance = Distance;
        this.Correction = Correction;
        this.MaxPower = MaxPower;
        this.Direction = Direction;
    }

    Movements(int Distance, int Correction, double MaxPower) {
        this.Distance = Distance;
        this.Correction = Correction;
        this.MaxPower = MaxPower;
    }

    int GetTotalDistance() {
        return Distance + Correction;
    }

    void setPowerSign(int sign) {
        this.MaxPower *= sign;
        this.MinPower *= sign;
    }
    Directions getDirection() {
        return Direction;
    }

    void setStartDistance(double StartDistance) {
        double totalNeededToTravel = StartDistance - this.GetTotalDistance();
        this.LastDistance = StartDistance;
        double TotalNeededCubed = totalNeededToTravel * totalNeededToTravel * totalNeededToTravel;
        this.Slope = -MaxPower / TotalNeededCubed; //gets the slope of the graph that is needed to make y = 0 when totalNeeded to travel is x

    }

    void UpdateDistanceTraveled(double NewDistance) {
        distanceTraveled += LastDistance - NewDistance;

        LastDistance = NewDistance;
    }

    double CalculatePower() {
        double distanceCubed = distanceTraveled * distanceTraveled * distanceTraveled;
        double power = (Slope * distanceCubed) + MaxPower; // the power is the x value in that position

        if (power > MaxPower) power = MaxPower;
        else if (power < -MaxPower) power = -MaxPower;

        else if (power < MinPower && power > 0) power = MinPower; //if the power is less than the min power level just set the power to the minpower level
        else if (power > MinPower && power < 0) power = -MinPower;

        return power;
    }

    void SetDirection (Directions Direction) {
        this.Direction = Direction;
    }
}


abstract class Robot_Controller {
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

    //declare color sensor
    // private ColorSensor FloorCS = null;
    // private ColorSensor RightCS = null;

    public class Drive{
        int P , I , D = 1;
        int integral, previous_error, setpoint = 0;
        Movements robotDrive;

    }
}
