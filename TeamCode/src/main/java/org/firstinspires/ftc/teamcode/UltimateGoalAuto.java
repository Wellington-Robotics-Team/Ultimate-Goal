package org.firstinspires.ftc.teamcode;




import org.firstinspires.ftc.robotcore.external.Telemetry;


public class UltimateGoalAuto {


    Telemetry telemetry;
     public UltimateGoalAuto(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    Move MoveToZone = new Move(0,0,0, Move.Direction.Forward);

    //Motor Speeds
    public double FlywheelSpeed = 0.9;
    public double NormPower = 0.4;

    public Move Path(int pathChoice) {
        switch (pathChoice) {
            case 0: //zone A
                MoveToZone = new Move(60, 12, NormPower, Move.Direction.Forward);
            case 1: // Zone B
                MoveToZone = new Move(36, 8, NormPower, Move.Direction.Backwards);
            case 2: //Zone C
                MoveToZone = new Move(12, 12, NormPower, Move.Direction.Forward);


        }
        return null;
    }



}
