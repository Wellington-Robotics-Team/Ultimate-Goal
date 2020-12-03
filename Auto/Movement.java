package org.firstinspires.ftc.teamcode;

public class Movement {
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
    Movement(int Distance, int Correction, double MaxPower, Directions Direction) {
        this.Distance = Distance;
        this.Correction = Correction;
        this.MaxPower = MaxPower;
        this.Direction = Direction;
    }

    Movement(int Distance, int Correction, double MaxPower) {
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
