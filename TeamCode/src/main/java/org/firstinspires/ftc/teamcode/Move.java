package org.firstinspires.ftc.teamcode;

public class Move {
    public enum Direction{
        Forward,
        Backwards,
        Right,
        Left;
    }

    double distanceLength;
    double distanceWidth;
    double distance = Math.sqrt(distanceLength*distanceLength + distanceWidth * distanceWidth);
    double power;
    Direction direction;
   public Move(double distanceLength, double distanceWidth, double power, Direction direction){
        this.distanceLength = distanceLength;
        this.distanceWidth = distanceWidth;
        this.power = power;
        this.direction = direction;
    }
    public Direction MovingToward(){
        return this.direction;
    }
    public double ForwardDistance(){
       return this.distanceLength;
    }
    public double SideDistance(){
        return this.distanceWidth;
    }
    public void Power(double power){
       this.power = power;
    }
    public double GetPower(){return this.power;}
    public double GetAngle(){
       return Math.tan(distanceLength / distanceWidth);
    }
}
