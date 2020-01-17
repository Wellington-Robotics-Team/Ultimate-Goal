package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//This auto was made by Gabo Gang on Oct 15 2019

@Autonomous(name = "Bl-Plate and Park", group = "Bl")
public class Bl_Plate_And_Park extends LinearOpMode {
    public class JankBot extends Robot {
        @Override
        public boolean AllowedToMove() {
            return opModeIsActive() && !isStopRequested();
        }
        JankBot(boolean BlueSide) {
            super(BlueSide);
        }

        @Override
        public void AddToTelemetry(String Tag, String message) {
            telemetry.addData(Tag, message);
        }

        @Override
        public void UpdateTelemetry() {
            telemetry.update();
        }

        @Override
        public void Sleep(int Time) {
            sleep(Time);
        }
    }

    private JankBot Bot = new JankBot(true);

    public void runOpMode() //when you press init
    {
        Bot.Init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart(); //waits for the start button

        //Move to bricks
        Bot.RushB(Bot.MoveToPlate);
        Bot.StopRobot();
        //move to black brick. 12 cm from first brick
        Bot.RushB(Bot.MoveToMiddleOfPlate);
        Bot.StopRobot();

        Bot.DropArm();
        //Drive to left wall with plate. 12 cm to plate, 17cm from front wall
        Bot.RushB(Bot.MoveToWallWithPlate);
        Bot.StopRobot();

        Bot.RaiseArm();

        //drive to tape
        

        Bot.DriveToTape(Bot.DirectionToTape);
        Bot.StopRobot();

        stop();
    }
}
