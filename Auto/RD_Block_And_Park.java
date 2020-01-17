package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//This auto was made by Gabo Gang on Oct 15 2019

@Autonomous(name = "Rd-Block And Park", group = "Rd")
public class RD_Block_And_Park extends LinearOpMode {
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

    private JankBot Bot = new JankBot(false);

    public void runOpMode() //when you press init
    {
        Bot.Init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart(); //waits for the start button

        //Move to bricks
        Bot.RushB(Bot.MoveToBricks);
        Bot.StopRobot();
        //move to black brick. 12 cm from first brick
        Bot.ScanForSkyStone(Bot.DirectionToScan);
        Bot.StopRobot();

        Bot.DropArm();

        //move to left wall. 12 cm from black block
        Bot.RushB(Bot.MoveToWallWithBlock);
        Bot.StopRobot();

        Bot.RushB(Bot.MoveToOtherSideOfField);
        Bot.StopRobot();

        Bot.RaiseArm();

        Bot.RushB(Bot.MoveToFarPark);
        Bot.StopRobot();

        Bot.DriveToTape(Bot.DirectionToTape);
        Bot.StopRobot();

        stop();


    }
}
