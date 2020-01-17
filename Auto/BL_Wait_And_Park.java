package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//This auto was made by Gabo Gang on Oct 15 2019

@Autonomous(name = "Bl-Wait and Park", group = "Bl")
public class BL_Wait_And_Park extends LinearOpMode {
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

        sleep(15 * 1000);
        //Move to bricks
        Bot.RushB(Bot.MoveToFarPark);
        Bot.StopRobot();

        Bot.DriveToTape(Bot.DirectionToTape.InverseDirection());
        Bot.StopRobot();

        stop();


    }
}
