package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//This auto was made by Gabo Gang on Oct 15 2019

@Autonomous(name = "Rd-Park D", group = "Rd")
public class Rd_Park_D extends LinearOpMode {
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

        Bot.DriveToTape(Bot.DirectionToTape.InverseDirection());
        Bot.StopRobot();

        stop();


    }
}
