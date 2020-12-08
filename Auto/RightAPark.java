//Randomization A 
//Move Wobble
//Park on Launch Line
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//This auto was made by SiB on 12/2/2020

@Autonomous(name = "RightAPark", group = "Red")

public class RightAPark extends LinearOpMode {
    public class JankBot extends Robot {
        @Override
        public boolean AllowedToMove() {
            return opModeIsActive() && !isStopRequested();
        }

        JankBot() {
            super(true);
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

    private JankBot Bot = new JankBot();
    private Robot_Controller Robot = new Robot_Controller();
        @Override
        public void UpdateTelemetry() {

        }

        @Override
        public void AddToTelemetry(String Tag, String Message) {

        }
    };

    public void runOpMode() //when you press init
    {
        Bot.Init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Sta" +
                "us", "Initialized");

        telemetry.update();

        waitForStart(); //waits for the start button


        //Move to Zone
        //Bot.Path(0);
        //Bot.RushB(Bot.MoveToZone);
        //Bot.RushB(Bot.MoveToCenterOfZone);
        Bot.Drive(Robot.PDPower(Bot.MoveToZone),0,0);
        Bot.StopRobot();
        stop();


    }
}
