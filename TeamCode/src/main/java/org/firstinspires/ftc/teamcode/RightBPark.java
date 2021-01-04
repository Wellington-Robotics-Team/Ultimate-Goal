//Randomization B
//Move Wobble
//Park on Launch Line

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//This auto was made by SiB on 12/2/2020

@Autonomous(name = "RightBPark")

public class RightBPark extends LinearOpMode {

    private UltimateGoalAuto UGA= new UltimateGoalAuto(telemetry);
    private Function Function = new Function();


    public class Function extends Functions {
        @Override
        public boolean CanMove() {
            return opModeIsActive() && !isStopRequested();
        }
        @Override
        public void AddToTelemetry(String Tag, String message) {
            telemetry.addData(Tag, message);
        }

        @Override
        public void UpdateTelemetry() {
            telemetry.update();
        }


}
    public void runOpMode() //when you press init
    {
        Function.Init(hardwareMap);


        // Tell the driver tha,t initialization is complete.
        telemetry.addData("Sta" +
                "us", "Initialized");

        telemetry.update();

        waitForStart(); //waits for the start button


        //Move to Zone

        Function.EncoderPID(84,24);

        stop();


    }
}

