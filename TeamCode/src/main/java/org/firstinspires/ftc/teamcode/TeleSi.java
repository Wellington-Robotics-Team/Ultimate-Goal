package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleSi")

public class TeleSi extends LinearOpMode {
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
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart(); //waits for the start button


        //Drive
        Function.TeleOp(.7,gamepad1);
        stop();


    }

}
