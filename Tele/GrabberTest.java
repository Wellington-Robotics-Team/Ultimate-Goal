package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//This auto was made by Gabo Gang on Oct 15 2019

@TeleOp(name = "GrabberTest", group = "Iterative Opmode")
public class GrabberTest extends OpMode {
    private Servo ClawGrab = null;
    private Servo ClawSwivel = null;

    private double ClawGrabRestPosition = 0;
    private double ClawGrabGrabPosition = 0.4;

    private double ClawSwivelRestPosition = 0;
    private double ClawSwivelOutPosition = 1;

    private boolean APressed = false;
    private boolean YPressed = false;

    @Override
    public void init() {
        //Init
        ClawGrab = hardwareMap.servo.get("ClawGrab");
        ClawGrab.setDirection(Servo.Direction.FORWARD);
        ClawGrab.setPosition(ClawGrabRestPosition);

        ClawSwivel = hardwareMap.servo.get("ClawSwivel");
        ClawSwivel.setDirection(Servo.Direction.REVERSE);
        ClawSwivel.setPosition(ClawSwivelRestPosition);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            if (!APressed) {
                APressed = true;
                if (ClawGrab.getPosition() != ClawGrabGrabPosition)
                    ClawGrab.setPosition(ClawGrabGrabPosition);
                else ClawGrab.setPosition(ClawGrabRestPosition);
            }
        } else if (APressed) APressed = false;

        if (gamepad1.y) {
            if (!YPressed) {
                YPressed = true;
                if (ClawSwivel.getPosition() != ClawSwivelOutPosition)
                    ClawSwivel.setPosition(ClawSwivelOutPosition);
                else ClawSwivel.setPosition(ClawSwivelRestPosition);
            }
        } else if (YPressed) YPressed = false;

        telemetry.addData("ClawGrabGrabPosition", ClawSwivelOutPosition);
        telemetry.addData("Servo Position", ClawSwivel.getPosition());
        telemetry.update();
    }
}
