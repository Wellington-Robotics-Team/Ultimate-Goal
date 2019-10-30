package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//This auto was made by Gabo Gang on Oct 15 2019

@TeleOp(name = "Drag Arm Test", group = "Iterative Opmode")
public class DragArmTele extends OpMode {
    private Servo DragArm = null;

    final private double DragArmRestPosition = 0.87;
    final private double DragArmDownPosition = 0.55;

    private boolean APressed = false;

    @Override
    public void init() {
        //Init
        DragArm = hardwareMap.servo.get("drag_arm");
        DragArm.setDirection(Servo.Direction.FORWARD);
        DragArm.setPosition(DragArmRestPosition);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            if (!APressed) {
                APressed = true;
                if (DragArm.getPosition() != DragArmRestPosition)
                    DragArm.setPosition(DragArmRestPosition);
                else DragArm.setPosition(DragArmDownPosition);
            }
        } else if (APressed) APressed = false;

        telemetry.addData("Servo Position", DragArm.getPosition());
        telemetry.update();
    }
}
