/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is a tele op mode that will be used during driver control
 */

//Made by Gabo on October 8th, 2019
/**
 * Gamepad 1:
 * Left stick: Drive
 * Right Stick: rotate
 * B: On/off Grabby things
 */

/**
 * Gamepad 2:
 * Left trigger: place
 * Left bumper: open/close claw
 * Right bumper: Position 5
 * A: Position 1
 * B: Position 2
 * Y: Position 3
 * X: Position 4
 */
@TeleOp(name = "Servo Test", group = "Test")
public class ServoTest extends OpMode {
    private Servo DragArm = null;
    private Servo LiftGrab = null;
    private Servo LiftSwivel = null;
    private Servo PushBlock = null;

    private double DragarmPosition = 0;
    private double LiftGrabPosition = 0;
    private double LiftSwivelPosition = 0;
    private double PushBlockPosition = 0;

    private boolean LeftTriggerPressed = false;
    private boolean RightTriggerPressed = false;
    private boolean LeftBumperPressed = false;
    private boolean RightBumperPressed = false;
    private boolean DPADLeftPressed = false;
    private boolean DPADRightPressed = false;
    private boolean DPADUpPressed = false;
    private boolean DPADDownPressed = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing"); //display on the drivers phone that its working

        DragArm = hardwareMap.servo.get("drag_arm");
        DragArm.setDirection(Servo.Direction.REVERSE);
        DragArm.setPosition(DragarmPosition);

        LiftGrab = hardwareMap.servo.get("GRAB");
        LiftGrab.setPosition(LiftGrabPosition);

        LiftSwivel = hardwareMap.servo.get("SWIVEL");
        LiftSwivel.setPosition(LiftSwivelPosition);

        PushBlock = hardwareMap.servo.get("PUSH");
        PushBlock.setPosition(PushBlockPosition);

        telemetry.addData("Status", "Initialized");   //
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running"); //inform the driver

        if (gamepad1.left_trigger == 1) {
            if (!LeftTriggerPressed) {
                LeftTriggerPressed = true;
                LiftGrabPosition -= 0.01;
            }
        } else if (LeftTriggerPressed) LeftTriggerPressed = false;

        if (gamepad1.right_trigger == 1) {
            if (!RightTriggerPressed) {
                RightTriggerPressed = true;
                LiftGrabPosition += 0.01;
            }
        } else if (RightTriggerPressed) RightTriggerPressed = false;

        if (gamepad1.left_bumper) {
            if (!LeftBumperPressed) {
                LeftBumperPressed = true;
                LiftSwivelPosition -= 0.01;
            }
        } else if (LeftBumperPressed) LeftBumperPressed = false;

        if (gamepad1.right_bumper) {
            if (!RightBumperPressed) {
                RightBumperPressed = true;
                LiftSwivelPosition += 0.01;
            }
        } else if (RightBumperPressed) RightBumperPressed = false;

        if (gamepad1.dpad_left) {
            if (!DPADLeftPressed) {
                DPADLeftPressed = true;
                DragarmPosition -= 0.05;
            }
        } else if (DPADLeftPressed) DPADLeftPressed = false;

        if (gamepad1.dpad_right) {
            if (!DPADRightPressed) {
                DPADRightPressed = true;
                DragarmPosition += 0.05;
            }
        } else if (DPADRightPressed) DPADRightPressed= false;

        if (gamepad1.dpad_up) {
            if (!DPADUpPressed) {
                DPADUpPressed = true;
                PushBlockPosition += 0.01;
            }
        } else if (DPADUpPressed) DPADUpPressed= false;

        if (gamepad1.dpad_down) {
            if (!DPADDownPressed) {
                DPADDownPressed = true;
                PushBlockPosition -= 0.01;
            }
        } else if (DPADDownPressed) DPADDownPressed = false;

        LiftSwivel.setPosition(LiftSwivelPosition);
        DragArm.setPosition(DragarmPosition);
        LiftGrab.setPosition(LiftGrabPosition);
        PushBlock.setPosition(PushBlockPosition);

        telemetry.addData("Drag Arm Position", DragArm.getPosition());
        telemetry.addData("Push Block Position", PushBlock.getPosition());
        telemetry.addData("Lift Grab Position", LiftGrab.getPosition());
        telemetry.addData("Lift Swivel Position", LiftSwivel.getPosition());
        telemetry.addData("Drag Arm Position Variable", DragarmPosition);
        telemetry.addData("Lift Grab Position Variable", LiftGrabPosition);
        telemetry.addData("Lift Swivel Position Variable", LiftSwivelPosition);
        telemetry.addData("Push Block Position Variable", PushBlockPosition);

        telemetry.update(); //update the telemetry
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
