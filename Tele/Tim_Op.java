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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is a tele op mode that will be used during driver control
 */

//Made by Gabo on October 8th, 2019

@TeleOp(name = "Tim Op", group = "Iterative Opmode")
public class Tim_Op extends OpMode {
    // Declare motors out here so we can use them globally
    private DcMotor LM = null; //private class name = null
    private DcMotor MM = null; //private because it's good coding practice
    private DcMotor RM = null; //DcMotor because that is what we will be assigning it to

    private Servo Swingy = null;

    private final double Power = 0.75; //decimal number that won't be changed named Power

    private double ServoPosition = 0;

    private double ServoSpeed = 0.1;

    private boolean LeftBumperPressed = false;
    private boolean RightBumperPressed = false;
    private boolean BPressed = false;
    private boolean XPressed = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing"); //display on the drivers phone that its working

        LM = hardwareMap.get(DcMotor.class, "LM"); //Go into the config and get the device named "FLM" and assign it to FLM
        RM = hardwareMap.get(DcMotor.class, "RM"); //device name doesn't have to be the same as the variable name
        MM = hardwareMap.get(DcMotor.class, "MM"); //DcMotor.class because that is what the object is

        //Make it so we don't have to add flip the sign of the power we are setting to half the motors
        //LM.setDirection(DcMotor.Direction.REVERSE); //Run the right side of the robot backwards
        MM.setDirection(DcMotor.Direction.REVERSE);

        LM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Swingy = hardwareMap.get(Servo.class, "Swingy");

        telemetry.addData("Status", "Initialized");   //
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running"); //inform the driver

        //Calculate the power to set to each motor

        //left side you subtract right_stick_x
        double LeftPower = gamepad1.left_stick_y - gamepad1.left_stick_x;
        double RightPower = gamepad1.left_stick_y + gamepad1.left_stick_x;

        LM.setPower(LeftPower * Power); //set the power to the motor
        RM.setPower(RightPower * Power);

        MM.setPower(-gamepad1.right_stick_x);

        if (gamepad1.b) {
            if (!BPressed) {
                BPressed = true;
                ServoSpeed += 0.1;
                if (ServoSpeed > 1) ServoSpeed = 1;
            }
        } else if (BPressed) BPressed = false;

        if (gamepad1.x) {
            if (!XPressed) {
                XPressed = true;
                ServoSpeed -= 0.1;
                if (ServoSpeed < 0) ServoSpeed = 0;
            }
        } else if (XPressed) XPressed = false;

        if (gamepad1.left_bumper) {
            if (!LeftBumperPressed) {
                LeftBumperPressed = true;
                ServoPosition -= ServoSpeed;
                if (ServoPosition < 0) ServoPosition = 0;
            }
        } else if (LeftBumperPressed) LeftBumperPressed = false;

        if (gamepad1.right_bumper) {
            if (!RightBumperPressed) {
                RightBumperPressed = true;
                ServoPosition += ServoSpeed;
                if (ServoPosition > 1) ServoPosition = 1;
            }
        } else if (RightBumperPressed) RightBumperPressed = false;

        Swingy.setPosition(ServoPosition);
        telemetry.update(); //update the telemetry
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
