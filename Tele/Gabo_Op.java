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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

/**
 * This is a tele op mode that will be used during driver control
 */

//Made by Gabo on October 8th, 2019
/**
 * Gamepad 1:
 * Left stick: Drive
 * Right Stick: rotate
 * B: On/off SUCK
 * Y: Drop/raise drop arm
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
@TeleOp(name = "TeleGabo", group = "Iterative Opmode")
public class Gabo_Op extends OpMode {
    // Declare motors out here so we can use them globally
    private DcMotor FLM = null; //private class name = null
    private DcMotor FRM = null; //private because it's good coding practice
    private DcMotor BLM = null; //DcMotor because that is what we will be assigning it to
    private DcMotor BRM = null; //BRM because it is the Back Right Motor (can be anything you want but make it readable)
    private DcMotor Suck = null;
    private DcMotor Lift = null;

    private Servo DragArm = null;
    private Servo LiftGrab = null;
    private Servo LiftSwivel = null;
    private Servo PushBlock = null;

    final private double DragArmRestPosition = 0.1;
    final private double DragArmUpPosition = 0.23;
    final private double DragArmDownPosition = 0.55;
    final private double LiftGrabRestPosition = 0;
    final private double LiftGrabGrabPosition = 0.3;
    final private double LiftSwivelRestPosition = 1;
    final private double LiftSwivelOutPosition = 0;
    final private double PushBlockRestPosition = 0;
    final private double PushBlockPushPosition = 0;
    final private double SuckPower = 0.333;
    final private double LiftPower = 1;
    final private int Position0Ticks = 0;
    final private int Position1Ticks = 0;
    final private int Position2Ticks = 6426;
    final private int Position3Ticks = 14364;
    final private int Position4Ticks = 22302;
    final private int Position5Ticks = 28350;


    private boolean LeftTrigger2 = false;
    private boolean LeftBumperPressed2 = false;
    private boolean RightBumperPressed2 = false;
    private boolean APressed2 = false;
    private boolean BPressed2 = false;
    private boolean YPressed2 = false;
    private boolean XPressed2 = false;
    private boolean BPressed1 = false;
    private boolean YPressed1 = false;

    private final double Power = 0.5; //decimal number that won't be changed named Power

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing"); //display on the drivers phone that its working

        FLM = hardwareMap.get(DcMotor.class, "FLM"); //Go into the config and get the device named "FLM" and assign it to FLM
        FRM = hardwareMap.get(DcMotor.class, "FRM"); //device name doesn't have to be the same as the variable name
        BLM = hardwareMap.get(DcMotor.class, "BLM"); //DcMotor.class because that is what the object is
        BRM = hardwareMap.get(DcMotor.class, "BRM");

        Suck = hardwareMap.get(DcMotor.class, "SUCK");

        Lift = hardwareMap.get(DcMotor.class, "LIFT");
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Make it so we don't have to add flip the sign of the power we are setting to half the motors
        //FRM.setDirection(DcMotor.Direction.REVERSE); //Run the right side of the robot backwards
        FLM.setDirection(DcMotor.Direction.REVERSE);
        BRM.setDirection(DcMotor.Direction.REVERSE); //the right motors are facing differently than the left handed ones

        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DragArm = hardwareMap.servo.get("drag_arm");
        DragArm.setDirection(Servo.Direction.REVERSE);
        DragArm.setPosition(DragArmRestPosition);

        LiftGrab = hardwareMap.servo.get("GRAB");

        LiftSwivel = hardwareMap.servo.get("SWIVEL");

        PushBlock = hardwareMap.servo.get("PUSH");

        telemetry.addData("Status", "Initialized");   //
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running"); //inform the driver

        //Calculate the power to set to each motor

        //left side you subtract right_stick_x
        double FrontLeftVal = (gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * Power; //front subtract left_stick_x
        double BackLeftVal = (gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * Power; //back subtract left_stick_x

        //right side you add right_stick_x
        double FrontRightVal = (gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * Power; //front add left_stick_x
        double BackRightVal = (gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * Power; //back subtract left_stick_x

        FLM.setPower(FrontLeftVal); //set the power to the motor
        FRM.setPower(FrontRightVal);
        BLM.setPower(BackLeftVal);
        BRM.setPower(BackRightVal);

        if (gamepad1.y) {
            if (!YPressed1) {
                YPressed1 = true;
                if (DragArm.getPosition() < DragArmDownPosition) {
                    DragArm.setPosition(DragArmDownPosition);
                } else {
                    DragArm.setPosition(DragArmUpPosition);
                }
            }
        } else if (YPressed1) YPressed1 = false;

        if (gamepad2.right_bumper) {
            if (!RightBumperPressed2) {
                RightBumperPressed2 = true;
                SetLiftPosition(Position5Ticks);
            }
        } else if (RightBumperPressed2) RightBumperPressed2 = false;

        if (gamepad2.a) {
            if (!APressed2) {
                APressed2 = true;
                SetLiftPosition(Position1Ticks);
            }
        } else if (APressed2) APressed2 = false;

        if (gamepad2.b) {
            if (!BPressed2) {
                BPressed2 = true;
                SetLiftPosition(Position2Ticks);
            }
        } else if (BPressed2) BPressed2 = false;

        if (gamepad2.y) {
            if (!YPressed2) {
                YPressed2 = true;
                SetLiftPosition(Position3Ticks);
            }
        } else if (YPressed2) YPressed2 = false;

        if (gamepad2.x) {
            if (!XPressed2) {
                XPressed2 = true;
                SetLiftPosition(Position4Ticks);
            }
        } else if (XPressed2) XPressed2 = false;

        if (gamepad2.left_trigger == 1) {
            if (!LeftTrigger2) {
                LeftTrigger2 = true;
                if (Lift.getCurrentPosition() >= Position1Ticks) {
                    SetLiftPosition(Lift.getCurrentPosition() - 1);
                } else {
                    SetLiftPosition(Position0Ticks);
                }
            }
        } else if (LeftTrigger2) LeftTrigger2 = false;

        if (gamepad2.left_bumper) {
            if (!LeftBumperPressed2) {
                LeftBumperPressed2 = true;
                if (LiftGrab.getPosition() != LiftGrabRestPosition)
                    LiftGrab.setPosition(LiftGrabRestPosition);
                else LiftGrab.setPosition(LiftGrabGrabPosition);
            }
        } else if (LeftBumperPressed2) LeftBumperPressed2 = false;

        if (gamepad1.b) {
            if (!BPressed1) {
                BPressed1 = true;
                if (Suck.getPower() == SuckPower) {
                    Suck.setPower(0);
                } else {
                    Suck.setPower(SuckPower);
                }
            }
        } else if (BPressed1) BPressed1 = false;


        telemetry.addData("Servo Position", DragArm.getPosition());
        telemetry.update(); //update the telemetry
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        if (FLM != null) FLM.setPower(0);
        if (FRM != null) FRM.setPower(0);
        if (BLM != null) BLM.setPower(0);
        if (BRM != null) BRM.setPower(0);
        if (Suck != null) Suck.setPower(0);
        if (Lift != null) Lift.setPower(0);
        if (DragArm != null) DragArm.setPosition(DragArmRestPosition);
        if (LiftGrab != null) LiftGrab.setPosition(LiftGrabRestPosition);
        if (LiftSwivel != null) LiftSwivel.setPosition(LiftSwivelRestPosition);
        if (PushBlock != null) PushBlock.setPosition(PushBlockRestPosition);
    }

    private void SetLiftPosition(int TargetPosition) {
        DragArm.setPosition(DragArmDownPosition);

        if (LiftSwivel.getPosition() != LiftSwivelOutPosition) {
            Lift.setTargetPosition(Position5Ticks);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Lift.isBusy()) {
                Lift.setPower(LiftPower);
            }
            LiftSwivel.setPosition(LiftSwivelOutPosition);
        }
        Lift.setTargetPosition(TargetPosition);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (Lift.getCurrentPosition() < TargetPosition) {
            if (TargetPosition == Position0Ticks) LiftSwivel.setPosition(LiftSwivelRestPosition);
            while (Lift.isBusy()) {
                Lift.setPower(-LiftPower);
            }
        } else if (Lift.getCurrentPosition() > TargetPosition) {
            while (Lift.isBusy()) {
                Lift.setPower(LiftPower);
            }
        }
        Lift.setPower(0);
        DragArm.setPosition(DragArmRestPosition);
    }
}
