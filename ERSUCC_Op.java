package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//Made by ERSUCC on October 21st, 2019

@TeleOp(name="ERSUCC Op", group="Iterative Opmode")
@Disabled
public class ERSUCC_Op extends OpMode {
    private DcMotor LeftMotor = null;
    private DcMotor RightMotor = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        LeftMotor = hardwareMap.get(DcMotor.class, "LeftMotor");
        RightMotor = hardwareMap.get(DcMotor.class, "RightMotor");

        RightMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");

        LeftMotor.setPower(gamepad1.right_trigger < 1 ? 0 : 0.5);
        RightMotor.setPower(gamepad1.right_trigger < 1 ? 0 : 0.5);

        telemetry.update();
    }

    public void SpinWheels() {

    }
}