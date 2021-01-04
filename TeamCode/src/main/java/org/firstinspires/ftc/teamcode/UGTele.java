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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.ForkJoinPool;

@TeleOp(name="UGTeleop", group="Iterative Opmode")
public class UGTele extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FLM = null;
    private DcMotor FRM = null;
    private DcMotor BRM = null;
    private DcMotor BLM = null;
   // private DcMotor IntakeMotor = null;
    //private DcMotor BeltMotor = null;
    //private DistanceSensor RDS;
    //private DistanceSensor BDS;
    private double MovePower;
    private boolean IntakeOn = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        MovePower = 0.5;
        FLM  = hardwareMap.get(DcMotor.class, "FLM");
        FRM = hardwareMap.get(DcMotor.class, "FRM");
        BLM = hardwareMap.get(DcMotor.class, "BLM");
        BRM = hardwareMap.get(DcMotor.class, "BRM");

       // RDS = hardwareMap.get(DistanceSensor.class, "RDS");
       // BDS = hardwareMap.get(DistanceSensor.class, "BDS");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FRM.setDirection(DcMotor.Direction.REVERSE);
        BRM.setDirection(DcMotor.Direction.REVERSE); //the right motors are facing differently than the left handed ones

        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//dont lock up when not moving

        //BeltMotor = hardwareMap.get(DcMotor.class,"BeltMotor");
        //BeltMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //BeltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //something about gyros figure this out later
        BNO055IMU imu;
        Orientation angles;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        runtime.reset();

    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running"); //inform the driver

        double forward = gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = gamepad1.left_stick_x;
        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("turn", turn);



        double power = .7;

        double FLMpower = (forward + strafe - turn)*power;
        double FRMpower = (forward - strafe + turn)*power;
        double BLMpower = (forward - strafe - turn)*power;
        double BRMpower = (forward + strafe + turn)*power;

        FLM.setPower(FLMpower); //set the power to the motor
        FRM.setPower(FRMpower);
        BLM.setPower(BLMpower);
        BRM.setPower(BRMpower);
        telemetry.addData("Power Average", (FLMpower+BLMpower+FRMpower+BRMpower)/4);


        if (gamepad1.b){
            if (IntakeOn = true){
                IntakeOn=false;
            }else {
                IntakeOn = true;
            }
        }
        /*while (IntakeOn=true){ //finish this (turn intake and belt on and off)
            IntakeMotor.setPower(0.8);
            BeltMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            BeltMotor.setPower(0.8);
        }
        if (gamepad1.a){
            IntakeOn=false;
            BeltMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            BeltMotor.setPower(0.3);
        }
        while (IntakeOn=false && !gamepad1.a) {
            BeltMotor.setPower(0);
        }
        if (gamepad1.y){
//spin to face goal/initialorientation. move right to allign with goal. raise shooter and start spinning wheel. Shooting is left to A (the person)

        }
*/
        // Show the elapsed game time
        telemetry.addData("Status", "Run Time: " + runtime.toString());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        FLM.setPower(0);
        BLM.setPower(0);
        BRM.setPower(0);
        FRM.setPower(0);
        //BeltMotor.setPower(0);
       // IntakeMotor.setPower(0);
    }
    public void RotateWithGyro (double desiredAngle, double TurningSpeed){

    }
    public void MoveUsingDistanceSensor (String direction,double DistanceFromWall) {
        //obtain direction (Forwards, backwards, right,left)
        //ramp up motor speed until half way to distancefromwall, then ramp down faster. Finish moving at 0.1 power

    }
}
