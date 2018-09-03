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

package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.Collections;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOpMode_OriSquirrely", group="Iterative Opmode")
public class TeleOpMode_OriSquirrely extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private Servo arm = null;
    private boolean debug = false;
    private double armPos = 0;
    private double armChange = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftfrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightfrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftbackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightbackDrive = hardwareMap.get(DcMotor.class, "backRight");
        arm = hardwareMap.get(Servo.class, "arm");

        leftfrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftbackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightbackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (!debug) {
            leftfrontDrive.setPower(0);
            rightfrontDrive.setPower(0);
            leftbackDrive.setPower(0);
            rightbackDrive.setPower(0);
        }
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
        // Setup a variable for each drive wheel to save power level for telemetry
        double powerLeftY;
        double powerRightX;
        double powerLeftX;
        double powerLeftFront = 0;
        double powerRightFront = 0;
        double powerLeftBack = 0;
        double powerRightBack = 0;
        double powerLeft = 0;
        double powerRight = 0;
        double powerMax = 1;
        boolean padRight = false;
        boolean padLeft = false;
        boolean triggerLeft = false;
        boolean bumperLeft = false;


        powerLeftY  = -gamepad1.left_stick_y;
        powerRightX = -gamepad1.right_stick_x;
        powerLeftX = gamepad1.left_stick_x;
        padLeft = gamepad1.dpad_left;
        padRight = gamepad1.dpad_right;
        triggerLeft = gamepad1.right_bumper;
        bumperLeft = gamepad1.left_bumper;
        powerLeftX = scaleInput(powerLeftX, powerMax);
        powerRightX = scaleInput(powerRightX, powerMax);
        powerLeftY = scaleInput(powerLeftY, powerMax);

        powerMax = Collections.max(Arrays.asList(powerMax, powerLeftFront, powerLeftBack, powerRightFront, powerRightBack, powerLeftX));

        boolean allZero = (abs(powerRightX) == 0) && (abs(powerLeftY) == 0) && (abs(powerLeftX) == 0);
        double powerTotal = (abs(powerRightX) + abs(powerLeftY) + abs(powerLeftX)) > powerMax ? (abs(powerRightX) + abs(powerLeftY) + abs(powerLeftX)) : powerMax;

        powerLeftFront = allZero ? 0 : ((-powerRightX + powerLeftY + powerLeftX) / (powerTotal * powerMax));
        powerLeftBack = allZero ? 0 : ((-powerRightX + powerLeftY - powerLeftX) / (powerTotal * powerMax));
        powerRightFront = allZero ? 0 : ((powerRightX + powerLeftY - powerLeftX) / (powerTotal * powerMax));
        powerRightBack = allZero ? 0 : ((powerRightX + powerLeftY + powerLeftX) / (powerTotal * powerMax));

        if (!debug) {
            // Send calculated power to wheels
            leftfrontDrive.setPower(powerLeftFront);
            rightfrontDrive.setPower(powerRightFront);
            leftbackDrive.setPower(powerLeftBack);
            rightbackDrive.setPower(powerRightBack);
            arm.setPosition(armPos);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftBack (%.2f), rightBack (%.2f)", powerLeftFront, powerRightFront, powerLeftBack, powerRightBack);
        telemetry.addData("padRight", padRight);
        telemetry.addData("padLeft", padLeft);
        telemetry.addData("powerMax", powerMax);
        telemetry.addData("allZero", allZero);
        telemetry.addData("powerTotal", powerTotal);
        telemetry.addData("powerLeftX", powerLeftX);
        telemetry.addData("powerRightX", powerRightX);
        telemetry.addData("powerLeftY", powerLeftY);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    double scaleInput(double dVal, double max_controller)  {
        return pow(dVal, 3)/pow(max_controller, 3);
    }
}
