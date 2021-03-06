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

import android.graphics.Color;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;
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

@TeleOp(name="TeleOpMode_TankJohn", group="Iterative Opmode")
public class TeleOpMode_TankJohn extends OpMode
{
    boolean debug = false;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private Servo servo = null;
    private Servo servo2 = null;
    // private GyroSensor gyro = null;
    private ColorSensor colorSensor;
    // private UltrasonicSensor distanceSensor = null;

    // The IMU sensor object
    BNO055IMU imu;

    private int x;
    private int triggerSet;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        x = 0;
        triggerSet = 0;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        try {
            leftfrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
            rightfrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
            leftbackDrive = hardwareMap.get(DcMotor.class, "backLeft");
            rightbackDrive = hardwareMap.get(DcMotor.class, "backRight");
            servo = hardwareMap.get(Servo.class, "gripper");
            servo2 = hardwareMap.get(Servo.class, "arm");
            //gyro = hardwareMap.get(GyroSensor.class, "gyro");
            colorSensor = hardwareMap.colorSensor.get("colorSensor");

            /*gyro.calibrate();

            while(gyro.isCalibrating()){
                try {Thread.sleep(100);}
                    catch(Exception e){}
            }*/

            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            leftfrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightfrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftbackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightbackDrive.setDirection(DcMotor.Direction.REVERSE);

            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }
        catch (IllegalArgumentException iax) {
            debug = true;
        }

        //distanceSensor = hardwareMap.get(UltrasonicSensor.class, "distanceSensor");
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        if(!debug) {
            leftfrontDrive.setPower(0);
            rightfrontDrive.setPower(0);
            leftbackDrive.setPower(0);
            rightbackDrive.setPower(0);
            servo.setPosition(0);
            servo2.setPosition(0);
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower = 0;
        double leftRearPower = 0;
        double rightFrontPower = 0;
        double rightRearPower = 0;
        double gripper = 0;
        double arm = 0;
        double joyStick;
        String nothing = "";

        joyStick  = -gamepad1.left_stick_y;

        if((gamepad1.right_trigger > 0) && (triggerSet == 0)){
            x = x + 1;
            triggerSet = 1;
            if(x > 5){
                x = 0;
            }
        }
        else if((triggerSet == 1) && (gamepad1.right_trigger == 0)) {
            triggerSet = 0;
        }

        if(x == 0){
            leftFrontPower = joyStick;
            nothing = "leftFront";
        }
        else if(x == 1){
            rightFrontPower = joyStick;
            nothing = "rightFront";
        }
        else if(x == 2){
            leftRearPower = joyStick;
            nothing = "leftRear";
        }
        else if(x == 3){
            rightRearPower = joyStick;
            nothing = "rightRear";
        }
        else if(x == 4){
            gripper = Math.abs(joyStick);
            nothing = "gripper";
        }
        else if(x == 5){
            arm = Math.abs(joyStick);
            nothing = "arm";
        }

        telemetry.addData("x", x);
        telemetry.addData("activated: ", nothing);
        telemetry.addData("leftFrontPower", leftFrontPower);
        telemetry.addData("rightFrontPower", rightFrontPower);
        telemetry.addData("leftRearPower", leftRearPower);
        telemetry.addData("rightRearPower", rightRearPower);
        telemetry.addData("gripperPosition", gripper);
        telemetry.addData("armPosition", arm);
        /*try {
            telemetry.addData("gyro", gyro.getHeading());
        }
        catch (IllegalArgumentException iax) {
            debug = true;
        }*/

        telemetry.addData("Luminosity", colorSensor.alpha());
        telemetry.addData("Red sensor", colorSensor.red());
        telemetry.addData("Blue sensor", colorSensor.blue());

        if(colorSensor.blue() - colorSensor.red() >= 0){
            telemetry.addData("Color Blue", colorSensor.blue() - colorSensor.red());
        }
        else if(colorSensor.red() - colorSensor.blue() >= 20){
            telemetry.addData("Color Red", colorSensor.red() - colorSensor.blue());
        }
        else{
            if(colorSensor.red() > colorSensor.blue()){
                telemetry.addData("Too close, red bigger than blue by", colorSensor.red() - colorSensor.blue());
            }
            else{
                telemetry.addData("Colors are equal", 0);
            }
        }

        final Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addLine()
                .addData("status", imu.getSystemStatus().toShortString())
                .addData("calib", imu.getCalibrationStatus().toString())
                .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));

        // telemetry.addData("distanceSensor", distanceSensor.status());

        // Send calculated power to wheels
        if(debug == false) {
            leftfrontDrive.setPower(leftFrontPower);
            rightfrontDrive.setPower(rightFrontPower);
            leftbackDrive.setPower(leftRearPower);
            rightbackDrive.setPower(rightRearPower);
            servo.setPosition(gripper);
            servo2.setPosition(arm);
        }

        // Show the elapsed game time
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
