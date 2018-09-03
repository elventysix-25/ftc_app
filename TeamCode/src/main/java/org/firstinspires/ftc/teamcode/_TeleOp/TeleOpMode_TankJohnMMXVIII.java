package org.firstinspires.ftc.teamcode._TeleOp;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
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

@TeleOp(name="TeleOpMode_TankJohnMMXVIII", group="Iterative Opmode")
public class TeleOpMode_TankJohnMMXVIII extends OpMode
{
    private boolean debugLeftFrontDrive = false;
    private boolean debugRightFrontDrive = false;
    private boolean debugLeftBackDrive = false;
    private boolean debugRightBackDrive = false;
    private boolean debugServo = false;
    private boolean debugColorSensor = false;
    private boolean debugImu = false;
    private boolean debugRightGripperDrive = false;
    private boolean debugLeftGripperDrive = false;
    private boolean debugLiftGripperDrive = false;
    private boolean debugTurnGripperDrive = false;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private Servo servo = null;
    private ColorSensor colorSensor;
    private DcMotor rightGripperDrive = null;
    private DcMotor leftGripperDrive = null;
    private DcMotor liftGripperDrive = null;
    private DcMotor turnGripperDrive = null;

    private BNO055IMU imu;

    private int x;
    private int triggerSet;
    private String turnStation = "Up";
    private double turnUpPosition;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        x = 0;
        triggerSet = 0;
        //hell Karter, this is your computer and I wrote this on 1/10/18, 8:06 PM

        try {
            leftfrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
            leftfrontDrive.setDirection(DcMotor.Direction.FORWARD);
        }
        catch (IllegalArgumentException iax) {
            debugLeftFrontDrive = true;
            telemetry.addData("IllegalArgumentException", "frontLeft");
        }
        try{
            rightfrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
            rightfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        }
        catch (IllegalArgumentException iax) {
            debugRightFrontDrive = true;
            telemetry.addData("IllegalArgumentException", "frontRight");
        }
        try{
            leftbackDrive = hardwareMap.get(DcMotor.class, "backLeft");
            leftbackDrive.setDirection(DcMotor.Direction.FORWARD);
        }
        catch (IllegalArgumentException iax) {
            debugLeftBackDrive = true;
            telemetry.addData("IllegalArgumentException", "backLeft");
        }
        try{
            rightbackDrive = hardwareMap.get(DcMotor.class, "backRight");
            rightbackDrive.setDirection(DcMotor.Direction.REVERSE);
        }
        catch (IllegalArgumentException iax) {
            debugRightBackDrive = true;
            telemetry.addData("IllegalArgumentException", "backRight");
        }
        try{
            servo = hardwareMap.get(Servo.class, "jewelArm");
        }
        catch (IllegalArgumentException iax) {
            debugServo = true;
            telemetry.addData("Can't Find the servo: ", "jewelArm");
        }
        try{
            colorSensor = hardwareMap.colorSensor.get("colorSensor");
        }
        catch (IllegalArgumentException iax) {
            debugColorSensor = true;
            telemetry.addData("IllegalArgumentException", "colorSensor");
        }
        try{
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }
        catch (IllegalArgumentException iax) {
            debugImu = true;
            telemetry.addData("IllegalArgumentException", "imu");
        }
        try{
            rightGripperDrive = hardwareMap.get(DcMotor.class, "gripperRight");
            rightGripperDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (IllegalArgumentException iax) {
            debugRightGripperDrive = true;
            telemetry.addData("Can't find the motor", "gripperRight");
        }
        try{
            leftGripperDrive = hardwareMap.get(DcMotor.class, "gripperLeft");
            leftGripperDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (IllegalArgumentException iax) {
            debugLeftGripperDrive = true;
            telemetry.addData("Can't find the motor", "gripperLeft");
        }
        try{
            liftGripperDrive = hardwareMap.get(DcMotor.class, "gripperLift");
            liftGripperDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (IllegalArgumentException iax) {
            debugLiftGripperDrive = true;
            telemetry.addData("Can't find the motor", "gripperLift");
        }
        try{
            turnGripperDrive = hardwareMap.get(DcMotor.class, "gripperTurn");
            turnGripperDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turnUpPosition = turnGripperDrive.getCurrentPosition();
            telemetry.addData("Up position for the turn motor: ", turnUpPosition);
        }
        catch (IllegalArgumentException iax) {
            debugLiftGripperDrive = true;
            telemetry.addData("Can't find the motor", "gripperTurn");
        }

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        if(!debugLeftFrontDrive) {
            leftfrontDrive.setPower(0);
        }
        if(!debugRightFrontDrive) {
            rightfrontDrive.setPower(0);
        }
        if(!debugLeftBackDrive) {
            leftbackDrive.setPower(0);
        }
        if(!debugRightBackDrive) {
            rightbackDrive.setPower(0);
        }
        if(!debugServo) {
            servo.setPosition(0.2);
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double leftFrontPower = 0;
        double leftRearPower = 0;
        double rightFrontPower = 0;
        double rightRearPower = 0;
        double arm = 0.2;
        double rightGripperPower = 0;
        double leftGripperPower = 0;
        double liftGripperPower = 0;
        double turnGripperPower = 0;
        double joyStick;
        String nothing = "";

        joyStick  = -gamepad1.left_stick_y;

        if((gamepad1.right_trigger > 0) && (triggerSet == 0)){
            x = x + 1;
            triggerSet = 1;
            if(x > 7){
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
            arm = Math.abs(joyStick);
            nothing = "arm";
        }
        else if(x==5){
            rightGripperPower = -joyStick;
            nothing = "rightGripper";
        }
        else if(x==6){
            leftGripperPower = joyStick;
            nothing = "leftGripper";
        }
        else if(x==7){
            liftGripperPower =joyStick;
            nothing = "liftGripper";
        }
        if (arm < 0.20){
            arm = 0.2;
        }

        telemetry.addData("x", x);
        telemetry.addData("activated: ", nothing);
        telemetry.addData("leftFrontPower", leftFrontPower);
        telemetry.addData("rightFrontPower", rightFrontPower);
        telemetry.addData("leftRearPower", leftRearPower);
        telemetry.addData("rightRearPower", rightRearPower);
        telemetry.addData("armPosition", arm);
        telemetry.addData("Up position for the turn motor: ", turnUpPosition);

        if(!debugColorSensor){

            telemetry.addData("Luminosity", colorSensor.alpha());
            telemetry.addData("Red sensor", colorSensor.red());
            telemetry.addData("Blue sensor", colorSensor.blue());

            if (colorSensor.blue() - colorSensor.red() >= 0) {
                telemetry.addData("Color Blue", colorSensor.blue() - colorSensor.red());
            }
            else if (colorSensor.red() - colorSensor.blue() >= 20) {
                telemetry.addData("Color Red", colorSensor.red() - colorSensor.blue());
            }
            else {
                if (colorSensor.red() > colorSensor.blue()) {
                    telemetry.addData("Too close, red bigger than blue by", colorSensor.red() - colorSensor.blue());
                }
                else {
                    telemetry.addData("Colors are equal", 0);
                }
            }
        }

        if(!debugImu) {

            final Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addLine()
                    .addData("status", imu.getSystemStatus().toShortString())
                    .addData("calib", imu.getCalibrationStatus().toString())
                    .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
        }

        if(!debugLeftFrontDrive) {
            leftfrontDrive.setPower(leftFrontPower);
        }
        if(!debugRightFrontDrive) {
            rightfrontDrive.setPower(rightFrontPower);
        }
        if(!debugLeftBackDrive) {
            leftbackDrive.setPower(leftRearPower);
        }
        if(!debugRightBackDrive) {
            rightbackDrive.setPower(rightRearPower);
        }
        if(!debugServo) {
            servo.setPosition(arm);
        }
        if(!debugRightGripperDrive) {
            rightGripperDrive.setPower(rightGripperPower);
        }
        if(!debugLeftGripperDrive) {
            leftGripperDrive.setPower(leftGripperPower);
        }
        if(!debugLiftGripperDrive) {
            liftGripperDrive.setPower(liftGripperPower);
        }

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