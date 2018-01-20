package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;


/**
 * simple example of using a Step that makes a bot with "squirrely wheels" drive along a given course
 * Created by phanau on 10/31/16.
 */


// simple example sequence that tests time based "squirrely wheel" drive steps to drive along a prescribed path
@Autonomous(name="KartertestcodeOp", group ="Opmode")
//@Disabled
public class KartertestcodeOp extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    boolean bDebug = false;
    private DcMotor[][] motors = new DcMotor[2][2];
    // Set default power to 100%
    private float motorPower = 1.f;
    private Servo servo = null;
    private ColorSensor colorSensor;
    //@Override;
    public void init() {


        try {
            motors[0][0] = hardwareMap.dcMotor.get("frontLeft");
            motors[0][1] = hardwareMap.dcMotor.get("frontRight");
            motors[1][0] = hardwareMap.dcMotor.get("backLeft");
            motors[1][1] = hardwareMap.dcMotor.get("backRight");
            servo = hardwareMap.get(Servo.class, "arm");
            colorSensor=hardwareMap.colorSensor.get("colorSensor");
            // The motors on the left side of the robot need to be in reverse mode
            for(DcMotor[] motor : motors){
                motor[0].setDirection(DcMotor.Direction.REVERSE);
            }
            // Being explicit never hurt anyone, right?
            for(DcMotor[] motor : motors){
                motor[1].setDirection(DcMotor.Direction.FORWARD);
            }
        }
        catch (IllegalArgumentException iax) {
            bDebug = true;
        }
        bDebug = false;

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (!bDebug) {
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Luminosity", colorSensor.alpha());
        telemetry.addData("Red sensor", colorSensor.red());
        telemetry.addData("Blue sensor", colorSensor.blue());
    }

    @Override
    public void stop() {
        super.stop();
    }
}

