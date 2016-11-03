/*
ADB guide can be found at:
https://ftcprogramming.wordpress.com/2015/11/30/building-ftc_app-wirelessly/
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@Autonomous(name="Red Alliance", group="Autonomous")
public class AutoMecanumFinal extends LinearOpMode {
    //TWEAKING VALUES
    public static final double BLOCKSERVOOPENVALUE = 0;
    public static final double BLOCKSERVOCLOSEDVALUE = 1;
    public static final double LEFTSERVOMAXVALUE = 1;
    public static final double LEFTSERVOMINVALUE = 0;
    public static final double RIGHTSERVOMAXVALUE = 1;
    public static final double RIGHTSERVOMINVALUE = 0;
    public static final double MAXINFEEDPOWER = 1;

    //GOOD VALUES
    //I know these should be switched, but the hardware map is a pain to fix.
    public static final String LEFT1NAME = "l2";
    public static final String LEFT2NAME = "l1";
    //I know these should be switched, but the hardware map is a pain to fix.
    public static final String RIGHT1NAME = "r1";
    public static final String RIGHT2NAME = "r2";
    public static final String SHOOT1NAME = "sh1";
    public static final String SHOOT2NAME = "sh2";
    public static final String INFEEDNAME = "in";
    public static final String BALLBLOCKNAME = "b";
    public static final String LEFTPUSHNAME = "lp";
    public static final String RIGHTPUSHNAME = "rp";
    public static final String RANGENAME = "r";
    public static final String COLORSIDENAME = "cs";
    public static final String COLORBOTTOMNAME = "cb";

    DcMotor leftFrontWheel, leftbBackWheel, rightFrontWheel, rightBackWheel, shoot1, shoot2, infeed;
    Servo leftButtonPusher, rightButtonPusher, ballBlock;
    ColorSensor colorSensorOnBottom, colorSensorOnSide;
    ModernRoboticsI2cRangeSensor range;

    //Runs op mode
    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontWheel= hardwareMap.dcMotor.get(LEFT1NAME);
        leftbBackWheel= hardwareMap.dcMotor.get(LEFT2NAME);
        rightFrontWheel=hardwareMap.dcMotor.get(RIGHT1NAME);
        rightBackWheel= hardwareMap.dcMotor.get(RIGHT2NAME);
        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftbBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot1=hardwareMap.dcMotor.get(SHOOT1NAME);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2=hardwareMap.dcMotor.get(SHOOT2NAME);
        infeed=hardwareMap.dcMotor.get(INFEEDNAME);
        infeed.setDirection(DcMotorSimple.Direction.REVERSE);
        ballBlock=hardwareMap.servo.get(BALLBLOCKNAME);
        leftButtonPusher =hardwareMap.servo.get(LEFTPUSHNAME);
        rightButtonPusher =hardwareMap.servo.get(RIGHTPUSHNAME);
        leftButtonPusher.setPosition(LEFTSERVOMAXVALUE);
        rightButtonPusher.setPosition(RIGHTSERVOMINVALUE);
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, RANGENAME);
        colorSensorOnBottom = hardwareMap.colorSensor.get(COLORBOTTOMNAME);
        colorSensorOnSide = hardwareMap.colorSensor.get(COLORSIDENAME);
        telemetry.addData("raw ultrasonic", range.rawUltrasonic());
        telemetry.update();

        leftFrontWheel.setMode(DcMotor.RunMode.RESET_ENCODERS);
        while(leftFrontWheel.getCurrentPosition()!=0){
        }//wait
        int leftFrontWheelEncoderPosition = leftFrontWheel.getCurrentPosition();

        //WAIT FOR START IS HERE
        waitForStart();

        resetEncs(leftFrontWheel, rightFrontWheel, leftbBackWheel, rightBackWheel);
        telemetry.addData("Left", leftFrontWheelEncoderPosition);
        telemetry.update();
        drive(-.85, leftFrontWheel, rightFrontWheel, leftbBackWheel, rightBackWheel);
        while(Math.abs((leftFrontWheel.getCurrentPosition()-leftFrontWheelEncoderPosition)) < 1000) {
            telemetry.addData("Left Encoder", leftFrontWheel.getCurrentPosition() - leftFrontWheelEncoderPosition);
            telemetry.update();
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftbBackWheel, rightBackWheel);
        ballBlock.setPosition(BLOCKSERVOOPENVALUE); //Make sure to update
        shoot1.setPower(1); shoot2.setPower(1);
        infeed.setPower(MAXINFEEDPOWER);
        sleep(1000);
        shoot1.setPower(0); shoot2.setPower(0);
        ballBlock.setPosition(BLOCKSERVOCLOSEDVALUE); //Make sure to update
        infeed.setPower(0);
        drive(-.85, leftFrontWheel, rightFrontWheel, leftbBackWheel, rightBackWheel);
        while(Math.abs((leftFrontWheel.getCurrentPosition()-leftFrontWheelEncoderPosition)) < 1500) {
            telemetry.addData("Left Encoder", leftFrontWheel.getCurrentPosition() - leftFrontWheelEncoderPosition);
            telemetry.update();
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftbBackWheel, rightBackWheel);

        while(range.getDistance(DistanceUnit.CM) > 12){
            //strafes LEFT
            double  val = .5;
            leftFrontWheel.setPower(val);
            leftbBackWheel.setPower(-val);
            rightFrontWheel.setPower(-val);
            rightBackWheel.setPower(val);
            telemetry.addData("raw ultrasonic", range.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftbBackWheel, rightBackWheel);
        telemetry.update();
        leftFrontWheelEncoderPosition = leftFrontWheel.getCurrentPosition();
        drive(.3, leftFrontWheel, rightFrontWheel, leftbBackWheel, rightBackWheel);
        while(Math.abs((leftFrontWheel.getCurrentPosition()-leftFrontWheelEncoderPosition)) < 300) {
            telemetry.addData("Left", leftFrontWheel.getCurrentPosition() - leftFrontWheelEncoderPosition);
            telemetry.update();
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftbBackWheel, rightBackWheel);
        leftFrontWheelEncoderPosition = leftFrontWheel.getCurrentPosition();
        while(Math.abs((leftFrontWheel.getCurrentPosition()-leftFrontWheelEncoderPosition)) < 150){
            //strafes LEFT
            double  val = -.2;
            leftFrontWheel.setPower(val);
            leftbBackWheel.setPower(-val);
            rightFrontWheel.setPower(-val);
            rightBackWheel.setPower(val);
            telemetry.addData("raw ultrasonic", range.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftbBackWheel, rightBackWheel);

        while( ((colorSensorOnBottom.red()+ colorSensorOnBottom.blue()+ colorSensorOnBottom.green())/3) < 2){
            drive(-.2, leftFrontWheel, rightFrontWheel, leftbBackWheel, rightBackWheel);
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftbBackWheel, rightBackWheel);

        //AT FIRST BEACON
        telemetry.addData("Red Value, First Beacon", colorSensorOnSide.red());
        if(colorSensorOnSide.red() > 5){
            rightButtonPusher.setPosition(LEFTSERVOMAXVALUE);
        } else {
            leftButtonPusher.setPosition(RIGHTSERVOMINVALUE);
        }
        sleep(1000);

        drive(-.2, leftFrontWheel, rightFrontWheel, leftbBackWheel, rightBackWheel);
        sleep(500);
        leftButtonPusher.setPosition(LEFTSERVOMINVALUE);
        rightButtonPusher.setPosition(RIGHTSERVOMINVALUE);
        while( ((colorSensorOnBottom.red()+ colorSensorOnBottom.blue()+ colorSensorOnBottom.green())/3) < 2) {
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftbBackWheel, rightBackWheel);
        //AT SECOND BEACON
        telemetry.addData("Red Value, Second Beacon", colorSensorOnSide.red());
        if(colorSensorOnSide.red() > 5){
            rightButtonPusher.setPosition(RIGHTSERVOMAXVALUE);
        } else {
            leftButtonPusher.setPosition(LEFTSERVOMINVALUE);
        }
        sleep(500);
        leftButtonPusher.setPosition(LEFTSERVOMINVALUE);
        rightButtonPusher.setPosition(RIGHTSERVOMINVALUE);


    }

    //Constant Power
    public static void drive(double val, DcMotor left1, DcMotor right1, DcMotor left2, DcMotor right2){
        left1.setPower(val);
        left2.setPower(val);
        right1.setPower(val);
        right2.setPower(val);
    }

    //Mecanum:
    // y - forwards
    // x - side
    // c - rotation
    public static void arcade(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        double leftFrontVal = y + x + c;
        double rightFrontVal = y - x - c;
        double leftBackVal = y - x + c;
        double rightBackVal = y + x - c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }

        leftFront.setPower(leftFrontVal);
        rightFront.setPower(rightFrontVal);
        leftBack.setPower(leftBackVal);
        rightBack.setPower(rightBackVal);
    }

    //resets encoders
    public static void resetEncs(DcMotor left1, DcMotor right1, DcMotor left2, DcMotor right2){
        while(left1.getCurrentPosition() != 0){
            left1.setMode(DcMotor.RunMode.RESET_ENCODERS);
            left2.setMode(DcMotor.RunMode.RESET_ENCODERS);
            right1.setMode(DcMotor.RunMode.RESET_ENCODERS);
            right2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        }
        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}