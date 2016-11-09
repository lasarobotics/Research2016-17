package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

/**
 * Created by Ethan Schaffer on 11/8/2016.
 */

public class newAuto extends LinearOpMode {
    //GOOD VALUES
    public static final String LEFT1NAME = "l1"; //LX Port 2
    public static final String LEFT2NAME = "l2"; //LX Port 1
    public static final String RIGHT1NAME = "r1";//0A Port 1
    public static final String RIGHT2NAME = "r2";//0A Port 2
    public static final String SHOOT1NAME = "sh1";//PN Port 1
    public static final String SHOOT2NAME = "sh2";//PN Port 2
    public static final String INFEEDNAME = "in"; //2S Port 2
    public static final String GYRONAME = "g"; //Port 4
    public static final String BALLBLOCKNAME = "b";//MO Port 3
    public static final String LEFTPUSHNAME = "lp";//MO Port 1
    public static final String RIGHTPUSHNAME = "rp";//MO Port 2
    public static final String RANGENAME = "r"; //Port 0
    public static final String COLORSIDENAME = "cs"; //Port 1
    public static final String COLORBOTTOMNAME = "cb";//Port 2
    public static final double LEFT_SERVO_OFF_VALUE = .75;
    public static final double LEFT_SERVO_ON_VALUE = .08;
    public static final double RIGHT_SERVO_ON_VALUE = .94;
    public static final double RIGHT_SERVO_OFF_VALUE = .25;
    public static final double BALLBLOCKOPEN = 0;
    public static final double BALLBLOCKCLOSED = 1;

    public static final double POWER1 = .5, DISTANCE1 = 1500;
    public static final double POWER2 = .2, DISTANCE2 = -300;
    public static final double POWER3 = .25, DISTANCE3 = 300;
    public static final double POWER4 = .35, DISTANCE4 = 1500;
    public static final double POWER5 = .2, CM_FROM_WALL_VALUE = 12;
    public static final double POWER6 = -.2, COLOR_READING_FOR_LINE = 4;
    public static final double POWER7 = .2;

    DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, shoot1, shoot2, infeed;
    Servo leftButtonPusher, rightButtonPusher, ballBlock;
    ColorSensor colorSensorOnBottom, colorSensorOnSide;
    ModernRoboticsI2cRangeSensor range;
    ModernRoboticsI2cGyro gyroSensor;

    //Runs op mode
    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontWheel= hardwareMap.dcMotor.get(LEFT1NAME);
        leftBackWheel = hardwareMap.dcMotor.get(LEFT2NAME);
        rightFrontWheel=hardwareMap.dcMotor.get(RIGHT1NAME);
        rightBackWheel= hardwareMap.dcMotor.get(RIGHT2NAME);
        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot1=hardwareMap.dcMotor.get(SHOOT1NAME);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2=hardwareMap.dcMotor.get(SHOOT2NAME);
        infeed=hardwareMap.dcMotor.get(INFEEDNAME);
        infeed.setDirection(DcMotorSimple.Direction.REVERSE);
        ballBlock=hardwareMap.servo.get(BALLBLOCKNAME);
        leftButtonPusher =hardwareMap.servo.get(LEFTPUSHNAME);
        rightButtonPusher =hardwareMap.servo.get(RIGHTPUSHNAME);
        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, RANGENAME);
        colorSensorOnBottom = hardwareMap.colorSensor.get(COLORBOTTOMNAME);
        colorSensorOnSide = hardwareMap.colorSensor.get(COLORSIDENAME);
        colorSensorOnBottom.setI2cAddress(I2cAddr.create8bit(0x4c));
        colorSensorOnSide.setI2cAddress(I2cAddr.create8bit(0x3c));
        gyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, GYRONAME);
        gyroSensor.calibrate();
        while(gyroSensor.isCalibrating()){
            telemetry.addData("Gyro", "Calibrating...");
            telemetry.update();
        }
        telemetry.addData("Gyro", "Calibrated");
        telemetry.addData("raw ultrasonic", range.rawUltrasonic());
        telemetry.update();

        waitForStart();
        resetEncoder(leftFrontWheel);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE1) {
            arcade(POWER1, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        shoot1.setPower(1);
        shoot2.setPower(1);
        infeed.setPower(1);
        ballBlock.setPosition(BALLBLOCKOPEN);
        sleep(2500);
        infeed.setPower(0);
        shoot1.setPower(0);
        shoot2.setPower(0);
        ballBlock.setPosition(BALLBLOCKCLOSED);

        resetEncoder(leftFrontWheel);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) > DISTANCE2) {
            arcade(POWER2, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        resetEncoder(leftFrontWheel);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE3) {
            arcade(0, POWER3, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        resetEncoder(leftFrontWheel);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE4) {
            arcade(POWER4, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        while(range.getDistance(DistanceUnit.CM) > CM_FROM_WALL_VALUE) {
            arcade(0, POWER5, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        while(colorSensorOnBottom.alpha() < 4) {
            arcade(POWER6, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        sleep(500);
        handleColor();
        sleep(500);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);

        while(colorSensorOnBottom.alpha() < COLOR_READING_FOR_LINE) {
            arcade(POWER7, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        sleep(500);
        handleColor();
        sleep(500);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);

    }

    public void handleColor(){
        if(colorSensorOnSide.blue() > colorSensorOnSide.red()){
            rightButtonPusher.setPosition(RIGHT_SERVO_ON_VALUE);
            leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        } else {
            rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
            leftButtonPusher.setPosition(LEFT_SERVO_ON_VALUE);
        }
    }
    public static void resetEncoder(DcMotor m){
        m.setMode(DcMotor.RunMode.RESET_ENCODERS);
        while(m.getCurrentPosition()!=0){
        }//wait
    }
    public void stopMotors(){
        leftFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        rightFrontWheel.setPower(0);
        rightBackWheel.setPower(0);
    }
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

}
