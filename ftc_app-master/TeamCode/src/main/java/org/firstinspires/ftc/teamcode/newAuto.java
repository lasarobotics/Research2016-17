package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name="Auto!", group="Autonomous")
public class newAuto extends LinearOpMode {
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

    public static final double POWER1 = -.5, DISTANCE1 = 1900, DISTANCE1PART2 = 1000;           //Distance Forwards before shot

    public static final long SHOTTIMEINMILLISECONDS = 1000;              //How long we shoot the ball
//    public static final long SHOTTIMEINMILLISECONDS = 4500;              //How long we shoot the ball
//UPDATE THIS


    //TODO: Make the numbers actually count out our steps
    //Don't do this until it all works, and then do it as a refactor
    //Once the refactor is done, make a pretty 'map'
    public static final double POWER2 = .2, DISTANCE2 = 500;             //Backup Distance
    public static final double POWER2PART2 = .25, GYRO2READINGTARGET = 37; //Turn Reading
    public static final double POWER3 = -.5, DISTANCE3 = 2750;            //Strafe Distance
    public static final double POWER3PART2 = -.25, GYRO3READINGTARGET = 0; //Turn Reading
    public static final double POWER4 = -.35, DISTANCE4 = 2500;          //Distance Forwards to get to strafing position
    public static final double POWER5 = .45, CM_FROM_WALL_VALUE = 8;    //Strafe to between the white lines.
    public static final double POWER6 = .2, COLOR_READING_FOR_LINE = 4; //Backwards to beacon #1
    public static final double POWER7 = -.2; //Forwards to Beacon #2
    public  static  final double POWER7PART2 = .2, GYRO7READINGTARGET = 45;
    public static final double POWER8 = .5, DISTANCE8 = 2500; //Distance to Ball


    DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, shoot1, shoot2, infeed;
    Servo leftButtonPusher, rightButtonPusher, ballBlock;
    ColorSensor colorSensorOnBottom, colorSensorOnSide;
    ModernRoboticsI2cRangeSensor range;
    ModernRoboticsI2cGyro gyroSensor;

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
        leftButtonPusher = hardwareMap.servo.get(LEFTPUSHNAME);
        rightButtonPusher = hardwareMap.servo.get(RIGHTPUSHNAME);

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

        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        ballBlock.setPosition(BALLBLOCKCLOSED);

        waitForStart();
        resetEncoder(leftFrontWheel);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE1) {
            arcade(POWER1, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        ballBlock.setPosition(BALLBLOCKOPEN);
        sleep(50);
        shoot1.setPower(1);
        shoot2.setPower(1);
        sleep(100);
        infeed.setPower(1);
        sleep(SHOTTIMEINMILLISECONDS);
        infeed.setPower(0);
        shoot1.setPower(0);
        shoot2.setPower(0);
        ballBlock.setPosition(BALLBLOCKCLOSED);

/*
        resetEncoder(leftFrontWheel);
        sleep(50);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE1PART2) {
            arcade(POWER1, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();
*/

        //Back Up
        resetEncoder(leftFrontWheel);
        sleep(50);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE2) {
            arcade(POWER2, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        //TURN
        while(gyroSensor.getIntegratedZValue() < GYRO2READINGTARGET){
            arcade(0, 0, POWER2PART2, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        //Forwards
        resetEncoder(leftFrontWheel);
        sleep(50);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE3) {
            arcade(POWER3, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        //TURN BACK
        int temp = gyroSensor.getIntegratedZValue()+5;
        while((gyroSensor.getIntegratedZValue() > GYRO3READINGTARGET) && (gyroSensor.getIntegratedZValue() < temp)) {
            arcade(0, 0, POWER3PART2, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        //strafe to wall
        sleep(50);
        while(range.getDistance(DistanceUnit.CM) > CM_FROM_WALL_VALUE) {
            telemetry.addData("Dist", range.getDistance(DistanceUnit.CM));
            telemetry.addData("Targ", CM_FROM_WALL_VALUE);
            telemetry.update();
            arcade(0, POWER5, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

/*
        resetEncoder(leftFrontWheel);
        sleep(50);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE4) {
            arcade(POWER4, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        sleep(50);
        while(range.getDistance(DistanceUnit.CM) > CM_FROM_WALL_VALUE) {
            arcade(0, POWER5, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();
*/
        //backwards to back color sensor
        sleep(50);
        while(colorSensorOnBottom.alpha() < 4) {
            arcade(POWER6, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        sleep(100);
        //press buttons
        handleColor();
        sleep(100);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);

        ///forwards to front color sensor
        while(colorSensorOnBottom.alpha() < COLOR_READING_FOR_LINE) {
            arcade(POWER7, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        sleep(100);
        //press buttons
        handleColor();
        sleep(100);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);

        //TURN BACK
        while(gyroSensor.getIntegratedZValue() < GYRO7READINGTARGET){
            arcade(0, 0, POWER7PART2, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        //Back Up
        resetEncoder(leftFrontWheel);
        sleep(50);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE8) {
            arcade(POWER8, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();


    }

    public void handleColor() {
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
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
