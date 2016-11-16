/*
ADB guide can be found at:
https://ftcprogramming.wordpress.com/2015/11/30/building-ftc_app-wirelessly/
*/
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

/**
 * Created by Ethan Schaffer on 10/31/2016.
 */
@TeleOp(name="Driver Controlled", group="TeleOp")
public class TeleOpFinal extends OpMode {

    //TWEAKING VALUES
    public static final double BLOCKSERVOOPENVALUE = 0;
    public static final double BLOCKSERVOCLOSEDVALUE = 1;
    public static final double LEFT_SERVO_OFF_VALUE = .25;
    public static final double LEFT_SERVO_ON_VALUE = .94;
    public static final double RIGHT_SERVO_ON_VALUE = 1;
    public static final double RIGHT_SERVO_OFF_VALUE = 0;

    public static final double MAXINFEEDPOWER = 1;
    public static final double MAXOUTFEEDPOWER = -1;

    //GOOD VALUES
    public enum INFEEDSTATE {
        IN, OUT, NOTGOING
    }
    public enum SHOOTERSTATE {
        SHOOTING, BACK, NOTSHOOTING
    }
    public enum SERVOSTATE {
        ON, OFF
    }
    public INFEEDSTATE INFEEDSTATUS = INFEEDSTATE.NOTGOING;
    public SHOOTERSTATE SHOOTERSTATUS = SHOOTERSTATE.NOTSHOOTING;
    public SERVOSTATE RIGHTSERVOSTATE = SERVOSTATE.OFF;
    public SERVOSTATE LEFTSERVOSTATE = SERVOSTATE.OFF;

    public boolean RECENT_A_BUTTON = false;
    public boolean RECENT_Y_BUTTON = false;
    public boolean RECENT_X_BUTTON = false;
    public boolean RECENT_B_BUTTON = false;
    public boolean RECENT_LB = false;
    public boolean RECENT_RB = false;

    public boolean RECENT_TRIGGER = false;

    public static final double SLOWDOWNVALUE = 5;
    public static final double TRIGGERTHRESHOLD = .2;
    public static final double ACCEPTINPUTTHRESHOLD = .15;
    public static final double SHOOTERMAXVALUE = 1;

    public static final String LEFT1NAME = "l1"; //LX Port 2
    public static final String LEFT2NAME = "l2"; //LX Port 1
    public static final String RIGHT1NAME = "r1";//0A Port 1
    public static final String RIGHT2NAME = "r2";//0A Port 2
    public static final String SHOOT1NAME = "sh1";//PN Port 1
    public static final String SHOOT2NAME = "sh2";//PN Port 2
    public static final String INFEEDNAME = "in"; //2S Port 2
    public static final String BALLBLOCKNAME = "b";//MO Port 3
    public static final String LEFTPUSHNAME = "lp";//MO Port 1
    public static final String RIGHTPUSHNAME = "rp";//MO Port 2
    public static final String RANGENAME = "r"; //Port 0
    public static final String COLORSIDENAME = "cs"; //Port 1
    public static final String COLORBOTTOMNAME = "cb";//Port 2


    DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, shoot1, shoot2, infeed;
    Servo leftButtonPusher, rightButtonPusher, ballBlock;
    ColorSensor colorSensorOnSide, colorSensorOnBottom;
    ModernRoboticsI2cRangeSensor range;

    @Override
    public void init() {
        leftFrontWheel = hardwareMap.dcMotor.get(LEFT1NAME);
        leftBackWheel = hardwareMap.dcMotor.get(LEFT2NAME);
        rightFrontWheel = hardwareMap.dcMotor.get(RIGHT1NAME);
        rightBackWheel = hardwareMap.dcMotor.get(RIGHT2NAME);
        rightFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        shoot1 = hardwareMap.dcMotor.get(SHOOT1NAME);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2 = hardwareMap.dcMotor.get(SHOOT2NAME);

        infeed = hardwareMap.dcMotor.get(INFEEDNAME);
//        infeed.setDirection(DcMotorSimple.Direction.REVERSE); // At Kieran's Request

        leftButtonPusher = hardwareMap.servo.get(LEFTPUSHNAME);
        rightButtonPusher = hardwareMap.servo.get(RIGHTPUSHNAME);
        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);

//        ballBlock = hardwareMap.servo.get(BALLBLOCKNAME);

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, RANGENAME);
        colorSensorOnBottom = hardwareMap.colorSensor.get(COLORBOTTOMNAME);
        colorSensorOnBottom = hardwareMap.colorSensor.get(COLORBOTTOMNAME);
        colorSensorOnBottom.setI2cAddress(I2cAddr.create8bit(0x4c));
        colorSensorOnBottom.enableLed(true);

        colorSensorOnSide = hardwareMap.colorSensor.get(COLORSIDENAME);
        colorSensorOnSide = hardwareMap.colorSensor.get(COLORSIDENAME);
        colorSensorOnSide.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensorOnSide.enableLed(false);
    }

    @Override
    public void loop() {

        /*
              _                 _
             | |               | |
          ___| |__   ___   ___ | |_
         / __| '_ \ / _ \ / _ \| __|
         \__ \ | | | (_) | (_) | |_
         |___/_| |_|\___/ \___/ \__|
        */
        if(RECENT_LB && !gamepad2.left_bumper){
            SHOOTERSTATUS =(SHOOTERSTATUS == SHOOTERSTATE.SHOOTING) ? SHOOTERSTATE.NOTSHOOTING : SHOOTERSTATE.SHOOTING;
        } else if (RECENT_RB && !gamepad2.right_bumper){
            SHOOTERSTATUS =(SHOOTERSTATUS == SHOOTERSTATE.BACK) ? SHOOTERSTATE.NOTSHOOTING : SHOOTERSTATE.BACK;
        }
        //Ternary Operations used to toggle SHOOTERSTATUS
        switch(SHOOTERSTATUS){
            case SHOOTING:
                shoot1.setPower(SHOOTERMAXVALUE);
                shoot2.setPower(SHOOTERMAXVALUE);
//                ballBlock.setPosition(BLOCKSERVOOPENVALUE);
                break;
            case BACK:
                shoot1.setPower(-SHOOTERMAXVALUE);
                shoot2.setPower(-SHOOTERMAXVALUE);
//                ballBlock.setPosition(BLOCKSERVOOPENVALUE);
                break;
            default:
                shoot1.setPower(0);
                shoot2.setPower(0);
//                ballBlock.setPosition(BLOCKSERVOCLOSEDVALUE);
        }
        RECENT_LB = gamepad2.left_bumper;
        RECENT_RB = gamepad2.right_bumper;
        /*
          _        __              _
         (_)      / _|            | |
          _ _ __ | |_ ___  ___  __| |
         | | '_ \|  _/ _ \/ _ \/ _` |
         | | | | | ||  __/  __/ (_| |
         |_|_| |_|_| \___|\___|\__,_|
        */

        double power = Math.abs(gamepad2.left_stick_y) > ACCEPTINPUTTHRESHOLD ? gamepad2.left_stick_y : 0;
        double modifier = Math.abs(gamepad2.right_stick_y) > ACCEPTINPUTTHRESHOLD ? 1.1-Math.abs(gamepad2.right_stick_y) : 1;
        if(power*modifier > MAXINFEEDPOWER){
            infeed.setPower(MAXINFEEDPOWER);
        } else if(power*modifier < MAXOUTFEEDPOWER){
            infeed.setPower(MAXOUTFEEDPOWER);
        } else {
            infeed.setPower(power*modifier);
        }

        /*
              _      _
             | |    (_)
           __| |_ __ ___   _____
          / _` | '__| \ \ / / _ \
         | (_| | |  | |\ V /  __/
          \__,_|_|  |_| \_/ \___|
         */
        double inputX = Math.abs(gamepad1.left_stick_y) > ACCEPTINPUTTHRESHOLD ? -gamepad1.left_stick_y : 0;
        double inputY = Math.abs(gamepad1.left_stick_x) > ACCEPTINPUTTHRESHOLD ? gamepad1.left_stick_x : 0;
        double inputC = Math.abs(gamepad1.right_stick_x)> ACCEPTINPUTTHRESHOLD ? gamepad1.right_stick_x: 0;

        double BIGGERTRIGGER = gamepad1.left_trigger > gamepad1.right_trigger ? gamepad1.left_trigger : gamepad1.right_trigger;
        //Ternary, the larger trigger value is set to the value BIGGERTRIGGER

        if(BIGGERTRIGGER > TRIGGERTHRESHOLD){
                inputX /= SLOWDOWNVALUE*BIGGERTRIGGER;
                inputY /= SLOWDOWNVALUE*BIGGERTRIGGER;
                inputC /= SLOWDOWNVALUE*BIGGERTRIGGER;
        }
        //Use the larger trigger value to scale down the inputs.
        arcadeMecanum(inputX, inputY, inputC, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);

        /*
          ___  ___ _ ____   _____
         / __|/ _ \ '__\ \ / / _ \
         \__ \  __/ |   \ V / (_) |
         |___/\___|_|    \_/ \___/
         */

        if(RECENT_X_BUTTON && !gamepad2.x){
            RIGHTSERVOSTATE = (RIGHTSERVOSTATE == SERVOSTATE.OFF ? SERVOSTATE.ON : SERVOSTATE.OFF);
        }
        switch (RIGHTSERVOSTATE){
            case ON:
                rightButtonPusher.setPosition(RIGHT_SERVO_ON_VALUE);
                break;
            case OFF:
                rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        }
        RECENT_X_BUTTON = gamepad2.x;

        if(RECENT_B_BUTTON && !gamepad2.b){
            LEFTSERVOSTATE = (LEFTSERVOSTATE == SERVOSTATE.OFF ? SERVOSTATE.ON : SERVOSTATE.OFF);
        }
        switch (LEFTSERVOSTATE){
            case ON:
                leftButtonPusher.setPosition(LEFT_SERVO_ON_VALUE);
                break;
            case OFF:
                leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        }
        RECENT_B_BUTTON = gamepad2.b;

//        setServo(leftButtonPusher, gamepad2.a, gamepad2.y, SERVOINCREMENTVALUE, LEFT_SERVO_ON_VALUE, LEFT_SERVO_ON_VALUE);
//        setServo(rightButtonPusher, gamepad2.x, gamepad2.b, SERVOINCREMENTVALUE, RIGHT_SERVO_ON_VALUE, RIGHT_SERVO_OFF_VALUE);



        /*
          _       _                     _
         | |     | |                   | |
         | |_ ___| | ___ _ __ ___   ___| |_ _ __ _   _
         | __/ _ \ |/ _ \ '_ ` _ \ / _ \ __| '__| | | |
         | ||  __/ |  __/ | | | | |  __/ |_| |  | |_| |
          \__\___|_|\___|_| |_| |_|\___|\__|_|   \__, |
                                                  __/ |
                                                 |___/
        */
//        telemetry.addData("LeftY / Y Power", gamepad1.left_stick_y + " / " + inputY);/
//        telemetry.addData("LeftX / X Power", gamepad1.left_stick_x + " / " + inputX);
//        telemetry.addData("RightY / Rot Power", gamepad1.right_stick_y + " / " + inputC);
        telemetry.addData("LF", leftFrontWheel.getPower());
        telemetry.addData("LB", leftBackWheel.getPower());
        telemetry.addData("RF", rightFrontWheel.getPower());
        telemetry.addData("RB", rightBackWheel.getPower());

        telemetry.addData("Infeed", infeed.getPower() > .1 ? "IN" : infeed.getPower() < -.1 ? "OUT" : "OFF");
        telemetry.addData("Shooter", SHOOTERSTATUS == SHOOTERSTATE.SHOOTING ? "Shooting" : "Not Shooting");
        telemetry.addData("Right Servo", RIGHTSERVOSTATE == SERVOSTATE.ON ? "On" : "Off");
        telemetry.addData("Left Servo", LEFTSERVOSTATE == SERVOSTATE.ON ? "On" : "Off");
        //Ternary, basically it just outputs the Infeed state.

        telemetry.update();


    }

    // y - forwards
    // x - side
    // c - rotation
    public static void arcadeMecanum(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
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
    public static void setServo(Servo s, boolean a, boolean b, double inc, double highVal, double lowVal){
        if(a){
            if(s.getPosition() < highVal)
                s.setPosition(s.getPosition()+inc);
        } else if(b){
            if(s.getPosition() > lowVal)
                s.setPosition(s.getPosition()-inc);
        }
    }
    public static void shoot(boolean condition, Servo s, DcMotor m1, DcMotor m2, double openVal, double closedVal, double rampVal, double maxPower, double minPower, double threshold){
        if(condition){
            s.setPosition(openVal);
            m1.setPower(maxPower);
            m2.setPower(maxPower);
        } else {
            s.setPosition(closedVal);
            if(m1.getPower() > threshold){
                m1.setPower(m1.getPower()/rampVal);
                m2.setPower(m2.getPower()/rampVal);
            } else {
                m1.setPower(minPower);
                m2.setPower(minPower);
            }
        }

    }
}
