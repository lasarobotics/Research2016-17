/*
ADB guide can be found at:
https://ftcprogramming.wordpress.com/2015/11/30/building-ftc_app-wirelessly/
*/
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public static final double MAXINFEEDPOWER = 1;
    public static final double MAXOUTFEEDPOWER = -1;
    public static final double INFEEDOFFPOWER = 0;
    public static final double SERVOINCREMENTVALUE = .02;
    public static final double LEFTSERVOMAXVALUE = 1;
    public static final double LEFTSERVOMINVALUE = 0;
    public static final double RIGHTSERVOMAXVALUE = 1;
    public static final double RIGHTSERVOMINVALUE = 0;

    //GOOD VALUES
    public enum GOING {
        IN, OUT, NOTGOING
    }
    public GOING INFEEDSTATUS = GOING.NOTGOING;
    public boolean RECENTLEFTBUMPERVALUE = false;
    public boolean RECENTRIGHTBUMPERVALUE = false;
    public static final double SLOWDOWNVALUE = 5;
    public static final double TRIGGERTHRESHOLD = .2;
    public static final double ACCEPTINPUTTHRESHOLD = .15;
    public static final double SHOOTERMAXVALUE = 1;
    public static final double SHOOTERMINVALUE = 0;
    public static final double SHOOTERTHRESHOLD = .2;
    public static final double SHOOTERLOWERRATE = 0;
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


    DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, shoot1, shoot2, infeed;
    Servo leftButtonPusher, rightButtonPusher, ballBlock;
    ColorSensor colorSensorOnSide, colorSensorOnBottom;
    ModernRoboticsI2cRangeSensor range;
    @Override
    public void init() {
        leftFrontWheel =hardwareMap.dcMotor.get(LEFT1NAME);
        leftBackWheel =hardwareMap.dcMotor.get(LEFT2NAME);
        rightFrontWheel =hardwareMap.dcMotor.get(RIGHT1NAME);
        rightBackWheel =hardwareMap.dcMotor.get(RIGHT2NAME);
        rightFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
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
        colorSensorOnSide = hardwareMap.colorSensor.get(COLORSIDENAME);
        colorSensorOnBottom = hardwareMap.colorSensor.get(COLORBOTTOMNAME);
    }

    @Override
    public void loop() {

/*
  _        __              _
 (_)      / _|            | |
  _ _ __ | |_ ___  ___  __| |
 | | '_ \|  _/ _ \/ _ \/ _` |
 | | | | | ||  __/  __/ (_| |
 |_|_| |_|_| \___|\___|\__,_|
*/
        if(RECENTLEFTBUMPERVALUE && !gamepad2.left_bumper){
            INFEEDSTATUS =(INFEEDSTATUS == GOING.IN) ? GOING.NOTGOING : GOING.IN;
        } else if (RECENTRIGHTBUMPERVALUE && !gamepad2.right_bumper){
            INFEEDSTATUS =(INFEEDSTATUS == GOING.OUT) ? GOING.NOTGOING : GOING.OUT;
        }
        //Ternary Operations used to toggle INFEEDSTATUS
        switch(INFEEDSTATUS){
            case IN:
                infeed.setPower(MAXINFEEDPOWER);
                break;
            case OUT:
                infeed.setPower(MAXOUTFEEDPOWER);
                break;
            case NOTGOING:
                infeed.setPower(INFEEDOFFPOWER);
            default:
                infeed.setPower(INFEEDOFFPOWER);
        }
        RECENTLEFTBUMPERVALUE = gamepad2.left_bumper;
        RECENTRIGHTBUMPERVALUE = gamepad2.right_bumper;

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
        double inputC = Math.abs(gamepad1.right_stick_x)> ACCEPTINPUTTHRESHOLD ? -gamepad1.right_stick_x: 0;

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

        setServo(leftButtonPusher, gamepad2.a, gamepad2.y, SERVOINCREMENTVALUE, LEFTSERVOMAXVALUE, LEFTSERVOMINVALUE);
        setServo(rightButtonPusher, gamepad2.x, gamepad2.b, SERVOINCREMENTVALUE, RIGHTSERVOMAXVALUE, RIGHTSERVOMINVALUE);

/*
      _                 _
     | |               | |
  ___| |__   ___   ___ | |_
 / __| '_ \ / _ \ / _ \| __|
 \__ \ | | | (_) | (_) | |_
 |___/_| |_|\___/ \___/ \__|
*/
        shoot((gamepad1.right_bumper || gamepad1.left_bumper), ballBlock, shoot1, shoot2,
                BLOCKSERVOOPENVALUE, BLOCKSERVOCLOSEDVALUE,
                SHOOTERLOWERRATE, SHOOTERMAXVALUE, SHOOTERMINVALUE, SHOOTERTHRESHOLD);
        //The shooter will shoot at full power, will slow down by diving it's power by 1.5, and will stop completely once it's power is lower than .2.
        //This will hopefully help us to avoid breakage of the gearbox.


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
        telemetry.addData("leftServo", leftButtonPusher.getPosition());
        telemetry.addData("rightServo", rightButtonPusher.getPosition());
        telemetry.addData("Controller LeftY", gamepad1.left_stick_y);
        telemetry.addData("Controller LeftX", gamepad1.left_stick_x);
        telemetry.addData("Controller RightY", gamepad1.right_stick_y);
        telemetry.addData("Infeed", INFEEDSTATUS == GOING.IN ? "Forwards" : INFEEDSTATUS == GOING.OUT ? "Backwards" : "None");
        //Ternary, basically it just outputs the Infeed state.
        telemetry.update();
        //Servos can be set to go between 1 and 0, and will be incremented by 2% of their range each cycle.
        //Pressing the 'a', 'b','x', or 'y' button will increment the servo.


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
