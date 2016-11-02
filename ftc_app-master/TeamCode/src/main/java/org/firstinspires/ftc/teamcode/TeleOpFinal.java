package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

/**
 * Created by Ethan Schaffer on 10/31/2016.
 */
@TeleOp(name="Driver Controlled", group="TeleOp")
public class TeleOpFinal extends OpMode {
    public static final double THRESHOLD = .1;
    public static final double BLOCKSERVOOPENVALUE = 1;
    public static final double BLOCKSERVOCLOSEDVALUE = 0;
    public static final double MAXINFEEDPOWER = .5;
    public static final double MAXOUTFEEDPOWER = -1;
    public static final double MININFEEDPOWER = 0;
    public static final double SERVOINCREMENTVALUE = .02;
    public static final double LEFTSERVOMAXVALUE = 1;
    public static final double LEFTSERVOMINVALUE = 0;
    public static final double RIGHTSERVOMAXVALUE = 1;
    public static final double RIGHTSERVOMINVALUE = 0;

    public static final double SHOOTERMAXVALUE = 1;
    public static final double SHOOTERMINVALUE = 0;
    public static final double SHOOTERTHRESHOLD = .2;
    public static final double SHOOTERLOWERRATE = 0;

    public static final String LEFT1NAME = "l2";
    public static final String LEFT2NAME = "l1";
    public static final String RIGHT1NAME = "r1";
    public static final String RIGHT2NAME = "r2";
    public static final String SHOOT1NAME = "sh1";
    public static final String SHOOT2NAME = "sh2";
    public static final String INFEEDNAME = "in";
    public static final String BALLBLOCKNAME = "b";
    public static final String LEFTPUSHNAME = "lp";
    public static final String RIGHTPUSHNAME = "rp";


    DcMotor left1, left2, right1, right2, shoot1, shoot2, infeed;
    Servo leftPush, rightPush, ballBlock;
    boolean infeedOn = false;
    @Override
    public void init() {
        left1 =hardwareMap.dcMotor.get(LEFT1NAME);
        left2 =hardwareMap.dcMotor.get(LEFT2NAME);
        right1=hardwareMap.dcMotor.get(RIGHT1NAME);
        right2=hardwareMap.dcMotor.get(RIGHT2NAME);
        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot1=hardwareMap.dcMotor.get(SHOOT1NAME);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2=hardwareMap.dcMotor.get(SHOOT2NAME);
        infeed=hardwareMap.dcMotor.get(INFEEDNAME);
        infeed.setDirection(DcMotorSimple.Direction.REVERSE);
        ballBlock=hardwareMap.servo.get(BALLBLOCKNAME);
        leftPush =hardwareMap.servo.get(LEFTPUSHNAME);
        rightPush=hardwareMap.servo.get(RIGHTPUSHNAME);
        leftPush.setPosition(LEFTSERVOMAXVALUE);
        rightPush.setPosition(RIGHTSERVOMAXVALUE);
    }

    @Override
    public void loop() {
        if(gamepad2.left_bumper) {
            infeed.setPower(MAXINFEEDPOWER);
        } else if(gamepad2.right_bumper) {
            infeed.setPower(MAXOUTFEEDPOWER);
        } else {
            infeed.setPower(MININFEEDPOWER);
        }
        arcadeMecanum(Math.abs(gamepad1.left_stick_y) > THRESHOLD ? -gamepad1.left_stick_y : 0,
                    Math.abs(gamepad1.left_stick_x) > THRESHOLD ? gamepad1.left_stick_x : 0,
                    Math.abs(gamepad1.right_stick_x)> THRESHOLD ? -gamepad1.right_stick_x: 0,
                    left1, right1, left2, right2);
        //The above code uses Ternary Operations to save space. The arcade() method can be found below.

/*        left1.setPower(0);
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);

        if(gamepad1.a)
            left1.setPower(1); //l2
        if(gamepad1.b)
            left2.setPower(1);
        if(gamepad1.x)
            right1.setPower(1);
        if(gamepad1.y)
            right2.setPower(1);
*/
        setServo(leftPush, gamepad2.a, gamepad2.b, SERVOINCREMENTVALUE, LEFTSERVOMAXVALUE, LEFTSERVOMINVALUE);
        setServo(rightPush, gamepad2.x, gamepad2.y, SERVOINCREMENTVALUE, RIGHTSERVOMAXVALUE, RIGHTSERVOMINVALUE);
        telemetry.addData("left", leftPush.getPosition());
        telemetry.addData("left", leftPush.getPosition());
        telemetry.update();
        //Servos can be set to go between 1 and 0, and will be incremented by 2% of their range each cycle.
        //Pressing the 'a', 'b','x', or 'y' button will increment the servo.

        shoot((gamepad1.right_bumper || gamepad1.left_bumper), ballBlock, shoot1, shoot2,
                BLOCKSERVOOPENVALUE, BLOCKSERVOCLOSEDVALUE,
                SHOOTERLOWERRATE, SHOOTERMAXVALUE, SHOOTERMINVALUE, SHOOTERTHRESHOLD);
        //The shooter will shoot at full power, will slow down by diving it's power by 1.5, and will stop completely once it's power is lower than .2.
        //This will hopefully help us to avoid breakage of the gearbox.

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
