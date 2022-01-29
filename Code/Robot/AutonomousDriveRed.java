package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.ArrayList;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="AutonomousDriveRed", group="Pushbot")


public class AutonomousDriveRed extends LinearOpMode {
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor arm = null;
    private Servo hand = null;
    private DcMotor wheel = null;
    private ElapsedTime runtime = new ElapsedTime();
    public void setPowers(double direction, double speed, double rotation){
        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;


        double forward = Math.cos(direction)*speed;
        double side  =  Math.sin(direction)*speed;

        rightBackPower    = Range.clip((forward + side + rotation), -1.0, 1.0);
        leftBackPower   = Range.clip((forward - side - rotation), -1.0, 1.0);
        rightFrontPower    = Range.clip((forward - side + rotation), -1.0, 1.0);
        leftFrontPower   = Range.clip((forward + side - rotation), -1.0, 1.0);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);

    }
    @Override
    public void runOpMode() {
//        ArrayList<double[]> Instructions = new ArrayList<double[]>();
//        double instruction1[] = {1, 2, 3};
//        Instructions.add(instruction1);
        leftBack  = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        arm = hardwareMap.get(DcMotor.class, "arm");
        hand = hardwareMap.get(Servo.class, "hand");
        wheel = hardwareMap.get(DcMotor.class, "wheel");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        wheel.setDirection(DcMotor.Direction.FORWARD);
        runtime.reset();
        waitForStart();
        double startTime = getRuntime();

        hand.setPosition(0.75);
        sleep(1250);
        arm.setPower(-1);

        //Diagonal
        leftFront.setPower(0.5);
        leftBack.setPower(-0.1);
        rightFront.setPower(-0.1);
        rightBack.setPower(0.5);
        sleep(800);
        arm.setPower(0);
        sleep(800);

        //Drop
        hand.setPosition(0.5);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(500);

        //Diagonal back
        leftFront.setPower(-0.5);
        leftBack.setPower(0.1);
        rightFront.setPower(0.1);
        rightBack.setPower(-0.5);
        sleep(1600);

        //Forward
        leftFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightFront.setPower(0.5);
        rightBack.setPower(0.5);
        sleep(250);

        //Left
        leftFront.setPower(-0.25);
        rightFront.setPower(0.25);
        leftBack.setPower(0.25);
        rightBack.setPower(-0.25);
        sleep(2200);

        //Stopish
        leftFront.setPower(-0.15);
        rightFront.setPower(0.15);
        leftBack.setPower(0.15);
        rightBack.setPower(-0.15);

        //Wheel movement
        wheel.setPower(0.5);
        sleep(4000);


        //Forward
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
        sleep(750);

        //Left
        leftFront.setPower(-0.25);
        rightFront.setPower(0.25);
        leftBack.setPower(0.25);
        rightBack.setPower(-0.25);
        sleep(300);


        stop();
    }

}