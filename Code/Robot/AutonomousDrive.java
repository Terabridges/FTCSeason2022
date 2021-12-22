package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.ArrayList;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="AutonomousDrive", group="Pushbot")


public class AutonomousDrive extends LinearOpMode {
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
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
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        runtime.reset();
        waitForStart();
        setPowers(0*Math.PI, 0.2, 0);
        while(true){
            telemetry.addData("I am", runtime);

        }
    }

}