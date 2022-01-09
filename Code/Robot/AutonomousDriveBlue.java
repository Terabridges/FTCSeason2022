package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="AutonomousDriveBlue", group="Pushbot")


public class AutonomousDriveBlue extends LinearOpMode {
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
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
        wheel = hardwareMap.get(DcMotor.class, "wheel");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        wheel.setDirection(DcMotor.Direction.FORWARD);
        runtime.reset();
        waitForStart();
        double startTime = getRuntime();

        //Backward
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(-0.5);
        while(opModeIsActive() && getRuntime() < startTime + 2){
            telemetry.addData("I am", runtime);
        }

        //Stop
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        //Wheel movement
        wheel.setPower(-0.25);
        while(opModeIsActive() && getRuntime() < startTime + 9){
            telemetry.addData("I am", runtime);
        }


        //Right
        leftFront.setPower(0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(0.5);
        while(opModeIsActive() && getRuntime() < startTime + 10.2){
            telemetry.addData("I am", runtime);
        }

        //Forwards
        leftFront.setPower(-0.25);
        rightFront.setPower(-0.25);
        leftBack.setPower(-0.25);
        rightBack.setPower(-0.25);
        while(opModeIsActive() && getRuntime() < startTime + 10.55){
            telemetry.addData("I am", runtime);
        }


        stop();
    }

}