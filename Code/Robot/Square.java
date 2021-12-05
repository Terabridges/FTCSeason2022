/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Square", group="Iterative Opmode")
//@Disabled
public class Square extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor wheel = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBack  = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        wheel = hardwareMap.get(DcMotor.class, "wheel");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        rightBackLastDistance = rightBack.getCurrentPosition();
        leftBackLastDistance = leftBack.getCurrentPosition();
        rightFrontLastDistance = rightFront.getCurrentPosition();
        leftFrontLastDistance = leftFront.getCurrentPosition();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    double rightBackScaling = 1;
    double leftBackScaling = 1;
    double rightFrontScaling = 1;
    double leftFrontScaling = 1;
    float rightBackLastDistance;
    float leftBackLastDistance;
    float rightFrontLastDistance;
    float leftFrontLastDistance;
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;

        double forward = -gamepad1.left_stick_y;
        double side  =  gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;
        rightBackPower = 0.53;
        rightFrontPower = 0.5;
        leftBackPower = -0.5;
        leftFrontPower = -0.5;
        rightBackPower    = Range.clip((forward + side + rotation), -1.0, 1.0) ;
        leftBackPower   = Range.clip((forward - side - rotation), -1.0, 1.0) ;
        rightFrontPower    = Range.clip((forward - side + rotation), -1.0, 1.0) ;
        leftFrontPower   = Range.clip((forward + side - rotation), -1.0, 1.0) ;

        leftBack.setPower(leftBackPower * leftBackScaling);
        rightBack.setPower(rightBackPower * rightBackScaling);
        leftFront.setPower(leftFrontPower * leftFrontScaling);
        rightFront.setPower(rightFrontPower * rightFrontScaling);


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Left Front", (leftFrontLastDistance - leftFront.getCurrentPosition()));
        telemetry.addData("Left Back", (leftBackLastDistance - leftBack.getCurrentPosition()));
        telemetry.addData("Right Front", (rightFrontLastDistance - rightFront.getCurrentPosition()));
        telemetry.addData("Right Back", (rightBackLastDistance - rightBack.getCurrentPosition()));



        rightBackLastDistance = rightBack.getCurrentPosition();
        leftBackLastDistance = leftBack.getCurrentPosition();
        rightFrontLastDistance = rightFront.getCurrentPosition();
        leftFrontLastDistance = leftFront.getCurrentPosition();

        if (gamepad1.dpad_left) {
            wheel.setPower(0.5);
        } else if (gamepad1.dpad_right) {
            wheel.setPower(-0.5);
        } else if (gamepad1.dpad_up) {
            wheel.setPower(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
