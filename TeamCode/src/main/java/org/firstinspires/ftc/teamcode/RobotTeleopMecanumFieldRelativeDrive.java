/* Copyright (c) 2025 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "Main Robot Drive (it cooks)", group = "Robot")
public class RobotTeleopMecanumFieldRelativeDrive extends OpMode {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    private DcMotor INTAKE;
    private Servo IntakePivotLeft;
    private Servo IntakePivotRight;
    private Servo LiftLeft;
    private Servo LiftRight;
    private DcMotor LAUNCH1;
    private DcMotor LAUNCH2;
    private Servo Spindexer;

    boolean LiftToggle = false;
    boolean LauncherToggle = false;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    ElapsedTime TimerSpindexer = new ElapsedTime(); // Timer for automatically rotating the spindexer
    boolean IsSpindexerSpinning = false; // Checks if spindexer is spinning!

    boolean WasSpindexerSpinning = false; // Checks if spindexer was spinning in the last loop

    @Override
    public void init() { // The initialization defines hardware and how they are configured
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BL");
        backRightDrive = hardwareMap.get(DcMotor.class, "BR");

        INTAKE = hardwareMap.get(DcMotor.class, "INTAKE");
        IntakePivotLeft = hardwareMap.get(Servo.class, "IntakePivotLeft");
        IntakePivotRight = hardwareMap.get(Servo.class, "IntakePivotRight");

        LiftLeft = hardwareMap.get(Servo.class, "LiftLeft");
        LiftRight = hardwareMap.get(Servo.class, "LiftRight");
        LiftLeft.setDirection(Servo.Direction.REVERSE);
        LiftRight.setDirection(Servo.Direction.FORWARD);

        INTAKE.setDirection(DcMotor.Direction.REVERSE);
        IntakePivotLeft.setDirection(Servo.Direction.REVERSE);
        IntakePivotRight.setDirection(Servo.Direction.FORWARD);

        LAUNCH1 = hardwareMap.get(DcMotor.class, "LAUNCH1");
        LAUNCH2 = hardwareMap.get(DcMotor.class, "LAUNCH2");

        Spindexer = hardwareMap.get(Servo.class, "Spindexer");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        LAUNCH1.setDirection(DcMotor.Direction.FORWARD);
        LAUNCH2.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {


        if (gamepad1.right_bumper) {
            IntakeServos(1);
            INTAKE.setPower(1);
            IsSpindexerSpinning = true;

        } else if (gamepad1.left_bumper) {
            IntakeServos(1);
            INTAKE.setPower(-1);
            IsSpindexerSpinning = false;
        } else {
            if (gamepad1.a) {
                INTAKE.setPower(1);
                IntakeServos(0);
                IsSpindexerSpinning = false;
            } else {
                IntakeServos(0.5);
                INTAKE.setPower(0);
                IsSpindexerSpinning = false;
            }
        }

        if (Spindexer.getPosition() > 1) {
            Spindexer.setPosition(0);
        }


        if (IsSpindexerSpinning == true && TimerSpindexer.seconds() > 3) {
            Spindexer.setPosition(Spindexer.getPosition() + 0.3);
            TimerSpindexer.reset();
        }

        if (WasSpindexerSpinning && IsSpindexerSpinning == false) {
            TimerSpindexer.reset();
        }

        WasSpindexerSpinning = IsSpindexerSpinning;

        if (gamepad1.xWasPressed()) {
            if (LiftToggle == true) {
                LiftToggle = false;
            } else {
                LiftToggle = true;
            }


            if (LiftToggle == true) {
                LiftLeft.setPosition(0.5);
                LiftRight.setPosition(0.5);
            } else {
                LiftLeft.setPosition(0);
                LiftRight.setPosition(0);
            }


        }

//        if (gamepad1.dpadUpWasPressed() && ((DcMotorEx) LAUNCH1).getVelocity() <= 0) {
//            LAUNCH1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            LAUNCH2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            LAUNCH1.setPower(1);
//            LAUNCH2.setPower(1);
//            ((DcMotorEx) LAUNCH1).setVelocity(1075);
//            ((DcMotorEx) LAUNCH2).setVelocity(1075);
//        } else if (gamepad1.dpadUpWasPressed() && ((DcMotorEx) LAUNCH1).getVelocity() > 0) {
//            LAUNCH1.setPower(0);
//            LAUNCH2.setPower(0);
//            ((DcMotorEx) LAUNCH1).setVelocity(0);
//            ((DcMotorEx) LAUNCH2).setVelocity(0);
//        }

        if (gamepad1.dpadUpWasPressed()) {
            if (LauncherToggle == true) {
                LauncherToggle = false;
            } else {
                LauncherToggle = true;
            }


            if (LauncherToggle == true) {
                LAUNCH1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LAUNCH2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LAUNCH1.setPower(1);
                LAUNCH2.setPower(1);
                ((DcMotorEx) LAUNCH1).setVelocity(1075);
                ((DcMotorEx) LAUNCH2).setVelocity(1075);
            } else {
                LAUNCH1.setPower(0);
                LAUNCH2.setPower(0);
                ((DcMotorEx) LAUNCH1).setVelocity(0);
                ((DcMotorEx) LAUNCH2).setVelocity(0);
            }


        }

        if (gamepad1.dpad_left) {
            Spindexer.setPosition(0);
        } else if (gamepad1.dpad_down) {
            Spindexer.setPosition(0.41);
        } else if (gamepad1.dpad_right) {
            Spindexer.setPosition(0.82);
        }
        if (gamepad1.aWasReleased()) {
            Spindexer.setPosition(Spindexer.getPosition() + 0.41);
            if (Spindexer.getPosition() >= 0.83) {
                Spindexer.setPosition(0);
            }
        }

        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        telemetry.addData("Velocity", ((DcMotorEx) LAUNCH1).getVelocity());
        telemetry.addData("Spindexer Position", Spindexer.getPosition());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
        telemetry.update();
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    private void IntakeServos(double position) {
        IntakePivotLeft.setPosition(position);
        IntakePivotRight.setPosition(position);
    }
}
