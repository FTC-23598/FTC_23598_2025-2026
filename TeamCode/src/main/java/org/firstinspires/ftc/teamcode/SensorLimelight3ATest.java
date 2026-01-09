/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;



@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")
public class SensorLimelight3ATest extends OpMode {

    private Limelight3A limelight;
    private DcMotorEx turretmotor;
    private IMU imu;

    @Override
    public void init() {
        //gets the limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //selects the pipeline version on the limelight
        limelight.pipelineSwitch(1);
        // gets the imu
        imu = hardwareMap.get(IMU.class,"imu");
        //gets how the control hub is mounted to the robot
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        // Initializes the IMU orientation
        imu.initialize((new IMU.Parameters(revHubOrientationOnRobot)));

        turretmotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void start(){
        limelight.start();
    }
    public void loop(){

        //gets the robots yaw pitch and roll
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        // sends the robots yaw to limelight(probably)
        limelight.updateRobotOrientation(orientation.getYaw());
        // updates the result
        LLResult llResult = limelight.getLatestResult();
        // checks if there is a target (with != null) and make sure its valid
        if (llResult != null && llResult.isValid()) {

            //prints the targeting data on the driver hub
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Tl", llResult.getTargetingLatency());

            // robots estimated position on the field
            Pose3D botpose = llResult.getBotpose();
            double angleToTargetDeg = llResult.getTx();

            double TicksPerRev = 28; //placeholder
            double GearRatio1 = 132;//placeholder
            double GearRatio2 = 5;//placeholder

            double currentTicks = turretmotor.getCurrentPosition();

            double launcherAngleDeg = (currentTicks / (TicksPerRev * GearRatio1 * GearRatio2)) * 360;
            double launcherErrorDeg = angleToTargetDeg - launcherAngleDeg;
            telemetry.addData("launcher angle", launcherAngleDeg);
            telemetry.addData("launcher error", launcherErrorDeg);

            if (launcherAngleDeg < -300) launcherAngleDeg = -299;
            if (launcherAngleDeg > 300) launcherAngleDeg = 299;

            double kp = 0.01;
            double motorpower = launcherErrorDeg * kp;
            turretmotor.setPower(motorpower);
            telemetry.update();
        } else {
            turretmotor.setPower(0.0);
            telemetry.addData("Target", "not found");
            telemetry.update();

        }
    }
}