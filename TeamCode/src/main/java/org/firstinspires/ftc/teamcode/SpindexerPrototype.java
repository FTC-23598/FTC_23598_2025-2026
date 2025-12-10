package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Spindexer Manager")

public class SpindexerPrototype extends OpMode {
    private Servo Spindexer;

    @Override
    public void init() {

        Spindexer = hardwareMap.get(Servo.class, "Spindexer");

    }

    @Override
    public void loop() {
        telemetry.addData("Artifact 1: ", "red");
        telemetry.addData("Artifact 2: ", "blue");
        telemetry.addData("Artifact 3: ", "green");

    }

}