package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Limelight 3A Test")
@Config
public class Limelight extends LinearOpMode {

    private Limelight3A limelight;


    @Override
    public void runOpMode() {

        // Get Limelight from Robot Configuration
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Select pipeline 0
        limelight.pipelineSwitch(0);

        telemetry.addLine("Limelight ready");
        telemetry.update();

        waitForStart();

        // Start Limelight
        limelight.start();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
//                if(result.getTx()>0){
//                    turnLeft();
//                }

                telemetry.addData("TX", result.getTx());
                telemetry.addData("TY", result.getTy());
                telemetry.addData("TA", result.getTa());
                if(result.getTx()>0){
                    telemetry.addData("Right or Left", "Right");
                }
                else{
                    telemetry.addData("Right or Left", "Left");
                }

            } else {
                telemetry.addLine("No target detected");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}