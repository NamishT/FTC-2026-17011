package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.limeLightV2;

import java.util.Arrays;

@TeleOp(name = "Limelight 3A Test")
@Config
public class Limelight extends LinearOpMode {

    private Limelight3A limelight;


    @Override
    public void runOpMode() {

//        while (limeLightV2.startGame == 1)
//            {
//                telemetry.addLine("Press Right Bumper for Red Alliance and Left Bumper for Blue Alliance");
//
//                if (gamepad1.rightBumperWasPressed())
//                {
//
//                    limeLightV2.allianceType = 1;
//                    limeLightV2.startGame = 0;
//                }
//                if (gamepad1.leftBumperWasPressed())
//                {
//                    limeLightV2.allianceType = 0;
//                    limeLightV2.startGame = 0;
//                }
//
//                telemetry.update();
//            }
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        waitForStart();

        while(opModeIsActive())
        {
            limeLightV2 lime = new limeLightV2(hardwareMap);

            String output = Arrays.deepToString(lime.limeLight());

            telemetry.addLine(output);

            telemetry.update();
        }

        // Get Limelight from Robot Configuration
    }
}