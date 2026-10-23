package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
@TeleOp(name = "limelight")
public class limeLightV2 extends LinearOpMode {
    private Limelight3A limelight;
    public void runOpMode()
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        waitForStart();
        while(opModeIsActive())
        {
            LLResult result = limelight.getLatestResult();
            if(result != null && result.isValid())
            {
                telemetry.addData("Valid", result.isValid());
                telemetry.addData("Pipeline", result.getPipelineIndex());
                telemetry.addData("Pipeline type", result.getPipelineType());
                int size = result.getBarcodeResults().size();
                LLResultTypes.BarcodeResult[] barcodeResults = result.getBarcodeResults().toArray(new LLResultTypes.BarcodeResult[size]);
                for(LLResultTypes.BarcodeResult results: barcodeResults)
                {
                    telemetry.addLine("BarCode" + results.getTargetXDegrees());
                }
                sleep(20);
                telemetry.update();
            }

            else
            {
                telemetry.addLine("Not reading an AprilTag / experiencing an error");
                telemetry.update();
            }
        }
    }
}