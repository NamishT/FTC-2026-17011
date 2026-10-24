package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name = "limelight")
public class limeLightV2{
    private Limelight3A limelight;

    private Pose3D pose;

    public static int startGame = 1;

//  0 = Blue | 1 = Red
    public static int allianceType = 0;

    public int[] redAllianceTags = {34, 35, 36, 37, 30, 31, 32, 33};

    public int[] blueAllianceTags = {38, 39, 40, 41, 42, 43, 44, 45};

    public double[][] limeLightList = new double[8][3];

    public limeLightV2(HardwareMap hardwareMap)
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        limelight.start();
    }


    public double[][] limeLight()
    {

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid())
        {
            int number = -1;

            for (LLResultTypes.FiducialResult tag : result.getFiducialResults())
            {

                if (allianceType == 1) {
                    for (int type : redAllianceTags) {
                        if (tag.getFiducialId() == type) {
                            number++;

                            pose = tag.getCameraPoseTargetSpace();

                            Position posee = pose.getPosition();

                            double posex = posee.x;
                            double posey = posee.y;
                            double posez = posee.z;

                            double distance = Math.sqrt(posex * posex + posey * posey + posez * posez);

                            double[] listing = {distance, tag.getTargetXDegrees(), tag.getFiducialId()};

                            limeLightList[number] = listing;
                        }
                    }
                }
                if (allianceType == 0)
                {
                    for (int type : blueAllianceTags)
                    {
                        if (tag.getFiducialId() == type)
                        {
                            number++;

                            pose = tag.getCameraPoseTargetSpace();

                            Position posee = pose.getPosition();

                            double posex = posee.x;
                            double posey = posee.y;
                            double posez = posee.z;

                            double distance = Math.sqrt(posex * posex + posey * posey + posez * posez);

                            double[] listing = {distance, tag.getTargetXDegrees(), tag.getFiducialId()};

                            limeLightList[number] = listing;
                        }
                    }

                }
            }
        }
        return limeLightList;
    }

}