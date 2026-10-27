package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.util.*;
public class limeLightV2{
    private Limelight3A limelight;
    private Pose3D pose;
    private static int startGame = 1;
    //  0 = Blue | 1 = Red
    private int[] redAllianceTags = {34, 35, 36, 37, 30, 31, 32, 33};
    private int[] blueAllianceTags = {38, 39, 40, 41, 42, 43, 44, 45};
    private double[][] limeLightList = new double[8][3];
    public limeLightV2(HardwareMap hardwareMap)
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }
    public double[][] limeLight(int allianceType) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return new double[0][];
        }
        ArrayList<double[]> tags = new ArrayList<>();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            boolean matches = false;
            if (allianceType == 1) {
                for (int type : redAllianceTags) {
                    if (id == type) {
                        matches = true;
                        break;
                    }
                }
            } else if (allianceType == 0) {
                for (int type : blueAllianceTags) {
                    if (id == type) {
                        matches = true;
                        break;
                    }
                }
            }
            if (!matches) {
                continue;
            }
            Position p = tag.getCameraPoseTargetSpace().getPosition();
            double distance = Math.sqrt(
                    p.x * p.x + p.y * p.y + p.z * p.
                            z
            );
            tags.add(new double[] {
                    distance,
                    tag.getTargetXDegrees(),
                    id
            });
        }
        return tags.toArray(new double[0][]);
    }
}