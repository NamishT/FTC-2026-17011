//
//package org.firstinspires.ftc.teamcode.subsystems;
//
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//
//
//import java.util.ArrayList;
//import java.util.List;
//
//
//public class limeLightV2 {
//
//
//    private Limelight3A limelight;
//    private Servo NectarShoot;
//    private Servo PollenShoot;
//
//
//    // 0 = Blue, 1 = Red
//    private final int[] blueAllianceTags =
//            {38, 39, 40, 41, 42, 43, 44, 45};
//
//
//    private final int[] redAllianceTags =
//            {34, 35, 36, 37, 30, 31, 32, 33};
//
//
//    private boolean cameraValid = false;
//
//
//    public limeLightV2(HardwareMap hardwareMap) {
//
//
////        limelight = hardwareMap.get(
////                Limelight3A.class, "limelight");
//
//
//        limelight.pipelineSwitch(0);
//        limelight.start();
//    }
//
//
//    public double[][] limeLight(int allianceType) {
//
//
//        List<double[]> detectedTags = new ArrayList<>();
//
//
//        LLResult result = limelight.getLatestResult();
//
//
//        cameraValid = result != null && result.isValid();
//
//
//        if (!cameraValid) {
//            return new double[0][];
//        }
//
//
//        int[] allowedTags =
//                allianceType == 1
//                        ? redAllianceTags
//                        : blueAllianceTags;
//
//
//        for (LLResultTypes.FiducialResult tag :
//                result.getFiducialResults()) {
//
//
//            boolean correctAlliance = false;
//
//
//            for (int id : allowedTags) {
//                if (tag.getFiducialId() == id) {
//                    correctAlliance = true;
//                    break;
//                }
//            }
//
//
//            if (!correctAlliance) {
//                continue;
//            }
//
//
//            Pose3D pose = tag.getCameraPoseTargetSpace();
//
//
//            if (pose == null) {
//                continue;
//            }
//
//
//            Position position = pose.getPosition();
//
//
//            double x = position.x;
//            double y = position.y;
//            double z = position.z;
//
//
//            double distance = Math.sqrt(
//                    x * x + y * y + z * z
//            );
//
//
//            double tx = tag.getTargetXDegrees();
//
//
//            detectedTags.add(new double[]{
//                    distance,
//                    tx,
//                    tag.getFiducialId()
//            });
//        }
//
//
//        return detectedTags.toArray(new double[0][]);
//    }
//
//
//    public boolean isCameraValid() {
//        return cameraValid;
//    }
//}