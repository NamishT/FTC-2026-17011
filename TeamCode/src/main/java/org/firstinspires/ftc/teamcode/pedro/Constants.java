package org.firstinspires.ftc.teamcode.pedro;


import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {


    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });


    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(7.741637342558133);
        c.yPodOffset.set(-1.1912250518798828);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4429376964097699);
                Controller secondaryTranslationalForward = Controller.proportional(0.16365364331018414);
                Controller primaryTranslationalLateral = Controller.proportional(1.215267172380196);
                Controller secondaryTranslationalLateral = Controller.proportional(0.44900874765758114);


                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));


                c.coast.set(Controller.proportionalFeedforward(0.013577394196151933));
                c.brake.set(Controller.proportionalFeedforward(0.011540785066729143));


                c.headingFeedback.set(Controller.proportional(8.706526443014454));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06858596075218377, 0.008167199761236026));


                c.linearBrakeCoefficients.set(Matrix.diag(0.21560963076284465, 0.08193281044809385));
                c.quadraticBrakeCoefficients.set(Matrix.diag(3.8081153337648984E-4, 0.0017792620289998871));


                c.maxAchievableForwardVelocity.set(78.4996932984441);
                c.maxAchievableStrafeVelocity.set(51.784975605690434);
                c.naturalForwardDeceleration.set(35.06918161111226);
                c.naturalStrafeDeceleration.set(69.61748817269898);
            }
    );
    public static Follower create(HardwareMap hardwareMap) {
        Localizer localizer = new PinpointLocalizer(hardwareMap, localizerConfig);
        Drivetrain drivetrain = new Mecanum(hardwareMap, drivetrainConfig);
        Algorithm algorithm = new Foresight(foresightConfig);
        return new Follower(localizer, drivetrain, algorithm);
    }
}