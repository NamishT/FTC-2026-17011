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
        c.frontLeftName.set("backLeft");
        c.frontRightName.set("backRight");
        c.backLeftName.set("frontLeft");
        c.backRightName.set("frontRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(7.286581204632136);
        c.yPodOffset.set(0.25816200286384644);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                // Cut lateral gains down so small errors don't slam 100% strafe power
                Controller primaryTranslationalForward = Controller.proportional(0.25);
                Controller secondaryTranslationalForward = Controller.proportional(0.12);
                Controller primaryTranslationalLateral = Controller.proportional(0.35);
                Controller secondaryTranslationalLateral = Controller.proportional(0.18);


                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));


                c.coast.set(Controller.proportionalFeedforward(0.013664912023065861));
                c.brake.set(Controller.proportionalFeedforward(0.011615175219605981));


                // Reduced from 10.05 -> 2.2 so heading holds smoothly without violent oscillation
                c.headingFeedback.set(Controller.proportional(10.05));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06975824916623499, 0.008267171697923747));


                c.linearBrakeCoefficients.set(Matrix.diag(0.08802774110344731, 0.06546845515926333));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0021280310602220937, 0.0023379447569587234));


                c.maxAchievableForwardVelocity.set(75.78939171742421);
                c.maxAchievableStrafeVelocity.set(52.695086598626474);
                c.naturalForwardDeceleration.set(34.585695644183474);
                // Lowered from 72.48 -> 42.0 so it starts braking in time on strafes
                c.naturalStrafeDeceleration.set(42.0);
            }
    );
    public static Follower create(HardwareMap hardwareMap) {
        Localizer localizer = new PinpointLocalizer(hardwareMap, localizerConfig);
        Drivetrain drivetrain = new Mecanum(hardwareMap, drivetrainConfig);
        Algorithm algorithm = new Foresight(foresightConfig);
        return new Follower(localizer, drivetrain, algorithm);
    }
}
