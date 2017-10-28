package org.firstinspires.ftc.finemen;

import android.graphics.Bitmap;
import android.os.Environment;
import android.speech.tts.TextToSpeech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.util.Arrays;
import java.util.Locale;

/**
 * Created by drew on 8/15/17.
 */
@TeleOp(name = "vuforia")
public class VuforiaOp extends LinearOpMode {

    public static final Scalar lowerBlue = new Scalar(80,50,0);
    public static final Scalar upperBlue = new Scalar(110,255,255);
    public static final Scalar lowerRed = new Scalar(0,50,0);
    public static final Scalar upperRed = new Scalar(8,255,255);
    public static final int BLUE = 0;
    public static final int RED = 1;
    public static final int SHITS_NOT_LINED_UP = -666;
    int iteration = 0;
    TextToSpeech tts;


    @Override
    public void runOpMode() throws InterruptedException {
        //initializes the camera monitor on the robot phone
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera M onitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //choose the front(low res) or back(high res)
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        params.vuforiaLicenseKey = "AUgSTBn/////AAAAGSU/cD15/UsujI6xLYV74ziGgnCxnhNN3o+oqCbjOAYeuTL3onL+U3IeZxlEpkmbZUZo3dM9ASoSZmIJSdJD4qql7aQoGkyiMmQrG0VrtDRYXGfD0S2gkiP9zyr+Cq+j0OFfrefZrq+k+29VF6ON1KOoPJdDVfUvfbj96xmLd9E6p3bGoJUQSbgnGu+ZkMK2+0Qu8tFe6v8Wx+0v3amf6kgOAaLbjdGqAygEwk9pEOWFxIjpUcwZj8qNqZvtRJP+7csocK3MYC+stHvVh42xXaXeShzC737bkSj0G4lWCtI3JNFDw6NRKX0dmwLbIVMizvudFRXwF2SahUpwh+h/2T5WWSfWP3lcrDYQRgJ54PWG";
        //tells vuforia to show exes ontop of identified targets
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        VuforiaLocalizer cryptoLocalizer = ClassFactory.createVuforiaLocalizer(params);
        //sets max tracked images to 1
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);

        //enables RGB565 format for the image
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        //tells VuforiaLocalizer to only store one frame at a time
        cryptoLocalizer.setFrameQueueCapacity(1);
        //defines where vuforia loads the targets from
        VuforiaTrackables cryptokey = cryptoLocalizer.loadTrackablesFromAsset("RelicVuMark");
        //grabs to first target form the specified asset above and sets the name for lols
        VuforiaTrackable cryptoTemplate = cryptokey.get(0);
        cryptoTemplate.setName("CryptoTemplate");
        tts = new TextToSpeech(hardwareMap.appContext, new TextToSpeech.OnInitListener() {
            @Override
            public void onInit(int status) {
                if (status != TextToSpeech.ERROR) {
                    tts.setLanguage(Locale.US);
                }
            }
        });
        telemetry.addData("setup complete",null);
        telemetry.update();
        tts.speak("setup complete", TextToSpeech.QUEUE_ADD, null);

        waitForStart();
        //begin tracking targets
        cryptokey.activate();
        while (opModeIsActive()) {
            //converts the trackable to a vumark so we can get the ID, or whch tower to put glyph in
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(cryptoTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {//only if we can identify the target
                telemetry.addData("VuMark", "%s visible", vuMark);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)cryptoTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));//prints the positon data

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            Image img = getFrame(cryptoLocalizer);
            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);

            Utils.bitmapToMat(bm, crop);

            float[] ballSetup = analyze(crop);
            telemetry.addData("Ball Setup", ballSetup[0] + "," + ballSetup[1]);
            telemetry.update();
        }
        cryptokey.deactivate();
    }

    Image getFrame(VuforiaLocalizer vuforia) throws InterruptedException {
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
        Image rgb = null;

        long numImages = frame.getNumImages();

        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }
        return rgb;
    }

    private float[] getBallSetup(Image img, OpenGLMatrix pose, CameraCalibration camCal) {

        //OpenGLMatrix pose = cryptokey.getPose();

        if (pose != null) {
            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);

            float[][] corners = new float[4][2];
            //-1in down to -6in 3.5in right to 14
            //(-1,3.5)top left
            //(-1,14) top right
            //(-6,3.5) bottom left
            //(-6,14) bottom right
            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(IN2MM(-1), IN2MM(3.5f), 0)).getData();
            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(IN2MM(-1), IN2MM(14), 0)).getData();
            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(IN2MM(-6), IN2MM(3.5f), 0)).getData();
            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(IN2MM(-6), IN2MM(14), 0)).getData();

            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);

            Utils.bitmapToMat(bm, crop);

            float x = corners[0][0];
            float y = corners[0][1];
            float width = Math.abs(corners[0][0] - corners[1][0]);
            float height = Math.abs(corners[0][1] - corners[2][1]);

            x = Math.max(x, 0);
            y = Math.max(y, 0);
            width = (x + width > crop.cols()) ? crop.cols() - x : width;
            height = (y + height > crop.rows()) ? crop.rows() - y : height;

            Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));
            return analyze(cropped);
        }
        return null;
    }

    private float[] analyze(Mat cropped) {
        iteration++;
        float[] result = new float[2];
        Core.rotate(cropped, cropped, Core.ROTATE_90_CLOCKWISE);
        Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV);
        Mat maskBlue = new Mat();
        Core.inRange(cropped, lowerBlue, upperBlue, maskBlue);
        Mat maskRed = new Mat();
        Core.inRange(cropped, lowerRed, upperRed, maskRed);

        Moments momentsBlue = Imgproc.moments(maskBlue, true);
        Moments momentsRed = Imgproc.moments(maskRed, true);
        if (iteration % 50 == 0) {
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        String fileblue = "blue" + System.nanoTime() + ".png";
        String filered = "red" + System.nanoTime() + ".png";
        File fileBlue = new File(path, fileblue);
        File fileRed = new File(path, filered);
        Imgcodecs.imwrite(fileBlue.toString(), maskBlue);
        Imgcodecs.imwrite(fileRed.toString(), maskRed);
        //telemetry.addData("imwrite", succ);
        }
        double[] centroidBlue = {momentsBlue.get_m10()/momentsBlue.get_m00(), momentsBlue.get_m01()/momentsBlue.get_m00()};
        double[] centroidRed = {momentsRed.get_m10()/momentsRed.get_m00(), momentsRed.get_m01()/momentsRed.get_m00()};

        if (centroidBlue[0] < cropped.cols()/2)
            result[0] = BLUE;
        else
            result[1] = BLUE;

        if (centroidRed[0] < cropped.cols()/2)
            result[0] = RED;
        else
            result[1] = RED;

        if (momentsBlue.get_m00() > maskBlue.total()*0.8 || momentsRed.get_m00() > maskRed.total()*0.8) {
            result[0] = SHITS_NOT_LINED_UP;
            result[1] = SHITS_NOT_LINED_UP;
        }
        if (momentsBlue.get_m00() > maskBlue.total()*0.2 || momentsRed.get_m00() > maskRed.total()*0.2) {
            result[0] = SHITS_NOT_LINED_UP;
            result[1] = SHITS_NOT_LINED_UP;
        }
//        result[0] = SHITS_NOT_LINED_UP;
    //      result[1] = SHITS_NOT_LINED_UP;
        return result;
    }


    private String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
    private static float IN2MM(float in) {
        return in * 25.4f;
    }
}


