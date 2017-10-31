package FineLib;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
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

/**
 * Created by drew on 10/30/17.
 */

public class FineVision {

    public static final Scalar lowerBlue = new Scalar(80, 50, 0);
    public static final Scalar upperBlue = new Scalar(110, 255, 255);
    public static final Scalar lowerRed = new Scalar(0, 50, 0);
    public static final Scalar upperRed = new Scalar(8, 255, 255);
    public static final int BLUE = 0;
    public static final int RED = 1;
    public static final int SHITS_NOT_LINED_UP = -666;
    int iteration = 0;
    HardwareMap hwMap;
    VuforiaLocalizer cryptoLocalizer;
    VuforiaTrackables cryptokey;
    VuforiaTrackable cryptoTemplate;
    OpenGLMatrix pose;
    Image vufImage;
    Mat cvImage;
    RelicRecoveryVuMark column = RelicRecoveryVuMark.UNKNOWN;
    int saveInterval = 0;

    public FineVision(HardwareMap hwMap, int saveInterval) {
        if (saveInterval == 0)
            this.saveInterval = Integer.MAX_VALUE;
        else
            this.saveInterval = saveInterval;
        this.hwMap = hwMap;
        //initializes the camera monitor on the robot phone
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera M onitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //choose the front(low res) or back(high res)
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        params.vuforiaLicenseKey = "AUgSTBn/////AAAAGSU/cD15/UsujI6xLYV74ziGgnCxnhNN3o+oqCbjOAYeuTL3onL+U3IeZxlEpkmbZUZo3dM9ASoSZmIJSdJD4qql7aQoGkyiMmQrG0VrtDRYXGfD0S2gkiP9zyr+Cq+j0OFfrefZrq+k+29VF6ON1KOoPJdDVfUvfbj96xmLd9E6p3bGoJUQSbgnGu+ZkMK2+0Qu8tFe6v8Wx+0v3amf6kgOAaLbjdGqAygEwk9pEOWFxIjpUcwZj8qNqZvtRJP+7csocK3MYC+stHvVh42xXaXeShzC737bkSj0G4lWCtI3JNFDw6NRKX0dmwLbIVMizvudFRXwF2SahUpwh+h/2T5WWSfWP3lcrDYQRgJ54PWG";
        //tells vuforia to show exes ontop of identified targets
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        cryptoLocalizer = ClassFactory.createVuforiaLocalizer(params);
        //sets max tracked images to 1
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);

        //enables RGB565 format for the image
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        //tells VuforiaLocalizer to only store one frame at a time
        cryptoLocalizer.setFrameQueueCapacity(1);
        //defines where vuforia loads the targets from
        cryptokey = cryptoLocalizer.loadTrackablesFromAsset("RelicVuMark");
        //grabs to first target form the specified asset above and sets the name for lols
        cryptoTemplate = cryptokey.get(0);
        cryptoTemplate.setName("CryptoTemplate");
    }
    public void activate() {
        cryptokey.activate();
    }
    public void grab() throws InterruptedException {
        pose = ((VuforiaTrackableDefaultListener) cryptoTemplate.getListener()).getPose();
        column = RelicRecoveryVuMark.from(cryptoTemplate);
        vufImage = getFrame(cryptoLocalizer);
        cvImage = vuforia2cv(vufImage);
    }
    public void deactivate() {
        cryptokey.deactivate();
    }

    public OpenGLMatrix getPose() {
        return pose;
    }
    public RelicRecoveryVuMark getColumn() {
        return column;
    }

    public VectorF getTranslation() {
        return getPose().getTranslation();
    }

    public Orientation getRotaion() {
        return Orientation.getOrientation(getPose(), AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }

    public String toString() {
        return format(((VuforiaTrackableDefaultListener) cryptoTemplate.getListener()).getPose());
    }

    public Mat getImage() throws InterruptedException {
        return cvImage;
    }

    public float[] getBallSetup() throws InterruptedException {
        return getBallSetup(vufImage, getPose(), cryptoLocalizer.getCameraCalibration());
    }

    private Image getFrame(VuforiaLocalizer vuforia) throws InterruptedException {
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

        if (pose != null && img != null) {
            /*
            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData)A;

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
            return analyze(cropped);*/
            Mat image = vuforia2cv(img);
            Rect cropRegion = new Rect(0, 2 * (image.rows() / 3), image.cols() - 1, image.rows() / 3 - 1);
            Mat cropped = image.submat(cropRegion);
            return analyze(cropped);
        }
        return null;
    }

    private float[] analyze(Mat raw) {
        iteration++;
        float[] result = new float[2];
        Mat cropped = new Mat();
        Imgproc.cvtColor(raw, cropped, Imgproc.COLOR_RGB2HSV);
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
            String fileraw = "raw" + System.nanoTime() + ".png";
            File fileBlue = new File(path, fileblue);
            File fileRed = new File(path, filered);
            File fileRaw = new File(path, fileraw);
            Imgcodecs.imwrite(fileBlue.toString(), maskBlue);
            Imgcodecs.imwrite(fileRed.toString(), maskRed);
            Imgcodecs.imwrite(fileRaw.toString(), raw);
            //telemetry.addData("imwrite", succ);
        }
        double[] centroidBlue = {momentsBlue.get_m10() / momentsBlue.get_m00(), momentsBlue.get_m01() / momentsBlue.get_m00()};
        double[] centroidRed = {momentsRed.get_m10() / momentsRed.get_m00(), momentsRed.get_m01() / momentsRed.get_m00()};

        if (centroidBlue[0] < cropped.cols() / 2)
            result[0] = BLUE;
        else
            result[1] = BLUE;

        if (centroidRed[0] < cropped.cols() / 2)
            result[0] = RED;
        else
            result[1] = RED;

        if (momentsBlue.get_m00() > maskBlue.total() * 0.8 || momentsRed.get_m00() > maskRed.total() * 0.8) {
            result[0] = SHITS_NOT_LINED_UP;
            result[1] = SHITS_NOT_LINED_UP;
        }
        if (momentsBlue.get_m00() > maskBlue.total() * 0.2 || momentsRed.get_m00() > maskRed.total() * 0.2) {
            result[0] = SHITS_NOT_LINED_UP;
            result[1] = SHITS_NOT_LINED_UP;
        }
//        result[0] = SHITS_NOT_LINED_UP;
        //      result[1] = SHITS_NOT_LINED_UP;
        return result;
    }

    private Mat vuforia2cv(Image img) {
        Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(img.getPixels());

        Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);

        Utils.bitmapToMat(bm, crop);
        Core.rotate(crop, crop, Core.ROTATE_90_CLOCKWISE);
        return crop;
    }

    private String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    private static float IN2MM(float in) {
        return in * 25.4f;
    }

}
