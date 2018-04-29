package org.firstinspires.ftc.teamcode.Competition.OpenCVClasses;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by KyleSmith on 2/4/18.
 */
public class OpenCVJewelID extends OpenCVPipeline{

    private boolean showRed = true;

    private Mat hsv = new Mat();
    private Mat thresholded = new Mat();
    private Mat thresholded_rgba = new Mat();

    public int iJewelX = 0;
    public int iJewelY = 0;
    double maxVal = 0;
    int maxValIdx = 0;

    public  boolean running;

    //Method run when a certain button it pressed
    public void setShowBlue(boolean enabled){ showRed = enabled; }


    @Override
    public Mat processFrame(Mat rgba, Mat gray) {

        // First, we change the colorspace from RGBA to HSV, which is usually better for color
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);

        // Then, we threshold our hsv image so that we get a black/white binary image where white
        // is the blues listed in the specified range
        // 2de 170, 255, 255
        // 1ste 90, 128, 30
        //146, 161, 207
        Core.inRange(hsv, new Scalar(90,128,30), new Scalar(170,255,255), thresholded);


        // Then we display our little binary threshold on screen
        if (showRed) {

            // since the thresholded image data is a black and white image, we have to convert it back to rgba
            Imgproc.cvtColor(thresholded, thresholded_rgba, Imgproc.COLOR_GRAY2RGBA);

            Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
            Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));

            //Morphological Operators
            Imgproc.erode(thresholded, thresholded_rgba, erodeElement);
            Imgproc.erode(thresholded, thresholded_rgba, erodeElement);

            Imgproc.dilate(thresholded, thresholded_rgba, dilateElement);
            Imgproc.dilate(thresholded, thresholded_rgba, dilateElement);

            //frame = this.findAndDrawBalls(thresholded_rgba, frame);

            return findAndDrawBalls(thresholded, thresholded_rgba);
        } else {
            // if we aren't displaying the binary image, just show the original frame onscreen.
            return rgba;
        }


    }

    private Mat findAndDrawBalls(Mat maskedImage, Mat frame) {
        // init
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        // find contours
        Imgproc.findContours(maskedImage, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);



        // if any contour exist...
        if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
            // for each contour, display it in yellow

            for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0]) {

                Rect rect = Imgproc.boundingRect(contours.get(idx));
                Imgproc.rectangle(frame, rect.tl(), rect.br(), new Scalar(255,255,255), 1);

                double contourArea = Imgproc.contourArea(contours.get(idx));

                if(maxVal <= contourArea){

                    maxVal = contourArea;
                    maxValIdx = idx;

                    running = true;

                    iJewelX = rect.x;
                    iJewelY = rect.y;



                }else{

                    //maxVal = 0;

                }

                Imgproc.drawContours(frame, contours, idx, new Scalar(0, 255, 255));


            }
        }

        return frame;
    }

    public int getiJewelX(){

        return iJewelX;
    }

    public int getiJewelY(){

        return iJewelY;

    }

    public boolean isRunning(){

        return running;
    }

    public void closeOpenCV(){

        setShowBlue(false);
        disable();

    }


}
