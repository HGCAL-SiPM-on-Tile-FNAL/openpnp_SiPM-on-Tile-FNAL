package org.openpnp.vision.pipeline.stages;

import java.awt.Color;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openpnp.vision.FluentCv;
import org.openpnp.vision.pipeline.CvPipeline;
import org.openpnp.vision.pipeline.CvStage;
import org.simpleframework.xml.Attribute;
import org.opencv.core.RotatedRect;
import java.lang.Math;
import org.pmw.tinylog.Logger;
/**
 * Mask everything in the working image outside of a circle centered at the center of the image with
 * the specified diameter.
 */
public class MaskMinAreaRect extends CvStage {


    @Override
    public Result process(CvPipeline pipeline) throws Exception {
        Mat mat = pipeline.getWorkingImage();
        Mat mask = mat.clone();
        Mat masked = mat.clone();
        Scalar color = FluentCv.colorToScalar(Color.black);
        mask.setTo(color);
        masked.setTo(color);
        
        

        RotatedRect minRect = (RotatedRect) pipeline.getResult("rect").getModel();
        
        Point centerPoint = minRect.center;
        double c_y = centerPoint.y;
        double c_x = centerPoint.x;
	double width = minRect.size.width;
	double height = minRect.size.height;
	double angle = minRect.angle;
	
	Point low = new Point(c_x + ((width/2)*Math.cos(Math.toRadians(angle)) - (height/2)*Math.sin(Math.toRadians(angle))) + 50, c_y + ((width/2)*Math.cos(Math.toRadians(angle)) + (height/2)*Math.sin(Math.toRadians(angle))) + 50);
	Point high = new Point(c_x - ((width/2)*Math.cos(Math.toRadians(angle)) - (height/2)*Math.sin(Math.toRadians(angle)))- 50, c_y - ((width/2)*Math.cos(Math.toRadians(angle)) + (height/2)*Math.sin(Math.toRadians(angle))) - 50);
	Logger.debug("width");	
	Logger.debug(width);	
	Logger.debug("height");	
	Logger.debug(height);	
	Logger.debug("center");
	Logger.debug(centerPoint);	
	Logger.debug("low");	
	Logger.debug(low);
	Logger.debug(high);
	Logger.debug("high");	
        Imgproc.rectangle(mask, low, high, new Scalar(255, 255, 255), -1);

        mat.copyTo(masked, mask);
        mask.release();
        return new Result(masked);
    }
}
