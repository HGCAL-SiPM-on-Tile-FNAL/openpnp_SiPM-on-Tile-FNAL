package org.openpnp.vision.pipeline.stages;
import java.util.ArrayList;
import java.util.List;
import java.util.Date;
import java.io.File;
import java.io.FileOutputStream;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileNotFoundException;

import java.awt.Color;
import java.lang.Math;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.MatOfPoint;
import org.opencv.core.CvType;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;
import org.openpnp.vision.FluentCv;
import org.openpnp.vision.pipeline.CvPipeline;
import org.openpnp.vision.pipeline.CvStage;
import org.openpnp.Timing;
import org.openpnp.model.Location;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Configuration;
import org.openpnp.model.Footprint.Pad;
import org.openpnp.util.VisionUtils;
import org.openpnp.spi.Camera;
import org.openpnp.spi.Nozzle;
import org.openpnp.spi.PnpJobProcessor.JobPlacement;
import org.openpnp.model.Part;
import org.openpnp.spi.Machine;
import org.openpnp.spi.Movable;
import org.simpleframework.xml.Attribute;
import org.pmw.tinylog.Logger;
import org.openpnp.model.Job;
import org.openpnp.machine.reference.ReferencePnpJobProcessor;
import org.openpnp.gui.JobPanel;
import java.time.LocalDateTime;
import org.openpnp.model.Placement;

class Pair{
    double x;
    double y;
}

/*
Class to hold lines and return lengths
*/
class LineLocal{
    public CornerPoint start;
    public CornerPoint end;
    
    LineLocal(){
        this.start = null;
        this.end = null;
    }
    
    LineLocal(CornerPoint first, CornerPoint second){
        this.start = first;
        this.end = second;
    }
    
    public double getLen(){
        return Math.sqrt((Math.pow((this.start.x - this.end.x),2) + Math.pow((this.start.y - this.end.y),2)));
    }
}
/*
Class to hold coordinate values for a point and the lines that it is connected to
*/
class CornerPoint{
    public float x;
    public float y;

    private ArrayList<CornerPoint> connected = new ArrayList<CornerPoint>();
    private ArrayList<LineLocal> lines = new ArrayList<LineLocal>();
    
    public CornerPoint(float newX, float newY) {
        this.x = newX;
        this.y = newY;
    }
    
    public void setPoint(int newX,int newY) {
        this.x = newX;
        this.y = newY;
    }
    
    public ArrayList<LineLocal>  returnLines() {
        return this.lines;
    }
    
    public void printLines(){
        for(int i = 0; i < this.lines.size(); i++){
            Logger.debug(this.lines.get(i).getLen());
        }
    }
    
    public void connect(CornerPoint newPoint){
        this.connected.add(newPoint);
    }
    
    public int isConnected(CornerPoint endpoint){
        for(int i = 0; i < this.connected.size(); i++){
            if(this.connected.get(i).x == endpoint.x){
                if(this.connected.get(i).y == endpoint.y){
                    return 1;
                }
            }
        }
        return 0;
    }
    
    public double distance(CornerPoint other){
        return Math.sqrt( Math.pow(this.x - other.x,2) + Math.pow(this.y - other.y,2));
    }
    
    public void chooseFromThree(CornerPoint point_1,CornerPoint point_2,CornerPoint point_3){

        double hypo_1 = 0;
        double hypo_2 = 0;
        double hypo_3 = 0;
        
        double ang_1_2 = Math.acos((Math.pow(this.distance(point_1),2) + Math.pow(this.distance(point_2),2) - (Math.pow((point_1.x - point_2.x), 2) + Math.pow((point_1.y - point_2.y),2))) / ((2*this.distance(point_1)*this.distance(point_2))));

        double ang_1_3 = Math.acos((Math.pow(this.distance(point_1),2) + Math.pow(this.distance(point_3),2) - (Math.pow((point_1.x - point_3.x), 2) + Math.pow((point_1.y - point_3.y),2))) / ((2*this.distance(point_1)*this.distance(point_3))));
        
        double ang_2_3 = Math.acos((Math.pow(this.distance(point_2),2) + Math.pow(this.distance(point_3),2) - (Math.pow((point_2.x - point_3.x), 2) + Math.pow((point_2.y - point_3.y),2))) / ((2*this.distance(point_2)*this.distance(point_3))));

        if(ang_1_2 > ang_1_3 && ang_1_2 >  ang_2_3){

            if(this.isConnected(point_1) == 0){
                this.connect(point_1);
                point_1.connect(this);
                this.lines.add(new LineLocal(this,point_1));
            }

            if(this.isConnected(point_2) == 0){
                this.connect(point_2);
                point_2.connect(this);
                this.lines.add(new LineLocal(this,point_2));
            }
        }
        
        if(ang_1_3 >  ang_1_2 && ang_1_3 >  ang_2_3){

            if(this.isConnected(point_1) == 0){
                this.connect(point_1);
                point_1.connect(this);
                this.lines.add(new LineLocal(this,point_1));
            }
                
            if(this.isConnected(point_3) == 0){
                this.connect(point_3);
                point_3.connect(this);
                this.lines.add(new LineLocal(this,point_3));
            }
        }
        
        if(ang_2_3 >  ang_1_3 && ang_2_3 >  ang_1_2){

            if(this.isConnected(point_3) == 0){
                this.connect(point_3);
                point_3.connect(this);
                this.lines.add(new LineLocal(this,point_3));
            }
            if(this.isConnected(point_2) == 0){
                this.connect(point_2);
                point_2.connect(this);
                this.lines.add(new LineLocal(this,point_2));
            }
        }
        
    }
}



public class MinAreaTrap extends CvStage {
    
    static int image_iteration = 0;

    /* These attributes and get and set functions are used to 
    interface with the pipeline and accept attribute values. 
    I do not understand why it is set up this way.*/

    @Attribute
    private double rho = 1;
    
    @Attribute
    private double theta = 0.01744;

    @Attribute
    private int threshold = 80;  

    @Attribute
    private double minLineLength = 1000;

    @Attribute
    private double maxGap = 20;

    @Attribute
    private double allowedError = 20;

    public double getrho() {
        return rho;
    }
    public void setrho(double rho) {
        this.rho = rho;
    }  

    public double gettheta() {
        return theta;
    }
    public void settheta(double theta) {
        this.theta = theta;
    }   

    public int getthreshold() {
        return threshold;
    }
    public void setthreshold(int threshold) {
        this.threshold = threshold;
    }  

    public double getminLineLength() {
        return minLineLength;
    }
    public void setminLineLength(double minLineLength) {
        this.minLineLength = minLineLength;
    } 

    public double getmaxGap() {
        return maxGap;
    }
    public void setmaxGap(double maxGap) {
        this.maxGap = maxGap;
    }   

    public double getallowedError() {
        return allowedError;
    }
    public void setallowedError(double allowedError) {
        this.allowedError = allowedError;
    }   

    public double calcDistance(double startX, double startY, double endX, double endY){
        return Math.sqrt((Math.pow((startX - endX),2) + Math.pow((startY - endY),2)));
    }

    @Override
    public Result process(CvPipeline pipeline) throws Exception {
        Logger.debug("start");
        int four_point_flag = 0;
        
        image_iteration += 1;
        String image_iteration_name = "" + image_iteration;
        /*Set up debugging file*/
        String noSpaceName = Timing.start + "";
        /*Windows rejects any name with a space / or : so we remove them*/
        noSpaceName = noSpaceName.replaceAll(" ", "_");
        noSpaceName = noSpaceName.replaceAll("/", "_");
        noSpaceName = noSpaceName.replaceAll(":", "_");
        
	    //Create directories if they don't exist
	    File dir_CornerLog = new File("CornerLog");
	    dir_CornerLog.mkdirs();
	    File dir_JobData = new File("JobData");
	    dir_JobData.mkdirs();


        byte[] bytesArray; /*Temp byte array for writing to the file*/
        String name = "CornerLog//"+ noSpaceName + "";
        File tileDataFile = new File(name+".csv");
	    Timing.fileName = name+".csv";
        /*Create the file if it does not already exist*/
        tileDataFile.createNewFile(); 
        FileOutputStream oFile = new FileOutputStream(tileDataFile, true);


        /*Temp string buf*/
        String toPrint = "";

        String toPrintRight = "";
        String toPrintLeft = "";
        String toPrintTop = "";
        String toPrintBot = "";
        Mat img = new Mat(); 
        Mat save = new Mat();
        Mat dest = new Mat();
        Mat result = new Mat();

        String t_job= "";
        String t_board = "";


        Logger.debug("/*Get job info*/");

        /*Get job info*/
        ReferencePnpJobProcessor jobProcessor = (ReferencePnpJobProcessor) Configuration.get().getMachine().getPnpJobProcessor();

        if(jobProcessor != null){
	//    t_job = "No Job";
        }else{
        //    t_job = "No Job";
        }

        String string_dir = "LogFiles//"+ noSpaceName + "//" + t_job;

        File testTileDataFile = new File(string_dir);
        testTileDataFile.mkdirs();
        
        // Logger.debug("/*Set up jobdata file*/");
        // /*Set up jobdata file*/
        // name = "JobData//"+ noSpaceName + "";
        // File tileJobDataFile = new File(name+".csv");
        // String last, line,previousLine;
        // previousLine = "";
        // last = "";

        // try{
        //     BufferedReader jobDataInput = new BufferedReader(new FileReader(tileJobDataFile));

        //     while ((line = jobDataInput.readLine()) != null) { 
        //         previousLine = last;
        //         last = line;
        //     }
        // }
        // catch (FileNotFoundException e){
        //     previousLine = "";
        //     last = "";
        // }

        Logger.debug("/*Writes to cornerLog File*/");
        /*Writes to cornerLog File*/
        toPrint = "DateTime:, " + LocalDateTime.now()+"\n";
        //toPrint += "job:, " + t_job+"\n";
        //toPrint += "board:, " + previousLine+"\n";
        //toPrint += "id:, " + last+"\n";
        bytesArray = toPrint.getBytes();
        oFile.write(bytesArray);
        oFile.flush();
        
        double top_len = 0;
        double bot_len = 0;
        double height = 0;
        double orient = 0;
        double special = 0;

        Logger.debug("5");
        /*Get current nozzle information*/

        Nozzle noz = (Nozzle) pipeline.getProperty("nozzle");
	    Part part = (Part) pipeline.getProperty("part");
	    //JobPlacement jobplacmnt = (JobPlacement) pipeline.getProperty("jobPlacement");
	    //Placement placmnt = (Placement) pipeline.getProperty("placement");
 
        //part.getName()
        Camera cam  = VisionUtils.getBottomVisionCamera();
        double unitPPX = cam.getUnitsPerPixel().getX();
        double unitPPY = cam.getUnitsPerPixel().getY();
    
        Mat lines = img.clone();
        ArrayList<Pair> corner_list = new ArrayList<Pair>();

        //while(four_point_flag == 0){
        Logger.debug("6");
        img = pipeline.getWorkingImage();
        save = img.clone();
        dest = img.clone();
        result = img.clone();
        Logger.debug("6.1");
        
        //Logger.debug("Pipeline",pipeline.properties);

        /*Write part id to file*/
        toPrint = "part, " + noz.getPart().getId()+"\n";
        bytesArray = toPrint.getBytes();
        oFile.write(bytesArray);
        oFile.flush();

        //this block returns null as of now
        toPrint = "job placement placement id, " + jobplacmnt.getPlacement() +"\n";
        bytesArray = toPrint.getBytes();
        oFile.write(bytesArray);
        oFile.flush();


        toPrint = "iteration, " + Timing.counter+"\n";
        bytesArray = toPrint.getBytes();
        oFile.write(bytesArray);
        oFile.flush();
        Logger.debug("6.2");
        Imgproc.cvtColor(result, result, Imgproc.COLOR_GRAY2RGB);


        Logger.debug("7");
        Logger.debug(this.rho);
        Logger.debug(this.theta);
        Logger.debug(this.threshold);
        Logger.debug(this.maxGap);
        Logger.debug(this.minLineLength);
        Imgproc.HoughLinesP(save, lines,this.rho, this.theta,this.threshold,this.minLineLength,this.maxGap);

	    Mat copy = img.clone();



      	for (int i = 0; i < lines.rows(); i++) {
            double[] val = lines.get(i, 0);
            //Logger.debug("X1:" + val[0] + " Y1:" + val[1] + "X2:" + val[2] + " Y2:" + val[3]);
            Imgproc.line(copy, new Point(val[0], val[1]), new Point(val[2], val[3]), new Scalar(255, 255, 255), 1);
        }

        Logger.debug("7.2");
        Imgcodecs.imwrite("CornerLog//"+ noSpaceName + image_iteration_name + ".jpg", copy);

        Logger.debug("8");

        for (int i = 0; i < lines.rows(); i++) {
            double[] val = lines.get(i, 0);
	    //Logger.debug("trial" + lines.get(i, 0));       
        }


        /*Set up lists*/
        //(Should simplify this, is probably overly complicated)

        List top = new ArrayList();
        List bot = new ArrayList();
        List left = new ArrayList();
        List right = new ArrayList();
        
        ArrayList<Double> intercept_list = new ArrayList<Double>();
        ArrayList<Double> angle_list = new ArrayList<Double>();
        ArrayList<Double>[] listOfLists = new ArrayList[4];
        
        for (int i = 0; i < 4; i++) {
            listOfLists[i] = new ArrayList<Double>();
        }
        
        ArrayList<Double>[] listOfInter = new ArrayList[4];
        
        for (int i = 0; i < 4; i++) {
            listOfInter[i] = new ArrayList<Double>();
        }

        ArrayList<Double>[] listOfInf = new ArrayList[4];
        
        for (int i = 0; i < 4; i++) {
            listOfInf[i] = new ArrayList<Double>();
        }
        
        ArrayList<Double>[] listOfAngle = new ArrayList[4];
        
        for (int i = 0; i < 4; i++) {
            listOfAngle[i] = new ArrayList<Double>();
        }
        
        ArrayList<Double> average_inter = new ArrayList<Double>();
        ArrayList<Double> average_inf = new ArrayList<Double>();
        ArrayList<Double> average_slope = new ArrayList<Double>();
        ArrayList<Double> average_angle = new ArrayList<Double>();
        ArrayList<Double> average_XCheck = new ArrayList<Double>();
        ArrayList<Double> average_YCheck = new ArrayList<Double>();
        ArrayList<Double> big_angle = new ArrayList<Double>();

        for(int m = 0; m < 4; m++){

            average_inter.add(0.0);
            average_inf.add(0.0);
            average_slope.add(0.0);
            average_angle.add(0.0);
            average_XCheck.add(0.0);
            average_YCheck.add(0.0);
            big_angle.add(0.0);


        }
        
        for(int m = 0; m < 4; m++){
            
            intercept_list.add(m,0.0);
            angle_list.add(m,0.0);

        }

        double first_slope = 0;
        double second_slope = 0;
        
        double first_angle = 0;
        double second_angle = 0;
        
        /*Reject tile if too few lines are found*/
        if(lines.rows() < 4){
            oFile.close();
            return new Result(result);
        }

        /*Iterate over all the found lines and store their data in various
         *lists for later use*/
        for (int i = 0; i < lines.rows(); i++) {
            double[] val = lines.get(i, 0);
            double diff_x = (int)val[0] - (int)val[2];
            double diff_y = (int)val[1] - (int)val[3];
            if(diff_y != 0 || diff_x != 0 ){
                Imgproc.line(result, new Point(val[0], val[1]), new Point(val[2], val[3]), new Scalar(0, 0, 255), 2);
                //Logger.debug("diff_X: " +diff_x + " diff_y: " +  diff_y + " X:" + (int)val[0]);

                double slope = diff_y/diff_x;

                
                double angle = Math.atan(diff_y/diff_x);
                double intercept = val[1]-(slope*val[0]);
                
                if(Math.abs(angle)  <= 0.9 &&  Math.abs(angle) >= 0.5){
                    continue;
                }
                if(diff_x == 0){
                    intercept = val[0];
                }
                
                if(first_angle == 0){
                    first_angle = angle;
                }
                else if(slope < (first_angle+0.2) && slope > (first_angle-0.2)){
                    
                }
                else if(second_angle == 0){
                    second_angle = angle;
                }

                for(int m = 0; m < 4; m++){
                    
                    double temp_val = intercept_list.get(m);
                    double temp_angle_val = intercept_list.get(m);

                    
                    if(temp_val == 0){
                        intercept_list.set(m,intercept);
                        angle_list.set(m,angle);
                        
                        listOfInter[m].add(intercept);
                        listOfLists[m].add(slope);
                        listOfAngle[m].add(angle);
                        average_XCheck.set(m,val[0]);
                        average_YCheck.set(m,val[1]);
                        
                        if(Math.abs(angle) > 0.7){
                            big_angle.set(m,1.0);
                        }

                        break;
                    }
                    

                    double percent_x = (Math.abs(val[0] - average_XCheck.get(m))/((val[0] + average_XCheck.get(m))/2))*100;
                    
                    double percent_y = (Math.abs(val[1] - average_YCheck.get(m))/((val[1] + average_YCheck.get(m))/2))*100;
                    
                    if(big_angle.get(m) == 0){
                      if(Math.abs(angle) < 0.5){
                        if(Math.abs(percent_y) < 40.0){
                            listOfInter[m].add(intercept);
                            listOfLists[m].add(slope);
                            if(slope == Double.POSITIVE_INFINITY){
                                listOfInf[m].add(intercept);
                            }
                            listOfAngle[m].add(Math.abs(angle));

                            break;
                        }
                      }
                    }
                    else{
                        if(Math.abs(angle) > 0.5){

                            if(Math.abs(percent_x) < 40.0){
                                listOfInter[m].add(intercept);
                                listOfLists[m].add(slope);
                                if(slope == Double.POSITIVE_INFINITY){
                                    listOfInf[m].add(intercept);
                                }
                                listOfAngle[m].add(Math.abs(angle));
                                //Logger.debug("m: " + m + "  firstVal   intercept:" +intercept + " slope:" + slope + " angle:" + angle);

                                break;
                            }
                        }
                    }
                }
            }
        }
        Logger.debug("next");
        
        for(int m = 0; m < 4; m++){
            double temp_avg_slope = 0.0;
            double temp_avg_inter = 0.0;
            double temp_avg_angle = 0.0;
            double temp_avg_inf = 0.0;

            for(int n = 0; n < listOfLists[m].size(); n++){

                temp_avg_slope += listOfLists[m].get(n);
                temp_avg_inter += listOfInter[m].get(n);
                temp_avg_angle += listOfAngle[m].get(n);

                
            }

            for(int n = 0; n < listOfInf[m].size(); n++){
                Logger.debug("next: " + n);

                temp_avg_inf += listOfInf[m].get(n);
            }

            average_slope.set(m, (temp_avg_slope/listOfLists[m].size()));
            average_inter.set(m, (temp_avg_inter/listOfInter[m].size()));
            average_angle.set(m, (temp_avg_angle/listOfInter[m].size()));
            average_inf.set(m, (temp_avg_inf/listOfInf[m].size()));

        }

        /* Draw the four averaged lines which should represent 
         * the edges of the tile */
        for(int i = 0; i < 4; i++){
	    //Logger.debug("slope: " +average_slope.get(i) + " inter: " +  average_inter.get(i));
            double inf = Double.POSITIVE_INFINITY;
            if(Math.abs(average_slope.get(i)) != inf && !Double.isNaN(average_slope.get(i))){
                Imgproc.line(result,new Point(0,0*average_slope.get(i) + average_inter.get(i)), new Point(1920,1920*average_slope.get(i) + average_inter.get(i)),new Scalar(0, 100, 0),5);
            }
            else{
                //Logger.debug("inf: " +average_inf.get(i) + " inter: " +  average_inter.get(i));
                average_inter.set(i, average_inf.get(i));
                Imgproc.line(result,new Point(average_inf.get(i),0.0), new Point(average_inf.get(i),1920 ),new Scalar(0, 100, 0),5);
            }
        }

        Imgcodecs.imwrite("cornerLog//"+ noSpaceName + image_iteration_name + "_result.jpg", result);

        toPrint = "List of Points\n";
        bytesArray = toPrint.getBytes();
        oFile.write(bytesArray);
        oFile.flush();
        
        
        for(int i = 0; i < average_slope.size(); i++){
            for(int j = 0; j < average_slope.size(); j++){
                if(i == j){
                    continue;
                }
       
                double inf = Double.POSITIVE_INFINITY;
                double x = 0;
                double y = 0;
                

                if(Math.abs(average_slope.get(i)) != inf){
                    x = (average_inter.get(j) - average_inter.get(i))/(average_slope.get(i) - average_slope.get(j));
                    y = x*average_slope.get(i) +average_inter.get(i);

                }
                else{
                    x = average_inter.get(i);
                    y = x*average_slope.get(j) +average_inter.get(j);
                    
                }
                
                
                Pair temp = new Pair();
                temp.x = x;
                temp.y = y;

                int cornerFlag = 0;
                if(x > 0.0 && x < 2500.0){
                    if(y > 0.0 && y < 1920.0){
                        for(int m = 0; m < corner_list.size(); m++){
                            double existingX = corner_list.get(m).x;
                            double existingY = corner_list.get(m).y;
                            double existingDiffX = existingX - x;
                            double existingDiffY = existingY - y;
                            double totalDiff = Math.abs(existingDiffX) + Math.abs(existingDiffY);
                            if(totalDiff < 50){
                                cornerFlag = 1;
                            }
                        }
                        if(cornerFlag == 0){
                            corner_list.add(temp);
                            Imgproc.circle(result, new Point(x, y), 5, new Scalar(0, 0,  100), Core.FILLED);
                            double slope_dif = 0.0;
                            //if(Math.abs(average_slope.get(i)) != inf){
                            slope_dif = average_angle.get(i) - average_angle.get(j);
                            //}
                            //else{
                                //slope_dif = -1*average_angle.get(j);                                
                            //}
                            
                            //Logger.debug("corner: " + Math.abs(slope_dif));
                            //Logger.debug("x: " +x + " y: " + y);

                              
                            toPrint = "x," + x + ", y," + y + "\n";
                            bytesArray = toPrint.getBytes();
                            oFile.write(bytesArray);
                            oFile.flush();
                        }
                    }
                    
                }
                
                    
     
            }
            
        }
        
        toPrint = "Tile Dimensions\n";
        bytesArray = toPrint.getBytes();
        oFile.write(bytesArray);
        oFile.flush();


        double smallest_angle = 10000000.0;
        double avg_angles = 0.0;

        if(corner_list.size() != 4){
	    Logger.debug("Not enough points");
                oFile.close();
            return new Result(result);
        }
        else{
            four_point_flag = 1;
            //break;
        }
    //}
        
        List<Pad> pad_list = noz.getPart().getPackage().getFootprint().getPads();
        String package_name = noz.getPart().getPackage().getId();

        
        for(int i = 0; i < pad_list.size(); i++){
            
            String temp_name = pad_list.get(i).getName();
            
            if(temp_name.equals("top")){
                top_len = pad_list.get(i).getWidth();
                Logger.debug("top pad:" + top_len);
            }
            
            if(temp_name.equals("bot")){
                bot_len = pad_list.get(i).getWidth();
            }

            if(temp_name.equals("height")){
                height = pad_list.get(i).getWidth();
            }

            if(temp_name.equals("orient")){
                orient = pad_list.get(i).getWidth();
            }
            
            if(temp_name.equals("special")){
                special = pad_list.get(i).getWidth();
            }
        }
        

        double topMM = top_len;
        double botMM = bot_len;
        double heightMM = height;

        
        

        RotatedRect minRect = (RotatedRect) pipeline.getResult("rect").getModel();
        
        Point centerPoint = minRect.center;
        double c_y = centerPoint.y;
        double c_x = centerPoint.x;
        
        int center_x = (int)c_x;
        int center_y = (int)c_y;
        
        RotatedRect bad_rect = new RotatedRect();

        ArrayList<CornerPoint> point_list = new ArrayList<CornerPoint>();
        
        for(int i = 0; i < corner_list.size(); i++){

            point_list.add(new CornerPoint((float)corner_list.get(i).x,(float)corner_list.get(i).y));
        }
        
        for(int i = 0; i < corner_list.size(); i++){
            point_list.get(i).chooseFromThree(point_list.get((i+1)%corner_list.size()),point_list.get((i+2)%corner_list.size()),point_list.get((i+3)%corner_list.size()));
        }
        
        point_list.get(0).printLines();
        
        ArrayList<LineLocal> lines_result = new ArrayList<LineLocal>();
        
        LineLocal max = new LineLocal(new CornerPoint(0,0), new CornerPoint(0,0));
        
        //Logger.debug("longest side: " + longestSide);
        String lss = "top";//longestSide;
        Logger.debug(lss);
        Logger.debug(lss.equals("top"));
        if (lss.equals("right")){
            Imgproc.line (
                        result,                    //Matrix obj of the image
                        new Point(center_x + ((height/unitPPY)/2), center_y + ((top_len/unitPPX)/2)),        //p1
                        new Point(center_x + ((height/unitPPY)/2), center_y - ((top_len/unitPPX)/2)),       //p2
                        new Scalar(0, 255, 255),     //Scalar object for color
                        2                          //Thickness of the line
                        );
            Imgproc.line (
                        result,                    //Matrix obj of the image
                        new Point(center_x - ((height/unitPPY)/2), center_y + ((bot_len/unitPPX)/2)),        //p1
                        new Point(center_x - ((height/unitPPY)/2), center_y - ((bot_len/unitPPX)/2)),       //p2
                        new Scalar(0, 255, 255),     //Scalar object for color
                        2                          //Thickness of the line
                        );
            Imgproc.line (
                        result,                    //Matrix obj of the image
                        new Point(center_x - ((height/unitPPY)/2), center_y + ((bot_len/unitPPX)/2)),        //p1
                        new Point(center_x + ((height/unitPPY)/2), center_y + ((top_len/unitPPX)/2)),       //p2
                        new Scalar(0, 255, 255),     //Scalar object for color
                        2                          //Thickness of the line
                        );
            Imgproc.line (
                        result,                    //Matrix obj of the image
                        new Point(center_x + ((height/unitPPY)/2), center_y - ((top_len/unitPPX)/2)),        //p1
                        new Point(center_x - ((height/unitPPY)/2), center_y - ((bot_len/unitPPX)/2)),       //p2
                        new Scalar(0, 255, 255),     //Scalar object for color
                        2                          //Thickness of the line
                        );
        }
        else if(lss.equals("bot")){
            Imgproc.line (
                result,                    //Matrix obj of the image
                new Point(center_x - ((top_len/unitPPY)/2), center_y + ((height/unitPPX)/2)),        //p1
                new Point(center_x + ((top_len/unitPPY)/2), center_y + ((height/unitPPX)/2)),       //p2
                new Scalar(0, 255, 255),     //Scalar object for color
                2                          //Thickness of the line
                );
  Imgproc.line (
                result,                    //Matrix obj of the image
                new Point(center_x - ((top_len/unitPPY)/2), center_y + ((height/unitPPX)/2)),        //p1
                new Point(center_x - ((bot_len/unitPPY)/2), center_y - ((height/unitPPX)/2)),       //p2
                new Scalar(0, 255, 255),     //Scalar object for color
                2                         //Thickness of the line
                );
  Imgproc.line (
                result,                    //Matrix obj of the image
                new Point(center_x + ((top_len/unitPPY)/2), center_y + ((height/unitPPX)/2)),        //p1
                new Point(center_x + ((bot_len/unitPPY)/2), center_y - ((height/unitPPX)/2)),       //p2
                new Scalar(0, 255, 255),     //Scalar object for color
                2                         //Thickness of the line
                );
  Imgproc.line (
                result,                    //Matrix obj of the image
                new Point(center_x + ((bot_len/unitPPY)/2), center_y - ((height/unitPPX)/2)),        //p1
                new Point(center_x - ((bot_len/unitPPY)/2), center_y - ((height/unitPPX)/2)),       //p2
                new Scalar(0, 255, 255),     //Scalar object for color
                2                          //Thickness of the line
                );
           
        }
        else if(lss.equals("top")){
            Logger.debug("line1: " + (center_x - ((height/unitPPY)/2)) + " " + (center_y + ((top_len/unitPPX)/2)));

            Imgproc.line (
                result,                    //Matrix obj of the image
                new Point(center_x - ((top_len/unitPPY)/2), center_y - ((height/unitPPX)/2)),        //p1
                new Point(center_x + ((top_len/unitPPY)/2), center_y - ((height/unitPPX)/2)),       //p2
                new Scalar(0, 255, 255),     //Scalar object for color
                2                          //Thickness of the line
                );
  Imgproc.line (
                result,                    //Matrix obj of the image
                new Point(center_x - ((bot_len/unitPPY)/2), center_y + ((height/unitPPX)/2)),        //p1
                new Point(center_x - ((top_len/unitPPY)/2), center_y - ((height/unitPPX)/2)),       //p2
                new Scalar(0, 255, 255),     //Scalar object for color
                2                          //Thickness of the line
                );
  Imgproc.line (
                result,                    //Matrix obj of the image
                new Point(center_x + ((bot_len/unitPPY)/2), center_y + ((height/unitPPX)/2)),        //p1
                new Point(center_x + ((top_len/unitPPY)/2), center_y - ((height/unitPPX)/2)),       //p2
                new Scalar(0, 255, 255),     //Scalar object for color
                2                          //Thickness of the line
                );
  Imgproc.line (
                result,                    //Matrix obj of the image
                new Point(center_x + ((bot_len/unitPPY)/2), center_y + ((height/unitPPX)/2)),        //p1
                new Point(center_x - ((bot_len/unitPPY)/2), center_y + ((height/unitPPX)/2)),       //p2
                new Scalar(0, 255, 255),     //Scalar object for color
                2                          //Thickness of the line
                );
        }
        else if(lss.equals("left")){
            Imgproc.line (
                            result,                    //Matrix obj of the image
                            new Point(center_x - ((height/unitPPY)/2), center_y + ((top_len/unitPPX)/2)),        //p1
                            new Point(center_x - ((height/unitPPY)/2), center_y - ((top_len/unitPPX)/2)),       //p2
                            new Scalar(0, 255, 255),     //Scalar object for color
                            2                          //Thickness of the line
                            );
            Imgproc.line (
                            result,                    //Matrix obj of the image
                            new Point(center_x + ((height/unitPPY)/2), center_y + ((bot_len/unitPPX)/2)),        //p1
                            new Point(center_x + ((height/unitPPY)/2), center_y - ((bot_len/unitPPX)/2)),       //p2
                            new Scalar(0, 255, 255),     //Scalar object for color
                            2                          //Thickness of the line
                            );
            Imgproc.line (
                            result,                    //Matrix obj of the image
                            new Point(center_x - ((height/unitPPY)/2), center_y + ((top_len/unitPPX)/2)),        //p1
                            new Point(center_x + ((height/unitPPY)/2), center_y + ((bot_len/unitPPX)/2)),       //p2
                            new Scalar(0, 255, 255),     //Scalar object for color
                            2                          //Thickness of the line
                            );
            Imgproc.line (
                            result,                    //Matrix obj of the image
                            new Point(center_x + ((height/unitPPY)/2), center_y - ((bot_len/unitPPX)/2)),        //p1
                            new Point(center_x - ((height/unitPPY)/2), center_y - ((top_len/unitPPX)/2)),       //p2
                            new Scalar(0, 255, 255),     //Scalar object for color
                            2                          //Thickness of the line
                            );
        }
        
        
        
        //Circle points
        for(int i = 0; i < point_list.size(); i++){
            Imgproc.circle (
                            save,                 //Matrix obj of the image
                            new Point(point_list.get(i).x, point_list.get(i).y),    //Center of the circle
                            3,                    //Radius
                            new Scalar(0, 0, 255),  //Scalar object for color
                            3                      //Thickness of the circle
                            );
            
            for(int m = 0; m < point_list.get(i).returnLines().size(); m++){
                lines_result.add(point_list.get(i).returnLines().get(m));
            }
        }
        

        
        
        double to_correct = 0;
        double best = 10000000;
        
        double bestOne = 0;
        double bestTwo = 0;
        double closest = 1000000000;
        
        LineLocal largeLine = new LineLocal();
        LineLocal smallLine = new LineLocal();
        
        
        //Parallel calc
        for(int i = 0; i < lines_result.size(); i++){

            if(lines_result.get(i).getLen() > max.getLen()){
                max = lines_result.get(i);
            }
            
            CornerPoint start = lines_result.get(i).start;
            CornerPoint end = lines_result.get(i).end;
            
            double y_diff = lines_result.get(i).start.y - lines_result.get(i).end.y;
            double x_diff = lines_result.get(i).start.x - lines_result.get(i).end.x;
            
            double line_len_check = Math.sqrt((y_diff * y_diff) + (x_diff * x_diff));
            int top_flag = 0;
            int bot_flag = 0;

            /* Write debugging info to file */
            if(Math.abs(y_diff) > Math.abs(x_diff)){
                if(lines_result.get(i).start.x > center_x){
                    double measured = (line_len_check*unitPPX);
                    double theoretical = calcDistance((center_x + ((bot_len/unitPPY)/2))*unitPPY,( center_y + ((height/unitPPX)/2))*unitPPY,(center_x + ((top_len/unitPPY)/2))*unitPPY, (center_y - ((height/unitPPX)/2))*unitPPY);
                    toPrintRight = "right, measured px: ," + line_len_check + ", measured mm: ,"+(line_len_check*unitPPX) + ", theoretical mm: ," + theoretical + ", diff: ,"+ (measured-theoretical) + "\n";
                }
                else{
                    double measured = (line_len_check*unitPPX);
                    double theoretical = calcDistance((center_x - ((bot_len/unitPPY)/2))*unitPPY, (center_y + ((height/unitPPX)/2))*unitPPY,(center_x - ((top_len/unitPPY)/2))*unitPPY,( center_y - ((height/unitPPX)/2))*unitPPY);
                    toPrintLeft = "left, measured px: ," + line_len_check+ ", measured mm: ,"+(line_len_check*unitPPX) + ", theoretical mm: ," + theoretical + ", diff: ,"+ (measured-theoretical) +"\n";
                }
            }
            else{
                if(lines_result.get(i).start.y > center_y){
                    double measured = (line_len_check*unitPPY);
                    double theoretical = calcDistance((center_x + ((bot_len/unitPPY)/2))*unitPPY, (center_y + ((height/unitPPX)/2))*unitPPY, (center_x - ((bot_len/unitPPY)/2))*unitPPY, (center_y + ((height/unitPPX)/2))*unitPPY);
                    toPrintBot = "bot, measured px: ," + line_len_check + ", measured mm: ,"+(line_len_check*unitPPY)+ ", theoretical mm: ," + theoretical + ", diff: ,"+ (measured-theoretical) +"\n";
                    top_flag = 1;
                }
                else{
                    double measured = (line_len_check*unitPPY);
                    double theoretical = calcDistance((center_x - ((top_len/unitPPY)/2)) * unitPPY,(center_y - ((height/unitPPX)/2)) * unitPPY ,(center_x + ((top_len/unitPPY)/2)) * unitPPY,(center_y - ((height/unitPPX)/2)) * unitPPY);
                    toPrintTop = "top, measured px: ," + line_len_check + ", measured mm: ,"+(line_len_check*unitPPY) + ", theoretical mm: ," + theoretical + ", diff: ,"+ (measured-theoretical) +"\n";
                    bot_flag = 1;
                }
            }
            //Logger.debug("(" + start.x + "," + start.y + ")(" + end.x + "," + end.y + ")");
		    //


            //if(special == 0){
            /*
            if( line_len_check*unitPPX <  (topMM - allowedError) ||  line_len_check*unitPPX  >  (topMM + allowedError)){
                if( line_len_check*unitPPX <  (botMM - allowedError) ||  line_len_check*unitPPX >  (botMM + allowedError)){
                    if( line_len_check*unitPPX <  (heightMM - allowedError) ||  line_len_check*unitPPX >  (heightMM + allowedError)){
                        Logger.debug("Package Rejection");
                        throw new Exception("Package Rejection");
                    }
                }
            }
            */

            if(line_len_check*unitPPX <  (topMM - allowedError) ||  line_len_check*unitPPX  >  (topMM + allowedError)){
                if(top_flag == 1){
                        Logger.debug("topMM :" + topMM);
                        Logger.debug("mm llc :" + (line_len_check * unitPPX) + " low_error:" + (topMM - allowedError) + " high_error:" +(topMM + allowedError));
                        Logger.debug("Package Rejection");
                        throw new Exception("Package Rejection");                   
                }
            }
            
            if(line_len_check*unitPPX <  (botMM - allowedError) ||  line_len_check*unitPPX  >  (botMM + allowedError)){
                if(bot_flag == 1){
                        Logger.debug("botMM :" + botMM);
                        Logger.debug("mm llc :" + (line_len_check * unitPPX) + " low_error:" + (botMM - allowedError) + " high_error:"+ (botMM + allowedError));
                        Logger.debug("Package Rejection");
                        throw new Exception("Package Rejection");                   
                }
            }
            //}
            
	    if(x_diff != 0){
            to_correct = (Math.atan(y_diff/x_diff)/Math.PI)*180;
        }
	    else{
		    to_correct = x_diff;
        }
            
            for(int y = 0; y < lines_result.size(); y++){
                if(y != i){
                    
                    LineLocal inner = lines_result.get(y);
                    
                    CornerPoint start_inner = lines_result.get(y).start;
                    CornerPoint end_inner = lines_result.get(y).end;
                    double y_diff_inner = lines_result.get(y).start.y - lines_result.get(y).end.y;
                    double x_diff_inner = lines_result.get(y).start.x - lines_result.get(y).end.x;
                    
                    double to_correct_inner = 0;
		            if(x_diff_inner != 0){
	                    to_correct_inner = (Math.atan(y_diff_inner/x_diff_inner)/Math.PI)*180;
                    }
	                else{
			            to_correct_inner = 0;
		            }
		   
                    
                    
                    if(Math.abs(Math.abs(to_correct) - Math.abs(to_correct_inner)) < Math.abs(closest)){
                        closest = Math.abs(Math.abs(to_correct) - Math.abs(to_correct_inner));
                        best = (to_correct +  to_correct_inner)/2; 
                        
                        if(lines_result.get(i).getLen() > lines_result.get(y).getLen()){
                            largeLine = lines_result.get(i);
                            smallLine = lines_result.get(y);
                        }else{
                            largeLine = lines_result.get(y);
                            smallLine = lines_result.get(i);
                        }
                    }
                }
            }
        }

        bytesArray = toPrintTop.getBytes();
        oFile.write(bytesArray);
        oFile.flush();
        bytesArray = toPrintBot.getBytes();
        oFile.write(bytesArray);
        oFile.flush();
        bytesArray = toPrintRight.getBytes();
        oFile.write(bytesArray);
        oFile.flush();
        bytesArray = toPrintLeft.getBytes();
        oFile.write(bytesArray);
        oFile.flush();        

        toPrint = "Package: "+ package_name +"\n";
        toPrint += "top:,"+ top_len +"\n";
        toPrint += "bot:,"+ bot_len +"\n";
        toPrint += "height:,"+ height +"\n";
        bytesArray = toPrint.getBytes();
        oFile.write(bytesArray);
        oFile.flush();

        toPrint = "\n\n";
        bytesArray = toPrint.getBytes();
        oFile.write(bytesArray);
        oFile.flush();
        
        double x_tot = 0;
        double y_tot = 0;
        
        for(int i = 0; i < corner_list.size(); i++){
            x_tot += corner_list.get(i).x;
            y_tot += corner_list.get(i).y;
        }
        
        /*Create the output obj which will be used for orientation correction*/
        RotatedRect rect = new RotatedRect(new Point(x_tot/corner_list.size(),y_tot/corner_list.size()), new Size(100,100), best);


        oFile.close();
        return new Result(result,rect);
    }
}
