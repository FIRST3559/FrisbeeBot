/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 * Original Code derived from Team 178.  Modified by Team 3559.
 */
package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport; 
/**
 *
 * @author 300540
 */
public class CameraFHS
{
private ImageAnalysis analysis;
private AxisCamera axis;
private Drivetrain drivetrain;
public CameraFHS(Drivetrain drivetrain)
{
    axis = AxisCamera.getInstance();
    this.drivetrain = drivetrain;
    analysis = new ImageAnalysis(axis);
}
public void centerOnFirstTarget() throws AxisCameraException, NIVisionException
{
    analysis.updateImage();
    ParticleAnalysisReport[] report = analysis.findRectangles();
    double xNormal;
    if(report.length >= 1)
    {
        xNormal = report[0].center_mass_x_normalized;
        System.out.println(xNormal);
        //drivetrain.frontLeftSet(xNormal*0.3);
        //drivetrain.frontRightSet(xNormal*0.3); 
        drivetrain.rearLeftSet(xNormal*0.3); 
        drivetrain.rearRightSet(xNormal*0.3);  
    }
    else
    {
        System.out.println("No Valid Targets Found!");
        //drivetrain.frontLeftSet(0.0); 
        //drivetrain.frontRightSet(0.0);
        drivetrain.rearLeftSet(0.0);  
        drivetrain.rearRightSet(0.0);   
    }
}

}

