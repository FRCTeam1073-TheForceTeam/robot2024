// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;


// import java.io.UnsupportedEncodingException;
// import java.util.ArrayList;

// //import com.ctre.phoenix.sensors.Pigeon2;

// //import edu.wpi.first.math.wpiilibj2.command.CommandBase;  // NSargent: deprecated I guess?
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Camera;
// import frc.robot.subsystems.SerialComms;


// public class GetTagData extends Command {

//   // Saved construction parameters:
//   private Camera camera;

//   // State variables for execution:
//   // not sure we need any of these

//   public GetTagData(Camera camera) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.camera = camera;  // NSargent: not 100% sure we actually need this
//     addRequirements(camera); // NSargent: no idea what this does. check it out.
//   }

//   // Called when the command is initially scheduled.
//   /**
//    * The robot looks for the closest ID, and saves that ID
//    * Also the Bling is cleared for later steps
//    */
//   @Override
//   public void initialize() {
//     System.out.println("Initializing");
//     // NSargent: not sure if we'll need this
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   /**
//    * gets apriltag data
//    * @param x - parameter descriptions
//    * @return return descriptions
//    */
//   @Override
//   public void execute() {
//     System.out.println("DATA");
//     try {
//       ArrayList<Byte> thedata = SerialComms.getVisionData("1,a\n".getBytes("ASCII"));
//       System.out.println(thedata);

//     } catch (UnsupportedEncodingException e) {
//       // TODO Auto-generated catch block
//       e.printStackTrace();
//     }
//   }

//   // Called once the command ends or is interrupted.
//   /**
//    * description
//    */
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//       return true;
//   } 
// }