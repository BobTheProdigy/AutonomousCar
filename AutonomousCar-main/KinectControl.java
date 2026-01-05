package edu.elon.robotics.auto;

import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.net.DatagramPacket;
import java.net.DatagramSocket;

@Configurable
@Autonomous(name = "KinectControl")
public class KinectControl extends AutoCommon {

    private DatagramSocket socket;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        try {
            socket = new DatagramSocket(5005);
            socket.setSoTimeout(10);   // non-blocking
        } catch (Exception e) {
            telemetry.addData("UDP", "Failed to open port");
            telemetry.update();
        }

        waitForStart();

        robot.imu.resetYaw();

        while (opModeIsActive()) {

            // 1. Get latest angle from Python
            double targetAngle = readAngleFromPython();

            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Heading", robot.getHeading());
            telemetry.update();

            // 2. Use proportional turning (smooth)
            double error = targetAngle - robot.getHeading();

            // normalize error to [-180, 180]
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            double Kp = 0.015;   // tuning parameter
            double turnPower = Kp * error;

            // limit power
            turnPower = Math.max(-0.25, Math.min(0.25, turnPower));

            // 3. Drive forward while correcting heading
            robot.startMove(0.25, 0, turnPower);
        }

        socket.close();
        robot.startMove(0, 0, 0);
    }

    private double readAngleFromPython() {
        try {
            byte[] buf = new byte[64];
            DatagramPacket packet = new DatagramPacket(buf, buf.length);
            socket.receive(packet);
            String msg = new String(packet.getData(), 0, packet.getLength()).trim();
            return Double.parseDouble(msg);
        } catch (Exception e) {
            return 0.0;   // no new data? keep going straight
        }
    }
}




//package edu.elon.robotics.auto;
//
//import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
////import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//import java.net.DatagramPacket;
//import java.net.DatagramSocket;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import java.net.DatagramPacket;
//import java.net.DatagramSocket;
//
//@Configurable
//@Autonomous(name = "KinectControl")
//public class KinectControl extends AutoCommon {
//
//    private DatagramSocket socket;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        super.runOpMode();
//
//        try {
//            socket = new DatagramSocket(5005);   // Listen on same port as Python
//            socket.setSoTimeout(50);             // prevent blocking
//        } catch (Exception e) {
//            telemetry.addData("UDP", "Failed to open port");
//        }
//
//        waitForStart();
//        robot.imu.resetYaw();
//        robot.resetDriveEncoders();
//
//        robot.startMove(0.2, 0, 0);
//
//        while (opModeIsActive()) {
//            double turnAngle = readAngleFromPython();
//            telemetry.addData("Received Angle", turnAngle);
//            telemetry.update();
//            turnIMU(turnAngle, 0.2);
//        }
//        socket.close();
//    }
//
//    private double readAngleFromPython() {
//        try {
//            byte[] buf = new byte[64];
//            DatagramPacket packet = new DatagramPacket(buf, buf.length);
//            socket.receive(packet);
//            String msg = new String(packet.getData(), 0, packet.getLength()).trim();
//            return Double.parseInt(msg);
//        } catch (Exception e) {
//            return 0.0; // no data? go straight
//        }
//    }
//}