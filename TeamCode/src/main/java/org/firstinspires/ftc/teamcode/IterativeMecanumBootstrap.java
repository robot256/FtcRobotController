/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Tom's Simple Mecanum", group="Iterative Opmode")
//@Disabled
public class IterativeMecanumBootstrap extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    List<LynxModule> allHubs;
    private DcMotorEx mecanumFrontLeft = null;
    private DcMotorEx mecanumFrontRight = null;
    private DcMotorEx mecanumBackLeft = null;
    private DcMotorEx mecanumBackRight = null;

    private FileWriter fileStream = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Get the motor objects for Mecanum Drive wheesl
        mecanumFrontLeft =  hardwareMap.get(DcMotorEx.class, "fl");
        mecanumFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumFrontRight = hardwareMap.get(DcMotorEx.class, "fr");
        mecanumFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //mecanumFrontRight.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumBackLeft =  hardwareMap.get(DcMotorEx.class, "bl");
        mecanumBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumBackRight = hardwareMap.get(DcMotorEx.class, "br");
        mecanumBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //mecanumBackRight.setDirection(DcMotorEx.Direction.REVERSE);

        // Configure the hubs for manual bulk reads, so we can query core APIS multiple times per
        // loop without triggering more serial data transfers
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();

        File file = new File(Environment.getExternalStorageDirectory(), "thisFileShouldntExist.csv");
        try {
            fileStream = new FileWriter(file);
            fileStream.write("Time,JoyX,JoyY,JoyR\n");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }

    private double joystickExponentialTransform(double input) {
        return Math.signum(input) * Math.expm1(5*Math.abs(input)) / Math.expm1(5);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }


        // Mecanum joystick control with scaling per axis
        double translateForwardCommand = joystickExponentialTransform(-gamepad1.left_stick_y);
        double translateRightCommand = joystickExponentialTransform(gamepad1.left_stick_x);
        double rotateLeftCommand = joystickExponentialTransform(-gamepad1.right_stick_x);
        // Calculate wheel speeds for cross-oriented mecanum wheels
        double frontLeftVelocity = translateForwardCommand + translateRightCommand - rotateLeftCommand;
        double backLeftVelocity = translateForwardCommand - translateRightCommand - rotateLeftCommand;
        double backRightVelocity = translateForwardCommand + translateRightCommand + rotateLeftCommand;
        double frontRightVelocity = translateForwardCommand - translateRightCommand + rotateLeftCommand;
        double maxVelocity = Math.max(Math.abs(frontLeftVelocity), Math.max(Math.abs(backLeftVelocity),
                Math.max(Math.abs(backRightVelocity), Math.abs(frontRightVelocity))));
        if(maxVelocity > 1) {
            // Scale translation and rotation evenly for driver control
            frontLeftVelocity /= maxVelocity;
            backLeftVelocity /= maxVelocity;
            backRightVelocity /= maxVelocity;
            frontRightVelocity /= maxVelocity;
        }

        // Send commands to motors
        mecanumFrontLeft.setPower(frontLeftVelocity);
        mecanumBackLeft.setPower(backLeftVelocity);
        mecanumBackRight.setPower(backRightVelocity);
        mecanumFrontRight.setPower(frontRightVelocity);

        telemetry.addData("Runtime:", runtime.toString());
        telemetry.addData("Joysticks:", String.format("X%1.4f Y%1.4f R%1.4f",translateRightCommand,translateForwardCommand,rotateLeftCommand));
        telemetry.addData("Mecanum:", String.format("FL%1.4f BL%1.4f BR%1.4f FR%1.4f",frontLeftVelocity,backLeftVelocity,backRightVelocity,frontRightVelocity));

        try {
            fileStream.write(String.format("%4.2f,%1.4f,%1.4f,%1.4f\n",runtime.milliseconds(), translateRightCommand,translateForwardCommand,rotateLeftCommand));
            fileStream.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        mecanumFrontLeft.setVelocity(0);
        mecanumBackLeft.setVelocity(0);
        mecanumBackRight.setVelocity(0);
        mecanumFrontRight.setVelocity(0);
        try {
            fileStream.write("End of file");
            fileStream.flush();
            fileStream.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

}
