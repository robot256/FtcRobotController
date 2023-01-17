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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="Tom's Ground-Up Powerplay", group="Iterative Opmode")
//@Disabled
public class coachCode1 extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    List<LynxModule> allHubs;
    private DcMotorEx mecanumFrontLeft = null;
    private DcMotorEx mecanumFrontRight = null;
    private DcMotorEx mecanumBackLeft = null;
    private DcMotorEx mecanumBackRight = null;

    private DcMotorEx fetcherMotor = null;
    private DigitalChannel fetcherLowerLimit = null;
    private Servo leverServo = null;
    private Servo grabberServo = null;

    private DcMotorEx liftMotor = null;
    private DigitalChannel liftLowerLimit = null;
    private Servo placerServo = null;
    private Servo dropperServo = null;

    private final double FETCHER_MAX_TICKS = 4000;
    private final double FETCHER_SAFE_TICKS = 250;
    private final double FETCHER_SUBSTATION_TICKS = 2000;
    private final double LEVER_HOME = 0;
    private final double LEVER_HORIZONTAL = 0.75;
    private final double LEVER_GROUND = 1;
    private final double GRABBER_OPEN = 0;
    private final double GRABBER_CLOSE = 1;

    private final double LIFT_MAX_TICKS = 2500;
    private final double LIFT_SAFE_TICKS = 175;
    private final double LIFT_LOW_TICKS = 1000;
    private final double LIFT_MEDIUM_TICKS = 1800;
    private final double LIFT_HIGH_TICKS = 2200;
    private final double PLACER_COLLECT = 0;
    private final double PLACER_READY = 0.25;
    private final double PLACER_BRACE = 0.5;
    private final double PLACER_SCORE = 0.75;
    private final double DROPPER_STOP = 0.5;
    private final double DROPPER_COLLECT = 0.25;
    private final double DROPPER_EJECT = 0.75;

    private boolean previousFetcherLimitPressed = false;
    private boolean previousLiftLimitPressed = false;
    private boolean fetcherHomed = false;
    private boolean liftHomed = false;
    private boolean fetcherManualControl = false;
    private boolean liftManualControl = false;
    private boolean fetcherAutoControl = false;
    private boolean liftAutoControl = false;
    private double fetcherPositionCommand = 0;
    private double liftPositionCommand = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Get the motor objects for Mecanum Drive wheels
        mecanumFrontLeft =  hardwareMap.get(DcMotorEx.class, "fl");
        mecanumFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //mecanumFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumFrontRight = hardwareMap.get(DcMotorEx.class, "fr");
        mecanumFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumFrontRight.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumBackLeft =  hardwareMap.get(DcMotorEx.class, "bl");
        mecanumBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumBackRight = hardwareMap.get(DcMotorEx.class, "br");
        mecanumBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //mecanumBackRight.setDirection(DcMotorEx.Direction.REVERSE);

        // Get hardware objects for Horizontal subsystem
        fetcherMotor = hardwareMap.get(DcMotorEx.class, "fetchMotor");
        fetcherLowerLimit = hardwareMap.get(DigitalChannel.class, "fetchLimitSwitch");
        leverServo = hardwareMap.get(Servo.class, "lever");
        grabberServo = hardwareMap.get(Servo.class, "grabber");

        // Get hardware objects for Vertical subsystem
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftLowerLimit = hardwareMap.get(DigitalChannel.class, "liftLimitSwitch");
        placerServo = hardwareMap.get(Servo.class, "placer");
        dropperServo = hardwareMap.get(Servo.class, "dropper");

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
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        leverServo.setPosition(LEVER_HOME);
        grabberServo.setPosition(GRABBER_CLOSE);
        placerServo.setPosition(PLACER_READY);
        dropperServo.setPosition(DROPPER_STOP);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Time in init", runtime.seconds());

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        // Read sensors
        double fetcherEncoder = fetcherMotor.getCurrentPosition();
        double liftEncoder = liftMotor.getCurrentPosition();
        boolean fetcherLimitPressed = !fetcherLowerLimit.getState();
        boolean liftLimitPressed = !liftLowerLimit.getState();

        // Read the servo current commands
        double previousLeverCommand = leverServo.getPosition();
        double previousGrabberCommand = grabberServo.getPosition();
        double previousPlacerCommand = placerServo.getPosition();
        double previousDropperCommand = dropperServo.getPosition();

        // Buttons to control the servos
        // Placer moves while button is held, returns to Brace position when released
        // While in Collect state, dropper collects automatically
        // Otherwise, dropper can be used independently.
        // Dropper stops when dropper & placerCollect buttons released.
        boolean dropperToEjectCommand = gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0;
        boolean dropperToIngestCommand = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean placerToScoreCommand = gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0;
        boolean placerToCollectCommand = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean dropperCommandedByMacro = false;
        if(placerToScoreCommand && !placerToCollectCommand) {
            placerServo.setPosition(PLACER_SCORE);
        } else if(placerToCollectCommand && !placerToScoreCommand) {
            placerServo.setPosition(PLACER_COLLECT);
            dropperServo.setPosition(DROPPER_COLLECT);
            dropperCommandedByMacro = true;
        } else {
            placerServo.setPosition(PLACER_BRACE);
        }
        if(!dropperCommandedByMacro) {
            if(dropperToEjectCommand && !dropperToIngestCommand) {
                dropperServo.setPosition(DROPPER_EJECT);
            } else if(dropperToIngestCommand && !dropperToEjectCommand) {
                dropperServo.setPosition(DROPPER_COLLECT);
            } else {
                dropperServo.setPosition(DROPPER_STOP);
            }
        }

        // Limit switch reset logic
        if(fetcherLimitPressed && !previousFetcherLimitPressed) {
            fetcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fetcherPositionCommand = 0;
            fetcherHomed = true;
        }
        if(liftLimitPressed && !previousLiftLimitPressed) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftPositionCommand = 0;
            liftHomed = true;
        }

        // Button to re-home the fetcher (disables auto moves and allows travel to -10000)
        if(gamepad2.a) {
            fetcherHomed = false;
        }

        // Button presets override joysticks
        // Home button takes priority
        // If joystick moves, it aborts the auto move and starts manual from wherever it is now
        if(fetcherHomed && gamepad2.dpad_down) {
            fetcherPositionCommand = 0;
            fetcherManualControl = false;
            fetcherAutoControl = true;
        } else if(fetcherHomed && gamepad2.dpad_left) {
            fetcherPositionCommand = FETCHER_SUBSTATION_TICKS;
            fetcherManualControl = false;
            fetcherAutoControl = true;
        } else {
            double fetcherJoystick = gamepad2.left_stick_y;
            if (fetcherJoystick > 0.1 || fetcherJoystick < -0.1) {
                if(fetcherAutoControl) {  // Abort automatic operation
                    fetcherPositionCommand = fetcherEncoder;
                    fetcherAutoControl = false;
                }
                fetcherPositionCommand += fetcherJoystick * 10;
                fetcherManualControl = true;
            } else if (fetcherManualControl) {
                fetcherPositionCommand = fetcherEncoder;
                fetcherManualControl = false;
            }
        }

        if(liftHomed && gamepad1.x) {
            liftPositionCommand = 0;
            liftManualControl = false;
            liftAutoControl = true;
        } else if(liftHomed && gamepad1.a) {
            liftPositionCommand = LIFT_LOW_TICKS;
            liftManualControl = false;
            liftAutoControl = true;
        } else if(liftHomed && gamepad1.b) {
            liftPositionCommand = LIFT_MEDIUM_TICKS;
            liftManualControl = false;
            liftAutoControl = true;
        } else if(liftHomed && gamepad1.y) {
            liftPositionCommand = LIFT_HIGH_TICKS;
            liftManualControl = false;
            liftAutoControl = true;
        } else {
            double liftJoystick = gamepad2.right_stick_y;
            if(liftJoystick > 0.1 || liftJoystick < -0.1) {
                if(liftAutoControl) {  // Abort automatic operation
                    liftPositionCommand = liftEncoder;
                    liftAutoControl = false;
                }
                liftPositionCommand += liftJoystick * 10;
                liftManualControl = true;
            } else if(liftManualControl) {
                liftPositionCommand = liftEncoder;
                liftManualControl = false;
            }
        }

        // Add safety zone checks here?

        // Check bounds of command produced by manual and/or automatic code
        fetcherPositionCommand = Math.min(fetcherPositionCommand, FETCHER_MAX_TICKS);
        if(fetcherLimitPressed) {
            fetcherPositionCommand = Math.max(fetcherPositionCommand, 0);
        } else if(fetcherHomed){
            fetcherPositionCommand = Math.max(fetcherPositionCommand, -50);
        } else {
            fetcherPositionCommand = Math.max(fetcherPositionCommand, -10000);
        }
        fetcherMotor.setTargetPosition((int) fetcherPositionCommand);

        liftPositionCommand = Math.min(liftPositionCommand, LIFT_MAX_TICKS);
        if(liftLimitPressed) {
            liftPositionCommand = Math.max(liftPositionCommand, 0);
        } else if(liftHomed){
            liftPositionCommand = Math.max(liftPositionCommand, -50);
        } else {
            liftPositionCommand = Math.max(liftPositionCommand, -10000);
        }
        liftMotor.setTargetPosition((int) liftPositionCommand);



        // Mecanum joystick control with scaling per axis
        double translateForwardCommand = -gamepad1.left_stick_y * 0.5;
        double translateRightCommand = gamepad1.left_stick_x * 0.5;
        double rotateLeftCommand = -gamepad1.right_stick_x * 0.5;
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
        mecanumFrontLeft.setVelocity(frontLeftVelocity);
        mecanumBackLeft.setVelocity(backLeftVelocity);
        mecanumBackRight.setVelocity(backRightVelocity);
        mecanumFrontRight.setVelocity(frontRightVelocity);

        telemetry.addData("Runtime", runtime.seconds());
        telemetry.addData("Fetcher", String.format("enc %d, tgt %d, lmt %s", fetcherEncoder, fetcherPositionCommand, String.valueOf(fetcherLimitPressed)));
        telemetry.addData("Lift",    String.format("enc %d, tgt %d, lmt %s", liftEncoder, liftPositionCommand, String.valueOf(liftLimitPressed)));
        telemetry.addData("Mecanum", String.format("FL%1.4f BL%1.4f BR%1.4f FR%1.4f",frontLeftVelocity,backLeftVelocity,backRightVelocity,frontRightVelocity));

        // Store values for next loop
        previousFetcherLimitPressed = fetcherLimitPressed;
        previousLiftLimitPressed = liftLimitPressed;
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
    }

}
