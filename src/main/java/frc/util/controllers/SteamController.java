package frc.util.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.controllers.Joystick.Axis;

public class SteamController {
    //REPLACE BELOW WITH ACTUAL AXIS
    private static final int AXIS_LEFT_X = 0;
    private static final int AXIS_LEFT_Y = 1;
    private static final int AXIS_RIGHT_X = 2;
    private static final int AXIS_RIGHT_Y = 3;
    
    private static final int AXIS_LEFT_TRIGGER = 4;
    private static final int AXIS_RIGHT_TRIGGER = 5;
    
    private static final int AXIS_TRACKPAD_LEFT_X = 6;
    private static final int AXIS_TRACKPAD_LEFT_Y = 7;
    private static final int AXIS_TRACKPAD_RIGHT_X = 8;
    private static final int AXIS_TRACKPAD_RIGHT_Y = 9;
    
    
    
    private static final int BUTTON_POV_UP = 16;
    private static final int BUTTON_POV_DOWN = 17;
    private static final int BUTTON_POV_LEFT = 18;
    private static final int BUTTON_POV_RIGHT = 19;
    
    private static final int BUTTON_A = 1;
    private static final int BUTTON_B = 2;
    private static final int BUTTON_X = 3;
    private static final int BUTTON_Y = 4;
    
    private static final int BUTTON_STICK_LEFT = 5;
    private static final int BUTTON_STICK_RIGHT = 6;
    
    private static final int BUTTON_BUMPER_LEFT = 7;
    private static final int BUTTON_BUMPER_RIGHT = 8;
    
    private static final int BUTTON_STEAM = 9;
    private static final int BUTTON_MENU = 10;
    private static final int BUTTON_VIEW = 11;
    private static final int BUTTON_QUICK_ACCESS = 12;
    
    private static final int BUTTON_GRIP_LT = 12;
    private static final int BUTTON_GRIP_LB = 13;
    private static final int BUTTON_GRIP_RT = 14;
    private static final int BUTTON_GRIP_RB = 15;
    
    private static final int BUTTON_TRACKPAD_LEFT = 10;
    private static final int BUTTON_TRACKPAD_RIGHT = 11;



    private final GenericHID hid;

    public final Joystick leftStick;
    public final Joystick rightStick;
    public final Joystick leftTrackpad;
    public final Joystick rightTrackpad;

    public final Axis leftTrigger;
    public final Axis rightTrigger;

    public SteamController(int port) {
        hid = new edu.wpi.first.wpilibj.Joystick(port);

        leftStick = new Joystick(() -> hid.getRawAxis(AXIS_LEFT_X), () -> hid.getRawAxis(AXIS_LEFT_Y));
        rightStick = new Joystick(() -> hid.getRawAxis(AXIS_RIGHT_X), () -> hid.getRawAxis(AXIS_RIGHT_Y));
        
        leftTrackpad = new Joystick(() -> hid.getRawAxis(AXIS_TRACKPAD_LEFT_X), () -> hid.getRawAxis(AXIS_TRACKPAD_LEFT_Y));
        rightTrackpad = new Joystick(() -> hid.getRawAxis(AXIS_TRACKPAD_RIGHT_X), () -> hid.getRawAxis(AXIS_TRACKPAD_RIGHT_Y));

        leftTrigger = new Axis(() -> hid.getRawAxis(AXIS_LEFT_TRIGGER));
        rightTrigger = new Axis(() -> hid.getRawAxis(AXIS_RIGHT_TRIGGER));
    }

    public Trigger povUp() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_POV_UP)); }
    public Trigger povDown() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_POV_DOWN)); }
    public Trigger povLeft() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_POV_LEFT)); }
    public Trigger povRight() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_POV_RIGHT)); }

    public Trigger a() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_A)); }
    public Trigger b() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_B)); }
    public Trigger x() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_X)); }
    public Trigger y() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_Y)); }

    public Trigger leftStickButton() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_STICK_LEFT)); }
    public Trigger rightStickButton() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_STICK_RIGHT)); }

    public Trigger leftTrackpadButton() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_TRACKPAD_LEFT)); }
    public Trigger rightTrackpadButton() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_TRACKPAD_RIGHT)); }

    public Trigger leftBumper() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_BUMPER_LEFT)); }
    public Trigger rightBumper() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_BUMPER_RIGHT)); }

    public Trigger leftTopGrip() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_GRIP_LT)); }
    public Trigger rightTopGrip() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_GRIP_RT)); }
    public Trigger leftBottomGrip() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_GRIP_LB)); }
    public Trigger rightBottomGrip() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_GRIP_RB)); }

    public Trigger steamButton() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_STEAM)); }

    public Trigger menuButton() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_MENU)); }

    public Trigger viewButton() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_VIEW)); }

    public Trigger quickAccessButton() { return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(BUTTON_QUICK_ACCESS)); }



    public boolean isConnected() {
        return hid.isConnected();
    }

    public void setRumble(RumbleType rumbleType, double value) {
        hid.setRumble(rumbleType, value);
    }
    public Command rumble(RumbleType rumbleType, double value) {
        return Commands.startEnd(
            () -> setRumble(rumbleType, value), 
            () -> setRumble(rumbleType, 0)
        );
    }
}
