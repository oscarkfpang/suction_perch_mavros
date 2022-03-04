This code is tested and compiled in ROS-Melodic (Under Python2)
If you want to run this in ROS-Noetic, change the # to python3

## Installation
1. Clone the repo to ROS workspace, preferably under mavros_ws/src/. 
2. Run *catkin_build* and source the workspace
3. chmod +x the python code
4. Run the python script

## Enabling Hardware PWM on Raspberry Pi 4

If you want to use the hardware PWM capabilities of your Raspberry Pi, you will need to activate this feature. And if you want to run your code without elevated root privileged to access them, then, you'll as well need to make couple of modifications. This tutorial is her to help you activating the PWM and making sure you'll get the right permissions.


### Enabling hardware PWM

In order to have the hardware PWM activated on the Raspberry Pi, you'll have to edit the /boot/config.txt (or /boot/firmware/config.txt) file and add an overlay.

https://blog.oddbit.com/post/2017-09-26-some-notes-on-pwm-on-the-raspb/

Main Raspberry Pi kernel documentation gives 2 possibilities. Either a [single channel](https://github.com/raspberrypi/linux/blob/04c8e47067d4873c584395e5cb260b4f170a99ea/arch/arm/boot/dts/overlays/README#L925), either a [dual channel](https://github.com/raspberrypi/linux/blob/04c8e47067d4873c584395e5cb260b4f170a99ea/arch/arm/boot/dts/overlays/README#L944).

Here are the possible options for each PWM channel:

| PWM | GPIO | Function | Alt | Exposed |
| --- | --- | --- | --- | --- |
| PWM0 | 12 | 4 | Alt0 | Yes |
| PWM0 | 18 | 2 | Alt5 | Yes |
| PWM0 | 40 | 4 | Alt0 | No |
| PWM0 | 52 | 5 | Alt1 | No |
| PWM1 | 13 | 4 | Alt0 | Yes |
| PWM1 | 19 | 2 | Alt5 | Yes |
| PWM1 | 41 | 4 | Alt0 | No |
| PWM1 | 45 | 4 | Alt0 | No |
| PWM1 | 53 | 5 | Alt1 | No |

Only accessible pin from this list on the Raspberry Pi pin out are GPIO 12, 18, 13 and 19. The other GPIO are not exposed.

### Activating only 1 channel

We have then 4 options for the exposed GPIO pins:

| PWM | GPIO | Function | Alt | dtoverlay |
| --- | --- | --- | --- | --- |
| PWM0 | 12 | 4 | Alt0 | dtoverlay=pwm,pin=12,func=4 |
| PWM0 | 18 | 2 | Alt5 | dtoverlay=pwm,pin=18,func=2 |
| PWM1 | 13 | 4 | Alt0 | dtoverlay=pwm,pin=13,func=4 |
| PWM1 | 19 | 2 | Alt5 | dtoverlay=pwm,pin=19,func=2 |

Edit the /boot/config.txt file and add the dtoverlay line in the file. You need root privileges for this:

```bash
sudo nano /boot/config.txt
```

Save the file with `ctrl + x` then `Y` then `enter`

Then reboot:

```bash
sudo reboot
```

You are all setup, the basic example should now work with the PWM and channel you have selected.

### Activating 2 channels

| PWM0 | PWM0 GPIO | PWM0 Function | PWM0 Alt |  PWM1 | PWM1 GPIO | PWM1 Function | PWM1 Alt | dtoverlay |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| PWM0 | 12 | 4 | Alt0 | PWM1 | 13 | 4 | Alt0 | dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4 |
| PWM0 | 18 | 2 | Alt5 | PWM1 | 13 | 4 | Alt0 | dtoverlay=pwm-2chan,pin=18,func=2,pin2=13,func2=4 |
| PWM0 | 12 | 4 | Alt0 | PWM1 | 19 | 2 | Alt5 | dtoverlay=pwm-2chan,pin=12,func=4,pin2=19,func2=2 |
| PWM0 | 18 | 2 | Alt5 | PWM1 | 19 | 2 | Alt5 | dtoverlay=pwm-2chan,pin=18,func=2,pin2=19,func2=2 |

Edit the /boot/config.txt file and add the dtoverlay line in the file. You need root privileges for this:

```bash
sudo nano /boot/config.txt
```

Save the file with `ctrl + x` then `Y` then `enter`

Then reboot:

```bash
sudo reboot
```

You are all setup, the basic example should now work with the PWM and channel you have selected.

## Solving permission issues


### Adding your user to the right permission group

If you're running, or just upgraded to a version published after August 2020, this should be already done.
You will have to create a [specific group in udev](https://raspberrypi.stackexchange.com/questions/66890/accessing-pwm-module-without-root-permissions).

```bash
sudo nano /etc/udev/rules.d/99-com.rules
```

Add the following lines:

```text
SUBSYSTEM=="pwm*", PROGRAM="/bin/sh -c '\
        chown -R root:gpio /sys/class/pwm && chmod -R 770 /sys/class/pwm;\
        chown -R root:gpio /sys/devices/platform/soc/*.pwm/pwm/pwmchip* && chmod -R 770 /sys/devices/platform/soc/*.pwm/pwm/pwmchip*\
'"
```

Save the file with `ctrl + x` then `Y` then `enter`

And add the user group gpio to the user (assuming the user name is ubuntu)

```
sudo groupadd gpio
sudo usermod -a -G gpio ubuntu
sudo grep gpio /etc/group
sudo reboot
```

Then reboot:

```bash
sudo reboot
```

You are all setup, the basic example should now work with the PWM and channel you have selected.

