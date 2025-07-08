# Robotic-Tank
A remote controlled robotic car with 180° rotation camera using SSH and VNC via Raspberry Pi 4.

## Items Required
* [Devastator Tank Mobile Robot Platform](https://www.amazon.com/dp/B014L1CF1K/ref=twister_B07YKSF8N7?_encoding=UTF8&th=1)
* [L298N Motor Drive Controller Board Module](https://www.amazon.com/Qunqi-Controller-Module-Stepper-Arduino/dp/B014KMHSW6/ref=asc_df_B014KMHSW6/?tag=hyprod-20&linkCode=df0&hvadid=167139094796&hvpos=&hvnetw=g&hvrand=8299905979766944063&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9021712&hvtargid=pla-306436938191&psc=1)
* [Raspberry PI Night Vision Camera](https://www.amazon.com/Raspberry-Camera-Webcam-OV5647-Adjustment/dp/B08QFM8TVV/ref=sr_1_1?crid=1AK0GBTKWLG6K&keywords=night+vision+camera+raspberry+pi&qid=1692113148&s=financial&sprefix=night+vision+camera+ras%2Cfinancial%2C99&sr=1-1)
* [Compact Battery Pack](https://www.amazon.com/EnergyQC-Portable-Ultra-Compact-Compatible-More-Black/dp/B0B12721V3/ref=sr_1_15?hvadid=174280170623&hvdev=c&hvlocphy=9021712&hvnetw=g&hvqmt=e&hvrand=5657593413140322175&hvtargid=kwd-1670787748&hydadcr=24659_9648993&keywords=usb%2Bbattery%2Bpack&qid=1691595030&sr=8-15&th=1)
* Raspberry Pi 4
* Servo Motor

## Wiring
### Wiring with breadboard
<img width="1049" alt="Screenshot 2023-08-15 at 11 13 35 AM" src="https://github.com/ChalmersPhua00/Raspberry-Pi-4-Robotic-Tank/assets/107158272/210f5ba8-1a1e-4f4a-b8df-810a26f585cf">

### Wiring without breadboard (Recommended)
| Servo  | Pi4 |
| ------------- | ------------- |
| GND  | Pin 20 |
| VCC  | Pin 4 |
| Signal  | GPIO12 |

| L298N  | Pi4 |
| ------------- | ------------- |
| ENA  | GPIO25 |
| IN1  | GPIO4 |
| IN2  | GPIO17 |
| IN3  | GPIO27 |
| IN4  | GPIO23 |
| ENB  | GPIO22 |
| GND  | Pin 6 + 12V Power Connector (Soldered) |
| +12V  | 12V Power Connector |

## Getting Started

robotankV1/V2.py
* WASD to move tank, Z and X to adjust speed, SPACE to stop
* Arrow keys (left, right, up, down) to tilt camera (for V2, repeatedly press them to adjust)

face_tracking.py
* Tracks faces using a 2-DOF servo-mounted camera.
* Uses a Haar Cascade classifier for face detection.

### Executing program
* On Pi's terminal enter "sudo raspi-config", then go to Interfacing Options to enable SSH and VNC.
* Make sure both computer and pi are connected to the same network.

#### SSH (without camera)
1. Open computer's terminal and enter "ssh <username>@<pi's IP address>"
2. Use personal Pi account username and password to connect

#### VNC (with camera)
1. Install TigerVNC on the host computer
2. Insert Pi's IP address to connect and navigate to Pi's terminal

On the directory where the .py is located:
```
sudo pigpiod
sudo python <FILE>.py
```
