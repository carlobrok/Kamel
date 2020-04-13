# Kamel Documentation

### Team members 

Björn Ellwert - 
Carlo Brokering - 
Jamal Salif -
Stephan Swiatek

</br>

## Robot parts - Hardware

### Microcontroller

* [Arduino nano v3](https://store.arduino.cc/arduino-nano)
* [Raspberry Pi 3B+](https://www.raspberrypi.org/products/raspberry-pi-3-model-b-plus/)

### Sensors

* [Raspberry Camera v2.1](https://www.raspberrypi.org/products/camera-module-v2/)
* [Sharp digital infrared distance sensor](https://www.pololu.com/product/1134)
* [Sharp analog infrared distance sensor](http://www.sharp-world.com/products/device/lineup/data/pdf/datasheet/gp2y0a51sk_e.pdf)
* [Sparkfun 9DoF Razor IMU](https://www.sparkfun.com/products/14001)
* [Touch sensors](https://www.reichelt.de/schnappschalter-1xum-5a-250vac-flachhebel-mar-1050-5202-p32729.html?&trstct=pol_5)

### Motordriver

* [Dual MC33926 Motor Driver Carrier](https://www.pololu.com/product/1213)
* [16 Channel 12-bit Servo Driver](https://cdn-learn.adafruit.com/downloads/pdf/16-channel-pwm-servo-driver.pdf)

### Motors

* [V-TEC 12V DC-Gearmotor 76 RPM](https://eckstein-shop.de/V-TEC-12V-Mini-37D-DC-Motor-Gleichstrom-Getriebe-Motor-Stirnradgetriebe-76-RPM)

</br>

## Software

### Environment

* Raspbian Lite

### Required Librarys

* OpenCV 4 or newer version (recommended), [Install guide for OpenCV 4.1.0](https://docs.opencv.org/4.1.0/d7/d9f/tutorial_linux_install.html)  
  **Important:** 
  1. as cmake run `cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ -D WITH_QT=ON -DWITH_OPENGL=ON -D OPENCV_GENERATE_PKGCONFIG=ON -D WITH_GTK=ON  ..` 
  2. make sure all packages are configured (OpenGL, Qt, ...)
  3. after finishing the installation run `sudo ldconfig`
* WiringPi, install with `sudo apt-get install wiringpi`
* Boost - [Getting started guide](https://www.boost.org/doc/libs/1_70_0/more/getting_started/unix-variants.html)
  </br>**Installation:**</br>
  1. Download latest version of [boost](https://www.boost.org/)
  2. Extract folder and cd into it
  3. `./bootstrap.sh`
  4. `sudo ./b2 install`
* spdlog 1.5.0 or newer version - [Wiki](https://github.com/gabime/spdlog/wiki/)
  </br>**Installation:**</br>
  1. `git clone https://github.com/gabime/spdlog.git`
  2. `cd spdlog/`
  3. `sudo cp -r include/spdlog/ /usr/local/include/`
