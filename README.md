# KamelPi

### Team members 

Björn Ellwert, 
Carlo Brokering, 
Jamal Salif, 
Stephan Swiatek

### Environment

* Raspberry Pi 3B+
* Raspbian Lite

### Required Librarys

* OpenCV 4 or newer Version (recommended), [Install guide for OpenCV 4.1.0](https://docs.opencv.org/4.1.0/d7/d9f/tutorial_linux_install.html)  
  **Important:** after finishing the installation run `sudo ldconfig`
* WiringPi, install with `sudo apt-get install wiringpi`
* Boost - [Getting started guide](https://www.boost.org/doc/libs/1_70_0/more/getting_started/unix-variants.html)
  </br>**Installation:**</br>
  1. Download latest version of boost
  2. Extract folder and cd into it
  3. `./bootstrap.sh`
  4. `./b2 headers`
  5. `./b2 -a`
  6. `./b2 install --prefix=/usr/local/boost`
