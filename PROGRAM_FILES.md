# Program files documentation

Es folgt eine genauere Beschreibung der einzelnen Methoden, Funktionen und Variablen, die sich in den jeweiligen Dateien befinden.

# 2020_Hannover

**Hier befindet sich das Hauptprogramm, indem alle Bibliotheken genutzt werden.**

### Logger debug_lg
Log für allgemeine Debug Nachrichten

### Mat img_rgb
Input img im [BGR Format](https://stackoverflow.com/questions/367449/what-exactly-is-bgr-color-space)

</br>

## void m_drive
**Hauptthread, in dem alle Daten zusammen geführt werden und alle Entscheidungen bezüglich der Bewegung des Roboters getroffen werden**

### Logger behavior_lg
Log für alle Richtungsänderungen

### Logger sensor_lg
Log für alle aktuellen Sensordaten

### int motor_fd
Filedescriptor für die I2C Schnittstelle des Motorarduinos

### int sensor_fd
Filedescriptor für die I2C Schnittstelle des Sensorarduinos

### vector<Point> m_line_points
* Enthält alle aktuellen linepoints
* kann nur im drive thread genutzt werden
* muss immer durch get_line_data aktualisiert werden

### boost::circular_buffer<vector<Point>> last_line_points(50)
Enthält die letzten 50 linepoints

### Point m_grcenter
* Enthält den aktuellen Mittelpunkt des Grünpunktes falls vorhanden
* Wenn kein Grünpunkt vorhanden x=0, y=0
* kann nur im drive thread genutzt werden
* muss immer durch get_gruen_data aktualisiert werden

### Point m_grstate
* Enthält den aktuellen Status des Grünpunktes falls vorhanden
* Wenn kein Grünpunkt vorhanden grstate=GRUEN_NICHT -> 0
* kann nur im drive thread genutzt werden
* muss immer durch get_gruen_data aktualisiert werden

### boost::circular_buffer<Point> last_grcenter(50)
Enthält die letzten 50 Mittelpunkte des Grünpunktes

### bool digital_sensor_data[8]
* Enthält Sensordaten der 6 digitalen IR-Sensoren und 2 Touchsensoren
* Index der Sensoren *(in KamelDevices.h)*
  * IR_VORNE_L 0
  * IR_VORNE_R 1
  * IR_LINKS_V 2
  * IR_LINKS_H 3
  * IR_RECHTS_V 4
  * IR_RECHTS_H 5
  * T_HINTEN_L 6
  * T_HINTEN_R 7

### array<boost::circular_buffer<bool>, 8> last_digital_data
Enthält alle letzten 100 Sensordaten der digitalen Sensoren

### uint16_t analog_sensor_data[1]
Enthält die Sensordaten des (zukünftig der) analogen Sensors(/en)

### boost::circular_buffer<uint16_t> last_analog_data(100)
Enthält die letzten 100 Sensordaten des analogen Sensors

</br>

## void image_processing
* Einlesen des aktuellen Bildes
* Bildauswertung
* Updaten der globalen linepoints
* Updaten der Grünpunktdaten

### Logger camera_lg
Enthält Namen der abgespeicherten Bilder

### Mat hsv
Input Bild konvertiert ins [HSV Format](https://en.wikipedia.org/wiki/HSL_and_HSV)

### Mat bin_sw
Binäres Bild, bei dem alles dunkler als THRESH_BLACK auf img_rgb ist, den Wert 255 hat, alles andere den Wert 0

### Mat bin_gr
Binäres Bild, bei dem alles grüne auf [img_rgb](#mat-img_rbg) den Wert 255 hat, alles andere den Wert 0

### CameraCapture cam(0)
* Klasse CameraCapture startet einen neuen Thread
* Ließt die Kamera 0 aus (Raspberry Pi camera)

### VideoServer
* Started neuen Thread und hosted die Bilder
* Auslesbar durch den [VideoClient](https://github.com/carlobrok/VideoClient)

</br>

## int main
* Startet [m_drive](#void-m_drive), [m_imu](#void-m_imu) und springt dann in [image_processing](#void-image_processing)

------

# KamelDevices

**In KamelDevices.cpp/.h befinden sich alle Funktionen, die nötig sind, um mit den Arduinos, sowie der IMU des Roboters zu kommunizieren**

## int kamelI2Copen
```cpp
int kamelI2Copen(int devId);
```
**Beschreibung:**
* öffnet den I2C-Bus */dev/i2c-1* (I2C-Bus des Raspberry 3B+)
* setzt die Adresse des I2C-Geräts als *slave*
* gibt Filedescriptor zurück

**parameters** | **possible values**
-------------|--------------------
devId        | I2C address of the device (e.g. 0x08)
</br>

## int setMotorDirPwm
```cpp
int setMotorDirPwm(int &fd, uint8_t side, uint8_t direction, uint8_t pwm);
```
**Beschreibung:** setzt die Richtung und pwm-Rate der **gegebenen Motoren** indem die Werte mithilfe des Filedescriptors an das I2C-Gerät gesendet werden

  **parameters** | **possible values**
  ---------------|--------------------
  fd             | filedescriptor returned by kamelI2Copen
  side           | *MOTOR_LEFT/ MOTOR_RIGHT / MOTOR_BOTH*
  direction      | *MOTOR_FORWARD, MOTOR_BACKWARD*
  pwm | *0 - 255*
</br>

## int setMotorDirPwmBoth

```cpp
int setMotorDirPwmBoth(int &fd, uint8_t direction_left, uint8_t pwm_left, uint8_t direction_right, uint8_t pwm_right);
```
**Beschreibung:** setzt die Richtung und die pwm-Rate **beider Motoren** indem die Werte mithilfe des Filedescriptors an das I2C-Gerät gesendet werden

  **parameters**                   | **possible values**
  ---------------------------------|--------------------
  fd                               | filedescriptor returned by kamelI2Copen
  direction_left / direction_right | *MOTOR_FORWARD, MOTOR_BACKWARD*
  pwm_left / pwm_right             | *0 - 255*
</br>

## int setMotorState

```cpp
int setMotorState(int &fd, uint8_t side, uint8_t state);
```
**Beschreibung:** setzt die **gegebenen Motoren** in den gegebenen Zustand indem die Werte mithilfe des Filedescriptors an das I2C-Gerät gesendet werden

  **parameters** | **possible values**
  ---------------|--------------------
  fd             | filedescriptor returned by kamelI2Copen
  side           | *MOTOR_LEFT/ MOTOR_RIGHT / MOTOR_BOTH*
  state          | *MOTOR_OFF / MOTOR_FORWARD_NORMAL / MOTOR_BACKWARD_NORMAL*
</br>

## bool get_bit

```cpp
bool get_bit(uint8_t byte, uint8_t bit_index);
```
**Beschreibung:** Gibt das angegebene bit eines bytes zurück. Der Index geht von **0-7**, ein Index von 0 gibt das rechte bit zurück.

## int readBytes

**Beschreibung:** Funktion zu **Testzwecken**. Sendet den *command* an das I2C-Gerät des Filedescriptors und schreibt die empfangenen Daten in  *in_data*. Die zurückgegebenen Daten müssen n=*data_length* bytes sein.

## int getSensorData

```cpp
int getSensorData(int &fd, bool (&digital_sensor_data)[8], uint16_t (&analog_sensor_data)[1]);
```
**Beschreibung:** Sendet dem über den Filedescriptor angegeben Arduino eine Anfrage zum Senden aller Sensordaten. Ließt die Sensordaten aus und schreibt diese in die übergeben Arrays.

## int getDigitalSensorData

```cpp
int getDigitalSensorData(int &fd, bool (&digital_sensor_data)[8]);
```
**Beschreibung:** Sendet dem über den Filedescriptor angegeben Arduino eine Anfrage zum Senden der Daten aller digitalen Sensoren. Ließt die Sensordaten aus und schreibt diese in das übergebene Array.

## int getAnalogSensorData

```cpp
int getAnalogSensorData(int &fd, uint16_t (&analog_sensor_data)[1]);
```
**Beschreibung:** Sendet dem über den Filedescriptor angegeben Arduino eine Anfrage zum Senden der Daten aller analogen Sensoren. Ließt die Sensordaten aus und schreibt diese in das übergebene Array.

</br>

## void get_imu_data

## void void set_imu_data

## void m_imu

------

# Logger

------

# gruen

------

# line

------

# util

------

# CameraCapture / VideoServer
