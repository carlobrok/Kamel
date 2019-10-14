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

```cpp
int readBytes(int &fd, uint8_t *in_data, uint16_t data_length, uint8_t command);
```

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
