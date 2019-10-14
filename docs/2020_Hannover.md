# 2020_Hannover

**Hauptprogramm, indem alle Bibliotheken genutzt werden. Die zusammengefasste Bildauswertung und die Fahrentscheidungen finden hier statt**

### Logger debug_lg
Log für allgemeine Debug Nachrichten

### Mat img_rgb
Input Bild im [BGR Format](https://stackoverflow.com/questions/367449/what-exactly-is-bgr-color-space).

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
