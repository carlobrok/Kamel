# Netzwerk #

### Raspberry Pi WLAN-Konfiguration ###

1. Einloggen auf den Raspberry Pi
   1. Über das Netzwerk
      1. Raspberry Pi über LAN mit dem Netzwerk verbinden
      1. `ssh pi@<ip-adresse>` - [IP-Adresse herausfinden](#ip-addresse-herausfinden)
      1. Passwort eingeben
   1. Über Monitor und Tastatur
1. [Anleitung](https://www.raspberrypi.org/documentation/configuration/wireless/wireless-cli.md). für das 
Hinzufügen von WLAN Netzwerken auf dem Raspberry Pi. Dabei **nicht** die Methode mit `sudo raspi-config` nutzen, 
sondern das Netzwerk in der **wpa-supplicant** config hinzufügen.
1. `sudo reboot`

### IP-Adresse herausfinden ###

**nmap**

1. [nmap](https://nmap.org/ "nmap.org") installieren mit
`sudo apt install nmap`

2. Alle mit dem Netzwerk verbundenen Geräten anzeigen </br>
`sudo nmap -n -sP xxx.xxx.xxx.0/24`</br>
Das `xxx.xxx.xxx` muss durch die ersten Ziffern der lokalen Netzwerkadresse ersetzt werden.

**Router**

Bei Zugriff auf den Router kann die Adresse natürlich auch direkt ausgelesen werden, indem man sich auf der Verwaltungsoberfläche des Routers einloggt.
