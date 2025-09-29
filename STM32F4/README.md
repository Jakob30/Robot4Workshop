# Roboterarm Bewegungssteuerung

Dieses Projekt implementiert die Bewegungssteuerung eines Roboterarms auf Basis eines STM32-Mikrocontrollers.  
Die Software wurde in C mithilfe der STM32CubeIDE entwickelt und organisiert den Code in logischen Modulen für Motorsteuerung, Kinematik, Fehlerüberwachung und Anzeige.

---

## Projektstruktur

Die wichtigsten Ordner und Dateien sind wie folgt aufgebaut:
```
Core/Src/
│ ├── main.c
│ ├── [TMC2209 Library von veysiadn]
│ └── motor/
│       │ ├── helper.c
│       │ ├── kinematics.c
│       │ ├── motor_control.c
│       │ ├── motor_init.c
│       │ └── status_check.c
│ └── display/
        │ ├── [SSD1306 Library von afiskon]
        | └── helper.c

Core/Inc/
│ ├── [TMC2209 Library von veysiadn]
│ └── motor/
│       │ ├── helper.h
│       │ ├── kinematics.h
│       │ ├── motor_control.h
│       │ ├── motor_defines.h
│       │ ├── motor_init.h
│       │ └── status_check.h
│ └── display/
        │ ├── [SSD1306 Library von afiskon]
        | └── helper.h
```

---

### Core/Src/

- **`tmc2209_if.c` & `tmc2209.c`**  
  Enthalten die Implementierung und Konfiguration der TMC2209 Schrittmotortreiber.  
  Diese Dateien stammen aus dem [GitHub-Repository](https://github.com/veysiadn/tmc_2209) und stellen die Schnittstelle zwischen Mikrocontroller und Motortreiber bereit.

- **`main.c`**  
  - Enthält die von der STM32CubeIDE generierte Initialisierung der Hardware (GPIO, UART, Timer usw.).  
  - Hier wird auch die Initialisierung der Motortreiber sowie der Motorvariablen vorgenommen.  
  - Weitere Logik ist nicht implementiert – die `main.c` dient ausschließlich als Einstiegspunkt, um die Bewegungsfunktionen aus dem `motor/`-Modul aufzurufen.

---

### motor/

Dieses Modul enthält alle Source- und Headerdateien, die direkt mit der Bewegung und Steuerung des Roboterarms zusammenhängen.

- **`helper.c`**  
  - Sammlung von mathematischen Hilfsfunktionen.  
  - Dient zur Durchführung von Umrechnungen, die bei der Motorsteuerung und Koordinatenberechnung benötigt werden.

- **`kinematics.c`**  
  - Führt die Berechnung der Gelenkwinkel für jeden Motor durch.  
  - Ermöglicht die Umrechnung von Zielkoordinaten (`x`, `y`, `z`) in konkrete Motorpositionen.  
  - Zentrale Funktion: `moveToCoordinates(float x, float y, float z)`.

- **`motor_control.c`**  
  Enthält die grundlegende Bewegungslogik sowie alle Bewegungsfunktionen des Roboterarms:  

  - `motor_error_t moveDegrees(float degrees, motor_t *motor)`  
    Bewegt einen Motor relativ zur aktuellen Position.  

  - `motor_error_t moveAbsolute(float degrees, motor_t *motor)`  
    Bewegt den Motor auf eine absolute Zielposition. Funktioniert nur zuverlässig, wenn zuvor eine Referenzfahrt durchgeführt wurde.  
    Wird ebenfalls in `moveToCoordinates(...)` verwendet.  

  - `motor_error_t moveToCoordinates(float x, float y, float z, float gripper_direction)`  
    Bewegt den Roboterarm an die gewünschten Koordinaten.  
    Empfehlung: Vorher eine Referenzfahrt durchführen, um die Nullposition zu setzen.  
    `gripper_direction` steuert die Ausrichtung des Greifers.  

  - `motor_error_t movePolar(float r, float theta, float z, float gripper_direction)`  
    Alternative zu `moveToCoordinates(...)`, jedoch im Polarkoordinatensystem.  

  - `motor_error_t goHome()`  
    Führt den Roboterarm in die Nullposition.  
    Diese wird durch das Anfahren des mechanischen Anschlags der Motoren erreicht.  

  - `motor_error_t grip()`  
    Schließt den Greifer, indem Motor 5 angesteuert wird.  

  - `motor_error_t openGripper()`  
    Öffnet den Greifer.  

- **`motor_init.c`**  
  - Initialisiert die Motorstrukturen und deren Parameter.  
  - Stellt sicher, dass alle Motoren beim Start korrekt konfiguriert sind.  

- **`status_check.c`**  
  - Ermöglicht das kontinuierliche Überprüfen des Motorstatus mithilfe der StallGuard-Technologie.  
  - Berechnet eine dynamische Schwellgrenze zur Erkennung von Stalls (Blockierungen) und liest regelmäßig die StallGuard-Werte aus den Treibern.  

---

### display/

- Enthält die **SSD1306-Library** von [afiskon](https://github.com/afiskon/stm32-ssd1306).  
- Wird für die Ansteuerung eines OLED-Displays genutzt, um Statusinformationen oder Debug-Ausgaben visuell darzustellen.  

---

## Core/Inc/

Dieser Ordner enthält alle Header-Dateien, die die Schnittstellen (Funktionsdeklarationen, Strukturen, Typen und Konstanten) definieren.  
Die meisten Header-Dateien enthalten lediglich Funktionsprototypen, die in den entsprechenden `.c`-Dateien implementiert werden.  

Besonders wichtig sind dabei die Motor-Definitionen und -Typen:

- **`motor_types.h`**  
  - Definiert zentrale Datentypen für die Motorsteuerung.  
  - Enthält Strukturen wie `motor_t`, die alle relevanten Eigenschaften eines Motors zusammenfassen (z. B. aktuelle Position, Statusflags, Konfigurationsparameter).  
  - Diese Abstraktion ermöglicht es, die Bewegungsfunktionen generisch zu halten und jeden Motor unabhängig voneinander zu verwalten.  

- **`motor_defines.h`**  
  - Enthält eine Sammlung von Makros und Konstanten, die im gesamten Projekt für die Motorsteuerung benötigt werden.  
  - Beispiele:   
    - Geschwindigkeits- und Beschleunigungsgrenzen  
    - Allgemeine Hardware-Zuordnungen (UART-Kanäle, Pins, etc.)  
  - Durch die Trennung in eine zentrale Header-Datei wird verhindert, dass "Magic Numbers" im Code verteilt sind. Änderungen an Motorparametern können so an einer zentralen Stelle erfolgen.  

---

### Weitere Header-Dateien

- **`helper.h`**  
  - Deklariert mathematische Hilfsfunktionen, z. B. Umrechnungen für Winkel oder Längen.  

- **`kinematics.h`**  
  - Stellt die Schnittstelle zur Berechnung der Gelenkwinkel bereit.  
  - Deklariert Funktionen, die Zielkoordinaten (`x`, `y`, `z`) in Motorwinkel umwandeln.  

- **`motor_control.h`**  
  - Enthält die Funktionsprototypen für die Bewegungsfunktionen (`moveDegrees`, `moveAbsolute`, `moveToCoordinates`, usw.).  

- **`motor_init.h`**  
  - Deklariert Initialisierungsfunktionen für die Motorstrukturen (`motor_t`).  
  - Wird zu Beginn des Programms genutzt, um alle Motoren in einen definierten Zustand zu versetzen.  

- **`status_check.h`**  
  - Schnittstelle zur Überwachung des Motorstatus.  
  - Deklariert Funktionen, die den StallGuard-Wert auslesen und Stalls (Blockierungen) erkennen.  

---

### display/

- **`helper.h`**  
  - Deklariert zusätzliche Display-Hilfsfunktionen.  

---


## Abhängigkeiten

- **STM32CubeIDE** für die Projektkonfiguration und Codegenerierung  
- **TMC2209 Treiberbibliothek** von [veysiadn](https://github.com/veysiadn/tmc_2209). 
- **SSD1306 OLED-Bibliothek** von [afiskon](https://github.com/afiskon/stm32-ssd1306). 

---

## Hinweise zur Verwendung

1. Führe vor der ersten Nutzung eine Referenzfahrt mit `goHome()` durch, um die Nullposition zu setzen.  
2. Bewegungsbefehle (`moveToCoordinates`, `movePolar`, etc.) setzen eine korrekte Initialisierung durch `motor_init.c` voraus.  
3. Für eine sichere Nutzung sollte `status_check.c` aktiv zur Überwachung von Stalls eingesetzt werden.  

---


