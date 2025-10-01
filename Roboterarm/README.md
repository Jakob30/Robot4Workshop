# 3D Modelle Roboterarm

Die M3-Einschmelzmuttern sind rot eingefärbt. Verschraubungen vollständig im Modell.

## Creo8 Modell
Alle 3D-Modelle der einzelnen Teile befinden sich im Ordner **`Creo8 Modelle`**.  
Die Baugruppenstruktur ist folgendermaßen verschachtelt:
```
roboterarm_gesamt.asm
│
├─ basis.asm
│ └─ riemenspanner.asm
├─ element1.asm
├─ element2.asm
├─ element3.asm
└─ greifer.asm
  └─ greiferfingereinheit.asm
```

> Hinweis: Alle Dateien liegen unsortiert in einem Ordner.  
> Dadurch kann dieser Ordner direkt als Arbeitsverzeichnis genutzt werden.  
> Die Überbaugruppe **`roboterarm_gesamt.asm`** ruft alle weiteren Dateien automatisch auf.

Die Gesamtbaugruppe ist flexibel und kann per *„Komponente ziehen“* bewegt werden.  
Auch die Greiferbaugruppe ist beweglich.

---

## STEP Modell
Für eine schnelle Ansicht der Gesamtbaugruppe oder für Nutzer ohne Creo8-Zugang kann die Datei  
**`roboterarm_gesamt_asm.stp`** in beliebiger CAD-Software importiert werden.

---

## 3D Druck
Die **`.stl`-Dateien** der 3D-Druckteile befinden sich im Ordner **`STL Dateien`**.  

- Für die **Zahnscheiben** wird höchstens eine **0.4 mm Düse** empfohlen.  
- Die **bionischen Finger** müssen mit flexiblem Filament (z. B. **TPU**) gedruckt werden.