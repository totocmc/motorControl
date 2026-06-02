# Croquis Arduino (IDE 2.x)

Ce dossier contient un **exemple de croquis** qui utilise la bibliothèque `motor_control` définie à la racine du dépôt (`library.properties` + `src/`).

## Faire reconnaître la bibliothèque par l’IDE

L’Arduino IDE ne charge les bibliothèques que depuis `libraries/` (ou le bundle carte). Il faut donc que ce dépôt soit **visible comme `motor_control`** dans ce dossier.

### Option A — Lien symbolique (recommandé pour le dev)

À la racine du clone (là où se trouve `library.properties`) :

```bash
ln -sf "$(pwd)" ~/Documents/Arduino/libraries/motor_control
```

Sous Windows (invite **administrateur**, PowerShell) :

```powershell
cmd /c mklink /D "%USERPROFILE%\Documents\Arduino\libraries\motor_control" "C:\chemin\vers\motorControl"
```

Puis redémarrer l’Arduino IDE.

### Option B — Copie

Copier tout le dossier du dépôt vers `Documents/Arduino/libraries/motor_control`.

## Ouvrir le croquis

**Fichier → Ouvrir** puis choisir :

`arduino/MotorControlDemo/MotorControlDemo.ino`

Choisis une carte **32 bits avec C++17** (ESP32, Raspberry Pi Pico, Arduino Giga, etc.) — voir le README principal.
