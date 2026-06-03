# motor_control

Bibliothèque C++17 (CMake) pour une cascade **position → courant** (moteur **DC brossé**), avec mode **maintien en courant** (sans asservissement position). Capteurs injectés via `ICurrentSensor` et `IPositionSensor`.

## Intégration dans un autre projet

### `add_subdirectory`

```cmake
add_subdirectory(third_party/motor_control EXCLUDE_FROM_ALL)
target_link_libraries(mon_firmware PRIVATE motor_control::motor_control)
```

### Install + `find_package`

```bash
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/chemin/install
cmake --build build
cmake --install build
```

```cmake
find_package(motor_control 0.1 REQUIRED)
target_link_libraries(mon_app PRIVATE motor_control::motor_control)
```

### Arduino IDE

Ce dépôt **est** une bibliothèque Arduino 1.5 (`library.properties` + dossier `src/`). Il n’y a qu’**une** façon fiable d’utiliser l’exemple avec l’IDE :

1. **Installer la bibliothèque** : le dossier du dépôt (celui qui contient `library.properties`) doit s’appeler **`motor_control`** et se trouver sous le répertoire des bibliothèques Arduino :
   - macOS / Linux : `~/Documents/Arduino/libraries/motor_control`
   - Windows : `Documents\Arduino\libraries\motor_control`

   Exemple (lien symbolique, à adapter avec ton chemin) :

   ```bash
   ln -sf "/Users/.../motorControl" ~/Documents/Arduino/libraries/motor_control
   ```

2. **Redémarrer** l’Arduino IDE (ou au minimum fermer tous les croquis).

3. **Ouvrir l’exemple** : **Fichier → Exemples → motor_control → MotorControlDemo**.  
   Ne pas partir d’un « nouveau croquis » vide : il n’aura pas la bibliothèque attachée.

4. Choisir une carte **32 bits** avec **C++17** (ex. RP2040 « Waveshare RP2040 Zero », ESP32). Éviter l’Uno AVR classique.

**Exemple RP2040** (`examples/MotorControlDemo/`) : avec une carte **RP2040** et le cœur *Raspberry Pi Pico / RP2040* (Earle Philhower recommandé), le croquis compile `MotorControlDemo_pico.cpp` : boucle **courant 10 kHz** (`repeating_timer` SDK), boucle **position 2 kHz** dans `loop()`, PWM **HG7881** sur deux broches, **ADC** courant (A0), angle **AS5600** en I2C. Brochage et constantes de calibration en tête de ce fichier ; touches série `z` / `h` / `r` / `s` décrites au démarrage.

Dans **tes** croquis (après installation), l’en-tête public attendu par `library.properties` (`includes`) est à la racine de `src/` :

```cpp
#include <motor_control.hpp>
```

Tu peux aussi inclure des modules seuls (`#include <motor_control/current_loop_controller.hpp>`, etc.) si tu veux limiter les dépendances de compilation.

**MounRiver / CH32** : voir le dossier `mounriver/` et son `README.md`.

Tu peux garder le même dépôt pour **CMake** et pour **Arduino** : les en-têtes sont dans `src/motor_control/` (voir [spécification bibliothèques 1.5](https://arduino.github.io/arduino-cli/library-specification/)).

## Fichiers publics

Les fichiers sources sont sous `src/` ; le point d’entrée unique est `src/motor_control.hpp` (`#include <motor_control.hpp>`). Les en-têtes détaillés sont sous `src/motor_control/`.

| En-tête | Rôle |
|--------|------|
| `motor_control.hpp` | Point d’entrée unique (réexporte tous les en-têtes publics ci-dessous) |
| `motor_control/current_sensor.hpp` | `ICurrentSensor::read_amperes()` |
| `motor_control/current_loop_controller.hpp` | PI courant + anti-windup |
| `motor_control/position_sensor.hpp` | `IPositionSensor::read_position()` (même unité que la consigne) |
| `motor_control/position_current_cascade.hpp` | PI position → courant ; `CurrentHold` / `PositionServo` |
| `motor_control/dc_brushed_motor.hpp` | `DcBrushedMotorParams` (τ ↔ I avec `Kt`) |
| `motor_control/motor_pwm_driver.hpp` | `IMotorPwmDriver` (PWM / pont abstrait) |
| `motor_control/hg7881_bridge_pins.hpp` | `IHg7881BridgePins` (HAL IA / IB) |
| `motor_control/hg7881_motor_driver.hpp` | `Hg7881MotorDriver` (`IMotorPwmDriver` pour HG7881 / L9110) |
| `motor_control/vnh7070as_bridge_pins.hpp` | `IVnh7070asBridgePins` (INA / INB / PWM) |
| `motor_control/vnh7070as_motor_driver.hpp` | `Vnh7070asMotorDriver` (`IMotorPwmDriver` pour VNH7070AS) |

**VNH7070AS** : régler le timer PWM de la broche **PWM** ≤ **20 kHz** (datasheet ST). Sortie veille recommandée après power-up : lever INA ou INB puis PWM ≥ **20 µs** plus tard (voir datasheet).

Après avoir atteint la cible : `enter_current_hold(i_hold)` ou `enter_current_hold_using_last_command()` pour figer une consigne courant tout en continuant d’appeler `step` (la boucle courant reste active). `set_wrap_angle_error(true)` pour erreur d’angle la plus courte en radians.

Période d’ISR fixe : `set_sample_time_s(T)` puis `step(consigne)` sans second argument (`dt` reste disponible si la période varie ou pour tests).

Découplage acquisition / calcul : `update()` (courant seul, ou courant + position via la cascade) puis `step(...)` ; si `update()` n’est pas appelé, `step` relit les capteurs comme avant.

La sortie de la boucle courant est bornée (`set_output_limits`) : à relier à `IMotorPwmDriver::set_command` (ou autre driver) côté application.
