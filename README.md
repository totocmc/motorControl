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

### Arduino IDE 2.x

1. Copie ou clone ce dépôt sous le nom du dossier **`motor_control`** dans le répertoire des bibliothèques Arduino, par exemple :
   - macOS : `~/Documents/Arduino/libraries/motor_control`
   - Windows : `Mes documents\Arduino\libraries\motor_control`
2. Vérifie que tu as bien `library.properties` à la racine du dossier et les sources sous `src/` (fichiers `.cpp` à la racine de `src/`, en-têtes dans `src/motor_control/`).
3. Redémarre l’IDE, puis **Croquis → Inclure la bibliothèque → motor_control**, ou dans ton `.ino` / `.cpp` :
   ```cpp
   #include <motor_control/motor_control.hpp>
   ```
4. **C++17** : le code utilise le standard C++17 (`std::clamp`, etc.). Les cartes **ESP32**, **RP2040** (pico), **STM32** (Giga, Portenta), **Renesas** récentes conviennent en général. Sur **AVR** (Uno classique), le compilateur est souvent trop ancien ou le binaire trop gros — privilégie une carte 32 bits.

Tu peux garder le même dépôt pour **CMake** et pour **Arduino** : les en-têtes sont dans `src/motor_control/` pour satisfaire la [spécification des bibliothèques 1.5](https://arduino.github.io/arduino-cli/library-specification/) (chemins d’inclusion : racine de la lib + `src/`).

**Exemple de croquis** : `arduino/MotorControlDemo/MotorControlDemo.ino` — installation lib via `arduino/README.md` (lien symbolique recommandé).

## Fichiers publics

Les fichiers sources sont sous `src/` ; les en-têtes publics sous `src/motor_control/` (inclusion `#include <motor_control/...>`).

| En-tête | Rôle |
|--------|------|
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
| `motor_control/motor_control.hpp` | Agrégat des en-têtes publics |

**VNH7070AS** : régler le timer PWM de la broche **PWM** ≤ **20 kHz** (datasheet ST). Sortie veille recommandée après power-up : lever INA ou INB puis PWM ≥ **20 µs** plus tard (voir datasheet).

Après avoir atteint la cible : `enter_current_hold(i_hold)` ou `enter_current_hold_using_last_command()` pour figer une consigne courant tout en continuant d’appeler `step` (la boucle courant reste active). `set_wrap_angle_error(true)` pour erreur d’angle la plus courte en radians.

Période d’ISR fixe : `set_sample_time_s(T)` puis `step(consigne)` sans second argument (`dt` reste disponible si la période varie ou pour tests).

Découplage acquisition / calcul : `update()` (courant seul, ou courant + position via la cascade) puis `step(...)` ; si `update()` n’est pas appelé, `step` relit les capteurs comme avant.

La sortie de la boucle courant est bornée (`set_output_limits`) : à relier à `IMotorPwmDriver::set_command` (ou autre driver) côté application.
