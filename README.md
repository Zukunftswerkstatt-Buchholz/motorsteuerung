# DRV8833 Dual-Motor + Encoder (Calliope mini v3)

Steuerung von **zwei DC-Getriebemotoren** über **DRV8833** mit **Quadratur-Encodern** und **PID-Drehzahlregelung**.  
Optimiert für **Calliope mini v3** und getestete N20-Encoder (DFRobot, 120:1, ~960 CPR @ 2×).

**Features**
- Zwei Motoren (Links / Rechts) mit separatem PID-Regler (RPM-Soll ±)
- Präzises Drehen um Umdrehungen (inkl. Feinkorrektur)
- Drehen **auf der Stelle** (Räder gegensinnig)
- **Einfach-Blöcke**: Richtung als Enum (Vorwärts/Rückwärts), Vorzeichen wird intern gesetzt
- **Pro-Blöcke**: Vorzeichen/RPM/Parameter direkt vom Benutzer
- nSLEEP/SLP (Treiber aktiv) steuerbar

---

## Hardware & Pinbelegung

> ⚠️ **Gemeinsame Masse**: GND von Calliope und DRV8833 **verbinden**.  
> ⚠️ **VM (Motorversorgung)** separat einspeisen (z. B. 6 V, je nach Motor).  

### Treiber / Motoren (DRV8833)

| Motor | + | - | Calliope-Pin |
|---|---|---|---|
| **Links** | AIN1 | AIN2 | **P1** (PWM), **P0** (PWM) |
| **Rechts** | BIN1 | BIN2 | **P3** (PWM), **P2** (PWM) |
| **Treiber-Sleep** | nSLEEP (SLP) | – | **C8** (Digital) |


### Encoder (Quadratur)

| Rad | Kanal A | Kanal B | Calliope-Pin |
|---|---|---|---|
| **Links** | A | B | **C9**, **C13** |
| **Rechts** | A | B | **C14**, **C15** |

**Encoder-Zählung**: Im Beispiel gilt `CPR_1X = 1920`, das entspricht **2×-Zählung** (Rising+Falling eines Kanals).  

> ℹ️ In der Quelldatei heißen die Variablen `ENC_A/B/C/D`. Entscheidend sind die **physischen Pins laut Tabelle**. Wenn Drehrichtung „vertauscht" ist, kannst du **die Vorzeichen in der ISR tauschen** oder die **Motorkabel** am DRV8833 kreuzen.


---

## Schnellstart

```ts
// In on start:
Motorsteuerung.begin()

// Einfach: 1,5 Umdrehungen LINKS vorwärts
Motorsteuerung.rotateSimple(Motorsteuerung.Motor.Links, Motorsteuerung.RichtungEinfach.Vorwaerts, 1.5)

// Einfach: auf der Stelle 0,25 Umdrehungen nach Links (also gegen den Uhrzeigersinn)
Motorsteuerung.rotateOnSpotSimple(Motorsteuerung.RichtungEinfach.Links, 0.25)

// Pro: Drehzahl LINKS = +60 RPM, RECHTS = -60 RPM (gegensinnig)
Motorsteuerung.setRPM_Pro(Motorsteuerung.Motor.Links, 60)
Motorsteuerung.setRPM_Pro(Motorsteuerung.Motor.Rechts, -60)

// Stop
Motorsteuerung.stopAll()
```

---

## Blöcke

### Gruppe „Einfach"
- **Starten**  
  Initialisiert nSLEEP, Encoder, Serielle (115200) und startet die Regel-Loop.

- **drehe (Links / Rechts / Beide) (Vorwärts / Rückwärts) (x) Umdrehungen**  
  Richtung über Enum; das Vorzeichen wird intern gesetzt.  
  Für **„Beide"** drehen beide Räder gleichsinnig die gewünschte Umdrehungszahl.  
  (Feineinstellungen: feste Toleranz/PWM aus der Logik.)

- **drehe auf der Stelle (Vorwärts/Rückwärts) (x) Umdrehungen**  
  Räder gegensinnig; „Vorwärts"/„Rückwärts" bestimmt das Drehsinn-Vorzeichen.

- **STOP (bremsen)**  
  Deaktiviert PID und legt beide Brücken auf Bremse.

### Gruppe „Pro" (direkte Vorzeichen/Parameter)
- **setze RPM (Links/Rechts/Beide) auf (±rpm)**  
  RPM kann positiv/negativ sein. PID-Regler pro Motor aktiv/inaktiv je nach Soll.

- **drehe LINKS / RECHTS (Umdr., dir ±1, slowPWM, Tol)**  
  Feindrehfunktion mit Feinkorrektur-Pulsung. `dir = +1 / −1`.

- **auf der Stelle (Umdr., dir ±1, Basis-RPM, Tol)**  
  Gegensinniges Drehen mit frei wählbarer Basis-Drehzahl.

- **PID LINKS/RECHTS (Kp, Ki, Kd)**  
  Regler-Tuning pro Motor.

- **Grenzen LINKS/RECHTS (minPWM, maxPWM, Imax)**  
  Mindest-PWM gegen Haftreibung; Sättigungen für Reglerausgang und Integrator.

- **Treiber aktiv (true/false)**  
  nSLEEP/SLP auf High/Low.

- **RPM messen (Links/Rechts)** & **Encoder Ticks (Links/Rechts)**  
  Telemetrie/Diagnose.

---

## Treiber-Hinweise (DRV8833)

- **Logik**: 3,3 V kompatibel. **nSLEEP** kann auf 3,3 V **oder 5 V** (Pull-Up) – High = aktiv.  
- **PWM**: Gib PWM auf **genau einen** Eingang (IN1 *oder* IN2) → anderer Eingang Low.  
- **Bremse**: Beide Eingänge High (im Code: `motor*_Brake()`).  
- **Versorgung**: VM nach Motordaten (z. B. 6 V). **Abblocken** (100 nF + 10–100 µF) nahe am Treiber.  
- **Strom**: DRV8833 bis ≈1,5 A pro Brücke.

---

## Troubleshooting

- **Drehrichtung vertauscht**  
  → In der ISR die Vorzeichen-Zeilen tauschen **oder** Motorleitungen am DRV8833 kreuzen.

- **Motor „ruckt" oder steht bei kleinen RPM**  
  → `minPWM_*` erhöhen (Haftreibung), `Kp/Ki` feinjustieren, `Imax_*` beachten.

- **Encoder zählt „falsch herum" oder gar nicht**  
  → Pull-Ups setzen (im Code schon aktiviert), A/B-Leitungen checken, Pins wie in Tabelle.

- **I²C/Grove-Konflikte**  
  → In diesem Setup werden **P0/P1/P2/P3** für PWM genutzt, **C9/C13/C14/C15** für Encoder. I²C-Pins (C19/C20) sind **frei**. Falls du andere Pins nutzt: Bus-Kollisionen vermeiden.

---

## Lizenz

- Lizenz: MIT.  



#### Metadaten (verwendet für Suche, Rendering)

* for PXT/calliopemini
<script src="https://makecode.com/gh-pages-embed.js"></script><script>makeCodeRender("{{ site.makecode.home_url }}", "{{ site.github.owner_name }}/{{ site.github.repository_name }}");</script>
