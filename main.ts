/**
 * Calliope mini V3 + DRV8833 + 2× DFRobot N20-Encoder (120:1, 960 CPR @2×)
 * Motor A: AIN1=P1, AIN2=P0, Encoder: A=C9, B=C13
 * Motor B: BIN1=P2, BIN2=P3, Encoder: C=C14, D=C15
 * Versorgung: DRV8833 VM separat (z.B. 6V), unbedingt gemeinsame Masse!
 * Hinweis: Drehrichtung ggf. durch Tauschen von ++/-- im ISR anpassen.
 */

//% color=#FF6F00 icon="\uf085" block="Motor"
//% groups='["Einfach", "Pro"]'
namespace Motorsteuerung {
    // ---------- Pinbelegung (dein Original, unverändert in der Logik) ----------
    let AIN1 = DigitalPin.P1
    let AIN2 = DigitalPin.P0
    let BIN1 = DigitalPin.P3
    let BIN2 = DigitalPin.P2
    let SLP = DigitalPin.C8

    let ENC_C = DigitalPin.C9    // Quad A (Motor A)
    let ENC_D = DigitalPin.C13   // Quad B (Motor A)
    let ENC_A = DigitalPin.C14   // Quad A (Motor B)
    let ENC_B = DigitalPin.C15   // Quad B (Motor B)

    // ---------- Encoder-Parameter ----------
    const CPR_1X = 1920          // 2×-Zählung: 2x960 Imp./U (ein Kanal, Rising and Falling Edge)
    let encCountA = 0
    let encCountB = 0

    // ---------- Motorfunktionen (Logik 1:1) ----------
    function setMotorA_PWM(speed: number) {
        if (speed > 1023) speed = 1023
        if (speed < -1023) speed = -1023
        if (speed > 0) {
            pins.analogWritePin(<AnalogPin><number>AIN1, speed)
            pins.digitalWritePin(AIN2, 0)
        } else if (speed < 0) {
            pins.digitalWritePin(AIN1, 0)
            pins.analogWritePin(<AnalogPin><number>AIN2, -speed)
        } else {
            pins.digitalWritePin(AIN1, 0)
            pins.digitalWritePin(AIN2, 0)
        }
    }

    function setMotorB_PWM(speed: number) {
        if (speed > 1023) speed = 1023
        if (speed < -1023) speed = -1023
        if (speed > 0) {
            pins.analogWritePin(<AnalogPin><number>BIN1, speed)
            pins.digitalWritePin(BIN2, 0)
        } else if (speed < 0) {
            pins.digitalWritePin(BIN1, 0)
            pins.analogWritePin(<AnalogPin><number>BIN2, -speed)
        } else {
            pins.digitalWritePin(BIN1, 0)
            pins.digitalWritePin(BIN2, 0)
        }
    }

    function motorA_Brake() { pins.digitalWritePin(AIN1, 1); pins.digitalWritePin(AIN2, 1) }
    function motorB_Brake() { pins.digitalWritePin(BIN1, 1); pins.digitalWritePin(BIN2, 1) }
    function motorBrakeAll() { motorA_Brake(); motorB_Brake() }

    function drvEnable(on: boolean) { pins.digitalWritePin(SLP, on ? 1 : 0) }

    // ---------- Encoder-Setup & ISR (Logik 1:1) ----------
    function encoderInit() {
        pins.setPull(ENC_A, PinPullMode.PullUp)
        pins.setPull(ENC_B, PinPullMode.PullUp)
        pins.setPull(ENC_C, PinPullMode.PullUp)
        pins.setPull(ENC_D, PinPullMode.PullUp)

        // --- Motor B (ENC_A/ENC_B): Kanal A beide Flanken, B bestimmt Richtung ---
        pins.onPulsed(ENC_A, PulseValue.High, function () { // steigende A
            if (pins.digitalReadPin(ENC_B)) encCountA++; else encCountA--;
        })
        pins.onPulsed(ENC_A, PulseValue.Low, function () {  // fallende A -> Vorzeichen umkehren!
            if (pins.digitalReadPin(ENC_B)) encCountA--; else encCountA++;
        })

        // --- Motor A (ENC_C/ENC_D): Kanal A beide Flanken, D bestimmt Richtung ---
        pins.onPulsed(ENC_C, PulseValue.High, function () { // steigende A
            if (pins.digitalReadPin(ENC_D)) encCountB--; else encCountB++;
        })
        pins.onPulsed(ENC_C, PulseValue.Low, function () {  // fallende A -> Vorzeichen umkehren!
            if (pins.digitalReadPin(ENC_D)) encCountB++; else encCountB--;
        })
    }

    // ===== PID-Speed-Control (pro Motor) =====
    let pidEnabledA = false
    let pidEnabledB = false

    let speedSP_RPM_A = 0
    let speedSP_RPM_B = 0

    let measRPM_A = 0
    let measRPM_B = 0

    let lastPidCountA = 0
    let lastPidCountB = 0

    // PID-Parameter (Startwerte)
    let Kp_A = 5.0, Ki_A = 20.0, Kd_A = 0.0
    let Kp_B = 5.0, Ki_B = 20.0, Kd_B = 0.0

    // Reglergrenzen / Mindest-PWM (gegen Haftreibung je Motor separat)
    let maxPWM_A = 1023, minPWM_A = 140, Imax_A = 900
    let maxPWM_B = 1023, minPWM_B = 140, Imax_B = 900

    // Zustände
    let pidI_A = 0, lastErr_A = 0
    let pidI_B = 0, lastErr_B = 0

    // Abtastzeit
    const sample_ms = 100
    const sample_s = sample_ms / 1000

    function setSpeedRPM_A(rpm: number) {
        speedSP_RPM_A = rpm
        pidI_A = 0; lastErr_A = 0
        pidEnabledA = Math.abs(rpm) > 0
        if (!pidEnabledA) { setMotorA_PWM(0); motorA_Brake() }
    }
    function setSpeedRPM_B(rpm: number) {
        speedSP_RPM_B = rpm
        pidI_B = 0; lastErr_B = 0
        pidEnabledB = Math.abs(rpm) > 0
        if (!pidEnabledB) { setMotorB_PWM(0); motorB_Brake() }
    }
    function stopPID_All() {
        pidEnabledA = false; pidEnabledB = false
        setMotorA_PWM(0); setMotorB_PWM(0)
        motorBrakeAll()
    }

    // ===== Genau-drehen-Funktionen (pro Motor) =====
    function rotateTurnsA(turns: number = 1, dir: number = 1, slowPWM: number = 300, toleranceCounts: number = 7) {
        const target = encCountA + dir * Math.round(turns * CPR_1X)
        const wasA = pidEnabledA
        if (wasA) setSpeedRPM_A(0)
        serial.writeLine("Rotate A")
        setMotorA_PWM(dir * slowPWM)
        while (dir * (target - encCountA) > toleranceCounts) {
            basic.pause(1)
        }
        motorA_Brake(); basic.pause(30)

        // Feinkorrektur
        let err = target - encCountA
        let attempts = 0
        while (Math.abs(err) > toleranceCounts && attempts < 6) {
            const stepDir = err > 0 ? 1 : -1
            setMotorA_PWM(stepDir * 200); basic.pause(10)
            motorA_Brake(); basic.pause(20)
            err = target - encCountA; attempts++
        }
        motorA_Brake()
        let error = target - encCountA
        serial.writeLine("Target=" + target + "  |  encCountA=" + encCountA + "  |  Error=" + error)
        if (wasA && speedSP_RPM_A != 0) setSpeedRPM_A(speedSP_RPM_A)
    }

    function rotateTurnsB(turns: number = 1, dir: number = 1, slowPWM: number = 300, toleranceCounts: number = 7) {
        const target = encCountB + dir * Math.round(turns * CPR_1X)
        const wasB = pidEnabledB
        if (wasB) setSpeedRPM_B(0)
        serial.writeLine("Rotate B")
        setMotorB_PWM(dir * slowPWM)
        while (dir * (target - encCountB) > toleranceCounts) {
            basic.pause(1)
        }

        motorB_Brake(); basic.pause(30)

        // Feinkorrektur
        let err = target - encCountB
        let attempts = 0
        while (Math.abs(err) > toleranceCounts && attempts < 6) {
            const stepDir = err > 0 ? 1 : -1
            setMotorB_PWM(stepDir * 200); basic.pause(10)
            motorB_Brake(); basic.pause(20)
            err = target - encCountB; attempts++
        }
        motorB_Brake()
        let error = target - encCountB
        serial.writeLine("Target=" + target + "  |  encCountB=" + encCountB + "  |  Error=" + error)
        if (wasB && speedSP_RPM_B != 0) setSpeedRPM_B(speedSP_RPM_B)
    }

    function rotate(
        turns: number = 1,           // Umdrehungen pro Rad
        dir: number = 1,             // +1 oder -1: Drehrichtung (A=dir, B=-dir)
        baseRPM: number = 10,        // Start-/Arbeits-RPM
        toleranceCounts: number = 7, // Toleranz in Encoder-Counts
    ) {
        const sgnA = dir
        const sgnB = -dir

        const targetA = encCountA + sgnA * Math.round(turns * CPR_1X)
        const targetB = encCountB + sgnB * Math.round(turns * CPR_1X)

        // PID-Zustand sichern & deaktivieren
        const wasA = pidEnabledA, wasB = pidEnabledB
        if (wasA) setSpeedRPM_A(0)
        if (wasB) setSpeedRPM_B(0)

        serial.writeLine("Rotate On-Spot (RPM)")

        let runA = true, runB = true
        setSpeedRPM_B(sgnB * baseRPM)
        setSpeedRPM_A(sgnA * baseRPM)

        while (runA || runB) {
            // Rest-Counts positiv in Bewegungsrichtung
            const remA = sgnA * (targetA - encCountA)
            const remB = sgnB * (targetB - encCountB)

            // Ziel erreicht?
            if (runA && remA <= toleranceCounts) {
                setSpeedRPM_A(0);
                motorA_Brake();
                runA = false
            }
            if (runB && remB <= toleranceCounts) {
                setSpeedRPM_B(0);
                motorB_Brake();
                runB = false
            }
            if (!runA && !runB) break
            basic.pause(1)
            serial.writeLine("A: " + encCountA + " | B: " + encCountB)
        }
        setSpeedRPM_A(0);
        setSpeedRPM_B(0);
        // Telemetrie
        serial.writeLine(
            "On-Spot done | A tgt=" + targetA + " enc=" + encCountA +
            " | B tgt=" + targetB + " enc=" + encCountB
        )
        // PID ggf. wieder aktivieren
        if (wasA && speedSP_RPM_A != 0) setSpeedRPM_A(speedSP_RPM_A)
        if (wasB && speedSP_RPM_B != 0) setSpeedRPM_B(speedSP_RPM_B)
    }

    function rotateBothSimple(turns: number, dir: number, baseRPM: number = 10, toleranceCounts: number = 7) {
        const sgnA = dir
        const sgnB = dir

        const targetA = encCountA + sgnA * Math.round(turns * CPR_1X)
        const targetB = encCountB + sgnB * Math.round(turns * CPR_1X)

        const wasA = pidEnabledA, wasB = pidEnabledB
        if (wasA) setSpeedRPM_A(0)
        if (wasB) setSpeedRPM_B(0)

        serial.writeLine("Rotate BOTH (same dir)")

        let runA = true, runB = true
        setSpeedRPM_A(sgnA * baseRPM)
        setSpeedRPM_B(sgnB * baseRPM)

        while (runA || runB) {
            const remA = sgnA * (targetA - encCountA)
            const remB = sgnB * (targetB - encCountB)

            if (runA && remA <= toleranceCounts) { setSpeedRPM_A(0); motorA_Brake(); runA = false }
            if (runB && remB <= toleranceCounts) { setSpeedRPM_B(0); motorB_Brake(); runB = false }
            if (!runA && !runB) break
            basic.pause(1)
        }
        setSpeedRPM_A(0); setSpeedRPM_B(0);

        if (wasA && speedSP_RPM_A != 0) setSpeedRPM_A(speedSP_RPM_A)
        if (wasB && speedSP_RPM_B != 0) setSpeedRPM_B(speedSP_RPM_B)
    }

    // ---------- Gemeinsamer PID-Regelkreis & Telemetrie (als Startfunktion gekapselt) ----------
    let _started = false
    function _runPidLoopOnce() {
        control.inBackground(function () {
            lastPidCountA = encCountA
            lastPidCountB = encCountB

            const alpha = 0.5 // Low-Pass für RPM (0..1), 1 = kein Filter

            while (true) {
                // Delta-Counts seit letztem Sample
                const nowA = encCountA
                const nowB = encCountB
                const dA = nowA - lastPidCountA
                const dB = nowB - lastPidCountB
                lastPidCountA = nowA
                lastPidCountB = nowB

                // RPM berechnen (signiert)
                const instRPM_A = dA * (60000 / (CPR_1X * sample_ms))
                const instRPM_B = dB * (60000 / (CPR_1X * sample_ms))
                measRPM_A = alpha * instRPM_A + (1 - alpha) * measRPM_A
                measRPM_B = alpha * instRPM_B + (1 - alpha) * measRPM_B

                // ----- PID A -----
                if (pidEnabledA) {
                    const errA = speedSP_RPM_A - measRPM_A
                    pidI_A += Ki_A * errA * sample_s
                    if (pidI_A > Imax_A) pidI_A = Imax_A
                    if (pidI_A < -Imax_A) pidI_A = -Imax_A
                    const dAerr = (errA - lastErr_A) / sample_s
                    let uA = Kp_A * errA + pidI_A + Kd_A * dAerr
                    lastErr_A = errA

                    if (uA > maxPWM_A) uA = maxPWM_A
                    if (uA < -maxPWM_A) uA = -maxPWM_A
                    if (Math.abs(uA) > 0 && Math.abs(uA) < minPWM_A) uA = uA > 0 ? minPWM_A : -minPWM_A
                    setMotorA_PWM(Math.round(uA))
                }

                // ----- PID B -----
                if (pidEnabledB) {
                    const errB = speedSP_RPM_B - measRPM_B
                    pidI_B += Ki_B * errB * sample_s
                    if (pidI_B > Imax_B) pidI_B = Imax_B
                    if (pidI_B < -Imax_B) pidI_B = -Imax_B
                    const dBerr = (errB - lastErr_B) / sample_s
                    let uB = Kp_B * errB + pidI_B + Kd_B * dBerr
                    lastErr_B = errB

                    if (uB > maxPWM_B) uB = maxPWM_B
                    if (uB < -maxPWM_B) uB = -maxPWM_B
                    if (Math.abs(uB) > 0 && Math.abs(uB) < minPWM_B) uB = uB > 0 ? minPWM_B : -minPWM_B
                    setMotorB_PWM(Math.round(uB))
                }

                // (optional) Telemetrie
                //serial.writeLine("A: SP=" + Math.round(speedSP_RPM_A) + " | MEAS=" + Math.round(measRPM_A))
                //serial.writeLine("B: SP=" + Math.round(speedSP_RPM_B) + " | MEAS=" + Math.round(measRPM_B))

                basic.pause(sample_ms)
            }
        })
    }

    // ====== Öffentliche Blöcke ======

    // --- Enums & UI ---
    export enum Motor { Links = 0, Rechts = 1, Beide = 2 }
    export enum RichtungEinfach { Vorwaerts = 1, Rueckwaerts = -1 }
    export enum RichtungDrehung { Links = 1, Rueckwaerts = -1 }

    //% color=#5B9BD5 icon="\uf1b9" block="CalliopeBot"
    //% groups='["Einfach", "Pro"]'

    /**
     * Initialisiert Treiber, Encoder und startet die Regelschleife.
     */
    //% block="Initialisieren"
    //% group="Einfach"
    //% weight=10
    export function begin() {
        if (_started) return
        _started = true
        serial.redirectToUSB()
        serial.setBaudRate(BaudRate.BaudRate115200)
        drvEnable(true)
        encoderInit()
        serial.writeLine("DRV8833 Dual + Encoder gestartet | 960 CPR @2×")
        _runPidLoopOnce()
    }

    // -------------------- EINFACH --------------------


    /**
     * Mit konstanter Drehzahl drehen.
     */
    //% block="drehe %motor mit %rpm RPM in Richtung %richtung"
    //% group="Einfach"
    //% turns.defl=1
    //% rpm.min=0 rpm.defl=20 rpm.max=90
    //% weight=80
    export function drive(motor: Motor, richtung: RichtungEinfach, rpm: number) {
        const dir = (richtung as number) | 0; // +1/-1
        if (motor == Motor.Links) {
            setSpeedRPM_A(dir * rpm)
        } else if (motor == Motor.Rechts) {
            setSpeedRPM_B(dir * rpm)
        } else {
            setSpeedRPM_A(dir * rpm)
            setSpeedRPM_B(dir * rpm)
        }
    }

    /**
     * Mit konstanter Drehzahl fahren.
     */
    //% block="fahre %richtung mit %rpm RPM"
    //% group="Einfach"
    //% milliSeconds.defl=1000
    //% rpm.min=0 rpm.defl=20 rpm.max=90
    //% weight=100
    export function driveBoth(richtung: RichtungEinfach, rpm: number) {
        const dir = (richtung as number) | 0; // +1/-1
        setSpeedRPM_A(dir * rpm)
        setSpeedRPM_B(dir * rpm)
    }

    /**
     * Mit konstanter Drehzahl fahren für bestimmte Zeit.
     */
    //% block="fahre %richtung mit %rpm RPM für %milliSeconds Millisekunden"
    //% group="Einfach"
    //% milliSeconds.defl=1000
    //% rpm.min=0 rpm.defl=20 rpm.max=90
    //% weight=90
    export function driveMilliSeconds(richtung: RichtungEinfach, rpm: number, milliSeconds: number) {
        const dir = (richtung as number) | 0; // +1/-1
        setSpeedRPM_A(dir * rpm)
        setSpeedRPM_B(dir * rpm)
        basic.pause(milliSeconds)
        setSpeedRPM_A(0)
        setSpeedRPM_B(0)
    }

    /**
     * Drehe Motor LINKS/RECHTS/BEIDE um Umdrehungen.
     */
    //% block="drehe %motor %richtung um %turns Umdrehungen"
    //% group="Einfach"
    //% turns.min=0 turns.defl=1
    //% weight=60
    export function rotateSimple(motor: Motor, richtung: RichtungEinfach, turns: number) {
        const dir = (richtung as number) | 0; // +1/-1
        if (motor == Motor.Links) {
            rotateTurnsA(turns, dir, 300, 7)
        } else if (motor == Motor.Rechts) {
            rotateTurnsB(turns, dir, 300, 7)
        } else {
            rotateBothSimple(turns, dir, 10, 7)
        }
    }

    /**
     * Auf-der-Stelle drehen (Räder gegensinnig).
     */
    //% block="drehe auf der Stelle nach %richtung für %turns Umdrehungen"
    //% group="Einfach"
    //% turns.min=0 turns.defl=1
    //% weight=70
    export function rotateOnSpotSimple(richtung: RichtungDrehung, turns: number) {
        const dir = (richtung as number) | 0; // +1/-1
        rotate(turns, dir, 10, 7)
    }

    /**
     * Stoppt beide Motoren (bremst) und deaktiviert PID.
     */
    //% block="STOP (bremsen)"
    //% group="Einfach"
    export function stopAll() { stopPID_All() }

    // -------------------- PRO (Untermenü) --------------------

    /**
     * Setze Soll-Drehzahl Motor A (RPM, Vorzeichen = Richtung).
     */
    //% block="Motor A RPM auf %rpm setzen"
    //% group="Pro" advanced=true
    export function setRPM_A(rpm: number) { setSpeedRPM_A(rpm) }

    /**
     * Setze Soll-Drehzahl Motor B (RPM, Vorzeichen = Richtung).
     */
    //% block="Motor B RPM auf %rpm setzen"
    //% group="Pro" advanced=true
    export function setRPM_B(rpm: number) { setSpeedRPM_B(rpm) }

    /**
     * Feindrehung Motor A (alle Parameter).
     */
    //% block="Motor A drehen |Umdr. %turns |Richtung %dir |langsam-PWM %slowPWM |Toleranz %tol"
    //% group="Pro" advanced=true
    //% turns.defl=1 slowPWM.defl=300 tol.defl=7
    export function rotateA_Pro(turns: number, dir: RichtungEinfach, slowPWM: number, tol: number) {
        rotateTurnsA(turns, dir  as number, slowPWM, tol)
    }

    /**
     * Feindrehung Motor B (alle Parameter).
     */
    //% block="Motor B drehen |Umdr. %turns |Richtung %dir |langsam-PWM %slowPWM |Toleranz %tol"
    //% group="Pro" advanced=true
    //% turns.defl=1 slowPWM.defl=300 tol.defl=7
    export function rotateB_Pro(turns: number, dir: RichtungEinfach, slowPWM: number, tol: number) {
        rotateTurnsB(turns, dir as number, slowPWM, tol)
    }

    /**
     * Auf-der-Stelle drehen (alle Parameter).
     */
    //% block="auf der Stelle |Umdr. %turns |Richtung %dir |Basis-RPM %baseRPM |Toleranz %tol"
    //% group="Pro" advanced=true
    //% turns.defl=1 baseRPM.defl=10 tol.defl=7
    export function rotateOnSpot_Pro(turns: number, dir: RichtungEinfach, baseRPM: number, tol: number) {
        rotate(turns, dir as number, baseRPM, tol)
    }

    /**
     * PID-Parameter für Motor A.
     */
    //% block="PID Motor A |Kp %kp |Ki %ki |Kd %kd"
    //% group="Pro" advanced=true
    //% kp.defl=5 ki.defl=20 kd.defl=0
    export function setPID_A(kp: number, ki: number, kd: number) {
        Kp_A = kp; Ki_A = ki; Kd_A = kd
    }

    /**
     * PID-Parameter für Motor B.
     */
    //% block="PID Motor B |Kp %kp |Ki %ki |Kd %kd"
    //% group="Pro" advanced=true
    //% kp.defl=5 ki.defl=20 kd.defl=0
    export function setPID_B(kp: number, ki: number, kd: number) {
        Kp_B = kp; Ki_B = ki; Kd_B = kd
    }

    /**
     * PWM-Grenzen und Imax für Motor A.
     */
    //% block="Grenzen Motor A |minPWM %min |maxPWM %max |Imax %imax"
    //% group="Pro" advanced=true
    //% min.defl=140 max.defl=1023 imax.defl=900
    export function setLimits_A(min: number, max: number, imax: number) {
        minPWM_A = min; maxPWM_A = max; Imax_A = imax
    }

    /**
     * PWM-Grenzen und Imax für Motor B.
     */
    //% block="Grenzen Motor B |minPWM %min |maxPWM %max |Imax %imax"
    //% group="Pro" advanced=true
    //% min.defl=140 max.defl=1023 imax.defl=900
    export function setLimits_B(min: number, max: number, imax: number) {
        minPWM_B = min; maxPWM_B = max; Imax_B = imax
    }

    /**
     * Treiber aktivieren/deaktivieren (nSLEEP).
     */
    //% block="Treiber aktiv %on"
    //% group="Pro" advanced=true
    export function setDriverEnabled(on: boolean) { drvEnable(on) }

    /**
     * Aktuelle RPM (gemessen).
     */
    //% block="RPM messen %motor"
    //% group="Pro" advanced=true
    export function rpm(motor: Motor): number { return motor == Motor.Links ? measRPM_A : measRPM_B }

    /**
     * Encoder-Zählerstand.
     */
    //% block="Encoder Ticks %motor"
    //% group="Pro" advanced=true
    export function encoderTicks(motor: Motor): number { return motor == Motor.Rechts ? encCountA : encCountB }
}
