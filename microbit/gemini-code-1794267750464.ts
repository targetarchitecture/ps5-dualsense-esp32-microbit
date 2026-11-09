// MakeCode / pxt-microbit TypeScript
const ESP32_ADDR = 0x42
const FRAME_LEN = 10

const CMD_RUMBLE = 0x01
const CMD_COLOR = 0x02

// Pre-allocated outbound I2C buffer to prevent heap churn
const txBuf = pins.createBuffer(6)

enum ButtonMask {
    A = 1 << 0,
    B = 1 << 1,
    X = 1 << 2,
    Y = 1 << 3,
    L1 = 1 << 4,
    R1 = 1 << 5,
    L2 = 1 << 6,
    R2 = 1 << 7,
    ThumbL = 1 << 8,
    ThumbR = 1 << 9,
    Select = 1 << 10,
    Start = 1 << 11,
    System = 1 << 12
}

enum DPadMask {
    Up = 1 << 0,
    Down = 1 << 1,
    Right = 1 << 2,
    Left = 1 << 3
}

namespace ControllerState {
    export let connected = false
    export let buttons = 0
    export let dpad = 0
    export let leftX = 0
    export let leftY = 0
    export let rightX = 0
    export let rightY = 0
    export let brake = 0
    export let throttle = 0

    export function reset(): void {
        connected = false
        buttons = 0
        dpad = 0
        leftX = 0
        leftY = 0
        rightX = 0
        rightY = 0
        brake = 0
        throttle = 0
    }

    export function isButtonPressed(mask: ButtonMask): boolean {
        return (buttons & mask) !== 0
    }

    export function isDPadPressed(mask: DPadMask): boolean {
        return (dpad & mask) !== 0
    }
}

function sendCommand(cmd: number, rumbleLeft: number, rumbleRight: number, r: number, g: number, b: number): void {
    txBuf.setNumber(NumberFormat.UInt8LE, 0, cmd & 0xff)
    txBuf.setNumber(NumberFormat.UInt8LE, 1, rumbleLeft & 0xff)
    txBuf.setNumber(NumberFormat.UInt8LE, 2, rumbleRight & 0xff)
    txBuf.setNumber(NumberFormat.UInt8LE, 3, r & 0xff)
    txBuf.setNumber(NumberFormat.UInt8LE, 4, g & 0xff)
    txBuf.setNumber(NumberFormat.UInt8LE, 5, b & 0xff)
    pins.i2cWriteBuffer(ESP32_ADDR, txBuf, false)
}

function setRumble(left: number, right: number): void {
    sendCommand(CMD_RUMBLE, left, right, 0, 0, 0)
}

function setLightbar(r: number, g: number, b: number): void {
    sendCommand(CMD_COLOR, 0, 0, r, g, b)
}

function pollPad(): boolean {
    const buf = pins.i2cReadBuffer(ESP32_ADDR, FRAME_LEN, false)
    if (!buf || buf.length !== FRAME_LEN) {
        ControllerState.reset()
        return false
    }

    // Explicitly check for 1 to reject 0xFF bus NACK states
    const isConnected = buf.getNumber(NumberFormat.UInt8LE, 0) === 1
    if (!isConnected) {
        ControllerState.reset()
        return true
    }

    ControllerState.connected = true
    ControllerState.buttons = buf.getNumber(NumberFormat.UInt16LE, 1)
    ControllerState.dpad = buf.getNumber(NumberFormat.UInt8LE, 3)
    ControllerState.leftX = buf.getNumber(NumberFormat.Int8LE, 4)
    ControllerState.leftY = buf.getNumber(NumberFormat.Int8LE, 5)
    ControllerState.rightX = buf.getNumber(NumberFormat.Int8LE, 6)
    ControllerState.rightY = buf.getNumber(NumberFormat.Int8LE, 7)
    ControllerState.brake = buf.getNumber(NumberFormat.UInt8LE, 8)
    ControllerState.throttle = buf.getNumber(NumberFormat.UInt8LE, 9)
    return true
}

function setLed(x: number, y: number, state: boolean): void {
    if (state) {
        led.plot(x, y)
    } else {
        led.unplot(x, y)
    }
}

// ==========================================
// FIBER 1: Dedicated I2C Data Intake Loop
// ==========================================
basic.forever(function () {
    const ok = pollPad()
    basic.pause(ok ? 15 : 5)
})

// ==========================================
// FIBER 2: Independent Display Renderer Loop
// ==========================================
control.inBackground(function () {
    let wasConnected = false

    while (true) {
        if (ControllerState.connected) {
            if (!wasConnected) {
                basic.clearScreen()
                led.plot(0, 0)
                wasConnected = true
            }

            setLed(2, 2, ControllerState.isButtonPressed(ButtonMask.A))
            setLed(2, 0, ControllerState.isDPadPressed(DPadMask.Up))
            setLed(0, 2, ControllerState.leftY < -20)
        } else {
            if (wasConnected) {
                basic.clearScreen()
                wasConnected = false
            }
            led.plot(4, 4)
        }

        basic.pause(30)
    }
})

// Async Outbound Triggers
input.onButtonPressed(Button.A, function () {
    setRumble(200, 200)
    basic.pause(50)
    setLightbar(randint(0, 255), randint(0, 255), randint(0, 255))
})