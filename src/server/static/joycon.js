/**
 * WebHID Joy-Con Controller for Remote Robot Arm Control
 * 
 * This module connects to Nintendo Joy-Con controllers via WebHID API
 * and converts their input (sticks, buttons, gyro/accelerometer) into
 * robot arm control commands.
 * 
 * Left Joy-Con controls the left arm, Right Joy-Con controls the right arm.
 */

// Nintendo Joy-Con HID constants
const JOYCON_VENDOR_ID = 0x057e;
const JOYCON_L_PRODUCT_ID = 0x2006;
const JOYCON_R_PRODUCT_ID = 0x2007;

// Input report IDs
const INPUT_REPORT_ID_FULL = 0x30;
const INPUT_REPORT_ID_SIMPLE = 0x3f;

// Output report command IDs
const OUTPUT_REPORT_ID = 0x01;
const RUMBLE_ONLY_REPORT_ID = 0x10;

// Subcommand IDs
const SUBCOMMAND_SET_INPUT_MODE = 0x03;
const SUBCOMMAND_ENABLE_IMU = 0x40;
const SUBCOMMAND_SET_PLAYER_LIGHTS = 0x30;
const SUBCOMMAND_SPI_FLASH_READ = 0x10;

// Input modes
const INPUT_MODE_STANDARD_FULL = 0x30;

// Default rumble data (neutral)
const RUMBLE_NEUTRAL = new Uint8Array([0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40]);

/**
 * Low-pass filter for smoothing values
 */
class LowPassFilter {
    constructor(alpha = 0.1) {
        this.alpha = alpha;
        this.prevValue = 0;
    }
    
    update(newValue) {
        this.prevValue = this.alpha * newValue + (1 - this.alpha) * this.prevValue;
        return this.prevValue;
    }
    
    reset() {
        this.prevValue = 0;
    }
}

/**
 * Attitude estimator using complementary filter for gyro + accelerometer fusion
 */
class AttitudeEstimator {
    constructor() {
        this.pitch = 0;
        this.roll = 0;
        this.yaw = 0;
        this.dt = 0.015; // 66.67 Hz update rate
        this.alpha = 0.55; // Complementary filter coefficient
        
        this.lpfRoll = new LowPassFilter(0.05);
        this.lpfPitch = new LowPassFilter(0.05);
        
        // Yaw tracking using quaternion-like accumulation
        this.yawAccumulator = 0;

        // Gyro bias tracking for yaw drift reduction (no magnetometer available)
        this.yawBias = 0;
        this.yawBiasAlpha = 0.02; // how quickly bias adapts when stationary
    }
    
    reset() {
        this.pitch = 0;
        this.roll = 0;
        this.yaw = 0;
        this.yawAccumulator = 0;
        this.yawBias = 0;
        this.lpfRoll.reset();
        this.lpfPitch.reset();
    }

    calibrateYawBias(currentGz) {
        // Use current Z gyro as the bias and reset accumulated yaw.
        this.yawBias = currentGz;
        this.yawAccumulator = 0;
        this.yaw = 0;
    }
    
    update(gyroRad, accelG) {
        const [gx, gy, gz] = gyroRad;
        const [ax, ay, az] = accelG;
        
        // Calculate pitch and roll from accelerometer
        const rollAcc = Math.atan2(ay, -az);
        const pitchAcc = Math.atan2(ax, Math.sqrt(ay * ay + az * az));
        
        // Update with gyro integration
        this.pitch += gy * this.dt;
        this.roll -= gx * this.dt;
        
        // Complementary filter fusion
        this.pitch = this.alpha * this.pitch + (1 - this.alpha) * pitchAcc;
        this.roll = this.alpha * this.roll + (1 - this.alpha) * rollAcc;
        
        // Low-pass filter final output
        this.pitch = this.lpfPitch.update(this.pitch);
        this.roll = this.lpfRoll.update(this.roll);
        
        // Yaw from gyro only (no magnetometer) => drift is expected.
        // Reduce drift by estimating a bias when the controller is stationary/flat.
        const accelMag = Math.sqrt(ax * ax + ay * ay + az * az);
        const gyroMag = Math.sqrt(gx * gx + gy * gy + gz * gz);
        const isStationary = gyroMag < 0.08 && Math.abs(accelMag - 1.0) < 0.15;

        if (isStationary) {
            this.yawBias = this.yawBiasAlpha * gz + (1 - this.yawBiasAlpha) * this.yawBias;
        }

        const gzUnbiased = gz - this.yawBias;
        this.yawAccumulator += gzUnbiased * this.dt;

        // If essentially not rotating, slowly decay accumulated yaw toward 0.
        if (Math.abs(gzUnbiased) < 0.01) {
            this.yawAccumulator *= 0.995;
        }
        this.yaw = this.yawAccumulator;
        
        // Scale for robot control
        const scaledRoll = this.roll * Math.PI / 1.5;
        const scaledPitch = this.pitch * Math.PI / 1.5;
        const scaledYaw = -this.yaw * Math.PI / 1.5;
        
        return { roll: scaledRoll, pitch: scaledPitch, yaw: scaledYaw };
    }
}

/**
 * Joy-Con controller class for WebHID
 */
class JoyCon {
    constructor(device) {
        this.device = device;
        this.isLeft = device.productId === JOYCON_L_PRODUCT_ID;
        this.isRight = device.productId === JOYCON_R_PRODUCT_ID;
        this.name = this.isLeft ? 'Left Joy-Con' : 'Right Joy-Con';
        
        // State
        this.connected = false;
        this.packetNumber = 0;
        
        // Button states
        this.buttons = {
            // Right Joy-Con buttons
            y: 0, x: 0, b: 0, a: 0,
            sr_r: 0, sl_r: 0, r: 0, zr: 0,
            // Left Joy-Con buttons  
            down: 0, up: 0, right: 0, left: 0,
            sr_l: 0, sl_l: 0, l: 0, zl: 0,
            // Shared buttons
            minus: 0, plus: 0,
            r_stick: 0, l_stick: 0,
            home: 0, capture: 0,
        };
        
        // Stick values (0-4095)
        this.stick = { horizontal: 2048, vertical: 2048 };
        this.stickCenter = { horizontal: 2048, vertical: 2048 };
        
        // IMU data
        this.accel = { x: 0, y: 0, z: 0 };
        this.gyro = { x: 0, y: 0, z: 0 };

        // Raw IMU samples (int16)
        this.accelRaw = { x: 0, y: 0, z: 0 };
        this.gyroRaw = { x: 0, y: 0, z: 0 };
        
        // Calibration
        this.accelOffset = { x: 0, y: 0, z: 0 };
        this.accelCoeff = { x: 1, y: 1, z: 1 };
        this.gyroOffset = { x: 0, y: 0, z: 0 };
        this.gyroCoeff = { x: 1, y: 1, z: 1 };
        
        // Attitude estimator
        this.attitude = new AttitudeEstimator();
        
        // Gripper state
        this.gripperState = 1.0; // 1 = open, 0 = closed
        this.gripperDirection = 1; // 1 = opening, -1 = closing
        this.lastGripperButtonState = 0;
        
        // Position control (EE position in meters)
        this.position = { x: 0.2, y: 0, z: 0.1 };
        this.positionOffset = { x: 0.2, y: 0, z: 0.1 };

        // Input smoothing to reduce jitter
        this.lpfStickH = new LowPassFilter(0.25);
        this.lpfStickV = new LowPassFilter(0.25);
        this._lastControlEnabled = false;
        
        // Callback
        this.onUpdate = null;
        this.onDisconnect = null;

        // Optional control enable gate (deadman switch)
        // When set, updateControl() will ignore all motion/gripper inputs unless this returns true.
        this.isControlEnabled = null;
    }
    
    async connect() {
        try {
            await this.device.open();
            this.connected = true;
            
            // Set up input report handler
            this.device.addEventListener('inputreport', (event) => this.handleInputReport(event));
            
            // Enable standard full input mode with IMU
            await this.setInputMode(INPUT_MODE_STANDARD_FULL);
            await this.enableIMU(true);
            
            // Set player LED to indicate connection
            await this.setPlayerLights(this.isLeft ? 0x01 : 0x08);
            
            console.log(`${this.name} connected`);
            return true;
        } catch (error) {
            console.error(`Failed to connect ${this.name}:`, error);
            return false;
        }
    }
    
    async disconnect() {
        if (this.connected) {
            try {
                await this.device.close();
            } catch (e) {
                console.warn('Error closing device:', e);
            }
            this.connected = false;
            console.log(`${this.name} disconnected`);
        }
    }
    
    async sendSubcommand(subcommand, data = new Uint8Array()) {
        const report = new Uint8Array(49);
        report[0] = OUTPUT_REPORT_ID;
        report[1] = this.packetNumber;
        this.packetNumber = (this.packetNumber + 1) & 0x0F;
        
        // Rumble data (neutral)
        report.set(RUMBLE_NEUTRAL, 2);
        
        // Subcommand
        report[10] = subcommand;
        
        // Subcommand data
        if (data.length > 0) {
            report.set(data, 11);
        }
        
        try {
            await this.device.sendReport(OUTPUT_REPORT_ID, report.slice(1));
        } catch (e) {
            console.warn('Failed to send subcommand:', e);
        }
    }
    
    async setInputMode(mode) {
        await this.sendSubcommand(SUBCOMMAND_SET_INPUT_MODE, new Uint8Array([mode]));
        await this.sleep(50);
    }
    
    async enableIMU(enable) {
        await this.sendSubcommand(SUBCOMMAND_ENABLE_IMU, new Uint8Array([enable ? 0x01 : 0x00]));
        await this.sleep(50);
    }
    
    async setPlayerLights(pattern) {
        await this.sendSubcommand(SUBCOMMAND_SET_PLAYER_LIGHTS, new Uint8Array([pattern]));
    }
    
    sleep(ms) {
        return new Promise(resolve => setTimeout(resolve, ms));
    }
    
    handleInputReport(event) {
        const { data, reportId } = event;
        
        if (reportId === INPUT_REPORT_ID_FULL) {
            this.parseFullReport(data);
        } else if (reportId === INPUT_REPORT_ID_SIMPLE) {
            this.parseSimpleReport(data);
        }
        
        // Update control logic
        this.updateControl();
        
        // Callback
        if (this.onUpdate) {
            this.onUpdate(this.getState());
        }
    }
    
    parseFullReport(data) {
        // Bytes 0: Timer
        // Byte 1: Battery + connection info
        // Bytes 2-4: Button states
        // Bytes 5-8: Left stick
        // Bytes 9-11: Right stick  
        // Byte 12: Vibrator input report
        // Bytes 13-48: 6-Axis sensor data (3 samples)
        
        this.parseButtons(data);
        this.parseSticks(data);
        this.parseIMU(data);
    }
    
    parseSimpleReport(data) {
        // Simple mode - less data, no IMU
        this.parseButtons(data);
        this.parseSticks(data);
    }
    
    parseButtons(data) {
        // Byte 2: Right buttons (Y, X, B, A, SR, SL, R, ZR)
        const byte2 = data.getUint8(2);
        this.buttons.y = (byte2 >> 0) & 1;
        this.buttons.x = (byte2 >> 1) & 1;
        this.buttons.b = (byte2 >> 2) & 1;
        this.buttons.a = (byte2 >> 3) & 1;
        this.buttons.sr_r = (byte2 >> 4) & 1;
        this.buttons.sl_r = (byte2 >> 5) & 1;
        this.buttons.r = (byte2 >> 6) & 1;
        this.buttons.zr = (byte2 >> 7) & 1;
        
        // Byte 3: Shared buttons (-, +, R Stick, L Stick, Home, Capture)
        const byte3 = data.getUint8(3);
        this.buttons.minus = (byte3 >> 0) & 1;
        this.buttons.plus = (byte3 >> 1) & 1;
        this.buttons.r_stick = (byte3 >> 2) & 1;
        this.buttons.l_stick = (byte3 >> 3) & 1;
        this.buttons.home = (byte3 >> 4) & 1;
        this.buttons.capture = (byte3 >> 5) & 1;
        
        // Byte 4: Left buttons (Down, Up, Right, Left, SR, SL, L, ZL)
        const byte4 = data.getUint8(4);
        this.buttons.down = (byte4 >> 0) & 1;
        this.buttons.up = (byte4 >> 1) & 1;
        this.buttons.right = (byte4 >> 2) & 1;
        this.buttons.left = (byte4 >> 3) & 1;
        this.buttons.sr_l = (byte4 >> 4) & 1;
        this.buttons.sl_l = (byte4 >> 5) & 1;
        this.buttons.l = (byte4 >> 6) & 1;
        this.buttons.zl = (byte4 >> 7) & 1;
    }
    
    parseSticks(data) {
        if (this.isLeft) {
            // Left stick: bytes 5-7
            const byte5 = data.getUint8(5);
            const byte6 = data.getUint8(6);
            const byte7 = data.getUint8(7);
            this.stick.horizontal = byte5 | ((byte6 & 0x0F) << 8);
            this.stick.vertical = (byte6 >> 4) | (byte7 << 4);
        } else {
            // Right stick: bytes 8-10
            const byte8 = data.getUint8(8);
            const byte9 = data.getUint8(9);
            const byte10 = data.getUint8(10);
            this.stick.horizontal = byte8 | ((byte9 & 0x0F) << 8);
            this.stick.vertical = (byte9 >> 4) | (byte10 << 4);
        }
    }
    
    parseIMU(data) {
        // IMU data starts at byte 12 (after vibrator input report at byte 12)
        // 3 samples of 12 bytes each (2 bytes per axis * 6 axes)
        // We use the first sample (most recent)
        
        const offset = 13; // First IMU sample
        
        // Accelerometer (16-bit signed, little-endian)
        const accelX = this.toInt16LE(data.getUint8(offset), data.getUint8(offset + 1));
        const accelY = this.toInt16LE(data.getUint8(offset + 2), data.getUint8(offset + 3));
        const accelZ = this.toInt16LE(data.getUint8(offset + 4), data.getUint8(offset + 5));
        
        // Gyroscope (16-bit signed, little-endian)
        const gyroX = this.toInt16LE(data.getUint8(offset + 6), data.getUint8(offset + 7));
        const gyroY = this.toInt16LE(data.getUint8(offset + 8), data.getUint8(offset + 9));
        const gyroZ = this.toInt16LE(data.getUint8(offset + 10), data.getUint8(offset + 11));
        
        // Store raw samples for robust calibration
        this.accelRaw.x = accelX;
        this.accelRaw.y = accelY;
        this.accelRaw.z = accelZ;
        this.gyroRaw.x = gyroX;
        this.gyroRaw.y = gyroY;
        this.gyroRaw.z = gyroZ;

        // Apply calibration and convert to standard units
        // Accelerometer: raw to g (gravity units)
        // Default sensitivity: 8192 LSB/g for ±4g range
        const accelScale = 1.0 / 8192.0;
        this.accel.x = (accelX - this.accelOffset.x) * accelScale * this.accelCoeff.x;
        this.accel.y = (accelY - this.accelOffset.y) * accelScale * this.accelCoeff.y;
        this.accel.z = (accelZ - this.accelOffset.z) * accelScale * this.accelCoeff.z;
        
        // Gyroscope: raw to rad/s
        // Default sensitivity: 13371 LSB/(rad/s) for ±2000 dps
        const gyroScale = 1.0 / 13371.0;
        this.gyro.x = (gyroX - this.gyroOffset.x) * gyroScale * this.gyroCoeff.x;
        this.gyro.y = (gyroY - this.gyroOffset.y) * gyroScale * this.gyroCoeff.y;
        this.gyro.z = (gyroZ - this.gyroOffset.z) * gyroScale * this.gyroCoeff.z;
    }
    
    toInt16LE(low, high) {
        const value = low | (high << 8);
        return value > 32767 ? value - 65536 : value;
    }
    
    calibrateStick() {
        // Set current stick position as center
        this.stickCenter.horizontal = this.stick.horizontal;
        this.stickCenter.vertical = this.stick.vertical;
        console.log(`${this.name} stick calibrated: center at (${this.stickCenter.horizontal}, ${this.stickCenter.vertical})`);
    }
    
    calibrateIMU() {
        // Set current raw IMU readings as offsets (should be at rest on flat surface)
        // Use RAW values so calibration remains correct even if offsets were previously applied.
        this.gyroOffset.x = this.gyroRaw.x;
        this.gyroOffset.y = this.gyroRaw.y;
        this.gyroOffset.z = this.gyroRaw.z;

        // Reset estimator and set yaw bias to current (near-zero) Z gyro.
        this.attitude.reset();
        this.attitude.calibrateYawBias(this.gyro.z);
        console.log(`${this.name} IMU calibrated`);
    }
    
    updateControl() {
        // Small incremental step per update tick (meters)
        const speedScale = 0.0008;

        const quantize = (value, step) => Math.round(value / step) * step;
        
        // Get attitude from IMU (keep updating even if controls are disabled)
        const gyroRad = [this.gyro.x, this.gyro.y, this.gyro.z];
        const accelG = [this.accel.x, this.accel.y, this.accel.z];
        const orientation = this.attitude.update(gyroRad, accelG);

        const controlEnabled = this.isControlEnabled ? this.isControlEnabled() : true;
        if (!controlEnabled) {
            this._lastControlEnabled = false;
            // Keep edge detector state in sync so we don't interpret a held button
            // as a new press when the deadman switch becomes enabled.
            const gripperButton = this.isRight ? this.buttons.a : this.buttons.down;
            this.lastGripperButtonState = gripperButton;
            return;
        }

        // Deadman just engaged: re-center sticks and reset filters to avoid a jump.
        if (!this._lastControlEnabled) {
            this.calibrateStick();
            this.lpfStickH.reset();
            this.lpfStickV.reset();
            const gripperButton = this.isRight ? this.buttons.a : this.buttons.down;
            this.lastGripperButtonState = gripperButton;
        }
        this._lastControlEnabled = true;
        
        // Stick control for X/Y position (with deadzone + smoothing)
        const stickH = this.stick.horizontal - this.stickCenter.horizontal;
        const stickV = this.stick.vertical - this.stickCenter.vertical;
        const stickThreshold = 450;
        const stickRange = 1700;

        const normalizeStick = (raw) => {
            if (Math.abs(raw) <= stickThreshold) return 0;
            const sign = raw >= 0 ? 1 : -1;
            const mag = Math.min(1, (Math.abs(raw) - stickThreshold) / (stickRange - stickThreshold));
            // Slight nonlinearity for fine control near center
            return sign * (mag * mag);
        };

        const stickNormV = this.lpfStickV.update(normalizeStick(stickV));
        const stickNormH = this.lpfStickH.update(normalizeStick(stickH));
        
        // Vertical stick: controls X (forward/back)
        if (stickNormV !== 0) {
            this.position.x += speedScale * stickNormV;
        }
        
        // Horizontal stick: controls Y (left/right)
        if (stickNormH !== 0) {
            this.position.y += speedScale * stickNormH;
        }
        
        // Z-axis button control (up/down)
        const buttonUp = this.isRight ? this.buttons.r : this.buttons.l;
        const buttonDown = this.isRight ? this.buttons.r_stick : this.buttons.l_stick;
        
        if (buttonUp) {
            this.position.z += speedScale;
        }
        if (buttonDown) {
            this.position.z -= speedScale;
        }
        
        // Home/Capture button: reset to offset position
        const resetButton = this.isRight ? this.buttons.home : this.buttons.capture;
        if (resetButton) {
            this.position.x = this.positionOffset.x;
            this.position.y = this.positionOffset.y;
            this.position.z = this.positionOffset.z;
            this.attitude.reset();
        }
        
        // Gripper control
        // NOTE: ZR/ZL are reserved for the global deadman switch (LT/RT).
        // Use A (right Joy-Con) / Down (left Joy-Con) for gripper.
        const gripperButton = this.isRight ? this.buttons.a : this.buttons.down;
        if (gripperButton && !this.lastGripperButtonState) {
            // Button just pressed - toggle direction
            this.gripperDirection *= -1;
        }
        this.lastGripperButtonState = gripperButton;
        
        // Hold ZR/ZL to actuate gripper
        if (gripperButton) {
            const gripperSpeed = 0.02;
            this.gripperState += gripperSpeed * this.gripperDirection;
            this.gripperState = Math.max(0, Math.min(1, this.gripperState));
        }
        
        // Position limits
        this.position.x = Math.max(0.1, Math.min(0.4, this.position.x));
        this.position.y = Math.max(-0.3, Math.min(0.3, this.position.y));
        this.position.z = Math.max(0.0, Math.min(0.3, this.position.z));

        // Quantize to reduce micro-jitter in repeated commands
        this.position.x = quantize(this.position.x, 0.0005);
        this.position.y = quantize(this.position.y, 0.0005);
        this.position.z = quantize(this.position.z, 0.0005);
    }
    
    getState() {
        const orientation = {
            roll: this.attitude.roll,
            pitch: this.attitude.pitch,
            yaw: this.attitude.yaw
        };
        
        return {
            side: this.isLeft ? 'left' : 'right',
            position: { ...this.position },
            orientation: orientation,
            gripper: this.gripperState,
            buttons: { ...this.buttons },
            stick: { ...this.stick },
            accel: { ...this.accel },
            gyro: { ...this.gyro }
        };
    }
}

/**
 * Joy-Con Manager for handling multiple controllers
 */
class JoyConManager {
    constructor() {
        this.leftJoyCon = null;
        this.rightJoyCon = null;
        this.onStateUpdate = null;
        this.sendInterval = null;
        this.sendRate = 50; // ms between sends (20 Hz)
    }

    isDeadmanEngaged() {
        const leftState = this.getLeftState();
        const rightState = this.getRightState();

        if (!leftState || !rightState) return false;

        // Deadman switch: require BOTH triggers held.
        // Joy-Con has both shoulder (L/R) and trigger (ZL/ZR) buttons.
        // Accept either pair to match user expectations for "LT/RT".
        const leftTrigger = Boolean(leftState.buttons?.zl || leftState.buttons?.l);
        const rightTrigger = Boolean(rightState.buttons?.zr || rightState.buttons?.r);
        return leftTrigger && rightTrigger;
    }
    
    async requestDevice(type = 'any') {
        const filters = [];
        
        if (type === 'left' || type === 'any') {
            filters.push({ vendorId: JOYCON_VENDOR_ID, productId: JOYCON_L_PRODUCT_ID });
        }
        if (type === 'right' || type === 'any') {
            filters.push({ vendorId: JOYCON_VENDOR_ID, productId: JOYCON_R_PRODUCT_ID });
        }
        
        try {
            const devices = await navigator.hid.requestDevice({ filters });
            
            for (const device of devices) {
                const joycon = new JoyCon(device);
                const success = await joycon.connect();
                
                if (success) {
                    if (joycon.isLeft) {
                        if (this.leftJoyCon) {
                            await this.leftJoyCon.disconnect();
                        }
                        this.leftJoyCon = joycon;
                    } else {
                        if (this.rightJoyCon) {
                            await this.rightJoyCon.disconnect();
                        }
                        this.rightJoyCon = joycon;
                    }

                    // Global deadman switch: both LT/RT (ZL+ZR) must be held.
                    // Until then, ignore all movement/gripper updates.
                    joycon.isControlEnabled = () => this.isDeadmanEngaged();
                    
                    joycon.onUpdate = (state) => {
                        if (this.onStateUpdate) {
                            this.onStateUpdate(state);
                        }
                    };
                }
            }
            
            return devices.length > 0;
        } catch (error) {
            console.error('Failed to request Joy-Con:', error);
            return false;
        }
    }
    
    async disconnectAll() {
        if (this.sendInterval) {
            clearInterval(this.sendInterval);
            this.sendInterval = null;
        }
        
        if (this.leftJoyCon) {
            await this.leftJoyCon.disconnect();
            this.leftJoyCon = null;
        }
        if (this.rightJoyCon) {
            await this.rightJoyCon.disconnect();
            this.rightJoyCon = null;
        }
    }
    
    calibrate() {
        if (this.leftJoyCon) {
            this.leftJoyCon.calibrateStick();
            this.leftJoyCon.calibrateIMU();
        }
        if (this.rightJoyCon) {
            this.rightJoyCon.calibrateStick();
            this.rightJoyCon.calibrateIMU();
        }
    }
    
    getLeftState() {
        return this.leftJoyCon ? this.leftJoyCon.getState() : null;
    }
    
    getRightState() {
        return this.rightJoyCon ? this.rightJoyCon.getState() : null;
    }
    
    isLeftConnected() {
        return this.leftJoyCon && this.leftJoyCon.connected;
    }
    
    isRightConnected() {
        return this.rightJoyCon && this.rightJoyCon.connected;
    }
    
    /**
     * Start sending Joy-Con state to robot via sendCommand callback
     * @param {Function} sendCommand - Function that sends command to robot
     */
    startSending(sendCommand) {
        if (this.sendInterval) {
            clearInterval(this.sendInterval);
        }
        
        this.sendInterval = setInterval(() => {
            const leftState = this.getLeftState();
            const rightState = this.getRightState();
            
            // Only send commands when LT+RT (ZL+ZR) are BOTH pressed.
            if (!this.isDeadmanEngaged()) {
                return;
            }

            if (leftState || rightState) {
                const armCommand = {
                    joycon_arms: {}
                };
                
                if (leftState) {
                    armCommand.joycon_arms.left = {
                        position: leftState.position,
                        orientation: leftState.orientation,
                        gripper: leftState.gripper
                    };
                }
                
                if (rightState) {
                    armCommand.joycon_arms.right = {
                        position: rightState.position,
                        orientation: rightState.orientation,
                        gripper: rightState.gripper
                    };
                }
                
                sendCommand(armCommand);
            }
        }, this.sendRate);
    }
    
    stopSending() {
        if (this.sendInterval) {
            clearInterval(this.sendInterval);
            this.sendInterval = null;
        }
    }
}

// Check WebHID support
function isWebHIDSupported() {
    // WebHID requires a secure context (HTTPS) except on localhost.
    return (window.isSecureContext === true) && ('hid' in navigator);
}

// Export for use in HTML
window.JoyConManager = JoyConManager;
window.isWebHIDSupported = isWebHIDSupported;
