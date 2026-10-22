# Anna's Avionics

This is the code for the avionics stack for the Macalester College High Power Rocketry team. It is currently in in active development. The Arduino version (under the `platformio` directory) is capable of active roll control, and a rewrite to Rust is underway.

For the Rust version, here's how to run it. For each, append `--release` if the code on the device is running in release mode. This includes all the "just attach" versions; the logger needs to match its lookup table to whatever is running on the device.

Build, flash, and attach: `cargo embed`

Reset the device and attach: `cargo embed reset_attach`

Just attach: `cargo embed attach`

New hardware plans:
- Custom PCB containing everything
- Microcontroller: RP2350B
- High-G accelerometer: ADXL375
- Gyroscope and low-G accelerometer: LSM6DSOX
- Magnetometer: Considering either the MMC5983 or LIS3MDL, leaning towards the former
- Barometer: BMP390
- GPS module: Probably ublox MAX-M10S

Hardware (old version):
- Microcontroller: Adafruit Feather RP2040 Adalogger
- High-G accelerometer: ADXL375
- Gyroscope and low-G accelerometer: LSM6DSOX
- Magnetometer: LIS3MDL
- Barometer: BMP390
- GPS module: MTK3333 on an Adafruit Ultimate GPS Featherwing

At the moment, all sensors but the GPS are connected over I²C using Stemma QT, and the GPS module is connected over UART. A custom FeatherWing PCB is in progress to combine all the sensors onto one board, improve interrupt support, add battery monitoring, and much more.

Supports a buzzer with built-in driver circuitry for state notification; PWM buzzers are not supported at the moment.

## Log decoder

Use `decoder.py` to decode a `logNN.bin` file to CSV and optionally re-export a filtered binary by timestamp (`micros`) range.

```bash
python decoder.py --search-dir /path/to/sd_dump --start-micros 1200000 --end-micros 15200000 --output-bin ignition_to_landing.bin --output-csv ignition_to_landing.csv
```

Notes:
- Range bounds are inclusive (`start <= micros <= end`).
- If `--input-bin` is omitted, the script selects the latest `logNN.bin` in `--search-dir`.
- The filtered `.bin` keeps the original file header and packet bytes so it can be decoded again later.



# TODO

Planning regarding sensor read scheduling:

Each sensor should be on its own async loop, preferably with a wait_for_rising_edge on the
DATA_READY interrupt GPIO pin, but a fixed tick rate would be fine. Most of the sensors then
publish their latest state, including a timestamp, to both the SD channel and a Watch,
since we only really need the latest value for the AHRS and control loops. They might also
grab FIFO data if available, though it's not really necessary, and send it all in one packet
to the SD card. (Since we're going at the sensor's ODR, this is only helpful to catch the
extra packet when a new packet is produced while the previous one is still being read.)
However, the LSM6DSOX is the main sensor that the AHRS relies on, so its loop should have
the AHRS integrated into it, which then publishes on the AHRS_STATE Watch.
Then, the state machine loop and the control loop, which are now fully separate from the
AHRS loop, can read the state and update the control surfaces. If the LSM6DSOX is out of
its bounds, it can grab the most recent value from the ADXL375.

Each loop should also check for staleness and mark a failure if it's been too long since
the last reading. This gets a bit complicated with the LSM6DSOX, though, since the AHRS
also depends on it; if it goes stale, maybe it can fall back to a simple tick loop or even
somehow follow the backup gyro's timing, though I'm not sure how that might be possible. And
for these staleness checkers, how would that work? Does it perhaps need a watchdog task
that checks the latest timestamps of each sensor, or does Embassy/async somehow support
cancelling a promise after enough time and running a function instead? And how can a
signal be sent to the sensor to cancel the promise if the watchdog does detect staleness?
And then how to recover from it; does it keep a promise running and cancel the new ticker
if the first one gets a new value? Or as another possible solution: what if two different
async loops are both trying to run the AHRS, but one of them (backup gyro) skips updating
if the most recent timestamp is too recent? That feels ripe for race conditions, though,
so combining promises/futures is probably better.

Also, the backup path doesn't actually matter as much right now, since there is no backup
gyro, so it's most important that it just marks the AHRS as invalid if the sensor goes
stale. Handling the possibility that the interrupts are broken but the sensor still works
is probably also a good idea, and that makes it way easier to add a backup in the future.

Here's something that an LLM suggested:

> ```rust
> match select(
>     int1.wait_for_high(),
>     Timer::after(MAX_DRDY_GAP),
> ).await {
>     Either::First(Ok(())) => {
>         // Read/drain FIFO, publish AHRS, mark healthy.
>     }
>     Either::Second(()) => {
>         // Mark interrupt/sample path stale; probe sensor status/recover.
>     }
>     Either::First(Err(_)) => {
>         // Treat as an input-path failure.
>     }
> }
> ```
> 
> When `select` completes, Rust drops the losing future. That cancels the outstanding edge wait; no separate cancel signal or watchdog task is needed. The next loop iteration creates a new wait.
> 
> Recovery should be a single task changing its own mode, not two competing loops:
> 
> - On timeout, read the LSM status/FIFO once.
> - If it responds and has new data, the sensor works but the data-ready path is suspect: switch that same task to a timer-driven polling mode and raise an interrupt-path fault.
> - If it does not respond or has no fresh data for the longer sensor deadline: mark the LSM failed.
> - After several successful samples, clear/debounce the fault and optionally retry interrupt mode.

And I think this makes a lot of sense; I'll just have to add some persistent-ish flag to the
loop to tell it to switch to polling mode if it detects a timeout, then it should check somehow
(FIFO?) if the sensor actually does have new data on each loop, and if it does for several samples,
try switching back to interrupt mode. And then find a way to detect if interrupt mode is consistently
failing, and if so, just stick with polling mode. If there's no new data in the sensor, mark
it as failed and use another sensor.


TODO: Figure out how to structure this. If the LSM is driving the AHRS, the code needs to be restructured
to make it really clear that the LSM loop is not simple reads like the others. Perhaps putting it in
some separate file (AHRS.rs?) and have it await LSM functions instead of having it in the LSM
file?

And how should launch detection work? If the sensor reads are decoupled from the state machine,
what part checks the magnitudes, and how is that communicated to the state machine and the AHRS?
I guess the AHRS should just read the current state from the state machine, but then I'm not totally
sure where the actual detection should go. That feels wrong to have in the sensor itself because
I think the sensor part should be focused on just reading the sensor and publishing its state,
so then I guess it should consistently publish state and let the state machine detect launch
when it runs next? The downside is that I won't then be able to use the ADXL hardware interrupt,
and averaging over a time window will be more difficult (plus a bit of latency).