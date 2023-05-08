# Wiggle Waggle LED Art

## overview
The wiggle waggle is a common dog toy, also fun for humans of all ages.  Lets make one thats 4 feet tall and has some cool electronics.

## Hardware
- 5 meters of ws2815 leds (12v but will operate down to 9.5 which is nice b/c no power injection)
- 18 AH deep cycle SLA and a fuse
- 12v to 5v DC to DC step down for powering MCU
- teensy or adafruit qt py esp32 pico, or both... this will be based on wheather or not the pico can keep up
- appropriate lvl shifter octo or neopixle BFF
- 4 haptic motors and FET drivers
- would be nice to have at least voltage divder for monitoring battery voltage
- decided 22 awg wire is "good enough"
- one I2S omnidirectional microphone
- one power switch
- easy access charging port
- one 4 foot ball with noise makers

## Software structure

### haptic effects
- pulse all motors on collision
- randomly pulse different motors

### Bluetooth / other Controls
- low power force
- lock out timer
- bed time timer (this thing should just default to turning off after 12 hours of on time)
- current time (for bedtime / lockout)
- low voltage cuttoff

### LED effects
This contians a way to access all of the LED effects I have come up with for this project, either written by me or taken from open source examples.


- plane distance: use one of the animated 3d planes to set hue, intencity, etc
- polkadot: instatiate a couple animated shperes / points around the surface of the wiggle waggle to set pixles
- bacterial: instantiate a couple lines / spheres , make it look like germs spreading
- wave: sinusoidal lookup table
- plane gradiant: make nice color gradients based on plane distance
- spotlight. make a line spin from the bottom, etc
-
- favs from photon collider
    - lightning: flash one strip at a time, but maybey quicly, would be a good collision effect
    - tron: see photon collider
    - 

### LED effect modifiers
This is ways that the wiggle waggle will respond to its enviroment
- Feedback and Mixing, utilizes last fram information to construct or modify the next frame
    - mod hue / sat / intesity (seperatly)
    -
    - spin: move all leds values down the line, last one in line goes to first
    - spread: spread pixels apart exponentially from a certain strip index
    - glitch: spin things away randomly, to be brough back in later frames
    -
    - decay: subtract n from intensity, constrained to 0, 255
    - decay rollover: dont constrain roll it over
    - brighten: add n to intensity, constrained to 0, 255
    - hue shift: add n to hue, rollover at 255
    - sat shift:
    - 
- Sound Modifiers
    - BPM pulse
    - intensity matching
    - frequency to hue? might be to cpu intensive to run FFT with my other stuff well see...
    - intinsity to effect period
    constrained to 0, 255
- Motion Modifiers
    - counter rotate: make the effect look "stationary"
    - multiply rotate: either double or multiply ball rotation in the effect to make it look like its spinning faster
    - orthogonal rotate: make it look like a bowling ball spinning sideways
    - rotational velocity intesity matching
    - collision detection: make some fun firworks during large angular acceleration(someone pushes the ball or it hits something)
    - pimp wheel: use lpf etc to "continue" motion even after collisions
    - roraty velocity to effect period.  the faster the spin the faster the effect behaves.
    - low power mode: if no motion is detected for a while just spin a pixel around
- Failsafe modifiers
    - over current: reduce intensity / number of active pixels if too much is happening
    - all black: sometimes, especially with uncontrolled decay, the whole thing may go dark. prevent this


### Dependencies
- https://github.com/cpreinholtz/EffectUtils3d: this repo contains some 3d geometry for pixel mapping.  heavily inspired by aerokeith's blog and open source 2d geometry library.
- https://github.com/Aerokeith/ColorUtilsHsi.git: speaking of aeorokeith, I desided to use his HSI utilities.



## hardware

## Directory structure
- VS: (THIS DIRECTORY IS IGNORED in the gitignore, just used for setting up Visual Studio projects for testing and debugging)
- Test: (constains sources specifically for testing)
- src: .h and .cpp (for arduino libs sources must be in root folder or root/src)
- Examples: ExScetch (contains ExScetch.ino for arduino IDE usage)