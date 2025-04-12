# ZMK POINTING ACCELERATION

This repository contains a pointer acceleration implementation for pointing devices in ZMK.

The acceleration makes fine cursor control more precise at slow speeds while allowing faster cursor movement when moving quickly. It supports customizable acceleration curves and can be configured for different input devices.

## Features

- Configurable minimum and maximum acceleration factors
- Adjustable speed thresholds for acceleration onset
- Customizable acceleration curve (linear, quadratic, etc.)
- Support for tracking fractional movement remainders
- Compatible with any relative input device (mouse, trackball, touchpad)

## Installation & Usage

To use pointer acceleration, there are several steps necessary:
- adjust the `west.yml` to make the acceleration module available
- import the dependencies into your configuration files
- configure the acceleration parameters
- add the acceleration processor to your input chain

We'll go through these steps one by one.

### Adjust west.yml

Add the acceleration module to your `west.yml`:

```yaml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: oleksandrmaslov
      url-base: https://github.com/oleksandrmaslov      
  projects:
    - name: zmk-pointing-acceleration
      remote: oleksandrmaslov
      revision: main
    - name: zmk
      remote: zmkfirmware
      revision: main
      import: app/west.yml
```

> [!WARNING]  
> Run `west update` if you're building on your local machine (not github actions).

### Import the dependencies

Add the necessary includes to your device overlay file (e.g. `yourkeyboard_left.overlay`):

```C
#include <input/processors.dtsi>
#include <behaviors/input_gestures_accel.dtsi>
```

### Configure Acceleration

Add the acceleration configuration to your device overlay. This example provides a balanced acceleration curve:

```devicetree
&pointer_accel {
    input-type = <INPUT_EV_REL>;  // For relative input devices
    track-remainders;             // Accumulate fractional movements
    min-factor = <800>;          // 0.8x at very slow speeds
    max-factor = <3000>;         // 3.0x at fast speeds
    speed-threshold = <1200>;     // 1200 counts/sec for 1x
    speed-max = <6000>;          // 6000 counts/sec for max accel
    acceleration-exponent = <2>;  // Quadratic acceleration curve
};
```

### Add to Input Chain

Add the acceleration processor to your input device's processor chain:

```devicetree
/ {
    tpad0: tpad0 {
        compatible = "zmk,input-listener";
        status = "okay";
        device = <&glidepoint>;
        input-processors = <
            &pointer_accel      // Acceleration processor
            &zip_xy_transform
        >;
    };
};
```

## Configuration Options

**Visualisation of these settings here:**

The acceleration processor provides several settings to customize how your pointing device behaves. Here's a detailed explanation of each option:

### Basic Settings

- `min-factor`: (Default: 1000)
  - Controls how slow movements are handled
  - Values below 1000 will make slow movements even slower for precision
  - Values are in thousandths (e.g., 800 = 0.8x speed)
  - Example: `min-factor = <800>` makes slow movements 20% slower

- `max-factor`: (Default: 3500)
  - Controls maximum acceleration at high speeds
  - Values are in thousandths (e.g., 3500 = 3.5x speed)
  - Example: `max-factor = <3000>` means fast movements are up to 3x faster

### Speed Settings

- `speed-threshold`: (Default: 1000)
  - Speed at which acceleration starts
  - Measured in counts per second
  - Below this speed, min-factor is applied
  - Above this speed, acceleration begins
  - Example: `speed-threshold = <1200>` means acceleration starts at moderate speeds

- `speed-max`: (Default: 6000)
  - Speed at which maximum acceleration is reached
  - Measured in counts per second
  - At this speed and above, max-factor is applied
  - Example: `speed-max = <6000>` means you reach max acceleration at high speeds

### Acceleration Behavior

- `acceleration-exponent`: (Default: 1)
  - Controls how quickly acceleration increases
  - 1 = Linear (smooth, gradual acceleration)
  - 2 = Quadratic (acceleration increases more rapidly)
  - 3 = Cubic (very rapid acceleration increase)
  - Example: `acceleration-exponent = <2>` for a more aggressive acceleration curve

### Advanced Options

- `track-remainders`: (Default: disabled)
  - Enables tracking of fractional movements
  - Improves precision by accumulating small movements
  - Enable with `track-remainders;` in your config


### Visual Examples

Here's how different configurations affect pointer movement:

```
Slow Speed │   Medium Speed   │   High Speed
───────────┼─────────────────┼────────────
0.8x      →│      1.0x     →│     3.0x     (Balanced)
0.9x      →│      1.0x     →│     2.0x     (Light)
0.7x      →│      1.0x     →│     4.0x     (Heavy)
0.5x      →│      1.0x     →│     1.5x     (Precision)
```

# Behavior Examples

## Example Configurations

### General Use:
```devicetree
&pointer_accel {
    min-factor = <800>;        // Slight slowdown for precision
    max-factor = <3000>;       // Good acceleration for large movements
    speed-threshold = <1200>;  // Balanced acceleration point
    acceleration-exponent = <2>; // Smooth quadratic curve
    track-remainders;         // Track fractional movements
};
```
### Light Acceleration
```devicetree
&pointer_accel {
    min-factor = <900>;        // 0.9x minimum
    max-factor = <2000>;       // 2.0x maximum
    speed-threshold = <1500>;  // Start accelerating later
    acceleration-exponent = <1>; // Linear acceleration
    track-remainders;          // Track fractional movements
};
```

### Heavy Acceleration
```devicetree
&pointer_accel {
    min-factor = <700>;        // 0.7x minimum
    max-factor = <4000>;       // 4.0x maximum
    speed-threshold = <1000>;  // Start accelerating earlier
    acceleration-exponent = <3>; // Cubic acceleration curve
    track-remainders;          // Track fractional movements
};
```

### Precision Mode
```devicetree
&pointer_accel {
    min-factor = <500>;        // 0.5x for fine control
    max-factor = <1500>;       // 1.5x maximum
    speed-threshold = <2000>;  // High threshold for stability
    acceleration-exponent = <1>; // Linear response
    track-remainders;          // Track fractional movements
};
```
