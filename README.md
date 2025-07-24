# ZMK Pointing Acceleration

This repository contains a velocity-based pointer acceleration implementation for pointing devices in ZMK.

> [!WARNING]
> This module is currently in beta. While it has been tested with Cirque trackpads, compatibility with other pointing devices may vary. Please report any issues you encounter.

The acceleration makes fine cursor control more precise at slow speeds while allowing faster cursor movement when moving quickly. It supports customizable acceleration curves and can be configured for different input devices.

**Device Compatibility:** This module has been primarily tested with Cirque trackpads. It should work with other relative input devices (trackballs, trackpoints, other trackpads), but these configurations are less tested.

**Prerequisites:** Before using this module, ensure you have a working input device by following the [ZMK pointing documentation](https://zmk.dev/docs/features/pointing).

## Features

- Configurable minimum and maximum acceleration factors
- Adjustable speed thresholds for acceleration onset
- Customizable acceleration curve (linear, quadratic, etc.)
- Support for tracking fractional movement remainders
- Compatible with any relative input device (mouse, trackball, touchpad)

## Installation & Usage

To use pointer acceleration, follow these steps:

1. Add the acceleration module to your `west.yml`
2. Import the dependencies into your configuration files
3. Configure the acceleration parameters
4. Add the acceleration processor to your input chain

### Step 1: Update west.yml

Add the acceleration module to your `west.yml`:

```yaml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: oleksandrmaslov
      url-base: https://github.com/oleksandrmaslov
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: main
      import: app/west.yml
    - name: zmk-pointing-acceleration
      remote: oleksandrmaslov
      revision: main
  self:
    path: config
```

> [!NOTE]
> Run `west update` after modifying west.yml if you're building locally.

### Step 2: Import Dependencies

Add the necessary includes to your device overlay file (e.g., `yourkeyboard_left.overlay`):

```devicetree
#include <behaviors/input_gestures_accel.dtsi>
```

### Step 3: Configure Acceleration

Add the acceleration configuration to your device overlay. This configuration should go **before** your input-listener.

**Option A: Override the included definition (Recommended)**

```devicetree
&pointer_accel {
    input-type = <INPUT_EV_REL>;       // For relative input devices
    codes = <INPUT_REL_X INPUT_REL_Y>; // X and Y axis events
    track-remainders;                  // Accumulate fractional movements
    min-factor = <800>;                // 0.8x at very slow speeds
    max-factor = <3000>;               // 3.0x at fast speeds
    speed-threshold = <1200>;          // 1200 counts/sec for 1x
    speed-max = <6000>;                // 6000 counts/sec for max accel
    acceleration-exponent = <2>;       // Quadratic acceleration curve
};
```

**Option B: Define your own instance**

```devicetree
/ {
    my_pointer_accel: my_pointer_accel {
        compatible = "zmk,input-processor-acceleration";
        status = "okay";
        #input-processor-cells = <0>;
        input-type = <INPUT_EV_REL>;
        codes = <INPUT_REL_X INPUT_REL_Y>;
        track-remainders;
        min-factor = <800>;
        max-factor = <3000>;
        speed-threshold = <1200>;
        speed-max = <6000>;
        acceleration-exponent = <2>;
    };
};
```

### Step 4: Add to Input Chain

Add the acceleration processor to your input device's processor chain:

```devicetree
/ {
    tpad0: tpad0 {
        compatible = "zmk,input-listener";
        status = "okay";
        device = <&glidepoint>;
        input-processors = <
            &pointer_accel       // Use the included definition
            // OR &my_pointer_accel if you defined your own
            &zip_xy_transform    // Other processors as needed
        >;
    };
};
```

## Configuration Options

> [!TIP] > **Visual Configuration Tool:** Use the interactive configuration visualizer at https://pointing.streamlit.app/ to experiment with different settings.

The acceleration processor provides several settings to customize how your pointing device behaves:

### Basic Settings

**`min-factor`** (Default: 1000)

- Controls how slow movements are handled
- Values below 1000 make slow movements slower for precision
- Values are in thousandths (e.g., 800 = 0.8x speed)
- Example: `min-factor = <800>;` makes slow movements 20% slower

**`max-factor`** (Default: 3500)

- Controls maximum acceleration at high speeds
- Values are in thousandths (e.g., 3500 = 3.5x speed)
- Example: `max-factor = <3000>;` means fast movements are up to 3x faster

### Speed Settings

**`speed-threshold`** (Default: 1000)

- Speed at which acceleration starts
- Measured in counts per second
- Below this speed, min-factor is applied
- Above this speed, acceleration begins
- Example: `speed-threshold = <1200>;` means acceleration starts at moderate speeds

**`speed-max`** (Default: 6000)

- Speed at which maximum acceleration is reached
- Measured in counts per second
- At this speed and above, max-factor is applied
- Example: `speed-max = <6000>;` means you reach max acceleration at high speeds

### Acceleration Behavior

**`acceleration-exponent`** (Default: 1)

- Controls how quickly acceleration increases
- `1` = Linear (smooth, gradual acceleration)
- `2` = Quadratic (acceleration increases more rapidly)
- `3` = Cubic (very rapid acceleration increase)
- Example: `acceleration-exponent = <2>;` for a more aggressive acceleration curve

### Advanced Options

**`track-remainders`** (Default: disabled)

- Enables tracking of fractional movements
- Improves precision by accumulating small movements
- Enable with `track-remainders;` in your config

**`input-type`** (Default: INPUT_EV_REL)

- Specifies the input event type to process
- Use `INPUT_EV_REL` for relative input devices (trackpads, mice)

**`codes`** (Required)

- Array of event codes to accelerate
- Use `<INPUT_REL_X INPUT_REL_Y>` for X and Y axis movement
- Can include additional codes like scroll wheel events if needed

### Device Tree Properties

**`compatible`** (Required)

- Must be `"zmk,input-processor-acceleration"`
- Identifies this as a ZMK input processor acceleration instance

**`status`** (Default: "okay")

- Device status, use `"okay"` to enable
- Set to `"disabled"` to temporarily disable without removing configuration

**`#input-processor-cells`** (Required)

- Must be `<0>` for this processor type
- Defines the number of cells in input processor references

### Visual Examples

Here's how different configurations affect pointer movement:

```
Slow Speed │  Medium Speed  │  High Speed
───────────┼────────────────┼────────────
0.8x      →│      1.0x     →│     3.0x     (Balanced)
0.9x      →│      1.0x     →│     2.0x     (Light)
0.7x      →│      1.0x     →│     4.0x     (Heavy)
0.5x      →│      1.0x     →│     1.5x     (Precision)
```

## Example Configurations

> [!NOTE] > **Interactive Configuration Tool:** Visit https://pointing.streamlit.app/ for easy configuration visualization.

The following configurations are starting points - every person's perfect pointer settings are unique. Feel free to experiment and find what works best for you.

### Why Share Your Settings?

- Help others find their ideal setup
- Contribute to community knowledge
- Get feedback and suggestions
- Inspire new configuration ideas

### How to Share

- Create a GitHub issue with your configuration
- Share on Discord ZMK community
- Comment on what worked/didn't work for your use case

> **Note:** These examples were primarily tested with Cirque trackpads. Results may vary with other pointing devices.

### General Use (Balanced)

```devicetree
&pointer_accel {
    input-type = <INPUT_EV_REL>;
    codes = <INPUT_REL_X INPUT_REL_Y>; // X and Y axis events
    min-factor = <800>;           // Slight slowdown for precision
    max-factor = <3000>;          // Good acceleration for large movements
    speed-threshold = <1200>;     // Balanced acceleration point
    speed-max = <6000>;           // Max acceleration at high speeds
    acceleration-exponent = <2>;  // Smooth quadratic curve
    track-remainders;             // Track fractional movements
};
```

### Light Acceleration (Conservative)

```devicetree
&pointer_accel {
    input-type = <INPUT_EV_REL>;
    codes = <INPUT_REL_X INPUT_REL_Y>; // X and Y axis events
    min-factor = <900>;           // 0.9x minimum
    max-factor = <2000>;          // 2.0x maximum
    speed-threshold = <1500>;     // Start accelerating later
    speed-max = <5000>;           // Conservative max speed
    acceleration-exponent = <1>;  // Linear acceleration
    track-remainders;             // Track fractional movements
};
```

### Heavy Acceleration (Aggressive)

```devicetree
&pointer_accel {
    input-type = <INPUT_EV_REL>;
    codes = <INPUT_REL_X INPUT_REL_Y>; // X and Y axis events
    min-factor = <700>;           // 0.7x minimum
    max-factor = <4000>;          // 4.0x maximum
    speed-threshold = <1000>;     // Start accelerating earlier
    speed-max = <6000>;           // High max speed
    acceleration-exponent = <3>;  // Cubic acceleration curve
    track-remainders;             // Track fractional movements
};
```

### Precision Mode (Fine Control)

```devicetree
&pointer_accel {
    input-type = <INPUT_EV_REL>;
    codes = <INPUT_REL_X INPUT_REL_Y>; // X and Y axis events
    min-factor = <500>;           // 0.5x for fine control
    max-factor = <1500>;          // 1.5x maximum
    speed-threshold = <2000>;     // High threshold for stability
    speed-max = <7000>;           // High max speed threshold
    acceleration-exponent = <1>;  // Linear response
    track-remainders;             // Track fractional movements
};
```

## Troubleshooting

### Build Issues

If you encounter build errors:

1. Ensure `west update` has been run after modifying `west.yml`
2. Check that the include path is correct: `#include <behaviors/input_gestures_accel.dtsi>`
3. Verify your ZMK version supports input processors
4. Make sure the acceleration configuration comes before your input-listener

### Runtime Issues

If acceleration doesn't work as expected:

1. Verify your input device is working without acceleration first
2. Check that the processor is in the correct order in your input chain
3. Try simpler settings first (linear acceleration with exponent = 1)
4. Enable `track-remainders` for better precision with small movements

## Contributing

Contributions are welcome! Please:

1. Test your changes with different input devices if possible
2. Update documentation for any new features
3. Follow the existing code style
4. Submit pull requests with clear descriptions

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
