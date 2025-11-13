# GPS Parameter Switcher

This module automatically switches between two sets of parameters based on which GPS is currently active. When GPS0 (GPS1) is selected, parameters with `_GPS0` suffix are applied. When GPS1 (GPS2) is selected, parameters with `_GPS1` suffix are applied.

## Usage

1. **Start the module:**
   ```
   gps_parameter_switcher start
   ```

2. **Set GPS-specific parameters:**

   For each parameter you want to switch, create two versions with `_GPS0` and `_GPS1` suffixes:

   ```
   param set EKF2_GPS_P_GATE_GPS0 5.0
   param set EKF2_GPS_P_GATE_GPS1 10.0

   param set FW_THR_SLEW_MAX_GPS0 0.5
   param set FW_THR_SLEW_MAX_GPS1 1.0

   # ... etc for all parameters you want to switch
   ```

3. **Enable/Disable the switcher:**
   ```
   param set GPS_PARAM_SWITCH_EN 1  # Enable (default)
   param set GPS_PARAM_SWITCH_EN 0  # Disable
   ```

## Supported Parameters

The following parameters can be switched automatically:

- `EKF2_GPS_P_GATE`
- `EKF2_REQ_EPH`
- `EKF2_REQ_PDOP`
- `EKF2_REQ_VDRIFT`
- `FW_L1_R_SLEW_MAX`
- `FW_RR_P`
- `FW_THR_SLEW_MAX`
- `FW_T_CLMB_MAX`
- `MC_PITCHRATE_D`
- `MC_PITCHRATE_I`
- `MC_PITCHRATE_P`
- `MC_ROLLRATE_D`
- `MPC_ACC_HOR_MAX`
- `MPC_ACC_UP_MAX`
- `MPC_MAN_TILT_MAX`
- `MPC_TILTMAX_AIR`
- `MPC_VEL_MANUAL`
- `MPC_XY_VEL_MAX`
- `MPC_Z_VEL_MAX_DN`
- `MPC_Z_VEL_MAX_UP`
- `VT_FW_QC_P`

## How It Works

1. The module monitors the `vehicle_gps_position` topic for GPS selection changes.
2. When GPS selection changes between GPS0 (selected=0) and GPS1 (selected=1), the module:
   - Reads the GPS-specific parameter (e.g., `EKF2_GPS_P_GATE_GPS0`)
   - Applies it to the base parameter (e.g., `EKF2_GPS_P_GATE`)
   - Triggers a parameter update notification so other modules pick up the change
3. Parameters are applied immediately when GPS switches (mid-flight is supported).

## Notes

- Parameters with `_GPS0`/`_GPS1` suffixes must be created manually using `param set`
- If a GPS-specific parameter doesn't exist, that parameter is skipped (no error)
- The module runs at 5 Hz and checks for GPS selection changes
- Parameter switching happens automatically - no manual intervention needed
- All parameter changes take effect immediately (no reboot required)

## Example Setup Script

```bash
#!/bin/bash

# GPS0 parameters (for GPS1)
param set EKF2_GPS_P_GATE_GPS0 5.0
param set EKF2_REQ_EPH_GPS0 3.0
param set FW_THR_SLEW_MAX_GPS0 0.5
param set MC_PITCHRATE_P_GPS0 0.15

# GPS1 parameters (for GPS2)
param set EKF2_GPS_P_GATE_GPS1 10.0
param set EKF2_REQ_EPH_GPS1 5.0
param set FW_THR_SLEW_MAX_GPS1 1.0
param set MC_PITCHRATE_P_GPS1 0.20

# Start the switcher
gps_parameter_switcher start
```

