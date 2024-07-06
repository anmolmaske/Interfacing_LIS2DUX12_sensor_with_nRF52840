# Interfacing LIS2DUX12 with nRF52840

### Overview

This sample application periodically reads accelerometer data from the
LIS2DUX12 sensor, and displays the sensor data on the console.

### Building and Running

`ref: nrf58240dk_nrf52840` includes an ST LIS2DUX12 accelerometer which
supports the LIS2DUX12 interface.

.. building-commands:   
   :board: nrf58240dk_nrf52840   
   :overlay: nrf_board.overlay   
   :goals: build flash   
   :compact:

Sample Output
=============

    00> [00:00:01.364,562] <inf> LIS2DUX: Acceleration measured are as follows:
    00> [00:00:01.365,631] <inf> LIS2DUX: X: -2.61,   Y: -1.47,   Z: -9.14
    00> [00:00:02.366,790] <inf> LIS2DUX: X: -2.64,   Y: -1.56,   Z: -9.50
    00> [00:00:03.368,011] <inf> LIS2DUX: X: -2.74,   Y: -1.12,   Z: -9.37
    00> [00:00:04.369,232] <inf> LIS2DUX: X: -2.80,   Y: -0.99,   Z: -9.41
    00> [00:00:05.370,452] <inf> LIS2DUX: X: -2.87,   Y: -0.94,   Z: -9.45
