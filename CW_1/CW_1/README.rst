Week 2 – Temperature Sensor (ADC) Lab
====================================

This repository provides the **student skeleton project** for
**Week 2 – Temperature Sensor (ADC)** of the *EEMEM0018 IoT System Prototyping* course.

Students will work with the ADC peripheral on the nRF54L15 DK to read an analog
temperature sensor and convert the measured voltage into temperature.

----

Target Platform
---------------

- **Board:** nRF54L15 DK
- **RTOS:** Zephyr (nRF Connect SDK)
- **Sensor:** LM335 analog temperature sensor
- **Interface:** ADC (SAADC)

----

Learning Objectives
-------------------

By completing this lab, students will learn how to:

- Read analog sensor values using the ADC peripheral
- Convert raw ADC readings to millivolts
- Convert sensor voltage to temperature (°C)
- Use devicetree-derived ADC configuration (gain, reference, input channel)
- Compare **polling** and **timer-based event-driven** sampling approaches

.. note::

   Bluetooth functionality is present in the project but is **not modified**
   in Week 2. BLE will be extended in Week 3.

----

What You Need to Do
-------------------

This repository contains **incomplete code**.

Students are required to complete the following tasks in ``src/main.c`` (marked with ``TODO``):

- **Task 1:** Convert ADC readings (raw -> mV -> °C) and print results via UART
- **Task 2:** Implement a temperature threshold indicator (Temp > 30°C -> LED ON)
- **Task 3:** Compare sampling styles:
  - **Polling:** periodic sampling in the main loop
  - **Event-driven:** timer-based sampling using ``k_timer`` + ``k_work``

----

Repository Structure
--------------------

::

    .
    ├── src/main.c          # Main application (students modify TODO sections)
    ├── prj.conf            # Zephyr configuration
    ├── CMakeLists.txt
    ├── boards/             # Board-specific overlay (ADC configuration)
    └── README.rst

----

Build and Flash
---------------

::

    west build -b nrf54l15dk_nrf54l15_cpuapp
    west flash

Open a serial terminal at **115200 baud** to observe output.

----

Important Notes
---------------

- Verify the analog supply voltage (VDDA) is configured appropriately on the DK.
- If VDDA is set too low (e.g., 1.8 V), ADC readings may saturate and temperature
  changes may not be observed reliably.
- This repository is intended for **teaching purposes** and does not contain
  a complete reference solution.
