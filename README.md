# HiL-Framework
A simple hardware in the loop test framework for practice with docker, pytest, and ci/cd


---

## Disclaimer

This isn't meant to be a serious framework, this is just me getting in some practice with each level of a ci/cd pipeline on a very small scale

---

## Backround

One thing i want to be doing more of, long term, is hardware system testing. A key part of that is HiL, hardware in the loop.

I've touched on different aspects of such a system, and even fully planned out one as NASA, but haven't yet built one end to end from scratch


---

## System Diagram

A general overview of the system is below
<p align="left">
  <img src="Pics/HiL framework project.drawio.png?raw=true">
</p>

---

## Hardware

- DUT (device under test)
-   Arduino Nano (for now, esp32 coming soon)
-   MCP4725 digital analog converter (DAC)
-   ADS1115 analog digital converter (ADC)
-   MCP2515
- Test runner
-   Raspberry pi 4
-   Raspberry pi CAN shield by seeed studio
-   DAC
-   ADC

---

## Software

- Docker, to creat a combined test runner and test repo image
- Pytest to do the actual assertions
- Arduino CLI to monitor the arduino serial interface, and push built code
- Gitlab for Ci/CD
- 
