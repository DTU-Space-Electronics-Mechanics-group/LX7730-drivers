# LX7730 driver

[![License](https://img.shields.io/badge/license-BSD3-red.svg)](LICENSE)

This repository contains a low-level driver for the Microchip LX7730 Rad-hard telemetry controller chip, written in C.

## Usage

In order to use the driver, include it in your C project. In order for the library to work the *if_write* and *if_read* functions must be declared externally. The function prototypes are documented in LX7730.h.