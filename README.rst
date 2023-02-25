.. _sht41_peripheral:

Bluetooth: SHT41 Peripheral
#####################

Overview
********

SHT41 bluetooth server that reads data from a SHT41 temperature and humidity sensor and notifies the client with the sensor values every 15 minutes.


Requirements
************

nrf52 board.
SHT41 sensor

Building and Running
********************

Build tested on ncs 2.2.0

The application will advertise the main service once started. Sensor reading and notifications will begin once a connection has been established
