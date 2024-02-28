# Control of SensorArray
#
# Copyright (C) 2023  Johannes Zischg <johannes.zischg@plasmics.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import serial
from . import bus
from serial import SerialException

# from queue import Queue, Empty

# determines the timing for all interactions with SensorArray including reading, writing and connection (attempts)
SERIAL_TIMER = 0.1
BAUD = 115200

class PLA_SensorArray:
    """Custom class for the plasmics SensorArray"""

    def __init__(self, config):
        """The sensor is initialized, this includes especially`
        - the registration for specific events (and how to handle those)
        - the configuration of SensorArray specific G_code commands

        :param config: config file passed down from the heater
        :type config: ?
        """
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.printer.add_object("pla_sear " + self.name, self)
        self.heater = None
        self.serial = None
        self.read_timer = None
        self.temp = 0.0
        self.target_temp = 0.0
        self.read_buffer = ""
        self.read_queue = []
        self.write_timer = None
        self.write_queue = []
        self._failed_connection_attempts = 0
        self._first_connect = True
        #this should be thrown away!
        self.debug_dictionaries = [None]
        self.read_from_board_outs = [None]
        #
        self.last_debug_timestamp = self.reactor.monotonic()
        self.last_debug_message = ""
        self.serial_port = config.get("serial")

        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect
        )
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)

        # add the gcode commands
        if "SEAR_START_CALIBRATION_PROCESS" in self.gcode.ready_gcode_handlers.keys():
            logging.info("J: SEAR_START_CALIBRATION_PROCESS already defined!")
        else:
            self.gcode.register_command(
                "SEAR_START_CALIBRATION_PROCESS",
                self.cmd_SEAR_START_CALIBRATION_PROCESS,
                desc=self.cmd_SEAR_START_CALIBRATION_PROCESS_help,
            )
            logging.info(f"J: All Gcode commands added.")
        self.sample_timer = self.reactor.register_timer(
            self._sample_PLA_SEAR, self.reactor.NOW
        )

    def _handle_connect(self):
        if self.serial is None:
            self._init_PLA_SEAR()

    def _handle_disconnect(self):
        logging.info("J: Klipper reports disconnect: SensorArray shutting down")
        self.disconnect()

    def _handle_shutdown(self):
        logging.info("J: Klipper reports shutdown: SensorArray shutting down")
        self._handle_disconnect()

    def disconnect(self, disconnect_message="d"):
        """Once disconnect is called, the sensor will start shutting down.
        This includes:
        - closing of the serial connection to the SensorArray
        - Unregisters the timers from this sensor
        """
        self.write_queue.append(disconnect_message)
        try:
            self.serial.close()
            logging.info("Serial port closed due to disconnect.")
        except Exception as e:
            logging.error(f"J: Disconnection failed due to: {e}")
        self.serial = None
        try:
            self.reactor.unregister_timer(self.read_timer)
        except:
            logging.info(
                "J: Reactor read timer already unregistered before disconnection."
            )
        self.read_timer = None

        try:
            self.reactor.unregister_timer(self.write_timer)
        except:
            logging.info(
                "J: Reactor write timer already unregistered before disconnection."
            )
        self.write_timer = None

        logging.info("J: SensorArray shut down complete.")

    def get_report_time_delta(self):
        return SERIAL_TIMER

    def get_status(self, _):
        return {
            "last_debug_timestamp": self.last_debug_timestamp,
            "last_debug_message": self.last_debug_message
        }

    ### INO specifics
    def _sample_PLA_SEAR(self, eventtime):
        """This function is called infinitely by the reactor class every SERIAL_TIMER interval.
        Upon execution, it either tries to establish a connection to the SensorArray OR - if connection for
        4 consecutive times was not possible, shut down the printer.

        :param eventtime: _description_
        :type eventtime: _type_
        :return: _description_
        :rtype: _type_
        """
        logging.info(f"J: SAMPLE PLA INO CALLED WITH TIME {eventtime}")
        if self._failed_connection_attempts < 5:
            try:
                if self.serial is None:
                    self._handle_connect()
                else:
                    self.write_queue.append("read")
            except serial.SerialException:
                logging.error("Unable to communicate with SensorArray. Sample")
                self.temp = 0.0
        else:
            logging.info("No connection to SensorArray possible - shutting down Klipper.")
            self.printer.invoke_shutdown(
                "Connection to SensorArray lost and could not be reestablished!"
            )
            return self.reactor.NEVER

        current_time = self.reactor.monotonic()
        return eventtime + SERIAL_TIMER

    def _init_PLA_SEAR(self):
        """Initializes the INO by starting a serial connection to the ino board
        and sending the pid control parameters
        """
        try:
            self.serial = serial.Serial(self.serial_port,BAUD,timeout=1)
            logging.info("Connection to SensorArray successfull.")
            self._failed_connection_attempts = 0
        except Exception as e:
            logging.error(
                f"Unable to connect to SensorArray. This was attempt number {self._failed_connection_attempts + 1}. Exception: {e}"
            )
            self._failed_connection_attempts += 1
            return

        self.write_queue = []
        self.read_queue = []

        logging.info("J: SensorArray queues cleared.")

        self.read_timer = self.reactor.register_timer(self._run_Read, self.reactor.NOW)
        self.write_timer = self.reactor.register_timer(
            self._run_Write, self.reactor.NOW
        )

        logging.info("SensorArray read/write timers started.")

    def _run_Read(self, eventtime):
        """Readout of the incoming messages over the serial port

        :param eventtime: current event time
        :type eventtime: ?
        :return: tell reactor not to call this function any more (if not available)
        :rtype: ?
        """
        # Do non-blocking reads from serial and try to find lines
        while True:
            try:
                raw_bytes = ""
                if self.serial.in_waiting > 0:
                    raw_bytes = self.serial.read()
                else:
                    logging.info("J: nothing to read")
            except Exception as e:
                logging.info(f"J: error in serial readout: {e}")
                self.disconnect()
                break

            if len(raw_bytes):
                text_buffer = self.read_buffer + str(raw_bytes.decode())
                logging.info(text_buffer)
                while True:
                    i = text_buffer.find("\r\n")
                    if i >= 0:
                        line = text_buffer[0 : i + 1]
                        self.read_queue.append(line.strip())
                        text_buffer = text_buffer[i + 1 :]
                    else:
                        break
                self.read_buffer = text_buffer
            
            if len(raw_bytes)>1000:
                logging.info("J: Buffer to large.")
                self.printer.invoke_shutdown(
                "Buffer to large."
                )


            else:
                break

        logging.info(f"J: Read queue contents: {self.read_queue}")
        
        self.last_debug_timestamp = self.reactor.monotonic()
        self._process_read_queue()
        return eventtime + SERIAL_TIMER

    def _run_Write(self, eventtime):
        """Write the messages that are in the queue to the serial connection

        :param eventtime: current event time
        :type eventtime: ?
        :return: tell reactor not to call this function any more (if not available)
        :rtype: ?
        """
        while not len(self.write_queue) == 0:
            0
            logging.info(
                f"J: Current elements in the write queue waiting to be sent to ino: {self.write_queue}"
            )
            text_line = self.write_queue.pop(0)

            if text_line:
                logging.info(f"J: {((text_line).encode())}, {text_line}")
                try:
                    self.serial.write(text_line.encode())

                except Exception as e:
                    logging.info(f"J: error in serial communication (writing): {e}")
                    self.disconnect()
                    break

        # logging.info("J: Write queue is empty.")
        return eventtime + SERIAL_TIMER

    cmd_SEAR_START_CALIBRATION_PROCESS_help = "Command to start the calibration wizard on the SensorArray."

    def cmd_SEAR_START_CALIBRATION_PROCESS(self, gcmd):
        """custom gcode command for starting the sear calibration process

        :param gcmd: gcode command (object) that is processed
        :type gcmd: ?
        """
        logging.info("SensorArray calibration process started.")



    def _process_read_queue(self):
        # Process any decoded lines from the device
        while not len(self.read_queue) == 0:
            text_line = self.read_queue.pop(0)
            tmp = str(text_line.rstrip("\x00"))
            self.gcode.respond_info(f"Output from SensorArray: {str(tmp)}"
            )
            logging.info(
                f"J: Output from INO: {str(tmp)}"
            )


def load_config(config):
    return PLA_SensorArray(config)
