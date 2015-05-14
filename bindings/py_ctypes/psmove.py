from ctypes import *
import os.path
from ctypes.util import find_library
import platform
import struct
global libctrl, libtrck

"""@package docstring
Documentation  for this module.

Python API for the PSMove.

The psmove.py provides an interface for the PS Move Motion Controller.

This module requires that an appropriately-named library binary is present 
on the path.
"""

# ==========================================================
# ======= Free Functions provided by the PSMove API  =======
# ==========================================================
class PSMove:

	def __init__(self):
		self.version = 0x030001 # this is the current PSMove Version

	def init(self):
	    """ Initialize the library and check for the right version.

	    	This library call should be used at the beginning of each application using
			the PS Move API to check if the correct version of the library is loaded (for
			dynamically-loaded versions of the PS Move API).

			\param version Should be \ref PSMOVE_CURRENT_VERSION to check for the same
			               version as was used at compile-time

			\return \ref PSMove_True on success (version compatible, library initialized)
			\return \ref PSMove_False otherwise (version mismatch, initialization error)
	    """
	    return libctrl.psmove_init(self.version)

	def set_remote_config(self):
		""" Enable or disable the useage of local or remote devices. 

			By default, both local (hidapi) and remote (moved) controllers will be
			used. If your application only wants to use locally-connected devices,
			and ignore any remote controllers, call this function with
			\ref PSMove_OnlyLocal - to use only remotely-connected devices, use
			\ref PSMove_OnlyRemote instead.
			 
			This function must be called before any other PS Move API functions are
			used, as it changes the behavior of counting and connecting to devices.
			 
			\param config \ref PSMove_LocalAndRemote, \ref PSMove_OnlyLocal or
			              \ref PSMove_OnlyRemote
			 
		"""
		return libctrl.psmove_set_remote_config(config)

	def count_connected(self):
		""" \brief Get the number of available controllers

			\return Number of controllers available (USB + Bluetooth + Remote)
		"""
		return libctrl.psmove_count_connected()

	def connect(self):
		""" \brief Connect to the default PS Move controller
		
		This is a convenience function, having the same effect as:
		
		\code psmove_connect_by_id(0) \endcode
		
		\return A new \ref PSMove handle, or \c NULL on error

		"""
		return libctrl.psmove_connect()

	def connect_by_id(self, id):
		""" \brief Connect to a specific PS Move controller

			This will connect to a controller based on its index. The controllers
			available are usually:

			1. The locally-connected controllers (USB, Bluetooth)
			2. The remotely-connected controllers (exported via \c moved)

			The order of controllers can be different on each application start,
			so use psmove_get_serial() to identify the controllers if more than
			one is connected, and you need some fixed ordering. The global ordering
			(first local controllers, then remote controllers) is fixed, and will
			be guaranteed by the library.

			\param id Zero-based index of the controller to connect to
			          (0 .. psmove_count_connected() - 1)

			\return A new \ref PSMove handle, or \c NULL on error
		"""
		return libctrl.psmove_connect_by_id(id)

	def connection_type(self):
		""" \brief Get the connection type of a PS Move controller

			For now, controllers connected via USB can't use psmove_poll() and
			all related features (sensor and button reading, etc..). Because of
			this, you might want to check if the controllers are connected via
			Bluetooth before using psmove_poll() and friends.

			\param move A valid \ref PSMove handle

			\return \ref Conn_Bluetooth if the controller is connected via Bluetooth
			\return \ref Conn_USB if the controller is connected via USB
			\return \ref Conn_Unknown on error
		"""
		return libctrl.psmove_connection_type(self.move)

	def is_remote(self):
		""" \brief Check if the controller is remote (\c moved) or local.

			This can be used to determine to which machine the controller is
			connected to, and can be helpful in debugging, or if you need to
			handle remote controllers differently from local controllers.

			\param move A valid \ref PSMove handle

			\return \ref PSMove_False if the controller is connected locally
			\return \ref PSMove_True if the controller is connected remotely
		"""
		return libctrl.psmove_is_remote(self.move)

	def get_serial(self):
		""" \brief Get the serial number (Bluetooth MAC address) of a controller.

			The serial number is usually the Bluetooth MAC address of a
			PS Move controller. This can be used to identify different
			controllers when multiple controllers are connected, and is
			especially helpful if your application needs to identify
			controllers or guarantee a special ordering.

			The resulting value has the format:

			\code "aa:bb:cc:dd:ee:ff" \endcode

			\param move A valid \ref PSMove handle

			\return The serial number of the controller. The caller must
			        free() the result when it is not needed anymore.
		"""
		return libctrl.psmove_get_serial(self.move)

	def pair(self):
		""" \brief Pair a controller connected via USB with the computer.

			This function assumes that psmove_connection_type() returns
			\ref Conn_USB for the given controller. This will set the
			target Bluetooth host address of the controller to this
			computer's default Bluetooth adapter address. This function
			has been implemented and tested with the following operating
			systems and Bluetooth stacks:

			* Linux 2.6 (Bluez)
			* Mac OS X >= 10.6
			* Windows 7 (Microsoft Bluetooth stack)

			\attention On Windows, this function does not work with 3rd
			party stacks like Bluesoleil. Use psmove_pair_custom() and
			supply the Bluetooth host adapter address manually. It is
			recommended to only use the Microsoft Bluetooth stack for
			Windows with the PS Move API to avoid problems.

			If your computer doesn't have USB host mode (e.g. because it
			is a mobile device), you can use psmove_pair_custom() on a
			different computer and specify the Bluetooth address of the
			mobile device instead. For most use cases, you can use the
			\c psmovepair command-line utility.

			\param move A valid \ref PSMove handle

			\return \ref PSMove_True if the pairing was successful
			\return \ref PSMove_False if the pairing failed
		"""
		return libctrl.psmove_pair(self.move)

	def pair_custom(self):
		""" \brief Pair a controller connected via USB to a specific address.

			This function behaves the same as psmove_pair(), but allows you to
			specify a custom Bluetooth host address.

			\param move A valid \ref PSMove handle
			\param new_host_string The host address in the format \c "aa:bb:cc:dd:ee:ff"

			\return \ref PSMove_True if the pairing was successful
			\return \ref PSMove_False if the pairing failed
		"""
		return libctrl.psmove_pair_custom(self.move, self.new_host_string)

	def set_rate_limiting(self, enabled):
		""" \brief Enable or disable LED update rate limiting.

			If LED update rate limiting is enabled, psmove_update_leds() will make
			ignore extraneous updates (and return \ref Update_Ignored) if the update
			rate is too high, or if the color hasn't changed and the timeout has not
			been hit.

			By default, rate limiting is enabled.

			\warning If rate limiting is disabled, the read performance might
			         be decreased, especially on Linux.

			\param move A valid \ref PSMove handle
			\param enabled \ref PSMove_True to enable rate limiting,
			               \ref PSMove_False to disable
		"""
		return libctrl.psmove_set_rate_limiting(self.move, enabled)

	def set_leds(self, r, g, b):
		""" \brief Set the RGB LEDs on the PS Move controller.

			This sets the RGB values of the LEDs on the Move controller. Usage examples:

			\code
			   psmove_set_leds(move, 255, 255, 255);  // white
			   psmove_set_leds(move, 255, 0, 0);      // red
			   psmove_set_leds(move, 0, 0, 0);        // black (off)
			\endcode

			This function will only update the library-internal state of the controller.
			To really update the LEDs (write the changes out to the controller), you
			have to call psmove_update_leds() after calling this function.

			\param move A valid \ref PSMove handle
			\param r The red value (0..255)
			\param g The green value (0..255)
			\param b The blue value (0..255)
		"""
		return libctrl.psmove_set_leds(self.move, r, g, b)

	def set_led_pwm_frequency(self, freq):
		""" \brief Set the PWM frequency used in dimming the RGB LEDs.

			The RGB LEDs in the Move controller are dimmed using pulse-width modulation (PWM).
			This function lets you modify the PWM frequency. The default is around 188 kHz and
			can also be restored by resetting the controller (using the small reset button on
			the back).

			\note Make sure to switch off the LEDs prior to calling this function. If you do
			      not do this, changing the PWM frequency will switch off the LEDs and keep
			      them off until the Bluetooth connection has been teared down and then
			      reestablished.

			\note Frequency values outside the valid range (see the parameter description) are
			      treated as errors.

			\note Even though the controller lets you increase the frequency to several
			      Megahertz, there is usually not much use in operating at such extreme rates.
			      Additionally, you will even lose resolution at these rates, i.e. the number
			      of distinct LED intensities between "off" and "fully lit" decreases. For
			      example: at 7 MHz there are only 5 different intensities left instead of the
			      usual 256. This is not a feature of PWM per se but is rather due to
			      software/hardware limitations of the Move controller.

			\param freq The PWM frequency in Hertz (range is 733 Hz to 24 MHz)

			\return \ref PSMove_True on success
			\return \ref PSMove_False on error
		"""
		return libctrl.psmove_set_led_pwm_frequency(self.move, freq)

	def set_rumble(self):
		""" \brief Set the rumble intensity of the PS Move controller.

			This sets the rumble (vibration motor) intensity of the
			Move controller. Usage example:

			\code
			  psmove_set_rumble(move, 255);  // strong rumble
			  psmove_set_rumble(move, 128);  // medium rumble
			  psmove_set_rumble(move, 0);    // rumble off
			\endcode

			This function will only update the library-internal state of the controller.
			To really update the rumble intensity (write the changes out to the
			controller), you have to call psmove_update_leds() (the rumble value is sent
			together with the LED updates, that's why you have to call it even for
			rumble updates) after calling this function.

			\param move A valid \ref PSMove handle
			\param rumble The rumble intensity (0..255)
		"""
		return libctrl.psmove_set_rumble(self.move, rumble)

	def update_leds(self):
		""" \brief Send LED and rumble values to the controller.

			This writes the LED and rumble changes to the controller. You have to call
			this function regularly, or the controller will switch off the LEDs and
			rumble automatically (after about 4-5 seconds). When rate limiting is
			enabled, you can just call this function in your main loop, and the LEDs
			will stay on properly (with extraneous updates being ignored to not flood
			the controller with updates).

			When rate limiting (see psmove_set_rate_limiting()) is disabled, you have
			to make sure to not call this function not more often then e.g. every
			80 ms to avoid flooding the controller with updates.

			\param move A valid \ref PSMove handle

			\return \ref Update_Success on success
			\return \ref Update_Ignored if the change was ignored (see psmove_set_rate_limiting())
			\return \ref Update_Failed (= \c 0) on error
		"""
		return libctrl.psmove_update_leds(self.move)

	def poll(self):
		""" \brief Read new sensor/button data from the controller.

			For most sensor and button functions, you have to call this function
			to read new updates from the controller.

			How to detect dropped frames:

			\code
			    int seq_old = 0;
			    while (1) {
			        int seq = psmove_poll(move);
			        if ((seq_old > 0) && ((seq_old % 16) != (seq - 1))) {
			            // dropped frames
			        }
			        seq_old = seq;
			    }
			\endcode

			In practice, you usually use this function in a main loop and guard
			all your sensor/button updating functions with it:

			\code
			    while (1) {
			        if (psmove_poll(move)) {
			            unsigned int pressed, released;
			            psmove_get_button_events(move, &pressed, &released);
			            // process button events
			        }

			        // update the application state
			        // draw the current frame
			    }
			\endcode

			\return a positive sequence number (1..16) if new data is
			        available
			\return \c 0 if no (new) data is available or an error occurred

			\param move A valid \ref PSMove handle
		"""
		return libctrl.psmove_poll(self.move)

	def get_ext_data(self):
		""" \brief Get the extension device's data as reported by the Move.

			You need to call psmove_poll() first to read new data from the
			controller.

			\param move A valid \ref PSMove handle
			\param data Pointer to store the data, must not be \ref NULL

			\return \ref PSMove_True on success
			\return \ref PSMove_False on error
		"""
		return libctrl.psmove_get_ext_data(self.move, self.data)

	def send_ext_data(self, data, length):
		""" \brief Send data to a connected extension device.

			\param move A valid \ref PSMove handle
			\param data Pointer to the data which to send
			\param length Number of bytes in \ref data

			\return \ref PSMove_True on success
			\return \ref PSMove_False on error
		"""
		return libctrl.psmove_send_ext_data(self.move, data, length)

	def get_buttons(self):
		""" \brief Get the current button states from the controller.

			The status of the buttons is described as a bitfield, with a bit
			in the result being \c 1 when the corresponding \ref PSMove_Button
			is pressed.

			You need to call psmove_poll() first to read new data from the
			controller.

			Example usage:

			\code
			    if (psmove_poll(move)) {
			        unsigned int buttons = psmove_get_buttons(move);
			        if (buttons & Btn_PS) {
			            printf("The PS button is currently pressed.\n");
			        }
			    }
			\endcode

			\param move A valid \ref PSMove handle

			\return A bit field of \ref PSMove_Button states
		"""
		return libctrl.psmove_get_buttons(self.move)

	def get_button_events(self):
		""" \brief Get new button events since the last call to this fuction.

			This is an advanced version of psmove_get_buttons() that takes care
			of tracking the previous button states and comparing the previous
			states with the current states to generate two bitfields, which is
			usually more suitable for event-driven applications:

			* \c pressed - all buttons that have been pressed since the last call
			* \c released - all buttons that have been released since the last call

			Example usage:

			\code
			    if (psmove_poll(move)) {
			        unsigned int pressed, released;
			        psmove_get_button_events(move, &pressed, &released);

			        if (pressed & Btn_MOVE) {
			            printf("The Move button has been pressed now.\n");
			        } else if (released & Btn_MOVE) {
			            printf("The Move button has been released now.\n");
			        }
			    }
			\endcode

			You need to call psmove_poll() first to read new data from the
			controller.

			\param move A valid \ref PSMove handle
			\param pressed Pointer to store a bitfield of new press events \c NULL
			\param released Pointer to store a bitfield of new release events \c NULL
		"""
		return libctrl.psmove_get_button_events(self.move, self.pressed, self.released)

	def is_ext_connected(self):
		""" \brief Check if an extension device is connected to the controller.

			\param move A valid \ref PSMove handle

			\return \ref PSMove_True if an extension device is connected
			\return \ref PSMove_False if no extension device is connected or in case of an error
		"""
		return libctrl.psmove_is_ext_connected(self.move)

	def get_ext_device_info(self):
		""" \brief Get information from an extension device connected to the controller.

			\note Since the information is retrieved from the extension device itself, a
			noticeable delay may occur when calling this function.

			\param move A valid \ref PSMove handle
			\param ext Pointer to a \ref PSMove_Ext_Device_Info that will store the
			           information. Must not be \ref NULL.

			\return \ref PSMove_True on success
			\return \ref PSMove_False on error
		"""
		return libctrl.psmove_get_ext_device_info(self.move, self.info)

	def get_battery(self):
		""" \brief Get the battery charge level of the controller.

			This function retrieves the charge level of the controller or
			the charging state (if the controller is currently being charged).

			See \ref PSMove_Battery_Level for details on the result values.

			You need to call psmove_poll() first to read new data from the
			controller.

			\param move A valid \ref PSMove handle
			\return A \ref PSMove_Battery_Level (\ref Batt_CHARGING when charging)
		"""
		return libctrl.psmove_get_battery(self.move)

	def get_temperature(self):
		""" \brief Get the current raw device temperature reading of the
			controller.

			This gets the raw sensor value of the internal temperature sensor.

			You need to call psmove_poll() first to read new data from the
			controller.

			\param move A valid \ref PSMove handle

			\return The raw temperature sensor reading
		"""
		return libctrl.psmove_get_temperature(self.move)

	def get_temperature_in_celsius(self):
		""" \brief Get the current device temperature reading in deg.
			Celsius.

			This gets the raw temperature sensor value of the internal
			temperature sensor and then converts it to deg. Celsius.

			The result range is -10...70 deg. Celcius. Values outside this range will be
			clipped.

			You need to call psmove_poll() first to read new data from the
			controller.

			\note This is NOT room temperature, but the temperature of a small
			thermistor on the controller's PCB. This means that under normal
			operation the temperature returned by this function will be higher
			than room temperature due to heat up from current flow.

			\param move A valid \ref PSMove handle

			\return The temperature sensor reading in deg. Celsius
		"""
		return libctrl.psmove_get_temperature_in_celsius(self.move)

	def get_trigger(self):
		""" \brief Get the value of the PS Move analog trigger.

			Get the current value of the PS Move analog trigger. The trigger
			is also exposed as digital button using psmove_get_buttons() in
			combination with \ref Btn_T.

			Usage example:

			\code
			    // Control the red LED brightness via the trigger
			    while (1) {
			        if (psmove_poll()) {
			            unsigned char value = psmove_get_trigger(move);
			            psmove_set_leds(move, value, 0, 0);
			            psmove_update_leds(move);
			        }
			    }
			\endcode

			You need to call psmove_poll() first to read new data from the
			controller.

			\param move A valid \ref PSMove handle

			\return 0 if the trigger is not pressed
			\return 1-254 when the trigger is partially pressed
			\return 255 if the trigger is fully pressed
		"""
		return libctrl.psmove_get_trigger(self.move)

	def get_accelerometer(self, ax, ay, az):
		""" \brief Get the raw accelerometer reading from the PS Move.

			This function reads the raw (uncalibrated) sensor values from
			the controller. To read calibrated sensor values, use
			psmove_get_accelerometer_frame().

			You need to call psmove_poll() first to read new data from the
			controller.

			\param move A valid \ref PSMove handle
			\param ax Pointer to store the raw X axis reading, or \c NULL
			\param ay Pointer to store the raw Y axis reading, or \c NULL
			\param az Pointer to store the raw Z axis reading, or \c NULL
		"""
		return libctrl.psmove_get_accelerometer(self.move, ax, ay, az)

	def get_gyroscope(self, gx, gy, gz):
		""" \brief Get the raw gyroscope reading from the PS Move.

			This function reads the raw (uncalibrated) sensor values from
			the controller. To read calibrated sensor values, use
			psmove_get_gyroscope_frame().

			You need to call psmove_poll() first to read new data from the
			controller.

			\param move A valid \ref PSMove handle
			\param gx Pointer to store the raw X axis reading, or \c NULL
			\param gy Pointer to store the raw Y axis reading, or \c NULL
			\param gz Pointer to store the raw Z axis reading, or \c NULL
		"""
		return libctrl.psmove_get_gyroscope(self.move, gx, gy, gz)

	def get_magnetometer(self, mx, my, mz):
		""" \brief Get the raw magnetometer reading from the PS Move.

			This function reads the raw sensor values from the controller,
			pointing to magnetic north.

			The result value range is -2048..+2047. The magnetometer is located
			roughly below the glowing orb - you can glitch the values with a
			strong kitchen magnet by moving it around the bottom ring of the orb.
			You can detect if a magnet is nearby by checking if any two values
			stay at zero for several frames.

			You need to call psmove_poll() first to read new data from the
			controller.

			\param move A valid \ref PSMove handle
			\param mx Pointer to store the raw X axis reading, or \c NULL
			\param my Pointer to store the raw Y axis reading, or \c NULL
			\param mz Pointer to store the raw Z axis reading, or \c NULL
		"""
		return libctrl.psmove_get_magnetometer(self.move, mx, my, mz)

	def get_accelerometer_frame(self, frame, ax, ay, az):
		""" \brief Get the calibrated accelerometer values (in g) from the controller.

			Assuming that psmove_has_calibration() returns \ref PSMove_True, this
			function will give you the calibrated accelerometer values in g. To get
			the raw accelerometer readings, use psmove_get_accelerometer().

			Usage example:

			\code
			    if (psmove_poll(move)) {
			        float ay;
			        psmove_get_accelerometer_frame(move, Frame_SecondHalf,
			                NULL, &ay, NULL);

			        if (ay > 0.5) {
			            printf("Controller is pointing up.\n");
			        } else if (ay < -0.5) {
			            printf("Controller is pointing down.\n");
			        }
			    }
			\endcode

			You need to call psmove_poll() first to read new data from the
			controller.

			\param move A valid \ref PSMove handle
			\param frame \ref Frame_FirstHalf or \ref Frame_SecondHalf (see \ref PSMove_Frame)
			\param ax Pointer to store the X axis reading, or \c NULL
			\param ay Pointer to store the Y axis reading, or \c NULL
			\param az Pointer to store the Z axis reading, or \c NULL
		"""
		return libctrl.psmove_get_accelerometer_frame(self.move, frame, ax, ay, az)

	def get_gyroscope_frame(self, frame, gx, gy, gz):
		""" \brief Get the calibrated gyroscope values (in rad/s) from the controller.

			Assuming that psmove_has_calibration() returns \ref PSMove_True, this
			function will give you the calibrated gyroscope values in rad/s. To get
			the raw gyroscope readings, use psmove_get_gyroscope().

			Usage example:

			\code
			    if (psmove_poll(move)) {
			        float gz;
			        psmove_get_gyroscope_frame(move, Frame_SecondHalf,
			                NULL, NULL, &gz);

			        // Convert rad/s to RPM
			        gz = gz * 60 / (2*M_PI);
			        printf("Rotation: %.2f RPM\n", gz);
			    }
			\endcode

			You need to call psmove_poll() first to read new data from the
			controller.

			\param move A valid \ref PSMove handle
			\param frame \ref Frame_FirstHalf or \ref Frame_SecondHalf (see \ref PSMove_Frame)
			\param gx Pointer to store the X axis reading, or \c NULL
			\param gy Pointer to store the Y axis reading, or \c NULL
			\param gz Pointer to store the Z axis reading, or \c NULL
		"""
		return libctrl.psmove_get_gyroscope_frame(self.move, frame, gx, gy, gz)

	def get_magnetometer_vector(self, mx, my, mz):
		""" \brief Get the normalized magnetometer vector from the controller.

			The normalized magnetometer vector is a three-axis vector where each
			component is in the range [-1,+1], including both endpoints. The range
			will be dynamically determined based on the highest (and lowest) value
			observed during runtime. To get the raw magnetometer readings, use
			psmove_get_magnetometer().

			You need to call psmove_poll() first to read new data from the
			controller.

			\param move A valid \ref PSMove handle
			\param mx Pointer to store the X axis reading, or \c NULL
			\param my Pointer to store the Y axis reading, or \c NULL
			\param mz Pointer to store the Z axis reading, or \c NULL
		"""
		return libctrl.psmove_get_magnetometer_vector(self.move, mx, my, mz)

	def has_calibration(self):
		""" \brief Check if calibration is available on this controller.

			For psmove_get_accelerometer_frame() and psmove_get_gyroscope_frame()
			to work, the calibration data has to be availble. This usually happens
			at pairing time via USB. The calibration files are stored in the PS
			Move API data directory (see psmove_util_get_data_dir()) and can be
			copied between machines (e.g. from the machine you do your pairing to
			the machine where you run the API on, which is especially important for
			mobile devices, where USB host mode might not be supported).

			If no calibration is available, the two functions returning calibrated
			values will return uncalibrated values. Also, the orientation features
			will not work without calibration.

			\param move A valid \ref PSMove handle

			\return \ref PSMove_True if calibration is supported, \ref PSMove_False otherwise
		"""
		return libctrl.psmove_has_calibration(self.move)

	def dump_calibration(self):
		""" \brief Dump the calibration information to stdout.

			This is mostly useful for developers wanting to analyze the
			calibration data of a given PS Move controller for debugging
			or development purposes.

			The current calibration information (if available) will be printed to \c
			stdout, including an interpretation of raw values where available.

			\param move A valid \ref PSMove handle
		"""
		return libctrl.psmove_dump_calibration(self.move)

	def enable_orientation(self, enabled):
		""" \brief Enable or disable orientation tracking.

			This will enable orientation tracking and update the internal orientation
			quaternion (which can be retrieved using psmove_get_orientation()) when
			psmove_poll() is called.

			In addition to enabling the orientation tracking features, calibration data
			and an orientation algorithm (usually built-in) has to be used, too. You can
			use psmove_has_orientation() after enabling orientation tracking to check if
			orientation features can be used.

			\param move A valid \ref PSMove handle
			\param enabled \ref PSMove_True to enable orientation tracking, \ref PSMove_False to disable
		"""
		return libctrl.psmove_enable_orientation(self.move, enabled)

	def has_orientation(self):
		""" \brief Check if orientation tracking is available for this controller.

			The orientation tracking feature depends on the availability of an
			orientation tracking algorithm (usually built-in) and the calibration
			data availability (as determined by psmove_has_calibration()). In addition
			to that (because orientation tracking is somewhat computationally
			intensive, especially on embedded systems), you have to enable the
			orientation tracking manually via psmove_enable_orientation()).

			If this function returns \ref PSMove_False, the orientation features
			will not work - check for missing calibration data and make sure that
			you have called psmove_enable_orientation() first.

			\param move A valid \ref PSMove handle

			\return \ref PSMove_True if calibration is supported, \ref PSMove_False otherwise
		"""
		return libctrl.psmove_has_orientation(self.move)

	def get_orientation(self, w, x, y, z):
		""" \brief Get the current orientation as quaternion.

			This will get the current 3D rotation of the PS Move controller as
			quaternion. You will have to call psmove_poll() regularly in order to have
			good quality orientation tracking.

			In order for the orientation tracking to work, you have to enable tracking
			first using psmove_enable_orientation().  In addition to enabling tracking,
			an orientation algorithm has to be present, and calibration data has to be
			available. You can use psmove_has_orientation() to check if all
			preconditions are fulfilled to do orientation tracking.

			\param move A valid \ref PSMove handle
			\param w A pointer to store the w part of the orientation quaternion
			\param x A pointer to store the x part of the orientation quaternion
			\param y A pointer to store the y part of the orientation quaternion
			\param z A pointer to store the z part of the orientation quaternion
		"""
		return libctrl.psmove_get_orientation(self.move, w, x, y, z)

	def reset_orientation(self):
		""" \brief Reset the current orientation quaternion.

			This will set the current 3D rotation of the PS Move controller as
			quaternion. You can use this function to re-adjust the orientation of the
			controller when it points towards the camera.

			This function will automatically be called by psmove_enable_orientation(),
			but you might want to call it manually when the controller points towards
			the screen/camera for accurate orientation readings relative to a
			known-good orientation of the controller.

			\param move A valid \ref PSMove handle
		"""
		return libctrl.psmove_reset_orientation(self.move)

	def get_angles(self, xAngle, yAngle, zAngle):
		""" \brief Get the Euler angles of the controller.

			This will get the current x,y,z Euler angles of the controller.
			You will have to call psmove_poll() regularly to have good quality data.

			In order for this to work, you have to enable orienation and have an IMU
			sensor fusion algorithm enabled (i.e., Madgwick). You can use
			psmove_has_orientation() to check if all preocnditions are met.

			\param move A valid \ref PSMove handle
			\param xAngle A pointer to store the xAngle
			\param yAngle A pointer to store the yAngle
			\param zAngle A pointer to store the zAngle
		"""
		return libctrl.psmove_get_angles(self.move, xAngle, yAngle, zAngle)

	def reset_magnetometer_calibration(self):
		""" \brief Reset the magnetometer calibration state.

			This will reset the magnetometer calibration data, so they can be
			re-adjusted dynamically. Used by the calibration utility.

			\ref move A valid \ref PSMove handle
		"""
		return libctrl.psmove_reset_magnetometer_calibration(self.move)

	def save_magnetometer_calibration(self):
		""" \brief Save the magnetometer calibration values.

			This will save the magnetometer calibration data to persistent storage.
			If a calibration already exists, this will overwrite the old values.

			\param move A valid \ref PSMove handle
		"""
		return libctrl.psmove_save_magnetometer_calibration(self.move)

	def get_magnetometer_calibration_range(self):
		""" \brief Return the raw magnetometer calibration range.

			The magnetometer calibration is dynamic at runtime - this function returns
			the raw range of magnetometer calibration. The user should rotate the
			controller in all directions to find the response range of the controller
			(this will be dynamically adjusted).

			\param move A valid \ref PSMove handle

			\return The smallest raw sensor range difference of all three axes
		"""
		return libctrl.psmove_get_magnetometer_calibration_range(self.move)

	def disconnect(self):
		""" \brief Disconnect from the PS Move and release resources.

			This will disconnect from the controller and release any resources allocated
			for handling the controller. Please note that this does not disconnect the
			controller from the system (as the Bluetooth stack of the operating system
			usually keeps the HID connection alive), but will rather disconnect the
			application from the controller.

			To really disconnect the controller from your computer, you can either press
			the PS button for ~ 10 seconds or use the Bluetooth application of your
			operating system to disconnect the controller.

			\param move A valid \ref PSMove handle (which will be invalid after this call)
		"""
		return libctrl.psmove_disconnect(self.move)

	def reinit(self):
		""" \brief Reinitialize the library.

			Required for detecting new and removed controllers (at least on Mac OS X).
			Make sure to disconnect all controllers (using psmove_disconnect) before
			calling this, otherwise it won't work.

			You do not need to call this function at application startup.

			\bug It should be possible to auto-detect newly-connected controllers
			     without having to rely on this function.
		"""
		return libctrl.psmove_reinit()

	def util_get_ticks(self):
		""" \brief Get milliseconds since first library use.

			This function is used throughout the library to take care of timing and
			measurement. It implements a cross-platform way of getting the current
			time, relative to library use.

			\return Time (in ms) since first library use.
		"""
		return libctrl.psmove_util_get_ticks()

	def util_get_data_dir(self):
		""" \brief Get local save directory for settings.

			The local save directory is a PS Move API-specific directory where the
			library and its components will store files such as calibration data,
			tracker state and configuration files.

			\return The local save directory for settings.
			        The returned value is reserved in static memory - it must not be freed!
		"""
		return libctrl.psmove_util_get_data_dir()

	def util_get_file_path(self):
		""" \brief Get a filename path in the local save directory.

			This is a convenience function wrapping psmove_util_get_data_dir()
			and will give the absolute path of the given filename.

			The data directory will be created in case it doesn't exist yet.

			\param filename The basename of the file (e.g. \c myfile.txt)

			\return The absolute filename to the file. The caller must
			         free() the result when it is not needed anymore.
			\return On error, \c NULL is returned.
		"""
		return libctrl.psmove_util_get_file_path(filename)

	def util_get_system_file_path(self):
		""" \brief Get a filename path in the system save directory.

			This is a convenience function, which gives the absolute path for
			a file stored in system-wide data directory.

			The data directory will NOT be created in case it doesn't exist yet.

			\param filename The basename of the file (e.g. \c myfile.txt)

			\return The absolute filename to the file. The caller must
			        free() the result when it is not needed anymore.
			\return On error, \c NULL is returned.
		"""
		return libctrl.psmove_util_get_system_file_path(filename)

	def util_get_env_int(self):
		""" \brief Get an integer from an environment variable

			Utility function used to get configuration from environment
			variables.

			\param name The name of the environment variable

			\return The integer value of the environment variable, or -1 if
			        the variable is not set or could not be parsed as integer.
		"""
		return libctrl.psmove_util_get_env_int(name)

	def util_get_env_string(self):
		""" \brief Get a string from an environment variable

			Utility function used to get configuration from environment
			variables.

			\param name The name of the environment variable

			\return The string value of the environment variable, or NULL if the
			        variable is not set. The caller must free() the result when
			        it is not needed anymore.
		"""
		return libctrl.psmove_util_get_env_string(name)

# =================================================================
# ======= Free Functions provided by the PSMoveTracker API  =======
# =================================================================
class PSMoveTracker:

	def __init__(self):
		self.tracker = self.new()
		if not self.tracker: print "Could not init PSMoveTracker.\n"


	def new(self):
		""" \brief Create a new PS Move Tracker instance and open the camera
		
			This will select the best camera for tracking (this means that if
			a PSEye is found, it will be used, otherwise the first available
			camera will be used as fallback).
			
			\return A new \ref PSMoveTracker instance or \c NULL on error
		"""
		return libtrck.psmove_tracker_new()

	def new_with_camera(self):
		""" \brief Create a new PS Move Tracker instance with a specific camera

			This function can be used when multiple cameras are available to
			force the use of a specific camera.

			Usually it's better to use psmove_tracker_new() and let the library
			choose the best camera, unless you have a good reason not to.

			\param camera Zero-based index of the camera to use

			\return A new \ref PSMoveTracker instance or \c NULL on error
		"""
		return libtrck.psmove_tracker_new_with_camera(self.camera)

	def set_auto_update_leds(self):
		""" \brief Configure if the LEDs of a controller should be auto-updated
		
		If auto-update is enabled (the default), the tracker will set and
		update the LEDs of the controller automatically. If not, the user
		must set the LEDs of the controller and update them regularly. In
		that case, the user can use psmove_tracker_get_color() to determine
		the color that the controller's LEDs have to be set to.
		
		\param tracker A valid \ref PSMoveTracker handle
		\param move A valid \ref PSMove handle
		\param auto_update_leds \ref PSMove_True to auto-update LEDs from
		                        the tracker, \ref PSMove_False if the user
		                        will take care of updating the LEDs
		"""
		return libtrck.psmove_tracker_set_auto_update_leds(self.tracker, self.move, auto_update_leds)

	def get_auto_update_leds(self, move):
		""" \brief Check if the LEDs of a controller are updated automatically

		This is the getter function for psmove_tracker_set_auto_update_leds().
		See there for details on what auto-updating LEDs means.

		\param tracker A valid \ref PSMoveTracker handle
		\param move A valid \ref PSMove handle

		\return \ref PSMove_True if the controller's LEDs are set to be
		        updated automatically, \ref PSMove_False otherwise
		"""
		return libtrck.psmove_tracker_get_auto_update_leds(self.tracker, move)

	def set_dimming(self, dimming):
		""" \brief Set the LED dimming value for all controller

		Usually it's not necessary to call this function, as the dimming
		is automatically determined when the first controller is enabled.

		\param tracker A valid \ref PSMoveTracker handle
		\param dimming A value in the range from 0 (LEDs switched off) to
		              1 (full LED intensity)
		"""
		return libtrck.psmove_tracker_set_dimming(self.tracker, dimming)

	def get_dimming(self):
		""" \brief Get the LED dimming value for all controllers

		See psmove_tracker_set_dimming() for details.

		\param tracker A valid \ref PSMoveTracker handle

		\return The dimming value for the LEDs
		"""
		return libtrck.psmove_tracker_get_dimming(self.tracker)

	def set_exposure(self, exposure):
		""" \brief Set the desired camera exposure mode

		This function sets the desired exposure mode. This should be
		called before controllers are added to the tracker, so that the
		dimming for the controllers can be determined for the specific
		exposure setting.

		\param tracker A valid \ref PSMoveTracker handle
		\param exposure One of the \ref PSMoveTracker_Exposure values
		"""
		return libtrck.psmove_tracker_set_exposure(self.tracker, exposure)

	def get_exposure(self):
		""" \brief Get the desired camera exposure mode

		See psmove_tracker_set_exposure() for details.

		\param tracker A valid \ref PSMoveTracker handle

		\return One of the \ref PSMoveTracker_Exposure values
		"""
		return libtrck.psmove_tracker_get_exposure(self.tracker)

	def enable_deinterlace(self, enabled):
		""" \brief Enable or disable camera image deinterlacing (line doubling)

		Enables or disables camera image deinterlacing for this tracker.
		You usually only want to enable deinterlacing if your camera source
		provides interlaced images (e.g. 1080i). The interlacing will be
		removed by doubling every other line. By default, deinterlacing is
		disabled.

		\param tracker A valid \ref PSMoveTracker handle
		\param enabled \ref PSMove_True to enable deinterlacing,
		               \ref PSMove_False to disable deinterlacing (default)
		"""
		return libtrck.psmove_tracker_enable_deinterlace(self.tracker, enabled)

	def set_mirror(self, enabled):
		""" \brief Enable or disable horizontal camera image mirroring

		Enables or disables horizontal mirroring of the camera image. The
		mirroring setting will affect the X coordinates of the controller
		positions tracked, as well as the output image. In addition, the
		sensor fusion module will mirror the orientation information if
		mirroring is set here. By default, mirroring is disabled.

		\param tracker A valid \ref PSMoveTracker handle
		\param enabled \ref PSMove_True to mirror the image horizontally,
		               \ref PSMove_False to leave the image as-is (default)
		"""
		return libtrck.psmove_tracker_set_mirror(self.tracker, enabled)

	def get_mirror(self):
		""" \brief Query the current camera image mirroring state

		See psmove_tracker_set_mirror() for details.

		\param tracker A valid \ref PSMoveTracker handle

		\return \ref PSMove_True if mirroring is enabled,
		        \ref PSMove_False if mirroring is disabled
		"""
		return libtrck.psmove_tracker_get_mirror(self.tracker)

	def enable(self, move):
		""" \brief Enable tracking of a motion controller

		Calling this function will register the controller with the
		tracker, and start blinking calibration. The user should hold
		the motion controller in front of the camera and wait for the
		calibration to finish.

		\param tracker A valid \ref PSMoveTracker handle
		\param move A valid \ref PSMove handle

		\return \ref Tracker_CALIBRATED if calibration succeeded
		\return \ref Tracker_CALIBRATION_ERROR if calibration failed
		"""
		return libtrck.psmove_tracker_enable(self.tracker, move)

	def enable_with_color(self, r, g, b):
		""" \brief Enable tracking with a custom sphere color

		This function does basically the same as psmove_tracker_enable(),
		but forces the sphere color to a pre-determined value.

		Using this function might give worse tracking results, because
		the color might not be optimal for a given lighting condition.

		\param tracker A valid \ref PSMoveTracker handle
		\param move A valid \ref PSMove handle
		\param r The red intensity of the desired color (0..255)
		\param g The green intensity of the desired color (0..255)
		\param b The blue intensity of the desired color (0..255)

		\return \ref Tracker_CALIBRATED if calibration succeeded
		\return \ref Tracker_CALIBRATION_ERROR if calibration failed
		"""
		return libtrck.psmove_tracker_enable_with_color(self.tracker, self.move, r, g, b)

	def disable(self):
		""" \brief Disable tracking of a motion controller

		If the \ref PSMove instance was never enabled, this function
		does nothing. Otherwise it removes the instance from the
		tracker and stops tracking the controller.

		\param tracker A valid \ref PSMoveTracker handle
		\param move A valid \ref PSMove handle
		"""
		return libtrck.psmove_tracker_disable(self.tracker, self.move)

	def get_color(self, r, g, b):
		""" \brief Get the desired sphere color of a motion controller

		Get the sphere color of the controller as it is set using
		psmove_update_leds(). This is not the color as the sphere
		appears in the camera - for that, see
		psmove_tracker_get_camera_color().

		\param tracker A valid \ref PSMoveTracker handle
		\param move A Valid \ref PSmove handle
		\param r Pointer to store the red component of the color
		\param g Pointer to store the green component of the color
		\param g Pointer to store the blue component of the color

		\return Nonzero if the color was successfully returned, zero if
		        if the controller is not enabled of calibration has not
		        completed yet.
		"""
		return libtrck.psmove_tracker_get_color(self.tracker, self.move, r, g, b)

	def get_camera_color(self, r, g, b):
		""" \brief Get the sphere color of a controller in the camera image

		Get the sphere color of the controller as it currently
		appears in the camera image. This is not the color that is
		set using psmove_update_leds() - for that, see
		psmove_tracker_get_color().

		\param tracker A valid \ref PSMoveTracker handle
		\param move A Valid \ref PSmove handle
		\param r Pointer to store the red component of the color
		\param g Pointer to store the green component of the color
		\param g Pointer to store the blue component of the color

		\return Nonzero if the color was successfully returned, zero if
		        if the controller is not enabled of calibration has not
		        completed yet.
		"""
		return libtrck.psmove_tracker_get_camera_color(self.tracker, self.move, r, g, b)

	def set_camera_color(self, r, g, b):
		""" \brief Set the sphere color of a controller in the camera image

		This function should only be used in special situations - it is
		usually not required to manually set the sphere color as it appears
		in the camera image, as this color is determined at runtime during
		blinking calibration. For some use cases, it might be useful to
		set the color manually (e.g. when the user should be able to select
		the color in the camera image after lighting changes).
		 
		\param tracker A valid \ref PSMoveTracker handle
		\param move A valid \ref PSMove handle
		\param r The red component of the color (0..255)
		\param g The green component of the color (0..255)
		\param b The blue component of the color (0..255)

		\return Nonzero if the color was successfully set, zero if
		        if the controller is not enabled of calibration has not
		        completed yet.
		"""
		return libtrck.psmove_tracker_set_camera_color(self.tracker, self.move, r, g, b)

	def get_status(self):
		""" \brief Query the tracking status of a motion controller

		This function returns the current tracking status (or calibration
		status if the controller is not calibrated yet) of a controller.

		\param tracker A valid \ref PSMoveTracker handle
		\param move A valid \ref PSMove handle

		\return One of the \ref PSMoveTracker_Status values
		"""
		return libtrck.psmove_tracker_get_status(self.tracker, self.move)

	def update_image(self):
		""" \brief Retrieve the next image from the camera

		This function should be called once per main loop iteration (even
		if multiple controllers are tracked), and will grab the next camera
		image from the camera input device.

		This function must be called before psmove_tracker_update().

		\param tracker A valid \ref PSMoveTracker handle
		"""
		return libtrck.psmove_tracker_update_image(self.tracker)

	def update(self, move):
		""" \brief Process incoming data and update tracking information

		This function tracks one or all motion controllers in the camera
		image, and updates tracking information such as position, radius
		and camera color.

		This function must be called after psmove_tracker_update_image().

		\param tracker A valid \ref PSMoveTracker handle
		\param move A valid \ref PSMove handle (to update a single controller)
		            or \c NULL to update all enabled controllers at once

		\return Nonzero if tracking was successful, zero otherwise
		"""
		return libtrck.psmove_tracker_update(self.tracker, move)

	def annotate(self):
		""" \brief Draw debugging information onto the current camera image

		This function has to be called after psmove_tracker_update(), and
		will annotate the camera image with sphere positions and other
		information. The camera image will be modified in place, so no
		call to psmove_tracker_update() should be carried out before the
		next call to psmove_tracker_update_image().

		This function is used for demonstration and debugging purposes, in
		production environments you usually do not want to use it.

		\param tracker A valid \ref PSMoveTracker handle
		"""
		return libtrck.psmove_tracker_annotate(self.tracker)

	def get_frame(self):
		""" \brief Get the current camera image as backend-specific pointer

		This function returns a pointer to the backend-specific camera
		image. Right now, the only backend supported is OpenCV, so the
		return value will always be a pointer to an IplImage structure.

		\param tracker A valid \ref PSMoveTracker handle

		\return A pointer to the camera image (currently always an IplImage)
		        - the caller MUST NOT modify or free the returned object.
		"""
		return libtrck.psmove_tracker_get_frame(self.tracker)

	def get_image(self):
		""" \brief Get the current camera image as 24-bit RGB data blob

		This function converts the internal camera image to a tightly-packed
		24-bit RGB image. The \ref PSMoveTrackerRGBImage structure is used
		to return the image data pointer as well as the width and height of
		the camera imaged. The size of the image data is 3 * width * height.

		The returned pixel data pointer points to tracker-internal data, and must
		not be freed. The returned RGB data will only be valid as long as the
		tracker exists.

		\param tracker A valid \ref PSMoveTracker handle
		 
		\return A \ref PSMoveTrackerRGBImage describing the RGB data and size.
		        The RGB data is owned by the tracker, and must not be freed by
		        the caller. The return value is valid only for the lifetime of
		        the tracker object.
		"""
		# ADDAPI PSMoveTrackerRGBImage
		return libtrck.psmove_tracker_get_image(self.tracker)

	def get_position(self, x, y, radius):
		""" \brief Get the current pixel position and radius of a tracked controller

		This function obtains the pixel position and radius of a controller in the
		camera image. Radius is actually the length of the major ellipse axis.

		\param tracker A valid \ref PSMoveTracker handle
		\param move A valid \ref PSMove handle
		\param x A pointer to store the X part of the position, or \c NULL
		\param y A pointer to store the Y part of the position, or \c NULL
		\param radius A pointer to store the controller radius, or \C NULL

		\return The age of the sensor reading in milliseconds, or -1 on error
		"""
		return libtrck.psmove_tracker_get_position(self.tracker, self.move, x, y, radius)

	def get_location(self, move, xcm, ycm, zcm):
		""" \brief Get the current 3D location of a tracked controller

		This function obtains the location of a controller in the
		world in cm.

		\param tracker A valid \ref PSMoveTracker handle
		\param move A valid \ref PSMove handle
		\param xcm A pointer to store the X part of the location, or \c NULL
		\param ycm A pointer to store the Y part of the location, or \c NULL
		\param zcm A pointer to store the Z part of the location, or \c NULL

		\return The age of the sensor reading in milliseconds, or -1 on error
		"""
		return libtrck.psmove_tracker_get_location(self.tracker, move, xcm, ycm, zcm)

	def get_size(self, width, height):
		""" \brief Get the camera image size for the tracker

		This function can be used to obtain the real camera image size used
		by the tracker. This is useful to convert the absolute position and
		radius values to values relative to the image size if a camera is
		used for which the size is now known. By default, the PS Eye camera
		is used with an image size of 640x480.

		\param tracker A valid \ref PSMoveTracker handle
		\param width A pointer to store the width of the camera image
		\param height A pointer to store the height of the camera image
		"""
		return libtrck.psmove_tracker_get_size(self.tracker, width, height)

	def free(self):
		""" \brief Destroy an existing tracker instance and free allocated resources

		This will shut down the tracker, clean up all state information for
		tracked controller as well as close the camera device. Return values
		for functions returning data pointing to internal tracker data structures
		will become invalid after this function call.

		\param tracker A valid \ref PSMoveTracker handle
		"""
		return libtrck.psmove_tracker_free(self.tracker)

# ==================================
# === Module Initialization Code ===
# ==================================
# libctrl*: library filename for libpsmoveapi (controller)
# libtrck*: library filename for libpsmoveapi_tracker
# find and load library
os_name = platform.system()
if os_name in ['Windows','Microsoft']:
    libext = '.dll'
elif os_name == 'Darwin':
    libext = '.dylib'
elif os_name == 'Linux':
    libext = '.so'
else:
    raise Exception("Unrecognized operating system:", os_name)
libctrlname = 'libpsmoveapi' + libext
libtrckname = 'libpsmoveapi_tracker' + libext
libctrlpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'build')
libctrlpath = libctrlpath + os.sep + libctrlname
libtrckpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'build')
libtrckpath = libtrckpath + os.sep + libtrckname

print 'PSMoveAPI Library Location: ', libctrlpath
print 'PSMoveTrackerAPI Library Location: ', libtrckpath

if not os.path.isfile(libctrlpath):
    libctrlpath = find_library(libctrlname)
if not libctrlpath:
    raise Exception("The library " + libctrlname + " was not found. Please make "
        "sure that it is on the search path (e.g., in the same folder as "
        "pylsl.py).")
libctrl = CDLL(libctrlpath)
libtrck = CDLL(libtrckpath)

# ===================================
# ==== set function return types ====
# ===================================

# ----------------------------------
# -------- libctrl.psmove_: --------
# ----------------------------------
libctrl.psmove_init.restype                         = c_int
# libctrl.psmove_set_remote_config.restype          = c_void
libctrl.psmove_count_connected.restype              = c_int
libctrl.psmove_connect.restype                      = c_void_p
libctrl.psmove_connect_by_id.restype                = c_void_p
libctrl.psmove_connection_type.restype              = c_int
libctrl.psmove_is_remote.restype                    = c_int
libctrl.psmove_get_serial.restype                   = c_char_p
libctrl.psmove_pair.restype                         = c_int
libctrl.psmove_pair_custom.restype                  = c_int
# libctrl.psmove_set_rate_limiting.restype          = c_void
# libctrl.psmove_set_leds.restype                   = c_void
libctrl.psmove_set_led_pwm_frequency.restype        = c_int
# libctrl.psmove_set_rumble.restype                 = c_void
libctrl.psmove_update_leds.restype                  = c_int
libctrl.psmove_poll.restype                         = c_int
libctrl.psmove_get_ext_data.restype                 = c_int
libctrl.psmove_send_ext_data.restype                = c_int
libctrl.psmove_get_buttons.restype                  = c_uint
# libctrl.psmove_get_button_events.restype          = c_void
libctrl.psmove_is_ext_connected.restype             = c_int
libctrl.psmove_get_ext_device_info.restype          = c_int
libctrl.psmove_get_battery.restype                  = c_int
libctrl.psmove_get_temperature.restype              = c_int
libctrl.psmove_get_temperature_in_celsius.restype   = c_float
libctrl.psmove_get_trigger.restype                  = c_ubyte
# libctrl.psmove_get_accelerometer.restype          = c_void
# libctrl.psmove_get_gyroscope.restype              = c_void
# libctrl.psmove_get_magnetometer.restype           = c_void
# libctrl.psmove_get_accelerometer_frame.restype    = c_void
# libctrl.psmove_get_gyroscope_frame.restype        = c_void
# libctrl.psmove_get_magneometer_vector.restype     = c_void
libctrl.psmove_has_calibration.restype              = c_int
# libctrl.psmove_dump_calibration.restype           = c_void
# libctrl.psmove_enable_orientation.restype         = c_void
libctrl.psmove_has_orientation.restype              = c_int
# libctrl.psmove_get_orientation.restype            = c_void
# libctrl.psmove_reset_orientation.restype          = c_void
# libctrl.psmove_get_angles.restype                 = c_void
# libctrl.psmove_reset_magnetometer_calibration.restype = c_void
# libctrl.psmove_save_magentometer_calibration.restype = c_void
libctrl.psmove_get_magnetometer_calibration_range.restype = c_int
# libctrl.psmove_disconnect.restype                 = c_void
# libctrl.psmove_reinit.restype                     = c_void
libctrl.psmove_util_get_ticks.restype               = c_long
libctrl.psmove_util_get_data_dir.restype            = c_char_p
libctrl.psmove_util_get_file_path.restype           = c_char_p
libctrl.psmove_util_get_system_file_path.restype    = c_char_p
libctrl.psmove_util_get_env_int.restype             = c_int
libctrl.psmove_util_get_env_string.restype          = c_char_p

# ----------------------------------
# ---- libtrck.psmove_tracker_: ----
# ----------------------------------
libtrck.psmove_tracker_new.restype                  = c_void_p
libtrck.psmove_tracker_new_with_camera.restype      = c_void_p
# libtrck.psmove_tracker_set_auto_update_leds.restype = c_void
libtrck.psmove_tracker_get_auto_update_leds.restype = c_int
# libtrck.psmove_tracker_set_dimming.restype        = c_void
libtrck.psmove_tracker_get_dimming.restype          = c_float
# libtrck.psmove_tracker_set_exposure.restype       = c_void
libtrck.psmove_tracker_get_exposure.restype         = c_int
# libtrck.psmove_tracker_enable_deinterlace.restype = c_void
# libtrck.psmove_tracker_set_mirror.restype         = c_void
libtrck.psmove_tracker_get_mirror.restype           = c_int
libtrck.psmove_tracker_enable.restype               = c_int
libtrck.psmove_tracker_enable_with_color.restype    = c_int
# libtrck.psmove_tracker_disable.restype            = c_void
libtrck.psmove_tracker_get_color.restype            = c_int
libtrck.psmove_tracker_get_camera_color.restype     = c_int
libtrck.psmove_tracker_set_camera_color.restype     = c_int
libtrck.psmove_tracker_get_status.restype           = c_int
# libtrck.psmove_tracker_update_image.restype       = void
libtrck.psmove_tracker_update.restype               = c_int
# libtrck.psmove_tracker_annotate.restype           = c_void
libtrck.psmove_tracker_get_frame.restype            = c_void_p
libtrck.psmove_tracker_get_image.restype            = c_int
libtrck.psmove_tracker_get_position.restype         = c_int
libtrck.psmove_tracker_get_location.restype         = c_int
# libtrck.psmove_tracker_get_size.restype           = c_void
# libtrck.psmove_tracker_free.restype               = c_void

# ==================================
# ==== set function args. types ====
# ==================================

# ----------------------------------
# -------- libctrl.psmove_: --------
# ----------------------------------
#*** are not working when uncommented
libctrl.psmove_init.argtypes                        = [c_int]
libctrl.psmove_set_remote_config.argtypes           = [c_int]
# libctrl.psmove_count_connected.argtypes           
# libctrl.psmove_connect.argtypes                   
libctrl.psmove_connect_by_id.argtypes               = [c_int]
libctrl.psmove_connection_type.argtypes             = [c_void_p]
libctrl.psmove_is_remote.argtypes                   = [c_void_p]
libctrl.psmove_get_serial.argtypes                  = [c_void_p]
libctrl.psmove_pair.argtypes                        = [c_void_p]
libctrl.psmove_pair_custom.argtypes                 = [c_void_p, c_char_p]
libctrl.psmove_set_rate_limiting.argtypes           = [c_void_p, c_int]
libctrl.psmove_set_leds.argtypes                    = [c_void_p, c_ubyte, c_ubyte, c_ubyte]
libctrl.psmove_set_led_pwm_frequency.argtypes       = [c_void_p, c_ulong]
libctrl.psmove_set_rumble.argtypes                  = [c_void_p, c_ubyte]
libctrl.psmove_update_leds.argtypes                 = [c_void_p]
libctrl.psmove_poll.argtypes                        = [c_void_p]
libctrl.psmove_get_ext_data.argtypes                = [c_void_p, c_int]
libctrl.psmove_send_ext_data.argtypes               = [c_void_p, c_char_p, c_ubyte]
libctrl.psmove_get_buttons.argtypes                 = [c_void_p]
libctrl.psmove_get_button_events.argtypes           = [c_void_p, c_uint, c_uint]
libctrl.psmove_is_ext_connected.argtypes            = [c_void_p]
libctrl.psmove_get_ext_device_info.argtypes         = [c_void_p, c_int] #*
libctrl.psmove_get_battery.argtypes                 = [c_void_p]
libctrl.psmove_get_temperature.argtypes             = [c_void_p]
libctrl.psmove_get_temperature_in_celsius.argtypes  = [c_void_p]
libctrl.psmove_get_trigger.argtypes                 = [c_void_p]
libctrl.psmove_get_accelerometer.argtypes           = [c_void_p, c_int, c_int, c_int]
libctrl.psmove_get_gyroscope.argtypes               = [c_void_p, c_int, c_int, c_int]
libctrl.psmove_get_magnetometer.argtypes            = [c_void_p, c_int, c_int, c_int]
libctrl.psmove_get_accelerometer_frame.argtypes     = [c_void_p, c_int, c_float, c_float, c_float]
libctrl.psmove_get_gyroscope_frame.argtypes         = [c_void_p, c_int, c_float, c_float, c_float]
# libctrl.psmove_get_magneometer_vector.argtypes      = [c_void_p, c_float, c_float, c_float] #***
libctrl.psmove_has_calibration.argtypes             = [c_void_p]
libctrl.psmove_dump_calibration.argtypes            = [c_void_p]
libctrl.psmove_enable_orientation.argtypes          = [c_void_p, c_int]
libctrl.psmove_has_orientation.argtypes             = [c_void_p]
libctrl.psmove_get_orientation.argtypes             = [c_void_p, c_float, c_float, c_float, c_float]
libctrl.psmove_reset_orientation.argtypes           = [c_void_p]
libctrl.psmove_get_angles.argtypes                  = [c_void_p, c_float, c_float, c_float]
libctrl.psmove_reset_magnetometer_calibration.argtypes     = [c_void_p]
# libctrl.psmove_save_magentometer_calibration.argtypes    = [c_void_p] #***
libctrl.psmove_get_magnetometer_calibration_range.argtypes = [c_void_p]
libctrl.psmove_disconnect.argtypes                  = [c_void_p]
# libctrl.psmove_reinit.argtypes                    
# libctrl.psmove_util_get_ticks.argtypes            
# libctrl.psmove_util_get_data_dir.argtypes         
libctrl.psmove_util_get_file_path.argtypes          = [c_char_p]
libctrl.psmove_util_get_system_file_path.argtypes   = [c_char_p]
libctrl.psmove_util_get_env_int.argtypes            = [c_char_p]
libctrl.psmove_util_get_env_string.argtypes         = [c_char_p]

# ----------------------------------
# ---- libtrck.psmove_tracker_: ----
# ----------------------------------
# libtrck.psmove_tracker_new.argtypes                
libtrck.psmove_tracker_new_with_camera.argtypes      = [c_int]
libtrck.psmove_tracker_set_auto_update_leds.argtypes = [c_void_p, c_void_p, c_int]
libtrck.psmove_tracker_get_auto_update_leds.argtypes = [c_void_p, c_void_p]
libtrck.psmove_tracker_set_dimming.argtypes          = [c_void_p, c_float]
libtrck.psmove_tracker_get_dimming.argtypes          = [c_void_p]
libtrck.psmove_tracker_set_exposure.argtypes         = [c_void_p, c_int]
libtrck.psmove_tracker_get_exposure.argtypes         = [c_void_p]
libtrck.psmove_tracker_enable_deinterlace.argtypes   = [c_void_p, c_int]
libtrck.psmove_tracker_set_mirror.argtypes           = [c_void_p, c_int]
libtrck.psmove_tracker_get_mirror.argtypes           = [c_void_p]
libtrck.psmove_tracker_enable.argtypes               = [c_void_p, c_void_p]
libtrck.psmove_tracker_enable_with_color.argtypes    = [c_void_p, c_void_p, c_ubyte, c_ubyte, c_ubyte]
libtrck.psmove_tracker_disable.argtypes              = [c_void_p, c_void_p]
libtrck.psmove_tracker_get_color.argtypes            = [c_void_p, c_void_p, c_ubyte, c_ubyte, c_ubyte]
libtrck.psmove_tracker_get_camera_color.argtypes     = [c_void_p, c_void_p, c_ubyte, c_ubyte, c_ubyte]
libtrck.psmove_tracker_set_camera_color.argtypes     = [c_void_p, c_void_p, c_ubyte, c_ubyte, c_ubyte]
libtrck.psmove_tracker_get_status.argtypes           = [c_void_p, c_void_p]
libtrck.psmove_tracker_update_image.argtypes         = [c_void_p]
libtrck.psmove_tracker_update.argtypes               = [c_void_p, c_void_p]
libtrck.psmove_tracker_annotate.argtypes             = [c_void_p]
libtrck.psmove_tracker_get_frame.argtypes            = [c_void_p]
libtrck.psmove_tracker_get_image.argtypes            = [c_void_p]
libtrck.psmove_tracker_get_position.argtypes         = [c_void_p, c_void_p, c_float, c_float, c_float]
libtrck.psmove_tracker_get_location.argtypes         = [c_void_p, c_void_p, POINTER(c_float), POINTER(c_float), POINTER(c_float)]
libtrck.psmove_tracker_get_size.argtypes             = [c_void_p, c_int, c_int]
libtrck.psmove_tracker_free.argtypes                 = [c_void_p]
