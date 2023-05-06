#include "chirp_bsp.h"
#include "chirp_board_config.h"
#include "Arduino.h"
#include "Wire.h"
#include "teensyChirpPins.h"
#include <avr/io.h>
#include <avr/interrupt.h>

/*!
 * \brief Main hardware initialization
 *
 * This function executes the required hardware initialization sequence for the board being used.
 * This includes clock, memory, and processor setup as well as any special handling that is needed.
 * This function is called at the beginning of an application, as the first operation.
 *
 * Along with the actual hardware initialization, this function must also initialize the following 
 * fields within the ch_group_t sensor group descriptor:
 *  - \b num_ports = number of ports on the board, usually the same as \a CHIRP_MAX_DEVICES 
 *  in \b chirp_board_config.h.
 *  - \b num_i2c_buses = number of ports on the board, usually the same as \a CHIRP_NUM_I2C_BUSES 
 *  in \b chirp_board_config.h.
 *  - \b rtc_cal_pulse_ms = length (duration) of the pulse sent on the INT line to each sensor 
 *  during calibration of the real-time clock
 *
 *	#### Discovering If a Sensor Is Present
 * Often, during initialization the BSP needs to determine which sensor ports (possible 
 * connections) actually have a sensor attached. Here is a short sequence you can use to 
 * confirm if a Chirp sensor is alive and communicating by reading two signature byte
 * values from the device using I2C.  This sequence applies to both CH101 
 * and CH201 devices.
 *
 * A couple key points:
 * - The initial I2C address for all sensors is \a CH_I2C_ADDR_PROG (0x45).  This address is 
 *   used during initialization and programming.  Once the device is programmed, a different 
 *   I2C address is assigned for normal operation.
 * - A device will only respond to this programming address (0x45) if its PROG line is asserted 
 *   (active high).
 *
 * So, the overall sequence should be:
 *  -# Power on board and device, initialize I2C bus.
 *  -# Assert the PROG line for the sensor port to be tested (active high).
 *  -# Perform a two-byte I2C register read from the device from this location:
 *  	- I2C address = \a CH_I2C_ADDR_PROG (0x45)
 *  	- Register address/offset = 0x00
 *  -# Check the byte values that were read from the device.  If a Chirp sensor is present, the 
 *  returned bytes should be: 
 *		- \a CH_SIG_BYTE_0	(hex value \b 0x0A)
 *		- \a CH_SIG_BYTE_1	(hex value \b 0x02)
 *  -# De-assert the PROG line for the sensor port.
 *	
 * This function is REQUIRED.
 */
void chbsp_board_init(ch_group_t *grp_ptr){
  grp_ptr->num_ports = CHIRP_MAX_NUM_SENSORS;
  grp_ptr->num_i2c_buses=CHIRP_NUM_I2C_BUSES;
  grp_ptr->rtc_cal_pulse_ms = 200;

  //initialize I2C bus
  Wire.setClock(400000); // setting 400KHz speed to match sensor
  Wire.begin();
  Wire.setSDA(sdaPin);
  Wire.setSCL(sclPin);
  // Assert PROG Line
  
  digitalWrite(progPin, HIGH);

  //Read 2 bytes from program address 0x45
  int nBytes = Wire.requestFrom(CH_I2C_ADDR_PROG,2);
  byte firstByte = Wire.read();
  byte secondByte = Wire.read();

  // NOT SURE IF THIS IS RIGHT, reading back A and FF
  Serial.println(nBytes);
  Serial.println(firstByte, HEX);
  Serial.println(secondByte, HEX);
  Wire.endTransmission();
  // the two values should be 0x0A and 0x02 if chirp is present

  // De assert PROG LINE
  digitalWrite(progPin, LOW);
  }

/*!
 * \brief Assert the reset pin for all sensors.
 *
 * This function should drive the Chirp sensor reset pin low (assert RESET_N) on all sensors.
 *
 * This function is REQUIRED.
 */
void chbsp_reset_assert(void){
  
  digitalWrite(rstPin, LOW);
}

/*!
 * \brief Deassert the reset pin for all sensors.
 *
 * This function should drive the Chirp sensor reset pin high (or open drain if there is a pull-up) 
 * on all sensors.
 *
 * This function is REQUIRED.
 */
void chbsp_reset_release(void){
  
  digitalWrite(rstPin, HIGH);
}

/*!
 * \brief Assert the PROG pin
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function should drive the Chirp sensor PROG pin high for the specified device.  It is used 
 * by the driver to initiate I2C communication with a specific Chirp sensor device before a unique 
 * I2C address is assigned to the device or when the programming interface is used.
 *
 * When the PROG pin is asserted, the device will respond to the standard programming I2C 
 * address (0x45).
 *
 * This function is REQUIRED.
 */
void chbsp_program_enable(ch_dev_t *dev_ptr){
  digitalWrite(progPin, HIGH);
}

/*!
 * \brief Deassert the PROG pin
 * 
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function should drive the Chirp sensor PROG pin low for the specified device.
 *
 * This function is REQUIRED.
 */
void chbsp_program_disable(ch_dev_t *dev_ptr){
  digitalWrite(progPin, LOW);
}

// THESE MIGHT NOT WORK
// I think i need to configure the chip's port as an input or output, not the teensy
/*!
 * \brief Configure the Chirp sensor INT pin as an output for a group of sensors
 * 
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * This function should configure each Chirp sensor's INT pin as an output (from the perspective 
 * of the host system).
 *
 * This function is REQUIRED.
 */
void chbsp_group_set_io_dir_out(ch_group_t *grp_ptr){
  pinMode(initPin, OUTPUT);
}

/*!
 * \brief Configure the Chirp sensor INT pins as inputs for a group of sensors
 * 
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 * 
 * This function should configure each Chirp sensor's INT pin as an input (from the perspective 
 * of the host system).
 *
 * This function is REQUIRED.
 */
void chbsp_group_set_io_dir_in(ch_group_t *grp_ptr){
  pinMode(initPin, INPUT);
}

/*!
 * \brief Initialize the set of I/O pins for a group of sensors.
 * 
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * This function initializes the set of I/O pins used for a group of sensors.  It should perform 
 * any clock initialization and other setup required to use the ports/pins.  It should then 
 * configure the RESET_N and PROG pins as outputs (from the perspective of the host system), and 
 * assert both RESET_N and PROG. It should also configure the INT pin as an input.
 *
 * This function is REQUIRED.
 */
void chbsp_group_pin_init(ch_group_t *grp_ptr){
  pinMode(progPin, OUTPUT);
  pinMode(rstPin, OUTPUT);
  pinMode(initPin, INPUT);
  digitalWrite(progPin,LOW);
  digitalWrite(rstPin,HIGH);
}

/*!
 * \brief Set the INT pins low for a group of sensors.
 * 
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * This function should drive the INT line low for each sensor in the group.
 *
 * This function is REQUIRED.
 */
void chbsp_group_io_clear(ch_group_t *grp_ptr){
  pinMode(initPin, OUTPUT);
  digitalWrite(initPin, LOW);
}

/*!
 * \brief Set the INT pins high for a group of sensors.
 * 
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * This function should drive the INT line high for each sensor in the group.
 *
 * This function is REQUIRED.
 */
void chbsp_group_io_set(ch_group_t *grp_ptr){
  pinMode(initPin, OUTPUT);
  digitalWrite(initPin, HIGH);
}

/*!
 * \brief Enable interrupts for a group of sensors.
 * 
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * For each sensor in the group, this function should enable the host interrupt associated with 
 * the Chirp sensor device's INT line.
 *
 * This function is REQUIRED.
 */
void chbsp_group_io_interrupt_enable(ch_group_t *grp_ptr){
  pinMode(initPin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(initPin),intInterrupt,RISING);
}

//Interrupt routine for int at pin 2
void intInterrupt(){
  Serial.println("Stop interrupting me!");
}
/*!
 * \brief Disable interrupts for a group of sensors
 * 
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * For each sensor in the group, this function should disable the host interrupt associated 
 * with the Chirp sensor device's INT line.
 * *
 * This function is REQUIRED.
 */
void chbsp_group_io_interrupt_disable(ch_group_t *grp_ptr){
  detachInterrupt(digitalPinToInterrupt(initPin));
}

/*!
 * \brief Disable the interrupt for one sensor
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function should disable the host interrupt associated with the Chirp sensor device's 
 * INT line.
 *
 * This function is REQUIRED.
 */
void chbsp_io_interrupt_disable(ch_dev_t *dev_ptr){
  detachInterrupt(digitalPinToInterrupt(initPin));
}


/*!
 * \brief Set callback routine for Chirp sensor I/O interrupt
 *
 * \param callback_func_ptr 	pointer to application function to be called when interrupt occurs
 *
 * This function sets up the specified callback routine to be called whenever the interrupt 
 * associated with the sensor's INT line occurs.  The implementation will typically save the 
 * callback routine address in a pointer variable that can later be accessed from within the 
 * interrupt handler to call the function.
 *
 * The callback function will generally be called at interrupt level from the interrupt 
 * service routine.
 *
 * This function is REQUIRED.
 */
void chbsp_io_callback_set(ch_io_int_callback_t callback_func_ptr);

/*!
 * \brief Delay for specified number of microseconds
 * 
 * \param us  	number of microseconds to delay before returning
 *
 * This function should wait for the specified number of microseconds before returning to 
 * the caller.
 *
 * This function is REQUIRED.
 */
void chbsp_delay_us(uint32_t us){
  delay(us);
}

/*!
 * \brief Delay for specified number of milliseconds.
 *
 * \param ms 	number of milliseconds to delay before returning
 *
 * This function should wait for the specified number of milliseconds before returning to 
 * the caller.
 *
 * This function is REQUIRED.
 *
 * \note This function is used during the \a ch_group_start() function to control the length 
 * of the calibration pulse sent to the Chirp sensor device during the real-time clock (RTC) 
 * calibration, based on the pulse length specified in the board support package.  The 
 * accuracy of this pulse timing will directly affect the accuracy of the range values 
 * calculated by the sensor.
 */
void chbsp_delay_ms(uint32_t ms){
  delay(ms);
}

/*!
 * \brief Initialize the host's I2C hardware.
 *
 * \return 0 if successful, 1 on error
 *
 * This function should perform general I2C initialization on the host system.  This includes 
 * both hardware initialization and setting up any necessary software structures.  Upon 
 * successful return from this routine, the system should be ready to perform I/O operations 
 * such as \a chbsp_i2c_read() and \a chbsp_i2c_write().
 *
 * This function is REQUIRED.
 */
int chbsp_i2c_init(void);

/*!
 * \brief Return I2C information for a sensor port on the board.
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 * \param dev_num		device number within sensor group
 * \param info_ptr		pointer to structure to be filled with I2C config values
 *
 * \return 0 if successful, 1 if error
 *
 * This function is called by SonicLib functions to obtain I2C operating parameters for 
 * a specific device on the board.
 *
 * This function returns I2C values in the ch_i2c_info_t structure specified by \a info_ptr.
 * The structure includes three fields.  
 *  - The \a address field contains the I2C address for the sensor.  
 *  - The \a bus_num field contains the I2C bus number (index).  
 *  - The \a drv_flags field contains various bit flags through which the BSP can inform 
 *  SonicLib driver functions to perform specific actions during I2C I/O operations.  The 
 *  possible flags include:
 *  - - \a I2C_DRV_FLAG_RESET_AFTER_NB - the I2C interface needs to be reset after non-blocking 
 *  transfers
 *  - - \a I2C_DRV_FLAG_USE_PROG_NB - use high-speed programming interface for non-blocking 
 *  transfers
 *
 * This function is REQUIRED.
 */
uint8_t chbsp_i2c_get_info(ch_group_t *grp_ptr, uint8_t dev_num, ch_i2c_info_t *info_ptr);

/*!
 * \brief Write bytes to an I2C slave.
 * 
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param data 			data to be transmitted
 * \param num_bytes 	length of data to be transmitted
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function should write one or more bytes of data to an I2C slave device.
 * The I2C interface will have already been initialized using \a chbsp_i2c_init().
 *
 * \note Implementations of this function should use the \a ch_get_i2c_address() function to 
 * obtain the device I2C address.
 */
int chbsp_i2c_write(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes){}

/*!
 * \brief Write bytes to an I2C slave using memory addressing.
 * 
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param mem_addr		internal memory or register address within device
 * \param data 			data to be transmitted
 * \param num_bytes 	length of data to be transmitted
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function should write one or more bytes of data to an I2C slave device using an internal 
 * memory or register address.  The remote device will write \a num_bytes bytes of
 * data starting at internal memory/register address \a mem_addr.
 * The I2C interface will have already been initialized using \a chbsp_i2c_init().
 *
 * The \a chbsp_i2c_mem_write() function is basically a standard I2C data write, except that the 
 * destination register address and the number of bytes being written must be included, like 
 * a header.
 * 
 * The byte sequence being sent should look like the following:
 *     0    |     1       |     2        |    3      |    4          |     5         |   etc...
 * :------: | :------:    | :------:     | :------:  | :------:      | :-------:     | :------:
 * I2C Addr | \a mem_addr | \a num_bytes |  *\a data | *(\a data + 1)| *(\a data + 2)|   ...
 * 
 * This function is REQUIRED.
 *
 * \note Implementations of this function should use the \a ch_get_i2c_address() function to obtain
 * the device I2C address.
 */
int chbsp_i2c_mem_write(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes);

/*!
 * \brief Read bytes from an I2C slave.
 * 
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param data 			pointer to receive data buffer
 * \param num_bytes 	number of bytes to read
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function should read the specified number of bytes from an I2C slave device.
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 *
 * This function is REQUIRED.
 *
 * \note Implementations of this function should use the \a ch_get_i2c_address() function to obtain
 * the device I2C address.
 */
int chbsp_i2c_read(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes);

/*!
 * \brief Read bytes from an I2C slave using memory addressing.
 * 
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param mem_addr		internal memory or register address within device
 * \param data 			pointer to receive data buffer
 * \param num_bytes 	number of bytes to read
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function should read the specified number of bytes from an I2C slave device, using
 * an internal memory or register address.  The remote device will return \a num_bytes bytes
 * starting at internal memory/register address \a mem_addr.
 *
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 *
 * There are two distinct phases to the transfer.  First, the register address must be written 
 * to the sensor, then a read operation must be done to obtain the data.  (Note that the byte 
 * count does not have to be sent to the device during a read operation, unlike in \a chbsp_i2c_mem_write()).)
 *
 * The preferred way to do this is with a *repeated-start operation*, in which the write phase is 
 * followed by another I2C Start condition (rather than a Stop) and then the read operation begins 
 * immediately.  This prevents another device on the bus from getting control between the write and 
 * read phases.  Many micro-controller I/O libraries include dedicated I2C functions to perform just 
 * such an operation for writing registers.
 *
 * When using the repeated start, the overall sequence looks like this:
 * 
 * |                         Write phase  ||    \|           |                   Read Phase                            ||||||   \|
 * :-----------: | :------:  | :------:    | :------------:  | :-----------------:  | :------:  | :-------:     | :------:      | :-------: | :---:
 * **I2C START** | I2C Addr  | \a mem_addr | \b I2C \b START | I2C Addr + read bit  |  *\a data | *(\a data + 1)| *(\a data + 2)| etc...    | \b I2C \b STOP
 * 
 * If the repeated-start technique is not used, it is possible to read from the sensor using separate, 
 * sequential I2C write and read operations.  In this case, the sequence is much the same, except that 
 * the write phase is ended by an I2C Stop condition, then an I2C Start is issued to begin the read phase.
 * However, this may be a problem if more than one bus master is present, 
 * because the bus is not held between the two phases.
 *
 * This function is REQUIRED.
 *
 * \note Implementations of this function should use the \a ch_get_i2c_address() function to obtain
 * the device I2C address.
 */
int chbsp_i2c_mem_read(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes);
