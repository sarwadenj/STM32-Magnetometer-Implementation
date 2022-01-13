#include "mbed.h"
#include <stdio.h>
#include "QMC5883L.h"    

/*LORA STACK libraries*/
#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "DummySensor.h"
#include "trace_helper.h"
#include "lora_radio_helper.h"

using namespace events;


/*****************************************************************************
* Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
* This example only communicates with much shorter messages (<30 bytes).
* If longer messages are used, these buffers must be changed accordingly.
* #define LORAMAC_PHY_MAXPAYLOAD                      255
* uint8_t tx_buffer[30];
* uint8_t rx_buffer[30];
*******************************************************************************/
#define LORAMAC_PHY_MAXPAYLOAD                      255

uint8_t tx_buffer[LORAMAC_PHY_MAXPAYLOAD];
uint8_t rx_buffer[LORAMAC_PHY_MAXPAYLOAD];

/* logic operators*/
static int status = 3;
static int statecount = 0;
static int id = 1;


/*MLX*/
DigitalOut led(LED1);
#define ENABLE_MAGNETOMETER 1
int addr = 0x0C << 1; // 8bit I2C address


I2C i2c(PB_9, PB_8);   //sda, scl

/***QMC5883L*****/
#ifdef ENABLE_MAGNETOMETER

QMC5883L capt_magnetometer(PB_14, PB_13);
#endif

// Default values at the init
static int16_t magnetic_field[3] = {}; // x, y, z

uint8_t ChipID;  //CHECKS QMC chip id status

/*threshold initilization and button assignment*/
static int16_t threshold_values[3] = {}; //tx,ty,tz threshold values

InterruptIn button(USER_BUTTON);

/******************************************************************************************************************************/
/* Reduce it to 2000 if your application is real time
 * Sets up an application dependent transmission timer in ms.
 *********************************************************************************************************************************/
#define TX_TIMER                        5000

 /**
  * Maximum number of events for the event queue.
  * 10 is the safe number for the stack events, however, if application
  * also uses the queue for whatever purposes, this number should be increased.
  */
#define MAX_NUMBER_OF_EVENTS            10

  /**
   * Maximum number of retries for CONFIRMED messages before giving up
   */
#define CONFIRMED_MSG_RETRY_COUNTER     3

   /**
    * Dummy pin for dummy sensor
    */
#define PC_9                            0

    /**
     * Dummy sensor class object
     */

DS1820  ds1820(PC_9);

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application.
*/

static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS* EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */

static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

/**
 * Threshold logic after button pressed
 * It's an Interrupt base service routine funcion call.
 * Whenever we press threshold button current magnetic values of sensor(X,Y,Z)
 * gets store in threshold values array.
 * usually we require only Z value but stored all values in case if we change algorithm.
 */

void pressed()
{

    printf("\r\n pressed \r\n");
    threshold_values[0] = magnetic_field[0];
    threshold_values[1] = magnetic_field[1];
    threshold_values[2] = magnetic_field[2];
    printf("QMC Magnetic field: x: %hd, y: %hd, z: %hd\r\n", threshold_values[0], threshold_values[1], threshold_values[2]);

}


/**
 * Entry point for application
 */


int main(void)
{
    // setup tracing
    setup_trace();

    /****************************************************************************************
    * Initilize the QMC5883L sensor
    * check CHIP ID
    * This register is chip identification register. It returns 0xFF
    *****************************************************************************************/

#ifdef ENABLE_MAGNETOMETER
    capt_magnetometer.init();
#endif
    ChipID = capt_magnetometer.ChipID();
    printf("I AM QMC5883: 0x%x \r\n", ChipID);
    wait(1);

    /****************************************************************************************
     * retcode-->stores the status of a call to LoRaWAN protocol
     * if any issue you find with lora stack always check retcode
     * to know issue
     ****************************************************************************************/

    lorawan_status_t retcode;

    /****************************************************************************************
    * Initialize LoRaWAN stack
    *****************************************************************************************/
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");
    /****************************************************************************************
    * prepare application callbacks, if any callback operation has to follow
    *****************************************************************************************/

    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    /****************************************************************************************
    * Set number of retries in case of CONFIRMED messages
    *****************************************************************************************/

    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
        != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n",
        CONFIRMED_MSG_RETRY_COUNTER);

    /****************************************************************************************
    * Enable adaptive data rate
    *****************************************************************************************/

    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");

    /****************************************************************************************
    * sent lorawan connect request
    * check retcode to see issue
    * if no issue lorawan connection happes successfully
    *****************************************************************************************/

    retcode = lorawan.connect();

    if (retcode == LORAWAN_STATUS_OK ||
        retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    }
    else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    /****************************************************************************************
    * make your event queue dispatching events forever, OS queue formed
    *****************************************************************************************/

    ev_queue.dispatch_forever();

    return 0;
}


/********************************************************************************************************************************************/


/****************************************************************************
 * Sends a message to the Network Server
 * As it's whole program we write using Mbed operating system
 * event queue matters, we are processing most of code under this event
 * CALLING of this function happens using event handler
 ****************************************************************************/

static void send_message()
{
    uint16_t packet_len;                //packet_len operator
    int16_t retcode;                    //retcode checks issue
    int32_t sensor_value;               //dummy sensor value store

    button.fall(&pressed);             //THRESHOLD button interrupt function call

   /*****************MLX90393 PROGRAMMING**********************************/

    char config[4];
    char data[8] = { 0 };


    config[0] = 0x60;
    config[1] = 0x00;
    config[2] = 0x5C;
    config[3] = 0x00;

    i2c.write(addr, config, 4, false);

    i2c.read(addr, data, 1);

    config[0] = 0x60;
    config[1] = 0x02;
    config[2] = 0xC4;
    config[3] = 0x02;

    i2c.write(addr, config, 4, false);

    i2c.read(addr, data, 2);

    wait(0.25);
    /***********************************************************************************************
    The single measurement command is used to instruct the MLX90393 to perform an acquisition cycle.
    It consists of a single byte with four user-defined bits: z, y, x, and t.
    These four bits determine which axes will be converted whenever they are set to a 1.
    *************************************************************************************************/
    config[0] = 0x3E; // Single measurement mode, ZYX enabled and also measure temperature

    i2c.write(addr, config, 1, false);
    i2c.read(addr, data, 2);

    wait(0.1);
    /********************************************************************************************************
    The Read Measurement command is used to retrieve the data previously acquired by the SM or SB command.
    Similar to these commands, the RM command can also select which data is transmitted.
    The data is output in the following order:
    T (MSB), T (LSB), X (MSB), X (LSB), Y (MSB), Y (LSB), Z (MSB), Z (LSB)
    ********************************************************************************************************/
    config[0] = 0x4E;

    i2c.write(addr, config, 1, false); // Read command, followed by ZYX bits set
    i2c.read(addr, data, 7);

    int xMag = ((data[1] * 256) + data[2]);
    int yMag = ((data[3] * 256) + data[4]);
    int zMag = ((data[5] * 256) + data[6]);
    int temp1 = data[7];
    /*
            printf("\r\n MLX X Axis = %d,Y Axis = %d, Z Axis = %d", xMag,yMag,zMag);
            printf("\r\nY Axis = %d", yMag);
            printf("\r\nZ Axis = %d", zMag);
            printf("\r\n-----------------------------------------------------------------------------");
            wait(0.5);
      */

      /*************************************************************************************************
      ds1820 dummy sensor code
      **************************************************************************************************/
    if (ds1820.begin()) {
        ds1820.startConversion();
        sensor_value = ds1820.read();
        //        printf("\r\n Dummy Sensor Value = %d \r\n", sensor_value);
        ds1820.startConversion();
    }
    else {
        //       printf("\r\n No sensor found \r\n");
        return;
    }

    //-------------------------------------QMC5883L-----------------------------------------------------------------------

#ifdef ENABLE_MAGNETOMETER
        // Magnetometer init
    capt_magnetometer.init();
    magnetic_field[0] = capt_magnetometer.getMagXvalue();
    magnetic_field[1] = capt_magnetometer.getMagYvalue();
    magnetic_field[2] = capt_magnetometer.getMagZvalue();
    capt_magnetometer.standby();
#endif

    printf("QMC threshold values: x: %hd, y: %hd, z: %hd\r\n", threshold_values[0], threshold_values[1], threshold_values[2]);
    /*form packet to send*/
    packet_len = sprintf((char*)tx_buffer, "{\"X\":%hd,\"Y\":%hd,\"Z\":%hd,\"status\":%d}", magnetic_field[0], magnetic_field[1], magnetic_field[2], status);

    printf("%s\n", tx_buffer);
   // retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
   //     MSG_UNCONFIRMED_FLAG);
    


    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
            : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(3000, send_message);
            }
        }
        return;
    }

    //1   printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(rx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 * Downlink message
 */
static void receive_message()
{

    uint8_t port;    //port from we receive data
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);    //check data valid using retcode

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);

    /***********************************************************************************
        read downlink message
    ************************************************************************************/
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }
    printf("\r\n");

    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 * After each succesfully event i.e. sent/receive/connected/disconnection/error
 * mbed os will call this function to check event
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
    case CONNECTED:
        printf("\r\n Connection - Successful \r\n");
        if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
            send_message();
        }
        else {
            ev_queue.call_every(TX_TIMER, send_message);
        }

        break;
    case DISCONNECTED:
        ev_queue.break_dispatch();
        printf("\r\n Disconnected Successfully \r\n");
        break;
    case TX_DONE:
        //1           printf("\r\n Message Sent to Network Server \r\n");
        if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
            send_message();
        }
        break;
    case TX_TIMEOUT:
    case TX_ERROR:
    case TX_CRYPTO_ERROR:
    case TX_SCHEDULING_ERROR:
        printf("\r\n Transmission Error - EventCode = %d \r\n", event);
        // try again
        if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
            send_message();
        }
        break;
    case RX_DONE:
        printf("\r\n Received message from Network Server \r\n");
        receive_message();
        break;
    case RX_TIMEOUT:
    case RX_ERROR:
        printf("\r\n Error in reception - Code = %d \r\n", event);
        break;
    case JOIN_FAILURE:
        printf("\r\n OTAA Failed - Check Keys \r\n");
        break;
    case UPLINK_REQUIRED:
        printf("\r\n Uplink required by NS \r\n");
        if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
            send_message();
        }
        break;
    default:
        MBED_ASSERT("Unknown Event");
    }
}

// EOF
