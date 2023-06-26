#include <stdio.h>
#include <string.h>

#include "xtimer.h"
#include "thread.h"
#include "periph/i2c.h"

#include "lsm6ds3.h"
#include "lsm6ds3_regs.h"

#include "cayenne_lpp.h"

#include "sx127x.h"
#include "sx127x_netdev.h"
#include "sx127x_params.h"
#include "net/loramac.h"     /* core loramac definitions */
#include "semtech_loramac.h" /* package API */

#define BUTTON1_PIN GPIO_PIN(PORT_B, 9)
#define BUTTON2_PIN GPIO_PIN(PORT_B, 8)
#define BUTTON3_PIN GPIO_PIN(PORT_A, 5)
#define BUTTON4_PIN GPIO_PIN(PORT_A, 4)

#define SECRET_LENGTH 5

typedef enum {
    APP_MSG_SEND,
} app_message_types_t;

typedef struct {
    uint8_t *buffer;
    uint32_t length;
} lora_data_t;

static msg_t recv_queue[4];
static msg_t send_queue[4];

static semtech_loramac_t loramac;  /* The loramac stack device descriptor */
static sx127x_t sx127x;            /* SX127x device descriptor            */
static kernel_pid_t sender_pid;    /* Pid of a thread which controls data sending */
static kernel_pid_t loramac_pid;   /* Pid of a thread which controls LoRaWAN stack */

/* define the required keys for activation */
static const uint8_t deveui[LORAMAC_DEVEUI_LEN] = { 0x80, 0x7b, 0x85, 0x90, 0x20, 0x00, 0x06, 0x1e };
static const uint8_t appeui[LORAMAC_APPEUI_LEN] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* we need AppKey for OTAA */
static const uint8_t appkey[LORAMAC_APPKEY_LEN] = { 0xfe, 0x49, 0x1b, 0xec, 0x6f, 0x57, 0xfd, 0xa3, 0xaa,
							0x62, 0xbd, 0x05, 0xe8, 0xbc, 0x19, 0x66 };

static lsm6ds3_t lsm6ds3;
static const lsm6ds3_param_t lsm6ds3_params = {
    .i2c = I2C_DEV(1),
    .i2c_addr = 0x6A,
    .gyro_enabled = true,
    .gyro_range = LSM6DS3_ACC_GYRO_FS_G_500dps,
    .gyro_sample_rate = LSM6DS3_ACC_GYRO_ODR_XL_1660Hz,
    .gyro_bandwidth = LSM6DS3_ACC_GYRO_BW_XL_400Hz,
    .gyro_fifo_enabled = true,
    .gyro_fifo_decimation = true,
    .accel_enabled = true,
    .accel_odr_off = true,
    .accel_range = LSM6DS3_ACC_GYRO_FS_XL_16g,
    .accel_sample_rate = LSM6DS3_ACC_GYRO_ODR_XL_1660Hz,
    .accel_bandwidth = LSM6DS3_ACC_GYRO_BW_XL_400Hz,
    .accel_fifo_enabled = true,
    .accel_fifo_decimation = true,
    .temp_enabled = true,
    .comm_mode = 1,
    .fifo_threshold = 3000,
    .fifo_sample_rate = LSM6DS3_ACC_GYRO_ODR_FIFO_1600Hz,
    .fifo_mode_word = 0,
};

static char recv_stack[THREAD_STACKSIZE_DEFAULT];
static char lsm6ds3_stack[THREAD_STACKSIZE_DEFAULT];
static cayenne_lpp_t lsm6ds3_lpp;

static void *lsm6ds3_thread(void *arg) {
    (void) arg;
    static msg_t data_msg = { .type = APP_MSG_SEND };
    static lora_data_t data;

    lsm6ds3.params = lsm6ds3_params;

    if (lsm6ds3_init(&lsm6ds3) == 0) {
        puts("\rLSM6DS3 succesfully initialised!\r\n");

        lsm6ds3_poweron(&lsm6ds3);
        lsm6ds3_data_t lsm6ds3_data;
        lsm6ds3_read_acc(&lsm6ds3, &lsm6ds3_data);
        lsm6ds3_poweroff(&lsm6ds3);

        int16_t x = lsm6ds3_data.acc_x;
        int16_t y = lsm6ds3_data.acc_y;
        int16_t z = lsm6ds3_data.acc_z;

        int16_t x1, y1, z1;

        int counter = 0;

        while(1) {
            xtimer_ticks32_t start_time = xtimer_now() / 1000000;

            while(1) {
                lsm6ds3_poweron(&lsm6ds3);
                lsm6ds3_read_acc(&lsm6ds3, &lsm6ds3_data);
                lsm6ds3_poweroff(&lsm6ds3);

                x1 = lsm6ds3_data.acc_x;
                y1 = lsm6ds3_data.acc_y;
                z1 = lsm6ds3_data.acc_z;

                printf("\rcounter = %i\r\n", counter);

                if ((abs(x1 - x) > 100) || (abs(y1 - y) > 100) || (abs(z1 - z) > 100)) {
                    printf("\rHit!\r\n");
                    counter++;
                    xtimer_msleep(1000);
                }
                else {
                    printf("\rNothing happened\r\n");
                }

                x = x1;
                y = y1;
                z = z1;

                xtimer_msleep(100);

                xtimer_ticks32_t finish_time = xtimer_now() / 1000000;

                if (finish_time - start_time >= 60) {
                    puts("minute is up!");
                    break;
                }
            }

            /* Put data into Cayenne LPP buffer */
            cayenne_lpp_reset(&lsm6ds3_lpp);
            cayenne_lpp_add_digital_output(&lsm6ds3_lpp, 1, (int) (counter));

            /* Send a message to a LoRaWAN thread */
            data.buffer = lsm6ds3_lpp.buffer;
            data.length = lsm6ds3_lpp.cursor;
            data_msg.content.ptr = &data;
            msg_send(&data_msg, sender_pid);
        }
    }

    return NULL;
}

static void *recv_thread(void *arg) {
    (void) arg;
    int res;
    msg_init_queue(recv_queue, 4);

    while (1) {
        /* blocks until some data is received */
        res = semtech_loramac_recv(&loramac);

        switch (res) {
        case SEMTECH_LORAMAC_RX_DATA:
            printf("[LoRa] Data received: %d bytes, port %d\n",
                            loramac.rx_data.payload_len,
                            loramac.rx_data.port);

            printf("[LoRa] Hex data: ");
            for (int l = 0; l < loramac.rx_data.payload_len; l++) {
                printf("%02X ", loramac.rx_data.payload[l]);
            }
            printf("\n");

            /* TODO: process data here */
            break;

        case SEMTECH_LORAMAC_RX_CONFIRMED:
            printf("[LoRa] Acknowledgement received\n");
            break;
        }
    }

    return NULL;
}

int main(void){
    int res;
    msg_t msg;

    gpio_init(BUTTON1_PIN, GPIO_IN);
    gpio_init(BUTTON2_PIN, GPIO_IN);
    gpio_init(BUTTON3_PIN, GPIO_IN);
    gpio_init(BUTTON4_PIN, GPIO_IN);
    gpio_init(LED0_PIN, GPIO_OUT);

    int x = 1;
    gpio_write(LED0_PIN, x);
    int secret[SECRET_LENGTH] = {2, 1, 3, 4, 4};
    int lastPressed = 0;
    int counter_pass = 0;
    gpio_clear(LED0_PIN);

    int Auth_Success = 0;

    if (Auth_Success == 0) {
        while(1) {
		if(gpio_read(BUTTON1_PIN) == 0) {
		    puts("Button 1 pressed\r\n");
		    xtimer_msleep(200);
		    lastPressed = 1;
		    if (lastPressed == secret[counter_pass]) {
		    	counter_pass += 1;
		    }
		    else {
		    	counter_pass = 0;
		    }
		}
		if(gpio_read(BUTTON2_PIN) == 0) {
		    puts("Button 2 pressed\r\n");
		    xtimer_msleep(200);
		    lastPressed = 2;
		    if (lastPressed == secret[counter_pass]) {
		    	counter_pass += 1;
		    }
		    else {
		    	counter_pass = 0;
		    }
		}
		if(gpio_read(BUTTON3_PIN) == 0) {
		    puts("Button 3 pressed\r\n");
		    xtimer_msleep(200);
		    lastPressed = 3;
		    if (lastPressed == secret[counter_pass]) {
		    	counter_pass += 1;
		    }
		    else {
		    	counter_pass = 0;
		    }
		}
		if(gpio_read(BUTTON4_PIN) == 0) {
		    puts("Button 4 pressed\r\n");
		    xtimer_msleep(200);
		    lastPressed = 4;
		    if (lastPressed == secret[counter_pass]) {
		    	counter_pass += 1;
		    }
		    else {
		    	counter_pass = 0;
		    }
		}
		if (counter_pass == SECRET_LENGTH) {
		    counter_pass = 0;
		    puts("Code is correct\r\n");
		    Auth_Success = 1;
		    printf("Device is unlocked\r\n");
		    for (int i = 0; i < 50; ++i) {
		    	x = !x;
		    	gpio_write(LED0_PIN, x);
		    	xtimer_msleep(100);
		    }

		    xtimer_msleep(50);
		    break;
		}
		xtimer_msleep(50);
		}
    }

    if (Auth_Success == 1) {

	    /* Adjust message queue size */
	    msg_init_queue(send_queue, 4);

	    /* Setup the transceiver and the LoRaWAN stack */
	    sx127x_setup(&sx127x, &sx127x_params[0], 0);
	    loramac.netdev = &sx127x.netdev;
	    loramac.netdev->driver = &sx127x_driver;
	    loramac_pid = semtech_loramac_init(&loramac);
	    sender_pid = thread_getpid();

	    /* Set the keys identifying the device */
	    semtech_loramac_set_deveui(&loramac, deveui);
	    semtech_loramac_set_appeui(&loramac, appeui);
	    semtech_loramac_set_appkey(&loramac, appkey);

	    /* Additional LoRaWAN setup */
	    semtech_loramac_set_tx_power(&loramac, -5);
	    semtech_loramac_set_dr(&loramac, LORAMAC_DR_1);
	    semtech_loramac_set_tx_mode(&loramac, LORAMAC_TX_CNF);   /* confirmed packets */
	    semtech_loramac_set_tx_port(&loramac, 2); /* port 2 */
	    semtech_loramac_set_class(&loramac, LORAMAC_CLASS_C); /* Always listen */

	    puts("\r[LoRa] LoRaMAC initialised\r\n");

	    do{
		res = semtech_loramac_join(&loramac, LORAMAC_JOIN_OTAA);

		switch (res) {
		case SEMTECH_LORAMAC_JOIN_SUCCEEDED:
		    puts("\r[LoRa] successfully joined to the network\r\n");
		    break;

		case SEMTECH_LORAMAC_ALREADY_JOINED:
		    /* ignore, can't be */
		    break;

		case SEMTECH_LORAMAC_BUSY:
		case SEMTECH_LORAMAC_NOT_JOINED:
		case SEMTECH_LORAMAC_JOIN_FAILED:
		case SEMTECH_LORAMAC_DUTYCYCLE_RESTRICTED:
		    printf("[LoRa] LoRaMAC join failed: code %d\n", res);
		    xtimer_sleep(10);
		    break;

		default:
		    printf("[LoRa] join request: unknown response %d\n", res);
		    break;
		}
	    } while (res != SEMTECH_LORAMAC_JOIN_SUCCEEDED);

	    /* Start another thread for sensor measurements */
	    thread_create(lsm6ds3_stack, sizeof(lsm6ds3_stack),
		      THREAD_PRIORITY_MAIN-1,
		      THREAD_CREATE_STACKTEST,
		      lsm6ds3_thread,
		      NULL,
		      "Sensor thread");

	    /* Start another thread for data receiving */
	    thread_create(recv_stack, sizeof(recv_stack),
		    THREAD_PRIORITY_MAIN-2,
		    THREAD_CREATE_STACKTEST,
		    recv_thread,
		    NULL,
		    "Receiving thread");

	    while (1) {
		msg_receive(&msg);

		/* Application message */
		if (msg.type == APP_MSG_SEND) {
		    lora_data_t *data = msg.content.ptr;
		    (void) data;
		    res = semtech_loramac_send(&loramac, data->buffer, data->length);

		    switch (res) {
		    case SEMTECH_LORAMAC_TX_DONE:
		        puts("\r[LoRa] TX done\r\n");
		        break;
		    case SEMTECH_LORAMAC_TX_CNF_FAILED:
		        puts("\r[LoRa] Uplink confirmation failed\r\n");
		        /* TODO: rejoin if there are too many failures */
		        break;
		    case SEMTECH_LORAMAC_BUSY:
		        puts("\r[LoRa] MAC already busy\r\n");
		        break;
		    case SEMTECH_LORAMAC_NOT_JOINED:
		        puts("\r[LoRa] Not joined to the network\r\n");
		        break;
		    case SEMTECH_LORAMAC_TX_OK:
		        puts("\r[LoRa] TX is in progress\r\n");
		        break;
		    case SEMTECH_LORAMAC_DUTYCYCLE_RESTRICTED:
		        puts("\r[LoRa] TX duty cycle restricted\r\n");
		        break;
		    default:
		        printf("\r[LoRa] Unknown send() response %d\r\n", res);
		        break;
		    }
		}
	    }
    }
}
