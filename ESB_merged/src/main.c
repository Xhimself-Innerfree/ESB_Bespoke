/*
 * Flavin Neuromachines Lab  https://flavinlab.io/
 *
 * JL, 2025 July 12
 */
#include <stdio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <dk_buttons_and_leds.h>
#if defined(CONFIG_CLOCK_CONTROL_NRF2)
#include <hal/nrf_lrcconf.h>
#endif
#include <zephyr/drivers/uart.h> /* include the package needed for UART*/

/* Define the size of the receive buffer for UART*/
#define RECEIVE_BUFF_SIZE 10
/* Define the receiving timeout period for UART*/
#define RECEIVE_TIMEOUT 100
/* UART Parition*/
const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0)); /* Get the device pointer of the UART hardware */
/* Define the receive buffer */
static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};
static uint8_t tx_buf[] = {"working\r\n"};
/* Get the device pointers of the LEDs through gpio_dt_spec */
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
/* ESB parition*/
LOG_MODULE_REGISTER(esb_ptx, CONFIG_ESB_PTX_APP_LOG_LEVEL);

static bool ready = true;
bool send_flag = false; // Flag to indicate if the payload should be sent
static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
														  0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08);
static struct esb_payload tx_payload_ptx_end = ESB_CREATE_PAYLOAD(0,
																  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08);
;

static struct esb_payload rx_payload_prx;
static struct esb_payload tx_payload_prx = ESB_CREATE_PAYLOAD(0,
															  0x01, 0x01, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08);

#define _RADIO_SHORTS_COMMON                                       \
	(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | \
	 RADIO_SHORTS_ADDRESS_RSSISTART_Msk |                          \
	 RADIO_SHORTS_DISABLED_RSSISTOP_Msk)

void event_handler(struct esb_evt const *event)
{
	ready = true;

	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT");
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		break;
	case ESB_EVENT_RX_RECEIVED:
		while (esb_read_rx_payload(&rx_payload) == 0)
		{
			LOG_DBG("Packet received, len %d : "
					"0x%02x, 0x%02x, 0x%02x, 0x%02x, "
					"0x%02x, 0x%02x, 0x%02x, 0x%02x",
					rx_payload.length, rx_payload.data[0],
					rx_payload.data[1], rx_payload.data[2],
					rx_payload.data[3], rx_payload.data[4],
					rx_payload.data[5], rx_payload.data[6],
					rx_payload.data[7]);
			// Update the status with the payload from tx. JL
			if (rx_payload.data[1] == 1)
			{
				gpio_pin_toggle_dt(&led0);
			}
		}
		break;
	}
}

void event_handler_prx(struct esb_evt const *event)
{
	ready = true;

	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT/PRX");
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		break;
	case ESB_EVENT_RX_RECEIVED:
		while (esb_read_rx_payload(&rx_payload_prx) == 0)
		{
			LOG_DBG("Packet received, len %d : "
					"0x%02x, 0x%02x, 0x%02x, 0x%02x, "
					"0x%02x, 0x%02x, 0x%02x, 0x%02x",
					rx_payload_prx.length, rx_payload_prx.data[0],
					rx_payload_prx.data[1], rx_payload_prx.data[2],
					rx_payload_prx.data[3], rx_payload_prx.data[4],
					rx_payload_prx.data[5], rx_payload_prx.data[6],
					rx_payload_prx.data[7]);
			// Update the status with the payload from tx. JL
			if (rx_payload_prx.data[1] == 1)
			{
				gpio_pin_toggle_dt(&led0);
			}
		}
		break;
	}
}

#if defined(CONFIG_CLOCK_CONTROL_NRF)
int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr)
	{
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0)
	{
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do
	{
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res)
		{
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	LOG_DBG("HF clock started");
	return 0;
}

#elif defined(CONFIG_CLOCK_CONTROL_NRF2)

int clocks_start(void)
{
	int err;
	int res;
	const struct device *radio_clk_dev =
		DEVICE_DT_GET_OR_NULL(DT_CLOCKS_CTLR(DT_NODELABEL(radio)));
	struct onoff_client radio_cli;

	/** Keep radio domain powered all the time to reduce latency. */
	nrf_lrcconf_poweron_force_set(NRF_LRCCONF010, NRF_LRCCONF_POWER_DOMAIN_1, true);

	sys_notify_init_spinwait(&radio_cli.notify);

	err = nrf_clock_control_request(radio_clk_dev, NULL, &radio_cli);

	do
	{
		err = sys_notify_fetch_result(&radio_cli.notify, &res);
		if (!err && res)
		{
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err == -EAGAIN);

	nrf_lrcconf_clock_always_run_force_set(NRF_LRCCONF000, 0, true);
	nrf_lrcconf_task_trigger(NRF_LRCCONF000, NRF_LRCCONF_TASK_CLKSTART_0);

	LOG_DBG("HF clock started");
	return 0;
}

#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif /* defined(CONFIG_CLOCK_CONTROL_NRF2) */

int esb_initialize(void)
{
	int err; // ptx
	/* addresses used to send packets*/
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.retransmit_delay = 600;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.event_handler = event_handler;
	config.mode = ESB_MODE_PTX;
	config.selective_auto_ack = true;
	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING))
	{
		config.use_fast_ramp_up = true;
	}

	err = esb_init(&config);
	if (err)
	{
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err)
	{
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err)
	{
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err)
	{
		return err;
	}

	return 0;
}

int esb_initialize_prx(void)
{
	int err;
	/* addresses used to receive packets*/
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE8, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler_prx;
	config.selective_auto_ack = true;
	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING))
	{
		config.use_fast_ramp_up = true;
	}

	err = esb_init(&config);
	if (err)
	{
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err)
	{
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err)
	{
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err)
	{
		return err;
	}

	return 0;
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type)
	{
	case UART_RX_RDY:
		if ((evt->data.rx.len) == 1)
		{
			if (evt->data.rx.buf[evt->data.rx.offset] == '1')
			{
				gpio_pin_toggle_dt(&led0);
				send_flag = true; // Set the flag to indicate that the payload should be sent
				LOG_DBG("LED toggled, sending payload");
			}
			break;
		}
	case UART_RX_DISABLED:
		uart_rx_enable(dev, rx_buf, sizeof rx_buf, RECEIVE_TIMEOUT);
		break;
	default:
		break;
	}
}

int main(void)
{
	int err;

	LOG_INF("Enhanced ShockBurst ptx sample");

	if (!device_is_ready(uart))
	{
		printk("UART device not ready\r\n");
		return 1;
	}

	if (!device_is_ready(led0.port))
	{
		printk("GPIO device is not ready\r\n");
		return 1;
	}

	err = clocks_start();
	if (err)
	{
		return 0;
	}

	err = dk_leds_init();
	if (err)
	{
		LOG_ERR("LEDs initialization failed, err %d", err);
		return 0;
	}

	err = esb_initialize_prx();
	if (err)
	{
		LOG_ERR("ESB initialization failed, err %d", err);
		return 0;
	}
	esb_write_payload(&tx_payload_prx);
	err = esb_start_rx();

	/*
	esb_stop_rx();
	esb_initialize();
	tx_payload.noack = false;
	esb_flush_tx();
	esb_write_payload(&tx_payload);
	esb_initialize_prx();
	esb_start_rx();
	*/

	LOG_INF("Initialization complete");
	LOG_INF("Sending test packet");
	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
	err = uart_callback_set(uart, uart_cb, NULL);
	if (err)
	{
		return 1;
	}

	err = uart_rx_enable(uart, rx_buf, sizeof rx_buf, RECEIVE_TIMEOUT);
	if (err)
	{
		return 1;
	}

	tx_payload.noack = false; // this line means that the payload will be acknowledged by the receiver
	tx_payload.pid = 0x01;	  // Set the PID for the payload
	while (1)
	{

		if (send_flag) //(ready && send_flag) // Check if ready and the flag is set
		{
			send_flag = false; // Reset the flag after sending

			esb_stop_rx();
			esb_initialize();
			tx_payload.noack = false;

			esb_flush_tx(); // This function clears the TX FIFO buffer.
			esb_write_payload(&tx_payload);
			k_sleep(K_MSEC(1)); // Small delay to ensure the payload is sent before the next iteration

			esb_flush_tx(); // This function clears the TX FIFO buffer.
			esb_write_payload(&tx_payload_ptx_end);
			k_sleep(K_MSEC(1)); // Small delay to ensure the payload is sent before the next iteration

			esb_initialize_prx();
			esb_write_payload(&tx_payload_prx);
			esb_start_rx();
		}
		k_sleep(K_MSEC(1000));
	}
}
