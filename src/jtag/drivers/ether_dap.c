#include <transport/transport.h>
#include "helper/replacements.h"
#include <jtag/adapter.h>
#include <jtag/swd.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <jtag/tcl.h>
#include <target/cortex_m.h>

#include "ether_dap.h"
#include "libusb_helper.h"


static struct ether_dap *ether_dap_handle;

/**
 * @defgroup misc_func Miscellaneous func
 */

static bool calculate_swo_prescaler(unsigned int traceclkin_freq,
		uint32_t trace_freq, uint16_t *prescaler)
{
	unsigned int presc = (traceclkin_freq + trace_freq / 2) / trace_freq;
	if (presc == 0 || presc > TPIU_ACPR_MAX_SWOSCALER + 1)
		return false;

	/* Probe's UART speed must be within 3% of the TPIU's SWO baud rate. */
	unsigned int max_deviation = (traceclkin_freq * 3) / 100;
	if (presc * trace_freq < traceclkin_freq - max_deviation ||
	    presc * trace_freq > traceclkin_freq + max_deviation)
		return false;

	*prescaler = presc;

	return true;
}

/**
 * @}
 */


/**
 * @defgroup swd_func SWD Related Func
 */

static int cmsis_dap_swd_init(void)
{
	swd_mode = true;
	return ERROR_OK;
}

static int cmsis_dap_swd_switch_seq(enum swd_special_seq seq)
{
	const uint8_t *s;
	unsigned int s_len;
	int retval;

	if (swd_mode)
		queued_retval = cmsis_dap_swd_run_queue();

	if (cmsis_dap_handle->quirk_mode && seq != LINE_RESET &&
			(output_pins & (SWJ_PIN_SRST | SWJ_PIN_TRST))
				== (SWJ_PIN_SRST | SWJ_PIN_TRST)) {
		/* Following workaround deasserts reset on most adapters.
		 * Do not reconnect if a reset line is active!
		 * Reconnecting would break connecting under reset. */

		/* First disconnect before connecting, Atmel EDBG needs it for SAMD/R/L/C */
		cmsis_dap_cmd_dap_disconnect();

		/* When we are reconnecting, DAP_Connect needs to be rerun, at
		 * least on Keil ULINK-ME */
		retval = cmsis_dap_cmd_dap_connect(CONNECT_SWD);
		if (retval != ERROR_OK)
			return retval;
	}

	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG_IO("SWD line reset");
		s = swd_seq_line_reset;
		s_len = swd_seq_line_reset_len;
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		s = swd_seq_jtag_to_swd;
		s_len = swd_seq_jtag_to_swd_len;
		break;
	case JTAG_TO_DORMANT:
		LOG_DEBUG("JTAG-to-DORMANT");
		s = swd_seq_jtag_to_dormant;
		s_len = swd_seq_jtag_to_dormant_len;
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		s = swd_seq_swd_to_jtag;
		s_len = swd_seq_swd_to_jtag_len;
		break;
	case SWD_TO_DORMANT:
		LOG_DEBUG("SWD-to-DORMANT");
		s = swd_seq_swd_to_dormant;
		s_len = swd_seq_swd_to_dormant_len;
		break;
	case DORMANT_TO_SWD:
		LOG_DEBUG("DORMANT-to-SWD");
		s = swd_seq_dormant_to_swd;
		s_len = swd_seq_dormant_to_swd_len;
		break;
	case DORMANT_TO_JTAG:
		LOG_DEBUG("DORMANT-to-JTAG");
		s = swd_seq_dormant_to_jtag;
		s_len = swd_seq_dormant_to_jtag_len;
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	retval = cmsis_dap_cmd_dap_swj_sequence(s_len, s);
	if (retval != ERROR_OK)
		return retval;

	/* Atmel EDBG needs renew clock setting after SWJ_Sequence
	 * otherwise default frequency is used */
	return cmsis_dap_cmd_dap_swj_clock(adapter_get_speed_khz());
}

static void cmsis_dap_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RNW));
	cmsis_dap_swd_queue_cmd(cmd, NULL, value);
}

static void cmsis_dap_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RNW);
	cmsis_dap_swd_queue_cmd(cmd, value, 0);
}

static int cmsis_dap_swd_run_queue(void)
{
	if (cmsis_dap_handle->write_count + cmsis_dap_handle->read_count) {
		if (cmsis_dap_handle->pending_fifo_block_count)
			cmsis_dap_swd_read_process(cmsis_dap_handle, CMSIS_DAP_NON_BLOCKING);

		cmsis_dap_swd_write_from_queue(cmsis_dap_handle);
	}

	while (cmsis_dap_handle->pending_fifo_block_count)
		cmsis_dap_swd_read_process(cmsis_dap_handle, CMSIS_DAP_BLOCKING);

	cmsis_dap_handle->pending_fifo_put_idx = 0;
	cmsis_dap_handle->pending_fifo_get_idx = 0;

	int retval = queued_retval;
	queued_retval = ERROR_OK;

	return retval;
}

/**
 * @}
 */



/**
 * @defgroup adapter_drv Adapter Driver Func
 * @{
 */

static int cmsis_dap_init(void)
{
	uint8_t *data;

	int retval = cmsis_dap_open();
	if (retval != ERROR_OK)
		return retval;

	cmsis_dap_flush_read(cmsis_dap_handle);

	retval = cmsis_dap_get_caps_info();
	if (retval != ERROR_OK)
		return retval;

	retval = cmsis_dap_get_version_info();
	if (retval != ERROR_OK)
		return retval;

	retval = cmsis_dap_get_serial_info();
	if (retval != ERROR_OK)
		return retval;

	if (swd_mode) {
		retval = cmsis_dap_swd_open();
		if (retval != ERROR_OK)
			return retval;
	} else {
		/* Connect in JTAG mode */
		if (!(cmsis_dap_handle->caps & INFO_CAPS_JTAG)) {
			LOG_ERROR("CMSIS-DAP: JTAG not supported");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		retval = cmsis_dap_cmd_dap_connect(CONNECT_JTAG);
		if (retval != ERROR_OK)
			return retval;

		LOG_INFO("CMSIS-DAP: Interface Initialised (JTAG)");
	}

	/* Be conservative and suppress submitting multiple HID requests
	 * until we get packet count info from the adaptor */
	cmsis_dap_handle->packet_count = 1;

	/* INFO_ID_PKT_SZ - short */
	retval = cmsis_dap_cmd_dap_info(INFO_ID_PKT_SZ, &data);
	if (retval != ERROR_OK)
		goto init_err;

	if (data[0] == 2) {  /* short */
		uint16_t pkt_sz = data[1] + (data[2] << 8);
		if (pkt_sz != cmsis_dap_handle->packet_size) {
			cmsis_dap_handle->backend->packet_buffer_free(cmsis_dap_handle);
			retval = cmsis_dap_handle->backend->packet_buffer_alloc(cmsis_dap_handle, pkt_sz);
			if (retval != ERROR_OK)
				goto init_err;

			LOG_DEBUG("CMSIS-DAP: Packet Size = %" PRIu16, pkt_sz);
		}
	}

	/* Maximal number of transfers which fit to one packet:
	 * Limited by response size: 3 bytes of response header + 4 per read
	 * Plus writes to full command size: 3 bytes cmd header + 1 per read + 5 per write */
	tfer_max_command_size = cmsis_dap_handle->packet_usable_size;
	tfer_max_response_size = cmsis_dap_handle->packet_usable_size;
	unsigned int max_reads = tfer_max_response_size / 4;
	pending_queue_len = max_reads + (tfer_max_command_size - max_reads) / 5;
	cmsis_dap_handle->write_count = 0;
	cmsis_dap_handle->read_count = 0;

	/* INFO_ID_PKT_CNT - byte */
	retval = cmsis_dap_cmd_dap_info(INFO_ID_PKT_CNT, &data);
	if (retval != ERROR_OK)
		goto init_err;

	if (data[0] == 1) { /* byte */
		unsigned int pkt_cnt = data[1];
		if (pkt_cnt > 1)
			cmsis_dap_handle->packet_count = MIN(MAX_PENDING_REQUESTS, pkt_cnt);

		LOG_DEBUG("CMSIS-DAP: Packet Count = %u", pkt_cnt);
	}

	LOG_DEBUG("Allocating FIFO for %u pending packets", cmsis_dap_handle->packet_count);
	for (unsigned int i = 0; i < cmsis_dap_handle->packet_count; i++) {
		cmsis_dap_handle->pending_fifo[i].transfers = malloc(pending_queue_len
									 * sizeof(struct pending_transfer_result));
		if (!cmsis_dap_handle->pending_fifo[i].transfers) {
			LOG_ERROR("Unable to allocate memory for CMSIS-DAP queue");
			retval = ERROR_FAIL;
			goto init_err;
		}
	}

	/* Intentionally not checked for error, just logs an info message
	 * not vital for further debugging */
	(void)cmsis_dap_get_status();

	/* Now try to connect to the target
	 * TODO: This is all SWD only @ present */
	retval = cmsis_dap_cmd_dap_swj_clock(adapter_get_speed_khz());
	if (retval != ERROR_OK)
		goto init_err;

	/* Ask CMSIS-DAP to automatically retry on receiving WAIT for
	 * up to 64 times. This must be changed to 0 if sticky
	 * overrun detection is enabled. */
	retval = cmsis_dap_cmd_dap_tfer_configure(0, 64, 0);
	if (retval != ERROR_OK)
		goto init_err;

	if (swd_mode) {
		/* Data Phase (bit 2) must be set to 1 if sticky overrun
		 * detection is enabled */
		retval = cmsis_dap_cmd_dap_swd_configure(0);	/* 1 TRN, no Data Phase */
		if (retval != ERROR_OK)
			goto init_err;
	}
	/* Both LEDs on */
	/* Intentionally not checked for error, debugging will work
	 * without LEDs */
	(void)cmsis_dap_cmd_dap_led(LED_ID_CONNECT, LED_ON);
	(void)cmsis_dap_cmd_dap_led(LED_ID_RUN, LED_ON);

	/* support connecting with srst asserted */
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
		if (jtag_reset_config & RESET_SRST_NO_GATING) {
			retval = cmsis_dap_cmd_dap_swj_pins(0, SWJ_PIN_SRST, 0, NULL);
			if (retval != ERROR_OK)
				goto init_err;
			LOG_INFO("Connecting under reset");
		}
	}
	LOG_INFO("CMSIS-DAP: Interface ready");
	return ERROR_OK;

init_err:
	cmsis_dap_quit();
	return retval;
}

static int cmsis_dap_quit(void)
{
	cmsis_dap_cmd_dap_disconnect();

	/* Both LEDs off */
	cmsis_dap_cmd_dap_led(LED_ID_RUN, LED_OFF);
	cmsis_dap_cmd_dap_led(LED_ID_CONNECT, LED_OFF);

	cmsis_dap_close(cmsis_dap_handle);

	return ERROR_OK;
}

static int cmsis_dap_reset(int trst, int srst)
{
	/* Set both TRST and SRST even if they're not enabled as
	 * there's no way to tristate them */

	output_pins = 0;
	if (!srst)
		output_pins |= SWJ_PIN_SRST;
	if (!trst)
		output_pins |= SWJ_PIN_TRST;

	int retval = cmsis_dap_cmd_dap_swj_pins(output_pins,
			SWJ_PIN_TRST | SWJ_PIN_SRST, 0, NULL);
	if (retval != ERROR_OK)
		LOG_ERROR("CMSIS-DAP: Interface reset failed");
	return retval;
}

static int cmsis_dap_speed(int speed)
{
	if (speed == 0) {
		LOG_ERROR("RTCK not supported. Set nonzero \"adapter speed\".");
		return ERROR_JTAG_NOT_IMPLEMENTED;
	}

	return cmsis_dap_cmd_dap_swj_clock(speed);
}

static int cmsis_dap_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

static int cmsis_dap_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;
	return ERROR_OK;
}

/**
 * @see adapter_driver::config_trace
 */
static int cmsis_dap_config_trace(
				bool trace_enabled,
				enum tpiu_pin_protocol pin_protocol,
				uint32_t port_size,
				unsigned int *swo_freq,
				unsigned int traceclkin_hz,
				uint16_t *swo_prescaler)
{
	int retval;

	if (!trace_enabled) {
		if (cmsis_dap_handle->trace_enabled) {
			retval = cmsis_dap_cmd_dap_swo_control(DAP_SWO_CONTROL_STOP);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to disable the SWO-trace.");
				return retval;
			}
		}
		cmsis_dap_handle->trace_enabled = false;
		LOG_INFO("SWO-trace disabled.");
		return ERROR_OK;
	}

	if (!(cmsis_dap_handle->caps & INFO_CAPS_SWO_UART) &&
	    !(cmsis_dap_handle->caps & INFO_CAPS_SWO_MANCHESTER)) {
		LOG_ERROR("SWO-trace is not supported by the device.");
		return ERROR_FAIL;
	}

	uint8_t swo_mode;
	if (pin_protocol == TPIU_PIN_PROTOCOL_ASYNC_UART &&
	   (cmsis_dap_handle->caps & INFO_CAPS_SWO_UART)) {
		swo_mode = DAP_SWO_MODE_UART;
	} else if (pin_protocol == TPIU_PIN_PROTOCOL_ASYNC_MANCHESTER &&
		  (cmsis_dap_handle->caps & INFO_CAPS_SWO_MANCHESTER)) {
		swo_mode = DAP_SWO_MODE_MANCHESTER;
	} else {
		LOG_ERROR("Selected pin protocol is not supported.");
		return ERROR_FAIL;
	}

	if (*swo_freq == 0) {
		LOG_INFO("SWO-trace frequency autodetection not implemented.");
		return ERROR_FAIL;
	}

	retval = cmsis_dap_cmd_dap_swo_control(DAP_SWO_CONTROL_STOP);
	if (retval != ERROR_OK)
		return retval;

	cmsis_dap_handle->trace_enabled = false;

	retval = cmsis_dap_get_swo_buf_sz(&cmsis_dap_handle->swo_buf_sz);
	if (retval != ERROR_OK)
		return retval;

	retval = cmsis_dap_cmd_dap_swo_transport(DAP_SWO_TRANSPORT_DATA);
	if (retval != ERROR_OK)
		return retval;

	retval = cmsis_dap_cmd_dap_swo_mode(swo_mode);
	if (retval != ERROR_OK)
		return retval;

	retval = cmsis_dap_cmd_dap_swo_baudrate(*swo_freq, swo_freq);
	if (retval != ERROR_OK)
		return retval;

	if (!calculate_swo_prescaler(traceclkin_hz, *swo_freq,
			swo_prescaler)) {
		LOG_ERROR("SWO frequency is not suitable. Please choose a "
			"different frequency or use auto-detection.");
		return ERROR_FAIL;
	}

	LOG_INFO("SWO frequency: %u Hz.", *swo_freq);
	LOG_INFO("SWO prescaler: %u.", *swo_prescaler);

	retval = cmsis_dap_cmd_dap_swo_control(DAP_SWO_CONTROL_START);
	if (retval != ERROR_OK)
		return retval;

	cmsis_dap_handle->trace_enabled = true;

	return ERROR_OK;
}

/**
 * @see adapter_driver::poll_trace
 */
static int cmsis_dap_poll_trace(uint8_t *buf, size_t *size)
{
	uint8_t trace_status;
	size_t trace_count;

	if (!cmsis_dap_handle->trace_enabled) {
		*size = 0;
		return ERROR_OK;
	}

	int retval = cmsis_dap_cmd_dap_swo_status(&trace_status, &trace_count);
	if (retval != ERROR_OK)
		return retval;
	if ((trace_status & DAP_SWO_STATUS_CAPTURE_MASK) != DAP_SWO_STATUS_CAPTURE_ACTIVE)
		return ERROR_FAIL;

	*size = trace_count < *size ? trace_count : *size;
	size_t read_so_far = 0;
	do {
		size_t rb = 0;
		uint32_t packet_size = cmsis_dap_handle->packet_size - 4 /*data-reply*/;
		uint32_t remaining = *size - read_so_far;
		if (remaining < packet_size)
			packet_size = remaining;
		retval = cmsis_dap_cmd_dap_swo_data(
						packet_size,
						&trace_status,
						&rb,
						&buf[read_so_far]);
		if (retval != ERROR_OK)
			return retval;
		if ((trace_status & DAP_SWO_STATUS_CAPTURE_MASK) != DAP_SWO_STATUS_CAPTURE_ACTIVE)
			return ERROR_FAIL;

		read_so_far += rb;
	} while (read_so_far < *size);

	return ERROR_OK;
}

/**
 * @}
 */

/**
 * @defgroup cmd_handler Command Handler Func
 * @{
*/
COMMAND_HANDLER(ether_dap_handle_info_command)
{
	if (cmsis_dap_get_version_info() == ERROR_OK)
		cmsis_dap_get_status();

	return ERROR_OK;
}

COMMAND_HANDLER(ether_dap_handle_scan_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
    else {


    }

	return ERROR_OK;
}

COMMAND_HANDLER(ether_dap_handle_test_conn_command)
{
	if (CMD_ARGC > 2 || CMD_ARGC == 0)
		return ERROR_COMMAND_SYNTAX_ERROR;
    else {


    }

	return ERROR_OK;
}

COMMAND_HANDLER(ether_dap_handle_scan_command)
{
	uint8_t *command = cmsis_dap_handle->command;

	for (unsigned i = 0; i < CMD_ARGC; i++)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[i], command[i]);

	int retval = cmsis_dap_xfer(cmsis_dap_handle, CMD_ARGC);

	if (retval != ERROR_OK) {
		LOG_ERROR("CMSIS-DAP command failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	uint8_t *resp = ether_dap_handle->response;
	LOG_INFO("Returned data %02" PRIx8 " %02" PRIx8 " %02" PRIx8 " %02" PRIx8,
		resp[1], resp[2], resp[3], resp[4]);

	return ERROR_OK;
}

COMMAND_HANDLER(ether_dap_handle_quirk_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 1)
		COMMAND_PARSE_ENABLE(CMD_ARGV[0], ether_dap_handle->quirk_mode);

	command_print(CMD, "ETHER-DAP quirk workarounds %s",
				  ether_dap_handle->quirk_mode ? "enabled" : "disabled");
	return ERROR_OK;
}
/**
 * @}
 */

static const struct command_registration ether_dap_subcommand_handlers[] = {
	{
		.name = "info",
		.handler = &ether_dap_handle_info_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "show ether-dap info",
	},
    {
        .name = "scan",
		.handler = &ether_dap_handle_scan_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "scan ether-dap in LAN",
    },
    {
        .name = "test_conn",
		.handler = &ether_dap_handle_test_conn_command,
		.mode = COMMAND_EXEC,
		.usage = "test_conn <ip> <port>",
		.help = "test connection with remote device",
    },
	{
		.name = "cmd",
		.handler = &ether_dap_handle_cmd_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "issue ether-dap command",
	},
	{
		.name = "quirk",
		.handler = &ether_dap_handle_quirk_command,
		.mode = COMMAND_ANY,
		.help = "allow expensive workarounds of known adapter quirks.",
		.usage = "[enable | disable]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration ether_dap_command_handlers[] = {
	{
		.name = "ether-dap",
		.mode = COMMAND_ANY,
		.help = "perform ETHER-DAP management",
		.usage = "<cmd>",
		.chain = ether_dap_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct swd_driver ether_dap_swd_driver = {
	.init = ether_dap_swd_init,
	.switch_seq = cmsis_dap_swd_switch_seq,
	.read_reg = cmsis_dap_swd_read_reg,
	.write_reg = cmsis_dap_swd_write_reg,
	.run = cmsis_dap_swd_run_queue,
};

static const char * const ether_dap_transport[] = { "swd", "jtag", NULL };

static struct jtag_interface ether_dap_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = ether_dap_execute_queue,
};

struct adapter_driver ether_dap_adapter_driver = {
	.name = "ether-dap",
	.transports = ether_dap_transport,
	.commands = ether_dap_command_handlers,

	.init = cmsis_dap_init,
	.quit = cmsis_dap_quit,
	.reset = cmsis_dap_reset,
	.speed = cmsis_dap_speed,
	.khz = cmsis_dap_khz,
	.speed_div = cmsis_dap_speed_div,
	.config_trace = cmsis_dap_config_trace,
	.poll_trace = cmsis_dap_poll_trace,

	.jtag_ops = &cmsis_dap_interface,
	.swd_ops = &cmsis_dap_swd_driver,
};
