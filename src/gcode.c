/*-
 * Copyright (c) 2023-2024 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
#include <sys/console.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/sem.h>
#include <sys/thread.h>

#include <arm/stm/stm32f4.h>

#include "board.h"
#include "gcode.h"
#include "pnp.h"

#define	GCODE_DEBUG
#undef	GCODE_DEBUG

#ifdef	GCODE_DEBUG
#define	dprintf(fmt, ...)	printf(fmt, ##__VA_ARGS__)
#else
#define	dprintf(fmt, ...)
#endif

#define	DMA_BUF_SIZE	4096
#define	MAX_GCODE_LEN	256

/* Upper bound for the 'Q' modifier, so a typo can't block the loop. */
#define	MAX_DURATION_MS	10000

static uint8_t dma_buffer[DMA_BUF_SIZE];
static uint8_t cmd_buffer[MAX_GCODE_LEN];
static int cmd_buffer_ptr;
static int cmd_buffer_ovf;

/*
 * The gcode_command_*() helpers print their own "error: ..." line and
 * return non-zero on failure, in which case COMPLETE is not sent.
 */

static int
gcode_command_sensor_read(struct gcode_command *cmd)
{
	int val;

	if (cmd->sensor_read_target == 1) {
		val = pin_get(&gpio_sc, PORT_B, 3) ? 0 : 1;
		printf("ok V:%d\n", val);
	} else if (cmd->sensor_read_target == 2) {
		val = pin_get(&gpio_sc, PORT_D, 4) ? 0 : 1;
		printf("ok W:%d\n", val);
	} else {
		printf("error: unknown sensor\n");
		return (-1);
	}

	return (0);
}

/*
 * mdx_usleep() longer than about a second rounds up to the next second
 * (16 bit timer), so sleep in shorter slices.
 */
static void
gcode_sleep_ms(int ms)
{
	int chunk;

	while (ms > 0) {
		chunk = ms > 500 ? 500 : ms;
		mdx_usleep(chunk * 1000);
		ms -= chunk;
	}
}

static int
gcode_actuate_needle(int val)
{
	int cur;

	cur = pin_get(&gpio_sc, PORT_B, 5);
	if (cur && val) {
		printf("error: needle already set\n");
		return (-1);
	}

	if (!cur && !val) {
		printf("error: needle already cleared\n");
		return (-1);
	}

	pin_set(&gpio_sc, PORT_E, 0, val);
	mdx_usleep(150000);

	if (val == 0) {
		/* Ensure it is not set. */
		do {
			cur = pin_get(&gpio_sc, PORT_B, 5);
			if (cur == 0)
				break;
			mdx_usleep(25000);
		} while (1);
	}

	return (0);
}

/*
 * Power the nozzle rotation drivers while a valve is open: called with
 * on = 1 before the valves are driven and on = 0 after.
 */
static void
gcode_actuate_hvacuum(struct gcode_command *cmd, int on)
{

	if (cmd->actuate_mask & (1 << PNP_ACT_AVAC1) &&
	    cmd->actuate_value[PNP_ACT_AVAC1] == on)
		pnp_hvacuum(1, on);

	if (cmd->actuate_mask & (1 << PNP_ACT_AVAC2) &&
	    cmd->actuate_value[PNP_ACT_AVAC2] == on)
		pnp_hvacuum(2, on);
}

static int
gcode_command_actuate(struct gcode_command *cmd)
{
	int error;
	int val;

	error = 0;

	gcode_actuate_hvacuum(cmd, 1);

	if (cmd->actuate_mask & (1 << PNP_ACT_PUMP)) {
		pin_set(&gpio_sc, PORT_B, 13,
		    cmd->actuate_value[PNP_ACT_PUMP]);
		mdx_usleep(250000);
	}

	if (cmd->actuate_mask & (1 << PNP_ACT_AVAC1)) {
		pin_set(&gpio_sc, PORT_E, 2,
		    cmd->actuate_value[PNP_ACT_AVAC1]);
		mdx_usleep(250000);
	}

	if (cmd->actuate_mask & (1 << PNP_ACT_AVAC2)) {
		pin_set(&gpio_sc, PORT_E, 1,
		    cmd->actuate_value[PNP_ACT_AVAC2]);
		mdx_usleep(250000);
	}

	gcode_actuate_hvacuum(cmd, 0);

	if (cmd->actuate_mask & (1 << PNP_ACT_NEEDLE)) {
		error = gcode_actuate_needle(cmd->actuate_value[
		    PNP_ACT_NEEDLE]);
		if (error)
			return (error);
	}

	if (cmd->actuate_mask & (1 << PNP_ACT_PEEL_L)) {
		val = cmd->actuate_value[PNP_ACT_PEEL_L];
		pin_set(&gpio_sc, PORT_B, 12, val);
		if (val)
			mdx_usleep(250000);
	}

	if (cmd->actuate_mask & (1 << PNP_ACT_PEEL_R)) {
		val = cmd->actuate_value[PNP_ACT_PEEL_R];
		pin_set(&gpio_sc, PORT_B, 11, val);
		if (val)
			mdx_usleep(250000);
	}

	/* PD5 is the top ring, PC10 the bottom one. */
	if (cmd->actuate_mask & (1 << PNP_ACT_LED_TOP))
		pin_set(&gpio_sc, PORT_D, 5,
		    cmd->actuate_value[PNP_ACT_LED_TOP]);

	if (cmd->actuate_mask & (1 << PNP_ACT_LED_BOTTOM))
		pin_set(&gpio_sc, PORT_C, 10,
		    cmd->actuate_value[PNP_ACT_LED_BOTTOM]);

	/* After the valves, so that E wins. */
	if (cmd->actuate_mask & (1 << PNP_ACT_HPOWER))
		pnp_hforce(cmd->actuate_value[PNP_ACT_HPOWER]);

	if (cmd->actuate_mask & (1 << PNP_ACT_VIBRATOR)) {
		val = cmd->actuate_value[PNP_ACT_VIBRATOR];
		pin_set(&gpio_sc, PORT_A, 3, val);
		if (val && cmd->duration_ms) {
			gcode_sleep_ms(cmd->duration_ms);
			pin_set(&gpio_sc, PORT_A, 3, 0);
		}
	}

	return (error);
}

static void
gcode_set_actuator(struct gcode_command *cmd, int target, float value)
{

	cmd->actuate_mask |= (1 << target);
	cmd->actuate_value[target] = value ? 1 : 0;
}

/* Clamp before converting to int. NaN lands on the lower bound. */
static float
gcode_clamp(float value, float min, float max)
{

	if (!(value >= min))
		return (min);
	if (!(value <= max))
		return (max);

	return (value);
}

static int
gcode_coord_nm(float value)
{

	return (gcode_clamp(value, -1000.0f, 1000.0f) * 1000000);
}

static void
gcode_command(char *line, int len)
{
	struct gcode_command cmd;
	uint8_t letter;
	char *endp;
	char *end;
	float value;
	int error;

#ifdef GCODE_DEBUG
	int i;
	printf("GCODE: ");
	for (i = 0; i < len; i++)
		printf("%c", line[i]);
	printf("\n");
#endif

	bzero(&cmd, sizeof(struct gcode_command));

	end = line + len;
	while (line < end) {
		letter = *line;

		/* Skip spaces. */
		if (letter == ' ' || letter == '\t') {
			line += 1;
			continue;
		}

		/* Rest of the line is a comment. */
		if (letter == ';')
			break;

		if (letter < 'A' || letter > 'Z') {
			printf("error: bad gcode\n");
			return;
		}

		/* Skip letter. */
		line += 1;

		/* Ensure we are dealing with digit, + or -. */
		if (line == end ||
		    ((*line < '0' || *line > '9') &&
		    (*line != '+') &&
		    (*line != '-'))) {
			printf("error: bad gcode\n");
			return;
		}

		value = strtof(line, &endp);
		line = endp;

		dprintf("%s: value %.3f\n", __func__, value);

		switch (letter) {
		case 'M':
			if (value == 800.0f)
				cmd.type = CMD_TYPE_ACTUATE;
			else if (value == 105.0f)
				cmd.type = CMD_TYPE_SENSOR_READ;
			else if (value == 114.0f)
				cmd.type = CMD_TYPE_POS_REPORT;
			break;
		case 'G':
			if (value == 0.0f) /* Linear move. */
				cmd.type = CMD_TYPE_MOVE;
			else if (value == 28.0f) /* Home. */
				cmd.type = CMD_TYPE_HOME;
			break;
		case 'X':
			cmd.x = gcode_coord_nm(value);
			cmd.x_set = 1;
			break;
		case 'Y':
			cmd.y = gcode_coord_nm(value);
			cmd.y_set = 1;
			break;
		case 'Z':
			cmd.z = gcode_coord_nm(value);
			cmd.z_set = 1;
			break;
		case 'I':
			cmd.h1 = gcode_coord_nm(value);
			cmd.h1_set = 1;
			break;
		case 'J':
			cmd.h2 = gcode_coord_nm(value);
			cmd.h2_set = 1;
			break;
		case 'P':
			gcode_set_actuator(&cmd, PNP_ACT_PUMP, value);
			break;
		case 'V':
			/* Air vacuum 1 */
			gcode_set_actuator(&cmd, PNP_ACT_AVAC1, value);
			break;
		case 'W':
			/* Air vacuum 2 */
			gcode_set_actuator(&cmd, PNP_ACT_AVAC2, value);
			break;
		case 'N':
			/* Air vac sensors read. */
			cmd.sensor_read_target = gcode_clamp(value, 0.0f, 9.0f);
			break;
		case 'D':
			/* Needle */
			gcode_set_actuator(&cmd, PNP_ACT_NEEDLE, value);
			break;
		case 'L':
			/* Peel, left bank. */
			gcode_set_actuator(&cmd, PNP_ACT_PEEL_L, value);
			break;
		case 'R':
			/* Peel, right bank. */
			gcode_set_actuator(&cmd, PNP_ACT_PEEL_R, value);
			break;
		case 'T':
			/* Top (down-looking) camera LED ring. */
			gcode_set_actuator(&cmd, PNP_ACT_LED_TOP, value);
			break;
		case 'B':
			/* Bottom (up-looking) camera LED ring. */
			gcode_set_actuator(&cmd, PNP_ACT_LED_BOTTOM, value);
			break;
		case 'S':
			/* Vibrator (stick / bulk feeder). */
			gcode_set_actuator(&cmd, PNP_ACT_VIBRATOR, value);
			break;
		case 'E':
			/* Nozzle rotation driver power. */
			gcode_set_actuator(&cmd, PNP_ACT_HPOWER, value);
			break;
		case 'Q':
			/* Actuation length, milliseconds. */
			cmd.duration_ms = gcode_clamp(value, 0.0f,
			    (float)MAX_DURATION_MS);
			break;
		case 'F':
			/* Feed rate, mm per minute. */
			cmd.f = gcode_clamp(value, 0.0f, 1000000.0f);
			cmd.f_set = 1;
			break;
		default:
			dprintf("%s: ignoring letter %c\n", __func__, letter);
			break;
		}
	}

	switch (cmd.type) {
	case CMD_TYPE_MOVE:
		error = pnp_command_move(&cmd);
		break;
	case CMD_TYPE_ACTUATE:
		error = gcode_command_actuate(&cmd);
		break;
	case CMD_TYPE_SENSOR_READ:
		error = gcode_command_sensor_read(&cmd);
		break;
	case CMD_TYPE_HOME:
		error = pnp_home_all();
		break;
	case CMD_TYPE_POS_REPORT:
		error = pnp_report_position();
		break;
	default:
		/* Nothing we act on. Still a well formed line. */
		error = 0;
		break;
	};

	/* A failed command has already printed its error line. */
	if (error == 0)
		printf("COMPLETE\n");
}

static void
gcode_process_data(int ptr, int len)
{
	uint8_t *start;
	uint8_t ch;
	int i;

	start = &dma_buffer[ptr];
	for (i = 0; i < len; i++) {
		ch = start[i];
		dprintf("ch %d\n", ch);

		/* Tolerate CRLF line endings. */
		if (ch == '\r')
			continue;

		if (ch == '\n') { /* LF */
			if (cmd_buffer_ovf) {
				printf("error: line too long\n");
				cmd_buffer_ovf = 0;
			} else if (cmd_buffer_ptr > 0) {
				/* strtof() needs the terminator. */
				cmd_buffer[cmd_buffer_ptr] = '\0';
				gcode_command((char *)cmd_buffer,
				    cmd_buffer_ptr);
			}
			cmd_buffer_ptr = 0;
			continue;
		}

		/* Drop the rest of an oversized line, keep room for NUL. */
		if (cmd_buffer_ptr == MAX_GCODE_LEN - 1) {
			cmd_buffer_ovf = 1;
			continue;
		}

		cmd_buffer[cmd_buffer_ptr] = ch;
		cmd_buffer_ptr += 1;
	}
}

static void
gcode_dmarecv_init(void)
{
	struct stm32f4_dma_conf conf;

	bzero(&conf, sizeof(struct stm32f4_dma_conf));
	conf.mem0 = (uintptr_t)dma_buffer;
	conf.sid = 2;
	conf.periph_addr = USART1_BASE + USART_DR;
	conf.dir = 0;
	conf.channel = 4;
	conf.circ = 1;
	conf.psize = 8;
	conf.nbytes = DMA_BUF_SIZE;

	stm32f4_dma_setup(&dma2_sc, &conf);
	stm32f4_dma_control(&dma2_sc, 2, 1);
}

int
gcode_mainloop(void)
{
	uint32_t cnt;
	int ptr;

	ptr = 0;
	cmd_buffer_ptr = 0;
	cmd_buffer_ovf = 0;

	gcode_dmarecv_init();

	/* Periodically poll for a new data. */
	while (1) {
		cnt = stm32f4_dma_getcnt(&dma2_sc, 2);
		cnt = DMA_BUF_SIZE - cnt;

		if (cnt > ptr) {
			gcode_process_data(ptr, (cnt - ptr));
			ptr = cnt;
		} else if (cnt < ptr) {
			/* Buffer wrapped. */
			gcode_process_data(ptr, DMA_BUF_SIZE - ptr);
			gcode_process_data(0, cnt);
			ptr = cnt;
		}

		mdx_usleep(10000);
		pnp_htick(10);
	}

	return (0);
}
