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

#include <lib/msun/src/math.h>

#include <arm/stm/stm32f4.h>

#include "board.h"
#include "gcode.h"
#include "pnp.h"
#include "trig.h"

#define	PNP_DEBUG
#undef	PNP_DEBUG

#ifdef	PNP_DEBUG
#define	dprintf(fmt, ...)	printf(fmt, ##__VA_ARGS__)
#else
#define	dprintf(fmt, ...)
#endif

#define	PNP_MAX_X_NM		(364000000)	/* nanometers */
#define	PNP_MAX_Y_NM		(368000000)	/* nanometers */
#define	CAM_RADIUS		(15000000)

/* XY steppers are in linear motion. */
#define	PNP_XY_FULL_REVO_NM	(40000000)
#define	PNP_XY_FULL_REVO_STEPS	(6400)
#define	PNP_XY_STEP_NM		(PNP_XY_FULL_REVO_NM / PNP_XY_FULL_REVO_STEPS)

/* Z stepper: we translate linear into rotational motion. */
#define	PNP_Z_FULL_REVO_DEG	(360000000)
#define	PNP_Z_FULL_REVO_STEPS	(12800)
#define	PNP_Z_STEP_DEG		(PNP_Z_FULL_REVO_DEG / PNP_Z_FULL_REVO_STEPS)

/* NR (Nozzle Rotation) steppers are in rotational motion. */
#define	PNP_NR_FULL_REVO_DEG	(360000000)
#define	PNP_NR_FULL_REVO_STEPS	(12800)
#define	PNP_NR_STEP_DEG		(PNP_NR_FULL_REVO_DEG / PNP_NR_FULL_REVO_STEPS)

#define	PNP_STEPS_X_MIN		0
#define	PNP_STEPS_X_MAX		(PNP_MAX_X_NM / PNP_XY_STEP_NM)
#define	PNP_STEPS_Y_MIN		0
#define	PNP_STEPS_Y_MAX		(PNP_MAX_Y_NM / PNP_XY_STEP_NM)
#define	PNP_STEPS_Z_MIN		(-111000000 / PNP_Z_STEP_DEG)
#define	PNP_STEPS_Z_MAX		(111000000 / PNP_Z_STEP_DEG)
#define	PNP_STEPS_H_MIN		(-180000000 / PNP_NR_STEP_DEG)
#define	PNP_STEPS_H_MAX		(180000000 / PNP_NR_STEP_DEG)

/*
 * Speed. A step is 400 ticks of the timer, so the step rate is
 * speed * 375 Hz on X and Y, and speed * 125 Hz on Z and the nozzles.
 * The frequency must stay below the 84 MHz timer clock, or the
 * prescaler wraps around.
 */
#define	PNP_XY_FREQ_MUL		150000
#define	PNP_ZH_FREQ_MUL		50000
#define	PNP_TIMER_FREQ		84000000
#define	PNP_SPEED_MIN		1
#define	PNP_SPEED_MAX_XY	(PNP_TIMER_FREQ / 2 / PNP_XY_FREQ_MUL)
#define	PNP_SPEED_MAX_ZH	(PNP_TIMER_FREQ / 2 / PNP_ZH_FREQ_MUL)
#define	PNP_SPEED_DEFAULT	100
#define	PNP_SPEED_RAMP_MIN	15

/*
 * Feed rate conversion, speed = F * f_num / f_den with F in mm/min.
 * X, Y: 160 steps/mm at 375 steps/s per unit of speed.
 * Z: sized on the steepest point of the cam (135.8 steps/mm at 90 deg),
 * so Z is never faster than asked.
 */
#define	PNP_XY_F_NUM		8
#define	PNP_XY_F_DEN		1125
#define	PNP_Z_F_NUM		181
#define	PNP_Z_F_DEN		10000

struct move_task {
	int steps;
	int check_home;
	int direction;
	int speed;
	mdx_sem_t task_compl_sem;
	int speed_control;

	/* Target position, committed to the motor when the move starts. */
	int pos_nm;

	/* Result */
	int home_found;
};

struct motor_state {
	int chanset;	/* PWM channels. */
	mdx_sem_t worker_sem;
	struct move_task task;
	const char *name;
	const char *axis;	/* GCode letter, for error and position reports. */
	void (*set_direction)(int dir);
	int (*is_at_home)(void);
	void (*step)(int chanset, int speed);
	mdx_sem_t step_sem;
	int step_nm;	/* Length of a step, nanometers. Has to be signed. */

	int (*cam_translate_mm_to_deg)(float z, float cam_radius, int *result);
	int cam_radius;

	/*
	 * Current offset from home in steps.
	 * Could be negative for Z or nozzles.
	 */
	int steps;

	/*
	 * Last position moved to, in nanometers. Used to report Z, whose
	 * steps are cam degrees.
	 */
	int pos_nm;

	/* Feed rate to speed conversion. Zero f_den means F is ignored. */
	int f_num;
	int f_den;
	int speed_max;

	/* Limits. */
	int steps_max;
	int steps_min;
};

struct pnp_state {
	struct motor_state motor_x;
	struct motor_state motor_y;
	struct motor_state motor_z;
	struct motor_state motor_h1;
	struct motor_state motor_h2;

	/* Moves are refused until homing has succeeded. */
	int homed;
};

static struct pnp_state pnp;

void
pnp_pwm_y_intr(void *arg, int irq)
{

	stm32f4_pwm_intr(arg, irq);
	mdx_sem_post(&pnp.motor_y.step_sem);
}

void
pnp_pwm_x_intr(void *arg, int irq)
{

	stm32f4_pwm_intr(arg, irq);
	mdx_sem_post(&pnp.motor_x.step_sem);
}

void
pnp_pwm_z_intr(void *arg, int irq)
{

	stm32f4_pwm_intr(arg, irq);
	mdx_sem_post(&pnp.motor_z.step_sem);
}

void
pnp_pwm_h1_intr(void *arg, int irq)
{

	stm32f4_pwm_intr(arg, irq);
	mdx_sem_post(&pnp.motor_h1.step_sem);
}

void
pnp_pwm_h2_intr(void *arg, int irq)
{

	stm32f4_pwm_intr(arg, irq);
	mdx_sem_post(&pnp.motor_h2.step_sem);
}

static inline int
pnp_is_x_home(void)
{

	return (pin_get(&gpio_sc, PORT_C, 6));
}

static inline int
pnp_is_yl_home(void)
{

	return (pin_get(&gpio_sc, PORT_C, 7));
}

static inline int
pnp_is_z_home(void)
{

	return (pin_get(&gpio_sc, PORT_B, 4));
}

static inline int __unused
pnp_is_yr_home(void)
{

	return (pin_get(&gpio_sc, PORT_C, 1));
}

static void
pnp_xenable(int enable)
{

	pin_set(&gpio_sc, PORT_D, 14, enable); /* X Vref */
	mdx_usleep(10000);
	pin_set(&gpio_sc, PORT_E, 6, enable); /* X ST */
	mdx_usleep(10000);
}

static void
pnp_yenable(int enable)
{

	pin_set(&gpio_sc, PORT_D, 15, enable); /* Y Vref */
	mdx_usleep(10000);
	pin_set(&gpio_sc, PORT_C, 0, enable); /* Y R ST */
	pin_set(&gpio_sc, PORT_A, 8, enable); /* Y L ST */
	mdx_usleep(10000);
}

static void
pnp_zenable(int enable)
{

	pin_set(&gpio_sc, PORT_D, 13, enable); /* Z Vref */
	mdx_usleep(10000);
	pin_set(&gpio_sc, PORT_E, 4, enable); /* ST */
	mdx_usleep(10000);
}

void
pnp_henable(int enable)
{

	pin_set(&gpio_sc, PORT_D, 12, enable); /* H Vref */
	mdx_usleep(10000);
	pin_set(&gpio_sc, PORT_D, 3, enable); /* H1 ST */
	pin_set(&gpio_sc, PORT_A, 15, enable); /* H2 ST */
	mdx_usleep(10000);
}

static void
pnp_xset_direction(int dir)
{

	pin_set(&gpio_sc, PORT_E, 5, dir); /* X FR */
}

static void
pnp_h1set_direction(int dir)
{

	pin_set(&gpio_sc, PORT_D, 1, dir); /* H1 FR */
}

static void
pnp_h2set_direction(int dir)
{

	pin_set(&gpio_sc, PORT_D, 0, dir); /* H2 FR */
}

static void
pnp_yset_direction(int dir)
{
	int rdir;

	rdir = dir ? 0 : 1;

	pin_set(&gpio_sc, PORT_C, 13, rdir); /* Y R FR */
	pin_set(&gpio_sc, PORT_C, 9, dir); /* Y L FR */
}

static void
pnp_yset_direction_rev(int dir)
{
	int rdir;

	rdir = dir ? 0 : 1;

	pin_set(&gpio_sc, PORT_C, 13, !rdir); /* Y R FR */
	pin_set(&gpio_sc, PORT_C, 9, !dir); /* Y L FR */
}

static void
pnp_zset_direction(int dir)
{

	pin_set(&gpio_sc, PORT_E, 3, dir); /* Z FR */
}

static uint32_t
pnp_step_freq(int speed, int mul, int max)
{

	if (speed < PNP_SPEED_MIN)
		speed = PNP_SPEED_MIN;
	if (speed > max)
		speed = max;

	return ((uint32_t)speed * mul);
}

static void
xstep(int chanset, int speed)
{
	uint32_t freq;

	freq = pnp_step_freq(speed, PNP_XY_FREQ_MUL, PNP_SPEED_MAX_XY);

	stm32f4_pwm_step(&pwm_x_sc, chanset, freq);
}

static void
ystep(int chanset, int speed)
{
	uint32_t freq;

	freq = pnp_step_freq(speed, PNP_XY_FREQ_MUL, PNP_SPEED_MAX_XY);

	stm32f4_pwm_step(&pwm_y_sc, chanset, freq);
}

static void
zstep(int chanset, int speed)
{
	uint32_t freq;

	freq = pnp_step_freq(speed, PNP_ZH_FREQ_MUL, PNP_SPEED_MAX_ZH);

	stm32f4_pwm_step(&pwm_z_sc, chanset, freq);
}

static void
h1step(int chanset, int speed)
{
	uint32_t freq;

	freq = pnp_step_freq(speed, PNP_ZH_FREQ_MUL, PNP_SPEED_MAX_ZH);

	stm32f4_pwm_step(&pwm_h1_sc, chanset, freq);
}

static void
h2step(int chanset, int speed)
{
	uint32_t freq;

	freq = pnp_step_freq(speed, PNP_ZH_FREQ_MUL, PNP_SPEED_MAX_ZH);

	stm32f4_pwm_step(&pwm_h2_sc, chanset, freq);
}

static int
calc_speed(int i, int steps, int target)
{
	int speed;
	int t;

	t = i < (steps - i) ? i : (steps - i);
	if (t >= 1000)
		return (target);

	/* Gradually increase/decrease speed */
	speed = t / 10;
	if (speed < PNP_SPEED_RAMP_MIN)
		speed = PNP_SPEED_RAMP_MIN;

	/* Never ramp past the requested speed. */
	if (speed > target)
		speed = target;

	return (speed);
}

/*
 * Convert a feed rate in mm/min into this motor's speed unit. The
 * nozzle rotations have no conversion and run at the default speed.
 */
static int
pnp_speed_from_f(struct motor_state *motor, struct gcode_command *cmd)
{
	int speed;

	if (!cmd->f_set || motor->f_den == 0 || cmd->f <= 0)
		return (PNP_SPEED_DEFAULT);

	speed = cmd->f * motor->f_num / motor->f_den;

	if (speed < PNP_SPEED_MIN)
		speed = PNP_SPEED_MIN;
	if (speed > motor->speed_max)
		speed = motor->speed_max;

	return (speed);
}

static void
pnp_worker_thread(void *arg)
{
	struct motor_state *motor;
	struct move_task *task;
	uint32_t steps;
	int target;
	int speed;
	int i;

	motor = arg;
	task = &motor->task;

	while (1) {
		mdx_sem_wait(&motor->worker_sem);
		dprintf("%s: task rcvd\n", __func__);

		steps = task->steps;
		target = task->speed;
		speed = target;

		dprintf("%s: steps needed %d\n", __func__, steps);

		motor->set_direction(task->direction);

		for (i = 0; i < steps; i++) {
			if (task->check_home && motor->is_at_home()) {
				task->home_found = 1;
				break;
			}

			if (task->speed_control)
				speed = calc_speed(i, steps, target);

			motor->step(motor->chanset, speed);
			mdx_sem_wait(&motor->step_sem);
			if (task->direction == 1)
				motor->steps += 1;
			else
				motor->steps -= 1;
		}

		mdx_sem_post(&task->task_compl_sem);
		dprintf("%s: task compl\n", __func__);
	}
}

#define	PNP_ERR_TRANSLATE	1
#define	PNP_ERR_LIMITS		2

/*
 * Validate a move and stage it on the motor's task without starting it,
 * so that a multi axis G0 is refused as a whole.
 */
static int
pnp_move_prepare(struct motor_state *motor, int new_pos, int speed)
{
	struct move_task *task;
	int new_steps;
	int target;
	int error;
	int tmp;

	task = &motor->task;
	target = new_pos;

	/* Convert required position from mm to degrees if needed. */
	if (motor->cam_translate_mm_to_deg) {
		if (motor->cam_radius == 0)
			return (PNP_ERR_TRANSLATE);
		error = motor->cam_translate_mm_to_deg(new_pos,
		    motor->cam_radius, &tmp);
		if (error)
			return (PNP_ERR_TRANSLATE);
		new_pos = tmp;
	}

	new_steps = new_pos / motor->step_nm;
	if (new_steps > motor->steps_max ||
	    new_steps < motor->steps_min)
		return (PNP_ERR_LIMITS);

	task->check_home = 0;
	task->speed_control = 1;
	task->speed = speed;

	if (new_steps > motor->steps) {
		task->direction = 1;
		task->steps = new_steps - motor->steps;
	} else {
		task->direction = 0;
		task->steps = motor->steps - new_steps;
	}

	task->pos_nm = target;

	return (0);
}

static void
pnp_move_error(struct motor_state *motor, int error)
{

	switch (error) {
	case PNP_ERR_TRANSLATE:
		printf("error: cannot translate %s coordinate\n", motor->axis);
		break;
	case PNP_ERR_LIMITS:
		printf("error: %s out of range\n", motor->axis);
		break;
	default:
		printf("error: %s move failed\n", motor->axis);
		break;
	}
}

static void
pnp_move_start(struct motor_state *motor)
{

	motor->pos_nm = motor->task.pos_nm;
	mdx_sem_post(&motor->worker_sem);
}

static void
pnp_move_wait(struct motor_state *motor)
{

	mdx_sem_wait(&motor->task.task_compl_sem);
}

static int
pnp_move(struct motor_state *motor, int new_pos)
{
	int error;

	error = pnp_move_prepare(motor, new_pos, PNP_SPEED_DEFAULT);
	if (error)
		return (error);

	pnp_move_start(motor);
	pnp_move_wait(motor);

	return (0);
}

static int
pnp_move_xy(int new_pos_x, int new_pos_y)
{
	int error;

	error = pnp_move_prepare(&pnp.motor_x, new_pos_x, PNP_SPEED_DEFAULT);
	if (error)
		return (error);

	error = pnp_move_prepare(&pnp.motor_y, new_pos_y, PNP_SPEED_DEFAULT);
	if (error)
		return (error);

	pnp_move_start(&pnp.motor_x);
	pnp_move_start(&pnp.motor_y);

	pnp_move_wait(&pnp.motor_x);
	pnp_move_wait(&pnp.motor_y);

	dprintf("%s: new pos %d %d\n", __func__, pnp.motor_x.steps,
	    pnp.motor_y.steps);

	return (0);
}

static int
pnp_move_home_motor(struct motor_state *motor)
{
	struct move_task *task;

	task = &motor->task;

	/* First reach home quickly. */
	if (motor->is_at_home() == 0) {
		task->steps = PNP_MAX_Y_NM / motor->step_nm;
		task->check_home = 1;
		task->speed = 20;
		task->speed_control = 0;
		task->direction = 0;
		mdx_sem_post(&motor->worker_sem);
		mdx_sem_wait(&task->task_compl_sem);
	}

	if (motor->is_at_home() == 0) {
		printf("error: %s home switch not reached\n", motor->axis);
		return (-1);
	}

	/* Now move back a bit. */
	task->direction = 1;
	task->steps = 10000000 / motor->step_nm;
	task->speed = 10;
	task->speed_control = 0;
	task->check_home = 0;
	mdx_sem_post(&motor->worker_sem);
	mdx_sem_wait(&task->task_compl_sem);

	if (motor->is_at_home()) {
		printf("error: %s home switch stuck\n", motor->axis);
		return (-2);
	}

	/* Now try to reach home slowly. */

	dprintf("%s is trying to reach home\n", motor->name);
	task->steps = PNP_MAX_Y_NM / motor->step_nm;
	task->check_home = 1;
	task->speed = 2;
	task->speed_control = 0;
	task->direction = 0;
	mdx_sem_post(&motor->worker_sem);
	mdx_sem_wait(&task->task_compl_sem);

	if (motor->is_at_home() == 0) {
		printf("error: %s home switch not reached\n", motor->axis);
		return (-1);
	}

	/* Now go into home for 1 mm. */

	dprintf("%s is going into home for 1mm\n", motor->name);
	task->steps = 1000000 / motor->step_nm;
	task->check_home = 0;
	task->speed = 2;
	task->speed_control = 0;
	task->direction = 0;
	mdx_sem_post(&motor->worker_sem);
	mdx_sem_wait(&task->task_compl_sem);

	motor->steps = 0;
	motor->pos_nm = 0;
	dprintf("%s home reached\n", motor->name);

	return (0);
}

static int
pnp_move_home_z(struct motor_state *motor)
{
	struct move_task *task;
	int found;
	int steps;
	int dir;
	int i;

	task = &motor->task;

	/* First leave home. */
	if (pnp_is_z_home()) {
		task->steps = 200;
		task->check_home = 0;
		task->speed = 15;
		task->speed_control = 0;
		task->home_found = 0;
		task->direction = 1;
		mdx_sem_post(&motor->worker_sem);
		mdx_sem_wait(&task->task_compl_sem);
		/* TODO: ensure we left it. */
	}

	steps = 100;
	dir = 1;
	found = 0;

	/*
	 * Search with a growing oscillation around the start position.
	 * 40 sweeps cover the whole +/-111 deg cam range.
	 */
	for (i = 0; i < 40; i++) {
		printf("Making %d steps towards %d\n", steps, dir);
		task->direction = dir;
		task->steps = steps;
		task->check_home = 1;
		task->speed = 15;
		task->speed_control = 0;
		task->home_found = 0;
		mdx_sem_post(&motor->worker_sem);
		mdx_sem_wait(&task->task_compl_sem);
		if (task->home_found) {
			found = 1;
			break;
		}
		steps += 200;
		dir = !dir;
	}

	if (found == 0) {
		printf("error: Z home switch not found\n");
		return (-1);
	}

	/* Now make 50 steps into home. */
	task->steps = 50;
	task->check_home = 0;
	task->speed = 15;
	task->speed_control = 0;
	task->home_found = 0;
	task->direction = dir;
	mdx_sem_post(&motor->worker_sem);
	mdx_sem_wait(&task->task_compl_sem);

	motor->steps = 0;
	motor->pos_nm = 0;
	dprintf("Z home found\n");

	return (0);
}

/*
 * Home every axis. Safe to run more than once (G28): after homing the Y
 * origin is moved to the far end and its direction inverted, so that is
 * undone first.
 */
static int
pnp_home_axes(void)
{
	int error;

	pnp.homed = 0;
	pnp.motor_y.set_direction = pnp_yset_direction;

	/* Bring the nozzle rotations back to their tracked zero. */
	error = pnp_move(&pnp.motor_h1, 0);
	if (error == 0)
		error = pnp_move(&pnp.motor_h2, 0);
	if (error) {
		printf("error: cannot rotate nozzles to zero\n");
		return (error);
	}

	error = pnp_move_home_z(&pnp.motor_z);
	if (error)
		return (error);

	error = pnp_move_home_motor(&pnp.motor_y);
	if (error)
		return (error);

	error = pnp_move_home_motor(&pnp.motor_x);
	if (error)
		return (error);

	/* Change location of 0,0. */
	error = pnp_move_xy(0, PNP_MAX_Y_NM);
	if (error) {
		printf("error: cannot reach the Y origin\n");
		return (error);
	}

	pnp.motor_y.steps = 0;
	pnp.motor_y.pos_nm = 0;
	pnp.motor_y.set_direction = pnp_yset_direction_rev;

	pnp.homed = 1;

	return (0);
}

int
pnp_home_all(void)
{

	return (pnp_home_axes());
}

int
pnp_command_move(struct gcode_command *cmd)
{
	struct motor_state *started[4];
	int nstarted;
	int error;
	int i;

	if (pnp.homed == 0) {
		printf("error: machine not homed\n");
		return (-1);
	}

	/* Validate every axis before any of them moves. */
	if (cmd->x_set) {
		error = pnp_move_prepare(&pnp.motor_x, cmd->x,
		    pnp_speed_from_f(&pnp.motor_x, cmd));
		if (error) {
			pnp_move_error(&pnp.motor_x, error);
			return (error);
		}
	}

	if (cmd->y_set) {
		error = pnp_move_prepare(&pnp.motor_y, cmd->y,
		    pnp_speed_from_f(&pnp.motor_y, cmd));
		if (error) {
			pnp_move_error(&pnp.motor_y, error);
			return (error);
		}
	}

	if (cmd->h1_set) {
		error = pnp_move_prepare(&pnp.motor_h1, -1 * cmd->h1,
		    pnp_speed_from_f(&pnp.motor_h1, cmd));
		if (error) {
			pnp_move_error(&pnp.motor_h1, error);
			return (error);
		}
	}

	if (cmd->h2_set) {
		error = pnp_move_prepare(&pnp.motor_h2, -1 * cmd->h2,
		    pnp_speed_from_f(&pnp.motor_h2, cmd));
		if (error) {
			pnp_move_error(&pnp.motor_h2, error);
			return (error);
		}
	}

	if (cmd->z_set) {
		error = pnp_move_prepare(&pnp.motor_z, cmd->z,
		    pnp_speed_from_f(&pnp.motor_z, cmd));
		if (error) {
			pnp_move_error(&pnp.motor_z, error);
			return (error);
		}
	}

	/* Z goes last, once the head has arrived. */
	nstarted = 0;

	if (cmd->x_set) {
		dprintf("moving X to %d\n", cmd->x);
		pnp_move_start(&pnp.motor_x);
		started[nstarted++] = &pnp.motor_x;
	}

	if (cmd->y_set) {
		dprintf("moving Y to %d\n", cmd->y);
		pnp_move_start(&pnp.motor_y);
		started[nstarted++] = &pnp.motor_y;
	}

	if (cmd->h1_set) {
		dprintf("moving H1 to %d\n", -1 * cmd->h1);
		pnp_move_start(&pnp.motor_h1);
		started[nstarted++] = &pnp.motor_h1;
	}

	if (cmd->h2_set) {
		dprintf("moving H2 to %d\n", -1 * cmd->h2);
		pnp_move_start(&pnp.motor_h2);
		started[nstarted++] = &pnp.motor_h2;
	}

	for (i = 0; i < nstarted; i++)
		pnp_move_wait(started[i]);

	if (cmd->z_set) {
		dprintf("moving Z to %d\n", cmd->z);
		pnp_move_start(&pnp.motor_z);
		pnp_move_wait(&pnp.motor_z);
	}

	return (0);
}

int
pnp_report_position(void)
{
	float x, y, z, h1, h2;

	x = (float)pnp.motor_x.steps * pnp.motor_x.step_nm / 1000000.0f;
	y = (float)pnp.motor_y.steps * pnp.motor_y.step_nm / 1000000.0f;

	/* Z steps are cam degrees. */
	z = (float)pnp.motor_z.pos_nm / 1000000.0f;

	/* The rotations are negated on the way in. */
	h1 = -(float)pnp.motor_h1.steps * pnp.motor_h1.step_nm / 1000000.0f;
	h2 = -(float)pnp.motor_h2.steps * pnp.motor_h2.step_nm / 1000000.0f;

	printf("ok X:%.4f Y:%.4f Z:%.4f I:%.4f J:%.4f\n", x, y, z, h1, h2);

	return (0);
}

static void
pnp_motor_initialize(struct motor_state *motor, const char *name,
    const char *axis)
{

	mdx_sem_init(&motor->worker_sem, 0);
	mdx_sem_init(&motor->step_sem, 0);
	motor->name = name;
	motor->axis = axis;
}

static int
pnp_thread_create(const char *name, void *arg)
{
	struct thread *td;

	td = mdx_thread_create(name, 1 /* prio */, 500 /* quantum */,
	    8192 /* stack */, pnp_worker_thread, arg);
	if (td == NULL) {
		printf("%s: Failed to create X mover thread\n", __func__);
		return (-1);
	}

	mdx_sched_add(td);

	return (0);
}

static int
pnp_initialize(void)
{
	int error;

	bzero(&pnp, sizeof(struct pnp_state));

	pnp_motor_initialize(&pnp.motor_x, "X Motor", "X");
	pnp.motor_x.step_nm = PNP_XY_STEP_NM;
	pnp.motor_x.set_direction = pnp_xset_direction;
	pnp.motor_x.step = xstep;
	pnp.motor_x.chanset = (1 << 0);
	pnp.motor_x.is_at_home = pnp_is_x_home;
	pnp.motor_x.steps_min = PNP_STEPS_X_MIN;
	pnp.motor_x.steps_max = PNP_STEPS_X_MAX;
	pnp.motor_x.f_num = PNP_XY_F_NUM;
	pnp.motor_x.f_den = PNP_XY_F_DEN;
	pnp.motor_x.speed_max = PNP_SPEED_MAX_XY;
	mdx_sem_init(&pnp.motor_x.task.task_compl_sem, 0);

	pnp_motor_initialize(&pnp.motor_y, "Y Motor", "Y");
	pnp.motor_y.step_nm = PNP_XY_STEP_NM;
	pnp.motor_y.set_direction = pnp_yset_direction;
	pnp.motor_y.step = ystep;
	pnp.motor_y.chanset = ((1 << 0) | (1 << 1));
	pnp.motor_y.is_at_home = pnp_is_yl_home;
	pnp.motor_y.steps_min = PNP_STEPS_Y_MIN;
	pnp.motor_y.steps_max = PNP_STEPS_Y_MAX;
	pnp.motor_y.f_num = PNP_XY_F_NUM;
	pnp.motor_y.f_den = PNP_XY_F_DEN;
	pnp.motor_y.speed_max = PNP_SPEED_MAX_XY;
	mdx_sem_init(&pnp.motor_y.task.task_compl_sem, 0);

	pnp_motor_initialize(&pnp.motor_z, "Z Motor", "Z");
	pnp.motor_z.step_nm = PNP_Z_STEP_DEG;
	pnp.motor_z.set_direction = pnp_zset_direction;
	pnp.motor_z.step = zstep;
	pnp.motor_z.chanset = (1 << 0);
	pnp.motor_z.is_at_home = pnp_is_z_home;
	pnp.motor_z.cam_translate_mm_to_deg = trig_translate_z;
	pnp.motor_z.cam_radius = CAM_RADIUS;
	pnp.motor_z.steps_min = PNP_STEPS_Z_MIN;
	pnp.motor_z.steps_max = PNP_STEPS_Z_MAX;
	pnp.motor_z.f_num = PNP_Z_F_NUM;
	pnp.motor_z.f_den = PNP_Z_F_DEN;
	pnp.motor_z.speed_max = PNP_SPEED_MAX_ZH;
	mdx_sem_init(&pnp.motor_z.task.task_compl_sem, 0);

	pnp_motor_initialize(&pnp.motor_h1, "H1 Motor", "I");
	pnp.motor_h1.step_nm = PNP_NR_STEP_DEG;
	pnp.motor_h1.set_direction = pnp_h1set_direction;
	pnp.motor_h1.step = h1step;
	pnp.motor_h1.chanset = (1 << 0);
	pnp.motor_h1.is_at_home = NULL;
	pnp.motor_h1.steps_min = PNP_STEPS_H_MIN;
	pnp.motor_h1.steps_max = PNP_STEPS_H_MAX;
	pnp.motor_h1.speed_max = PNP_SPEED_MAX_ZH;
	mdx_sem_init(&pnp.motor_h1.task.task_compl_sem, 0);

	pnp_motor_initialize(&pnp.motor_h2, "H2 Motor", "J");
	pnp.motor_h2.step_nm = PNP_NR_STEP_DEG;
	pnp.motor_h2.set_direction = pnp_h2set_direction;
	pnp.motor_h2.step = h2step;
	pnp.motor_h2.chanset = (1 << 0);
	pnp.motor_h2.is_at_home = NULL;
	pnp.motor_h2.steps_min = PNP_STEPS_H_MIN;
	pnp.motor_h2.steps_max = PNP_STEPS_H_MAX;
	pnp.motor_h2.speed_max = PNP_SPEED_MAX_ZH;
	mdx_sem_init(&pnp.motor_h2.task.task_compl_sem, 0);

	error = pnp_thread_create("X Motor", &pnp.motor_x);
	if (error) {
		printf("%s: Failed to create X mover thread\n", __func__);
		return (-1);
	}

	error = pnp_thread_create("Y Motor", &pnp.motor_y);
	if (error) {
		printf("%s: Failed to create Y mover thread\n", __func__);
		return (-1);
	}

	error = pnp_thread_create("Z Motor", &pnp.motor_z);
	if (error) {
		printf("%s: Failed to create Z mover thread\n", __func__);
		return (-1);
	}

	error = pnp_thread_create("H1 Motor", &pnp.motor_h1);
	if (error) {
		printf("%s: Failed to create H1 mover thread\n", __func__);
		return (-1);
	}

	error = pnp_thread_create("H2 Motor", &pnp.motor_h2);
	if (error) {
		printf("%s: Failed to create H2 mover thread\n", __func__);
		return (-1);
	}

	pnp_xenable(1);
	pnp_yenable(1);
	pnp_zenable(1);
	pnp_henable(1);

	return (0);
}

static void
pnp_deinitialize(void)
{

	pnp_xenable(0);
	pnp_yenable(0);
	pnp_zenable(0);
	pnp_henable(0);
}

static void
pnp_test_heads(void)
{
	int i;

	printf("starting moving head\n");
	for (i = 0; i < 1; i++) {
		pnp_move(&pnp.motor_h1, 10000000);
		pnp_move(&pnp.motor_h2, 10000000);
		mdx_usleep(500000);

		pnp_move(&pnp.motor_h1, -10000000);
		pnp_move(&pnp.motor_h2, -10000000);
		mdx_usleep(500000);

		pnp_move(&pnp.motor_h1, 0);
		pnp_move(&pnp.motor_h2, 0);
		mdx_usleep(500000);
	}
	printf("head moving done\n");
}

static void __unused
pnp_move_random(void)
{
	uint32_t new_x;
	uint32_t new_y;
	int i;

	for (i = 0; i < 2; i++) {
		new_x = board_get_random() % PNP_MAX_X_NM;
		new_y = board_get_random() % PNP_MAX_Y_NM;
		printf("%d: moving xy to %u %u\n", i, new_x, new_y);
		pnp_move_xy(new_x, new_y);
		pnp_move(&pnp.motor_z, -10000000);
		pnp_move(&pnp.motor_z, 10000000);
		pnp_move(&pnp.motor_z, 0);
	}

	pnp_move_xy(0, 0);
}

static int
pnp_test_z(void)
{
	int error;
	int i;

	error = pnp_move_home_z(&pnp.motor_z);
	if (error)
		return (error);

	while (1) {
		for (i = 6; i < 17; i++) {
			pnp_move(&pnp.motor_z, i * 1000000);
			mdx_usleep(500000);
			mdx_usleep(500000);
		}
	}

	return (0);
}

int
pnp_main(void)
{
	int error;

	pnp_initialize();

	if (1 == 0) {
		pnp_test_heads();
		pnp_test_z();
		pnp_move_random();
	}

	/* Enter the GCode loop anyway, so that G28 can be retried. */
	error = pnp_home_all();
	if (error)
		printf("error: homing failed\n");

	gcode_mainloop();
	pnp_deinitialize();

	return (0);
}
