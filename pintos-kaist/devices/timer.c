#include "devices/timer.h"
#include <debug.h>
#include <inttypes.h>
#include <round.h>
#include <stdio.h>
#include "threads/interrupt.h"
#include "threads/io.h"
#include "threads/synch.h"
#include "threads/thread.h"
#include "lib/kernel/list.h"

/* See [8254] for hardware details of the 8254 timer chip. */

#if TIMER_FREQ < 19
#error 8254 timer requires TIMER_FREQ >= 19
#endif
#if TIMER_FREQ > 1000
#error TIMER_FREQ <= 1000 recommended
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))

/* Number of timer ticks since OS booted. */
static int64_t ticks; // tick? 하드웨어 타이머가 일정 주기로 발생시키는 신호이자, 특정한 시간 간격을 나타내는 단위
static int64_t global_tick = INT64_MAX;

/* Number of loops per timer tick.
   Initialized by timer_calibrate(). */
static unsigned loops_per_tick;

static intr_handler_func timer_interrupt;
static bool too_many_loops (unsigned loops);
static void busy_wait (int64_t loops);
static void real_time_sleep (int64_t num, int32_t denom);

/* Sets up the 8254 Programmable Interval Timer (PIT) to
   interrupt PIT_FREQ times per second, and registers the
   corresponding interrupt. */
// 타이머 칩 = 타이머 인터럽트를 주기적을 발생시키는 하드웨어
void 
timer_init (void) { // 타이머 칩의 타이머 인터럽트 주기를 초기화하는 함수, 주기적으로 호출되면서 ticks를 증가시킴
	/* 8254 input frequency divided by TIMER_FREQ, rounded to
	   nearest. */
	uint16_t count = (1193180 + TIMER_FREQ / 2) / TIMER_FREQ;
// outb는 주어진 포트 주소로 값을 전송하는 함수이다.
	outb (0x43, 0x34);    /* CW: counter 0, LSB then MSB, mode 2, binary. */
  // Ox43은 타이머 칩의 제어 레지스터를 나타내는 포트 주소
	outb (0x40, count & 0xff);
	outb (0x40, count >> 8);

	intr_register_ext (0x20, timer_interrupt, "8254 Timer");
  // 타이머 인터럽트와 예외 처리 핸들러를 연결시켜주는 함수, 인터럽트 번호와 핸들러 함수를 연결시켜준다.
  // 이 함수를 통해 타이머 칩이 인터럽트를 발생시킬 때마다, timer_interrupt()가 호출됨
}

/* Calibrates loops_per_tick, used to implement brief delays. */
void
timer_calibrate (void) {
	unsigned high_bit, test_bit;

	ASSERT (intr_get_level () == INTR_ON);
	printf ("Calibrating timer...  ");

	/* Approximate loops_per_tick as the largest power-of-two
	   still less than one timer tick. */
	loops_per_tick = 1u << 10;
	while (!too_many_loops (loops_per_tick << 1)) {
		loops_per_tick <<= 1;
		ASSERT (loops_per_tick != 0);
	}

	/* Refine the next 8 bits of loops_per_tick. */
	high_bit = loops_per_tick;
	for (test_bit = high_bit >> 1; test_bit != high_bit >> 10; test_bit >>= 1)
		if (!too_many_loops (high_bit | test_bit))
			loops_per_tick |= test_bit;

	printf ("%'"PRIu64" loops/s.\n", (uint64_t) loops_per_tick * TIMER_FREQ);
}

/* Returns the number of timer ticks since the OS booted. */
// OS가 부팅 되고 난 뒤부터 현재 시점까지의 틱수를 반환하는 함수
int64_t
timer_ticks (void) {
	enum intr_level old_level = intr_disable (); // 현재 인터럽트 상태를 저장하고, 인터럽트를 비활성화함 -> 틱을 읽어오는 도중에 인터럽트가 발생해서 ticks를 갱신하는 것을 막아줌
	int64_t t = ticks; // 현재 tick값을 읽어옴
	intr_set_level (old_level);// tick값을 가져왔으니, 인터럽트를 원래 수준으로 되돌림
	barrier (); // 컴파일 단계에서 이부분을 최적화(코드 재배치 = 순서 변경)하는 것을 방지해, 이 함수의 원자성을 보존해줌.
	return t;
}

/* Returns the number of timer ticks elapsed since THEN, which
   should be a value once returned by timer_ticks(). */
// 인자로 받은 then의 시점부터 현재까지의 시간 간격(tick)을 반환하는 함수
int64_t
timer_elapsed (int64_t then) {
	return timer_ticks () - then;
}

/* Suspends execution for approximately TICKS timer ticks. */
void
timer_sleep (int64_t ticks) {
  enum intr_level old_level = intr_disable ();
  int64_t start = timer_ticks();
  global_tick = MIN((global_tick), (start + ticks));
  thread_sleep (start+ticks);
  intr_set_level (old_level);
}

/* Suspends execution for approximately MS milliseconds. */
void
timer_msleep (int64_t ms) {
	real_time_sleep (ms, 1000);
}

/* Suspends execution for approximately US microseconds. */
void
timer_usleep (int64_t us) {
	real_time_sleep (us, 1000 * 1000);
}

/* Suspends execution for approximately NS nanoseconds. */
void
timer_nsleep (int64_t ns) {
	real_time_sleep (ns, 1000 * 1000 * 1000);
}

/* Prints timer statistics. */
void
timer_print_stats (void) {
	printf ("Timer: %"PRId64" ticks\n", timer_ticks ());
}

/* Timer interrupt handler. */
static void
timer_interrupt (struct intr_frame *args UNUSED) {
	ticks++;
	thread_tick ();
  if (global_tick < ticks)
  {
    wake_up(ticks);
  }
  
}

/* Returns true if LOOPS iterations waits for more than one timer
   tick, otherwise false. */
static bool
too_many_loops (unsigned loops) {
	/* Wait for a timer tick. */
	int64_t start = ticks;
	while (ticks == start)
		barrier ();

	/* Run LOOPS loops. */
	start = ticks;
	busy_wait (loops);

	/* If the tick count changed, we iterated too long. */
	barrier ();
	return start != ticks;
}

/* Iterates through a simple loop LOOPS times, for implementing
   brief delays.

   Marked NO_INLINE because code alignment can significantly
   affect timings, so that if this function was inlined
   differently in different places the results would be difficult
   to predict. */
static void NO_INLINE
busy_wait (int64_t loops) {
	while (loops-- > 0)
		barrier ();
}

/* Sleep for approximately NUM/DENOM seconds. */
static void
real_time_sleep (int64_t num, int32_t denom) {
	/* Convert NUM/DENOM seconds into timer ticks, rounding down.

	   (NUM / DENOM) s
	   ---------------------- = NUM * TIMER_FREQ / DENOM ticks.
	   1 s / TIMER_FREQ ticks
	   */
	int64_t ticks = num * TIMER_FREQ / denom;

	ASSERT (intr_get_level () == INTR_ON);
	if (ticks > 0) {
		/* We're waiting for at least one full timer tick.  Use
		   timer_sleep() because it will yield the CPU to other
		   processes. */
		timer_sleep (ticks);
	} else {
		/* Otherwise, use a busy-wait loop for more accurate
		   sub-tick timing.  We scale the numerator and denominator
		   down by 1000 to avoid the possibility of overflow. */
		ASSERT (denom % 1000 == 0);
		busy_wait (loops_per_tick * num / 1000 * TIMER_FREQ / (denom / 1000));
	}
}

