#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <stdbool.h>
#include <string.h>
#include <avr/io.h>
//#include <util/setbaud.h>
#include <stdio.h>

#include "tools.h"

#include "event.h"

#define MAX_EVENTS 12


volatile uint32_t tick;

static rb_dlink_list event_list;
static uint32_t event_time_min;
static struct ev_entry event_heap[MAX_EVENTS];

static struct ev_entry *event_alloc(void)
{
	for(uint8_t i = 0; i < MAX_EVENTS; i++)
	{
		if(event_heap[i].used == false)
		{
			memset(&event_heap[i], 0, sizeof(event_heap[i]));
			event_heap[i].used = true;
			return &event_heap[i];
		}
	}
	return NULL;
}

static void event_free(struct ev_entry *ev)
{
	ev->used = false;
}

struct ev_entry *
rb_event_add(EVH * func, void *arg, uint32_t frequency, uint16_t count)
{
	struct ev_entry *ev;
	uint32_t now;

	ev = event_alloc();

	if(ev == NULL) 
		return NULL; 

	ev->func = func;
	ev->arg = arg;
	ev->count = count;
	if(count > 0)
		ev->counter = true;
	ev->frequency = frequency;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		now = tick;
	}
	ev->when = now;

	if((ev->when < event_time_min) || (event_time_min == -1))
	{
		event_time_min = ev->when;
	}

	rb_dlinkAdd(ev, &ev->node, &event_list);
	return ev;
}

ISR(TIMER1_COMPA_vect)
{
	tick++;	
}


void rb_event_init()
{
	TCCR1B = _BV(WGM12) | _BV(CS10);  
	TIMSK1 = _BV(OCIE1A);
	OCR1A = F_CPU / 1000;
}


static void
rb_set_back_events(uint32_t by)
{
	rb_dlink_node *ptr;
	struct ev_entry *ev;
	RB_DLINK_FOREACH(ptr, event_list.head)
	{
		ev = ptr->data;
		if(ev->when > by)
			ev->when -= by;
		else
			ev->when = 0;
	}
}


void rb_event_run(void)
{
	rb_dlink_node *ptr, *next;
	struct ev_entry *ev;
	uint32_t now;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		now = tick;
	}

	/* reset the counter before it gets anywhere near close to overflow */	
	if(now > INT32_MAX) 
	{
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tick = tick - INT32_MAX;
		}
		rb_set_back_events(INT32_MAX); 
	}

	RB_DLINK_FOREACH_SAFE(ptr, next, event_list.head)
	{
		ev = ptr->data;
		if(ev->when <= now)
		{
			ev->func(ev->arg);
			
			if((ev->counter == true && --ev->count == 0) || ev->frequency == 0)
			{
				rb_dlinkDelete(&ev->node, &event_list);
				event_free(ev);
			} else
			{
				ev->when = now + ev->frequency;
			}
		}
		else
		{
			if((ev->when < event_time_min) || (event_time_min == -1))
				event_time_min = ev->when;
		}
	}
}
