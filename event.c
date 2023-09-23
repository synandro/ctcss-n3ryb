/*
 * event.c - event loop stuff
 * 
 *  Borrowed from ircd-ratbox and adopted for AVR 
 *  Copyright (C) 2022 Aaron Sethman <androsyn@ratbox.org>
 * 
 *  The original boilerplate from ircd-ratbox
 *
 *  Copyright (C) 1998-2000 Regents of the University of California
 *  Copyright (C) 2001-2002 Hybrid Development Team
 *  Copyright (C) 2002-2012 ircd-ratbox development team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
 *  USA
 *

 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <stdio.h>
#include "tools.h"
#include "event.h"

#define MAX_EVENTS 10


uint32_t tick;
uint16_t tick_overflow;
static rb_dlink_list event_list;
static struct ev_entry event_heap[MAX_EVENTS];


static struct ev_entry *event_alloc(void)
{
	for(uint8_t i = 0; i < MAX_EVENTS; i++)
	{
		if(event_heap[i].used == false)
		{
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
rb_event_add(EVH * func, uint32_t frequency, uint16_t count)
{
	struct ev_entry *ev;
	uint32_t now;

	ev = event_alloc();

	if(ev == NULL) 
		return NULL; 

	ev->func = func;
	ev->count = count;
	if(count > 0)
		ev->counter = true;
	else
		ev->counter = false;

	ev->frequency = frequency;

	now = current_ts();

	ev->when = now;

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


static void rb_set_back_events(int32_t by)
{
	rb_dlink_node *ptr;
	struct ev_entry *ev;
	dprintf(PSTR("rb_set_back_events called\r\n"));
	tick_overflow++;
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
	int32_t now;

	now = current_ts();
	/* reset the counter before it gets anywhere near close to overflow */	
	if(now > (INT32_MAX / 2))
	{
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			rb_set_back_events((INT32_MAX / 2) + 1000);
			tick = tick - (INT32_MAX / 2);
			now = tick;
		}
	}

	RB_DLINK_FOREACH_SAFE(ptr, next, event_list.head)
	{
		ev = ptr->data;
		if(ev->when <= now)
		{
			ev->func();
			
			if((ev->counter == true && --ev->count == 0) || ev->frequency == 0)
			{
				rb_dlinkDelete(&ev->node, &event_list);
				event_free(ev);
			} else
			{
				ev->when = now + ev->frequency;
			}
		}
	}
}

void
rb_event_delete(struct ev_entry *ev)
{
        if(ev == NULL)
                return;

        rb_dlinkDelete(&ev->node, &event_list);
        event_free(ev);
}
