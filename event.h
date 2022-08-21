/* 
 * event.h - definitions for event.c
 *
 */
#ifndef EVENT_H
#define EVENT_H 1
#include <util/atomic.h>


typedef void EVH(void *);

struct ev_entry
{
        rb_dlink_node node;
        EVH *func;
        void *arg;
        int32_t frequency;
        int32_t when;
        uint16_t count; // counts back to zero, frees then
        bool counter;
        bool used;
};

struct ev_entry * rb_event_add(EVH * func, void *arg, uint32_t frequency, uint16_t count);

void rb_event_init(void);
void rb_event_run(void);
void rb_event_delete(struct ev_entry *ev);


extern volatile uint32_t tick;

static inline __attribute__((always_inline)) uint32_t current_ts(void)
{
        uint32_t ts;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
                ts = tick;
        }
        return ts;
}
#endif
