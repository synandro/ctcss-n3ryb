#ifndef EVENT_H
#define EVENT_H 1

typedef void EVH(void *);



struct ev_entry
{
        rb_dlink_node node;
        EVH *func;
        void *arg;
        uint32_t frequency;
        uint32_t when;
        uint16_t count; // counts back to zero, frees then
        bool counter;
        bool used;
};

struct ev_entry * rb_event_add(EVH * func, void *arg, uint32_t frequency, uint16_t count);

void rb_event_init(void);
void rb_event_run(void);
#endif