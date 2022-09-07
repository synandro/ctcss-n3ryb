/* 
 *  tools.h: Various functions needed here and there.
 *  
 *  Copyright (C) 2022 Aaron Sethman <androsyn@ratbox.org>
 *
 *  More stuff borrowed from ircd-ratbox, mostly and adopted for AVR
 * 
 *  Copyright (C) 1996-2002 Hybrid Development Team
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
 */


#ifndef TOOLS_H
#define TOOLS_H 1


#define dprintf(...) printf_P(__VA_ARGS__)



/* static memory allocations for linebufs.  reduce these if you want to reduce SRAM memory footprint */
#define BUF_DATA_SIZE 64
#define LINEBUF_MAXLINES 6


#define rb_likely(x)       __builtin_expect(!!(x), 1)
#define rb_unlikely(x)     __builtin_expect(!!(x), 0)

//#define rb_unlikely

#define RB_DLINK_FOREACH(pos, head) for (pos = (head); pos != NULL; pos = pos->next)

/*
 * Walks forward of a list safely while removing nodes 
 * pos is your node
 * n is another list head for temporary storage
 * head is your list head
 */
#define RB_DLINK_FOREACH_SAFE(pos, n, head) for (pos = (head), n = pos ? pos->next : NULL; pos != NULL; pos = n, n = pos ? pos->next : NULL)

#define RB_DLINK_FOREACH_PREV(pos, head) for (pos = (head); pos != NULL; pos = pos->prev)


/* Returns the list length */
#define rb_dlink_list_length(list) (list)->length

#define rb_dlinkAddAlloc(data, list) rb_dlinkAdd(data, rb_make_rb_dlink_node(), list)
#define rb_dlinkAddTailAlloc(data, list) rb_dlinkAddTail(data, rb_make_rb_dlink_node(), list)
#define rb_dlinkDestroy(node, list) do { rb_dlinkDelete(node, list); rb_free_rb_dlink_node(node); } while(0)


typedef struct _rb_dlink_node rb_dlink_node;
typedef struct _rb_dlink_list rb_dlink_list;

struct _rb_dlink_node
{
	rb_dlink_node *prev;
	rb_dlink_node *next;
	void *data;
};

struct _rb_dlink_list
{
	rb_dlink_node *head;
	rb_dlink_node *tail;
	uint16_t length;
};


typedef struct _buf_line
{
	rb_dlink_node node;
	int16_t len;		/* How much data we've got */
	bool terminated;     /* Whether we've terminated the buffer */
	bool used; 
	bool raw;	    /* Whether this linebuf may hold 8-bit data */
	char buf[BUF_DATA_SIZE + 2];
} buf_line_t;

typedef struct _buf_head
{
	rb_dlink_list list;     /* the actual dlink list */
	int16_t len;		/* length of all the data */
	int16_t writeofs;	   /* offset in the first line for the write */
} buf_head_t;


static inline void
rb_dlinkDelete(rb_dlink_node *m, rb_dlink_list *list)
{
	/* Assumption: If m->next == NULL, then list->tail == m
	 *      and:   If m->prev == NULL, then list->head == m
	 */
	if(m->next)
		m->next->prev = m->prev;
	else
		list->tail = m->prev;

	if(m->prev)
		m->prev->next = m->next;
	else
		list->head = m->next;

	m->next = m->prev = NULL;
	list->length--;
}


static inline void
rb_dlinkAdd(void *data, rb_dlink_node *m, rb_dlink_list *list)
{
        m->data = data;
        m->prev = NULL;
        m->next = list->head;

        /* Assumption: If list->tail != NULL, list->head != NULL */
        if(list->head != NULL)
                list->head->prev = m;
        else if(list->tail == NULL)
                list->tail = m;

        list->head = m;
        list->length++;
}



static inline void
rb_dlinkAddTail(void *data, rb_dlink_node *m, rb_dlink_list *list)
{
	m->data = data;
	m->next = NULL;
	m->prev = list->tail;

	/* Assumption: If list->tail != NULL, list->head != NULL */
	if(list->tail != NULL)
		list->tail->next = m;
	else if(list->head == NULL)
		list->head = m;

	list->tail = m;
	list->length++;
}


buf_line_t *rb_linebuf_new_line(buf_head_t * bufhead);
void rb_linebuf_newbuf(buf_head_t * bufhead);

int rb_linebuf_parse(buf_head_t * bufhead, char *data, int len, bool raw);

int rb_linebuf_get(buf_head_t * bufhead, char *buf, int16_t buflen, bool partial, bool raw);

int rb_string_to_array(char *string, char **parv, int16_t maxpara);

#endif