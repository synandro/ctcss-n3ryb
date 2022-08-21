/* 
 *  tools.c: Various functions needed here and there.
 *  
 *  Copyright (C) 2022 Aaron Sethman <androsyn@ratbox.org>
 *
 *  More stuff borrowed from ircd-ratbox, mostly.  
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

#include <stdint.h>
#include <stdlib.h>
#include <util/atomic.h>
#include <stdbool.h>
#include <string.h>
#include <avr/io.h>

#include "tools.h"

#include "event.h"


static buf_line_t linebuf_heap[LINEBUF_MAXLINES];


static buf_line_t *rb_linebuf_allocate(void)
{
	for(uint8_t i = 0; i < LINEBUF_MAXLINES; i++)
	{
		if(linebuf_heap[i].used == false)
		{
			memset(&linebuf_heap[i], 0, sizeof(linebuf_heap[i]));
			linebuf_heap[i].used = true;
			return &linebuf_heap[i];
		}
	}
	return NULL;
}

static void
rb_linebuf_free(buf_line_t * p)
{
	p->used = false;
}


buf_line_t *
rb_linebuf_new_line(buf_head_t * bufhead)
{
	buf_line_t *bufline;

	bufline = rb_linebuf_allocate();
	if(bufline == NULL)
		return NULL;


	/* Stick it at the end of the buf list */
	rb_dlinkAddTail(bufline, &bufline->node, &bufhead->list);

	return bufline;
}

static inline int
rb_linebuf_skip_crlf(char *ch, int16_t len)
{
	int16_t orig_len = len;

	/* First, skip until the first non-CRLF */
	for(; len; len--, ch++)
	{
		if(*ch == '\r')
			break;
		else if(*ch == '\n')
			break;
	}

	/* Then, skip until the last CRLF */
	for(; len; len--, ch++)
	{
		if((*ch != '\r') && (*ch != '\n'))
			break;
	}
	return (orig_len - len);
}




void
rb_linebuf_newbuf(buf_head_t * bufhead)
{
	/* not much to do right now :) */
	memset(bufhead, 0, sizeof(buf_head_t));
}

static int
rb_linebuf_copy_raw(buf_head_t * bufhead, buf_line_t * bufline, char *data, int16_t len)
{
	int16_t cpylen = 0;	 /* how many bytes we've copied */
	char *ch = data;	/* Pointer to where we are in the read data */
	char *bufch = bufline->buf + bufline->len;
	int16_t clen = 0;	   /* how many bytes we've processed,
				   and don't ever want to see again.. */

	/* If its full or terminated, ignore it */

	bufline->raw = 1;
	if(bufline->terminated == 1)
		return 0;

	clen = cpylen = rb_linebuf_skip_crlf(ch, len);
	if(clen == -1)
		return -1;

	/* This is the overflow case..This doesn't happen often.. */
	if(cpylen > (BUF_DATA_SIZE - bufline->len - 1))
	{
		clen = BUF_DATA_SIZE - bufline->len - 1;
		memcpy(bufch, ch, clen);
		bufline->buf[BUF_DATA_SIZE - 1] = '\0';
		bufch = bufline->buf + BUF_DATA_SIZE - 2;
		bufline->terminated = 1;
		bufline->len = BUF_DATA_SIZE - 1;
		bufhead->len += BUF_DATA_SIZE - 1;
		return clen;
	}

	memcpy(bufch, ch, cpylen);
	bufch += cpylen;
	*bufch = '\0';
	bufch--;

	if(*bufch != '\r' && *bufch != '\n' && *bufch != ';')
	{
		/* No linefeed, bail for the next time */
		bufhead->len += cpylen;
		bufline->len += cpylen;
		bufline->terminated = 0;
		return clen;
	}

	bufline->terminated = 1;
	bufhead->len += cpylen;
	bufline->len += cpylen;
	return clen;
}


static int
rb_linebuf_copy_line(buf_head_t * bufhead, buf_line_t * bufline, char *data, int16_t len)
{
	int16_t cpylen = 0;	 /* how many bytes we've copied */
	char *ch = data;	/* Pointer to where we are in the read data */
	char *bufch = bufline->buf + bufline->len;
	int16_t clen = 0;	   /* how many bytes we've processed,
				   and don't ever want to see again.. */

	/* If its full or terminated, ignore it */

	bufline->raw = 0;
	if(bufline->terminated == 1)
		return 0;

	clen = cpylen = rb_linebuf_skip_crlf(ch, len);
	if(clen == -1)
		return -1;

	/* This is the ~overflow case..This doesn't happen often.. */
	if(cpylen > (BUF_DATA_SIZE - bufline->len - 1))
	{
		size_t mcpylen = BUF_DATA_SIZE - bufline->len - 2;
		memcpy(bufch, ch, mcpylen);
		bufline->buf[BUF_DATA_SIZE - 1] = '\0';
		bufch = bufline->buf + BUF_DATA_SIZE - 2;
		while(cpylen && (*bufch == '\r' || *bufch == '\n'))
		{
			*bufch = '\0';
			cpylen--;
			bufch--;
		}
		bufline->terminated = 1;
		bufline->len = BUF_DATA_SIZE - 1;
		bufhead->len += BUF_DATA_SIZE - 1;
		return clen;
	}

	memcpy(bufch, ch, cpylen);
	bufch += cpylen;
	*bufch = '\0';
	bufch--;

	if(*bufch != '\r' && *bufch != '\n')
	{
		/* No linefeed, bail for the next time */
		bufhead->len += cpylen;
		bufline->len += cpylen;
		bufline->terminated = 0;
		return clen;
	}

	/* Yank the CRLF off this, replace with a \0 */
	while(cpylen && (*bufch == '\r' || *bufch == '\n'))
	{
		*bufch = '\0';
		cpylen--;
		bufch--;
	}

	bufline->terminated = 1;
	bufhead->len += cpylen;
	bufline->len += cpylen;
	return clen;
}

int
rb_linebuf_parse(buf_head_t * bufhead, char *data, int len, bool raw)
{
	buf_line_t *bufline;
	int16_t cpylen;
	int16_t linecnt = 0;
	/* First, if we have a partial buffer, try to squeze data into it */
	if(bufhead->list.tail != NULL)
	{
		/* Check we're doing the partial buffer thing */
		bufline = bufhead->list.tail->data;
		/* just try, the worst it could do is *reject* us .. */
		if(!raw)
			cpylen = rb_linebuf_copy_line(bufhead, bufline, data, len);
		else
			cpylen = rb_linebuf_copy_raw(bufhead, bufline, data, len);


		if(cpylen == -1)
			return -1;

		linecnt++;
		/* If we've copied the same as what we've got, quit now */
		if(cpylen == len)
			return linecnt; /* all the data done so soon? */

		/* Skip the data and update len .. */
		len -= cpylen;
		data += cpylen;
	}

	/* Next, the loop */
	while(len > 0)
	{
		/* We obviously need a new buffer, so .. */
		bufline = rb_linebuf_new_line(bufhead);

		if(bufline == NULL)
			break;

		/* And parse */
		if(!raw)
			cpylen = rb_linebuf_copy_line(bufhead, bufline, data, len);
		else
			cpylen = rb_linebuf_copy_raw(bufhead, bufline, data, len);

		if(cpylen == -1)
			return -1;

		len -= cpylen;
		data += cpylen;
		linecnt++;
	}
	return linecnt;
}

static void
rb_linebuf_done_line(buf_head_t * bufhead, buf_line_t * bufline, rb_dlink_node *node)
{
	/* Remove it from the linked list */
	rb_dlinkDelete(&bufline->node, &bufhead->list);
	/* Update the allocated size */
	bufhead->len -= bufline->len;
	rb_linebuf_free(bufline);
}


int
rb_linebuf_get(buf_head_t * bufhead, char *buf, int16_t buflen, bool partial, bool raw)
{
	buf_line_t *bufline;
	int16_t cpylen;
	char *start, *ch;

	/* make sure we have a line */
	if(bufhead->list.head == NULL)
		return 0;       /* Obviously not.. hrm. */

	bufline = bufhead->list.head->data;

	/* make sure that the buffer was actually *terminated */
	if(!(partial || bufline->terminated))
		return 0;       /* Wait for more data! */

	if(buflen < bufline->len)
		cpylen = buflen - 1;
	else
		cpylen = bufline->len;

	/* Copy it */
	start = bufline->buf;

	/* if we left extraneous '\r\n' characters in the string,
	 * and we don't want to read the raw data, clean up the string.
	 */
	if(bufline->raw && !raw)
	{
		/* skip leading EOL characters */
		while(cpylen && (*start == '\r' || *start == '\n'))
		{
			start++;
			cpylen--;
		}
		/* skip trailing EOL characters */
		ch = &start[cpylen - 1];
		while(cpylen && (*ch == '\r' || *ch == '\n'))
		{
			ch--;
			cpylen--;
		}
	}

	memcpy(buf, start, cpylen);

	/* convert CR/LF to NULL */
	if(!raw)
		buf[cpylen] = '\0';

	/* Deallocate the line */
	rb_linebuf_done_line(bufhead, bufline, bufhead->list.head);

	/* return how much we copied */
	return cpylen;
}


/* parv[0] == source, and parv[LAST] == NULL */
// static char *para[MAXPARA + 2];

int
rb_string_to_array(char *string, char **parv, int16_t maxpara)
{
        char *p, *xbuf = string;
        int16_t x = 0;

        parv[x] = NULL;

        if(string == NULL || string[0] == '\0')
                return x;

        while(*xbuf == ' ')     /* skip leading spaces */
                xbuf++;
        if(*xbuf == '\0')       /* ignore all-space args */
                return x;

        do
        {
                if(*xbuf == ':')        /* Last parameter */
                {
                        xbuf++;
                        parv[x++] = xbuf;
                        parv[x] = NULL;
                        return x;
                }
                else
                {
                        parv[x++] = xbuf;
                        parv[x] = NULL;
                        if((p = strchr(xbuf, ' ')) != NULL)
                        {
                                *p++ = '\0';
                                xbuf = p;
                        }
                        else
                                return x;
                }
                while(*xbuf == ' ')
                        xbuf++;
                if(*xbuf == '\0')
                        return x;
        }
        while(x < maxpara - 1);

        if(*p == ':')
                p++;

        parv[x++] = p;
        parv[x] = NULL;
        return x;
}

