/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Reliable single-hop unicast example
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include <stdio.h>
#include <stdlib.h>

#include "contiki.h"
#include "net/rime/rime.h"

#include "lib/list.h"
#include "lib/memb.h"

#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "dev/serial-line.h"

#include "utils.h"

#define NUM_HISTORY_ENTRIES 4

/*---------------------------GLOBAL VARIABLES--------------------------------*/
static child_t** children = NULL;
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(sensor_process, "sensor_process");
PROCESS(serial_process, "serial_process");
AUTOSTART_PROCESSES(&sensor_process, &serial_process);
/*---------------------------------------------------------------------------*/
/* OPTIONAL: Sender history.
 * Detects duplicate callbacks at receiving nodes.
 * Duplicates appear when ack messages are lost. */
struct history_entry {
  struct history_entry *next;
  linkaddr_t addr;
  uint8_t seq;
};
LIST(history_table);
MEMB(history_mem, struct history_entry, NUM_HISTORY_ENTRIES);
/*---------------------------------------------------------------------------*/
static void
recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno)
{
  /* OPTIONAL: Sender history */
  struct history_entry *e = NULL;
  for(e = list_head(history_table); e != NULL; e = e->next) {
    if(linkaddr_cmp(&e->addr, from)) {
      break;
    }
  }
  if(e == NULL) {
    /* Create new history entry */
    e = memb_alloc(&history_mem);
    if(e == NULL) {
      e = list_chop(history_table); /* Remove oldest at full history */
    }
    linkaddr_copy(&e->addr, from);
    e->seq = seqno;
    list_push(history_table, e);
  } else {
    /* Detect duplicate callback */
    if(e->seq == seqno) {
      printf("runicast message received from %d.%d, seqno %d (DUPLICATE)\n",
      from->u8[0], from->u8[1], seqno);
      return;
    }
    /* Update existing history entry */
    e->seq = seqno;
  }
  /*
   * Pseudo-code :
   *  - Check if recv is child
   *    - If true : update timeout
   *    - If false : check if parent exist
   *      -If false : broadcast no more parent
   *      -If true : Add rcv to children if not parent
   *  - Check message type
   */
  child_t* child = is_mote_child(children, from);
  if (child != NULL){
    child->timeout = clock_seconds();
  }
  else{
    add_to_children(children, from);
  }

  packet_t* packet = (packet_t*) packetbuf_dataptr();
  int type = packet->type;
  switch (type){
    case DATA:
      printf("Data : ");
      data_t* data = (data_t*) packet;
      printf("%f : %d.%d\n", data->sensor_value, data->from.u8[0], data->from.u8[1]);
      break;
    case COMMAND:
      printf(" : Command\n");
      break;
    case ALIVE:
      break;
    case REQUEST:
      add_to_children(children, from);
      printf("Added to children : %d \n", from->u8[0]);
      break;
    case UNLINKED:
      printf(" : Unlinked\n");
      break;
    default:
      break;
  }
}
static void
sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions)
{
}
static void
timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions)
{
  printf("runicast message timed out when sending to %d.%d, retransmissions %d\n",
	 to->u8[0], to->u8[1], retransmissions);
}
static const struct runicast_callbacks runicast_callbacks = {recv_runicast,
							     sent_runicast,
							     timedout_runicast};
static struct runicast_conn runicast;

static void
recv_broadcast(struct broadcast_conn *c, const linkaddr_t *from) {
  update_child_timeout(children, from);
}
static const struct broadcast_callbacks broadcast_callbacks = {recv_broadcast};

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sensor_process, ev, data)
{
  /**
   *  Initialize children and databuffer
   *  Root has no parent if not server
   */
  if (children == NULL) {
    children = (child_t**) malloc(sizeof(child_t*) * CHILDREN_SIZE);
    if (children == NULL) {
      printf("Error allocating memory for children!\n");
    }
    int i;
    for(i = 0; i<CHILDREN_SIZE; i++){
      children[i] = NULL;
    }
    printf("Allocated memory for children\n");
  }

  PROCESS_EXITHANDLER(runicast_close(&runicast);)
  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

  PROCESS_BEGIN();

  runicast_open(&runicast, 144, &runicast_callbacks);
  broadcast_open(&broadcast, 129, &broadcast_callbacks);

  /* OPTIONAL: Sender history */
  list_init(history_table);
  memb_init(&history_mem);

  while(1) {
    static struct etimer et;

    remove_timedout_children(children);
    etimer_set(&et, 20*CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    alive_t alive;
    alive.type = ALIVE;
    alive.id = 0; // Root is id 0
    packetbuf_clear();
    packetbuf_copyfrom(&alive, sizeof(alive_t));
    broadcast_send(&broadcast);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(serial_process, ev, data)
{
  PROCESS_BEGIN();

  for(;;) {
     PROCESS_YIELD();
     if(ev == serial_line_event_message) {
       char* tmpmsg = malloc(strlen((char*)data));
       strcpy(tmpmsg,(char*)data);
       char * splittedMsg = strtok((char *)tmpmsg, " ");
       if(strcmp(splittedMsg, "message")==0){
         printf("borderinfo received line: %s\n", (char *)data);
       }
       free(tmpmsg);
     }
   }

  PROCESS_END();
}
