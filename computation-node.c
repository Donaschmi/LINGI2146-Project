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

#include "utils.h"

#define NUM_HISTORY_ENTRIES 4

/*---------------------------GLOBAL VARIABLES--------------------------------*/
static child_t** children = NULL;
static parent_t* parent = NULL;
static value_t** values = NULL;
static node_t** head = NULL;
static int slots = 0;
static uint8_t forwarding = 1; // Is this node forwarding data to parent ?
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(sensor_process, "sensor_process");
AUTOSTART_PROCESSES(&sensor_process);
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
  struct history_entry *e = NULL;
  for(e = list_head(history_table); e != NULL; e = e->next) {
    if(linkaddr_cmp(&e->addr, from)) {
      break;
    }
  }
  if(e == NULL) {
    e = memb_alloc(&history_mem);
    if(e == NULL) {
      e = list_chop(history_table);
    }
    linkaddr_copy(&e->addr, from);
    e->seq = seqno;
    list_push(history_table, e);
  } else {
    if(e->seq == seqno) {
      printf("runicast message received from %d.%d, seqno %d (DUPLICATE)\n",
       from->u8[0], from->u8[1], seqno);
      return;
    }
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
  update_child_timeout(children, from);
  child_t* child = is_mote_child(children, from);
  if (child == NULL){
    if (parent != NULL){
      if (!linkaddr_cmp(&parent->addr, from)){
        add_to_children(children, from);
      }
    }
    else{
      send_unlinked(children);
    }
  }
  packet_t* packet = (packet_t*) packetbuf_dataptr();
  int type = packet->type;
  switch (type){
    case DATA: ;
      data_t* data = (data_t*) packet;
      if (get_forward_addr(head, &data->from) == NULL){
        node_t* node = add_to_table(head, &data->from, from);
        if (node != NULL)
          printf("Added entry\n");
      }

      printf("Received data from %d\n", data->from.u8[0]);
      value_t* value = is_computing_child(values, &data->from);
      if (value != NULL){
        update_sensor_data(value, packet);
        if (value->count == 3){ // TODO CHANGE TO 30
          printf("Computing\n");
          // Compute
          if (least_squares(value)){
            send_open_valve_command(child, from);
          }
        }
        else // Do nothing
          printf("Added data for child %d, count : %d\n",value->addr.u8[0], value->count);
      }
      else{
      // If less than 5 sensors to compute, compute, else forward
        if (slots < MAX_COMPUTATION_PER_SENSOR){
          forwarding = 0; // If we are still adding computation slot then we are not forwarding data to parent
          printf("Empry slot available\n");
          value_t* v = add_to_computing(values, &data->from);
          if(v == NULL)
            printf("Failed to add new value_t\n");
          else
            update_sensor_data(v, packet);
          // We don't compute cause only one value inside
        }
        else{
          printf("No slot for %d\n", from->u8[0]);
          forwarding = 1;
          forward_parent(packet, &parent->addr);
        }
      }
      break;
    case COMMAND: ;
      command_t* command = (command_t*) packet;
      linkaddr_t* next = get_forward_addr(head, &command->dest);
      if (next == NULL)
        printf("No entry for this destination\n");
      else{ //Forward
        packetbuf_clear();
        packetbuf_copyfrom(packet, sizeof(command_t));
        runicast_send(&runicast, next, MAX_RETRANSMISSIONS);
      }
      break;
    case ALIVE:
      break;
    case REQUEST:
      add_to_children(children, from);
      break;
    case UNLINKED:
      break;
    default:
      break;
  }
}
static void
sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions)
{
  if (no_parent(parent)){
    parent->addr.u8[0] = to->u8[0];
    parent->addr.u8[1] = to->u8[1];
    printf("New parent : %d\n", to->u8[0]);
  }
}
static void
timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions)
{
  if (linkaddr_cmp(&parent->addr, to)){
    // Send unlinked broadcast 
    parent = NULL;
    send_unlinked(children);
  }
  // If we get timeout from parent, the link is dead and we should try to find a new parent
}
static const struct runicast_callbacks runicast_callbacks = {recv_runicast,
							     sent_runicast,
							     timedout_runicast};
static struct runicast_conn runicast;

static void
recv_broadcast(struct broadcast_conn *c, const linkaddr_t *from) {
  /*
   * Logic :
   *  -Is From child?
   *    -If true : update last seen
   *  - Check message type
   *    -If no parent or better hops : send request to parent
   *    -If parent_dead : set parent to null and warn children
   */
  update_child_timeout(children, from);
  packet_t* packet = (packet_t*) packetbuf_dataptr();
  int type = packet->type;
  switch (type){
    case DATA:
      break;
    case COMMAND:
      break;
    case ALIVE: ;
      // Do i have a parent ? If not this one can be.
      alive_t* pkt = (alive_t*) packet;
      if (no_parent(parent)){
        // send request to be child to "from"
        send_request(parent, pkt->id, from);
      } else if (pkt->id < parent->id){
        send_request(parent, pkt->id, from);
      }
      break;
    case REQUEST:
      break;
    case UNLINKED:
      if (linkaddr_cmp(&parent->addr, from)){
        parent = NULL;
        send_unlinked(children);
      }
      break;
    default:
      break;
  }
}
static const struct broadcast_callbacks broadcast_callbacks = {recv_broadcast};

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sensor_process, ev, data)
{

  /**
   *  Initialize parent and children and databuffer
   */
  if (parent == NULL){
    parent = malloc(sizeof(parent_t));
    if (parent == NULL){
      printf("Error allocating memory for parent!\n");
    }
    parent->addr.u8[0] = 0;
    parent->addr.u8[1] = 0;
    parent->RSSI = 0;
    parent->id = UINT8_MAX;
  }

  if (children == NULL) {
    children = (child_t**) malloc(sizeof(child_t*) * CHILDREN_SIZE);
    if (children == NULL) {
      printf("Error allocating memory for children!\n");
    }
    int i;
    for(i = 0; i<CHILDREN_SIZE; i++){
      children[i] = NULL;
    }
  }

  if (values == NULL) {
    values = (value_t**) malloc(sizeof(value_t*) * MAX_COMPUTATION_PER_SENSOR);
    if (values == NULL)
      printf("Error allocating memory for computation sensor\n");
    int i;
    for(i=0; i < MAX_COMPUTATION_PER_SENSOR; i++){
      values[i] = NULL;
    }
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

    etimer_set(&et, 60*CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    if (!no_parent(parent)){
      remove_timedout_children(children);
      remove_timedout_sensors(values);
      // Broadcast
      alive_t alive;
      alive.type = ALIVE;
      alive.id = parent->id + 1;
      packetbuf_clear();
      packetbuf_copyfrom(&alive, sizeof(alive_t));
      broadcast_send(&broadcast);
      
      if (!forwarding){
        packetbuf_clear();
        alive.id = 0;
        packetbuf_copyfrom(&alive, sizeof(alive_t));
        runicast_send(&runicast, &parent->addr, MAX_RETRANSMISSIONS);
        printf("Send alive to parent\n");
      }
      
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
