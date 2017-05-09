/******************************************************************************
 *
 *  Copyright (C) 2014 Google, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#define LOG_TAG "bt_hci"

#include "hci_layer.h"

#include <base/logging.h>
#include <signal.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <mutex>

#include "btcore/include/module.h"
#include "btsnoop.h"
#include "buffer_allocator.h"
#include "hci_inject.h"
#include "hci_internals.h"
#include "hcidefs.h"
#include "hcimsgs.h"
#include "osi/include/alarm.h"
#include "osi/include/list.h"
#include "osi/include/log.h"
#include "osi/include/properties.h"
#include "osi/include/reactor.h"
#include "packet_fragmenter.h"

#define BT_HCI_TIMEOUT_TAG_NUM 1010000

extern void hci_initialize();
extern void hci_transmit(BT_HDR* packet);
extern void hci_close();

typedef struct {
  uint16_t opcode;
  future_t* complete_future;
  command_complete_cb complete_callback;
  command_status_cb status_callback;
  void* context;
  BT_HDR* command;
  std::chrono::time_point<std::chrono::steady_clock> timestamp;
} waiting_command_t;

// Using a define here, because it can be stringified for the property lookup
#define DEFAULT_STARTUP_TIMEOUT_MS 8000
#define STRING_VALUE_OF(x) #x

// Abort if there is no response to an HCI command.
static const uint32_t COMMAND_PENDING_TIMEOUT_MS = 2000;

// Our interface
static bool interface_created;
static hci_t interface;

// Modules we import and callbacks we export
static const allocator_t* buffer_allocator;
static const btsnoop_t* btsnoop;
static const packet_fragmenter_t* packet_fragmenter;

static future_t* startup_future;
static thread_t* thread;  // We own this

static alarm_t* startup_timer;

// Outbound-related
static int command_credits = 1;
static fixed_queue_t* command_queue;
static fixed_queue_t* packet_queue;

// Inbound-related
static alarm_t* command_response_timer;
static list_t* commands_pending_response;
static std::recursive_mutex commands_pending_response_mutex;

// The hand-off point for data going to a higher layer, set by the higher layer
static fixed_queue_t* upwards_data_queue;

static bool filter_incoming_event(BT_HDR* packet);
static waiting_command_t* get_waiting_command(command_opcode_t opcode);

static void event_finish_startup(void* context);
static void startup_timer_expired(void* context);

static void event_command_ready(fixed_queue_t* queue, void* context);
static void event_packet_ready(fixed_queue_t* queue, void* context);
static void command_timed_out(void* context);

static void update_command_response_timer(void);

static void transmit_fragment(BT_HDR* packet, bool send_transmit_finished);
static void dispatch_reassembled(BT_HDR* packet);
static void fragmenter_transmit_finished(BT_HDR* packet,
                                         bool all_fragments_sent);

static const packet_fragmenter_callbacks_t packet_fragmenter_callbacks = {
    transmit_fragment, dispatch_reassembled, fragmenter_transmit_finished};

void initialization_complete() {
  thread_post(thread, event_finish_startup, NULL);
}

void hci_event_received(BT_HDR* packet) {
  btsnoop->capture(packet, true);

  if (!filter_incoming_event(packet)) {
    data_dispatcher_dispatch(interface.event_dispatcher, packet->data[0],
                             packet);
  }
}

void acl_event_received(BT_HDR* packet) {
  btsnoop->capture(packet, true);
  packet_fragmenter->reassemble_and_dispatch(packet);
}

void sco_data_received(BT_HDR* packet) {
  btsnoop->capture(packet, true);
  packet_fragmenter->reassemble_and_dispatch(packet);
}

// Module lifecycle functions

static future_t* hci_module_shut_down();

static future_t* hci_module_start_up(void) {
  LOG_INFO(LOG_TAG, "%s", __func__);

  // The host is only allowed to send at most one command initially,
  // as per the Bluetooth spec, Volume 2, Part E, 4.4 (Command Flow Control)
  // This value can change when you get a command complete or command status
  // event.
  command_credits = 1;

  // For now, always use the default timeout on non-Android builds.
  period_ms_t startup_timeout_ms = DEFAULT_STARTUP_TIMEOUT_MS;

  // Grab the override startup timeout ms, if present.
  char timeout_prop[PROPERTY_VALUE_MAX];
  if (!osi_property_get("bluetooth.enable_timeout_ms", timeout_prop,
                        STRING_VALUE_OF(DEFAULT_STARTUP_TIMEOUT_MS)) ||
      (startup_timeout_ms = atoi(timeout_prop)) < 100)
    startup_timeout_ms = DEFAULT_STARTUP_TIMEOUT_MS;

  startup_timer = alarm_new("hci.startup_timer");
  if (!startup_timer) {
    LOG_ERROR(LOG_TAG, "%s unable to create startup timer.", __func__);
    goto error;
  }

  command_response_timer = alarm_new("hci.command_response_timer");
  if (!command_response_timer) {
    LOG_ERROR(LOG_TAG, "%s unable to create command response timer.", __func__);
    goto error;
  }

  command_queue = fixed_queue_new(SIZE_MAX);
  if (!command_queue) {
    LOG_ERROR(LOG_TAG, "%s unable to create pending command queue.", __func__);
    goto error;
  }

  packet_queue = fixed_queue_new(SIZE_MAX);
  if (!packet_queue) {
    LOG_ERROR(LOG_TAG, "%s unable to create pending packet queue.", __func__);
    goto error;
  }

  thread = thread_new("hci_thread");
  if (!thread) {
    LOG_ERROR(LOG_TAG, "%s unable to create thread.", __func__);
    goto error;
  }

  commands_pending_response = list_new(NULL);
  if (!commands_pending_response) {
    LOG_ERROR(LOG_TAG,
              "%s unable to create list for commands pending response.",
              __func__);
    goto error;
  }

  // Make sure we run in a bounded amount of time
  future_t* local_startup_future;
  local_startup_future = future_new();
  startup_future = local_startup_future;
  alarm_set(startup_timer, startup_timeout_ms, startup_timer_expired, NULL);

  packet_fragmenter->init(&packet_fragmenter_callbacks);

  fixed_queue_register_dequeue(command_queue, thread_get_reactor(thread),
                               event_command_ready, NULL);
  fixed_queue_register_dequeue(packet_queue, thread_get_reactor(thread),
                               event_packet_ready, NULL);

  hci_initialize();

  LOG_DEBUG(LOG_TAG, "%s starting async portion", __func__);
  return local_startup_future;

error:
  hci_module_shut_down();  // returns NULL so no need to wait for it
  return future_new_immediate(FUTURE_FAIL);
}

static future_t* hci_module_shut_down() {
  LOG_INFO(LOG_TAG, "%s", __func__);

  // Free the timers
  {
    std::lock_guard<std::recursive_mutex> lock(commands_pending_response_mutex);
    alarm_free(command_response_timer);
    command_response_timer = NULL;
    alarm_free(startup_timer);
    startup_timer = NULL;
  }

  // Stop the thread to prevent Send() calls.
  if (thread) {
    thread_stop(thread);
    thread_join(thread);
  }

  // Close HCI to prevent callbacks.
  hci_close();

  fixed_queue_free(command_queue, osi_free);
  command_queue = NULL;
  fixed_queue_free(packet_queue, buffer_allocator->free);
  packet_queue = NULL;
  {
    std::lock_guard<std::recursive_mutex> lock(commands_pending_response_mutex);
    list_free(commands_pending_response);
    commands_pending_response = NULL;
  }

  packet_fragmenter->cleanup();

  thread_free(thread);
  thread = NULL;

  return NULL;
}

EXPORT_SYMBOL extern const module_t hci_module = {
    .name = HCI_MODULE,
    .init = NULL,
    .start_up = hci_module_start_up,
    .shut_down = hci_module_shut_down,
    .clean_up = NULL,
    .dependencies = {BTSNOOP_MODULE, NULL}};

// Interface functions

static void set_data_queue(fixed_queue_t* queue) { upwards_data_queue = queue; }

static void transmit_command(BT_HDR* command,
                             command_complete_cb complete_callback,
                             command_status_cb status_callback, void* context) {
  waiting_command_t* wait_entry = reinterpret_cast<waiting_command_t*>(
      osi_calloc(sizeof(waiting_command_t)));

  uint8_t* stream = command->data + command->offset;
  STREAM_TO_UINT16(wait_entry->opcode, stream);
  wait_entry->complete_callback = complete_callback;
  wait_entry->status_callback = status_callback;
  wait_entry->command = command;
  wait_entry->context = context;

  // Store the command message type in the event field
  // in case the upper layer didn't already
  command->event = MSG_STACK_TO_HC_HCI_CMD;

  fixed_queue_enqueue(command_queue, wait_entry);
}

static future_t* transmit_command_futured(BT_HDR* command) {
  waiting_command_t* wait_entry = reinterpret_cast<waiting_command_t*>(
      osi_calloc(sizeof(waiting_command_t)));
  future_t* future = future_new();

  uint8_t* stream = command->data + command->offset;
  STREAM_TO_UINT16(wait_entry->opcode, stream);
  wait_entry->complete_future = future;
  wait_entry->command = command;

  // Store the command message type in the event field
  // in case the upper layer didn't already
  command->event = MSG_STACK_TO_HC_HCI_CMD;

  fixed_queue_enqueue(command_queue, wait_entry);
  return future;
}

static void transmit_downward(data_dispatcher_type_t type, void* data) {
  if (type == MSG_STACK_TO_HC_HCI_CMD) {
    // TODO(zachoverflow): eliminate this call
    transmit_command((BT_HDR*)data, NULL, NULL, NULL);
    LOG_WARN(LOG_TAG,
             "%s legacy transmit of command. Use transmit_command instead.",
             __func__);
  } else {
    fixed_queue_enqueue(packet_queue, data);
  }
}

// Start up functions

static void event_finish_startup(UNUSED_ATTR void* context) {
  LOG_INFO(LOG_TAG, "%s", __func__);
  std::lock_guard<std::recursive_mutex> lock(commands_pending_response_mutex);
  alarm_cancel(startup_timer);
  future_ready(startup_future, FUTURE_SUCCESS);
  startup_future = NULL;
}

static void startup_timer_expired(UNUSED_ATTR void* context) {
  LOG_ERROR(LOG_TAG, "%s", __func__);

  std::lock_guard<std::recursive_mutex> lock(commands_pending_response_mutex);
  future_ready(startup_future, FUTURE_FAIL);
  startup_future = NULL;
}

// Command/packet transmitting functions

static void event_command_ready(fixed_queue_t* queue,
                                UNUSED_ATTR void* context) {
  if (command_credits > 0) {
    waiting_command_t* wait_entry =
        reinterpret_cast<waiting_command_t*>(fixed_queue_dequeue(queue));
    command_credits--;

    // Move it to the list of commands awaiting response
    {
      std::lock_guard<std::recursive_mutex> lock(
          commands_pending_response_mutex);
      wait_entry->timestamp = std::chrono::steady_clock::now();
      list_append(commands_pending_response, wait_entry);

      // Send it off
      packet_fragmenter->fragment_and_dispatch(wait_entry->command);

      update_command_response_timer();
    }
  }
}

static void event_packet_ready(fixed_queue_t* queue,
                               UNUSED_ATTR void* context) {
  // The queue may be the command queue or the packet queue, we don't care
  BT_HDR* packet = (BT_HDR*)fixed_queue_dequeue(queue);

  packet_fragmenter->fragment_and_dispatch(packet);
}

// Callback for the fragmenter to send a fragment
static void transmit_fragment(BT_HDR* packet, bool send_transmit_finished) {
  btsnoop->capture(packet, false);

  hci_transmit(packet);

  uint16_t event = packet->event & MSG_EVT_MASK;
  if (event != MSG_STACK_TO_HC_HCI_CMD && send_transmit_finished)
    buffer_allocator->free(packet);
}

static void fragmenter_transmit_finished(BT_HDR* packet,
                                         bool all_fragments_sent) {
  if (all_fragments_sent) {
    buffer_allocator->free(packet);
  } else {
    // This is kind of a weird case, since we're dispatching a partially sent
    // packet up to a higher layer.
    // TODO(zachoverflow): rework upper layer so this isn't necessary.
    data_dispatcher_dispatch(interface.event_dispatcher,
                             packet->event & MSG_EVT_MASK, packet);
  }
}

static void command_timed_out(UNUSED_ATTR void* context) {
  std::unique_lock<std::recursive_mutex> lock(commands_pending_response_mutex);

  if (list_is_empty(commands_pending_response)) {
    LOG_ERROR(LOG_TAG, "%s with no commands pending response", __func__);
  } else {
    waiting_command_t* wait_entry = reinterpret_cast<waiting_command_t*>(
        list_front(commands_pending_response));

    int wait_time_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - wait_entry->timestamp)
            .count();
    // We shouldn't try to recover the stack from this command timeout.
    // If it's caused by a software bug, fix it. If it's a hardware bug, fix it.
    LOG_ERROR(LOG_TAG, "%s: Waited %d ms for a response to opcode: 0x%x",
              __func__, wait_time_ms, wait_entry->opcode);
    LOG_EVENT_INT(BT_HCI_TIMEOUT_TAG_NUM, wait_entry->opcode);
    lock.unlock();
  }

  LOG_ERROR(LOG_TAG, "%s restarting the bluetooth process.", __func__);
  usleep(10000);
  abort();
}

// Event/packet receiving functions

// Returns true if the event was intercepted and should not proceed to
// higher layers. Also inspects an incoming event for interesting
// information, like how many commands are now able to be sent.
static bool filter_incoming_event(BT_HDR* packet) {
  waiting_command_t* wait_entry = NULL;
  uint8_t* stream = packet->data;
  uint8_t event_code;
  command_opcode_t opcode;

  STREAM_TO_UINT8(event_code, stream);
  STREAM_SKIP_UINT8(stream);  // Skip the parameter total length field

  if (event_code == HCI_COMMAND_COMPLETE_EVT) {
    STREAM_TO_UINT8(command_credits, stream);
    STREAM_TO_UINT16(opcode, stream);

    wait_entry = get_waiting_command(opcode);
    if (!wait_entry) {
      // TODO: Currently command_credits aren't parsed at all; here or in higher
      // layers...
      if (opcode != HCI_COMMAND_NONE) {
        LOG_WARN(LOG_TAG,
                 "%s command complete event with no matching command (opcode: "
                 "0x%04x).",
                 __func__, opcode);
      }
    } else {
      update_command_response_timer();
      if (wait_entry->complete_callback) {
        wait_entry->complete_callback(packet, wait_entry->context);
      } else if (wait_entry->complete_future) {
        future_ready(wait_entry->complete_future, packet);
      }
    }

    goto intercepted;
  } else if (event_code == HCI_COMMAND_STATUS_EVT) {
    uint8_t status;
    STREAM_TO_UINT8(status, stream);
    STREAM_TO_UINT8(command_credits, stream);
    STREAM_TO_UINT16(opcode, stream);

    // If a command generates a command status event, it won't be getting a
    // command complete event

    wait_entry = get_waiting_command(opcode);
    if (!wait_entry) {
      LOG_WARN(
          LOG_TAG,
          "%s command status event with no matching command. opcode: 0x%04x",
          __func__, opcode);
    } else {
      update_command_response_timer();
      if (wait_entry->status_callback)
        wait_entry->status_callback(status, wait_entry->command,
                                    wait_entry->context);
    }

    goto intercepted;
  }

  return false;

intercepted:
  if (wait_entry) {
    // If it has a callback, it's responsible for freeing the packet
    if (event_code == HCI_COMMAND_STATUS_EVT ||
        (!wait_entry->complete_callback && !wait_entry->complete_future))
      buffer_allocator->free(packet);

    // If it has a callback, it's responsible for freeing the command
    if (event_code == HCI_COMMAND_COMPLETE_EVT || !wait_entry->status_callback)
      buffer_allocator->free(wait_entry->command);

    osi_free(wait_entry);
  } else {
    buffer_allocator->free(packet);
  }

  return true;
}

// Callback for the fragmenter to dispatch up a completely reassembled packet
static void dispatch_reassembled(BT_HDR* packet) {
  // Events should already have been dispatched before this point
  CHECK((packet->event & MSG_EVT_MASK) != MSG_HC_TO_STACK_HCI_EVT);
  CHECK(upwards_data_queue != NULL);

  fixed_queue_enqueue(upwards_data_queue, packet);
}

// Misc internal functions

static waiting_command_t* get_waiting_command(command_opcode_t opcode) {
  std::lock_guard<std::recursive_mutex> lock(commands_pending_response_mutex);

  for (const list_node_t* node = list_begin(commands_pending_response);
       node != list_end(commands_pending_response); node = list_next(node)) {
    waiting_command_t* wait_entry =
        reinterpret_cast<waiting_command_t*>(list_node(node));

    if (!wait_entry || wait_entry->opcode != opcode) continue;

    list_remove(commands_pending_response, wait_entry);

    return wait_entry;
  }

  return NULL;
}

static void update_command_response_timer(void) {
  std::lock_guard<std::recursive_mutex> lock(commands_pending_response_mutex);

  if (command_response_timer == NULL) return;
  if (list_is_empty(commands_pending_response)) {
    alarm_cancel(command_response_timer);
  } else {
    alarm_set(command_response_timer, COMMAND_PENDING_TIMEOUT_MS,
              command_timed_out, NULL);
  }
}

static void init_layer_interface() {
  if (!interface_created) {
    // It's probably ok for this to live forever. It's small and
    // there's only one instance of the hci interface.
    interface.event_dispatcher = data_dispatcher_new("hci_layer");
    if (!interface.event_dispatcher) {
      LOG_ERROR(LOG_TAG, "%s could not create upward dispatcher.", __func__);
      return;
    }

    interface.set_data_queue = set_data_queue;
    interface.transmit_command = transmit_command;
    interface.transmit_command_futured = transmit_command_futured;
    interface.transmit_downward = transmit_downward;
    interface_created = true;
  }
}

void hci_layer_cleanup_interface() {
  if (interface_created) {
    data_dispatcher_free(interface.event_dispatcher);
    interface.event_dispatcher = NULL;

    interface.set_data_queue = NULL;
    interface.transmit_command = NULL;
    interface.transmit_command_futured = NULL;
    interface.transmit_downward = NULL;
    interface_created = false;
  }
}

const hci_t* hci_layer_get_interface() {
  buffer_allocator = buffer_allocator_get_interface();
  btsnoop = btsnoop_get_interface();
  packet_fragmenter = packet_fragmenter_get_interface();

  init_layer_interface();

  return &interface;
}

const hci_t* hci_layer_get_test_interface(
    const allocator_t* buffer_allocator_interface,
    const btsnoop_t* btsnoop_interface,
    const packet_fragmenter_t* packet_fragmenter_interface) {
  buffer_allocator = buffer_allocator_interface;
  btsnoop = btsnoop_interface;
  packet_fragmenter = packet_fragmenter_interface;

  init_layer_interface();
  return &interface;
}
