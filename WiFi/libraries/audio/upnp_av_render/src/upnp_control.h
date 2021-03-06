/* upnp_control.h - UPnP RenderingControl definitions
 *
 * Copyright (C) 2005   Ivo Clarysse
 *
 * This file is part of GMediaRender.
 *
 * GMediaRender is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * GMediaRender is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Library General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GMediaRender; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, 
 * MA 02110-1301, USA.
 *
 */
#ifndef _UPNP_CONTROL_H
#define _UPNP_CONTROL_H

#include "variable-container.h"

struct upnp_device;
struct action_event;

void upnp_control_init(struct upnp_device *device);
void upnp_control_deinit(void);
struct service *upnp_control_get_service(void);
void upnp_control_register_variable_listener(variable_change_listener_t cb,
					     void *userdata);

int set_volume(struct action_event *event);
int get_volume(struct action_event *event);
#endif /* _UPNP_CONTROL_H */
