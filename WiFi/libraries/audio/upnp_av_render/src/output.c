/* output.c - Output module frontend
 *
 * Copyright (C) 2007 Ivo Clarysse,  (C) 2012 Henner Zeller
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "wiced.h"

#include "logging.h"
#include "output_module.h"
#include "output_wiced_audio.h"
#include "output.h"

static wiced_semaphore_t output_semaphore;

static struct output_module *modules[] = {
    &wiced_audio_output
};

static struct output_module *output_module = NULL;

void output_dump_modules(void)
{
	int count;
	
	count = sizeof(modules) / sizeof(struct output_module *);
	if (count == 0) {
		puts("  NONE!");
	} else {
		int i;
		for (i=0; i<count; i++) {
			Log_info("output", "Available output: %s\t%s%s\n",
			        modules[i]->shortname,
			        modules[i]->description,
			        (i==0) ? " (default)" : "");
		}
	}
}

int output_init(const char *shortname)
{
	int count;

	wiced_rtos_init_semaphore(&output_semaphore);

	count = sizeof(modules) / sizeof(struct output_module *);
	if (count == 0) {
		Log_error("output", "No output module available");
		return -1;
	}
	if (shortname == NULL) {
		output_module = modules[0];
	} else {
		int i;
		for (i=0; i<count; i++) {
			if (strcmp(modules[i]->shortname, shortname)==0) {
				output_module = modules[i];
				break;
			}
		}
	}
	
	if (output_module == NULL) {
		Log_error("error", "ERROR: No such output module: '%s'",
			  shortname);
		return -1;
	}

	Log_info("output", "Using output module: %s (%s)",
		 output_module->shortname, output_module->description);

	if (output_module->init) {
		return output_module->init();
	}

	return 0;
}

int output_deinit(void)
{
    int rc = 0;

    wiced_rtos_deinit_semaphore(&output_semaphore);
    if ( (output_module != NULL) && (output_module->deinit != NULL) )
    {
        rc = output_module->deinit();
    }

    return rc;
}

int output_rendering_loop()
{
    wiced_rtos_get_semaphore(&output_semaphore, WICED_WAIT_FOREVER);
    return 0;
}

int output_add_options(void *ctx)
{
  	int count, i;

	count = sizeof(modules) / sizeof(struct output_module *);
	for (i = 0; i < count; ++i) {
		if (modules[i]->add_options) {
			int result = modules[i]->add_options(ctx);
			if (result != 0) {
				return result;
			}
		}
	}

	return 0;
}

void output_set_uri(const char *uri, output_update_meta_cb_t meta_cb) {
	if (output_module && output_module->set_uri) {
		output_module->set_uri(uri, meta_cb);
	}
}
void output_set_next_uri(const char *uri) {
	if (output_module && output_module->set_next_uri) {
		output_module->set_next_uri(uri);
	}
}

int output_play(output_transition_cb_t transition_callback) {
	if (output_module && output_module->play) {
		return output_module->play(transition_callback);
	}
	return -1;
}

int output_pause(void) {
	if (output_module && output_module->pause) {
		return output_module->pause();
	}
	return -1;
}

int output_stop(void) {
	if (output_module && output_module->stop) {
		return output_module->stop();
	}
	return -1;
}

int output_seek(int64_t position_nanos) {
	if (output_module && output_module->seek) {
		return output_module->seek(position_nanos);
	}
	return -1;
}

int output_get_position(int64_t *track_dur, int64_t *track_pos) {
	if (output_module && output_module->get_position) {
		return output_module->get_position(track_dur, track_pos);
	}
	return -1;
}

int output_get_volume(float *value) {
	if (output_module && output_module->get_volume) {
		return output_module->get_volume(value);
	}
	return -1;
}
int output_set_volume(float value, int level) {
	if (output_module && output_module->set_volume) {
		return output_module->set_volume(value, level);
	}
	return -1;
}
int output_get_mute(int *value) {
	if (output_module && output_module->get_mute) {
		return output_module->get_mute(value);
	}
	return -1;
}
int output_set_mute(int value) {
	if (output_module && output_module->set_mute) {
		return output_module->set_mute(value);
	}
	return -1;
}
void* output_get_audio_handle(void){
	if (output_module && output_module->get_audio_handle) {
		return output_module->get_audio_handle();
	}
	return NULL;
}
