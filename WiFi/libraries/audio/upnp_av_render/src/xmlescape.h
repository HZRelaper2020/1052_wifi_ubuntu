/* xmlescape.h - helper routines for escaping XML strings
 *
 * Copyright (C) 2007   Ivo Clarysse
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

#ifndef _XMLESCAPE_H
#define _XMLESCAPE_H

// XML escape string "str". If "attribute" is 1, then this is considered
// to be within an xml attribute (i.e. quotes are escaped as well).
// Returns a malloc()ed string; caller needs to free().
char *xmlescape(const char *str, int attribute);

#endif /* _XMLESCAPE_H */
