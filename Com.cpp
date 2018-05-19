/*
 * Copyright (c) 2018 Guo Xiang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <dirent.h>

#include "Com.hpp"
#include "ComIo.hpp"

void CCom::ListComPort(CStringList &list) const
{
	DIR *dir = opendir("/dev/");
	if (nullptr == dir) {
		throw E("Fail to open /dev/");
	}

	struct dirent *dirent;

	while (NULL != (dirent = readdir(dir))) {
		const char *name = dirent->d_name;

		if ('t' == name[0] &&
			't' == name[1] &&
			'y' == name[2]) {
			if (
				/* ttyS* */
				('S' == name[3]) ||
				/* ttyUSB* */
				(('U' == name[3] &&
				  'S' == name[4] &&
				  'B' == name[5]))) {
				list.PushBack(name);
			}
		}
	}
}

IIoPtr CCom::Connect(const CConstStringPtr &port,
					 int baudrate,
					 bool flowctl)
{
	CComIoPtr com;
	com->Init(port, baudrate, flowctl);

	return com;
}

