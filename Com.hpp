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

#ifndef __COM_HPP__
#define __COM_HPP__

#include <IIo.hpp>

DEFINE_CLASS(Com);

class CCom :
	public ICom
{
public:
	virtual void ListComPort(CStringList &list) const;
	virtual IIoPtr Connect(const CConstStringPtr &port,
						   int baudrate,
						   bool flowctl);
};

#endif /* __COM_HPP__ */

