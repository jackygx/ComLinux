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

#ifndef __COM_IO_HPP__
#define __COM_IO_HPP__

#include <IIo.hpp>

DEFINE_CLASS(ComIo);

class CComIo :
	public IIo,
	public CEnableSharedPtr<CComIo>
{
public:
	CComIo(void);
	virtual ~CComIo(void);

	void Init(const CConstStringPtr &port,
			  uint32_t baudrate,
			  bool flowctl);

	virtual IReaderPtr Read(const OnReadFn &onRead,
							const OnReadFailFn &onFail);

	virtual IWriterPtr Write(const CConstStringPtr &data,
							 const OnWriteFailFn &onFail,
							 const OnWrittenFn &onWritten);

private:
	void ReadLoop(void);

private:
	int mFd;
	OnReadFn mOnRead;
	OnReadFailFn mOnFail;
};

#endif /* __COM_IO_HPP__ */

