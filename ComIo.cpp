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

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <EasyCpp.hpp>

#include <Thread.hpp>
#include "ComIo.hpp"

CComIo::CComIo(void) :
	mFd(-1),
	mOnRead(nullptr),
	mOnFail(nullptr)
{
	/* Does nothing */
}

CComIo::~CComIo(void)
{
	if (mFd >= 0) {
		close(mFd);
		mFd = -1;
	}
}

#define DEFINE_BAUDRATE(speed) case speed: return B##speed

static inline speed_t BaudrateToSpeed(uint32_t baudrate)
{
	switch (baudrate) {
	DEFINE_BAUDRATE(19200);
	DEFINE_BAUDRATE(38400);
	DEFINE_BAUDRATE(57600);
	DEFINE_BAUDRATE(115200);
	DEFINE_BAUDRATE(230400);
	DEFINE_BAUDRATE(460800);
	DEFINE_BAUDRATE(576000);
	DEFINE_BAUDRATE(921600);
	default:
		throw E("Unknown baudrate: ", DEC(baudrate));
	}
}

void CComIo::Init(const CConstStringPtr &port,
				  uint32_t baudrate,
				  bool flowctl)
{
	CConstStringPtr path("/dev/", port);

	mFd = ::open(path->ToCStr(), O_RDWR | O_NOCTTY | O_SYNC);
	if (mFd < 0) {
		throw E("Fail to open: ", path);
	}

	struct termios tty;

	if (tcgetattr(mFd, &tty) < 0) {
		close(mFd);
		mFd = -1;
		throw E("Fail to get tty attribute");
	}

	cfsetospeed(&tty, BaudrateToSpeed(baudrate));
	cfsetispeed(&tty, BaudrateToSpeed(baudrate));

	tty.c_cflag |= (CLOCAL | CREAD);	/* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;					/* 8-bit characters */
	tty.c_cflag &= ~PARENB;				/* no parity bit */
	tty.c_cflag &= ~CSTOPB;				/* only need 1 stop bit */
	if (flowctl) {
		tty.c_cflag |= CRTSCTS;
	} else {
		tty.c_cflag &= ~CRTSCTS;
	}

	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(mFd, TCSANOW, &tty) != 0) {
		close(mFd);
		mFd = -1;
		throw E("Fail to set tty attribute");
	}
}

IReaderPtr CComIo::Read(const OnReadFn &onRead,
						const OnReadFailFn &onFail)
{
	CHECK_PARAM(onRead, "onRead is null");

	mOnRead = onRead;
	mOnFail = onFail;

	Platform::CreateThread([&](void) {
		ReadLoop();
	});

	return Share();
}

IWriterPtr CComIo::Write(const CConstStringPtr &data,
						 const OnWriteFailFn &onFail,
						 const OnWrittenFn &onWritten)
{
	CHECK_PARAM(data, "data is null");

	int ret = write(mFd, data->ToCStr(), data->GetSize());
	if (ret <= 0) {
		TRACE_ERROR("Fail to write com port, error: ", DEC(errno), EOS);
		if (onFail) {
			onFail();
		}
	} else {
		if (onWritten) {
			onWritten();
		}
	};

	return Share();
}

void CComIo::ReadLoop(void)
{
	while (true) {
		CStringPtr buf;

		int ret = read(mFd, buf, buf->GetCapacity() - 1);
		if (ret <= 0) {
			TRACE_ERROR("Fail to read com port, error: ", DEC(errno), EOS);
			if (mOnFail) {
				mOnFail();
			}
			break;
		}

		buf->SetSize(ret);
		mOnRead(buf);
	}
}

