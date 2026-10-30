// SPDX-License-Identifier: GPL-2.0-only
/*
 * Synaptics TouchComm (TCM) touchscreen driver, protocol core.
 *
 * This part is bus agnostic: synaptics_tcm_spi.c provides the SPI read/write
 * pair plus the power and reset wiring.
 *
 * Two protocol generations are implemented.  The part on this board is an
 * S3910 and speaks TouchComm v1; the v2 path is the one recovered from the
 * vendor module synaptics_tcm2_ts.ko of the Meizu 22 (SM8735), which is an
 * ET_REL object, so every "vendor .text+0x..." citation below is its symbol
 * value, i.e. file offset = sh_offset(.text) + st_value.  v1 follows the
 * reference implementation in vendor-ref/oneplus-dt/.../s3910/tcm/
 * synaptics_touchcom_core_v1.c, whose values are cited at the constants they
 * define; where the two agree with the bytes measured on this board, the
 * measured exchange is quoted as well.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/slab.h>

#include "synaptics_tcm.h"

/*
 * How long a command may take to answer, from CMD_RESPONSE_TIMEOUT_MS in the
 * reference headers (core_dev.h:119); both generations use it there.
 */
#define TCM_RESPONSE_TIMEOUT_MS	3000
#define TCM_IDENTIFY_TIMEOUT_MS	1000
#define TCM_PARSE_LOOPS		512	/* guard against a looping config */

/*
 * Retry bounds for a corrupted/busy (0xfe) answer, taken from
 * syna_tcm_v2_execute_cmd_request(): the command re-send counter is bounded by
 * "cmp w22, #6" after the increment at +0x32bc/+0x32d0 (five re-sends), the
 * payload-chunk counter by "cmp w28, #6" before the increment at
 * +0x30d8/+0x30e0 (six retries).
 */
#define TCM_COMMAND_RETRIES	5
#define TCM_CHUNK_RETRIES	6

/*
 * v1 timings, all from the reference headers and core:
 *
 *  - CMD_RESPONSE_POLLING_DELAY_MS (core_dev.h:121) is how often the vendor
 *    asks a v1 part for its answer after a command: syna_tcm_v1_write_message()
 *    sleeps that long and re-reads until the part stops answering STATUS_IDLE
 *    or CMD_RESPONSE_TIMEOUT_MS passes.  A v1 part really does answer "not
 *    ready yet" first: this board's part answered STATUS_IDLE to the
 *    application info command and only produced it on later polls.
 *  - RD_RETRY_US_MIN/MAX (core_dev.h:123-124) is the pause before re-reading a
 *    frame whose first byte is not the 0xa5 marker, in syna_tcm_v1_read().
 */
#define TCM_V1_POLL_DELAY_US		20000
#define TCM_V1_READ_RETRIES		10
#define TCM_V1_RD_RETRY_US_MIN		5000
#define TCM_V1_RD_RETRY_US_MAX		10000

/*
 * How long to keep asking a part that has just been powered up whether it is
 * there.  See the call site for why this is measured in seconds.
 */
#define TCM_IDENTIFY_ATTEMPTS	6
#define TCM_IDENTIFY_RETRY_MS	500

/*
 * Upper bound for the whole detect and identify phase.  A part that answers the
 * handshake but not the commands would otherwise spend one
 * TCM_RESPONSE_TIMEOUT_MS per attempt (up to 18 s) before the probe gives up,
 * and that delay lands on every boot.  An absent part is unaffected: its
 * detection fails in microseconds, so it still gets all the retries.
 */
#define TCM_IDENTIFY_PHASE_MS	5000

/* How many identify answers an unrelated queued report may precede. */
#define TCM_IDENTIFY_READS	3

static void tcm_handle_report(struct synaptics_tcm *tcm,
			      const struct tcm_packet *pkt);

/*
 * CRC-16 of a TouchComm packet.  The vendor module carries the 256-entry
 * table (syna_tcm_crc16.crc16_table at .rodata+0x256, polynomial 0x1021,
 * initial value 0xffff); the bitwise form below produces the identical table.
 */
static u16 tcm_crc16(const u8 *data, unsigned int length)
{
	u16 crc = 0xffff;
	unsigned int i;

	while (length--) {
		crc ^= (u16)*data++ << 8;
		for (i = 0; i < 8; i++)
			crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
	}

	return crc;
}

/*
 * CRC-6 over the first 26 bits of the header.  Table and shift sequence are
 * the vendor's: syna_tcm_crc6.crc6_table lives at .rodata.cst32 and the same
 * algorithm is inlined in syna_tcm_v2_write() and syna_tcm_v2_read().
 */
static u8 tcm_crc6(const u8 *data, unsigned int bits)
{
	static const u16 crc6_table[16] = {
		0, 268, 536, 788, 1072, 1340, 1576, 1828,
		2144, 2412, 2680, 2932, 3152, 3420, 3656, 3908,
	};
	u16 r = 0x003f << 2;
	u16 x;

	for (; bits > 8; bits -= 8) {
		r ^= *data++;
		r = (r << 4) ^ crc6_table[(r >> 4) & 0xf];
		r = (r << 4) ^ crc6_table[(r >> 4) & 0xf];
	}

	if (bits) {
		x = *data;
		while (bits--) {
			if (x & 0x80)
				r ^= 0x80;
			x <<= 1;
			r <<= 1;
			if (r & 0x100)
				r ^= 0x03 << 2;
		}
	}

	return (r >> 2) & 0x3f;
}

static int tcm_bus_write(struct synaptics_tcm *tcm, const void *buf,
			 size_t count)
{
	return tcm->ops->write(tcm, buf, count);
}

static int tcm_bus_read(struct synaptics_tcm *tcm, void *buf, size_t count)
{
	return tcm->ops->read(tcm, buf, count);
}

/*
 * Send one TouchComm v2 packet: 4-byte header, optional payload and, when a
 * payload is present, a big-endian CRC-16 over header+payload.  Layout and
 * sequence toggle follow the vendor syna_tcm_v2_write() (.text+0x3fdc).
 */
static int tcm_v2_write_message(struct synaptics_tcm *tcm, u8 command,
				const u8 *payload, unsigned int length,
				bool resend)
{
	unsigned int size = TCM_HEADER_SIZE;
	u16 crc;

	if (length > TCM_MAX_PAYLOAD)
		return -EINVAL;

	if (length)
		size += length + TCM_CRC16_SIZE;
	if (tcm->max_write_size && size > tcm->max_write_size)
		return -EINVAL;

	/*
	 * The whole packet is built in tcm->tx_buf and handed to the bus in
	 * exactly one transfer, like syna_tcm_v2_write() (.text+0x3fdc): its
	 * csel at +0x402c picks 4 or payload_len + 6 bytes and there is a
	 * single bus write at +0x4254.  Sending the header on its own first
	 * would make the part eat the next header as this packet's payload.
	 *
	 * The sequence bit is bumped and then used, unless the packet is a
	 * re-send (vendor: "if (resend) seq_toggle -= 1" at +0x4064, the
	 * increment at +0x4068, the field at +0x40a4).
	 */
	if (!resend)
		tcm->seq++;

	tcm->tx_buf[0] = command;
	tcm->tx_buf[1] = length & 0xff;
	tcm->tx_buf[2] = length >> 8;
	tcm->tx_buf[3] = (tcm->seq & 0x01) << 6;
	tcm->tx_buf[3] |= tcm_crc6(tcm->tx_buf, 26);

	if (length) {
		memcpy(tcm->tx_buf + TCM_HEADER_SIZE, payload, length);
		crc = tcm_crc16(tcm->tx_buf, TCM_HEADER_SIZE + length);
		tcm->tx_buf[TCM_HEADER_SIZE + length] = crc >> 8;
		tcm->tx_buf[TCM_HEADER_SIZE + length + 1] = crc & 0xff;
	}

	return tcm_bus_write(tcm, tcm->tx_buf, size);
}

/*
 * Read one TouchComm v2 payload chunk into tcm->chunk_buf and verify both
 * CRCs.  Chunks after the first one are released by writing CMD_TCM2_ACK
 * first; see the caller for why the first chunk is read without it.
 */
static int tcm_v2_read_chunk(struct synaptics_tcm *tcm, unsigned int offset,
			     unsigned int payload_len, bool ack, bool resend)
{
	unsigned int transfer = TCM_HEADER_SIZE + payload_len + TCM_CRC16_SIZE;
	int error;

	/* A transfer is capped by the identified max read size, header and
	 * CRC-16 included (syna_tcm_v2_read, .text+0x43d4 at +0x43f0).
	 */
	if (tcm->max_read_size && transfer > tcm->max_read_size)
		return -EMSGSIZE;

	if (ack) {
		error = tcm_v2_write_message(tcm, TCM_CMD_TCM2_ACK, NULL, 0,
					     resend);
		if (error)
			return error;
	}

	error = tcm_bus_read(tcm, tcm->chunk_buf, transfer);
	if (error)
		return error;

	if (tcm_crc6(tcm->chunk_buf, 32))
		return -EBADMSG;

	if (tcm_crc16(tcm->chunk_buf, transfer))
		return -EBADMSG;

	memcpy(tcm->rx_buf + TCM_HEADER_SIZE + offset,
	       tcm->chunk_buf + TCM_HEADER_SIZE, payload_len);

	return 0;
}

/*
 * Read one complete TouchComm v2 packet (header plus all payload chunks) into
 * tcm->rx_buf and point @pkt at it.  The payload stays valid until the next
 * read.
 */
static int tcm_v2_read_message(struct synaptics_tcm *tcm, struct tcm_packet *pkt)
{
	unsigned int offset = 0;
	unsigned int remaining;
	unsigned int length;
	unsigned int chunk_space;
	int error;

	error = tcm_bus_read(tcm, tcm->rx_buf, TCM_HEADER_SIZE);
	if (error)
		return error;

	if (tcm_crc6(tcm->rx_buf, 32))
		return -EBADMSG;

	length = tcm->rx_buf[1] | (tcm->rx_buf[2] << 8);
	if (length > TCM_MAX_PAYLOAD)
		return -EMSGSIZE;

	pkt->code = tcm->rx_buf[0];
	pkt->length = length;
	pkt->payload = length ? tcm->rx_buf + TCM_HEADER_SIZE : NULL;

	if (!length)
		return 0;

	/* Before the identify packet: read the payload in one piece. */
	if (tcm->max_read_size > TCM_HEADER_SIZE + TCM_CRC16_SIZE)
		chunk_space = tcm->max_read_size - TCM_HEADER_SIZE -
			      TCM_CRC16_SIZE;
	else
		chunk_space = TCM_MAX_PAYLOAD;

	remaining = length;
	while (remaining) {
		unsigned int chunk = min(remaining, chunk_space);
		unsigned int retries = 0;

		for (;;) {
			/*
			 * The part hands out the first chunk right after its
			 * header; only the chunks after it must be released
			 * with CMD_TCM2_ACK (syna_tcm_v2_execute_cmd_request
			 * writes the ACK at +0x3094 only when the ioctl
			 * override, a retry or the chunk counter is set;
			 * +0x307c..+0x3088).
			 *
			 * A chunk that fails its CRC or answers 0xfe is
			 * retried by writing a bare ACK -- sequence bit
			 * unchanged -- and re-reading the same chunk, up to
			 * six times (+0x30cc..+0x3108, bound at +0x30d8).
			 */
			error = tcm_v2_read_chunk(tcm, offset, chunk,
						  offset != 0 || retries != 0,
						  retries != 0);
			if (!error &&
			    tcm->chunk_buf[0] != TCM_STATUS_PACKET_CORRUPTED)
				break;

			if (error && error != -EBADMSG)
				return error;

			if (retries >= TCM_CHUNK_RETRIES) {
				dev_err(tcm->dev,
					"payload chunk %u unusable after %u retries (last status 0x%02x, error %d)\n",
					offset, retries,
					error ? TCM_STATUS_PACKET_CORRUPTED :
						tcm->chunk_buf[0], error);
				return error ? error : -EIO;
			}

			retries++;
		}

		offset += chunk;
		remaining -= chunk;
	}

	return 0;
}

/*
 * Send one TouchComm v1 command frame: 3-byte header { command, length_le16 }
 * plus the payload, and no CRC.  Layout from syna_tcm_v1_write() in the
 * reference core, and confirmed on this board: the 3 bytes "02 00 00" make the
 * part answer with the identify message.
 */
static int tcm_v1_write_message(struct synaptics_tcm *tcm, u8 command,
				const u8 *payload, unsigned int length)
{
	if (length > TCM_MAX_TRANSFER - TCM_V1_HOST_HEADER_SIZE)
		return -EINVAL;

	tcm->tx_buf[0] = command;
	tcm->tx_buf[1] = length & 0xff;
	tcm->tx_buf[2] = length >> 8;
	if (length)
		memcpy(tcm->tx_buf + TCM_V1_HOST_HEADER_SIZE, payload, length);

	return tcm_bus_write(tcm, tcm->tx_buf,
			     TCM_V1_HOST_HEADER_SIZE + length);
}

/*
 * Clock @count bytes out of the part, retrying while the frame does not start
 * with the v1 marker.  The reference re-reads up to ten times, sleeping
 * RD_RETRY_US_MIN..MAX (5-10 ms) in between (syna_tcm_v1_read), because a part
 * that has nothing to hand out pads with 0x5a or 0x00 instead of answering.
 *
 * A marker further into the buffer means the part was still holding a trailing
 * byte of the message before this one -- its end-of-message 0x5a, or padding.
 * The frame is then picked up where it starts and the missing bytes are clocked
 * behind it, instead of throwing the whole buffer away and losing a message
 * that has already begun.
 */
static int tcm_v1_read_checked(struct synaptics_tcm *tcm, u8 *buf,
			       unsigned int count)
{
	unsigned int retry;
	unsigned int skip;
	int error;

	for (retry = 0; retry < TCM_V1_READ_RETRIES; retry++) {
		error = tcm_bus_read(tcm, buf, count);
		if (error)
			return error;

		if (buf[0] == TCM_V1_MESSAGE_MARKER)
			return 0;

		for (skip = 1; skip < count; skip++) {
			if (buf[skip] != TCM_V1_MESSAGE_MARKER)
				continue;

			memmove(buf, buf + skip, count - skip);
			error = tcm_bus_read(tcm, buf + count - skip, skip);
			if (error)
				return error;
			if (buf[0] == TCM_V1_MESSAGE_MARKER)
				return 0;

			break;
		}

		dev_dbg(tcm->dev, "v1 frame starts with 0x%02x, retry %u\n",
			buf[0], retry + 1);
		usleep_range(TCM_V1_RD_RETRY_US_MIN, TCM_V1_RD_RETRY_US_MAX);
	}

	return -EBADMSG;
}

/*
 * Clock out the payload of a v1 message whose header has already been read and
 * copy it behind that header in tcm->rx_buf.  A payload arrives as one or more
 * continuation chunks, each led by { 0xa5, STATUS_CONTINUED_READ }; the
 * reference walks them the same way (syna_tcm_v1_read_message() hands the
 * remaining length to syna_tcm_v1_continued_read(), which loops until the
 * payload is complete), and the vendor module has a continued-read function of
 * its own for the same reason.  A chunk whose second byte is not 0x03 is a
 * frame that has gone wrong.
 *
 * A v1 frame is 4 + length + 1 bytes: one 0x5a end-of-message byte follows the
 * payload, and the vendor's continued read counts it in ("remember a padding
 * at the end": remaining_length = length + 1, read as xfer_length + 2), which
 * is why only the last chunk asks for it.  The EOM is consumed but not
 * validated -- only its position matters, and a part left in the middle of a
 * frame answers the next read one byte early.
 */
static int tcm_v1_read_payload(struct synaptics_tcm *tcm, unsigned int length)
{
	unsigned int offset = 0;
	unsigned int remaining = length;
	int error;

	if (length > TCM_MAX_TRANSFER - TCM_V1_HEADER_SIZE)
		return -EMSGSIZE;

	while (remaining) {
		unsigned int chunk = min(remaining, TCM_V1_CHUNK_PAYLOAD);
		unsigned int transfer = chunk + TCM_V1_CHUNK_HEADER_SIZE;

		/* The end-of-message byte closes the frame after the last
		 * payload byte.
		 */
		if (chunk == remaining)
			transfer++;

		error = tcm_v1_read_checked(tcm, tcm->chunk_buf, transfer);
		if (error)
			return error;

		if (tcm->chunk_buf[1] != TCM_STATUS_CONTINUED_READ)
			return -EBADMSG;

		memcpy(tcm->rx_buf + TCM_V1_HEADER_SIZE + offset,
		       tcm->chunk_buf + TCM_V1_CHUNK_HEADER_SIZE, chunk);

		offset += chunk;
		remaining -= chunk;
	}

	return 0;
}

/*
 * Read one TouchComm v1 message into tcm->rx_buf and point @pkt at it: the
 * 4-byte header { 0xa5, code, length_le16 } first, then the payload.  The
 * payload stays valid until the next read.
 */
static int tcm_v1_read_message(struct synaptics_tcm *tcm, struct tcm_packet *pkt)
{
	unsigned int length;
	int error;

	error = tcm_v1_read_checked(tcm, tcm->rx_buf, TCM_V1_HEADER_SIZE);
	if (error)
		return error;

	/*
	 * A continuation where the header belongs means the part is still in
	 * the middle of a message that an earlier, short read left unfinished;
	 * the beginning of it is gone, and the reference calls this an
	 * out-of-sync continued read (_EIO in syna_tcm_v1_read_message()).
	 * Reporting it as a bad frame makes the command path re-send its
	 * command, which starts the part on a fresh message.
	 */
	if (tcm->rx_buf[1] == TCM_STATUS_CONTINUED_READ)
		return -EBADMSG;

	length = tcm->rx_buf[2] | (tcm->rx_buf[3] << 8);

	pkt->code = tcm->rx_buf[1];
	pkt->length = length;
	pkt->payload = length ? tcm->rx_buf + TCM_V1_HEADER_SIZE : NULL;

	if (length)
		return tcm_v1_read_payload(tcm, length);

	/*
	 * A frame is 4 + length + 1 bytes even with no payload: the byte after a
	 * zero length header is the 0x5a end-of-message terminator (measured:
	 * the not-implemented answer to a v2 command reads "a5 0e 00 00 5a 5a
	 * 5a 5a"), and every answer of the command poll is such a frame.  It is
	 * clocked out here without being validated -- it is a terminator, and
	 * leaving it behind makes the next read start on it and lose the
	 * message that comes after.
	 */
	return tcm_bus_read(tcm, tcm->chunk_buf, 1);
}

/* Send a command using the framing of the detected protocol generation. */
static int tcm_write_message(struct synaptics_tcm *tcm, u8 command,
			     const u8 *payload, unsigned int length,
			     bool resend)
{
	if (tcm->protocol == TCM_PROTOCOL_V1)
		return tcm_v1_write_message(tcm, command, payload, length);

	return tcm_v2_write_message(tcm, command, payload, length, resend);
}

/* Read one message using the framing of the detected protocol generation. */
static int tcm_read_message(struct synaptics_tcm *tcm, struct tcm_packet *pkt)
{
	if (tcm->protocol == TCM_PROTOCOL_V1)
		return tcm_v1_read_message(tcm, pkt);

	return tcm_v2_read_message(tcm, pkt);
}

/*
 * Write a command packet and return its first answer in @pkt.
 *
 * A corrupted read, or the part's 0xfe "busy, send it again" status, makes the
 * vendor re-send the very same packet -- sequence bit unchanged -- after
 * 500-1000 us, and give up once its counter reaches six, i.e. after five
 * re-sends (syna_tcm_v2_execute_cmd_request: 0xfe detected at +0x2f00, retry
 * counter incremented at +0x32bc, bound "cmp w22, #6" at +0x32d0, sleep at
 * +0x32f4..+0x3300, re-send with the resend flag set at +0x3314..+0x3328; the
 * give-up log is at +0x36a0).
 *
 * v1 has no sequence bit and no CRC, so a wrong marker takes the place of the
 * CRC failure and re-sending the command is what starts the part on a fresh
 * message; the delays are the v1 ones.
 */
static int tcm_write_command(struct synaptics_tcm *tcm, u8 command,
			     const u8 *payload, unsigned int length,
			     struct tcm_packet *pkt)
{
	unsigned int retries = 0;
	int error;

	for (;;) {
		error = tcm_write_message(tcm, command, payload, length,
					  retries != 0);
		if (error)
			return error;

		/* Vendor cadence: 100-200 us after the write (+0x2ec0), then
		 * 500-1000 us before a re-send (+0x32f4).
		 */
		usleep_range(100, 200);

		error = tcm_read_message(tcm, pkt);
		if (!error && pkt->code != TCM_STATUS_PACKET_CORRUPTED)
			return 0;

		/* Only a corrupted frame (CRC-6/CRC-16, i.e. what the vendor
		 * turns into its 0xfe status) or an explicit 0xfe is worth
		 * another try; a bus error is terminal (vendor +0x2efc is a
		 * negative read and ends the command at +0x36d8).
		 */
		if (error && error != -EBADMSG)
			return error;

		if (retries >= TCM_COMMAND_RETRIES) {
			dev_err(tcm->dev,
				"command 0x%02x unusable after %u retries (error %d, last code 0x%02x)\n",
				command, retries, error,
				error ? 0 : pkt->code);
			return error ? error : -EIO;
		}

		dev_dbg(tcm->dev, "command 0x%02x answer corrupt, retry %u\n",
			command, retries + 1);
		usleep_range(TCM_V1_RD_RETRY_US_MIN, TCM_V1_RD_RETRY_US_MAX);
		retries++;
	}
}

/*
 * Write a command and wait for its response.  A pending touch report can
 * arrive before the response, so such packets are handed to the report
 * handler and the read is retried.  With @accept_report set, a report packet
 * is returned to the caller instead (CMD_IDENTIFY answers with one, and a v1
 * part answers it with either STATUS_OK or a REPORT_IDENTIFY).
 *
 * A v1 part answers STATUS_IDLE while the command is still being worked on,
 * so that is a "not ready yet": the vendor keeps polling every
 * CMD_RESPONSE_POLLING_DELAY_MS (20 ms) until CMD_RESPONSE_TIMEOUT_MS passes
 * (syna_tcm_v1_write_message), and this does the same.  This board's part did
 * exactly that for its application info command.
 *
 * Returns 0 with @response filled in, -ENODATA when the part answered
 * STATUS_NO_REPORT_AVAILABLE (or STATUS_IDLE on v2, where it is not a
 * "working on it" answer), -ETIMEDOUT when nothing arrived.
 */
static int tcm_send_command(struct synaptics_tcm *tcm, u8 command,
			    const u8 *payload, unsigned int length,
			    struct tcm_packet *response, bool accept_report)
{
	unsigned long timeout;
	int error;

	error = tcm_write_command(tcm, command, payload, length, response);
	if (error)
		return error;

	timeout = jiffies + msecs_to_jiffies(TCM_RESPONSE_TIMEOUT_MS);
	while (time_before(jiffies, timeout)) {
		switch (response->code) {
		case TCM_STATUS_OK:
			return 0;
		case TCM_STATUS_IDLE:
			/* v1: the command is not finished yet; poll again. */
			if (tcm->protocol != TCM_PROTOCOL_V1)
				return -ENODATA;
			break;
		case TCM_STATUS_ACK:
			/* Write accepted, the real response is still to come. */
			break;
		case TCM_STATUS_NO_REPORT_AVAILABLE:
			return -ENODATA;
		default:
			if (response->code >= TCM_REPORT_IDENTIFY) {
				if (accept_report)
					return 0;
				tcm_handle_report(tcm, response);
				break;	/* a report raced the response */
			}
			dev_dbg(tcm->dev, "command 0x%02x failed: 0x%02x\n",
				command, response->code);
			return -EIO;
		}

		if (tcm->protocol == TCM_PROTOCOL_V1)
			usleep_range(TCM_V1_POLL_DELAY_US,
				     TCM_V1_POLL_DELAY_US + 1000);
		else
			usleep_range(2000, 3000);

		error = tcm_read_message(tcm, response);
		/* A read error is terminal, like the vendor's (a negative read
		 * return ends the command, only 0xfe is retried); continuing
		 * would turn it into a timeout with msg_lock held.
		 */
		if (error)
			return error;
	}

	dev_dbg(tcm->dev, "command 0x%02x timed out\n", command);

	return -ETIMEDOUT;
}

static int tcm_enable_touch_report(struct synaptics_tcm *tcm, bool enable)
{
	struct tcm_packet response;
	u8 report = TCM_REPORT_TOUCH;
	u8 command = enable ? TCM_CMD_ENABLE_REPORT : TCM_CMD_DISABLE_REPORT;
	int error;

	/* Payload is a single report code, from syna_tcm_enable_report()
	 * (func_base.c: one byte, the code passed by the caller), and the v1
	 * part answers exactly this frame with STATUS_OK -- measured on this
	 * board: "05 01 00 11" -> "a5 01 00 00".
	 */
	error = tcm_send_command(tcm, command, &report, 1, &response, false);
	if (error)
		dev_dbg(tcm->dev, "cannot %s the touch report: %d\n",
			enable ? "enable" : "disable", error);

	return error;
}

static int tcm_read_app_info(struct synaptics_tcm *tcm)
{
	struct tcm_packet response;
	int error;

	error = tcm_send_command(tcm, TCM_CMD_GET_APPLICATION_INFO, NULL, 0,
				 &response, false);
	if (error)
		return error;

	/* The S3910 answers with 46 bytes, two short of the v2 structure the
	 * driver keeps them in (measured: "20 00 00" -> "a5 01 2e 00 ..."), so
	 * every field past max_objects may be missing and is zeroed instead of
	 * rejected.
	 */
	if (response.length < TCM_APP_INFO_MIN_LEN) {
		dev_err(tcm->dev, "short application info: %u bytes\n",
			response.length);
		return -EINVAL;
	}

	memset(&tcm->app_info, 0, sizeof(tcm->app_info));
	memcpy(&tcm->app_info, response.payload,
	       min_t(unsigned int, response.length, sizeof(tcm->app_info)));

	/* Field offsets are those of syna_tcm_get_app_info() (.text+0x628c):
	 * status at +2, max_x at +32, max_y at +34, max_objects at +36.  The
	 * measured reply has version = 1, status = 0 (APP_STATUS_OK) and
	 * plausible config sizes at +4..+15, which is where they are expected.
	 */
	tcm->dev_max_x = le16_to_cpu(tcm->app_info.max_x);
	tcm->dev_max_y = le16_to_cpu(tcm->app_info.max_y);
	tcm->max_objects = le16_to_cpu(tcm->app_info.max_objects);
	if (!tcm->max_objects || tcm->max_objects > TCM_MAX_OBJECTS)
		tcm->max_objects = TCM_MAX_OBJECTS;

	dev_info(tcm->dev,
		 "application info: %ux%u, %u objects, app status 0x%04x\n",
		 tcm->dev_max_x, tcm->dev_max_y, tcm->max_objects,
		 le16_to_cpu(tcm->app_info.status));

	return 0;
}

static int tcm_read_report_config(struct synaptics_tcm *tcm)
{
	struct tcm_packet response;
	u8 *config;
	int error;

	error = tcm_send_command(tcm, TCM_CMD_GET_TOUCH_REPORT_CONFIG, NULL, 0,
				 &response, false);
	if (error)
		return error;

	if (!response.length) {
		dev_err(tcm->dev, "empty touch report configuration\n");
		return -EINVAL;
	}

	/*
	 * Sized from the response and kept for the lifetime of the device, as
	 * syna_tcm_preserve_touch_report_config() does (.text+0xee7c: the
	 * length comes from the packet, the buffer is devm_kmalloc'ed).  A
	 * fixed-size buffer would silently cut the configuration in half and
	 * every report would then fail to parse.
	 */
	config = devm_kmalloc(tcm->dev, response.length, GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	memcpy(config, response.payload, response.length);
	if (tcm->report_config)
		devm_kfree(tcm->dev, tcm->report_config);

	tcm->report_config = config;
	tcm->report_config_len = response.length;

	dev_info(tcm->dev, "touch report configuration: %u bytes\n",
		 response.length);

	return 0;
}

/*
 * Read @bits bits at @offset from the report.  Fields are little-endian
 * across the bit stream: syna_tcm_get_touch_data() shifts the byte right by
 * the bit offset and packs the extracted bits into the low end of the value,
 * so a field must not be bit reversed.
 *
 * A field that runs past the end of the report reads as zero, which is what
 * the vendor does: a report can be shorter than the configuration expects
 * (fewer bytes than the declared objects need), and dropping the whole frame
 * over that would lose touches that the part did report.
 */
static int tcm_report_get_bits(const u8 *report, unsigned int report_length,
			       unsigned int offset, unsigned int bits,
			       unsigned int *value)
{
	unsigned int i;
	unsigned int data = 0;

	if (!bits || bits > 32)
		return -EINVAL;

	if (offset + bits > report_length * 8) {
		*value = 0;
		return 0;
	}

	for (i = 0; i < bits; i++) {
		unsigned int bit = offset + i;

		data |= ((report[bit / 8] >> (bit % 8)) & 1) << i;
	}

	*value = data;

	return 0;
}

/*
 * Walk the touch report configuration and pull the fields we care about out
 * of the report.  The configuration is a byte stream of entities; object
 * entities are enclosed in a FOREACH_ACTIVE_OBJECT / FOREACH_END block that
 * is replayed once per object, exactly like syna_tcm_parse_touch_report().
 */
static int tcm_parse_touch_report(struct synaptics_tcm *tcm,
				  const u8 *report,
				  unsigned int report_length,
				  struct tcm_touch_data *data)
{
	const u8 *config = tcm->report_config;
	unsigned int config_length = tcm->report_config_len;
	unsigned int idx = 0, offset = 0;
	unsigned int obj = 0, next = 0, objects = 0;
	unsigned int active_objects = 0;
	unsigned int loops = 0;
	bool active_only = false;
	bool have_active_objects = false;
	unsigned int value;
	unsigned int bits;
	u8 entity;
	int error;

	if (!config_length || !report || !report_length)
		return -EINVAL;

	memset(data, 0, sizeof(*data));

	while (idx < config_length) {
		if (++loops > TCM_PARSE_LOOPS)
			return -EINVAL;

		entity = config[idx++];

		switch (entity) {
		case TCM_TR_END:
			return 0;
		case TCM_TR_FOREACH_ACTIVE_OBJECT:
			obj = 0;
			next = idx;
			active_only = true;
			break;
		case TCM_TR_FOREACH_OBJECT:
			obj = 0;
			next = idx;
			active_only = false;
			break;
		case TCM_TR_FOREACH_END:
			if (active_only && have_active_objects) {
				objects++;
				obj++;
				if (objects < active_objects &&
				    obj < tcm->max_objects)
					idx = next;
			} else if (active_only) {
				if (offset < report_length * 8 &&
				    obj + 1 < tcm->max_objects) {
					obj++;
					idx = next;
				}
			} else if (obj + 1 < tcm->max_objects) {
				obj++;
				idx = next;
			}
			break;
		case TCM_TR_PAD_TO_NEXT_BYTE:
			offset = round_up(offset, 8);
			break;
		case TCM_TR_NUM_OF_ACTIVE_OBJECTS:
			if (idx >= config_length)
				return -EINVAL;
			bits = config[idx++];
			error = tcm_report_get_bits(report, report_length,
						    offset, bits, &value);
			if (error)
				return error;
			active_objects = value;
			have_active_objects = true;
			offset += bits;
			if (!active_objects)
				return 0;
			break;
		case TCM_TR_OBJECT_INDEX:
			if (idx >= config_length)
				return -EINVAL;
			bits = config[idx++];
			error = tcm_report_get_bits(report, report_length,
						    offset, bits, &value);
			if (error)
				return error;
			obj = value;
			offset += bits;
			break;
		case TCM_TR_BUTTONS_STATE:
			if (idx >= config_length)
				return -EINVAL;
			bits = config[idx++];
			error = tcm_report_get_bits(report, report_length,
						    offset, bits, &value);
			if (error)
				return error;
			data->buttons = value;
			offset += bits;
			break;
		default:
			/*
			 * The object entities all carry the field width in
			 * bits in the byte that follows; every other entity
			 * does the same, so an unknown one can be skipped.
			 */
			if (idx >= config_length)
				return -EINVAL;
			bits = config[idx++];

			/* Only the entities this driver actually stores are
			 * read out of the report.  The reference's default
			 * branch is literally
			 *     bits = config_data[idx++];
			 *     offset += bits;
			 * i.e. an unrecognised entity only ADVANCES the
			 * report offset and never extracts a value.
			 *
			 * Getting this wrong is what stopped this board's
			 * touch from working: the 128-byte configuration
			 * begins with the vendor's custom 0xca block, whose
			 * declared width is 0x70 = 112 bits, and extracting
			 * it from a 29-byte touch report read past the end
			 * and returned -EINVAL.  The report was then never
			 * consumed, the controller kept ATTN asserted, and
			 * the threaded IRQ handler spun on a message it
			 * could not parse.
			 */
			if (obj >= TCM_MAX_OBJECTS) {
				offset += bits;
				break;
			}

			switch (entity) {
			case TCM_TR_OBJECT_CLASSIFICATION:
			case TCM_TR_OBJECT_X:
			case TCM_TR_OBJECT_Y:
			case TCM_TR_OBJECT_Z:
			case TCM_TR_OBJECT_X_WIDTH:
			case TCM_TR_OBJECT_Y_WIDTH:
				error = tcm_report_get_bits(report,
							    report_length,
							    offset, bits, &value);
				if (error)
					return error;
				break;
			default:
				value = 0;
				break;
			}
			offset += bits;

			switch (entity) {
			case TCM_TR_OBJECT_CLASSIFICATION:
				data->object[obj].status = value;
				break;
			case TCM_TR_OBJECT_X:
				data->object[obj].x = value;
				break;
			case TCM_TR_OBJECT_Y:
				data->object[obj].y = value;
				break;
			case TCM_TR_OBJECT_Z:
				data->object[obj].z = value;
				break;
			case TCM_TR_OBJECT_X_WIDTH:
				data->object[obj].x_width = value;
				break;
			case TCM_TR_OBJECT_Y_WIDTH:
				data->object[obj].y_width = value;
				break;
			default:
				break;
			}
			break;
		}
	}

	return 0;
}

static unsigned int tcm_scale(unsigned int value, unsigned int source_max,
			      unsigned int target_max)
{
	u64 scaled;

	if (!source_max || source_max == target_max)
		return min(value, target_max);

	scaled = div_u64((u64)value * target_max, source_max);
	if (scaled > target_max)
		scaled = target_max;

	return scaled;
}

static void tcm_report_input(struct synaptics_tcm *tcm,
			     const struct tcm_touch_data *data)
{
	struct input_dev *input = tcm->input;
	unsigned int i;

	for (i = 0; i < tcm->max_objects; i++) {
		const struct tcm_object *object = &data->object[i];
		unsigned int x, y, major, minor;

		input_mt_slot(input, i);

		if (object->status != TCM_OBJECT_FINGER &&
		    object->status != TCM_OBJECT_GLOVED &&
		    object->status != TCM_OBJECT_STYLUS &&
		    object->status != TCM_OBJECT_ERASER &&
		    object->status != TCM_OBJECT_PALM) {
			input_mt_report_slot_state(input, MT_TOOL_FINGER,
						   false);
			continue;
		}

		x = tcm_scale(object->x, tcm->dev_max_x, tcm->input_max_x);
		y = tcm_scale(object->y, tcm->dev_max_y, tcm->input_max_y);

		major = max(object->x_width, object->y_width);
		minor = min(object->x_width, object->y_width);

		input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
		/* Applies touchscreen-swapped-x-y / -inverted-* as well. */
		touchscreen_report_pos(input, &tcm->prop, x, y, true);
		input_report_abs(input, ABS_MT_PRESSURE, min(object->z, 255u));
		input_report_abs(input, ABS_MT_TOUCH_MAJOR, min(major, 255u));
		input_report_abs(input, ABS_MT_TOUCH_MINOR, min(minor, 255u));
	}

	/* BTN_TOUCH and the single touch emulation come from the slot states;
	 * the finger count adds BTN_TOOL_FINGER once the device advertises it.
	 */
	input_mt_report_pointer_emulation(input, true);

	for (i = 0; i < tcm->num_keycodes; i++)
		input_report_key(input, tcm->keycodes[i],
				 data->buttons & BIT(i));

	input_sync(input);
}

static void tcm_handle_report(struct synaptics_tcm *tcm,
			      const struct tcm_packet *pkt)
{
	struct tcm_touch_data data;
	int error;

	switch (pkt->code) {
	case TCM_REPORT_IDENTIFY:
		/* The part restarted: it is back in application mode but the
		 * touch report is off until it is enabled again.
		 */
		dev_info(tcm->dev, "touch controller restarted\n");
		break;
	case TCM_REPORT_TOUCH:
		if (!tcm->input || tcm->suspended)
			return;
		error = tcm_parse_touch_report(tcm, pkt->payload, pkt->length,
					       &data);
		if (error) {
			/* Once: a report that cannot be parsed is a
			 * configuration the driver does not understand, and
			 * it would otherwise show up only as "no touch".
			 */
			dev_err_once(tcm->dev,
				     "cannot parse touch report (%u bytes, %u byte configuration): %d\n",
				     pkt->length, tcm->report_config_len,
				     error);
			return;
		}
		tcm_report_input(tcm, &data);
		break;
	default:
		dev_dbg(tcm->dev, "unhandled report 0x%02x (%u bytes)\n",
			pkt->code, pkt->length);
		break;
	}
}

/*
 * Fetch one queued event.
 *
 * v2 does not hand a report out on its own: the host asks for it with
 * CMD_TCM2_GET_REPORT (0x0a), which is what the vendor's
 * syna_tcm_v2_read_message() (.text+0x2174) does before reading the packet.
 *
 * A v1 part pushes its messages instead: ATTN is asserted and the host only
 * clocks the message out, with nothing written in between
 * (syna_tcm_get_event_data() -> syna_tcm_v1_read_message()).  Writing anything
 * there would make the part discard the message it is asserting ATTN for.
 */
static int tcm_fetch_event(struct synaptics_tcm *tcm, struct tcm_packet *pkt)
{
	unsigned long timeout;
	int error;

	if (tcm->protocol == TCM_PROTOCOL_V1) {
		error = tcm_read_message(tcm, pkt);
		if (error)
			return error;
		/* Measured on this board: an idle v1 part answers 0xa5 00 00 00
		 * (STATUS_IDLE) once nothing is pending.
		 */
		if (pkt->code == TCM_STATUS_IDLE)
			return -ENODATA;

		return 0;
	}

	error = tcm_write_command(tcm, TCM_CMD_TCM2_GET_REPORT, NULL, 0, pkt);
	if (error)
		return error;

	if (pkt->code == TCM_STATUS_NO_REPORT_AVAILABLE)
		return -ENODATA;
	if (pkt->code >= TCM_REPORT_IDENTIFY)
		return 0;
	if (pkt->code != TCM_STATUS_ACK)
		return -EIO;

	/* STATUS_ACK is followed by the event itself. */
	timeout = jiffies + msecs_to_jiffies(TCM_RESPONSE_TIMEOUT_MS);
	while (time_before(jiffies, timeout)) {
		usleep_range(2000, 3000);

		error = tcm_read_message(tcm, pkt);
		if (error)
			return error;

		if (pkt->code == TCM_STATUS_NO_REPORT_AVAILABLE)
			return -ENODATA;
		if (pkt->code >= TCM_REPORT_IDENTIFY)
			return 0;
		if (pkt->code != TCM_STATUS_ACK)
			return -EIO;
	}

	return -ETIMEDOUT;
}

irqreturn_t synaptics_tcm_irq(int irq, void *data)
{
	struct synaptics_tcm *tcm = data;
	struct tcm_packet pkt;
	int error;

	mutex_lock(&tcm->msg_lock);

	error = tcm_fetch_event(tcm, &pkt);
	if (!error && pkt.code == TCM_REPORT_IDENTIFY) {
		/* Re-arm the touch report after an unexpected part reset. */
		tcm_enable_touch_report(tcm, true);
	} else if (!error) {
		tcm_handle_report(tcm, &pkt);
	}

	mutex_unlock(&tcm->msg_lock);

	/* A level-low interrupt stays asserted while the part has more data,
	 * so the thread is entered again for the next event.
	 */
	return IRQ_HANDLED;
}

/*
 * Tell which TouchComm generation the part speaks.  The vendor's
 * syna_tcm_detect_device() (.text+0x514c) writes one bare byte -- 0x02, i.e.
 * CMD_IDENTIFY, materialised there as "mov w9, #0x2 / strb w9, [sp, #4]", then
 * reads four bytes back without any framing and decides from those.  A v1
 * message is recognised by its 0xa5 marker (syna_tcm_v1_detect, .text+0x104c:
 * "ldrb w8, [x1] / cmp w8, #0xa5"), a v2 one by a header whose CRC-6 passes;
 * the vendor tries v2 first, this does v1 first, which is the stronger test
 * (1/256 against 1/64, and no v1 message measured here has a zero CRC-6).
 *
 * Measured on this board: the bare 0x02 write is answered with "a5 01 1c 00",
 * the v1 identify response, and a 4-byte v2 frame never gets a valid answer.
 *
 * For a v1 part @pkt describes the answer: its header has been consumed, and
 * when the part announced itself with an IDENTIFY report -- which is what a
 * part that has just come up does -- its payload has been read as well, the
 * way syna_tcm_v1_detect() continues that message instead of sending a command
 * while the part still holds its payload back.
 */
static int tcm_detect_protocol(struct synaptics_tcm *tcm,
			       struct tcm_packet *pkt)
{
	u8 data[TCM_HEADER_SIZE];
	unsigned int length;
	int error;

	tcm->protocol = TCM_PROTOCOL_NONE;
	memset(pkt, 0, sizeof(*pkt));

	tcm->tx_buf[0] = TCM_CMD_IDENTIFY;
	error = tcm_bus_write(tcm, tcm->tx_buf, 1);
	if (error)
		return error;

	error = tcm_bus_read(tcm, data, sizeof(data));
	if (error)
		return error;

	dev_dbg(tcm->dev, "bare answer: %*ph\n", (int)sizeof(data), data);

	if (data[0] != TCM_V1_MESSAGE_MARKER) {
		if (!tcm_crc6(data, 32)) {
			tcm->protocol = TCM_PROTOCOL_V2;
			return 0;
		}

		return -ENODEV;
	}

	tcm->protocol = TCM_PROTOCOL_V1;

	/* Keep the header where the payload read appends to it. */
	memcpy(tcm->rx_buf, data, sizeof(data));

	length = data[2] | (data[3] << 8);
	pkt->code = data[1];
	pkt->length = length;

	if (data[1] == TCM_REPORT_IDENTIFY && length) {
		error = tcm_v1_read_payload(tcm, length);
		if (error)
			return error;

		pkt->payload = tcm->rx_buf + TCM_V1_HEADER_SIZE;

		return 0;
	}

	/*
	 * Anything else (measured: STATUS_OK with the identify info) is drained
	 * so that the part is left at a frame boundary; syna_tcm_v1_detect()
	 * does not use that answer either and sends CMD_IDENTIFY for a fresh
	 * one.  A failure here is not fatal -- the command that follows starts
	 * a new message anyway.
	 */
	if (length) {
		error = tcm_v1_read_payload(tcm, length);
		if (error)
			dev_dbg(tcm->dev, "cannot drain the probe answer: %d\n",
				error);
	}

	return 0;
}

/*
 * v2 identify.  A part that has just been powered up or reset announces itself
 * with an IDENTIFY report; the vendor reads it by first sending
 * CMD_TCM2_GET_REPORT and, if that does not yield the report, falls back to an
 * explicit CMD_IDENTIFY and finally CMD_RESET (syna_tcm_v2_detect(), layer 3
 * of the same core library).
 */
static int tcm_v2_identify(struct synaptics_tcm *tcm, struct tcm_packet *pkt)
{
	unsigned long timeout;
	int error;

	error = tcm_write_command(tcm, TCM_CMD_TCM2_GET_REPORT, NULL, 0, pkt);
	if (!error) {
		timeout = jiffies + msecs_to_jiffies(TCM_IDENTIFY_TIMEOUT_MS);
		while (time_before(jiffies, timeout)) {
			if (pkt->code == TCM_REPORT_IDENTIFY)
				return 0;
			if (pkt->code < TCM_REPORT_IDENTIFY)
				break;	/* a response, not the report */
			/* Another report is queued: ask again. */
			usleep_range(5000, 6000);
			error = tcm_write_command(tcm, TCM_CMD_TCM2_GET_REPORT,
						  NULL, 0, pkt);
			if (error)
				break;
		}
	}

	error = tcm_send_command(tcm, TCM_CMD_IDENTIFY, NULL, 0, pkt, true);
	if (error)
		return error;
	if (pkt->code != TCM_REPORT_IDENTIFY) {
		dev_err(tcm->dev, "no identify report, got 0x%02x\n", pkt->code);
		return -EIO;
	}

	return 0;
}

/*
 * v1 identify.  The part answers CMD_IDENTIFY with STATUS_OK whose payload is
 * the identify info, or -- right after it has come up -- with a
 * REPORT_IDENTIFY (0x10) carrying the same payload; both were measured on this
 * board.  The reference accepts either (syna_tcm_v1_detect() reads the startup
 * packet when the header code is REPORT_IDENTIFY, and otherwise sends
 * CMD_IDENTIFY), and no reset is sent: CMD_RESET silences this part for about
 * a second, which is what made earlier bring-up attempts look like a dead
 * chip.
 */
static int tcm_v1_identify(struct synaptics_tcm *tcm,
			   const struct tcm_packet *detected,
			   struct tcm_packet *pkt)
{
	unsigned int attempt;
	int error;

	/* The probe handshake already brought this one in whole. */
	if (detected->code == TCM_REPORT_IDENTIFY &&
	    detected->length >= TCM_IDENTIFY_MIN_LEN) {
		*pkt = *detected;
		return 0;
	}

	for (attempt = 0; attempt < TCM_IDENTIFY_READS; attempt++) {
		error = tcm_send_command(tcm, TCM_CMD_IDENTIFY, NULL, 0, pkt,
					 true);
		if (error)
			return error;

		if (pkt->code == TCM_STATUS_OK ||
		    pkt->code == TCM_REPORT_IDENTIFY)
			return 0;

		/* Another report was queued ahead of the answer, e.g. a touch
		 * report from a part whose report is already on; deliver it and
		 * ask again.
		 */
		tcm_handle_report(tcm, pkt);
	}

	dev_err(tcm->dev, "no identify answer, last code 0x%02x\n", pkt->code);

	return -EIO;
}

static int tcm_identify(struct synaptics_tcm *tcm,
			const struct tcm_packet *detected)
{
	struct tcm_packet pkt;
	unsigned int length;
	int error;

	if (tcm->protocol == TCM_PROTOCOL_V1)
		error = tcm_v1_identify(tcm, detected, &pkt);
	else
		error = tcm_v2_identify(tcm, &pkt);
	if (error)
		return error;

	length = min_t(unsigned int, pkt.length, sizeof(tcm->id_info));
	if (length < TCM_IDENTIFY_MIN_LEN) {
		dev_err(tcm->dev, "short identify report: %u bytes\n",
			pkt.length);
		return -EINVAL;
	}

	memset(&tcm->id_info, 0, sizeof(tcm->id_info));
	memcpy(&tcm->id_info, pkt.payload, length);

	/* Identify layout, from syna_tcm_v1_parse_idinfo() and
	 * syna_tcm_v2_parse_idinfo() (.text+0x2c58): version at +0, mode at +1,
	 * part number at +2, build id at +18, max write size at +22.  Only the
	 * v2 identify carries a max read size at +24 (it sizes the v2 payload
	 * chunks); v1 leaves it at zero.
	 */
	tcm->max_write_size = le16_to_cpu(tcm->id_info.max_write_size);
	if (tcm->protocol == TCM_PROTOCOL_V2) {
		tcm->max_read_size = le16_to_cpu(tcm->id_info.max_read_size);
		if (!tcm->max_read_size ||
		    tcm->max_read_size > TCM_MAX_TRANSFER ||
		    tcm->max_read_size < TCM_HEADER_SIZE + TCM_CRC16_SIZE + 1)
			tcm->max_read_size = TCM_MAX_TRANSFER;
	}

	return 0;
}

static int tcm_input_register(struct synaptics_tcm *tcm)
{
	/* Pre-set to the firmware range: touchscreen_parse_properties() returns
	 * early (leaving @prop untouched) if it cannot allocate absinfo.
	 */
	struct touchscreen_properties prop = {
		.max_x = tcm->dev_max_x,
		.max_y = tcm->dev_max_y,
	};
	struct input_dev *input;
	unsigned int i;
	int error;

	if (!tcm->dev_max_x || !tcm->dev_max_y) {
		dev_err(tcm->dev, "no touch range in application info\n");
		return -EINVAL;
	}

	input = devm_input_allocate_device(tcm->dev);
	if (!input)
		return -ENOMEM;

	input->name = "Synaptics TCM touchscreen";
	input->id.bustype = BUS_SPI;
	input->id.vendor = 0x06cb;	/* Synaptics */

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, tcm->dev_max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, tcm->dev_max_y, 0, 0);

	/* Let the device tree override the firmware's sensor range and pick up
	 * touchscreen-swapped-x-y / touchscreen-inverted-*.
	 */
	touchscreen_parse_properties(input, true, &prop);
	tcm->prop = prop;
	tcm->input_max_x = prop.max_x;
	tcm->input_max_y = prop.max_y;
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, tcm->input_max_x,
			     0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, tcm->input_max_y,
			     0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	input_set_capability(input, EV_KEY, BTN_TOUCH);
	/* INPUT_MT_DIRECT does not advertise the finger count, so
	 * input_mt_report_pointer_emulation() would not emit BTN_TOOL_FINGER
	 * without this.
	 */
	input_set_capability(input, EV_KEY, BTN_TOOL_FINGER);
	for (i = 0; i < tcm->num_keycodes; i++)
		input_set_capability(input, EV_KEY, tcm->keycodes[i]);

	error = input_mt_init_slots(input, tcm->max_objects, INPUT_MT_DIRECT);
	if (error) {
		dev_err(tcm->dev, "cannot init MT slots: %d\n", error);
		return error;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(tcm->dev, "cannot register input device: %d\n", error);
		return error;
	}

	tcm->input = input;

	return 0;
}

static int tcm_read_keycodes(struct synaptics_tcm *tcm)
{
	struct device *dev = tcm->dev;
	int count;

	count = device_property_count_u32(dev, "linux,keycodes");
	if (count <= 0)
		return 0;

	if (count > 32) {
		dev_warn(dev, "only the first 32 keycodes are used\n");
		count = 32;
	}

	tcm->keycodes = devm_kcalloc(dev, count, sizeof(*tcm->keycodes),
				     GFP_KERNEL);
	if (!tcm->keycodes)
		return -ENOMEM;

	if (device_property_read_u32_array(dev, "linux,keycodes",
					   tcm->keycodes, count)) {
		devm_kfree(dev, tcm->keycodes);
		tcm->keycodes = NULL;
		return 0;
	}

	tcm->num_keycodes = count;

	return 0;
}

int synaptics_tcm_probe(struct synaptics_tcm *tcm, const char *firmware_name)
{
	struct tcm_packet detected;
	const struct firmware *firmware;
	unsigned long deadline;
	unsigned int attempt;
	int error;

	tcm->rx_buf = devm_kzalloc(tcm->dev, TCM_MAX_TRANSFER, GFP_KERNEL);
	tcm->tx_buf = devm_kzalloc(tcm->dev, TCM_MAX_TRANSFER, GFP_KERNEL);
	tcm->chunk_buf = devm_kzalloc(tcm->dev, TCM_MAX_TRANSFER, GFP_KERNEL);
	if (!tcm->rx_buf || !tcm->tx_buf || !tcm->chunk_buf)
		return -ENOMEM;

	error = tcm_read_keycodes(tcm);
	if (error)
		return error;

	/*
	 * The vendor requests "synaptics/firmware.img" in
	 * syna_dev_reflash_startup_work() (.text+0x19198) and reflashes the
	 * part only when the image differs from what is running.  Reflashing
	 * is not implemented here, so the image is only loaded and reported; a
	 * missing image must not stop a part that already runs its
	 * application firmware.
	 */
	error = request_firmware(&firmware, firmware_name, tcm->dev);
	if (error) {
		dev_warn(tcm->dev, "firmware %s not available (%d), continuing\n",
			 firmware_name, error);
		firmware = NULL;
	} else {
		dev_info(tcm->dev, "firmware %s: %zu bytes\n", firmware_name,
			 firmware->size);
	}

	/*
	 * The part is not always up 300 ms after its reset.  The one time this
	 * board's controller was seen announcing itself -- its attention line
	 * driven low -- it was minutes into the boot, long after the identify
	 * at probe time had given up; a reset-only cycle then left it silent
	 * again.  Retry for a few seconds before declaring it absent, so a part
	 * that is merely slow is not mistaken for one that is missing.  The
	 * protocol detection runs in the same loop: a part that is not up yet
	 * answers neither the v1 marker test nor the v2 CRC-6 test, and fails
	 * that test in microseconds, so the retries stay cheap.  A part that
	 * does answer the handshake but then stalls is bounded by
	 * TCM_IDENTIFY_PHASE_MS instead of one full response timeout per try.
	 */
	deadline = jiffies + msecs_to_jiffies(TCM_IDENTIFY_PHASE_MS);
	for (attempt = 0; attempt < TCM_IDENTIFY_ATTEMPTS; attempt++) {
		error = tcm_detect_protocol(tcm, &detected);
		if (!error)
			error = tcm_identify(tcm, &detected);
		if (!error)
			break;
		if (attempt + 1 >= TCM_IDENTIFY_ATTEMPTS)
			break;
		if (time_after(jiffies, deadline))
			break;
		msleep(TCM_IDENTIFY_RETRY_MS);
	}
	if (error) {
		dev_err(tcm->dev,
			"touch controller did not identify (%d) after %u attempts; probe continues without touch\n",
			error, attempt);
		goto out;
	}

	/* The part number is a 16-byte, NUL padded ASCII field ("S3910VA0A0"
	 * on this board); %.16s stops at the NUL and never reads past it.
	 */
	dev_info(tcm->dev, "TouchComm v%u, mode 0x%02x, part %.16s\n",
		 tcm->id_info.version, tcm->id_info.mode,
		 (const char *)tcm->id_info.part_number);

	/* The framing that was detected at the top of the loop and the version
	 * the part reports must agree; if they do not, one of the two tests
	 * matched something that is not a TouchComm message.
	 */
	if (tcm->id_info.version != tcm->protocol) {
		dev_err(tcm->dev,
			"identify reports TouchComm v%u but the framing detected is v%u\n",
			tcm->id_info.version, tcm->protocol);
		goto out;
	}

	if (tcm->id_info.mode != TCM_MODE_APPLICATION_FIRMWARE) {
		dev_err(tcm->dev,
			"application firmware not running (mode 0x%02x); firmware update is not implemented\n",
			tcm->id_info.mode);
		goto out;
	}

	error = tcm_read_app_info(tcm);
	if (error) {
		dev_err(tcm->dev,
			"cannot read the application info (%d); probe continues without touch\n",
			error);
		goto out;
	}

	error = tcm_read_report_config(tcm);
	if (error) {
		dev_err(tcm->dev,
			"cannot read the touch report configuration (%d); probe continues without touch\n",
			error);
		goto out;
	}

	error = tcm_input_register(tcm);
	if (error) {
		dev_err(tcm->dev,
			"cannot register the input device (%d); probe continues without touch\n",
			error);
		goto out;
	}

	/*
	 * The event path; this is the only request for the interrupt, and it
	 * is the core that owns it because only the core knows when the report
	 * has been turned on.  The vendor's syna_dev_connect() (.text+0x17858)
	 * requests it as a threaded handler with the device tree's trigger
	 * flags plus IRQF_ONESHOT (.text+0x17b28), and only enables it after
	 * the touch report is on -- a v1 part asserts ATTN as soon as its
	 * report is enabled, and the handler is not ready for that before the
	 * input device exists.
	 */
	if (tcm->irq > 0) {
		error = devm_request_threaded_irq(tcm->dev, tcm->irq, NULL,
						  synaptics_tcm_irq,
						  IRQF_ONESHOT |
						  IRQF_NO_AUTOEN |
						  irq_get_trigger_type(tcm->irq),
						  "synaptics_tcm", tcm);
		if (error) {
			dev_err(tcm->dev, "cannot request IRQ %d (%d)\n",
				tcm->irq, error);
			goto out;
		}
	}

	error = tcm_enable_touch_report(tcm, true);
	if (error) {
		/* The input device stays registered; without the report there
		 * will simply be no events.
		 */
		dev_err(tcm->dev, "touch report not enabled: %d\n", error);
	} else if (tcm->irq > 0) {
		enable_irq(tcm->irq);
	}

out:
	release_firmware(firmware);

	/* Deliberately not an error: the SPI device and the device tree are
	 * fine, only the panel (or its firmware) is missing or still in
	 * bootloader mode.  Every path that gets here has said with dev_err
	 * which step failed, so "bound but silent" is never ambiguous.
	 */
	return 0;
}

void synaptics_tcm_remove(struct synaptics_tcm *tcm)
{
	if (!tcm->input)
		return;

	mutex_lock(&tcm->msg_lock);
	tcm_enable_touch_report(tcm, false);
	mutex_unlock(&tcm->msg_lock);
}

int synaptics_tcm_suspend(struct synaptics_tcm *tcm)
{
	struct tcm_packet response;
	int error;

	if (!tcm->input)
		return 0;

	/* Stop the threaded handler first (it takes msg_lock), then talk to
	 * the part synchronously.
	 */
	if (tcm->irq > 0)
		disable_irq(tcm->irq);
	tcm->suspended = true;

	mutex_lock(&tcm->msg_lock);
	tcm_enable_touch_report(tcm, false);
	error = tcm_send_command(tcm, TCM_CMD_ENTER_DEEP_SLEEP, NULL, 0,
				 &response, false);
	mutex_unlock(&tcm->msg_lock);

	if (error)
		dev_warn(tcm->dev, "cannot enter deep sleep: %d\n", error);

	return 0;
}

int synaptics_tcm_resume(struct synaptics_tcm *tcm)
{
	struct tcm_packet response;
	int error;

	if (!tcm->input)
		return 0;

	mutex_lock(&tcm->msg_lock);
	error = tcm_send_command(tcm, TCM_CMD_EXIT_DEEP_SLEEP, NULL, 0,
				 &response, false);
	if (!error)
		error = tcm_enable_touch_report(tcm, true);
	mutex_unlock(&tcm->msg_lock);

	tcm->suspended = false;
	if (tcm->irq > 0)
		enable_irq(tcm->irq);

	if (error)
		dev_warn(tcm->dev, "cannot resume the touch controller: %d\n",
			 error);

	return 0;
}
