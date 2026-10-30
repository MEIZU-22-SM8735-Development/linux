/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Synaptics TouchComm (TCM) touchscreen protocol definitions.
 *
 * Both protocol generations are implemented.  The part on this board (an
 * S3910) speaks TouchComm v1, which was measured byte for byte with a raw SPI
 * probe; the v2 framing was recovered from the vendor module
 * synaptics_tcm2_ts.ko (vendor-ref/meizu-stock/vendor_dlkm/lib/modules/) and
 * is kept for v2 parts.  The runtime discriminator is in the core driver.
 *
 * TouchComm v2:
 *   - header: 4 bytes { code, length_le16, seq << 6 | crc6 }
 *     (syna_tcm_v2_write, inlined in the vendor module at .text+0x3fdc)
 *   - a payload carrying packet appends a big-endian CRC-16
 *   - payload chunks are read one after another, each chunk preceded by a
 *     CMD_TCM2_ACK (0x07) write from the host
 *   - reads clock data out with 0xff dummy bytes in full duplex
 *     (syna_spi_read at .text+0x1342c)
 *
 * TouchComm v1 (reference implementation:
 * vendor-ref/oneplus-dt/.../s3910/tcm/synaptics_touchcom_core_v1.c):
 *   - host frame: 3 bytes { command, length_le16 } plus payload, no CRC
 *     (syna_tcm_v1_write)
 *   - device message: { 0xa5, code, length_le16 } plus payload, then 0x5a
 *     padding; the length field says where the payload ends
 *   - the header is clocked out first, the payload follows as a chunk led by
 *     { 0xa5, STATUS_CONTINUED_READ }
 */

#ifndef _SYNAPTICS_TCM_H
#define _SYNAPTICS_TCM_H

#include <linux/device.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/mutex.h>
#include <linux/stddef.h>
#include <linux/types.h>

/* TouchComm v2 framing. */
#define TCM_HEADER_SIZE			4
#define TCM_CRC16_SIZE			2

/*
 * TouchComm v1 framing.  The host header size is the reference's
 * syna_tcm_v1_write() (three bytes), the marker, padding and continuation
 * prefix come from its syna_tcm_v1_read_message()/syna_tcm_v1_continued_read()
 * (TCM_V1_MESSAGE_MARKER, TCM_V1_MESSAGE_PADDING, and a chunk header of
 * { marker, STATUS_CONTINUED_READ } whose data starts at buf[2]).  The frame
 * layout was confirmed on this board: writing "02 00 00" makes the part answer
 * "a5 01 1c 00 <28-byte identify payload>", and reading only the header and
 * then reading again returns a "a5 03 ..." continuation.
 */
#define TCM_V1_HOST_HEADER_SIZE		3
#define TCM_V1_HEADER_SIZE		4
#define TCM_V1_CHUNK_HEADER_SIZE	2
#define TCM_V1_MESSAGE_MARKER		0xa5
#define TCM_V1_MESSAGE_PADDING		0x5a

/*
 * Payload bytes fetched per v1 continuation chunk.  The reference sizes its
 * chunks from the part's max read size (syna_tcm_v1_continued_read: chunk_space
 * = max_rd_size - 2), but a v1 part never reports one, so it always asks for
 * the whole remaining payload in a single read.  A bounded chunk instead keeps
 * a long answer -- the report configuration is 128 bytes -- from depending on
 * the part handing out a whole frame in one transfer, and every continuation is
 * framed the same way, so this costs one extra transfer at most.
 */
#define TCM_V1_CHUNK_PAYLOAD		64

/*
 * Touch report configuration entities.  The names say v2 but the v1 part
 * answers CMD_GET_TOUCH_REPORT_CONFIG with the same entity stream (measured on
 * this board: 0x25 returns a 128-byte configuration starting with the gesture
 * entities 0xca/0x10/0xc6 and then a FOREACH_ACTIVE_OBJECT block), so one
 * configuration walker serves both protocols.
 */
#define TCM_TR_END			0x00
#define TCM_TR_FOREACH_ACTIVE_OBJECT	0x01
#define TCM_TR_FOREACH_OBJECT		0x02
#define TCM_TR_FOREACH_END		0x03
#define TCM_TR_PAD_TO_NEXT_BYTE		0x04
#define TCM_TR_TIMESTAMP		0x05
#define TCM_TR_OBJECT_INDEX		0x06
#define TCM_TR_OBJECT_CLASSIFICATION	0x07
#define TCM_TR_OBJECT_X			0x08
#define TCM_TR_OBJECT_Y			0x09
#define TCM_TR_OBJECT_Z			0x0a
#define TCM_TR_OBJECT_X_WIDTH		0x0b
#define TCM_TR_OBJECT_Y_WIDTH		0x0c
#define TCM_TR_BUTTONS_STATE		0x0f
#define TCM_TR_NUM_OF_ACTIVE_OBJECTS	0x18

/* Object classification, from the same report entities.  Values match the
 * reference enum in synaptics_touchcom_func_touch.h (LIFT = 0 .. PALM = 6).
 */
#define TCM_OBJECT_LIFT		0
#define TCM_OBJECT_FINGER	1
#define TCM_OBJECT_GLOVED	2
#define TCM_OBJECT_STYLUS	3
#define TCM_OBJECT_ERASER	4
#define TCM_OBJECT_SMALL	5
#define TCM_OBJECT_PALM		6

/*
 * Commands, status codes and report codes.  The reference uses one enum for
 * both protocol generations (synaptics_touchcom_core_dev.h: enum tcm_command,
 * enum tcm_status_code, enum tcm_report_type), and so does this driver; only
 * the framing around them differs.  "02 00 00" was measured on this board to
 * return the identify info, and "0a 00 00" (a v2-only command) to return
 * 0x0e "not implemented", which is how this enum reads in the v1 core too.
 */
#define TCM_CMD_IDENTIFY		0x02
#define TCM_CMD_RESET			0x04
#define TCM_CMD_ENABLE_REPORT		0x05
#define TCM_CMD_DISABLE_REPORT		0x06
#define TCM_CMD_TCM2_ACK		0x07
#define TCM_CMD_TCM2_GET_REPORT		0x0a
#define TCM_CMD_ENTER_DEEP_SLEEP	0x2c
#define TCM_CMD_EXIT_DEEP_SLEEP		0x2d
#define TCM_CMD_GET_APPLICATION_INFO	0x20
#define TCM_CMD_GET_TOUCH_REPORT_CONFIG	0x25

/* v1: no command pending and no report available. */
#define TCM_STATUS_IDLE			0x00
#define TCM_STATUS_OK			0x01
#define TCM_STATUS_CONTINUED_READ	0x03
#define TCM_STATUS_NO_REPORT_AVAILABLE	0x04
#define TCM_STATUS_ACK			0x07
#define TCM_STATUS_CMD_FAILED		0x09
#define TCM_STATUS_NOT_IMPLEMENTED	0x0e
/* The vendor treats this as "read corrupted" and retries it (see
 * tcm_write_command() and the payload chunk loop in tcm_v2_read_message()).
 */
#define TCM_STATUS_PACKET_CORRUPTED	0xfe

#define TCM_REPORT_IDENTIFY		0x10
#define TCM_REPORT_TOUCH		0x11

/* Where the identify info names the running firmware mode. */
#define TCM_MODE_APPLICATION_FIRMWARE	0x01

/* TouchComm generations, as reported by the identify info's version byte. */
#define TCM_PROTOCOL_NONE		0
#define TCM_PROTOCOL_V1			1
#define TCM_PROTOCOL_V2			2

#define TCM_MAX_OBJECTS			10
/* Largest payload we assemble; the vendor caps a transfer at the identify
 * packet's max_read_size and we clamp that to this value.
 */
#define TCM_MAX_PAYLOAD			4096
#define TCM_MAX_TRANSFER \
	(TCM_MAX_PAYLOAD + TCM_HEADER_SIZE + TCM_CRC16_SIZE)

struct tcm_identify_info {
	u8 version;
	u8 mode;
	u8 part_number[16];
	u8 build_id[4];
	__le16 max_write_size;
	__le16 max_read_size;
	u8 reserved[6];
} __packed;

struct tcm_application_info {
	__le16 version;
	__le16 status;
	__le16 static_config_size;
	__le16 dynamic_config_size;
	__le16 app_config_start_write_block;
	__le16 app_config_size;
	__le16 max_touch_report_config_size;
	__le16 max_touch_report_payload_size;
	u8 customer_config_id[16];
	__le16 max_x;
	__le16 max_y;
	__le16 max_objects;
	__le16 num_of_buttons;
	__le16 num_of_image_rows;
	__le16 num_of_image_cols;
	__le16 has_hybrid_data;
	__le16 num_of_force_elecs;
} __packed;

/*
 * Shortest identify and application info packets the driver can work with.
 * The v1 identify payload (reference: struct tcm_identification_info, whose
 * v2 extension starts at max_read_size) is 24 bytes and this board's part
 * answers with 28; the application info it returns is 46 bytes, two short of
 * the 48-byte v2 structure, so only the fields up to max_objects are required.
 */
#define TCM_IDENTIFY_MIN_LEN \
	(offsetof(struct tcm_identify_info, max_write_size) + sizeof(__le16))
#define TCM_APP_INFO_MIN_LEN \
	(offsetof(struct tcm_application_info, max_objects) + sizeof(__le16))

struct synaptics_tcm;

struct synaptics_tcm_ops {
	int (*read)(struct synaptics_tcm *tcm, void *buf, size_t count);
	int (*write)(struct synaptics_tcm *tcm, const void *buf, size_t count);
};

struct tcm_packet {
	u8 code;
	unsigned int length;
	const u8 *payload;
};

struct tcm_object {
	unsigned int status;
	unsigned int x;
	unsigned int y;
	unsigned int z;
	unsigned int x_width;
	unsigned int y_width;
};

struct tcm_touch_data {
	struct tcm_object object[TCM_MAX_OBJECTS];
	unsigned int buttons;
};

struct synaptics_tcm {
	struct device *dev;
	const struct synaptics_tcm_ops *ops;

	struct input_dev *input;
	/* Serialises every message exchange; the threaded IRQ handler and the
	 * command paths both take it.
	 */
	struct mutex msg_lock;
	u8 seq;

	/* TCM_PROTOCOL_V1 or TCM_PROTOCOL_V2, set by the probe's detection
	 * step; it selects the framing used by every message exchange.
	 */
	u8 protocol;

	u8 *rx_buf;
	u8 *tx_buf;
	u8 *chunk_buf;
	unsigned int max_read_size;
	unsigned int max_write_size;

	struct tcm_identify_info id_info;
	struct tcm_application_info app_info;

	/* Touch report configuration, sized from the part's response. */
	u8 *report_config;
	unsigned int report_config_len;

	struct touchscreen_properties prop;
	unsigned int dev_max_x;
	unsigned int dev_max_y;
	unsigned int input_max_x;
	unsigned int input_max_y;
	unsigned int max_objects;

	u32 *keycodes;
	unsigned int num_keycodes;

	int irq;
	bool suspended;
};

int synaptics_tcm_probe(struct synaptics_tcm *tcm, const char *firmware_name);
void synaptics_tcm_remove(struct synaptics_tcm *tcm);
int synaptics_tcm_suspend(struct synaptics_tcm *tcm);
int synaptics_tcm_resume(struct synaptics_tcm *tcm);
irqreturn_t synaptics_tcm_irq(int irq, void *data);

#endif /* _SYNAPTICS_TCM_H */
