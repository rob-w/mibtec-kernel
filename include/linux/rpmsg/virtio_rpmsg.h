#ifndef _LINUX_RPMSG_VIRTIO_RPMSG_H
#define _LINUX_RPMSG_VIRTIO_RPMSG_H

/*
 * We're allocating buffers of 512 bytes each for communications. The
 * number of buffers will be computed from the number of buffers supported
 * by the vring, upto a maximum of 512 buffers (256 in each direction).
 *
 * Each buffer will have 16 bytes for the msg header and 496 bytes for
 * the payload.
 *
 * This will utilize a maximum total space of 256KB for the buffers.
 *
 * We might also want to add support for user-provided buffers in time.
 * This will allow bigger buffer size flexibility, and can also be used
 * to achieve zero-copy messaging.
 *
 * Note that these numbers are purely a decision of this driver - we
 * can change this without changing anything in the firmware of the remote
 * processor.
 */

#define MAX_RPMSG_NUM_BUFS	(2048)
#define MAX_RPMSG_BUF_SIZE	(2048)

/**
 * struct rpmsg_hdr - common header for all virtio rpmsg messages
 * @src: source address
 * @dst: destination address
 * @reserved: reserved for future use
 * @len: length of payload (in bytes)
 * @flags: message flags
 * @data: @len bytes of message payload data
 *
 * Every message sent(/received) on the rpmsg bus begins with this header.
 */
struct rpmsg_hdr {
	u32 src;
	u32 dst;
	u32 reserved;
	u16 len;
	u16 flags;
	u8 data[0];
} __packed;

#endif
