/*
 * This code demonstrates how to use
 * the proposed patch for USB Gadget FunctionFS,
 * which implements mmap.
 * By using mmap you avoid copying data between
 * Linux kernel and user space application.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/eventfd.h>
#include <sys/mman.h>

#include <libaio.h>
#define IOCB_FLAG_RESFD         (1 << 0)

#include <linux/usb/functionfs.h>

/*
 * Define where ConfigFS and FunctionFS are mounted
 */
#define CFGFS_DIR "/config"
#define FFS_DIR   "/usbfunc"

/*
 * Define name of your USB Device Controller
 */
static const char g_udc_name[] = "fe200000.dwc3\n";

/*
 * Use proposed mmap implementation for USB Gadget FunctionFS
 */
#define WITH_FFS_MMAP 1



#define DBG_PRINTF(...) do { \
	printf("%s : ", __func__); \
	printf(__VA_ARGS__); \
	fflush(stdout); \
} while ((0))

struct tst_descriptors {
	struct usb_functionfs_descs_head_v2 header;
	__le32 fs_count;
	__le32 hs_count;
	__le32 ss_count;
	struct {
		struct usb_interface_descriptor intf;
		struct usb_endpoint_descriptor_no_audio sink;
		struct usb_endpoint_descriptor_no_audio source;
	} __attribute__((packed)) fs_descs, hs_descs;
	struct {
		struct usb_interface_descriptor intf;
		struct usb_endpoint_descriptor_no_audio sink;
		struct usb_ss_ep_comp_descriptor sink_comp;
		struct usb_endpoint_descriptor_no_audio source;
		struct usb_ss_ep_comp_descriptor source_comp;
	} ss_descs;
} __attribute__((packed));

// This only works if the architecture is little endian.
static struct tst_descriptors g_ffs_setup = {
	.header = {
		.magic = FUNCTIONFS_DESCRIPTORS_MAGIC_V2,
		.length = sizeof(g_ffs_setup),
		.flags =
			FUNCTIONFS_HAS_FS_DESC |
			FUNCTIONFS_HAS_HS_DESC |
			FUNCTIONFS_HAS_SS_DESC
	},
	.fs_count = 3,
	.fs_descs = {
		.intf = {
			.bLength = sizeof(g_ffs_setup.fs_descs.intf),
			.bDescriptorType = USB_DT_INTERFACE,
			.bNumEndpoints = 2,
			.bInterfaceClass = USB_CLASS_VENDOR_SPEC,
			.iInterface = 1,
		},
		.sink = {
			.bLength = sizeof(g_ffs_setup.fs_descs.sink),
			.bDescriptorType = USB_DT_ENDPOINT,
			.bEndpointAddress = 1 | USB_DIR_IN,
			.bmAttributes = USB_ENDPOINT_XFER_BULK,
			/* .wMaxPacketSize = autoconfiguration (kernel) */
		},
		.source = {
			.bLength = sizeof(g_ffs_setup.fs_descs.source),
			.bDescriptorType = USB_DT_ENDPOINT,
			.bEndpointAddress = 2 | USB_DIR_OUT,
			.bmAttributes = USB_ENDPOINT_XFER_BULK,
			/* .wMaxPacketSize = autoconfiguration (kernel) */
		},
	},
	.hs_count = 3,
	.hs_descs = {
		.intf = {
			.bLength = sizeof(g_ffs_setup.hs_descs.intf),
			.bDescriptorType = USB_DT_INTERFACE,
			.bNumEndpoints = 2,
			.bInterfaceClass = USB_CLASS_VENDOR_SPEC,
			.iInterface = 1,
		},
		.sink = {
			.bLength = sizeof(g_ffs_setup.hs_descs.sink),
			.bDescriptorType = USB_DT_ENDPOINT,
			.bEndpointAddress = 1 | USB_DIR_IN,
			.bmAttributes = USB_ENDPOINT_XFER_BULK,
			.wMaxPacketSize = 512,
		},
		.source = {
			.bLength = sizeof(g_ffs_setup.hs_descs.source),
			.bDescriptorType = USB_DT_ENDPOINT,
			.bEndpointAddress = 2 | USB_DIR_OUT,
			.bmAttributes = USB_ENDPOINT_XFER_BULK,
			.wMaxPacketSize = 512,
			.bInterval = 1, /* NAK every 1 uframe */
		},
	},
	.ss_count = 5,
	.ss_descs = {
		.intf = {
			.bLength = sizeof(g_ffs_setup.ss_descs.intf),
			.bDescriptorType = USB_DT_INTERFACE,
			.bNumEndpoints = 2,
			.bInterfaceClass = USB_CLASS_VENDOR_SPEC,
			.iInterface = 1,
		},
		.sink = {
			.bLength = sizeof(g_ffs_setup.ss_descs.sink),
			.bDescriptorType = USB_DT_ENDPOINT,
			.bEndpointAddress = 1 | USB_DIR_IN,
			.bmAttributes = USB_ENDPOINT_XFER_BULK,
			.wMaxPacketSize = 1024,
		},
		.sink_comp = {
			.bLength = USB_DT_SS_EP_COMP_SIZE,
			.bDescriptorType = USB_DT_SS_ENDPOINT_COMP,
			.bMaxBurst = 0,
			.bmAttributes = 0,
			.wBytesPerInterval = 0,
		},
		.source = {
			.bLength = sizeof(g_ffs_setup.ss_descs.source),
			.bDescriptorType = USB_DT_ENDPOINT,
			.bEndpointAddress = 2 | USB_DIR_OUT,
			.bmAttributes = USB_ENDPOINT_XFER_BULK,
			.wMaxPacketSize = 1024,
			.bInterval = 1, /* NAK every 1 uframe */
		},
		.source_comp = {
			.bLength = USB_DT_SS_EP_COMP_SIZE,
			.bDescriptorType = USB_DT_SS_ENDPOINT_COMP,
			.bMaxBurst = 0,
			.bmAttributes = 0,
			.wBytesPerInterval = 0,
		},
	},
};

#define STR_INTERFACE_TEXT "TST"

struct tst_usb_strings {
	struct usb_functionfs_strings_head header;
	struct {
		__le16 code;
		const char str1[sizeof(STR_INTERFACE_TEXT)];
	} __attribute__((packed)) lang0;
} __attribute__((packed));

static struct tst_usb_strings g_ffs_strings = {
	.header = {
		.magic = FUNCTIONFS_STRINGS_MAGIC,
		.length = sizeof(g_ffs_strings),
		.str_count = 1,
		.lang_count = 1,
	},
	.lang0 = {
		0x0409, /* en-us */
		STR_INTERFACE_TEXT,
	}
};
#define STR_INTERFACE g_ffs_strings.lang0.str1

enum {
	USB_REQ_ECHO     = 1,
	USB_REQ_GENERATE = 2,
};
enum {
	USB_RESP_ECHO     = 1,
	USB_RESP_GENERATE = 2,
};

// define number of IOCBs used for each bulk endpoint
// MUST be a power of 2
#define IOCB_NR 4
#define FREE_ENTRIES_NR (IOCB_NR*2)

// DMA buffer is 1MiB
// Total allocation 8MiB
#define ALLOC_SZ 0x100000

struct free_array {
	int idx;
	int sz;
	uint8_t *buffers[FREE_ENTRIES_NR];
};

static struct {
	int ep0_fd;
	int usb_ready;

	int ep_in_fd;     // WRITE: device -> host     ( IN EP)
	int ep_out_fd;    // READ:  host   -> device   (OUT EP)

	int bulk_ev_fd;   // events for completed IN/OUT requests

	uint8_t *usb_data_buffers[FREE_ENTRIES_NR];

	struct free_array free_array;

	io_context_t ioctx;

	int rd_submit_idx;
	int rd_pending;
	struct iocb rd_iocb[IOCB_NR];

	int wr_submit_idx;
	int wr_pending;
	struct iocb wr_iocb[IOCB_NR];
} g;

#define IOCB_NEXT_IDX(idx_) do { \
	idx_ = ((idx_)+1)&(IOCB_NR-1); \
} while ((0))


static void transfer_buf_init(struct free_array *free_array, uint8_t **buf)
{
	int i;

	free_array->idx = 0;
	free_array->sz  = FREE_ENTRIES_NR;
	for (i=0; i < FREE_ENTRIES_NR; i++) {
		free_array->buffers[i] = buf[i];
	}
}

static uint8_t* transfer_buf_alloc(struct free_array *free_array)
{
	uint8_t *res;

	if (free_array->sz <= 0)
		return NULL;

	res = free_array->buffers[free_array->idx];
	free_array->idx = (free_array->idx+1)&(FREE_ENTRIES_NR - 1);
	free_array->sz--;
	return res;
}

static void transfer_buf_free(struct free_array *free_array, uint8_t *buf)
{
	int idx;

	if (free_array->sz >= FREE_ENTRIES_NR)
		return;

	idx = (free_array->idx + free_array->sz)&(FREE_ENTRIES_NR - 1);
	free_array->buffers[idx] = buf;
	free_array->sz++;
}

static void ep0_display_event(struct usb_functionfs_event *event)
{
	static const char *const names[] = {
		[FUNCTIONFS_BIND] = "BIND",
		[FUNCTIONFS_UNBIND] = "UNBIND",
		[FUNCTIONFS_ENABLE] = "ENABLE",
		[FUNCTIONFS_DISABLE] = "DISABLE",
		[FUNCTIONFS_SETUP] = "SETUP",
		[FUNCTIONFS_SUSPEND] = "SUSPEND",
		[FUNCTIONFS_RESUME] = "RESUME",
	};
	switch (event->type) {
	case FUNCTIONFS_SETUP:
	case FUNCTIONFS_BIND:
	case FUNCTIONFS_UNBIND:
	case FUNCTIONFS_ENABLE:
	case FUNCTIONFS_DISABLE:
	case FUNCTIONFS_SUSPEND:
	case FUNCTIONFS_RESUME:
		DBG_PRINTF("ep0 event %s\n", names[event->type]);
		break;
	default:
		DBG_PRINTF("ep0 UNKNOWN event: %d\n", event->type);
		break;
	}
}

static void ep_out_start(void);

static void ep0_handle_event(int events)
{
	struct usb_functionfs_event event[8];
	int ret, i;

	ret = read(g.ep0_fd, event, sizeof(event));
	if (ret <= 0) {
		DBG_PRINTF("unable to read event from ep0\n");
		return;
	}
	ret = ret/sizeof(event[0]);
	for (i=0; i < ret; i++) {
		ep0_display_event(event+i);
		switch (event[i].type) {
		case FUNCTIONFS_SETUP:
			if (event[i].u.setup.bRequestType & USB_DIR_IN)
				write(g.ep0_fd, NULL, 0);
			else
				read(g.ep0_fd, NULL, 0);
			break;

		case FUNCTIONFS_ENABLE:
			g.usb_ready = 1;
			ep_out_start();
			break;

		case FUNCTIONFS_UNBIND:
		case FUNCTIONFS_DISABLE:
			g.usb_ready = 0;
			break;

		default:
			break;
		}
	}
}


// start read requests for host->device transfers, via OUT Endpoint
static void ep_out_start(void)
{
	struct iocb *rd_iocb;
	uint8_t *transfer_buf;
	int ret;

	if (!g.usb_ready)
		return;

	while (g.rd_pending < 2) {
		rd_iocb = &g.rd_iocb[g.rd_submit_idx];
		transfer_buf = transfer_buf_alloc(&g.free_array);

		io_prep_pread(
			rd_iocb, g.ep_out_fd,
			transfer_buf, ALLOC_SZ,
			0
		);
		// make this IOCB generate an event
		// on bulk_ev_fd, once it is completed
		rd_iocb->u.c.flags |= IOCB_FLAG_RESFD; // Result FD
		rd_iocb->u.c.resfd = g.bulk_ev_fd;
retry:
		// submit IOCB to receive data from host
		ret = io_submit(g.ioctx, 1, &rd_iocb);
		if (ret < 0) {
			DBG_PRINTF("Error io_submit\n");
			DBG_PRINTF("    ret == %d\n", ret);
			if (ret == -EINTR)
				goto retry;
			transfer_buf_free(&g.free_array, transfer_buf);
			if (ret == -EAGAIN || ret == -ESHUTDOWN)
				DBG_PRINTF("    Cable disconnected.\n");
			return;
		}
		if (ret != 1) {
			DBG_PRINTF("io_submit returned %d ?!\n", ret);
			return;
		}
		g.rd_pending++;
		IOCB_NEXT_IDX(g.rd_submit_idx);
	}
}

// start a write to host via IN Endpoint
static void ep_in_submit(uint8_t *usb_data, uint32_t usb_data_sz)
{
	struct iocb *wr_iocb;
	int ret;

	if (!g.usb_ready) {
		transfer_buf_free(&g.free_array, usb_data);
		return;
	}

	wr_iocb = &(g.wr_iocb[g.wr_submit_idx]);
	io_prep_pwrite(
		wr_iocb, g.ep_in_fd,
		usb_data, usb_data_sz,
		0
	);
	// make this IOCB generate an event
	// on bulk_ev_fd, once it is completed
	wr_iocb->u.c.flags |= IOCB_FLAG_RESFD; // Result FD
	wr_iocb->u.c.resfd  = g.bulk_ev_fd;
retry:
	// submit IOCB to send data back to USB host
	ret = io_submit(g.ioctx, 1, &wr_iocb);
	if (ret < 0) {
		DBG_PRINTF("Error io_submit\n");
		DBG_PRINTF("    ret == %d\n", ret);
		if (ret == -EINTR)
			goto retry;
		transfer_buf_free(&g.free_array, usb_data);
		if (ret == -EAGAIN || ret == -ESHUTDOWN)
			DBG_PRINTF("    Cable disconnected.\n");
		return;
	}
	if (ret != 1) {
		DBG_PRINTF("io_submit returned %d ?!\n", ret);
		transfer_buf_free(&g.free_array, usb_data);
		return;
	}
	g.wr_pending++;
	IOCB_NEXT_IDX(g.wr_submit_idx);
}

static void req_handle_echo(uint8_t *req_data, int req_sz)
{
	ep_in_submit(req_data, req_sz);
}

static void ffs_poll(void);

static void fill_gen_data(
	uint64_t *buf, int buf_sz,
	uint64_t value, uint64_t add
)
{
	uint64_t *buf_end;

	value &= ~0xFFull;
	value |= USB_RESP_GENERATE;
	buf--;
	buf_end = buf + (buf_sz/sizeof(uint64_t));
	while (buf < buf_end) {
		*(++buf) = value;
		value += add;
	}
}

static void req_handle_generate_data(uint8_t *req_data, int req_sz)
{
	uint32_t transfer_sz;
	uint32_t total;
	uint32_t started;
	uint32_t pending;
	uint32_t finished;
	uint64_t value;
	uint64_t add;
	uint8_t *resp_buf;

	if (req_sz != 24) {
		DBG_PRINTF("wrong request size %d\n", req_sz);
		transfer_buf_free(&g.free_array, req_data);
		return;
	}
	transfer_sz = *((uint32_t *)(req_data+0)) >> 8;
	total       = *((uint32_t *)(req_data+4));
	value       = *((uint64_t *)(req_data+8));
	add         = *((uint64_t *)(req_data+16));
	transfer_buf_free(&g.free_array, req_data);

	DBG_PRINTF(
		"starting %d transfers of %d bytes each\n",
		total, transfer_sz
	);
	started  = 0;
	finished = 0;
	while (finished < total) {
		while (started < total && g.wr_pending < 3) {
			resp_buf = transfer_buf_alloc(&g.free_array);
			fill_gen_data(
				(uint64_t *)resp_buf, transfer_sz,
				value, add
			);
			ep_in_submit(resp_buf, transfer_sz);
			started++;
			value += 0x0100010001000100ull;
		}
		pending = g.wr_pending;
		ffs_poll();
		finished += pending - g.wr_pending;
	}
	DBG_PRINTF("%d transfers finished\n", finished);
}

// handle received data
static void ep_out_complete(struct io_event *rd_event)
{
	int rd_result;
	uint8_t *rd_data;

	if (!g.rd_pending) {
		DBG_PRINTF("no read pending\n");
		return;
	}
	g.rd_pending--;

	// already start next read request
	// for truly pipelined operation
	ep_out_start();

	// rd_event->res: result
	//   < 0   error
	//   >=0   number of transmitted bytes
	// need to cast to make sure this is signed!
	rd_result = (int)rd_event->res;
	rd_data = rd_event->obj->u.c.buf;
	if (rd_result < 0) {
		DBG_PRINTF("READ error: %d\n", rd_result);
		transfer_buf_free(&g.free_array, rd_data);
		return;
	}

	switch(rd_data[0]) {
	case USB_REQ_ECHO:
		req_handle_echo(rd_data, rd_result);
		break;
	case USB_REQ_GENERATE:
		req_handle_generate_data(rd_data, rd_result);
		break;
	default:
		DBG_PRINTF("received %d bytes\n", rd_result);
		transfer_buf_free(&g.free_array, rd_data);
	}
}

// write finished == device->host USB transfer finished.
static void ep_in_complete(struct io_event *wr_event)
{
	int wr_result;
	uint8_t *usb_data;

	if (!g.wr_pending) {
		DBG_PRINTF("no write pending\n");
		return;
	}
	g.wr_pending--;

	// wr_event->res: result
	//   < 0   error
	//   >=0   number of transmitted bytes
	// need to cast to make sure this is signed!
	wr_result = (int)wr_event->res;
	usb_data  = wr_event->obj->u.c.buf;

	if (wr_result < 0) {
		DBG_PRINTF("WRITE error: %d\n", wr_result);
	}
	if ((int)wr_event->obj->u.c.nbytes != wr_result) {
		DBG_PRINTF("length mismatch\n");
		DBG_PRINTF(
			"   requestd %d bytes, wrote %d bytes\n",
			(int)wr_event->obj->u.c.nbytes,
			wr_result
		);
	}
	transfer_buf_free(&g.free_array, usb_data);
}

static void ep_bulk_handle_event(int events)
{
	ssize_t sz, i;
	uint64_t events_nr;
	int events_max;
	struct io_event ioevents[IOCB_NR*2];

	sz = read(g.bulk_ev_fd, &events_nr, sizeof(events_nr));
	if (sz<0) {
		DBG_PRINTF("Can't read eventfd\n");
		return;
	}
	if (events_nr > IOCB_NR*2) {
		DBG_PRINTF(
			"Received too many ioevents: %d\n",
			(int)events_nr
		);
		return;
	}
	if (events_nr == 0) {
		DBG_PRINTF("Received ZERO ioevents ?!\n");
		return;
	}
	events_max = g.rd_pending + g.wr_pending;
	if (events_nr > (uint64_t)events_max) {
		DBG_PRINTF("Received more ioevents than expected\n");
		DBG_PRINTF(
			"    max: %d, events_nr: %d\n",
			events_max, (int)events_nr
		);
		return;
	}
	// we should read exactly events_nr,
	// to keep event counter in sync
	sz = io_getevents(g.ioctx, events_nr, events_nr, ioevents, NULL);
	if (sz != events_nr) {
		DBG_PRINTF("Received wrong number of io ioevents\n");
		DBG_PRINTF(
			"    expected %d got %d\n",
			(int)events_nr, (int)sz
		);
		return;
	}
	// DBG_PRINTF("received %d ioevents\n",(int)sz);
	for(i = 0; i < sz; i++) {
		if (ioevents[i].obj->aio_fildes == g.ep_out_fd) {
			ep_out_complete(&ioevents[i]);
			continue;
		}
		if (ioevents[i].obj->aio_fildes == g.ep_in_fd) {
			ep_in_complete(&ioevents[i]);
			continue;
		}
		DBG_PRINTF("Event for unknown file descriptor!\n");
		return;
	}

}

static int ffs_init(void)
{
	ssize_t ret;
	int udc_fd, i;

	udc_fd = open(CFGFS_DIR "/usb_gadget/g1/UDC", O_WRONLY);
	if (udc_fd < 0) {
		DBG_PRINTF(
			"Unable to open "
			CFGFS_DIR "/usb_gadget/g1/UDC"
			" for writing\n"
		);
		return -2;
	}

	// setup AIO context to handle up to 2*IOCB_NR requests
	ret = io_setup(2*IOCB_NR, &g.ioctx);
	if (ret < 0) {
		DBG_PRINTF("Unable to setup aio\n");
		return -3;
	}
	// eventfd to get notified about completed USB bulk EP transactions
	g.bulk_ev_fd = eventfd(0, 0);
	if (g.bulk_ev_fd < 0) {
		perror("unable to create eventfd\n");
		return -4;
	}
	g.ep0_fd = open(FFS_DIR "/ep0", O_RDWR);
	if (g.ep0_fd < 0) {
		DBG_PRINTF("Cannot open " FFS_DIR "/ep0\n");
		return -5;
	}
	// Tell FunctionFS about descriptors/endpoints
	ret = write(g.ep0_fd, &g_ffs_setup, sizeof(g_ffs_setup));
	if (ret < 0) {
		DBG_PRINTF("Cannot setup Interface/Endpoint descriptors\n");
		return -6;
	}
	// Tell FunctionFS about strings
	ret = write(g.ep0_fd, &g_ffs_strings, sizeof(g_ffs_strings));
	if (ret < 0) {
		DBG_PRINTF("Cannot setup string descriptors\n");
		return -7;
	}

#if WITH_FFS_MMAP
	for(i=0; i < FREE_ENTRIES_NR; i++) {
		g.usb_data_buffers[i] = mmap(
			NULL, ALLOC_SZ,
			PROT_READ|PROT_WRITE, MAP_SHARED,
			g.ep0_fd, 0
		);
		if (g.usb_data_buffers[i] == MAP_FAILED) {
			DBG_PRINTF(
				"Cannot mmap usb_data_buffers[%d].\n",
				i
			);
			return -1;
		}
	}
#else
	for(i=0; i < FREE_ENTRIES_NR; i++) {
		ret = posix_memalign(
			(void **)(&g.usb_data_buffers[i]),
			0x1000, ALLOC_SZ
		);
		if (ret) {
			DBG_PRINTF(
				"Cannot allocate usb_data_buffers[%d]: "
				"err = %d\n",
				i, (int)ret
			);
			return -1;
		}
	}
#endif
	transfer_buf_init(&g.free_array, g.usb_data_buffers);

	// now endpoint files should be created
	// Use O_NONBLOCK to not block, when USB device is disconnected
	g.ep_in_fd = open(FFS_DIR "/ep1", O_RDWR|O_NONBLOCK);
	if (g.ep_in_fd < 0) {
		DBG_PRINTF("Cannot open IN  " FFS_DIR "/ep1\n");
		return -8;
	}
	g.ep_out_fd = open(FFS_DIR "/ep2", O_RDWR|O_NONBLOCK);
	if (g.ep_out_fd < 0) {
		DBG_PRINTF("Cannot open OUT " FFS_DIR "/ep2\n");
		return -9;
	}

	// Now finally bind to USB Device Controller driver
	write(udc_fd, g_udc_name, sizeof(g_udc_name) - 1);
	close(udc_fd);

	return 0;
}

static void ffs_poll(void)
{
	int status;
	struct pollfd poll_array[2];

	poll_array[0].fd      = g.ep0_fd;
	poll_array[0].events  = POLLIN;
	poll_array[0].revents = 0;
	poll_array[1].fd      = g.bulk_ev_fd;
	poll_array[1].events  = POLLIN;
	poll_array[1].revents = 0;

	status = poll(poll_array, 2, 1000);

	if (status <= 0)
		return;

	if (poll_array[0].revents & POLLIN)
		ep0_handle_event(poll_array[0].revents);

	if (poll_array[1].revents & POLLIN)
		ep_bulk_handle_event(poll_array[1].revents);
}

int main(void)
{
	int rc;

	rc = ffs_init();
	if (rc < 0)
		return 1;

	for(;;) {
		ffs_poll();
	}
}
