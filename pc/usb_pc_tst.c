#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <linux/usbdevice_fs.h>
#include <linux/usb/ch9.h>
#include <stdint.h>
#include <errno.h>
#include <error.h>
#include <malloc.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <time.h>
#include <poll.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>

#define DBG_PRINTF(...) do { \
	printf("%s : ", __func__); \
	printf(__VA_ARGS__); \
	fflush(stdout); \
} while ((0))

#define ALLOC_SZ (0x100000)

#define MAX_URBS 16
union transferData {
	uint8_t  data8[ALLOC_SZ];
	uint32_t data32[ALLOC_SZ/sizeof(uint32_t)];
	uint64_t data64[ALLOC_SZ/sizeof(uint64_t)];
};

static struct {
	int usbFd;

	union transferData *wrData[4];
	union transferData *rdData[4];

	uint8_t devDesc[512];
	char isSuperSpeed;
	uint8_t usbInEP;
	uint8_t usbOutEP;

	// host -> device
	int outUrbSubmitIdx;
	int outUrbReapIdx;
	int outUrbFetchIdx;
	struct usbdevfs_urb outUrbs[MAX_URBS];

	// device -> host
	int inUrbSubmitIdx;
	int inUrbReapIdx;
	int inUrbFetchIdx;
	struct usbdevfs_urb inUrbs[MAX_URBS];
} g = {
	.usbFd = -1
};

static int claimInterface(void)
{
	int intf;
	int err;

	if (g.usbFd < 0)
		return -1;


	intf = 0;
	err = ioctl(g.usbFd, USBDEVFS_CLAIMINTERFACE, &intf);
	if (err)
		return -2;
	return 0;
}

static int releaseInterface(void)
{
	int intf;

	if (g.usbFd < 0)
		return -1;

	intf = 0;
	ioctl(g.usbFd, USBDEVFS_RELEASEINTERFACE, &intf);

	return 0;
}

static int pollUrbs(int pollOutEP, int timeout_ms)
{
	int state,err;
	struct pollfd pollArray[1];
	struct usbdevfs_urb *urb;

	if (pollOutEP) {
		if (g.outUrbFetchIdx == g.outUrbSubmitIdx)
			return -1;
		if (g.outUrbFetchIdx != g.outUrbReapIdx)
			return 0;
	}
	else {
		if (g.inUrbFetchIdx == g.inUrbSubmitIdx)
			return -1;
		if (g.inUrbFetchIdx != g.inUrbReapIdx)
			return 0;
	}
	for(;;) {
		pollArray[0].fd = g.usbFd;
		pollArray[0].events = POLLOUT|POLLERR;
		pollArray[0].revents = 0;
		state = poll(pollArray, 1, timeout_ms);
		if (state == 0)
			return -4;
		if (state < 0) {
			if (errno == EINTR)
				continue;
			return -5;
		}

		err = ioctl(g.usbFd, USBDEVFS_REAPURB, &urb);
		if (err < 0) {
			return -2;
		}
		if (urb->endpoint == g.usbOutEP) {
			struct usbdevfs_urb *outUrb;

			outUrb = &g.outUrbs[g.outUrbReapIdx];
			if (urb != outUrb) {
				return -3;
			}
			g.outUrbReapIdx = (g.outUrbReapIdx+1)&(MAX_URBS-1);
		}
		else if (urb->endpoint == g.usbInEP) {
			struct usbdevfs_urb *inUrb;

			inUrb = &g.inUrbs[g.inUrbReapIdx];
			if (urb != inUrb) {
				return -3;
			}
			g.inUrbReapIdx = (g.inUrbReapIdx+1)&(MAX_URBS-1);
		}
		else
			return -8;

		if (pollOutEP) {
			if (g.outUrbFetchIdx != g.outUrbReapIdx)
				return 0;
		}
		else if (g.inUrbFetchIdx != g.inUrbReapIdx)
			return 0;
	}
}

static int startWrite(uint8_t *data, int dataLen)
{
	int err;
	struct usbdevfs_urb *outUrb;

retry:
	outUrb = &g.outUrbs[g.outUrbSubmitIdx];
	memset(outUrb, 0, sizeof(*outUrb));
	outUrb->type     = USBDEVFS_URB_TYPE_BULK;
	outUrb->endpoint = g.usbOutEP;
	outUrb->buffer   = data;
	outUrb->buffer_length = dataLen;

	err = ioctl(g.usbFd, USBDEVFS_SUBMITURB, outUrb);
	if (err < 0 ) {
		if (errno == EINTR)
			goto retry;
		return -1;
	}
	g.outUrbSubmitIdx = (g.outUrbSubmitIdx+1)&(MAX_URBS-1);
	return 0;
}

static int finishWrite(uint8_t **data, int *dataLen, int timeout_ms)
{
	struct usbdevfs_urb *urb;
	int err;

	err = pollUrbs(1, timeout_ms);
	if (err < 0)
		return err;

	urb = &g.outUrbs[g.outUrbFetchIdx];
	g.outUrbFetchIdx = (g.outUrbFetchIdx+1)&(MAX_URBS-1);
	*data    = urb->buffer;
	*dataLen = urb->actual_length;
	return 0;
}

static int startRead(uint8_t *data, int dataLen)
{
	int err;
	struct usbdevfs_urb *inUrb;

retry:
	inUrb = &g.inUrbs[g.inUrbSubmitIdx];
	memset(inUrb, 0, sizeof(*inUrb));
	inUrb->type     = USBDEVFS_URB_TYPE_BULK;
	inUrb->endpoint = g.usbInEP;
	inUrb->buffer   = data;
	inUrb->buffer_length = dataLen;

	err = ioctl(g.usbFd, USBDEVFS_SUBMITURB, inUrb);
	if (err < 0 ) {
		if (errno == EINTR)
			goto retry;
		return -1;
	}
	g.inUrbSubmitIdx = (g.inUrbSubmitIdx+1)&(MAX_URBS-1);
	return 0;
}

static int finishRead(uint8_t **data, int *dataLen, int timeout_ms)
{
	struct usbdevfs_urb *urb;
	int err;

	err = pollUrbs(0, timeout_ms);
	if (err < 0)
		return err;

	urb = &g.inUrbs[g.inUrbFetchIdx];
	g.inUrbFetchIdx = (g.inUrbFetchIdx+1)&(MAX_URBS-1);
	*data    = urb->buffer;
	*dataLen = urb->actual_length;
	if (urb->status)
		DBG_PRINTF("status = %d\n", urb->status);
	return 0;
}

static void scanEndpointDescriptors(uint8_t *desc, int descLen)
{
	int pos;
	pos = 0;
	for(;;) {
		if (pos >= descLen)
			break;
		if (pos + desc[pos+0] > descLen)
			break;
		if (
			desc[pos+0] >= 4 &&
			desc[pos+1] == USB_DT_ENDPOINT &&
			(desc[pos+3] & 0x3) == USB_ENDPOINT_XFER_BULK
		) {
			int epAddr;
			epAddr = desc[pos+2];
			// found BULK endpoint descriptor
			if (epAddr & USB_ENDPOINT_DIR_MASK) {
				DBG_PRINTF(
					"Found bulk IN  endpoint 0x%02x\n",
					epAddr&0x8F
				);
				g.usbInEP = (uint8_t)(epAddr&0x8F);
			}
			else {
				DBG_PRINTF(
					"Found bulk OUT endpoint 0x%02x\n",
					epAddr&0x8F
				);
				g.usbOutEP = (uint8_t)(epAddr&0x8F);
			}
		}
		pos += desc[pos+0];
	}
}

static int checkUsbDevice(void)
{
	int descLen;

	descLen = (int)read(g.usbFd, g.devDesc, 256);
	if (descLen < 0x1a)
		return -1;

	g.isSuperSpeed = 0;
	if (g.devDesc[7] == 0x09) {
		/* HACK : */
		/*
		 * devDesc[7] is the bMaxPacketSize0 field
		 * of the device descriptor.
		 * For SuperSpeed this MUST be 0x09,
		 *  for all other speeds 0x09 is illegal.
		 * We use this field of the device descriptor
		 * to detect a device connected via SuperSpeed.
		 */
		g.isSuperSpeed = 1;
		DBG_PRINTF("Device is using SuperSpeed\n");
	}
	scanEndpointDescriptors(g.devDesc, descLen);

	return 0;
}


enum {
	USB_REQ_ECHO = 1,
	USB_REQ_GENERATE = 2,
};
enum {
	USB_RESP_ECHO    = 1,
	USB_RESP_GENERATE = 2,
};

static int gTestErrNr;

static void test_print_result(
	const char *testName,
	struct timeval *start, struct timeval *stop,
	uint32_t transferNr, uint32_t transferSz

)
{
	uint64_t start_usecs, stop_usecs;
	uint32_t usecs;

	start_usecs = start->tv_sec * 1000000 + start->tv_usec;
	stop_usecs  = stop->tv_sec  * 1000000 + stop->tv_usec;
	usecs = (uint32_t)(stop_usecs - start_usecs);
	printf("Total time in usecs        : %d\n", usecs);

	if (gTestErrNr == 0) {
		uint64_t bps;

		bps  = transferNr;
		bps *= transferSz;
		printf(
			"%s test successful, transferred %d bytes\n",
			testName, (uint32_t)bps
		);
		bps *= 1000000;
		bps /= usecs;
		printf("bytes per second: %d\n", (uint32_t)bps);
	}
	else {
		printf("%s test %d errors!\n", testName, gTestErrNr);
	}

}

static int test_check_finish(
	const char *opName,
	uint8_t *transferBuf, uint32_t transferSz,
	uint8_t *finishedBuf, uint32_t finishedSz
)
{
	int err;

	err = 0;
	if (finishedBuf != transferBuf) {
		DBG_PRINTF("%s: Wrong pointer.\n", opName);
		err--;
	}
	if (finishedSz != transferSz) {
		DBG_PRINTF("%s: Wrong size %d.\n", opName, finishedSz);
		err--;
	}
	return err;
}

static int test_echo_send_buf(
	int idx, uint32_t transferSz,
	uint64_t value, uint64_t add
)
{
	uint64_t *buf, *bufEnd;

	buf = g.wrData[idx]->data64;
	buf--;
	bufEnd = buf + transferSz/sizeof(uint64_t);

	value &= ~0xFFull;
	value |= USB_REQ_ECHO;
	while (buf < bufEnd) {
		*++buf = value;
		value += add;
	}

	return startWrite(g.wrData[idx]->data8, transferSz);
}


static void test_echo_check_buf(int idx, uint32_t transferSz)
{
	int rc;

	rc = memcmp(g.wrData[idx]->data8, g.rdData[idx]->data8, transferSz);
	if (rc) {
		if (gTestErrNr<10)
			DBG_PRINTF("found error\n");
		gTestErrNr++;
	}
}


#define ECHO_TEST_TRANSFERS (256*16)
static void test_echo_run(void)
{
	struct timeval start;
	struct timeval stop;

	int rc;
	uint32_t transferSz;
	uint64_t wrValue, wrAdd;

	int wrPacketNr;
	int wrNextIdx;
	int wrPendingIdx;
	int wrPendingCnt;
	uint8_t *wrData;
	int      wrDataSz;

	int rdPacketNr;
	int rdNextIdx;
	int rdPendingIdx;
	int rdPendingCnt;
	int rdFinishedNr;
	uint8_t *rdData;
	int      rdDataSz;

	transferSz = 0x4000-0x10;
	if (g.isSuperSpeed)
		transferSz = 0x20000-0x10;

	printf("Running ECHO test\n");
	printf(
		"Transfering %d x %d bytes\n",
		ECHO_TEST_TRANSFERS, transferSz
	);

	gTestErrNr = 0;

	wrValue   = 0x0100020003000400ull;
	wrAdd     = 0x0001000100010001ull;

	rdFinishedNr = 0;

	wrPacketNr   = 0;
	wrNextIdx    = 0;
	wrPendingIdx = 0;
	wrPendingCnt = 0;

	rdPacketNr   = 0;
	rdNextIdx    = 0;
	rdPendingIdx = 0;
	rdPendingCnt = 0;

	gettimeofday(&start, NULL);
	while (rdFinishedNr < ECHO_TEST_TRANSFERS) {
		if (wrPendingCnt > 0) {
			rc = finishWrite(&wrData, &wrDataSz, 1000);
			if (rc) {
				DBG_PRINTF("finishWrite: error %d\n", rc);
				return;
			}
			rc = test_check_finish(
				"finishWrite",
				g.wrData[wrPendingIdx]->data8, transferSz,
				wrData, wrDataSz
			);
			if (rc)
				return;
			wrPendingIdx=(wrPendingIdx+1)&0x3;
			wrPendingCnt--;
		}
		if (rdPendingCnt > 0) {
			rc = finishRead(&rdData, &rdDataSz, 1000);
			if (rc) {
				DBG_PRINTF("finishRead: error %d\n", rc);
				return;
			}
			rc = test_check_finish(
				"finishRead",
				g.rdData[rdPendingIdx]->data8, transferSz,
				rdData, rdDataSz
			);
			if (rc)
				return;
			test_echo_check_buf(rdPendingIdx, transferSz);
			rdPendingIdx=(rdPendingIdx+1)&0x3;
			rdPendingCnt--;
			rdFinishedNr++;
		}
		while (
			wrPacketNr < ECHO_TEST_TRANSFERS &&
			wrPendingCnt < 3
		) {
			rc = test_echo_send_buf(
				wrNextIdx, transferSz,
				wrValue, wrAdd
			);
			if (rc) {
				DBG_PRINTF(
					"Error while sending data: %d\n",
					rc
				);
				return;
			}
			wrValue += 0x0100010001000100ull;
			wrNextIdx=(wrNextIdx+1)&0x3;
			wrPendingCnt++;
			wrPacketNr++;
		}
		while (
			rdPacketNr < ECHO_TEST_TRANSFERS &&
			rdPendingCnt < 3
		) {
			rc = startRead(
				g.rdData[rdNextIdx]->data8,
				transferSz+0x10
			);
			if (rc) {
				DBG_PRINTF("startRead: error %d\n", rc);
				return;
			}
			rdNextIdx=(rdNextIdx+1)&0x3;
			rdPendingCnt++;
			rdPacketNr++;
		}
	}
	gettimeofday(&stop, NULL);
	test_print_result("Echo", &start, &stop, rdFinishedNr, transferSz);
}

static int test_generate_send_req(
	uint32_t cnt, uint32_t transferSz,
	uint64_t value, uint64_t add
)
{
	uint64_t *buf, *finishBuf;
	int rc, finishSz;

	buf = g.wrData[0]->data64;
	buf[0] =
		(uint64_t)USB_REQ_GENERATE |
		(uint64_t)transferSz << 8  |
		(uint64_t)cnt << 32;
	buf[1] = value;
	buf[2] = add;
	rc = startWrite((uint8_t *)buf, 24);
	if (rc)
		return rc;
	rc = finishWrite((uint8_t **)(&finishBuf), &finishSz, 1000);
	if (rc)
		return rc;
	if (finishBuf != buf || finishSz != 24) {
		DBG_PRINTF("Inconsistencies while sending request ?!\n");
		return -1;
	}
	return 0;
}

static void test_generate_check_buf(
	int idx, uint32_t transferSz,
	uint64_t value, uint64_t add
)
{
	uint64_t *buf, *bufEnd, cmp;

	buf = g.rdData[idx]->data64;
	buf--;
	bufEnd = buf + transferSz/sizeof(uint64_t);

	value &= ~0xFFull;
	value |= USB_RESP_GENERATE;
	while (buf < bufEnd) {
		cmp = *++buf;
		if (cmp != value) {
			if (gTestErrNr<10)
				DBG_PRINTF("found error\n");
			gTestErrNr++;
			return;
		}
		value += add;
	}
}

#define TEST_GENERATE_TRANSFER_NR (256*16)
static void test_generate_run(void)
{
	struct timeval start;
	struct timeval stop;

	uint64_t wrValue, wrAdd;
	uint32_t total, started, pending, finished, transferSz;
	int rdNextIdx;
	int rdPendingIdx;
	uint8_t *rdData;
	int      rdDataSz;
	int rc;

	transferSz = 0x4000-0x10;
	if (g.isSuperSpeed)
		transferSz = 0x40000-0x10;

	printf("Running Device->Host test\n");
	printf(
		"Transfering %d x %d bytes\n",
		TEST_GENERATE_TRANSFER_NR, transferSz
	);

	gTestErrNr = 0;

	wrValue    = 0x0100020003000400ull;
	wrAdd      = 0x0001000100010001ull;
	total      = TEST_GENERATE_TRANSFER_NR;


	started    = 0;
	pending    = 0;
	finished   = 0;

	rdNextIdx    = 0;
	rdPendingIdx = 0;

	test_generate_send_req(total, transferSz, wrValue, wrAdd);

	gettimeofday(&start, NULL);
	while (finished < total) {
		while (started < total && pending < 3) {
			rc = startRead(
				g.rdData[rdNextIdx]->data8,
				transferSz+0x10
			);
			if (rc) {
				DBG_PRINTF("startRead: error %d\n", rc);
				return;
			}
			rdNextIdx=(rdNextIdx+1)&0x3;
			started++;
			pending++;
		}
		rc = finishRead(&rdData, &rdDataSz, 1000);
		if (rc) {
			DBG_PRINTF(
				"finishRead: error %d, transfer %d\n",
				rc, finished
			);
			return;
		}
		rc = test_check_finish(
			"finishRead",
			g.rdData[rdPendingIdx]->data8, transferSz,
			rdData, rdDataSz
		);
		if (rc)
			return;
		test_generate_check_buf(
			rdPendingIdx, rdDataSz, wrValue, wrAdd
		);
		wrValue += 0x0100010001000100ull;
		rdPendingIdx=(rdPendingIdx+1)&0x3;
		pending--;
		finished++;
	}
	gettimeofday(&stop, NULL);
	test_print_result("Gen", &start, &stop, total, transferSz);
}

int main(int argc, char **argv)
{
	uint8_t *wrData;
	int wrDataSz;
	int rc;
	int ret;
	int i, cmd;

	if (argc != 3) {
		printf("Usage: usb_pc_tst <usb device node> <echo|gen>\n");
		return 1;
	}
	cmd = 0;
	if (strcmp(argv[2], "echo") == 0)
		cmd = 1;
	else if (strcmp(argv[2], "gen") == 0)
		cmd = 2;

	if (!cmd) {
		printf("Usage: usb_pc_tst <usb device node> <echo|gen>\n");
		return 1;
	}

	g.usbFd = open(argv[1], O_RDWR);
	if (g.usbFd < 0) {
		printf("Cannot open %s\n", argv[1]);
		return 1;
	}
	rc = checkUsbDevice();
	if (rc)
		return 1;
	if (g.usbInEP == 0 || g.usbOutEP == 0) {
		printf("Cannot find OUT/IN Endpoints.\n");
		return 1;
	}

	// allocate transfer buffers
	for (i=0; i<4; i++) {
		rc = posix_memalign(
			(void **)(&g.wrData[i]),
			0x10000, ALLOC_SZ
		);
		if (rc) {
			DBG_PRINTF(
				"Cannot allocate g.wrData[%d]: "
				"err = %d\n",
				i, rc
			);
			return 1;
		}
		rc = posix_memalign(
			(void **)(&g.rdData[i]),
			0x10000, ALLOC_SZ
		);
		if (rc) {
			DBG_PRINTF(
				"Cannot allocate g.rdData[%d]: "
				"err = %d\n",
				i, rc
			);
			return 1;
		}
	}

	rc = claimInterface();
	if (rc) {
		printf("Cannot claim USB interface\n");
		return 1;
	}

	ret = 0;
	wrData = g.wrData[0]->data8;
	wrData[0] = 0x11;
	wrData[1] = 0x22;
	wrData[2] = 0x33;
	wrData[3] = 0x44;
	rc = startWrite(wrData, 4);
	if (rc) {
		printf("Cannot start write of 4 bytes to OUT EP\n");
		ret = 1;
		goto main_exit;
	}
	printf("Started write of 4 bytes to OUT EP\n");
	rc = finishWrite(&wrData, &wrDataSz, 1000);
	if (rc) {
		printf("Cannot finish write of 4 bytes to OUT EP\n");
		ret = 1;
		goto main_exit;
	}
	printf("Finished write of 4 bytes to OUT EP\n");

	switch(cmd) {
	case 1:
		test_echo_run();
		break;
	case 2:
		test_generate_run();
		break;
	default:
		;
	}

main_exit:
	releaseInterface();
	return ret;
}
