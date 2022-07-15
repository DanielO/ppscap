/******************************************************************
*******************************************************************
**
**
** Copyright 2022 Daniel O'Connor <darius@dons.net.au>
**
** Redistribution and use in source and binary forms, with or
** without modification, are permitted provided that the following
** conditions are met:
**
** 1. Redistributions of source code must retain the above copyright
** notice, this list of conditions and the following disclaimer.
**
** 2. Redistributions in binary form must reproduce the above
** copyright notice, this list of conditions and the following
** disclaimer in the documentation and/or other materials provided
** with the distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
** CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
** INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
** BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
** OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
** PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
** PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
** OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
** USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
**
*******************************************************************
******************************************************************/

/*
** PPSCAP.C
**
** Capture GPS time (from GGA or RMC messages) and PPS edges from a
** serial port to feed ntpd.
** Based on /usr/src/tools/test/ppsapi/ppsapitest.c
**
** The PPS API is described at:
** https://datatracker.ietf.org/doc/html/rfc2783
**
** Note that it is recommended to use RMC as GGA only has time of day
** so the PC clock is used to get the day.
*/

#include <sys/ipc.h>
#include <sys/queue.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/timepps.h>
#include <sys/un.h>
#include <ctype.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>

static int aflag, cflag, vflag;
static struct shmTime *shmntp = NULL;

/* NTP SHM structure
 * https://www.eecis.udel.edu/~mills/ntp/html/drivers/driver28.html
 */
struct shmTime {
    int    mode;
    int    count;
    time_t clockTimeStampSec;
    int    clockTimeStampUSec;
    time_t receiveTimeStampSec;
    int    receiveTimeStampUSec;
    int    leap;
    int    precision;
    int    nsamples;
    int    valid;
    int    dummy[10];
};

/* Process a PPS edge and GPS time to feed to NTP (if enabled) */
static void
processtime(struct timespec* tsa, struct timespec* tsc, unsigned sa, unsigned sc, time_t gpstime) {
    int posted = 0;

    if (shmntp != NULL && gpstime > 0 && shmntp->valid == 0) {
	/* +1 for up coming edge */
	shmntp->clockTimeStampSec = gpstime + 1;
	shmntp->clockTimeStampUSec = 0;
	if (aflag) {
	    /* Report assert edge */
	    shmntp->receiveTimeStampSec = tsa->tv_sec;
	    shmntp->receiveTimeStampUSec = tsa->tv_nsec / 1000;
	} else {
	    /* Report clear edge */
	    shmntp->receiveTimeStampSec = tsc->tv_sec;
	    shmntp->receiveTimeStampUSec = tsc->tv_nsec / 1000;
	}
	shmntp->valid = 1;
	posted = 1;
    }

    if (vflag) {
	printf("%jd.%09ld %u %jd.%09ld %u %jd%s\n", (intmax_t)tsa->tv_sec, tsa->tv_nsec, sa,
	  (intmax_t)tsc->tv_sec, tsc->tv_nsec, sc, (intmax_t)gpstime, posted ? " posted" : "");

	fflush(stdout);
    }
}

/* Convert talker ID into name */
static const char *
gnsid2str(char gnsid) {
    switch (gnsid) {
    case 'A':
	return "Galileo";

    case 'B':
	return "BeiDou";

    case 'L':
	return "GLONASS";

    case 'N':
	return "Combined";

    case 'P':
	return "GPS";

    default:
	return "Unknown";
    }
}

/*
 * Parse GPS engine output looking for GGA message
 *
 * Returns the GPS time found, or -1 if it can't be determined
 */
static time_t
parsebuf(char *buf) {
    int i, res;
    char gnsid, msgbuf[200], msgtype[4];
    uint8_t wantsum, calcsum;
    struct tm tm;
    struct timeval tv;
    time_t utcepoch;
    static char msg[200];
    static int state = 0;
    static int msgpos = 0;

#if 0
    for (i = 0; buf[i] != '\0'; i++) {
	if (buf[i] == '\r')
	    continue;
	else if (buf[i] == '\n' || isprint(buf[i]))
	    fputc(buf[i], stdout);
	else
	    printf("\\x%02x", buf[i]);
    }
    fflush(stdout);
#endif
    switch (state) {
    case 0:
	for (; *buf != '\0' && *buf != '$'; buf++)
	    continue;

	if (*buf == 0)
	    return -1;

	state = 1;
	msgpos = 0;
	/* FALLTHROUGH */

    case 1:
	/* Copy message to internal buffer, stop 3 bytes before end (checksum + nil) */
	for (; *buf != '\0' && *buf != '*' && msgpos < sizeof(msg) - 3; buf++, msgpos++)
	    msg[msgpos] = *buf;
	if (msgpos >= sizeof(msg) - 3) {
	    /* Overflow */
	    state = 0;
	    return -1;
	}
	/* End of input buffer */
	if (*buf == '\0')
	    return -1;
	/* End of message */
	if (*buf == '*') {
	    msg[msgpos++] = *buf;
	    buf++;
	    state = 2;
	}
	/* FALLTHROUGH */

    case 2:
	if (*buf == '\0')
	    return -1;
	msg[msgpos++] = *buf;
	buf++;
	state = 3;
	/* FALLTHROUGH */

    case 3:
	if (*buf == '\0')
	    return -1;
	msg[msgpos++] = *buf;
	msg[msgpos++] = '\0';
	state = 0;
	break;
    }

    /* Validate checksum (XOR of bytes between $ and *) */
    calcsum = 0;
    for (i = 1; msg[i] != '\0' && msg[i] != '*'; i++) {
	calcsum = calcsum ^ msg[i];
    }
    wantsum = strtol(msg + i + 1, NULL, 16);
    if (calcsum != wantsum) {
	if (vflag > 3)
	    printf("checksum mismatch 0x%0x != 0x%02x\n", calcsum, wantsum);
	return -1;
    } else
	if (vflag > 3)
	    printf("Message: %s\n", msg);

    if ((res = sscanf(msg, "$G%c%3[^,],%[^*]*%*x", &gnsid, msgtype, msgbuf)) != 3) {
	if (vflag > 2)
	    printf("Unable to parse NMEA sentance\n");
	return -1;
    }

    if (!strcmp(msgtype, "GGA")) {
	int utchr, utcmin, utcsec, utchsec, qual, nsat, age, diffid;
	char latdir, londir, altunits;
	float lat, lon, hdop, alt, undulation;

	/* $GNGGA,040751.00,3454.4907,S,13835.7617,E,2,12,0.8,51.2,M,-3.5,M,,*4A */
	if (sscanf(msgbuf, "%2d%2d%2d.%d,%f,%c,%f,%c,%d,%d,%f,%f,%c,%f,%d,%d", &utchr,
	    &utcmin, &utcsec, &utchsec, &lat, &latdir, &lon, &londir, &qual, &nsat, &hdop,
	    &alt, &altunits, &undulation, &age, &diffid) < 14) {
	    if (vflag > 2)
		printf("unable to parse GGA\n");
	    return -1;
	}

	/* Only use if locked */
	if (qual != 1 && qual != 2) {
	    if (vflag > 2)
		printf("Poor quality\n");
	    return -1;
	}

	/* GGA messages only have time of day, use PC date */
	gettimeofday(&tv, NULL);
	gmtime_r(&tv.tv_sec, &tm);
	tm.tm_hour = utchr;
	tm.tm_min = utcmin;
	tm.tm_sec = utcsec;
	utcepoch = timegm(&tm);

	if (vflag > 1)
	    printf("GGA: GNS %s Quality: %d UTC %02d:%02d:%02d.%02d -> %jd\n", gnsid2str(gnsid),
	      qual, utchr, utcmin, utcsec, utchsec, (intmax_t)utcepoch);
	return utcepoch;
    } else if (!strcmp(msgtype, "RMC")) {
	int utchr, utcmin, utcsec, utchsec, utcday, utcmon, utcyr;
	char status;

	/* $GNRMC,073418.00,A,3500.22805,S,13833.89082,E,0.007,,290422,,,A*78 */
	if ((res = sscanf(msgbuf, "%2d%2d%2d.%d,%c,%*f,%*c,%*f,%*c,%*[^,],,%2d%2d%2d,%*s",
	      &utchr, &utcmin, &utcsec, &utchsec, &status, &utcday, &utcmon, &utcyr)) != 8) {
	    if (vflag > 2)
		printf("Unable to parse RMC (%d) - %s\n", res, msgbuf);
	    return -1;
	}

	if (status != 'A') {
	    if (vflag > 2)
		printf("Not locked\n");
	    return -1;
	}

	tm.tm_year = utcyr + 100;
	tm.tm_mon = utcmon - 1;
	tm.tm_mday = utcday;
	tm.tm_hour = utchr;
	tm.tm_min = utcmin;
	tm.tm_sec = utcsec;
	utcepoch = timegm(&tm);

	if (vflag > 1)
	    printf("RMC: GNS %s UTC %04d/%02d/%02d %02d:%02d:%02d.%02d -> %jd\n",
	      gnsid2str(gnsid), utcyr + 2000, utcmon, utcday, utchr, utcmin,
	      utcsec, utchsec, (intmax_t)utcepoch);
	return utcepoch;
    } else {
	if (vflag > 2)
	    printf("Unknown message type %s\n", msgtype);
	return -1;
    }
}

void
setfdnonblock(int fd) {
    int i;

    if (fcntl(fd, F_GETFL, &i) == -1)
	err(1, "fcntl F_GETFL");
    i |= O_NONBLOCK;
    if (fcntl(fd, F_SETFL, i) == -1)
	err(1, "fcntl F_SETFL");
}

int
main(int argc, char** argv) {
    int ppsfd, datafd, servfd, i, mode, baud, amt, shmid, shmunit,
	dtr_pos, dtr_neg, force_dtr;
    pps_info_t pi;
    pps_params_t pp;
    pps_handle_t ph;
    u_int olda, oldc;
    struct timespec to;
    struct termios t;
    struct timeval ts, writedone, tmpts;
    fd_set rdfds;
    char buf[1024], *socketpath = NULL;
    time_t gpstime;
    struct sockaddr_un serv_addr;
    socklen_t socklen;
    struct client_info_t {
	int fd;
	uid_t euid;
	gid_t egid;
	struct sockaddr_un addr;
	STAILQ_ENTRY(client_info_t) link;
    };
    struct client_info_t *cl, *tmpcl;

    STAILQ_HEAD(, client_info_t) client_list = STAILQ_HEAD_INITIALIZER(client_list);

    aflag = cflag = vflag = 0;
    baud = 115200;
    shmunit = -1;
    dtr_pos = TIOCCDTR;
    dtr_neg = TIOCSDTR;
    ppsfd = -1;
    force_dtr = 0;

    while ((i = getopt(argc, argv, "aAcdDl:p:r:s:v")) != -1) {
	switch (i) {
	case 'a':
	    aflag = 1;
	    break;
	case 'c':
	    cflag = 1;
	    break;
	case'd':
	    dtr_pos = TIOCSDTR;
	    dtr_neg = TIOCCDTR;
	    break;
	case 'D':
	    force_dtr = 1;
	    break;
	case 'l':
	    socketpath = optarg;
	    break;
	case 'p':
	    if ((ppsfd = open(optarg, O_RDONLY)) == -1)
		err(1, "open(%s)", optarg);
	    break;
	case 'r':
	    if ((baud = atoi(optarg)) == 0)
		errx(1, "invalid baud rate");
	    break;
	case 's':
	    shmunit = atoi(optarg);
	    break;
	case 'v':
	    vflag++;
	    break;
	case '?':
	default:
	    fprintf(stderr, "\
Usage: %s [-acdDv] [-l socketpath] [-r baud] [-s shmunit] [-p ppsdev] device\n\
\n\
	-a		Capture PPS on edge assert\n\
	-c		Capture PPS on edge clear\n\
	-d		Invert DTR control (normally cleared, raised to write to serial port)\n\
	-D		Force DTR to always be set for writing (raised or cleared as per -d)\n\
	-v		Increase verbosity (can be specified multiple times)\n\
	socketpath	Path to unix domain socket to listen on\n\
	baud		Baud rate to talk to device (default: 115200)\n\
	shmunit		NTP shared memory unit to connect to (default: don't connect)\n\
	ppsdev		Device to get PPS timestamps from (default: same as serial device)\n\
	device		Serial device to talk to\n\
\n\
Example:\n\
	%s -cs 2 -l /tmp/ppscap.sock /dev/cuaU0\n\
		Talk to /dev/cuaU0 at 115200 capturing PPS edge clear and feeding to\n\
		NTP unit 2. Listen to socket to allow reading and writing traffic.\n", argv[0], argv[0]);
	    exit(1);
	}
    }
    argc -= optind;
    argv += optind;
    if (argc < 1) {
	fprintf(stderr, "Must specify device\n");
	exit(1);
    }
    if ((datafd = open(argv[0], O_RDWR)) == -1)
	err(1, "open(%s)", argv[0]);
    if (tcflush(datafd, TCIOFLUSH) == -1)
	err(1, "tcflush");
    setfdnonblock(datafd);

    if (ppsfd == -1)
	ppsfd = datafd;

    if (socketpath) {
	signal(SIGPIPE, SIG_IGN);
	if ((servfd = socket(PF_UNIX, SOCK_STREAM, 0)) == -1)
	    err(1, "socket");

	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sun_family = AF_UNIX;
	strcpy(serv_addr.sun_path, socketpath);
	if (unlink(socketpath) && errno != ENOENT)
	    err(1, "unlink");
	if (bind(servfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
	    err(1, "bind");

	listen(servfd, 5);
    }

    /* Setup serial port */
    if (tcgetattr(datafd, &t))
	err(1, "tcgetattr");

    /* Make raw and setup for 8N1, no flow control */
    cfmakeraw(&t);
    t.c_cflag &= ~CSTOPB;
    t.c_cflag &= ~CRTSCTS;
    t.c_cflag &= ~CDSR_OFLOW;
    t.c_cflag &= ~CDTR_IFLOW;
    t.c_cflag &= ~CCAR_OFLOW;
    t.c_cflag &= ~HUPCL;
    t.c_cflag |= CLOCAL;
    cfsetspeed(&t, baud);

    if (tcsetattr(datafd, TCSANOW, &t))
	err(1, "tcsetattr");

    /* Set DTR as the user requested */
    if (force_dtr) {
	if (ioctl(datafd, dtr_neg, 0) == -1)
	    err(1, "ioctl TIO[CS]DTR");
    } else {
	if (ioctl(datafd, dtr_pos, 0) == -1)
	    err(1, "ioctl TIO[CS]DTR");
    }

    /* Attach to NTP SHM segment */
    if (shmunit >= 0) {
	if ((shmid = shmget(0x4e545030 + shmunit, sizeof(struct shmTime), 0)) == -1)
	    err(1, "shmget of NTP unit %d failed", shmunit);
	else
	    if ((intptr_t)(shmntp = (struct shmTime *)shmat(shmid, 0, 0)) == -1)
		err(1, "shmat of NTP unit %d failed", shmunit);
    }
    /* Setup PPS API */
    if (time_pps_create(ppsfd, &ph) == -1)
	err(1, "time_pps_create");

    if (time_pps_getcap(ph, &mode) == -1)
	err(1, "time_pps_getcap");

    if (vflag) {
	fprintf(stderr, "Supported modebits:");
	if (mode & PPS_CAPTUREASSERT)
	    fprintf(stderr, " CAPTUREASSERT");
	if (mode & PPS_CAPTURECLEAR)
	    fprintf(stderr, " CAPTURECLEAR");
	if (mode & PPS_OFFSETASSERT)
	    fprintf(stderr, " OFFSETASSERT");
	if (mode & PPS_OFFSETCLEAR)
	    fprintf(stderr, " OFFSETCLEAR");
	if (mode & PPS_ECHOASSERT)
	    fprintf(stderr, " ECHOASSERT");
	if (mode & PPS_ECHOCLEAR)
	    fprintf(stderr, " ECHOCLEAR");
	if (mode & PPS_CANWAIT)
	    fprintf(stderr, " CANWAIT");
	if (mode & PPS_CANPOLL)
	    fprintf(stderr, " CANPOLL");
	if (mode & PPS_TSFMT_TSPEC)
	    fprintf(stderr, " TSPEC");
	if (mode & PPS_TSFMT_NTPFP)
	    fprintf(stderr, " NTPFP");
	fprintf(stderr, "\n");
    }

    if ((!aflag && !cflag) || (aflag && cflag))
	errx(1, "Must set one and only one of -a or -c");
    if (vflag)
	printf("Capturing edge %s\n", aflag ? "assertion" : "clear");

    if (time_pps_getparams(ph, &pp) == -1)
	err(1, "time_pps_getparams():");

    if (aflag)
	pp.mode |= PPS_CAPTUREASSERT;
    if (cflag)
	pp.mode |= PPS_CAPTURECLEAR;

    if (!(pp.mode & PPS_TSFMT_TSPEC))
	pp.mode |= PPS_TSFMT_TSPEC;

    if (time_pps_setparams(ph, &pp) == -1)
	err(1, "time_pps_setparams(mode %x):", pp.mode);

    /*
     * Pick up first event outside the loop in order to not
     * get something ancient into the outfile.
     */
    to.tv_nsec = 0;
    to.tv_sec = 0;
    if (time_pps_fetch(ph, PPS_TSFMT_TSPEC, &pi, &to) == -1)
	err(1, "time_pps_fetch()");
    olda = pi.assert_sequence;
    oldc = pi.clear_sequence;

    gpstime = -1;
    writedone.tv_sec = 0;
    writedone.tv_usec = 0;
    while (1) {
	to.tv_nsec = 0;
	to.tv_sec = 0;
	if (time_pps_fetch(ph, PPS_TSFMT_TSPEC, &pi, &to) == -1)
	    err(1, "time_pps_fetch()");
	if (oldc != pi.clear_sequence && cflag)
	    ; /* Got an edge clear */
	else if (olda != pi.assert_sequence && aflag)
	    ; /* Got an edge assertion */
	else {
	    /* Setup FDs to wake on */
	    FD_ZERO(&rdfds);
	    FD_SET(datafd, &rdfds);
	    i = datafd;

	    /* Wake up for new connections */
	    if (socketpath) {
		FD_SET(servfd, &rdfds);
		if (servfd > i)
		    i = servfd;
	    }

	    /* Wake up for writes from clients */
	    if (socketpath)
		STAILQ_FOREACH(cl, &client_list, link) {
		    FD_SET(cl->fd, &rdfds);
		    if (cl->fd > i)
			i = cl->fd;
		}
	    ts.tv_usec = 10000;
	    ts.tv_sec = 0;
	    if (select(i + 1, &rdfds, NULL, NULL, &ts) == -1)
		if (errno != EINTR)
		    err(1, "select");

	    /* Got some data, read it and send to clients */
	    if (FD_ISSET(datafd, &rdfds)) {
		if ((amt = read(datafd, buf, sizeof(buf) - 1)) == -1)
		  err(1, "read(tty)");
		buf[amt] = 0;
		if (socketpath) {
		    STAILQ_FOREACH_SAFE(cl, &client_list, link, tmpcl) {
			if (write(cl->fd, buf, amt) == -1) {
			    warn("write to FD %d failed", cl->fd);
			    STAILQ_REMOVE(&client_list, cl, client_info_t, link);
			    free(cl);
			}
		    }
		}

		i = parsebuf(buf);
		if (i != -1)
		    gpstime = i;
	    }

	    /* New connection */
	    if (socketpath && FD_ISSET(servfd, &rdfds)) {
		if ((cl = malloc(sizeof(*cl))) == NULL)
		    err(1, "malloc");
		socklen = sizeof(cl->addr);
		if ((cl->fd = accept(servfd, (struct sockaddr *)&cl->addr, &socklen)) == -1)
		    err(1, "accept");
		setfdnonblock(cl->fd);
		if (getpeereid(cl->fd, &cl->euid, &cl->egid) == -1)
		    err(1, "getpeereid");
		warnx("accepted connection on FD %d from EUID %d EGID %d", cl->fd, cl->euid, cl->egid);
		STAILQ_INSERT_TAIL(&client_list, cl, link);
	    }

	    /* Data from a client, sent to tty */
	    STAILQ_FOREACH_SAFE(cl, &client_list, link, tmpcl) {
		if (!FD_ISSET(cl->fd, &rdfds))
		    continue;
		if ((amt = read(cl->fd, buf, sizeof(buf) - 1)) == -1) {
		    STAILQ_REMOVE(&client_list, cl, client_info_t, link);
		    warn("read from FD %d failed", cl->fd);
		}
		/* Flip DTR to we can talk to the module */
		if (ioctl(datafd, dtr_neg, 0))
		    err(1, "ioctl TIOC[CS]DTR");
		/* Write it */
		if (write(datafd, buf, amt) == -1)
		    err(1, "write");
		/* microseconds/byte */
		gettimeofday(&writedone, NULL);
		tmpts.tv_sec = 0;
		tmpts.tv_usec = amt * 110000 / baud; /* usec/byte + 10% */
		timeradd(&writedone, &tmpts, &writedone);
	    }
	    /* Is the write timer running? */
	    if (writedone.tv_sec != 0 && writedone.tv_usec != 0) {
		/* Check if it's expired */
		gettimeofday(&tmpts, NULL);
		if (timercmp(&tmpts, &writedone, >)) {
		    /* Reset DTR if required */
		    if (force_dtr)
			if (ioctl(datafd, dtr_pos, 0))
			    err(1, "ioctl TIOC[CS]DTR");
		    writedone.tv_sec = 0;
		    writedone.tv_usec = 0;
		}
	    }
	    /* Back to check for an edge */
	    continue;
	}

	processtime(&pi.assert_timestamp, &pi.clear_timestamp,
	  pi.assert_sequence, pi.clear_sequence, gpstime);
	olda = pi.assert_sequence;
	oldc = pi.clear_sequence;
	gpstime = -1;
    }
    return (0);
}
