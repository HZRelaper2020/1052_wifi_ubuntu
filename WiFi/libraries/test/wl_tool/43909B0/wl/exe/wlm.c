/*
 * Wlm (Wireless LAN Manufacturing) test library.
 *
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *
 * $Id: wlm.c 500795 2014-09-05 04:05:34Z jkoster $
 */

#if defined(WIN32)
#include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <bcmcdc.h>      // cdc_ioctl_t used in wlu_remote.h
#if defined(WIN32)
#include <bcmstdlib.h>
#endif
#include <bcmendian.h>
#include <bcmutils.h>    // ARRAYSIZE, bcmerrorstr()
#include <bcmsrom_fmt.h> // SROM4_WORDS
#include <bcmsrom_tbl.h> // pavars_t
#include <wlioctl.h>
#if defined(WIN32)
#include <epictrl.h>     // ADAPTER
#endif
#include <proto/ethernet.h>	 // ETHER_ADDR_LEN

#if defined(LINUX) || defined (MACOSX)
#include <sys/socket.h>
#include <proto/bcmip.h> // ipv4_addr
#include <arpa/inet.h>	// struct sockaddr_in
#include <string.h>
#include <signal.h>
#endif

#include <epivers.h>
#include "wlu_remote.h"  // wl remote type defines (ex: NO_REMOTE)
#include "wlu_pipe.h"    // rwl_open_pipe()
#include "wlu.h"         // wl_ether_atoe()
#include "wlm.h"
#include "wlc_ppr.h"

/* IOCTL swapping mode for Big Endian host with Little Endian dongle.  Default to off */
#ifdef IL_BIGENDIAN
bool swap = FALSE;
#define htod64(i) (swap?bcmswap64(i):i)
#define htod32(i) (swap?bcmswap32(i):i)
#define htod16(i) (swap?bcmswap16(i):i)
#define dtoh64(i) (swap?bcmswap64(i):i)
#define dtoh32(i) (swap?bcmswap32(i):i)
#define dtoh16(i) (swap?bcmswap16(i):i)
#define htodchanspec(i) htod16(i)
#define dtohchanspec(i) dtoh16(i)
#define htodenum(i) ((sizeof(i) == 4) ? htod32(i) : ((sizeof(i) == 2) ? htod16(i) : i))
#define dtohenum(i) ((sizeof(i) == 4) ? dtoh32(i) : ((sizeof(i) == 2) ? htod16(i) : i))
#else /* IL_BIGENDIAN */
#define htod32(i) i
#define htod16(i) i
#define dtoh32(i) i
#define dtoh16(i) i
#define htodchanspec(i) i
#define dtohchanspec(i) i
#define htodenum(i) i
#define dtohenum(i) i
#endif /* IL_BIGENDIAN */

#if defined(WIN32)
static HANDLE irh;
int adapter;
#else
static void * irh;
#define HANDLE void *
#endif

#if !defined(TARGETOS_nucleus)
#define MAX_INTERFACE_NAME_LENGTH     128
static char interfaceName[MAX_INTERFACE_NAME_LENGTH + 1] = {0};
#endif
static WLM_BAND curBand = WLM_BAND_AUTO;
static int curPhyType = PHY_TYPE_NULL;
static int ioctl_version;

#if (defined( RWL_WIFI ) || defined( RWL_LOCAL ) ) && (!defined(WIN32))
extern int wl_os_type_get_rwl(void);
extern void wl_os_type_set_rwl(int os_type);
extern int wl_ir_init_rwl(HANDLE *irh);
#endif /* if (defined( RWL_WIFI ) || defined( RWL_LOCAL ) ) && (!defined(WIN32)) */
#if (defined( RWL_WIFI ) || defined( RWL_LOCAL ) ) && (defined(WIN32))
extern int wl_ir_init_adapter_rwl(HANDLE *irh, int adapter);
#endif /* if (defined( RWL_WIFI ) || defined( RWL_LOCAL ) ) && (defined(WIN32)) */
extern void wl_close_rwl(int remote_type, HANDLE irh);
#ifdef RWL_SOCKET
extern int rwl_init_socket(void);
#endif /*ifdef RWL_SOCKET*/

extern int wlu_get(void *wl, int cmd, void *buf, int len);
extern int wlu_set(void *wl, int cmd, void *buf, int len);

extern int wlu_iovar_get(void *wl, const char *iovar, void *outbuf, int len);
extern int wlu_iovar_set(void *wl, const char *iovar, void *param, int paramlen);
extern int wlu_iovar_getint(void *wl, const char *iovar, int *pval);
extern int wlu_iovar_setint(void *wl, const char *iovar, int val);
extern int wlu_iovar_getbuf(void *wl, const char *iovar, void *param, int paramlen,
    void *bufptr, int buflen);
extern int wlu_iovar_setbuf(void *wl, const char *iovar, void *param, int paramlen,
    void *bufptr, int buflen);

extern int wlu_var_getbuf(void *wl, const char *iovar, void *param, int param_len, void **bufptr);
extern int wlu_var_setbuf(void *wl, const char *iovar, void *param, int param_len);
extern int wlu_var_getbuf_med(void *wl, const char *iovar, void *param, int parmlen, void **bufptr);
extern int wlu_var_getbuf_sm(void *wl, const char *iovar, void *param, int parmlen, void **bufptr);

extern int wl_seq_batch_in_client(bool enable);
extern int wl_seq_start(void *wl, cmd_t *cmd, char **argv);
extern int wl_seq_stop(void *wl, cmd_t *cmd, char **argv);

extern char *ver2str(unsigned int vms, unsigned int vls);
static int wlmPhyTypeGet(void);
extern chanspec_t wl_chspec_to_legacy(chanspec_t chspec);
extern int process_clm_data(void *wl, char *clmfilename, int ds_id);
extern int process_txcap_data(void *wl, char *txcapfilename);
extern int wl_parse_country_spec(const char *spec, char *ccode, int *regrev);

#if defined(WIN32)
static void wlm_dump_bss_info(wl_bss_info_t *bi, char *scan_dump_buf, int *len);
static int wlm_format_ssid(char* buf, uint8* ssid, int ssid_len);
static char* wlm_ether_etoa(const struct ether_addr *n);
static const char* wlm_capmode2str(uint16 capability);
static char* wlm_wf_chspec_ntoa(chanspec_t chspec, char *buf);

static int wlm_buf_to_args(char *line, char *new_argv[]);
static int wlm_process_args(void *wl, char **argv);
static cmd_t *wlm_find_cmd(char *name, int ap_mode, int remote_os_type);
#define WLM_NUM_ARGS 32
#define MAX_COMMAND_RESULT_BUF_LEN 8192
static const char *outf = "output.txt";
#endif

#define WLMPRVAL(name) stats += sprintf(stats, "%s %u ", #name, dtoh32(cnt->name))
#define WLMPRNL() stats += sprintf(stats, "\n")

static char errorString[256];
static const char *wlm_version_string = WLM_VERSION_STR;

static const char *
wlmLastError(void)
{
	static const char *bcmerrorstrtable[] = BCMERRSTRINGTABLE;
	static char errorString[256];
	int bcmerror;

	memset(errorString, 0, sizeof(errorString));

	if (wlu_iovar_getint(irh, "bcmerror", &bcmerror)) {
		sprintf(errorString, "%s", "Failed to retrieve error");
		return errorString;
	}

	if (bcmerror > 0 || bcmerror < BCME_LAST) {
		sprintf(errorString, "Undefined error (%d)", bcmerror);
	} else {
		sprintf(errorString, "%s (%d)", bcmerrorstrtable[-bcmerror], bcmerror);
	}

	return errorString;
}

const char *
wlmGetLastError(void)
{
	return errorString;
}

int wlmWLMVersionGet(const char **buffer)
{
	*buffer = wlm_version_string;
	return TRUE;
}

int wlmApiInit(void)
{
	curBand = WLM_BAND_AUTO;
	return TRUE;
}

int wlmApiCleanup(void)
{
#if defined(RWL_SERIAL) || defined(RWL_DONGLE)
#if !defined(TARGETOS_nucleus)
            rwl_close_pipe(remote_type, irh);
#endif
#endif /* if defined(RWL_SERIAL) || defined(RWL_DONGLE) */
	irh = 0;
	return TRUE;
}

int wlmSelectInterface(WLM_DUT_INTERFACE ifType, char *ifName,
	WLM_DUT_SERVER_PORT dutServerPort, WLM_DUT_OS dutOs)
{
	int val;

#if !defined(TARGETOS_nucleus)
	/* close previous handle */
	if (irh != NULL) {
		wlmApiCleanup();
	}

	switch (ifType) {
#ifdef RWL_LOCAL
		case WLM_DUT_LOCAL:
			rwl_set_remote_type(NO_REMOTE);
			break;
#endif /* ifdef RWL_LOCAL */
#ifdef RWL_SERIAL
		case WLM_DUT_SERIAL:
			rwl_set_remote_type(REMOTE_SERIAL);
			break;
#endif /* ifdef RWL_SERIAL */
#ifdef RWL_SOCKET
		case WLM_DUT_SOCKET:
			rwl_set_remote_type(REMOTE_SOCKET);
			break;
#endif /* ifdef RWL_SOCKET */
#ifdef RWL_WIFI
		case WLM_DUT_WIFI:
			rwl_set_remote_type(REMOTE_WIFI);
			break;
#endif /* ifdef RWL_WIFI */
#ifdef RWL_DONGLE
		case WLM_DUT_DONGLE:
			rwl_set_remote_type(REMOTE_DONGLE);
			break;
#endif /* ifdef RWL_DONGLE */
		default:
			/* ERROR! Unknown interface! */
			return FALSE;
	}

	if (ifName) {
		strncpy(interfaceName, ifName, MAX_INTERFACE_NAME_LENGTH);
		interfaceName[MAX_INTERFACE_NAME_LENGTH] = 0;
	}
#if 0 //def RWL_SOCKET
	switch (dutOs) {
		case WLM_DUT_OS_LINUX:
			wl_os_type_set_rwl(LINUX_OS);
			break;
		case WLM_DUT_OS_WIN32:
			wl_os_type_set_rwl(WIN32_OS);
			break;
		default:
			/* ERROR! Unknown OS! */
			return FALSE;
	}
#endif /* ifdef RWL_SOCKET */
	switch (rwl_get_remote_type()) {
		struct ipv4_addr temp;
#ifdef RWL_SOCKET
		case REMOTE_SOCKET:
			if (!wl_atoip(interfaceName, &temp)) {
				printf("wlmSelectInterface: IP address invalid\n");
				return FALSE;
			}
			rwl_set_server_ip(interfaceName);
			rwl_set_server_port(dutServerPort);
			rwl_init_socket();
			break;
#endif /* ifdef RWL_SOCKET */
#ifdef RWL_SERIAL
		case REMOTE_SERIAL:
			rwl_set_serial_port_name(interfaceName); /* x (port number) or /dev/ttySx */
			if ((irh = rwl_open_pipe(rwl_get_remote_type(),
				rwl_get_serial_port_name(), 0, 0)) == NULL) {
				printf("wlmSelectInterface: rwl_open_pipe failed\n");
				return FALSE;
			}
			break;
#endif /* ifdef RWL_SERIAL */
#ifdef RWL_DONGLE
		case REMOTE_DONGLE:
			rwl_set_serial_port_name(interfaceName); /* COMx or /dev/ttySx */
			if ((irh = rwl_open_pipe(rwl_get_remote_type(), "\0", 0, 0)) == NULL) {
				printf("wlmSelectInterface: rwl_open_pipe failed\n");
				return FALSE;
			}
			break;
#endif /* ifdef RWL_DONGLE */
#ifdef RWL_WIFI
		case REMOTE_WIFI:
			if (!wl_ether_atoe(interfaceName,
				(struct ether_addr *)rwl_get_wifi_mac())) {
				printf("wlmSelectInterface: ethernet MAC address invalid\n");
				return FALSE;
			}
			/* intentionally no break here to pass through to NO_REMOTE case */
#endif /* ifdef RWL_WIFI */
#if defined( RWL_WIFI ) || defined( RWL_LOCAL )
		case NO_REMOTE:
#if defined(WIN32)
			adapter = atoi(interfaceName);
			if (adapter == 0)
				adapter = -1;

			if (wl_ir_init_adapter_rwl(&irh, adapter) != 0) {
				printf("wlmSelectInterface: Adapter %d init failed\n", adapter);
				return FALSE;
			}
#else
			if (wl_ir_init_rwl(&irh) != 0) {
				printf("wlmSelectInterface: initialize failed\n");
				return FALSE;
			}
#endif
			break;
#endif /* if defined( RWL_WIFI ) || defined( RWL_LOCAL ) */
		default:
			/* ERROR! Invalid interface!
			 * NOTE: API should not allow code to come here.
			 */
			return FALSE;
	}
#endif /* !defined(TARGETOS_nucleus) */

	/* Query the IOCTL API version */
	if (wlu_get(irh, WLC_GET_VERSION, &val, sizeof(int)) < 0) {
		printf("wlmSelectInterface: IOCTL Version query failed\n");
		return FALSE;
	}

	ioctl_version = dtoh32(val);
	if (ioctl_version != WLC_IOCTL_VERSION &&
	    ioctl_version != 1) {
		printf("wlmSelectInterface: Version mismatch, please upgrade."
		       "Got %d, expected %d or 1\n",
		        ioctl_version, WLC_IOCTL_VERSION);
		return FALSE;
	}


	return TRUE;
}

int wlmVersionGet(char *buf, int len)
{
	char fwVersionStr[256];
	int n = 0;

	if (buf == 0) {
		printf("wlmVersionGet: buffer invalid\n");
		return FALSE;
	}

	memset(buf, 0, sizeof(buf));

	n += sprintf(buf, "wlm: ");
	n += sprintf(buf + n, "%s",
	     ver2str(((EPI_MAJOR_VERSION) << 16) |
		EPI_MINOR_VERSION, (EPI_RC_NUMBER << 16) |
		EPI_INCREMENTAL_NUMBER));
	n += sprintf(buf + n, " ");

	/* query for 'ver' to get version info */
	if (wlu_iovar_get(irh, "ver", fwVersionStr,
		(len < WLC_IOCTL_SMLEN) ? len : WLC_IOCTL_SMLEN)) {
		printf("wlmVersionGet: %s\n", wlmLastError());
		return FALSE;
	}

	n += sprintf(buf + n, "%s", fwVersionStr);
	return TRUE;
}

int wlmEnableAdapterUp(int enable)
{
	/*  Enable/disable adapter  */
	if (enable)
	{
		if (wlu_set(irh, WLC_UP, NULL, 0)) {
			printf("wlmEnableAdapterUp: %s\n", wlmLastError());
			return FALSE;
		}
	}
	else {
		if (wlu_set(irh, WLC_DOWN, NULL, 0)) {
			printf("wlmEnableAdapterUp: %s\n", wlmLastError());
			return FALSE;
		}
	}

	return TRUE;
}

int wlmIsAdapterUp(int *up)
{
	/*  Get 'isup' - check if adapter is up */
	up = dtoh32(up);
	if (wlu_get(irh, WLC_GET_UP, up, sizeof(int))) {
		printf("wlmIsAdapterUp: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmMinPowerConsumption(int enable)
{
	if (wlu_iovar_setint(irh, "mpc", enable)) {
		printf("wlmMinPowerConsumption: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmMimoPreambleGet(int* type)
{
	if (wlu_iovar_getint(irh, "mimo_preamble", type)) {
		printf("wlmMimoPreambleGet(): %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int wlmMimoPreambleSet(int type)
{
	if (wlu_iovar_setint(irh, "mimo_preamble", type)) {
		printf("wlmMimoPreambleSet(): %s\n", wlmLastError());
		return FALSE;
	}
	return  TRUE;
}

int wlmChannelSet(int channel)
{

	/* Check band lock first before set  channel */
	if ((channel <= 14) && (curBand != WLM_BAND_2G)) {
		curBand = WLM_BAND_2G;
	} else if ((channel > 14) && (curBand != WLM_BAND_5G)) {
		curBand = WLM_BAND_5G;
	}

	/* Set 'channel' */
	channel = htod32(channel);
	if (wlu_set(irh, WLC_SET_CHANNEL, &channel, sizeof(channel))) {
		printf("wlmChannelSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmRateSet(WLM_RATE rate)
{
	char aname[] = "a_rate";
	char bgname[] = "bg_rate";
	char *name;

	switch (curBand) {
	        case WLM_BAND_AUTO :
			printf("wlmRateSet: must set channel or band lock first\n");
			return FALSE;
	        case WLM_BAND_DUAL :
			printf("wlmRateSet: must set channel or band lock first\n");
			return FALSE;
		case WLM_BAND_5G :
			name = (char *)aname;
			break;
		case WLM_BAND_2G :
			name = (char *)bgname;
			break;
		default :
			return FALSE;
	}

	rate = htod32(rate);
	if (wlu_iovar_setint(irh, name, rate)) {
		printf("wlmRateSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmLegacyRateSet(WLM_RATE rate)
{
	uint32 nrate = 0;

	if (ioctl_version == 1) {
		nrate |= (rate & OLD_NRATE_RATE_MASK);
		nrate |= (OLD_NRATE_STF_SISO << OLD_NRATE_STF_SHIFT) & OLD_NRATE_STF_MASK;
	} else {
		nrate = WL_RSPEC_ENCODE_RATE;	/* 11abg */
		nrate |= (rate & WL_RSPEC_RATE_MASK);
	}

	if (wlu_iovar_setint(irh, "nrate", (int)nrate)) {
		printf("wlmLegacyRateSet: %s\n", wlmLastError());
		return FALSE;
	}

	return  TRUE;
}

int wlmMcsRateSet(WLM_MCS_RATE mcs_rate, WLM_STF_MODE stf_mode)
{
	uint32 nrate = 0;
	uint stf;

	if (mcs_rate > 32) {
		printf("wlmMcsRateSet: MCS %d out of range\n", mcs_rate);
		return FALSE;
	}

	if (ioctl_version == 1) {
		nrate |= mcs_rate;
		nrate |= OLD_NRATE_MCS_INUSE;

		if (!stf_mode) {
			stf = 0;
			if (mcs_rate <= HIGHEST_SINGLE_STREAM_MCS ||
			    mcs_rate == 32)
				stf = OLD_NRATE_STF_SISO;	/* SISO */
			else
				stf = OLD_NRATE_STF_SDM;	/* SDM */
		} else
			stf = stf_mode;

		nrate |= (stf << OLD_NRATE_STF_SHIFT) & OLD_NRATE_STF_MASK;
	} else {
		nrate = WL_RSPEC_ENCODE_HT;	/* 11n HT */
		nrate |= mcs_rate;

		/* decode WLM stf value into tx expansion and STBC */
		if (stf_mode == WLM_STF_MODE_CDD) {
			nrate |= (1 << WL_RSPEC_TXEXP_SHIFT);
		} else if (stf_mode == WLM_STF_MODE_STBC) {
			nrate |= WL_RSPEC_STBC;
		}
	}

	if (wlu_iovar_setint(irh, "nrate", (int)nrate)) {
		printf("wlmMcsRateSet: %s\n", wlmLastError());
		return FALSE;
	}
	return  TRUE;
}

int wlmHTRateSet(WLM_MCS_RATE mcs_rate, int stbc, int ldpc, int sgi, int tx_exp, int bw)
{
	uint32 rspec = 0;
	char iov_2g[] = "2g_rate";
	char iov_5g[] = "5g_rate";
	char *name;

	switch (curBand) {
	        case WLM_BAND_AUTO :
			printf("wlmHTRateSet: must set channel or band lock first \n");
			return FALSE;
	        case WLM_BAND_DUAL :
			printf("wlmHTRateSet: must set channel or band lock first\n");
			return FALSE;
		case WLM_BAND_5G :
			name = (char *)iov_5g;
			break;
		case WLM_BAND_2G :
			name = (char *)iov_2g;
			break;
		default :
			printf("wlmHTRateSet: band setting unknow\n");
			return FALSE;
	}

	/* set rate to auto ??? */
	if (mcs_rate < 0)
		rspec = 0;

	/* 11n HT rate */
	rspec = WL_RSPEC_ENCODE_HT;
	rspec |= mcs_rate;

	/* set bandwidth */
	if (bw == 20) {
		bw = WL_RSPEC_BW_20MHZ;
	} else if (bw == 40) {
		bw = WL_RSPEC_BW_40MHZ;
	} else if (bw == 80) {
		bw = WL_RSPEC_BW_80MHZ;
	} else if (bw == 160) {
		bw = WL_RSPEC_BW_160MHZ;
	} else {
		printf("wlmHTRateSet: unexpected bandwidth specified \"%d\", "
		       "expected 20, 40, 80, or 160\n", bw);
		return FALSE;
	}

	if (tx_exp < 0 || tx_exp > 3) {
		printf("wlmHTRataSet: tx expansion %d out of range [0-3]\n", tx_exp);
		return FALSE;
	}
	/* set the other rspec fields */
	rspec |= (tx_exp << WL_RSPEC_TXEXP_SHIFT);
	rspec |= bw;
	rspec |= (stbc ? WL_RSPEC_STBC : 0);
	rspec |= (ldpc ? WL_RSPEC_LDPC : 0);
	rspec |= (sgi  ? WL_RSPEC_SGI  : 0);


	if (wlu_iovar_setint(irh, name, rspec)) {
		printf("wlmHTRateSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmVHTRateSet(WLM_MCS_RATE mcs_rate, int nss, int stbc, int ldpc, int sgi, int tx_exp, int bw)
{
	uint32 rspec = 0;
	char iov_2g[] = "2g_rate";
	char iov_5g[] = "5g_rate";
	char *name;

	switch (curBand) {
	        case WLM_BAND_AUTO :
			printf("wlmVHTRateSet: must set channel or band lock first \n");
			return FALSE;
	        case WLM_BAND_DUAL :
			printf("wlmVHTRateSet: must set channel or band lock first\n");
			return FALSE;
		case WLM_BAND_5G :
			name = (char *)iov_5g;
			break;
		case WLM_BAND_2G :
			name = (char *)iov_2g;
			break;
		default :
			printf("wlmVHTRateSet: band setting unknow\n");
			return FALSE;
	}

	/* set rate to auto */
	if (mcs_rate < 0)
		rspec = 0;

	/* 11n VHT rate */
	rspec = WL_RSPEC_ENCODE_VHT;

	/* default NSS = 1 */
	if (nss == 0)
		nss = 1;

	rspec |= (nss << WL_RSPEC_VHT_NSS_SHIFT) | mcs_rate;

	/* set bandwidth */
	if (bw == 20) {
		bw = WL_RSPEC_BW_20MHZ;
	} else if (bw == 40) {
		bw = WL_RSPEC_BW_40MHZ;
	} else if (bw == 80) {
		bw = WL_RSPEC_BW_80MHZ;
	} else if (bw == 160) {
		bw = WL_RSPEC_BW_160MHZ;
	} else {
		printf("wlmVHTRateSet: unexpected bandwidth specified \"%d\", "
		       "expected 20, 40, 80, or 160\n", bw);
		return FALSE;
	}

	if (tx_exp < 0 || tx_exp > 3) {
		printf("wlmVHTRataSet: tx expansion %d out of range [0-3]\n", tx_exp);
		return FALSE;
	}

	/* set the other rspec fields */
	rspec |= tx_exp << WL_RSPEC_TXEXP_SHIFT;
	rspec |= bw;
	rspec |= (stbc ? WL_RSPEC_STBC : 0);
	rspec |= (ldpc ? WL_RSPEC_LDPC : 0);
	rspec |= (sgi  ? WL_RSPEC_SGI  : 0);

	if (wlu_iovar_setint(irh, name, rspec)) {
		printf("wlmVHTRateSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmPreambleSet(WLM_PREAMBLE preamble)
{
	preamble = htod32(preamble);

	if (wlu_set(irh, WLC_SET_PLCPHDR, &preamble, sizeof(preamble))) {
		printf("wlmPreambleSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmBandSet(WLM_BAND band)
{
	band = htod32(band);

	if (wlu_set(irh, WLC_SET_BAND, (void *)&band, sizeof(band))) {
		printf("wlmBandSet: %s\n", wlmLastError());
		return FALSE;
	}

	curBand = band;

	return TRUE;
}

int wlmBandGet(WLM_BAND *band)
{
	if (wlu_get(irh, WLC_GET_BAND, band, sizeof(band))) {
		printf("wlmBandGet: %s\n", wlmLastError());
		return FALSE;
	}

	/* set curBand to make sure it matches the current band status */
	curBand = *band;
	return TRUE;
}

int wlmGetBandList(WLM_BAND * bands)
{
	unsigned int list[3];
	unsigned int i;

	if (wlu_get(irh, WLC_GET_BANDLIST, list, sizeof(list))) {
		printf("wlmGetBandList: %s\n", wlmLastError());
		return FALSE;
	}

	list[0] = dtoh32(list[0]);
	list[1] = dtoh32(list[1]);
	list[2] = dtoh32(list[2]);

	/* list[0] is count, followed by 'count' bands */

	if (list[0] > 2)
		list[0] = 2;

	for (i = 1, *bands = (WLM_BAND)0; i <= list[0]; i++)
		*bands |= list[i];

	return TRUE;
}

int wlmGmodeSet(WLM_GMODE gmode)
{
	/*  Set 'gmode' - select mode in 2.4G band */
	gmode = htod32(gmode);

	if (wlu_set(irh, WLC_SET_GMODE, (void *)&gmode, sizeof(gmode))) {
		printf("wlmGmodeSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmRxAntSet(int antenna)
{
	/*  Set 'antdiv' - select receive antenna */
	antenna = htod32(antenna);

	if (wlu_set(irh, WLC_SET_ANTDIV, &antenna, sizeof(antenna))) {
		printf("wlmRxAntSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmTxAntSet(int antenna)
{
	/*  Set 'txant' - select transmit antenna */
	antenna = htod32(antenna);

	if (wlu_set(irh, WLC_SET_TXANT, &antenna, sizeof(antenna))) {
		printf("wlmTxAntSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmEstimatedPowerGet(int *estPower, int chain)
{
	int mimo;
	int is2g;

	size_t pprsize = ppr_ser_size_by_bw(ppr_get_max_bw());
	/* Allocate memory for structure and 2 serialisation buffer */
	tx_pwr_rpt_t *ppr_wl = (tx_pwr_rpt_t *)malloc(sizeof(tx_pwr_rpt_t) + pprsize*2);
	uint8 *ppr_ser;
	if (ppr_wl == NULL)
		return FALSE;

	ppr_ser = ppr_wl->pprdata;
	memset(ppr_wl, 0, sizeof(tx_pwr_rpt_t) + pprsize*2);
	ppr_wl->board_limit_len  = pprsize;
	ppr_wl->target_len       = pprsize;
	ppr_wl->version          = TX_POWER_T_VERSION;
	/* init allocated mem for serialisation */
	ppr_init_ser_mem_by_bw(ppr_ser, ppr_get_max_bw(), ppr_wl->board_limit_len);
	ppr_ser += ppr_wl->board_limit_len;
	ppr_init_ser_mem_by_bw(ppr_ser, ppr_get_max_bw(), ppr_wl->target_len);


	if (wlu_get(irh, WLC_CURRENT_PWR, ppr_wl, sizeof(tx_pwr_rpt_t) + pprsize*2) < 0) {
		printf("wlmEstimatedPowerGet: %s\n", wlmLastError());
		free(ppr_wl);
		return FALSE;
	}
	ppr_wl->flags = dtoh32(ppr_wl->flags);
	ppr_wl->chanspec = dtohchanspec(ppr_wl->chanspec);
	mimo = (ppr_wl->flags & WL_TX_POWER_F_MIMO);

	/* value returned is in units of quarter dBm, need to multiply by 250 to get milli-dBm */
	if (mimo) {
		*estPower = ppr_wl->est_Pout[chain] * 250;
	} else {
		*estPower = ppr_wl->est_Pout[0] * 250;
	}

	if (ioctl_version == 1) {
		is2g = LCHSPEC_IS2G(ppr_wl->chanspec);
	} else {
		is2g = CHSPEC_IS2G(ppr_wl->chanspec);
	}

	if (!mimo && is2g) {
		*estPower = ppr_wl->est_Pout_cck * 250;
	}

	free(ppr_wl);
	return TRUE;
}

int wlmTxPowerGet(int *power)
{
	int val;
	int8 temp_val;

	if ((wlu_iovar_getint(irh, "qtxpower", &val)) < 0) {
		printf("wlmTxPowerGet: %s\n", wlmLastError());
		return FALSE;
	}

	val &= ~WL_TXPWR_OVERRIDE;
	temp_val = (int8)(val & 0xff);

	/* value returned is in units of quarter dBm, need to multiply by 250 to get milli-dBm */
	*power = temp_val * 250;
	return TRUE;
}

#if 0
int wlmTxPowerSet(int powerValue)
{
	int newValue = 0;

	if (powerValue == -1) {
		newValue = 127;		/* Max val of 127 qdbm */
	} else {
		/* expected to be in units of quarter dBm */
		newValue = powerValue / 250;
		newValue |= WL_TXPWR_OVERRIDE;
	}

	if (wlu_iovar_setint(irh, "qtxpower", newValue)) {
		printf("wlmTxPowerSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}
#endif /* 0 */

int wlmTxPowerSet(int powerValue)
{
	int newValue = 0;
	int8 temp_val;

	if (powerValue == -1) {
		newValue = WLC_TXPWR_MAX ; 	/* Max val of 127 qdbm */
	} else {
		/* expected input param to be in units of milli dBm */
		/* convert to milli quater dBm */
		newValue = powerValue / 250;

		if (newValue > WLC_TXPWR_MAX) {
			printf("wlmTxPowerSet: %d is invalid target power,"
			       "should be within [-128, 127] qdBm.\n", newValue);
			return FALSE;
		}

		if (newValue < WL_RATE_DISABLED) {
			newValue = WL_RATE_DISABLED;
		}
		temp_val = newValue;
		newValue = (uint8) temp_val;
		newValue |= WL_TXPWR_OVERRIDE;
	}

	if (wlu_iovar_setint(irh, "qtxpower", newValue)) {
		printf("wlmTxPowerSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}
static int wlmPhyTypeGet(void)
{

	int phytype = PHY_TYPE_NULL;

	if (wlu_get(irh, WLC_GET_PHYTYPE, &phytype, sizeof(int)) < 0) {
	        printf("wlmPhyTypeGet: %s\n", wlmLastError());
		return FALSE;
	}

	return phytype;
}

int wlmPaParametersGet(WLM_BANDRANGE bandrange,
	unsigned int *a1, unsigned int *b0, unsigned int *b1)
{
	uint16 inpa[WL_PHY_PAVARS_LEN];
	void *ptr = NULL;
	uint16 *outpa;
	int i = 0;
	int phytype = PHY_TYPE_NULL;

	*a1 = 0;
	*b0 = 0;
	*b1 = 0;

	/* Do not rely on user to have knowledge of phytype */
	phytype = wlmPhyTypeGet();
	if (phytype != PHY_TYPE_NULL) {
		inpa[i++] = phytype;
		inpa[i++] = bandrange;
		inpa[i++] = 0;  /* Fix me: default with chain 0 for all SISO system */
	} else {
		printf("wlmPaParametersGet: unknown Phy type\n");
		return FALSE;
	}

	if (wlu_var_getbuf_sm(irh, "pavars", inpa, WL_PHY_PAVARS_LEN * sizeof(uint16), &ptr)) {
		printf("wlmPaParametersGet: %s\n", wlmLastError());
		return FALSE;
	}

	outpa = (uint16 *)ptr;
	*b0 = outpa[i++];
	*b1 = outpa[i++];
	*a1 = outpa[i++];

	return TRUE;
}

int wlmPaParametersSet(WLM_BANDRANGE bandrange,
	unsigned int a1, unsigned int b0, unsigned int b1)
{
	uint16 inpa[WL_PHY_PAVARS_LEN];
	int i = 0;
	int phytype = PHY_TYPE_NULL;

	/* Do not rely on user to have knowledge of phy type */
	phytype = wlmPhyTypeGet();
	if (phytype != PHY_TYPE_NULL) {
		inpa[i++] = phytype;
		inpa[i++] = bandrange;
		inpa[i++] = 0;  /* Fix me: default with chain 0 for all SISO system */
	} else {
	        printf("wlmPaParametersSet: unknown Phy type\n");
		return FALSE;
	}

	inpa[i++] = b0;
	inpa[i++] = b1;
	inpa[i++] = a1;

	if (wlu_var_setbuf(irh, "pavars", inpa, WL_PHY_PAVARS_LEN * sizeof(uint16))) {
		printf("wlmPaParametersSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}


int wlmMIMOPaParametersGet(WLM_BANDRANGE bandrange, int chain,
	unsigned int *a1, unsigned int *b0, unsigned int *b1)
{
	uint16 inpa[WL_PHY_PAVARS_LEN];
	void *ptr = NULL;
	uint16 *outpa;
	int i = 0;
	int phytype = PHY_TYPE_NULL;

	/* Do not rely on user to have knowledge of phytype */
	phytype = wlmPhyTypeGet();
	if (phytype != PHY_TYPE_NULL) {
		inpa[i++] = phytype;
		inpa[i++] = bandrange;
		inpa[i++] = chain;
	} else {
		printf("wlmMIMOPaParametersGet: unknown Phy type\n");
		return FALSE;
	}

	if (wlu_var_getbuf_sm(irh, "pavars", inpa, WL_PHY_PAVARS_LEN * sizeof(uint16), &ptr)) {
		printf("wlmMIMOPaParametersGet: %s\n", wlmLastError());
		return FALSE;
	}

	outpa = (uint16 *)ptr;
	*a1 = outpa[i++];
	*b0 = outpa[i++];
	*b1 = outpa[i++];

	return TRUE;
}

int wlmMIMOPaParametersSet(WLM_BANDRANGE bandrange, int chain,
	unsigned int a1, unsigned int b0, unsigned int b1)
{
	uint16 inpa[WL_PHY_PAVARS_LEN];
	int i = 0;
	int phytype = PHY_TYPE_NULL;

	/* Do not rely on user to have knowledge of phy type */
	phytype = wlmPhyTypeGet();
	if (phytype != PHY_TYPE_NULL) {
	        inpa[i++] = phytype;
		inpa[i++] = bandrange;
		inpa[i++] = chain;
	} else {
	        printf("wlmMIMOPaParametersSet: unknown Phy type\n");
		return FALSE;
	}

	inpa[i++] = a1;
	inpa[i++] = b0;
	inpa[i++] = b1;

	if (wlu_var_setbuf(irh, "pavars", inpa, WL_PHY_PAVARS_LEN * sizeof(uint16))) {
		printf("wlmMIMOPaParametersSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmACPaParametersGet(WLM_BANDRANGE bandrange, int modetype,
	unsigned int *a1, unsigned int *b0, unsigned int *b1)
{
	uint16 inpa[WL_PHY_PAVARS_LEN] = {0};
	void *ptr = NULL;
	uint16 *outpa;
	int i = 0;
	int phytype = PHY_TYPE_NULL;

	/* Do not rely on user to have knowledge of phytype */
	phytype = wlmPhyTypeGet();
	if (phytype != PHY_TYPE_AC) {
		printf("wlmACPaParametersGet: Wrong PHY type\n");
		return FALSE;
	}
	inpa[i++] = phytype;

	if (bandrange == WL_CHAN_FREQ_RANGE_2G) {
		inpa[i++] = WL_CHAN_FREQ_RANGE_2G;
	} else if ((bandrange > WL_CHAN_FREQ_RANGE_2G) &&
		(bandrange < WL_CHAN_FREQ_RANGE_5G_4BAND)) {
		inpa[i++] = WL_CHAN_FREQ_RANGE_5G_4BAND;
	} else {
		printf("wlmACPaParametersGet: Wrong bandrange [0-4]\n");
		return FALSE;
	}

	if (modetype > 2) {
		printf("wlmACPaParametersGet: Wrong modetype number [0-2]\n");
		return FALSE;
	}
	inpa[i++] = modetype;

	if (wlu_var_getbuf_sm(irh, "pavars", inpa, WL_PHY_PAVARS_LEN * sizeof(uint16), &ptr)) {
		printf("wlmACPaParametersGet: %s\n", wlmLastError());
		return FALSE;
	}

	outpa = (uint16 *)ptr;

	if (bandrange > 0)
		i += (bandrange - WL_CHAN_FREQ_RANGE_5G_BAND0) * 3;
	/*
	  printf("0x%x\n", outpa[i++]);
	  printf("0x%x\n", outpa[i++]);
	  printf("0x%x\n", outpa[i++]);
	*/
	 *a1 = outpa[i++];
	 *b0 = outpa[i++];
	 *b1 = outpa[i++];

	return TRUE;
}

int wlmACPaParametersSet(WLM_BANDRANGE bandrange, int modetype,
	unsigned int a1, unsigned int b0, unsigned int b1)
{
	uint16 inpa[WL_PHY_PAVARS_LEN] = {0};
	int i = 0;
	int phytype = PHY_TYPE_NULL;
	void *ptr = NULL;
	uint16 *outpa;
	int pnum = 0, n = 0;

	/* Do not rely on user to have knowledge of phy type */
	phytype = wlmPhyTypeGet();
	if (phytype != PHY_TYPE_AC) {
		printf("wlmACPaParametersSet: Wrong PHY type\n");
		return FALSE;
	}
	inpa[i++] = phytype;

	if (bandrange == WL_CHAN_FREQ_RANGE_2G) {
		pnum = 3;
		inpa[i++] = WL_CHAN_FREQ_RANGE_2G;
	} else if ((bandrange > WL_CHAN_FREQ_RANGE_2G) &&
		(bandrange < WL_CHAN_FREQ_RANGE_5G_4BAND)) {
		pnum = 12;
		inpa[i++] = WL_CHAN_FREQ_RANGE_5G_4BAND;
	} else {
		printf("wlmACPaParametersSet: Wrong bandrange [0-4]\n");
		return FALSE;
	}

	if (modetype > 2) {
		printf("wlmACPaParametersSet: Wrong modetype number [0-2]\n");
		return FALSE;
	}
	inpa[i++] = modetype;

	if (wlu_var_getbuf_sm(irh, "pavars", inpa, WL_PHY_PAVARS_LEN * sizeof(uint16), &ptr)) {
		printf("wlmACPaParametersSet: %s\n", wlmLastError());
		return FALSE;
	}

	outpa = (uint16 *) ptr;

	for (n = 0; n < pnum + 3; n++) {
		if ((bandrange > 0) && (n == 3 +((bandrange - WL_CHAN_FREQ_RANGE_5G_BAND0) * 3))) {
			inpa[n++] = a1;
			/* printf("n: %d 0x%hx 0x%hx\n", n -1, inpa[n - 1], outpa[n - 1]); */
			inpa[n++] = b0;
			/* printf("n: %d 0x%hx 0x%hx\n", n - 1, inpa[n -1], outpa[n - 1]); */
			inpa[n] = b1;
			/* printf("n: %d 0x%hx 0x%hx\n", n, inpa[n], outpa[n]); */
		} else if ((bandrange == WL_CHAN_FREQ_RANGE_2G) && (n > 2)) {
			inpa[n++] = a1;
			/* printf("n: %d 0x%hx 0x%hx\n", n - 1, inpa[n -1], outpa[n - 1]); */
			inpa[n++] = b0;
			/* printf("n: %d 0x%hx 0x%hx\n", n - 1, inpa[n - 1], outpa[n - 1]); */
			inpa[n] = b1;
			/* printf("n: %d 0x%hx 0x%hx\n", n, inpa[n], outpa[n]); */
		} else {
			inpa[n] = outpa[n];
			/* printf("n: %d 0x%hx 0x%hx\n", n, inpa[n], outpa[n]); */
		}
	}


	if (wlu_var_setbuf(irh, "pavars", inpa, WL_PHY_PAVARS_LEN * sizeof(uint16))) {
		printf("wlmACPaParametersSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmMacAddrGet(char *macAddr, int length)
{
	struct ether_addr ea = {{0, 0, 0, 0, 0, 0}};

	/* query for 'cur_etheraddr' to get MAC address */
	if (wlu_iovar_get(irh, "cur_etheraddr", &ea, ETHER_ADDR_LEN) < 0) {
		printf("wlmMacAddrGet: %s\n", wlmLastError());
		return FALSE;
	}

	strncpy(macAddr, wl_ether_etoa(&ea), length);

	return TRUE;
}

int wlmMacAddrSet(const char* macAddr)
{
	struct ether_addr ea;

	if (!wl_ether_atoe(macAddr, &ea)) {
		printf("wlmMacAddrSet: MAC address invalid: %s\n", macAddr);
		return FALSE;
	}

	/*  Set 'cur_etheraddr' to set MAC address */
	if (wlu_iovar_set(irh, "cur_etheraddr", (void *)&ea, ETHER_ADDR_LEN)) {
		printf("wlmMacAddrSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmEnableCarrierTone(int enable, int channel)
{
	int val = channel;

	if (!enable) {
		val = 0;
	}
	else {
		wlmEnableAdapterUp(1);
		if (wlu_set(irh, WLC_OUT, NULL, 0) < 0) {
		printf("wlmEnableCarrierTone: %s\n", wlmLastError());
		return FALSE;
		}
	}
	val = htod32(val);
	if (wlu_set(irh, WLC_FREQ_ACCURACY, &val, sizeof(int)) < 0) {
		printf("wlmEnableCarrierTone: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int wlmEnableEVMTest(int enable, WLM_RATE rate, int channel)
{
	int val[3] = {0};
	val[1] = WLM_RATE_1M; /* default value */
	if (enable) {
		val[0] = htod32(channel);
		val[1] = htod32(rate);
		wlmEnableAdapterUp(1);
		if (wlu_set(irh, WLC_OUT, NULL, 0) < 0) {
			printf("wlmEnableEVMTest: %s\n", wlmLastError());
			return FALSE;
		}
	}
	if (wlu_set(irh, WLC_EVM, val, sizeof(val)) < 0) {
		printf("wlmEnableEVMTest: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int wlmTxPacketStart(unsigned int interPacketDelay,
	unsigned int numPackets, unsigned int packetLength,
	const char* destMac, int withAck, int syncMode)
{
	wl_pkteng_t pkteng;

	if (!wl_ether_atoe(destMac, (struct ether_addr *)&pkteng.dest)) {
		printf("wlmTxPacketStart: destMac invalid\n");
		return FALSE;
	}

	pkteng.flags = withAck ? WL_PKTENG_PER_TX_WITH_ACK_START : WL_PKTENG_PER_TX_START;

	if (syncMode)
		pkteng.flags |= WL_PKTENG_SYNCHRONOUS;
	else
		pkteng.flags &= ~WL_PKTENG_SYNCHRONOUS;

	pkteng.delay = interPacketDelay;
	pkteng.length = packetLength;
	pkteng.nframes = numPackets;

	pkteng.seqno = 0;			/* not used */
	pkteng.src = ether_null;	/* implies current ether addr */

	if (wlu_var_setbuf(irh, "pkteng", &pkteng, sizeof(pkteng))) {
		printf("wlmTxPacketStart: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmTxPacketStop(void)
{
	wl_pkteng_t pkteng;

	memset(&pkteng, 0, sizeof(pkteng));
	pkteng.flags = WL_PKTENG_PER_TX_STOP;

	if (wlu_var_setbuf(irh, "pkteng", &pkteng, sizeof(pkteng)) < 0) {
		printf("wlmTxPacketStop: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmRxPacketStart(const char* srcMac, int withAck,
	int syncMode, unsigned int numPackets, unsigned int timeout)
{
	wl_pkteng_t pkteng;

	if (!wl_ether_atoe(srcMac, (struct ether_addr *)&pkteng.dest)) {
		printf("wlmRxPacketStart: srcMac invalid\n");
		return FALSE;
	}

	pkteng.flags = withAck ? WL_PKTENG_PER_RX_WITH_ACK_START : WL_PKTENG_PER_RX_START;

	if (syncMode) {
		pkteng.flags |= WL_PKTENG_SYNCHRONOUS;
		pkteng.nframes = numPackets;
		pkteng.delay = timeout;
	}
	else
		pkteng.flags &= ~WL_PKTENG_SYNCHRONOUS;

	pkteng.length = 0;

	if (wlu_var_setbuf(irh, "pkteng", &pkteng, sizeof(pkteng))) {
		printf("wlmRxPacketStart: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmRxPacketStop(void)
{
	wl_pkteng_t pkteng;

	memset(&pkteng, 0, sizeof(pkteng));
	pkteng.flags = WL_PKTENG_PER_RX_STOP;

	if (wlu_var_setbuf(irh, "pkteng", &pkteng, sizeof(pkteng)) < 0) {
		printf("wlmRxPacketStop: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmTxGetAckedPackets(unsigned int *count)
{
	wl_cnt_t *cnt;

	if (wlu_var_getbuf_med(irh, "counters", NULL, 0, (void **)&cnt)) {
		printf("wlmTxGetAckedPackets: %s\n", wlmLastError());
		return FALSE;
	}

	*count = dtoh32(cnt->rxackucast);

	return TRUE;
}

int wlmRxGetReceivedPackets(unsigned int *count)
{
	wl_cnt_t *cnt;

	if (wlu_var_getbuf_med(irh, "counters", NULL, 0, (void **)&cnt)) {
		printf("wlmRxGetReceivedPackets: %s\n", wlmLastError());
		return FALSE;
	}

	cnt->version = dtoh16(cnt->version);
	cnt->length = dtoh16(cnt->version);

	/* current wl_cnt_t version is 7 */
	if (cnt->version == WL_CNT_T_VERSION) {
		*count = dtoh32(cnt->pktengrxducast);
	} else {
		*count = dtoh32(cnt->rxdfrmucastmbss);
	}

	return TRUE;
}

int wlmRssiGet(int *rssi)
{
	wl_pkteng_stats_t *cnt;

	if (wlu_var_getbuf_sm(irh, "pkteng_stats", NULL, 0, (void **)&cnt)) {
		printf("wlmRssiGet: %s\n", wlmLastError());
		return FALSE;
	}

	*rssi = dtoh32(cnt->rssi);
	return TRUE;
}

int wlmUnmodRssiGet(int* rssi)
{
	if (wlu_iovar_getint(irh, "unmod_rssi", rssi)) {
		printf("wlmUnmodRssiGet: %s\n", wlmLastError());
		return FALSE;
	}
	*rssi = dtoh32(*rssi);

	return TRUE;
}

#if 0
/* Disabling this function as the lostfrmcnt
 * is a large value and doesn't make any sense.
 */
int wlmRxGetLostPackets(unsigned int *count)
{
	wl_pkteng_stats_t *cnt;

	if (wlu_var_getbuf_sm(irh, "pkteng_stats", NULL, 0, (void **)&cnt)) {
		printf("wlmRxGetLostPackets: %s\n", wlmLastError());
		return FALSE;
	}

	*count = dtoh32(cnt->lostfrmcnt);

	return TRUE;
}
#endif /* disable function */


int wlmSequenceStart(int clientBatching)
{
	if (wl_seq_batch_in_client((bool)clientBatching)) {
		printf("wlmSequenceStart: %s\n", wlmLastError());
		return FALSE;
	}

	if (wl_seq_start(irh, 0, 0)) {
		printf("wlmSequenceStart: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmSequenceStop(void)
{
	if (wl_seq_stop(irh, 0, 0)) {
		printf("wlmSequenceStop: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmSequenceDelay(int msec)
{
	if (wlu_iovar_setint(irh, "seq_delay", msec)) {
		printf("wlmSequenceDelay: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmSequenceErrorIndex(int *index)
{
	if (wlu_iovar_getint(irh, "seq_error_index", index)) {
		printf("wlmSequenceErrorIndex: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmDeviceImageWrite(const char* byteStream, int length, WLM_IMAGE_TYPE imageType)
{
	srom_rw_t *srt;
	char buffer[WLC_IOCTL_MAXLEN] = {0};

	char *bufp;
	char *cisp, *cisdata;
	cis_rw_t cish;

	if (byteStream == NULL) {
		printf("wlmDeviceImageWrite: Buffer is invalid!\n");
		return FALSE;
	}
	if (length > SROM_MAX+1) {
	    printf("wlmDeviceImageWrite: Data length should be less than %d bytes\n", SROM_MAX);
	    return FALSE;
	}

	switch (imageType) {
	case WLM_TYPE_SROM:
		srt = (srom_rw_t *)buffer;
		memcpy(srt->buf, byteStream, length);

		if (length == SROM4_WORDS * 2) {
			if ((srt->buf[SROM4_SIGN] != SROM4_SIGNATURE) &&
			    (srt->buf[SROM8_SIGN] != SROM4_SIGNATURE)) {
				printf("wlmDeviceImageWrite: Data lacks a REV4 signature!\n");
			    return FALSE;
			}
		} else if ((length != SROM_WORDS * 2) && (length != SROM_MAX)) {
		    printf("wlmDeviceImageWrite: Data length is invalid!\n");
		    return FALSE;
		}

		srt->nbytes = length;
		if (wlu_set(irh, WLC_SET_SROM, buffer, length + 8)) {
		    printf("wlmDeviceImageWrite: %s\n", wlmLastError());
		    return FALSE;
		}


		break;
	case WLM_TYPE_OTP:
		bufp = buffer;
		strcpy(bufp, "ciswrite");
		bufp += strlen("ciswrite") + 1;
		cisp = bufp;
		cisdata = cisp + sizeof(cish);
		cish.source = htod32(0);
		memcpy(cisdata, byteStream, length);

		cish.byteoff = htod32(0);
		cish.nbytes = htod32(length);
		memcpy(cisp, (char*)&cish, sizeof(cish));

		if (wl_set(irh, WLC_SET_VAR, buffer, (cisp - buffer) + sizeof(cish) + length) < 0) {
		    printf("wlmDeviceImageWrite: %s\n", wlmLastError());
		    return FALSE;
		 }
		break;
	case WLM_TYPE_AUTO:
		if (!wlmDeviceImageWrite(byteStream, length, WLM_TYPE_SROM) &&
			!wlmDeviceImageWrite(byteStream, length, WLM_TYPE_OTP)) {
		    printf("wlmDeviceImageWrite: %s\n", wlmLastError());
		    return FALSE;
	    }
	    break;
	default:
		printf("wlmDeviceImageWrite: Invalid image type!\n");
		return FALSE;
	}
	return TRUE;
}

int wlmDeviceImageRead(char* byteStream, unsigned int len, WLM_IMAGE_TYPE imageType)
{
	srom_rw_t *srt;
	cis_rw_t *cish;
	char buf[WLC_IOCTL_MAXLEN] = {0};
	unsigned int numRead = 0;

	if (byteStream == NULL) {
		printf("wlmDeviceImageRead: Buffer is invalid!\n");
		return FALSE;
	}

	if (len > SROM_MAX) {
	    printf("wlmDeviceImageRead: byteStream should be less than %d bytes!\n", SROM_MAX);
	    return FALSE;
	}

	if (len & 1) {
			printf("wlmDeviceImageRead: Invalid byte count %d, must be even\n", len);
			return FALSE;
	}

	switch (imageType) {
	case WLM_TYPE_SROM:
		if (len < 2*SROM4_WORDS) {
			printf("wlmDeviceImageRead: Buffer not large enough!\n");
			return FALSE;
		}

		srt = (srom_rw_t *)buf;
		srt->byteoff = 0;
		srt->nbytes = htod32(2 * SROM4_WORDS);
		/* strlen("cisdump ") = 9 */
		if (wlu_get(irh, WLC_GET_SROM, buf, 9 + (len < SROM_MAX ? len : SROM_MAX)) < 0) {
			printf("wlmDeviceImageRead: %s\n", wlmLastError());
			return FALSE;
		}
		memcpy(byteStream, buf + 8, srt->nbytes);
		numRead = srt->nbytes;
		break;
	case WLM_TYPE_OTP:
		strcpy(buf, "cisdump");
		/* strlen("cisdump ") = 9 */
		if (wl_get(irh, WLC_GET_VAR, buf,
		   9 + sizeof(cis_rw_t) + (len < SROM_MAX ? len : SROM_MAX)) < 0) {
		    printf("wlmDeviceImageRead: %s\n", wlmLastError());
		    return FALSE;
		}

		cish = (cis_rw_t *)buf;
		cish->source = dtoh32(cish->source);
		cish->byteoff = dtoh32(cish->byteoff);
		cish->nbytes = dtoh32(cish->nbytes);

		if (len < cish->nbytes) {
			printf("wlmDeviceImageRead: Buffer not large enough!\n");
			return FALSE;
		}
		memcpy(byteStream, buf + sizeof(cis_rw_t), cish->nbytes);
		numRead = cish->nbytes;
		break;
	case WLM_TYPE_AUTO:
	  numRead = wlmDeviceImageRead(byteStream, len, WLM_TYPE_SROM);
		if (!numRead) {
			numRead = wlmDeviceImageRead(byteStream, len, WLM_TYPE_OTP);
		    printf("wlmDeviceImageRead: %s\n", wlmLastError());
		    return FALSE;
	    }
	    break;
	default:
		printf("wlmDeviceImageRead: Invalid image type!\n");
		return FALSE;
	}
	return numRead;
}

int wlmSecuritySet(WLM_AUTH_TYPE authType, WLM_AUTH_MODE authMode,
	WLM_ENCRYPTION encryption, const char *key)
{
	int length = 0;
	int wpa_auth;
	int sup_wpa;
	int primary_key = 0;
	wl_wsec_key_t wepKey[4];
	wsec_pmk_t psk;
	int wsec;

	if (encryption != WLM_ENCRYPT_NONE && key == 0) {
		printf("wlmSecuritySet: invalid key\n");
		return FALSE;
	}

	if (key) {
		length = strlen(key);
	}

	switch (encryption) {

	case WLM_ENCRYPT_NONE:
		wpa_auth = WPA_AUTH_DISABLED;
		sup_wpa = 0;
		break;

	case WLM_ENCRYPT_WEP: {
		int i;
		int len = length / 4;

		wpa_auth = WPA_AUTH_DISABLED;
		sup_wpa = 0;

		if (!(length == 40 || length == 104 || length == 128 || length == 256)) {
			printf("wlmSecuritySet: invalid WEP key length %d"
			"       - expect 40, 104, 128, or 256"
			" (i.e. 10, 26, 32, or 64 for each of 4 keys)\n", length);
			return FALSE;
		}

		/* convert hex key string to 4 binary keys */
		for (i = 0; i < 4; i++) {
			wl_wsec_key_t *k = &wepKey[i];
			const char *data = &key[i * len];
			unsigned int j;

			memset(k, 0, sizeof(*k));
			k->index = i;
			k->len = len / 2;

			for (j = 0; j < k->len; j++) {
				char hex[] = "XX";
				char *end = NULL;
				strncpy(hex, &data[j * 2], 2);
				k->data[j] = (char)strtoul(hex, &end, 16);
				if (*end != 0) {
					printf("wlmSecuritySet: invalid WEP key"
					"       - expect hex values\n");
					return FALSE;
				}
			}

			switch (k->len) {
			case 5:
				k->algo = CRYPTO_ALGO_WEP1;
				break;
			case 13:
				k->algo = CRYPTO_ALGO_WEP128;
				break;
			case 16:
				k->algo = CRYPTO_ALGO_AES_CCM;
				break;
			case 32:
				k->algo = CRYPTO_ALGO_TKIP;
				break;
			default:
				/* invalid */
				return FALSE;
			}

			k->flags |= WL_PRIMARY_KEY;
		}

		break;
	}

	case WLM_ENCRYPT_TKIP:
	case WLM_ENCRYPT_AES: {

		if (authMode != WLM_WPA_AUTH_PSK && authMode != WLM_WPA2_AUTH_PSK) {
			printf("wlmSecuritySet: authentication mode must be WPA PSK or WPA2 PSK\n");
			return FALSE;
		}

		wpa_auth = authMode;
		sup_wpa = 1;

		if (length < WSEC_MIN_PSK_LEN || length > WSEC_MAX_PSK_LEN) {
			printf("wlmSecuritySet: passphrase must be between %d and %d characters\n",
			WSEC_MIN_PSK_LEN, WSEC_MAX_PSK_LEN);
			return FALSE;
		}

		psk.key_len = length;
		psk.flags = WSEC_PASSPHRASE;
		memcpy(psk.key, key, length);

		break;
	}

	case WLM_ENCRYPT_WSEC:
	case WLM_ENCRYPT_FIPS:
	default:
		printf("wlmSecuritySet: encryption not supported\n");
		return FALSE;
	}

	if (wlu_iovar_setint(irh, "auth", authType)) {
		printf("wlmSecuritySet: %s\n", wlmLastError());
		return FALSE;
	}

	if (wlu_iovar_setint(irh, "wpa_auth", wpa_auth)) {
		printf("wlmSecuritySet: %s\n", wlmLastError());
		return FALSE;
	}

	/* Fix me: need firmware support for sup_wpa */
	/*
	if (wlu_iovar_setint(irh, "sup_wpa", sup_wpa)) {
		printf("wlmSecuritySet: %s\n", wlmLastError());
		return FALSE;
	}
	*/

	if (encryption == WLM_ENCRYPT_WEP) {
		int i;
		for (i = 0; i < 4; i++) {
			wl_wsec_key_t *k = &wepKey[i];
			k->index = htod32(k->index);
			k->len = htod32(k->len);
			k->algo = htod32(k->algo);
			k->flags = htod32(k->flags);

			if (wlu_set(irh, WLC_SET_KEY, k, sizeof(*k))) {
				printf("wlmSecuritySet: %s\n", wlmLastError());
				return FALSE;
			}
		}

		primary_key = htod32(primary_key);
		if (wlu_set(irh, WLC_SET_KEY_PRIMARY, &primary_key, sizeof(primary_key)) < 0) {
			printf("wlmSecuritySet: %s\n", wlmLastError());
			return FALSE;
		}
	}
	else if (encryption == WLM_ENCRYPT_TKIP || encryption == WLM_ENCRYPT_AES) {
		psk.key_len = htod16(psk.key_len);
		psk.flags = htod16(psk.flags);

		if (wlu_set(irh, WLC_SET_WSEC_PMK, &psk, sizeof(psk))) {
			printf("wlmSecuritySet: %s\n", wlmLastError());
			return FALSE;
		}
	}

	wsec = htod32(encryption);
	/*
	if (wlu_set(irh, WLC_SET_WSEC, &wsec, sizeof(wsec)) < 0) {
		printf("wlmSecuritySet: %s\n", wlmLastError());
		return FALSE;
	}
	*/
	if (wlu_iovar_setint(irh, "wsec", wsec) < 0) {
		printf("wlmSecuritySet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmJoinNetwork(const char* ssid, WLM_JOIN_MODE mode)
{
	wlc_ssid_t wlcSsid;
	int infra = htod32(mode);
	if (wlu_set(irh, WLC_SET_INFRA, &infra, sizeof(int)) < 0) {
	    printf("wlmJoinNetwork: %s\n", wlmLastError());
	    return FALSE;
	}

	wlcSsid.SSID_len = htod32(strlen(ssid));
	memcpy(wlcSsid.SSID, ssid, wlcSsid.SSID_len);

	if (wlu_set(irh, WLC_SET_SSID, &wlcSsid, sizeof(wlc_ssid_t)) < 0) {
		printf("wlmJoinNetwork: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmDisassociateNetwork(void)
{
	if (wlu_set(irh, WLC_DISASSOC, NULL, 0) < 0) {
		printf("wlmDisassociateNetwork: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int wlmSsidGet(char *ssid, int length)
{
	wlc_ssid_t wlc_ssid;

	if (length < SSID_FMT_BUF_LEN) {
		printf("wlmSsidGet: Ssid buffer too short - %d bytes at least\n",
		SSID_FMT_BUF_LEN);
		return FALSE;
	}

	/* query for 'ssid' */
	if (wlu_get(irh, WLC_GET_SSID, &wlc_ssid, sizeof(wlc_ssid_t))) {
		printf("wlmSsidGet: %s\n", wlmLastError());
		return FALSE;
	}

	wl_format_ssid(ssid, wlc_ssid.SSID, dtoh32(wlc_ssid.SSID_len));

	return TRUE;
}

int wlmBssidGet(char *bssid, int length)
{
	struct ether_addr ea;

	if (length != ETHER_ADDR_LEN) {
		printf("wlmBssiGet: bssid requires %d bytes", ETHER_ADDR_LEN);
		return FALSE;
	}

	if (wlu_get(irh, WLC_GET_BSSID, &ea, ETHER_ADDR_LEN) == 0) {
		/* associated - format and return bssid */
		strncpy(bssid, wl_ether_etoa(&ea), length);
	}
	else {
		/* not associated - return empty string */
		memset(bssid, 0, length);
	}

	return TRUE;
}


int wlmGlacialTimerSet(int val)
{
	if (wlu_iovar_setint(irh, "glacial_timer", val)) {
		printf("wlmGlacialTimerSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmFastTimerSet(int val)
{
	if (wlu_iovar_setint(irh, "fast_timer", val)) {
		printf("wlmFastTimerSet: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int wlmSlowTimerSet(int val)
{
	if (wlu_iovar_setint(irh, "slow_timer", val)) {
		printf("wlmSlowTimerSet: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}


int wlmScanSuppress(int on)
{
	int val;
	if (on)
		val = 1;
	else
		val = 0;

	if (wlu_set(irh, WLC_SET_SCANSUPPRESS, &val, sizeof(int))) {
		printf("wlmScanSuppress: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int wlmCountryCodeSet(const char * country_name)
{
	wl_country_t cspec;
	int err;

	memset(&cspec, 0, sizeof(cspec));
	cspec.rev = -1;

	/* arg matched a country name */
	memcpy(cspec.country_abbrev, country_name, WLC_CNTRY_BUF_SZ);

    /* parse a country spec, e.g. "US/1", or a country code.
     * cspec.rev will be -1 if not specified.
     */
    err = wl_parse_country_spec(country_name, cspec.ccode, &cspec.rev);

	/* first try the country iovar */
	if (cspec.rev == -1 && cspec.ccode[0] == '\0')
		err = wlu_iovar_set(irh, "country", &cspec, WLC_CNTRY_BUF_SZ);
	else
		err = wlu_iovar_set(irh, "country", &cspec, sizeof(cspec));

	if (err == 0)
		return TRUE;
	return FALSE;
}

int wlmFullCal(void)
{
	return TRUE;
}

int wlmIoctlGet(int cmd, void *buf, int len)
{
	if (wlu_get(irh, cmd, buf, len)) {
		printf("wlmIoctlGet: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int wlmIoctlSet(int cmd, void *buf, int len)
{
	if (wlu_set(irh, cmd, buf, len)) {
		printf("wlmIoctlSet: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int wlmIovarGet(const char *iovar, void *buf, int len)
{
	if (wlu_iovar_get(irh, iovar, buf, len)) {
		printf("wlmIovarGet: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int wlmIovarSet(const char *iovar, void *buf, int len)
{
	if (wlu_iovar_set(irh, iovar, buf, len)) {
		printf("wlmIovarSet: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int wlmIovarIntegerGet(const char *iovar, int *val)
{
	if (wlu_iovar_getint(irh, iovar, val)) {
		printf("wlmIovarIntegerGet: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int wlmIovarIntegerSet(const char *iovar, int val)
{
	if (wlu_iovar_setint(irh, iovar, val)) {
		printf("wlmIovarIntegerSet: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int wlmIovarBufferGet(const char *iovar, void *param, int param_len, void **bufptr)
{
	if (wlu_var_getbuf(irh, iovar, param, param_len, bufptr)) {
		printf("wlmIovarBufferGet: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int wlmIovarBufferSet(const char *iovar, void *param, int param_len)
{
	if (wlu_var_setbuf(irh, iovar, param, param_len)) {
		printf("wlmIovarBufferSet: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int wlmCga5gOffsetsSet(char* values, int len)
{
	if (len != CGA_5G_OFFSETS_LEN) {
		printf("wlmCga5gOffsetsSet() requires a %d-value array as a parameter\n",
		      CGA_5G_OFFSETS_LEN);
		return FALSE;
	}

	if ((wlu_var_setbuf(irh, "sslpnphy_cga_5g", values,
	                    CGA_5G_OFFSETS_LEN * sizeof(int8))) < 0) {
		printf("wlmCga5gOffsetsSet(): Error setting offset values (%s)\n",  wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmCga5gOffsetsGet(char* buf, int len)
{
	if (len != CGA_5G_OFFSETS_LEN) {
		printf("wlmCga5gOffsetsGet() requires a %d-value array as a parameter\n",
		       CGA_5G_OFFSETS_LEN);
		return FALSE;
	}

	if ((wlu_iovar_get(irh, "sslpnphy_cga_5g", buf, CGA_5G_OFFSETS_LEN * sizeof(int8))) < 0) {
		printf("wlmCga5gOffsetsGet(): Error setting offset values (%s)\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int wlmCga2gOffsetsSet(char* values, int len)
{
	if (len != CGA_2G_OFFSETS_LEN) {
		printf("wlmCga2gOffsetsSet(): requires a %d-value array as a parameter\n",
		       CGA_2G_OFFSETS_LEN);
		return FALSE;
	}

	if ((wlu_var_setbuf(irh, "sslpnphy_cga_2g", values,
	                    CGA_2G_OFFSETS_LEN * sizeof(int8))) < 0) {
		printf("wlmCga2gOffsetsSet(): Error setting offset values (%s)\n", wlmLastError());
		return FALSE;
	}

	return TRUE;

}

int wlmCga2gOffsetsGet(char* buf, int len)
{
	if (len != CGA_2G_OFFSETS_LEN) {
		printf("wlmCga2gOffsetsGet(): requires a %d-value array as a parameter\n",
		       CGA_2G_OFFSETS_LEN);
		return FALSE;
	}

	if ((wlu_iovar_get(irh, "sslpnphy_cga_2g", buf, CGA_2G_OFFSETS_LEN * sizeof(int8))) < 0) {
		printf("wlmCga2gOffsetsGet(): Error setting offset values (%s)\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmNvramVarGet(const char *varname, char *buffer, int len)
{
	strcpy(buffer, varname);
	if (wlu_get(irh, WLC_NVRAM_GET, buffer, len) < 0) {
		printf("wlmNvramVarGet: Error getting \"%s\" value(%s)\n",
		       varname, wlmLastError());
		return FALSE;
	}
	return TRUE;
}
#ifdef SERDOWNLOAD
/* parsed values are the filename of the firmware and the args file */
int wlmDhdDownload(const char* firmware, const char* vars)
{
	char *args[3] = {0};
	bool ret;

	if (!firmware) {
		printf("Missing firmware path/filename\n");
		return FALSE;
	}

	if (!vars) {
		printf("Missing vars file\n");
		return FALSE;
	}

	args[0] = malloc(sizeof(char*) * (strlen(" ")+1));
	args[1] = malloc(sizeof(char*) * (strlen(firmware)+1));
	args[2] = malloc(sizeof(char*) * (strlen(vars)+1));

	if (args[0] == NULL || args[1] == NULL || args[2] == NULL) {
		ret = FALSE;
		printf("Malloc failures, aborting download\n");
		goto cleanup;
	}

	memset(args[0], 0, sizeof(char*) * (strlen(" ")+1));
	memset(args[1], 0, sizeof(char*) * (strlen(firmware)+1));
	memset(args[2], 0, sizeof(char*) * (strlen(vars)+1));

	strncpy(args[0], " ", strlen(" "));
	strncpy(args[1], firmware, strlen(firmware));
	strncpy(args[2], vars, strlen(vars));

	printf("downloading firmware...\n");

	if (dhd_download(irh, 0, (char **) args)) {
		printf("wlmDhdDownload for firmware %s and vars file %s failed\n", firmware, vars);
		ret = FALSE;
	} else {
		printf("firmware download complete\n");
		ret = TRUE;
	}

cleanup:
	    if (args[0] != NULL)
		    free(args[0]);
	    if (args[1] != NULL)
		    free(args[1]);
	    if (args[2] != NULL)
		    free(args[2]);

	    return ret;

}

/* only parsed argv value is the chip string */

int wlmDhdInit(const char *chip)
{
	bool ret;
	char *args[2] = {0};

	args[0] = malloc(sizeof(char*) * (strlen(" ")+1));
	args[1] = malloc(sizeof(char*) * (strlen(chip)+1));

	if (args[0] == NULL || args[1] == NULL) {
		ret = FALSE;
		printf("wlmDhdInit: Malloc failures, aborting download\n");
		goto cleanup;
	}

	memset(args[0], 0, sizeof(char*) * (strlen(" ")+1));
	memset(args[1], 0, sizeof(char*) * (strlen(" ")+1));

	strncpy(args[0], " ", strlen(" "));
	strncpy(args[1], chip, strlen(chip));

	printf("wlmDhdInit: initializing firmware download...\n");

	if (dhd_init(irh, 0, (char **)args)) {
		printf("wlmDhdInit for chip %s failed\n", chip);
		ret = FALSE;
	}
	    ret = TRUE;

cleanup:
	    if (args[0] != NULL)
		    free(args[0]);
	    if (args[1] != NULL)
		    free(args[1]);

	    return ret;
}
#endif /* SERDOWNLOAD */

int wlmRadioOn(void)
{
	int val;

	/* val = WL_RADIO_SW_DISABLE << 16; */
	val = (1<<0) << 16;

	if (wlu_set(irh, WLC_SET_RADIO, &val, sizeof(int)) < 0) {
		printf("wlmRadioOn: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmRadioOff(void)
{
	int val;

	/* val = WL_RADIO_SW_DISABLE << 16 | WL_RADIO_SW_DISABLE; */
	val = (1<<0) << 16 | (1<<0);

	if (wlu_set(irh, WLC_SET_RADIO, &val, sizeof(int)) < 0) {
		 printf("wlmRadioOff: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmPMmode(int val)
{
	if (val < 0 || val > 2) {
	        printf("wlmPMmode: setting for PM mode out of range [0,2].\n");
		/* 0: CAM constant awake mode */
		/* 1: PS (Power save) mode */
		/* 2: Fast PS mode */
	}

	if (wlu_set(irh, WLC_SET_PM, &val, sizeof(int)) < 0) {
		printf("wlmPMmode: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmRoamingOn(void)
{
	if (wlu_iovar_setint(irh, "roam_off", 0) < 0) {
		printf("wlmRoamingOn: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmRoamingOff(void)
{
	if (wlu_iovar_setint(irh, "roam_off", 1) < 0) {
		printf("wlmRoamingOff: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmRoamTriggerLevelGet(int *val, WLM_BAND band)
{
	struct {
		int val;
		int band;
	} x;

	x.band = htod32(band);
	x.val = -1;

	if (wlu_get(irh, WLC_GET_ROAM_TRIGGER, &x, sizeof(x)) < 0) {
		printf("wlmRoamTriggerLevelGet: %s\n", wlmLastError());
		return FALSE;
	}

	*val = htod32(x.val);

	return TRUE;
}

int wlmRoamTriggerLevelSet(int val, WLM_BAND band)
{
	struct {
		int val;
		int band;
	} x;

	x.band = htod32(band);
	x.val = htod32(val);

	if (wlu_set(irh, WLC_SET_ROAM_TRIGGER, &x, sizeof(x)) < 0) {
		printf("wlmRoamTriggerLevelSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmFrameBurstOn(void)
{
	int val = 1;

	val = htod32(val);
	if (wlu_set(irh, WLC_SET_FAKEFRAG, &val, sizeof(int)) < 0) {
		printf("wlmFrameBurstOn: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmFrameBurstOff(void)
{
	int val = 0;

	val = htod32(val);
	if (wlu_set(irh, WLC_SET_FAKEFRAG, &val, sizeof(int)) < 0) {
		printf("wlmFrameBurstOff: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}


int wlmBeaconIntervalSet(int val)
{
	val = htod32(val);
	if (wlu_set(irh, WLC_SET_BCNPRD, &val, sizeof(int)) < 0) {
		printf("wlmBeaconIntervalSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmAMPDUModeSet(int val)
{
	val = htod32(val);
	if (wlu_iovar_setint(irh, "ampdu", val) < 0) {
		printf("wlmAMPDUModeSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmMIMOBandwidthCapabilitySet(int val)
{
	val = htod32(val);

	if (wlu_iovar_setint(irh, "mimo_bw_cap", val) < 0) {
		printf("wlmMIMOBandwidthCapabilitySet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmInterferenceSet(int val)
{
	val = htod32(val);

	if (val < 0 || val > 4) {
		printf("wlmInterferenceSet: interference setting out of range [0, 4]\n");
		return FALSE;
	}

	if (wlu_set(irh, WLC_SET_INTERFERENCE_MODE, &val, sizeof(int)) < 0) {
		printf("wlmInterferenceSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmInterferenceOverrideSet(int val)
{
	val = htod32(val);

	if (val < 0 || val > 4) {
		printf("wlmInterferenceOverrideSet: interference setting out of range [0, 4]\n");
		return FALSE;
	}

	if (wlu_set(irh, WLC_SET_INTERFERENCE_OVERRIDE_MODE, &val, sizeof(int)) < 0) {
		printf("wlmInterferenceOverrideSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmTransmitBandwidthSet(int val)
{
	val = htod32(val);

	if (wlu_iovar_setint(irh, "mimo_txbw", val) < 0) {
		printf("wlmTransmitBandwidthSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmShortGuardIntervalSet(int val)
{
	val = htod32(val);

	if (wlu_iovar_setint(irh, "sgi_tx", val) < 0) {
		printf("wlmShortGuardIntervalSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmShortGuardIntervalRxSet(int val)
{
	val = htod32(val);

	if (wlu_iovar_setint(irh, "sgi_rx", val) < 0) {
		printf("wlmShortGuardIntervalRxSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmObssCoexSet(int val)
{
	val = htod32(val);

	if (wlu_iovar_setint(irh, "obss_coex", val) < 0) {
		printf("wlmObssCoexSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmPHYPeriodicalCalSet(void)
{
	if (wlu_iovar_setint(irh, "phy_percal", 0) < 0) {
		printf("wlmPHYPeriodicalCalSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmPHYForceCalSet(void)
{
	if (wlu_iovar_setint(irh, "phy_forcecal", 0) < 0) {
		printf("wlmPHYForceCalSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmPHYForceCalNoiseSet(void)
{
	/* does only noise cal */
	if (wlu_iovar_setint(irh, "phy_forcecal_noise", 1) < 0) {
		printf("wlmPHYForceCalNoiseSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}
int wlmPHYScramblerUpdateDisable(void)
{
	if (wlu_iovar_setint(irh, "phy_scraminit", 0x7f) < 0) {
		printf("wlmPHYScramblerUpdateDisable: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmPHYScramblerUpdateEnable(void)
{
	if (wlu_iovar_setint(irh, "phy_scraminit", -1) < 0) {
		printf("wlmPHYScramblerUpdateEnable: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmPHYWatchdogSet(int val)
{
	val = htod32(val);

	if (wlu_iovar_setint(irh, "phy_watchdog", val) < 0) {
	        printf("wlmPHYWatchdogSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmTemperatureSensorDisable(void)
{
	int val = 1; /* 0 = temp sensor enabled; 1 = temp sensor disabled */
	if (wlu_iovar_setint(irh, "tempsense_disable", val) < 0) {
		printf("wlmTempSensorDisable %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmTemperatureSensorEnable(void)
{
	int val = 0; /* 0 = temp sensor enabled; 1 = temp sensor disabled */
	if (wlu_iovar_setint(irh, "tempsense_disable", val) < 0) {
		printf("wlmTempSensorEnable %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmTransmitCoreSet(int core, int streams)
{
	uint32 coremask[2] = {0, 0};
	uint8 mcs_mask[4] = {0, 0, 0, 0}; /* pre-initialize # of streams {core:4 | stream:4} */
	uint8 idx;
	uint8 cck_mask = 0;
	uint8 ofdm_mask = 0;

	core = htod32(core);

	core = core & 0x0f;
	cck_mask = ofdm_mask = core;  /* assume cck, ofdm and mcs core mask are the same */
	core = core << 4;
	if (core == 0) {
		printf("wlmTransmitCoreSet, %1d-stream core cannot be zero\n",  streams);
		return TRUE;
	}

	streams = (streams & 0x0f);
	if (streams > 4)  {
		printf("wlmTransmitCoreSet: Nsts > 4\n");
		return FALSE;
	}

	idx = streams - 1;
	mcs_mask[idx] = (uint8)(core|streams);

	coremask[0] |= mcs_mask[0] << 0;
	coremask[0] |= mcs_mask[1] << 8;
	coremask[0] |= mcs_mask[2] << 16;
	coremask[0] |= mcs_mask[3] << 24;

	coremask[1] |= cck_mask;

	coremask[1] |= ofdm_mask << 8;

	if (wlu_var_setbuf(irh, "txcore", coremask, sizeof(uint32) * 2) < 0) {
		printf("wlmTransmitCoreSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmPhyTempSenseGet(int *val)
{
	if (wlu_iovar_getint(irh, "phy_tempsense", val) < 0) {
		printf("wlmPhyTempSenseGet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmOtpFabidGet(int *val)
{
	if (wlu_iovar_getint(irh, "otp_fabid", val)) {
		printf("wlmOtpFabidGet: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}


/* Defined located in src/wl/bcmwifi/src/bcmwifi_channels.c */
/* 40MHz channels in 5GHz band */
static const uint8 wf_5g_40m_chans[] =
	{38, 46, 54, 62, 102, 110, 118, 126, 134, 142, 151, 159};
#define WF_NUM_5G_40M_CHANS \
	(sizeof(wf_5g_40m_chans)/sizeof(uint8))

/* 80MHz channels in 5GHz band */
static const uint8 wf_5g_80m_chans[] =
	{42, 58, 106, 122, 138, 155};
#define WF_NUM_5G_80M_CHANS \
	(sizeof(wf_5g_80m_chans)/sizeof(uint8))

/* 160MHz channels in 5GHz band */
static const uint8 wf_5g_160m_chans[] =
	{50, 114};
#define WF_NUM_5G_160M_CHANS \
	(sizeof(wf_5g_160m_chans)/sizeof(uint8))

static uint8
center_chan_to_edge(uint bw)
{
	/* edge channels separated by BW - 10MHz on each side
	 * delta from cf to edge is half of that,
	 * MHz to channel num conversion is 5MHz/channel
	 */
	return (uint8)(((bw - 20) / 2) / 5);
}

/* return channel number of the low edge of the band
 * given the center channel and BW
 */
static uint8
channel_low_edge(uint center_ch, uint bw)
{
	return (uint8)(center_ch - center_chan_to_edge(bw));
}

/* return side band number given center channel and control channel
 * return -1 on error
 */
static int
channel_to_sb(uint center_ch, uint ctl_ch, uint bw)
{
	uint lowest = channel_low_edge(center_ch, bw);
	uint sb;

	if ((ctl_ch - lowest) % 4) {
		/* bad ctl channel, not mult 4 */
		return -1;
	}

	sb = ((ctl_ch - lowest) / 4);

	/* sb must be a index to a 20MHz channel in range */
	if (sb >= (bw / 20)) {
		/* ctl_ch must have been too high for the center_ch */
		return -1;
	}

	return sb;
}

int
wlmChannelSpecSet(int channel, int bandwidth, int sideband)
{
	chanspec_t chanspec = 0;
	uint chspec_ch, chspec_band, chspec_bw, chspec_sb;
	uint ctl_ch;

	/* set current band status */
	if ((channel <= 14) && (curBand != WLM_BAND_2G)) {
		curBand = WLM_BAND_2G;
	} else if ((channel > 14) && (curBand != WLM_BAND_5G)) {
		curBand = WLM_BAND_5G;
	}

	/* Set control channel */
	if (channel > MAXCHANNEL) {
		printf("wlmChannelSpecSet: %d ivalid channel exceed %d\n",
		       channel, MAXCHANNEL);
		return FALSE;
	}

	ctl_ch = channel;

	/* Set band */
	if (channel > CH_MAX_2G_CHANNEL)
		chspec_band = WL_CHANSPEC_BAND_5G;
	else
		chspec_band = WL_CHANSPEC_BAND_2G;

	/* Set bandwidth */
	if (bandwidth == 10)
		chspec_bw = WL_CHANSPEC_BW_10;
	else if (bandwidth == 20)
		chspec_bw = WL_CHANSPEC_BW_20;
	else if (bandwidth == 40)
		chspec_bw = WL_CHANSPEC_BW_40;
	else if (bandwidth == 80)
		chspec_bw = WL_CHANSPEC_BW_80;
	else if (bandwidth == 160)
		chspec_bw = WL_CHANSPEC_BW_160;
	else if (bandwidth == 8080)
		chspec_bw = WL_CHANSPEC_BW_8080;
	else {
		printf("wlmChannelSpecSet: %d is invalid channel bandwith,"
		       "must be 10, 20, 40, 80, 160 or 8080\n", bandwidth);
		return FALSE;
	}

	/* Set side band  */
	/* Sideband 1 or -1, gurantee 40Mhz channel */
	if ((bandwidth == 20) || (bandwidth == 10)) {
		chspec_sb = 0;
		chspec_ch = ctl_ch;
	}
	else if ((bandwidth == 40) && (sideband == -1)) {
		chspec_sb = WL_CHANSPEC_CTL_SB_LLL;
		chspec_ch = UPPER_20_SB(ctl_ch);
	}
	else if ((bandwidth == 40) && (sideband == 1)) {
		chspec_sb = WL_CHANSPEC_CTL_SB_LLU;
		chspec_ch = LOWER_20_SB(ctl_ch);
	}
	else if ((bandwidth == 80) || (bandwidth == 160)) {
		/* figure out ctl sideband based on ctl channel and bandwidth */
		const uint8 *center_ch = NULL;
		int num_ch = 0;
		int sb = -1;
		int i;

		if (chspec_bw == WL_CHANSPEC_BW_80) {
			center_ch = wf_5g_80m_chans;
			num_ch = WF_NUM_5G_80M_CHANS;
		}
		else if (chspec_bw == WL_CHANSPEC_BW_160) {
			center_ch = wf_5g_160m_chans;
			num_ch = WF_NUM_5G_160M_CHANS;
		} else {
			printf("wlmChannelSpecSet: unsupported channel bandwidth %d\n", bandwidth);
			return FALSE;
		}

		for (i = 0; i < num_ch; i++) {
			sb = channel_to_sb(center_ch[i], channel, bandwidth);
			if (sb > 0) {
				chspec_ch = center_ch[i];
				chspec_sb = sb << WL_CHANSPEC_CTL_SB_SHIFT;
				break;
			}
		}

		if (sb < 0) {
			printf("wlmChannelSpecSet:"
			       "Can't find valid side band for channel %d\n", channel);
			return FALSE;
		}
	}
	else {
		/* Otherwise, 80+80 */
		printf("wlmChannelSpecSet: Wrong bandwdith."
		       "For 80+80 chanspec, call wlm80Plus80ChannelSpecSet() instead.\n");
		return FALSE;
	}

	/* Set channel */
	chanspec = chspec_band | chspec_ch |chspec_bw | chspec_sb;

	if (ioctl_version == 1) {
		chanspec = wl_chspec_to_legacy(chanspec);
		if (chanspec == INVCHANSPEC) {
			printf("wlmChannelSpecSet: Invalid chanspec 0x%4x\n", chanspec);
			return FALSE;
		}
	}

	if (wlu_iovar_setint(irh, "chanspec", (int) chanspec) < 0) {
		printf("wlmChannelSpecSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

static int
channel_80mhz_to_id(uint ch)
{
	uint i;
	for (i = 0; i < WF_NUM_5G_80M_CHANS; i ++) {
		if (ch == wf_5g_80m_chans[i])
			return i;
	}

	return -1;
}


int
wlm80Plus80ChannelSpecSet(int ch1, int ch2)
{
	chanspec_t chanspec = 0;
	uint chspec_ch, chspec_band, chspec_bw, chspec_sb, ctl_ch;
	int ch1_id = 0, ch2_id = 0;
	int bw, sb;

	if ((ch1 < ch2) && (ch1 > CH_MAX_2G_CHANNEL))
		chspec_band = WL_CHANSPEC_BAND_5G;
	else {
		printf("wlm80Plus80ChannelSpecSet: Invalid channels. "
		       "Both channels must be on 5G band "
		       "and first channel should be lower than second channel\n");
		return FALSE;
	}

	ch1_id = channel_80mhz_to_id(ch1);
	ch2_id = channel_80mhz_to_id(ch2);
	ctl_ch = ch1; /* first channel is control channel? */

	/* validate channels */
	if (ch1 >= ch2 || ch1_id < 0 || ch2_id < 0) {
		printf("wlm80Plus80ChannelSpecSet: Invalid channels.\n");
		return FALSE;
	}

	/* combined channel in chspec */
	chspec_ch = (((uint16)ch1_id << WL_CHANSPEC_CHAN1_SHIFT) |
	    ((uint16)ch2_id << WL_CHANSPEC_CHAN2_SHIFT));

	/* set channel bandwidth */
	bw = 80;
	chspec_bw = WL_CHANSPEC_BW_8080;

	/* does the primary channel fit with the 1st 80MHz channel ? */
	sb = channel_to_sb(ch1, ctl_ch, bw);
	if (sb < 0) {
		/* no, so does the primary channel fit with the 2nd 80MHz channel ? */
		sb = channel_to_sb(ch2, ctl_ch, bw);
		if (sb < 0) {
			/* no match for ctl_ch to either 80MHz center channel */
			return 0;
		}
		/* sb index is 0-3 for the low 80MHz channel, and 4-7 for
		 * the high 80MHz channel. Add 4 to to shift to high set.
		 */
		sb += 4;
	}

	chspec_sb = sb << WL_CHANSPEC_CTL_SB_SHIFT;
	chanspec = chspec_band | chspec_ch |chspec_bw | chspec_sb;

	if (wlu_iovar_setint(irh, "chanspec", (int) chanspec) < 0) {
		printf("wlmChannelSpecSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmRtsThresholdOverride(int val)
{
	val = htod32(val);

	if (wlu_iovar_setint(irh, "rtsthresh", val) < 0) {
	        printf("wlmRtsThresholdOverride: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}


int wlmSTBCTxSet(int val)
{
	val = htod32(val);

	if (wlu_iovar_setint(irh, "stbc_tx", val) < 0) {
	        printf("wlmSTBCTxSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}


int wlmSTBCRxSet(int val)
{
	val = htod32(val);

	if (wlu_iovar_setint(irh, "stbc_rx", val) < 0) {
	        printf("wlmSTBCRxSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}


int wlmTxChainSet(int val)
{
	val = htod32(val);

	if (wlu_iovar_setint(irh, "txchain", val) < 0) {
	        printf("wlmTxChainSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmRxChainSet(int val)
{
	val = htod32(val);

	if (wlu_iovar_setint(irh, "rxchain", val) < 0) {
	        printf("wlmRxChainSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int wlmRxIQEstGet(float *val, int sampleCount, int ant)
{
	uint32 rxiq;
	int sample_count = sampleCount;  /* [0, 16], default: maximum 15 sample counts */
	int antenna = ant ;       /* [0, 3], default: antenna 0 */
	uint8 resolution = 1;     /* resolution default to 0.25dB */
	uint8 lpf_hpc = 1;
	uint8 gain_correct = 0;
	uint8 elna = 0;
	float x, y;

	/* DEFAULT:
	 * gain_correct = 0 (disable gain correction),
	 * lpf_hpc = 1 (sets lpf hpc to lowest value),
	 * elna (shared LSB with lpf_hpc) = 1
	 * resolution = 0 (coarse),
	 * samples = 1024 (2^10) and antenna = 3
	 * index = 0x78
	 */
	/* XXX Currently the lpf_hpc override option applies only for HTPHY */
	/* BitMap:
	   7:0 Antenna for all phy's except lcn/lcn40
	   6:0 Antenna for Lcn40
	   7   Gain Index Valid for Lcn/Lcn40
	   15:8 Samples
	   19:16 Resolution
	   23:20 lpf_hpc
	   20: Elna On/off for Lcn/Lcn40
	   23:21 Index[2:0] for Lcn/Lcn40
	   27:24 gain_correct
	   31:28 Index[6:3] for Lcn/Lcn40
	*/

	/* XXX Currently the lpf_hpc override option applies only for HTPHY */
	rxiq = (0xF << 28) | (gain_correct << 24) | (lpf_hpc << 20) |
		(resolution << 16) | (10 << 8) | 3;
	/* printf("wlmRxIQGet: initial rxiq = 0x%x\n.", rxiq); */

	if (gain_correct > 1) {
		printf("wlmRxIQEstGet: invalid gain-correction select [0|1]\n");
		return FALSE;
	} else {
		gain_correct = gain_correct & 0xf;
		rxiq = ((gain_correct << 24) | (rxiq & 0xf0ffffff));
	}
	/* printf("wlmRxIQGet: rxiq after gain_correct = 0x%x\n.", rxiq); */

	if (lpf_hpc > 1) {
		printf("wlmRXIQEstGet: invalid lpf-hpc override select [0|1].\n");
		return FALSE;
	} else {
		lpf_hpc = lpf_hpc & 0xf;
		rxiq = ((lpf_hpc << 20) | (rxiq & 0xff0fffff));
	}

	if (elna > 1) {
		printf("wlmRXIQEstGet: invalid elna on/off select [0|1].\n");
		return FALSE;
	} else {
		elna = elna & 0x1;
		rxiq = ((elna << 20) | (rxiq & 0xffefffff));
	}
	/* printf("wlmRxIQGet: rxiq after lpf-hpc = 0x%x\n.", rxiq); */

	if (resolution > 1) {
		printf("wlmRxIQEstGet: invalid resolution [0|1].\n");
		return FALSE;
	} else {
		resolution = resolution & 0xf;
		rxiq = ((resolution << 16) | (rxiq & 0xfff0ffff));
	}
	/* printf("wlmRxIQGet: rxiq after resolution = 0x%x\n.", rxiq); */

	if ((sample_count < 0) || (sample_count > 15)) {
		printf("wlmRxIQEstGet: SampleCount out of range of [0, 15].\n");
		return FALSE;
	} else {
		/* printf("wlmRxIQEstGet: SampleCount %d\n", sample_count); */
		rxiq = (((sample_count & 0xff) << 8) | (rxiq & 0xffff00ff));
	}
	/* printf("wlmRxIQGet: rxiq after sample count = 0x%x\n.", rxiq); */

	if ((antenna < 0) || (antenna > 3)) {
		printf("wlmRxIQEstGet: Antenna out of range of [0, 3].\n");
		return FALSE;
	} else {
		rxiq = ((rxiq & 0xffffff00) | (antenna & 0xff));
	}

	/*
	printf("wlmRxIQGet: rxiq after antenna = 0x%x\n.", rxiq);
	printf("wlmRxIQGet: rxiq = 0x%x before wlu_iovar_setint().\n", rxiq);
	*/
	if ((wlu_iovar_setint(irh, "phy_rxiqest", (int) rxiq) < 0)) {
		printf("wlmRxIQEstGet: %s\n", wlmLastError());
		return FALSE;
	}

	if ((wlu_iovar_getint(irh, "phy_rxiqest", (int*)&rxiq) < 0)) {
		printf("wlmRxIQEstGet: %s\n", wlmLastError());
		return FALSE;
	}

	/* printf("wlmRxIQGet: rxiq = 0x%x after wlu_iovar_getint\n.", rxiq); */
	if (resolution == 1) {
		/* fine resolutin power reporting (0.25dB resolution) */
		uint8 core;
		int16 tmp;
		/*
		  printf("wlmRxIQGet: inside resulution == 1 block, rxiq = 0x%x\n", rxiq);
		*/
		if (rxiq >> 20) {
			/* Three chains: return the core matches the antenna number */
			for (core = 0; core < 3; core++) {
				if (core == antenna) {
					tmp = (rxiq >>(10*core)) & 0x3ff;
					tmp = ((int16)(tmp << 6)) >>6;  /* sign extension */
					break;
				}
			}
		} else if (rxiq >> 10) {
			/* Two chains: return the core matches the antenna number */
			for (core = 0; core < 2; core++) {
				if (core == antenna) {
					tmp = (rxiq >>(10*core)) & 0x3ff;
					tmp = ((int16)(tmp << 6)) >> 6;  /* sign extension */
					break;
				}
			}
		} else {
			/* 1-chain specific */
			tmp = (rxiq & 0x3ff);
			tmp = ((int16)(tmp << 6)) >> 6; /* signed extension */
			/*
			  printf("wlmRxIQGet: single core, tmp 0x%x\n", tmp);
			  printf("wlmRxIQGet: tmp before processing 0x%x\n", tmp);
			*/
		}

		if (tmp < 0) {
			tmp = -1 * tmp;
		}
		x = (float) (tmp >> 2);
		y = (float) (tmp & 0x3);
		*val = (float)(x + (y * 25) /100) * (-1);
	} else {
		/* return the core matches the antenna number */
		*val = (float)((rxiq >> (8 *antenna)) & 0xff);
	}

	return TRUE;
}

int wlmRxIQEstExtGet(float *val, int sampleCount, int ant, int elna, int gi)
{
	uint32 rxiq = 0;
	int sample_count = sampleCount;  /* [0, 16], default: maximum 15 sample counts */
	int antenna = ant ;       /* [0, 3], default: antenna 0 */
	uint8 resolution = 1;     /* resolution default to 0.25dB */
	uint8 lpf_hpc = 1;
	uint8 dig_lpf = 1;
	uint8 gain_correct = 0;
	uint8 extra_gain_3dBsteps = 0;
	uint8 force_gain_type = 0;
	float x, y;

	/* DEFAULT:
	 * gain_correct = 0 (disable gain correction),
	 * lpf_hpc = 1 (sets lpf hpc to lowest value),
	 * dig_lpf = 1; (sets to ltrn_lpf mode)
	 * resolution = 0 (coarse),
	 * samples = 1024 (2^10) and antenna = 3
	 * force_gain_type = 0 (init gain mode)
	 */

	/* XXX Currently the lpf_hpc override option applies only for HTPHY */
	rxiq = (extra_gain_3dBsteps << 28) | (gain_correct << 24) | (dig_lpf << 22)
		| (lpf_hpc << 20) | (resolution << 16) | (10 << 8) | (force_gain_type << 4)
		| ant;
	/* printf("wlmRxIQGet: initial rxiq = 0x%x\n.", rxiq); */

	if (gain_correct > 1) {
		printf("wlmRxIQEstExtGet: invalid gain-correction select [0|1]\n");
		return FALSE;
	} else {
		gain_correct = gain_correct & 0xf;
		rxiq = ((gain_correct << 24) | (rxiq & 0xf0ffffff));
	}
	/* printf("wlmRxIQGet: rxiq after gain_correct = 0x%x\n.", rxiq); */

	if (lpf_hpc > 1) {
		printf("wlmRXIQEstExtGet: invalid lpf-hpc override select [0|1].\n");
		return FALSE;
	} else {
		lpf_hpc = lpf_hpc & 0xf;
		rxiq = ((lpf_hpc << 20) | (rxiq & 0xff0fffff));
	}
	/* printf("wlmRxIQGet: rxiq after lpf_hpc = 0x%x\n.", rxiq); */

	if (dig_lpf > 2) {
		printf("wlmRXIQEstExtGet: invalid dig_lpf override select [0|1].\n");
		return FALSE;
	} else {
		dig_lpf = dig_lpf & 0x3;
		rxiq = ((dig_lpf << 22) | (rxiq & 0xff3fffff));
	}
	/* printf("wlmRxIQGet: rxiq after dig_lpf = 0x%x\n.", rxiq); */

	if (resolution > 1) {
		printf("wlmRxIQEstExtGet: invalid resolution [0|1].\n");
		return FALSE;
	} else {
		resolution = resolution & 0xf;
		rxiq = ((resolution << 16) | (rxiq & 0xfff0ffff));
	}
	/* printf("wlmRxIQGet: rxiq after resolution = 0x%x\n.", rxiq); */

	if ((sample_count < 0) || (sample_count > 16)) {
		printf("wlmRxIQEstExtGet: SampleCount out of range of [0, 15].\n");
		return FALSE;
	} else {
		/* printf("wlmRxIQGet: sample_count %d\n.", sample_count); */
		rxiq = (((sample_count & 0xff) << 8) | (rxiq & 0xffff00ff));
	}
	/* printf("wlmRxIQGet: rxiq after sample count = 0x%x\n.", rxiq); */

	if ((antenna < 0) || (antenna > 3)) {
		printf("wlmRxIQEstExtGet: Antenna out of range of [0, 3].\n");
		return FALSE;
	} else {
		rxiq = ((rxiq & 0xfffffff0) | (antenna & 0xf));
	}
	/* printf("wlmRxIQGet: rxiq after antenna = 0x%x\n.", rxiq); */

	if ((extra_gain_3dBsteps > 24) || (extra_gain_3dBsteps % 3 != 0)) {
		printf("wlmRXIQESTExtGet: invalid extra INITgain, should be {0, 3, .., 21, 24}\n");
		return FALSE;
	} else {
		rxiq = ((((extra_gain_3dBsteps /3) & 0xf) << 28) | (rxiq & 0x0fffffff));
	}

	if (elna > 1) {
		printf("wlmRXIQEstExtGet: invalid elna on/off select [0|1].\n");
		return FALSE;
	} else {
		elna = elna & 0x1;
		rxiq = ((elna << 20) | (rxiq & 0xffefffff));
	}
	/* printf("wlmRxIQGet: rxiq after elna = 0x%x\n.", rxiq); */


	if ((gi != 0) && (gi != 1) && (gi != 4)) {
		printf("wlmRXIQEstExtGet: gain index option {0,1, 4}\n");
		return FALSE;
	} else {
		rxiq = ((rxiq & 0xffffff0f) | ((gi << 4) & 0xf0));
	}


	/* printf("wlmRxIQGet: rxiq before set gain index = 0x%x\n.", rxiq); */
	if ((wlu_iovar_setint(irh, "phy_rxiqest", (int) rxiq) < 0)) {
		printf("wlmRxIQEstExtGet: %s\n", wlmLastError());
		return FALSE;
	}

	if ((wlu_iovar_getint(irh, "phy_rxiqest", (int*)&rxiq) < 0)) {
		printf("wlmRxIQEstExtGet: %s\n", wlmLastError());
		return FALSE;
	}

	/* printf("wlmRxIQGet: after getint 0x%x\n.", rxiq); */

	if (resolution == 1) {
		/* fine resolutin power reporting (0.25dB resolution) */
		uint8 core;
		int16 tmp;

		/* printf("wlmRxIQGet: inside resulution == 1 block, rxiq = 0x%x\n", rxiq); */

		if (rxiq >> 20) {
			/* Three chains: return the core matches the antenna number */
			for (core = 0; core < 3; core++) {
				if (core == antenna) {
					tmp = (rxiq >>(10*core)) & 0x3ff;
					tmp = ((int16)(tmp << 6)) >> 6;  /* sign extension */
					break;
				}
			}
		} else if (rxiq >> 10) {
			/* Two chains: return the core matches the antenna number */
			for (core = 0; core < 2; core++) {
				if (core == antenna) {
					tmp = (rxiq >>(10*core)) & 0x3ff;
					tmp = ((int16)(tmp << 6)) >> 6;  /* sign extension */
					break;
				}
			}
		} else {
			/* 1-chain specific */
			tmp = (rxiq & 0x3ff);
			tmp = ((int16)(tmp << 6)) >> 6; /* signed extension */
		}

		/* printf("wlmRxIQGet: core %d, tmp 0x%x\n", core, tmp); */
		if (tmp < 0) {
			tmp = -1 * tmp;
		}
		x = (float) (tmp >> 2);
		y = (float) (tmp & 0x3);
		*val = (float)(x + (y * 25) /100) * (-1);
	} else {
		/* return the core matches the antenna number */
		*val = (float)((rxiq >> (8 *antenna)) & 0xff);
	}

	return TRUE;
}

int wlmRxIQEstACGet(float *val, int sampleCount, int ant, int extragain, int gi, int gain_correct)
{
	uint32 rxiq = 0;
	int sample_count = sampleCount;  /* [0, 16], default: maximum 15 sample counts */
	int antenna = ant ;       /* [0, 3], default: antenna 0 */
	uint8 resolution = 1;     /* resolution default to 0.25dB */
	uint8 lpf_hpc = 1;
	uint8 dig_lpf = 1;
	uint8 extra_gain_3dBsteps = 0;
	uint8 force_gain_type = 0;
	float x, y;

	/* DEFAULT:
	 * gain_correct = 0 (disable gain correction),
	 * lpf_hpc = 1 (sets lpf hpc to lowest value),
	 * dig_lpf = 1; (sets to ltrn_lpf mode)
	 * resolution = 0 (coarse),
	 * samples = 1024 (2^10) and antenna = 3
	 * force_gain_type = 0 (init gain mode)
	 */

	/* XXX Currently the lpf_hpc override option applies only for HTPHY */
	rxiq = (extra_gain_3dBsteps << 28) | (gain_correct << 24) | (dig_lpf << 22)
		| (lpf_hpc << 20) | (resolution << 16) | (10 << 8) | (force_gain_type << 4)
		| ant;
	/* printf("wlmRxIQGet: initial rxiq = 0x%x\n.", rxiq); */

	if (gain_correct > 8) {
		printf("wlmRxIQEstExtGet: invalid gain-correction select [0|1|2|3|4|7|8]\n");
		return FALSE;
	} else {
		gain_correct = gain_correct & 0xf;
		rxiq = ((gain_correct << 24) | (rxiq & 0xf0ffffff));
	}
	/* printf("wlmRxIQGet: rxiq after gain_correct = 0x%x\n.", rxiq); */

	if (lpf_hpc > 1) {
		printf("wlmRXIQEstExtGet: invalid lpf-hpc override select [0|1].\n");
		return FALSE;
	} else {
		lpf_hpc = lpf_hpc & 0xf;
		rxiq = ((lpf_hpc << 20) | (rxiq & 0xff0fffff));
	}
	/* printf("wlmRxIQGet: rxiq after lpf_hpc = 0x%x\n.", rxiq); */

	if (dig_lpf > 2) {
		printf("wlmRXIQEstExtGet: invalid dig_lpf override select [0|1].\n");
		return FALSE;
	} else {
		dig_lpf = dig_lpf & 0x3;
		rxiq = ((dig_lpf << 22) | (rxiq & 0xff3fffff));
	}
	/* printf("wlmRxIQGet: rxiq after dig_lpf = 0x%x\n.", rxiq); */

	if (resolution > 1) {
		printf("wlmRxIQEstExtGet: invalid resolution [0|1].\n");
		return FALSE;
	} else {
		resolution = resolution & 0xf;
		rxiq = ((resolution << 16) | (rxiq & 0xfff0ffff));
	}
	/* printf("wlmRxIQGet: rxiq after resolution = 0x%x\n.", rxiq); */

	if ((sample_count < 0) || (sample_count > 16)) {
		printf("wlmRxIQEstExtGet: SampleCount out of range of [0, 15].\n");
		return FALSE;
	} else {
		/* printf("wlmRxIQGet: sample_count %d\n.", sample_count); */
		rxiq = (((sample_count & 0xff) << 8) | (rxiq & 0xffff00ff));
	}
	/* printf("wlmRxIQGet: rxiq after sample count = 0x%x\n.", rxiq); */

	if ((antenna < 0) || (antenna > 3)) {
		printf("wlmRxIQEstExtGet: Antenna out of range of [0, 3].\n");
		return FALSE;
	} else {
		rxiq = ((rxiq & 0xffffff00) | (antenna & 0xff));
	}
	/* printf("wlmRxIQGet: rxiq after antenna = 0x%x\n.", rxiq); */

	if ((extragain < 0) || (extragain > 24) || (extragain % 3 != 0)) {
		printf("wlmRXIQEstExtGet: valid extra INITgain ={0,3, ..., 21,24}\n");
		return FALSE;
	} else {
		rxiq = ((((extragain/3) & 0xf) << 28) | (rxiq & 0x0fffffff));
	}
	/* printf("wlmRxIQGet: rxiq after elna = 0x%x\n.", rxiq); */

	if ((gi < 0) || (gi > 9))  {
		printf("wlmRXIQEstExtGet: gain index option {0,1, 4}\n");
		return FALSE;
	} else {
		rxiq = ((rxiq & 0xffffff0f) | ((gi << 4) & 0xf0));
	}

	/* printf("wlmRxIQGet: rxiq before set gain index = 0x%x\n.", rxiq); */
	if ((wlu_iovar_setint(irh, "phy_rxiqest", (int) rxiq) < 0)) {
		printf("wlmRxIQEstExtGet: %s\n", wlmLastError());
		return FALSE;
	}

	if ((wlu_iovar_getint(irh, "phy_rxiqest", (int*)&rxiq) < 0)) {
		printf("wlmRxIQEstExtGet: %s\n", wlmLastError());
		return FALSE;
	}

	/* printf("wlmRxIQGet: after getint 0x%x\n.", rxiq); */

	if (resolution == 1) {
		/* fine resolutin power reporting (0.25dB resolution) */
		uint8 core;
		int16 tmp;

		/* printf("wlmRxIQGet: inside resulution == 1 block, rxiq = 0x%x\n", rxiq); */

		if (rxiq >> 20) {
			/* Three chains: return the core matches the antenna number */
			for (core = 0; core < 3; core++) {
				if (core == antenna) {
					tmp = (rxiq >>(10*core)) & 0x3ff;
					tmp = ((int16)(tmp << 6)) >> 6;  /* sign extension */
					break;
				}
			}
		} else if (rxiq >> 10) {
			/* Two chains: return the core matches the antenna number */
			for (core = 0; core < 2; core++) {
				if (core == antenna) {
					tmp = (rxiq >>(10*core)) & 0x3ff;
					tmp = ((int16)(tmp << 6)) >> 6;  /* sign extension */
					break;
				}
			}
		} else {
			/* 1-chain specific */
			tmp = (rxiq & 0x3ff);
			tmp = ((int16)(tmp << 6)) >> 6; /* signed extension */
		}

		/* printf("wlmRxIQGet: core %d, tmp 0x%x\n", core, tmp); */
		if (tmp < 0) {
			tmp = -1 * tmp;
		}
		x = (float) (tmp >> 2);
		y = (float) (tmp & 0x3);
		*val = (float)(x + (y * 25) /100) * (-1);
	} else {
		/* return the core matches the antenna number */
		*val = (float)((rxiq >> (8 *antenna)) & 0xff);
	}

	return TRUE;
}

int wlmPHYTxPowerIndexGet(unsigned int *val, const char *chipid)
{
	uint32 power_index = (uint32)-1;
	uint32 txpwridx[4] = {0};
	int chip = atoi(chipid);

	switch (chip) {
	        case 4329:
		case 43291:
		  if (wlu_iovar_getint(irh, "sslpnphy_txpwrindex", (int*)&power_index) < 0) {
				printf("wlmPHYTxPowerIndexGet: %s\n", wlmLastError());
				return FALSE;
			}
			*val = power_index;
			break;
	        case 4325:
			if (wlu_iovar_getint(irh, "lppphy_txpwrindex", (int*)&power_index) < 0) {
				printf("wlmPHYTxPowerIndexGet: %s\n", wlmLastError());
				return FALSE;
			}
			*val = power_index;
			break;
	        default:
			if (wlu_iovar_getint(irh, "phy_txpwrindex", (int*)&txpwridx[0]) < 0) {
				printf("wlmPHYTxPowerIndexGet: %s\n", wlmLastError());
				return FALSE;
			}
			*val = dtoh32(txpwridx[0]);
			break;
	}

	return TRUE;
}

int wlmPHYTxPowerIndexSet(unsigned int val, const char *chipid)
{
	uint32 power_index;
	uint32 txpwridx[4] = {0};
	int chip = atoi(chipid);

	power_index = dtoh32(val);
	switch (chip) {
	        case 4329:
	        case 43291:
		  if (wlu_iovar_setint(irh, "sslpnphy_txpwrindex", (int)power_index) < 0) {
				printf("wlmPHYTxPowerIndexSet: %s\n", wlmLastError());
				return FALSE;
			}
			break;
	        case 4325:
		  if (wlu_iovar_setint(irh, "lppphy_txpwrindex", (int)power_index) < 0) {
				printf("wlmPHYTxPowerIndexSet: %s\n", wlmLastError());
				return FALSE;
			}
			break;
	        default:
			txpwridx[0] = (int)(power_index & 0xff);
			txpwridx[1] = (int)((power_index >> 8) & 0xff);
			txpwridx[2] = (int)((power_index >> 16) & 0xff);
			txpwridx[3] = (int)((power_index >> 24) & 0xff);

			if (wlu_var_setbuf(irh,
				"phy_txpwrindex", txpwridx, 4*sizeof(uint32)) < 0) {
				printf("wlmPHYTxPowerIndexSet: %s\n", wlmLastError());
				return FALSE;
			}
			break;
	}

	return TRUE;
}

int wlmRIFSEnable(int enable)
{
	int val, rifs;

	val = rifs = htod32(enable);
	if (rifs != 0 && rifs != 1) {
		printf("wlmRIFSEnable: Usage: input must be 0 or 1\n");
		return FALSE;
	}

	if (wlu_set(irh, WLC_SET_FAKEFRAG, &val, sizeof(int)) < 0) {
		printf("wlmRIFSEnable: %s\n", wlmLastError());
		return FALSE;
	}

	if (wlu_iovar_setint(irh, "rifs", (int)rifs) < 0) {
		printf("wlmRIFSEnable: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

#if defined(WIN32)
#define WLM_SCAN_PARAMS_SSID_MAX  10
#define WLM_DUMP_BUF_LEN  (127 * 1024)
#define WLM_SCAN_PARAMS_FIXED_SIZE WL_SCAN_PARAMS_FIXED_SIZE  /* 64 */
#define WLM_NUMCHANNELS WL_NUMCHANNELS
#define WLM_SCANNETWORK_DELAY 5000 /* ms */
#define TEMP_BUF_LEN 2048
int wlmScanNetworks(char* results_buf, int len)
{
	wl_scan_params_t *params;
	int params_size = WLM_SCAN_PARAMS_FIXED_SIZE + WLM_NUMCHANNELS * sizeof(uint16);
	wlc_ssid_t ssids[WLM_SCAN_PARAMS_SSID_MAX];
	char *dump_buf, *dump_buf_orig;
	int err, i;
	wl_scan_results_t *results;
	wl_bss_info_t *bi;
	bool pass = TRUE;
	uint offset;
	int n;
	char temp_buf[TEMP_BUF_LEN];
	if (len > WLM_DUMP_BUF_LEN) {
		printf("wlmScanNetworks: requested len %d exceeded maximum %d bytes.\n",
		       len, WLM_DUMP_BUF_LEN);
		return FALSE;
	}
	params_size += WLM_SCAN_PARAMS_SSID_MAX * sizeof(wlc_ssid_t);
	params = (wl_scan_params_t*)malloc(params_size);
	if (params == NULL) {
		printf("wlmScanNetworks: Error allocating %d bytes for scan params\n", params_size);
		return FALSE;
	}
	memset(params, 0, params_size);
	memcpy(&params->bssid, &ether_bcast, ETHER_ADDR_LEN);
	params->bss_type = DOT11_BSSTYPE_ANY;
	params->scan_type = 0;   /* active scan */
	params->nprobes = -1;
	params->active_time = -1;
	params->passive_time = -1;
	params->home_time = -1;
	params->channel_num = 0;
	memset(ssids, 0, WLM_SCAN_PARAMS_SSID_MAX * sizeof(wlc_ssid_t));
	err = wlu_set(irh, WLC_SCAN, params, params_size);
	free(params);
	if (err < 0) {
		printf("wlmScanNetworks: Failed sending scan request\n");
		return FALSE;
	}
	Sleep(WLM_SCANNETWORK_DELAY);
	dump_buf = dump_buf_orig = (char*) malloc(WLM_DUMP_BUF_LEN);
	if (dump_buf == NULL) {
		printf("wlmScanNetworks: Error allocating %d bytes for dump_buf\n", WLM_DUMP_BUF_LEN);
		return FALSE;
	}
	results = (wl_scan_results_t*)dump_buf;
	results->buflen = WLM_DUMP_BUF_LEN;
	err = wlu_get(irh, WLC_SCAN_RESULTS, dump_buf, WLM_DUMP_BUF_LEN);
	if (err == 0) {
		if (results->count == 0) {
			printf("wlmScanNetworks: no network has been found.\n");
			pass = FALSE;
		} else if (results->version != WL_BSS_INFO_VERSION &&
			results->version != LEGACY2_WL_BSS_INFO_VERSION &&
			results->version != LEGACY_WL_BSS_INFO_VERSION) {
			printf("wlmScanNetworks: driver has bss_info_version %d "
			       "but this program supports only version %d.\n",
			       results->version, WL_BSS_INFO_VERSION);
			pass = FALSE;
		} else {
			bi = results->bss_info;
			offset = 0;
			for (i = 0; i < (int)results->count; i++) {
				n = 0;
				memset(temp_buf, 0, TEMP_BUF_LEN);
				wlm_dump_bss_info(bi, &temp_buf[0], &n);
				offset += snprintf(results_buf + offset, n, "%s", &temp_buf[0]);
				bi = (wl_bss_info_t*)((int8*)bi + dtoh32(bi->length));
			}
		}
	} else {
		printf("wlmScanNetworks: failed to get scan results.\n");
		pass = FALSE;
	}
	free(dump_buf);
	return pass;
}

static void
wlm_dump_bss_info(wl_bss_info_t *bi, char *scan_dump_buf, int *len)
{
	char ssidbuf[SSID_FMT_BUF_LEN];
	char chspec_str[CHANSPEC_STR_LEN];
	wl_bss_info_107_t *old_bi;
	int offset = 0;
	int n;
	*len = 0;
	if (dtoh32(bi->version) == LEGACY_WL_BSS_INFO_VERSION) {
		old_bi = (wl_bss_info_107_t *)bi;
		bi->chanspec = CH20MHZ_CHSPEC(old_bi->channel);
		bi->ie_length = old_bi->ie_length;
		bi->ie_offset = sizeof(wl_bss_info_107_t);
	}
	n = wlm_format_ssid(ssidbuf, bi->SSID, bi->SSID_len);
	offset += sprintf(scan_dump_buf, "SSID: \"%s\"\n", ssidbuf);
	offset += sprintf(scan_dump_buf + offset,
			  "Mode: %s\t", wlm_capmode2str(dtoh16(bi->capability)));
	offset += sprintf(scan_dump_buf + offset, "RSSI: %d dBm\t", (int16)(dtoh16(bi->RSSI)));
	if (dtoh32(bi->version) == WL_BSS_INFO_VERSION) {
		offset += sprintf(scan_dump_buf + offset,
				  "SNR: %d dB\t", (int16)(dtoh16(bi->SNR)));
	}
	offset += sprintf(scan_dump_buf + offset, "noise: %d dBm\t", bi->phy_noise);
	offset += sprintf(scan_dump_buf + offset,
			  "Channel: %s\n", wlm_wf_chspec_ntoa(bi->chanspec, chspec_str));
	offset += sprintf(scan_dump_buf + offset,
			  "BSSID: %s\t", wlm_ether_etoa(&bi->BSSID));
	offset += sprintf(scan_dump_buf + offset, "Capability: ");
	bi->capability = dtoh16(bi->capability);
	if (bi->capability & DOT11_CAP_ESS)
		offset +=  sprintf(scan_dump_buf + offset, "ESS ");
	if (bi->capability & DOT11_CAP_IBSS)
		offset += sprintf(scan_dump_buf + offset, "IBSS ");
	if (bi->capability & DOT11_CAP_POLLABLE)
		offset += sprintf(scan_dump_buf + offset, "Pollable ");
	if (bi->capability & DOT11_CAP_POLL_RQ)
		offset += sprintf(scan_dump_buf + offset, "PollReq ");
	if (bi->capability & DOT11_CAP_PRIVACY)
		offset += sprintf(scan_dump_buf + offset, "WEP ");
	if (bi->capability & DOT11_CAP_SHORT)
		offset += sprintf(scan_dump_buf + offset, "ShortPre ");
	if (bi->capability & DOT11_CAP_PBCC)
		offset += sprintf(scan_dump_buf + offset, "PBCC ");
	if (bi->capability & DOT11_CAP_AGILITY)
		offset += sprintf(scan_dump_buf + offset, "Agility ");
	if (bi->capability & DOT11_CAP_SHORTSLOT)
		offset += sprintf(scan_dump_buf + offset, "ShortSlot ");
	if (bi->capability & DOT11_CAP_CCK_OFDM)
		offset += sprintf(scan_dump_buf + offset, "CCK-OFDM ");
	offset += sprintf(scan_dump_buf + offset, "\n\n");
	*len = offset;
}
int wlm_format_ssid(char* buf, uint8* ssid, int ssid_len)
{
	int i;
	uint8 c;
	char *p = buf;
	for (i = 0; i < ssid_len; i++) {
		c = ssid[i];
		if (c == '\\') {
			*p++ = '\\';
			*p++ = '\\';
		} else if (isprint((uchar)c)) {
			*p++ = (char)c;
		} else {
			p += sprintf(p, "\\x%0x2X", c);
		}
	}
	*p = '\0';
	return p - buf;
}
static char nwm_etoa_buf[ETHER_ADDR_LEN *3];
static char *
wlm_ether_etoa(const struct ether_addr *n)
{
	char *c = nwm_etoa_buf;
	int i;
	for (i = 0; i < ETHER_ADDR_LEN; i++) {
		if (i)
			*c++ = ':';
		c += sprintf(c, "%02X", n->octet[i] & 0xff);
	}
	return nwm_etoa_buf;
}
static const char *
wlm_capmode2str(uint16 capability)
{
	capability &= (DOT11_CAP_ESS | DOT11_CAP_IBSS);
	if (capability == DOT11_CAP_ESS)
		return "Managed";
	else if (capability == DOT11_CAP_IBSS)
		return "Ad Hoc";
	else
		return "<unknown>";
}
static char *
wlm_wf_chspec_ntoa(chanspec_t chspec, char *buf)
{
	const char *band, *bw, *sb;
	uint channel;
	band = "";
	bw = "";
	sb = "";
	channel = CHSPEC_CHANNEL(chspec);
	if ((CHSPEC_IS2G(chspec) && channel > CH_MAX_2G_CHANNEL) ||
	    (CHSPEC_IS5G(chspec) && channel <= CH_MAX_2G_CHANNEL))
		band = (CHSPEC_IS2G(chspec)) ? "b" : "a";
	if (CHSPEC_IS40(chspec)) {
		if (CHSPEC_SB_UPPER(chspec)) {
			sb = "u";
			channel += CH_10MHZ_APART;
		} else {
			sb = "l";
			channel -= CH_10MHZ_APART;
		}
	} else if (CHSPEC_IS10(chspec)) {
		bw = "n";
	}
	snprintf(buf, 6, "%d%s%s%s", channel, band, bw, sb);
	return (buf);
}
#endif /* WIN32 */

int
wlmCisUpdate(int off, char* bufp)
{
	int ret = 0;
	int preview = 0;
	uint32 len;
	uint32 updatelen;
	uint32 i;
	char hexstr[3];
	char bytes[SROM_MAX];
	char buf[WLC_IOCTL_MAXLEN] = {0};
	cis_rw_t cish;
	char *cisp;
	updatelen = strlen(bufp);
	if (updatelen % 2) {
		printf("wlmCisUpdate: cisupdate hex string must contain even number of digits\n");
		return FALSE;
	}
	updatelen /= 2;
	for (i = 0; i < updatelen; i++) {
		hexstr[0] = *bufp;
		hexstr[1] = *(bufp + 1);
		if (!isxdigit((int)hexstr[0]) || !isxdigit((int)hexstr[1])) {
			printf("wlmCisUpdate: cisupdate invalid hex digit(s) in %s\n", bufp);
			return FALSE;
		}
		hexstr[2] = '\0';
		bytes[i] = (char) strtol(hexstr, NULL, 16);
		bufp += 2;
	}
	cish.source = 0;
	cish.byteoff = 0;
	cish.nbytes = 0;
	memset(buf, 0, WLC_IOCTL_MAXLEN);
	strcpy(buf + 9, "cisdump");
	bufp = buf + strlen("cisdump") + 1 + 9;
	memcpy(bufp, (char*)&cish, sizeof(cish));
	bufp += sizeof(cish);
	ret = wl_get(irh, WLC_GET_VAR, buf + 9, (bufp - (buf + 9)) + SROM_MAX);
	if (ret < 0) {
		printf("wlmCisUpdate, cisupdate failed to read cis: %d\n", ret);
		return FALSE;
	}
	bufp = buf + 9;
	memcpy((char*)&cish, bufp, sizeof(cish));
	len = dtoh32(cish.nbytes);
	if ((off + updatelen) > len) {
		printf("wlmCisUpdate, cisupdate offset %d plus update len %d exceeds CIS len %d\n",
		        off, updatelen, len);
		return FALSE;
	}
	bufp += sizeof(cish);
	if (cish.source == WLC_CIS_SROM) {
		for (i = 0; i < updatelen; ++i)
			bufp[off + i] = bytes[i] & 0xff;
	} else {
		for (i = 0; i < updatelen; ++i) {
			if (~bytes[i] & bufp[off + i]) {
				printf("wlmCisUpdate: OTP update incompatible:"
				        " update[%d](0x%02x)->cis[%d](0x%02x)\n",
				        i,  bytes[i], off + i, bufp[off + i]);
				return FALSE;
			}
			bufp[off + i] |= bytes[i];
		}
	}
	bufp = buf;
	strcpy(bufp, "ciswrite");
	bufp += strlen("ciswrite") + 1;
	cisp = bufp;
	cish.source = 0;
	cish.byteoff = 0;
	cish.nbytes = htod32(len);
	memcpy(cisp, (char*)&cish, sizeof(cish));
	if (preview) {
		printf("wlmCisUpdate: offset %d data %s cislen %d\n", off, bufp, len);
		bufp += sizeof(cish);
		for (i = 0; i < len; i++) {
			if ((i % 8) == 0)
				printf("\nByte %3d: ", i);
			printf("0x%02x ", (uint8)bufp[i]);
		}
		printf("\n");
	} else {
		ret = wl_set(irh, WLC_SET_VAR, buf, (cisp - buf) + sizeof(cish) + len);
		if (ret < 0) {
			printf("wlmCisUpdate: cisupdate cis write failed: %d\n", ret);
			return FALSE;
		}
	}
	return TRUE;
}

#if defined(WIN32)
wlmCommandPassThrough(char *cmd_line, char *result_buf, int *len)
{
	char *tmp_argv[WLM_NUM_ARGS];
	char **argv = tmp_argv;
	int argc;
	FILE *pf;
	char *buf;
	int ret = 0;
	*len = 0;


	buf = (char *)malloc(MAX_COMMAND_RESULT_BUF_LEN);
	if (buf == NULL) {
		printf("wlmCommandPassThrough: failed allocate memory\n");
		return FALSE;
	}
	memset(buf, 0, MAX_COMMAND_RESULT_BUF_LEN);

	argc = wlm_buf_to_args(cmd_line, argv);
	if (argc < 1) {
		printf("wlmCommandPassThrough: failed to convert string to argv\n");
		return FALSE;
	}

	ret = wlm_process_args(irh, argv);
	/* restore stdout */
	freopen("CON", "w", stdout);

	/* read from output file and copy to return buf */
	pf = fopen(outf, "r");
	if (pf == NULL) {
		printf("wlmCommandPassThrough: failed to open %s\n", outf);
		ret = -1;
		goto done;
	}

	*len = fread(buf, 1, MAX_COMMAND_RESULT_BUF_LEN, pf);
	strncpy(result_buf, buf, *len);

done:
	free(buf);
	if (pf)
		fclose(pf);

	if (ret < 0) {
		printf("wlmCommandPassThrough: %s",  wlmLastError());
		return FALSE;
	}

	return TRUE;
}

static int
wlm_buf_to_args(char *tmp, char *new_args[])
{
	char line[512];
	char *token;
	int argc = 0;

	strcpy(line, tmp);
	while  ((argc < (WLM_NUM_ARGS - 1)) &&
		((token = strtok(argc ? NULL : line, " \t")) != NULL)) {

		/* Specifically make sure empty arguments (like SSID) are empty */
		if (token[0] == '"' && token[1] == '"') {
			token[0] = '\0';
		}

		new_args[argc] = malloc(strlen(token)+1);
		strncpy(new_args[argc], token, strlen(token)+1);
		argc++;
	}
	new_args[argc] = NULL;
	return argc;
}


static int
wlm_process_args(HANDLE irh, char **argv)
{
	char *ifname = NULL;
	int help = 0;
	int init_err = FALSE;
	int status = 0;
	int err = 0;
	cmd_t *cmd = NULL;
	FILE *pf;

	/* redirect stdout to a file */
	pf = freopen(outf, "w", stdout);
	if (pf == NULL) {
		printf("wlm_process_args: failed to open %s\n", outf);
		return FALSE;
	}

	if (*argv) {
		cmd = wlm_find_cmd(*argv, 0, rwl_get_remote_type());

		/* defaults to using the set_var and get_var commands  */
		if (!cmd)
			cmd = &wl_varcmd;

		/* do command */
		if (cmd->name)
			err = (*cmd->func)((void *) irh, cmd, argv);
	}

	/* restore stdout */
	freopen("CON", "w", stdout);
//	flushall();

	if (pf)
		fclose(pf);

	if (err < 0)
		return FALSE;

	return TRUE;
}

static cmd_t *
wlm_find_cmd(char *name, int ap_mode, int remote_os_type)
{
	cmd_t *cmd = NULL;
	cmd_t *ndis_cmd = NULL;
	cmd_t *base_cmd = NULL;

	/* search the wl_cmds */
	for (cmd = wl_cmds; cmd->name && strcmp(cmd->name, name); cmd++);

	if (cmd->name != NULL)
		base_cmd = cmd;

	if (remote_os_type == LINUX_OS || remote_os_type == WINVISTA_OS) {
		/* prefer a wl cmd always */
		cmd = (base_cmd)?base_cmd:ndis_cmd;
	} else {
		/* if in ap mode, prefer a base wl_cmd over an ndis cmd,
		 * if in sta mode, prefer the ndis cmds
		 */
		if (ap_mode)
			cmd = (base_cmd)?base_cmd:ndis_cmd;
		else
			cmd = (ndis_cmd)?ndis_cmd:base_cmd;
	}

	return cmd;
}
#endif /* defined(WIN32) */

int
wlmGpioOut(unsigned int mask, unsigned int val)
{
	uint32 *int_ptr;
	char buf[32] = "\0";

	mask = htod32(mask);

	val = htod32(val);

	if ((~mask & val) != 0) {
		printf("wlmGpioOut: mask and val don't matcch");
		return FALSE;
	}

	int_ptr = (uint32 *)buf;
	memcpy(int_ptr, (const void *)&mask, sizeof(mask));
	int_ptr++;
	memcpy(int_ptr, (const void *)&val, sizeof(val));

	if (wlu_iovar_set(irh, "gpioout", buf, sizeof(uint32) *2) < 0) {
		printf("wlmGpioOut: failed toggle gpio %d to %d\n", mask, val);
		return FALSE;
	} else
		return TRUE;
}

int
wlmPhyRssiGainDelta2gGet(char *varname, int *deltaValues, int core)
{
	int i, index;
	int8 vals[18];
	uint32 phytype;
	uint8 N = 5;

	if (curPhyType == PHY_TYPE_NULL)
		phytype = wlmPhyTypeGet();
	else
		phytype = curPhyType;

	if (phytype != WLC_PHY_TYPE_AC) {
		printf("wlmPhyRssiGainDelta2gGet: phy type %d is not supported\n", phytype);
		return FALSE;
	} else {
		memset(vals, 0, sizeof(vals));
		if (wlu_iovar_get(irh, varname, vals, sizeof(vals)) < 0) {
			printf("wlmPhyRssiGainDelta2gGet: failed to get delta values.\n");
			return FALSE;
		}

		memset(deltaValues, 0, N * sizeof(int));
		for (i = 0; i < N * 3; i++) {
			if (i == core * N) {
				core = (int)vals[i++];
				for (index = 0; index < N -1; index++)
					deltaValues[index] = (int)vals[i++];
				break;
			}
		}
	}

	return TRUE;
}

int
wlmPhyRssiGainDelta2gSet(char *varname, int *deltaValues, int core)
{
	int i, index;
	int8 vals[18];
	uint32 phytype;
	uint8 N = 5;

	if (curPhyType == PHY_TYPE_NULL)
		phytype = wlmPhyTypeGet();
	else
		phytype = curPhyType;

	if (phytype != WLC_PHY_TYPE_AC) {
		printf("wlmPhyRssiGainDelta2gSet: phy type %d is not supported\n", phytype);
		return FALSE;
	} else {
		for (i = 0; i < N * 3; i++) {
			if (i == core * N) {
				vals[i++] = core;
				for (index = 0; index < N -1; index++) {
					if (deltaValues[index] > 63 || deltaValues[index] < -64) {
						printf("wlmPhyRssiGainDelta2gSet:"
						       "input data out of range (-64,63)\n");
						return FALSE;
					}
					vals[i++] = deltaValues[index];
				}
				break;
			}
		}

		if (wlu_var_setbuf(irh, varname, vals + N * core, N * sizeof(int8)) < 0) {
			printf("wlmPhyRssiGainDelta2gSet: failed to set delta values.\n");
			return FALSE;
		}
	}
	return TRUE;
}

int
wlmPhyRssiGainDelta5gGet(char *varname, int *deltaValues, int core)
{
	int i, index;
	int8 vals[28];
	uint32 phytype;
	uint8 N, n_per_core, n_per_core_p1;
	int err = -1;
	int32 value;
	char *nvvarname = "rssi_cal_rev";
	const char *iovar = "nvram_get";
	char *endptr;
	void *p;

	if (curPhyType == PHY_TYPE_NULL)
		phytype = wlmPhyTypeGet();
	else
		phytype = curPhyType;

	if (phytype != WLC_PHY_TYPE_AC) {
		printf("wlmPhyRssiGainDelta5gGet: phy type %d is not supported\n", phytype);
		return FALSE;
	} else {
		err = wlu_var_getbuf_med(irh, iovar, nvvarname, strlen(nvvarname) + 1, &p);

		if (err == 0) {
			/* This means, NVRAM variable found. */
			/* Calls new function for accommodating ROuts */
			value = strtol((char*)p, &endptr, 0);
		}

		if ((err < 0) || ((err == 0) && (value == 0))) {
			/* This means, NVRAM variable not found or Variable is 0 */
			/* Calls old function */
			n_per_core = 6;
		} else {
			n_per_core = 12;
		}
		/* n_per_core stand for number per core plus 1, which is core id */
		n_per_core_p1 = n_per_core  + 1;
		N = n_per_core_p1;

		memset(vals, 0, sizeof(vals));
		if (wlu_iovar_get(irh, varname, vals, sizeof(vals)) < 0) {
			printf("wlmPhyRssiGainDelta5gGet: failed to get delta value.\n");
			return FALSE;
		}

		memset(deltaValues, 0, N * sizeof(int));
		for (i = 0; i < N * 3; i++) {
			if (i == core * N) {
				core = (int)vals[i++];
				for (index = 0; index < N -1; index++)
					deltaValues[index] = (int)vals[i++];
				break;
			}
		}
	}

	return TRUE;
}

int
wlmPhyRssiGainDelta5gSet(char *varname, int *deltaValues, int core)
{
	int i, index;
	int8 vals[28];
	uint32 phytype;
	uint8 N, n_per_core, n_per_core_p1;
	int err = -1;
	int32 value;
	char *nvvarname = "rssi_cal_rev";
	const char *iovar = "nvram_get";
	char *endptr;
	void *p;

	if (curPhyType == PHY_TYPE_NULL)
		phytype = wlmPhyTypeGet();
	else
		phytype = curPhyType;

	if (phytype != WLC_PHY_TYPE_AC) {
		printf("wlmPhyRssiGainDelta5gSet: phy type %d is not supported\n", phytype);
		return FALSE;
	} else {
		err = wlu_var_getbuf_med(irh, iovar, nvvarname, strlen(nvvarname) + 1, &p);
		if (err == 0) {
			/* This means, NVRAM variable found. */
			/* Calls new function for accommodating ROuts */
			value = strtol((char*)p, &endptr, 0);
		}

		if ((err < 0) || ((err == 0) && (value == 0))) {
			/* This means, NVRAM variable not found or Variable is 0 */
			/* Calls old function */
			n_per_core = 6;
		} else {
			n_per_core = 12;
		}
		/* n_per_core stand for number per core plus 1, which is core id */
		n_per_core_p1 = n_per_core  + 1;
		N = n_per_core_p1;

		for (i = 0; i < N * 3; i++) {
			if (i == core * N) {
				vals[i++] = core;
				for (index = 0; index < N -1; index++) {
					if (deltaValues[index] > 63 || deltaValues[index] < -64) {
						printf("wlmPhyRssiGainDelta5gSet:"
						       "input data out of range (-64,63)\n");
						return FALSE;
					}
					vals[i++] = (int8)deltaValues[index];
				}
				break;
			}
		}

		if (wlu_var_setbuf(irh, varname, vals + N * core, N * sizeof(int8)) < 0) {
			printf("wlmPhyRssiGainDelta5gSet: failed to set delta value.\n");
			return FALSE;
		}
	}

	return TRUE;
}

int
wlmVHTFeaturesSet(int val)
{
	val = htod32(val);

	if (wlu_iovar_setint(irh, "vht_features", val) < 0) {
		printf("wlmVHTFeaturesSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmTxBFEnable(void)
{
	if (wlu_iovar_setint(irh, "txbf", 1) < 0) {
		printf("wlmTxBFEnable: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmTxBFDisable(void)
{
	if (wlu_iovar_setint(irh, "txbf", 0) < 0) {
		printf("wlmTxBFDisable: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int
wlmSpectModeSet(int val)
{
	int spect;

	spect = htod32(val);

	if ((spect < SPECT_MNGMT_OFF) || (spect > SPECT_MNGMT_LOOSE_11H_D)) {
		printf("wlmSpectEnable: spect %d out of range [0, 4] \n", spect);
		return FALSE;
	}

	if (wlu_set(irh, WLC_SET_SPECT_MANAGMENT, &spect, sizeof(spect)) < 0) {
		printf("wlmSpectEnable: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int
wlmIBSSGmodeEnable(void)
{
	/* check parameter -1 here */
	if (wlu_iovar_setint(irh, "ibss_gmode", -1) < 0) {
		printf("wlmIBSSGmodeEnable: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int
wlmIBSSGmodeDisable(void)
{
	/* check parameter 0 here */
	if (wlu_iovar_setint(irh, "ibss_gmode", 0) < 0) {
		printf("wlmIBSSGmodeDisable: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int
wlmBandWidthCapabilitySet(WLM_BAND band, int val)
{
	struct {
		uint32 band;
		uint32 bw_cap;
	} param = {0, 0};

	param.band = band;
	param.bw_cap = (uint32)val;

	if (wlu_var_setbuf(irh, "bw_cap", &param, sizeof(param)) < 0) {
		printf("wlmBandWidthCapabilitySet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmBandWidthCapabilityGet(WLM_BAND band, int *val)
{
	struct {
		uint32 band;
		uint32 bw_cap;
	} param = {0, 0};
	void *ptr = NULL;

	param.band = band;
	param.bw_cap = 0;

	if (wlu_var_getbuf_sm(irh, "bw_cap", &param, sizeof(param), &ptr) < 0) {
		printf("wlmBandWidthCapabilityGet: %s\n", wlmLastError());
		return FALSE;
	}
	*val = *((uint32 *)ptr);

	return TRUE;
}

int
wlmEnableLongPacket(int enable)
{
	if ((enable < 0) || (enable > 1)) {
		printf("wlmEnableLongPacket: invalid input select [0,1]\n");
		return FALSE;
	}

	if (wlu_iovar_setint(irh, "longpkt", enable) < 0) {
		printf("wlmEnableLongPacket: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmSpatialPolicySet(int mode_2g, int mode_5g_low, int mode_5g_mid,
    int mode_5g_high, int mode_5g_upper)
{
	int mode[SPATIAL_MODE_MAX_IDX] = { -1, -1, -1, -1, -1};

	mode[SPATIAL_MODE_2G_IDX] = mode_2g;
	mode[SPATIAL_MODE_5G_LOW_IDX] = mode_5g_low;
	mode[SPATIAL_MODE_5G_MID_IDX] = mode_5g_mid;
	mode[SPATIAL_MODE_5G_HIGH_IDX] = mode_5g_high;
	mode[SPATIAL_MODE_5G_UPPER_IDX] = mode_5g_upper;

	if (wlu_var_setbuf(irh, "spatial_policy", &mode, sizeof(mode)) < 0) {
		printf("wlmSpatialPolicySet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmPHYRPTableSet(void)
{
	if (wlu_var_setbuf(irh, "phy_setrptbl", NULL, 0) < 0) {
		printf("wlmPHYRPTableSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmPHYForceImplicitBeamforming(void)
{
	if (wlu_var_setbuf(irh, "phy_forceimpbf", NULL, 0) < 0) {
		printf("wlmPHYForceImplicitBeamforming: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmPHYForceSteer(int on)
{
	if ((on < 0) || (on > 1)) {
		printf("wlmPHYForceSteer: invalid input select [0,1]\n");
		return FALSE;
	}

	if (wlu_iovar_setint(irh, "phy_forcesteer", on) < 0) {
		printf("wlmPHYForceSteer: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmPhyRssiGainError2gGet(char *varname, int *deltaValues)
{
	int i;
	int8 vals[18];
	uint32 phytype;
	uint8 N = 2;

	if (curPhyType == PHY_TYPE_NULL)
		phytype = wlmPhyTypeGet();
	else
		phytype = curPhyType;

	if (phytype != WLC_PHY_TYPE_AC) {
		printf("wlmPhyRssiGainError2gGet: phy type %d is not supported\n", phytype);
		return FALSE;
	} else {
		memset(vals, 0, sizeof(vals));
		if (wlu_iovar_get(irh, varname, vals, sizeof(vals)) < 0) {
			printf("wlmPhyRssiGainError2gGet: failed to get error values.\n");
			return FALSE;
		}

		memset(deltaValues, 0, N * sizeof(int));
		for (i = 0; i < N; i++) {
			deltaValues[i] = (int)vals[i];
		}
	}

	return TRUE;
}

int
wlmPhyRssiGainError2gSet(char *varname, int *deltaValues)
{
	int i;
	int8 vals[18];
	uint32 phytype;
	uint8 N = 2;

	if (curPhyType == PHY_TYPE_NULL)
		phytype = wlmPhyTypeGet();
	else
		phytype = curPhyType;

	if (phytype != WLC_PHY_TYPE_AC) {
		printf("wlmPhyRssiGainDelta2gSet: phy type %d is not supported\n", phytype);
		return FALSE;
	} else {
		memset(vals, 0, sizeof(vals));
		for (i = 0; i < N; i++) {
			if (deltaValues[i] > 63 || deltaValues[i] < -64) {
				printf("wlmPhyRssiGainError2gSet:"
				       "deltaValues[%d] out of range (-64,63) \n", i);
				return FALSE;
			}
			vals[i] = (int8)htod32(deltaValues[i]);
		}

		if (wlu_var_setbuf(irh, varname, vals, sizeof(vals)) < 0) {
			printf("wlmPhyRssiGainError2gSet: failed to set error values.\n");
			return FALSE;
		}
	}
	return TRUE;
}

int
wlmPhyRssiGainError5gGet(char *varname, int *deltaValues)
{
	int i;
	int8 vals[28];
	uint32 phytype;
	uint8 N = 2;

	if (curPhyType == PHY_TYPE_NULL)
		phytype = wlmPhyTypeGet();
	else
		phytype = curPhyType;

	if (phytype != WLC_PHY_TYPE_AC) {
		printf("wlmPhyRssiGainError5gGet: phy type %d is not supported\n", phytype);
		return FALSE;
	} else {
		memset(vals, 0, sizeof(vals));
		if (wlu_iovar_get(irh, varname, vals, sizeof(vals)) < 0) {
			printf("wlmPhyRssiGainError5gGet: failed to get error value.\n");
			return FALSE;
		}

		memset(deltaValues, 0, N * sizeof(int));
		for (i = 0; i < N; i++) {
			deltaValues[i] = (int)vals[i];
		}
	}

	return TRUE;
}

int
wlmPhyRssiGainError5gSet(char *varname, int *deltaValues)
{
	int i;
	int8 vals[28];
	uint32 phytype;
	uint8 N = 2;

	if (curPhyType == PHY_TYPE_NULL)
		phytype = wlmPhyTypeGet();
	else
		phytype = curPhyType;

	if (phytype != WLC_PHY_TYPE_AC) {
		printf("wlmPhyRssiGainError5gSet: phy type %d is not supported\n", phytype);
		return FALSE;
	} else {
		memset(vals, 0, sizeof(vals));
		for (i = 0; i < N;  i++) {
			if (deltaValues[i] > 63 || deltaValues[i] < -64) {
				printf("wlmPhyRssiGainError5gSet:"
				       "gainerror  %d out of range (-64,63) \n", deltaValues[i]);
				return FALSE;
			}
			vals[i] = (int8)htod32(deltaValues[i]);
		}

		if (wlu_var_setbuf(irh, varname, vals, sizeof(vals)) < 0) {
			printf("wlmPhyRssiGainError5gSet: failed to set error value.\n");
			return FALSE;
		}
	}

	return TRUE;
}

int
wlmPhyRawTempsenseGet(int *val)
{
	uint32 phytype;
	int8 rawtempsense;

	if (curPhyType == PHY_TYPE_NULL)
		phytype = wlmPhyTypeGet();
	else
		phytype = curPhyType;

	if (phytype != WLC_PHY_TYPE_AC) {
		printf("wlmPhyRawTempsenseGet: phy type %d is not supported\n", phytype);
		return FALSE;
	} else {
		if (wlu_iovar_get(irh, "phy_sromtempsense",
			&rawtempsense, sizeof(rawtempsense)) < 0) {
			printf("wlmPhyRawTempsenseGet: failed phy_rawtempsense.\n");
			return FALSE;
		}
	}

	*val = (int)rawtempsense;

	return TRUE;
}

int
wlmPhyRawTempsenseSet(int val)
{
	uint32 phytype;

	if (curPhyType == PHY_TYPE_NULL)
		phytype = wlmPhyTypeGet();
	else
		phytype = curPhyType;

	if (phytype != WLC_PHY_TYPE_AC) {
		printf("wlmPhyRawTempsenseSet: phy type %d is not supported\n", phytype);
		return FALSE;
	} else {
		if ((val > 63) || (val < -64)) {
			printf("wlmPhyRawTempsenseSet:"
			       "value %d out of  range (-64,63)\n", val);
			return FALSE;
		}

		if (wlu_var_setbuf(irh, "phy_sromtempsense",
			&val, sizeof(val)) < 0) {
			printf("wlmPhyRawTempsenseSet: failed to set value %s.\n", wlmLastError());
			return FALSE;
		}
	}

	return TRUE;
}

int
wlmPhyRPCalValsSet(int rpcal2g, int rpcal5gb0, int rpcal5gb1, int rpcal5gb2, int rpcal5gb3)
{
	wl_rpcal_t rpcal[WL_NUM_RPCALVARS];

	rpcal[WL_CHAN_FREQ_RANGE_2G].value = (uint16) (rpcal2g & 0xffff);
	rpcal[WL_CHAN_FREQ_RANGE_2G].update = 1;

	rpcal[WL_CHAN_FREQ_RANGE_5G_BAND0].value = (uint16) (rpcal5gb0 & 0xffff);
	rpcal[WL_CHAN_FREQ_RANGE_5G_BAND0].update = 1;

	rpcal[WL_CHAN_FREQ_RANGE_5G_BAND1].value = (uint16) (rpcal5gb1 & 0xffff);
	rpcal[WL_CHAN_FREQ_RANGE_5G_BAND1].update = 1;

	rpcal[WL_CHAN_FREQ_RANGE_5G_BAND2].value = (uint16) (rpcal5gb2 & 0xffff);
	rpcal[WL_CHAN_FREQ_RANGE_5G_BAND2].update = 1;

	rpcal[WL_CHAN_FREQ_RANGE_5G_BAND3].value = (uint16) (rpcal5gb3 & 0xffff);
	rpcal[WL_CHAN_FREQ_RANGE_5G_BAND3].update = 1;

	if (wlu_var_setbuf(irh, "rpcalvars", &(rpcal[0]),
		sizeof(wl_rpcal_t) * WL_NUM_RPCALVARS) < 0) {
		printf("wlmPhyRPCalValsSet: failed to set rpcalvals %s\n", wlmGetLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmPhyRPCalValsGet(int *rpcal2g, int *rpcal5gb0, int *rpcal5gb1, int *rpcal5gb2, int *rpcal5gb3)
{
	wl_rpcal_t rpcal[WL_NUM_RPCALVARS], *rpcal_out;
	void *ptr = NULL;

	if (wlu_var_getbuf_sm(irh, "rpcalvars", &(rpcal[0]),
	      sizeof(wl_rpcal_t) * WL_NUM_RPCALVARS, &ptr) < 0) {
		printf("wlmPhyRPCalValsGet: failed to get rpcalvals %s\n", wlmGetLastError());
		return FALSE;
	}

	rpcal_out = (wl_rpcal_t *) ptr;

	*rpcal2g = rpcal_out[WL_CHAN_FREQ_RANGE_2G].value;
	*rpcal5gb0 = rpcal_out[WL_CHAN_FREQ_RANGE_5G_BAND0].value;
	*rpcal5gb1 = rpcal_out[WL_CHAN_FREQ_RANGE_5G_BAND1].value;
	*rpcal5gb2 = rpcal_out[WL_CHAN_FREQ_RANGE_5G_BAND2].value;
	*rpcal5gb3 = rpcal_out[WL_CHAN_FREQ_RANGE_5G_BAND3].value;

	return TRUE;
}

#define WL_DUMP_BUF_LEN (4 * 1024)

int
wlmPhyRegDump(char *reg_name, int *dump_vals, int *len)
{
	char *dump_buf;
	char	*pchar = NULL;
	int i = 0;

	dump_buf = malloc(WL_DUMP_BUF_LEN);

	if (dump_buf == NULL) {
		printf("wlmPhyRegDump: "
		       "failed to allocate %d bytes for dump_buf\n", WL_DUMP_BUF_LEN);
		return FALSE;
	}

	memset(dump_buf, 0, WL_DUMP_BUF_LEN);

	if (dump_buf[0] != '\0')
		strcat(dump_buf, " ");

	strcat(dump_buf, reg_name);
	strcat(dump_buf, " ");

	if (wlu_iovar_getbuf(irh, "dump",
		dump_buf, strlen(dump_buf), dump_buf, WL_DUMP_BUF_LEN) < 0) {
		printf("wlmPhyRegDump: failed dump %s.\n", reg_name);
		return FALSE;
	}

	*len = 0;

	pchar = strtok(dump_buf, "\n");
	while (pchar) {
		/* printf("%s\n", pchar); */
		dump_vals[i] = (int)strtol(pchar, NULL, 0);
		pchar = strtok(NULL, "\n");
		i++;
	}

	*len = i;

	return TRUE;
}

int
wlmPhyRssiCalibrationFrequencyGroup2gGet(int *values)
{
	int i;
	uint8 nvramValues[14];
	uint8 N = 0;
	uint32 phytype;

	if (curPhyType == PHY_TYPE_NULL)
		phytype = wlmPhyTypeGet();
	else
		phytype = curPhyType;

	if (phytype != WLC_PHY_TYPE_AC) {
		printf("wlmPhyRssiCalibrationFrequencyGroup2gGet: "
		       "phy type %d is not supported\n", phytype);
		return FALSE;
	} else {
		/* Reading the NVRAM variable */
		if (wlu_iovar_get(irh, "rssi_cal_freq_grp_2g",
			nvramValues, sizeof(nvramValues)) < 0) {
			printf("wlmPhyRssiCalibrationFrequencyGroup2gGet: "
			       "failed to get group values %s.\n", wlmGetLastError());
			return FALSE;
		}

		N = 14; /* 14 corresponds to number of channels in 2g */

		for (i = 0; i < N-2; i++) {
			values[i] = (int)nvramValues[i];
			values[i+1] = (int)nvramValues[i+1];
			i++;
		}
		values[i] = (int)nvramValues[i];
		values[i+1] = (int)nvramValues[i+1];
	}

	return TRUE;
}

int
wlmPhyRssiCalibrationFrequencyGroup2gSet(int *values)
{
	int i;
	uint8 N = 0;
	uint8 nvramValues[14];
	uint32 phytype;

	if (curPhyType == PHY_TYPE_NULL)
		phytype = wlmPhyTypeGet();
	else
		phytype = curPhyType;

	if (phytype != WLC_PHY_TYPE_AC) {
		printf("wlmPhyRssiCalibrationFrequencyGroup2gSet: "
		       "phy type %d is not supported\n", phytype);
		return FALSE;
	} else {
		N = 7;
		i = 0;
		while (i < 14) {
			nvramValues[i] = (uint8)(values[i] & 0xf);
			i++;
		}

		if (i != 14) {
			printf("wlmPhyRssicalibrationFrequencyGroup2gSet: "
			       "Insufficient arguments \n");
			return FALSE;
		}

		if (wlu_var_setbuf(irh, "rssi_cal_freq_grp_2g",
			nvramValues, N*2*sizeof(int8)) < 0) {
			printf("wlmPhyRssicalibrationFrequencyGroup2gSet: "
			       "failed to set group values %s\n", wlmGetLastError());
			return FALSE;
		}
	}
	return TRUE;
}

int
wlmPhyRssiGainDelta2gSubGet(char *varname, int *values, int coreid)
{
	int i;
	int8 deltaValues[28];
	uint32 phytype;
	uint8 N = 0;

	if (curPhyType == PHY_TYPE_NULL)
		phytype = wlmPhyTypeGet();
	else
		phytype = curPhyType;

	if (phytype != WLC_PHY_TYPE_AC) {
		printf("wlmPhyRssiGainDelta2GSubGet: phy type %d is not supported\n", phytype);
		return FALSE;
	} else {
		if (wlu_iovar_get(irh, varname,
			deltaValues, sizeof(deltaValues)) < 0) {
			printf("wlmPhyRssiGainDelta2GSubGet: "
			       "failed to get delta values %s\n", wlmGetLastError());
			return FALSE;
		}
		N = 9; /* 9 entries per core, 4350 - 18 MAX entries; 4345 9 MAX entries */
		for (i = 0; i < N - 1; i++) {
			if (deltaValues[(9 * coreid) + i] ==  -1)
				break;
			values[i] = deltaValues[(9 * coreid) + i + 1];
			printf("wlmSubGet: %d\n", values[i]);
		}
	}
	return TRUE;
}

int
wlmPhyRssiGainDelta2gSubSet(char *varname, int *values, int coreid)
{
	int i;
	int8 deltaValues[28];
	uint32 phytype;
	uint8 N = 0;

	if (curPhyType == PHY_TYPE_NULL)
		phytype = wlmPhyTypeGet();
	else
		phytype = curPhyType;

	if (phytype != WLC_PHY_TYPE_AC) {
		printf("wlmPhyRssiGainDelta2gSubSet: "
		       "phy type %d is not supported\n", phytype);
		return FALSE;
	} else {
		/* ACPHY : 8/9 entries for a core; core 0 delta's can be
		 * given with or with out core_num as first element
		 */
		N = 9;
		/*
		  if (wlu_iovar_get(irh, varname, deltaValues, sizeof(deltaValues)) < 0) {
			printf("wlmPhyRssiGainDelta2GSubSet: "
			       "failed to get existing delta values %s\n", wlmGetLastError());
			return FALSE;
		}
		*/
		deltaValues[coreid * 9] = (int8)(coreid & 0xff);
		for (i = 0; i < N - 1; i++) {
			deltaValues[(coreid * 9) + i + 1] = (int8)(values[i] & 0xff);
			printf("wlmSubSet: %d\n", deltaValues[(coreid * 9) + i + 1]);
		}

		if (wlu_var_setbuf(irh, varname,
			(deltaValues + coreid * 9), 9*sizeof(int8)) < 0) {
			printf("wlmPhyRssiGainDelta2GSubSet: "
			       "failed to set values %s\n", wlmGetLastError());
			return FALSE;
		}
	}

	return TRUE;
}

int
wlmAdjustedTssiGet(int core, int *adjTssi)
{
	int val;
	void *ptr = NULL;

	val = htod32(core);

	if (wlu_var_getbuf_sm(irh, "phy_adj_tssi", &val, sizeof(int), &ptr) < 0) {
		printf("wlmAdjTssiGet: failed to get adj_tssi value %s\n", wlmLastError());
		return FALSE;
	}

	*adjTssi = dtoh32(*(int*)ptr);

	return TRUE;
}

int
wlmTxCalGainSweepMeasGet(int count, int *tssi, int *txpower, int core)
{
	wl_txcal_meas_t *txcal_meas;
	void *ptr = NULL;

	uint8 i;

	if (count > MAX_NUM_TXCAL_MEAS) {
		printf("wlmTxCalGainSweepMeasGet: Entries exceeded max allowed\n");
		return FALSE;
	}

	if (wlu_var_getbuf_med(irh, "txcal_gainsweep_meas", NULL, 0, &ptr) < 0) {
		printf("wlmTxCalGainSweepMeasGet: failed to get txcal gain sweep measurement\n");
		return FALSE;
	}

	txcal_meas = ptr;

	for (i = 0; i < txcal_meas->valid_cnt; i++) {
		tssi[i] = txcal_meas->tssi[core][i];
		txpower[i] = txcal_meas->pwr[core][i];
		/* printf("tssi[%d] %d, txpower[%d] %d\n", i, tssi[i], i, txpower[i]); */
	}

	return TRUE;
}

int
wlmTxCalGainSweepMeasSet(int *measPower, int count, int core)
{
	wl_txcal_meas_t *txcal_meas;
	int16 pwr[WLC_TXCORE_MAX][MAX_NUM_TXCAL_MEAS];
	void *ptr = NULL;
	uint8 i;

	memset(pwr, 0, WLC_TXCORE_MAX*MAX_NUM_TXCAL_MEAS*sizeof(pwr[0][0]));

	if (wlu_var_getbuf_med(irh, "txcal_gainsweep_meas", NULL, 0, &ptr) < 0) {
		printf("wlmTxCalGainSweepMeasSet: failed to get current TxCalGainSweep entries.\n");
		return FALSE;
	}

	txcal_meas = ptr;

	memcpy(&pwr[0][0], &txcal_meas->pwr[0][0],
	       WLC_TXCORE_MAX*MAX_NUM_TXCAL_MEAS*sizeof(pwr[0][0]));

	i = 0;

	while (i < count) {
		if (i >= MAX_NUM_TXCAL_MEAS) {
			printf("wlmTxCalGainSweepMeasSet: Entries exceeded max allowed\n");
			return FALSE;
		}

		pwr[core][i] = (int16)measPower[i];
		i++;
	}

	if (i != txcal_meas->valid_cnt) {
		printf("wlmTxCalGainSweepMeasSet: Incorrect Number of Entries."
		      " Expected %d, Entered %d\n", txcal_meas->valid_cnt, i);
		return FALSE;
	}

	if (wlu_var_setbuf(irh, "txcal_gainsweep_meas",
		pwr, WLC_TXCORE_MAX*MAX_NUM_TXCAL_MEAS*sizeof(pwr[0][0])) < 0) {
		printf("wlmTxCalGainSweepMeasSet: failed to set TxCalGainSweep entries\n");
		return FALSE;
	}

	return TRUE;
}

int
wlmTxCalGainSweep(char *desMac, int delay, int length, int nFrames, int start, int stop, int step)
{
	wl_txcal_params_t txcal_params;
	uint8 gidx_start, gidx_stop;
	int8 gidx_step;

	memset(&txcal_params, 0, sizeof(txcal_params));

	if (!wl_ether_atoe(desMac, (struct ether_addr *)&txcal_params.pkteng.dest)) {
		printf("wlmTxCalGainSweep: destinatin Mac address incorrect %s\n", desMac);
		return FALSE;
	}

	txcal_params.pkteng.delay = delay;
	txcal_params.pkteng.length = length;
	txcal_params.pkteng.nframes = nFrames;

	if (txcal_params.pkteng.nframes == 0)
		txcal_params.pkteng.nframes = 4;

	txcal_params.pkteng.flags = WL_PKTENG_PER_TX_START;
	txcal_params.pkteng.flags |= WL_PKTENG_SYNCHRONOUS;
	txcal_params.pkteng.flags = htod32(txcal_params.pkteng.flags);
	txcal_params.pkteng.delay = htod32(txcal_params.pkteng.delay);
	txcal_params.pkteng.nframes = htod32(txcal_params.pkteng.nframes);
	txcal_params.pkteng.length = htod32(txcal_params.pkteng.length);

	gidx_start = start;
	gidx_stop = stop;
	gidx_step = step;

	txcal_params.gidx_start = gidx_start;
	txcal_params.gidx_step = gidx_step;
	txcal_params.gidx_stop = gidx_stop;

	if (wlu_var_setbuf(irh, "txcal_gainsweep", &txcal_params, sizeof(txcal_params)) < 0) {
		printf("wlmTxCalGainSweep: failed to run TxCalGainSweep\n");
		return FALSE;
	}

	return TRUE;
}

int
wlmTxCalPowerTssiTableGet(int core, int *channel, int *startPower, int *num, int *tssiValues)
{
	wl_txcal_power_tssi_t *txcal_pwr_tssi_ptr;
	void *ptr = NULL;
	uint8 i;

	if (wlu_var_getbuf_med(irh, "txcal_pwr_tssi_tbl",
		(uint8*)channel, sizeof(uint8), &ptr) < 0) {
		printf("wlmTxCalPowerTssiTableGet: failed to read the table\n");
		return FALSE;
	}

	txcal_pwr_tssi_ptr = ptr;

	*channel = txcal_pwr_tssi_ptr->channel;
	*startPower = txcal_pwr_tssi_ptr->pwr_start[core];
	*num = txcal_pwr_tssi_ptr->num_entries[core];
	for (i = 0; i < txcal_pwr_tssi_ptr->num_entries[core]; i++)
		tssiValues[i] = txcal_pwr_tssi_ptr->tssi[core][i];

	return TRUE;
}

int
wlmTxCalPowerTssiTableSet(int core, int channel, int startPower, int num, int *tssiValues)
{
	wl_txcal_power_tssi_t txcal_pwr_tssi;
	wl_txcal_power_tssi_t *txcal_pwr_tssi_ptr;
	void *ptr = NULL;
	uint8 i;

	if (wlu_var_getbuf_med(irh, "txcal_pwr_tssi_tbl",
		(uint8*)&channel, sizeof(uint8), &ptr) < 0) {
		printf("wlmTxCalPowerTssiTableSet: "
		       "could not set the consolidated power tssi table, %s\n", wlmLastError());
		return FALSE;
	}
	txcal_pwr_tssi_ptr = ptr;

	txcal_pwr_tssi = *txcal_pwr_tssi_ptr;
	txcal_pwr_tssi.set_core = core;
	txcal_pwr_tssi.channel = channel;
	txcal_pwr_tssi.pwr_start[core] = startPower;
	txcal_pwr_tssi.num_entries[core] = num;

	memset(txcal_pwr_tssi.tssi[core], 0, MAX_NUM_PWR_STEP*sizeof(txcal_pwr_tssi.tssi[0][0]));

	i = 0;
	for (i = 0; i < num; i++) {
		if (i >= MAX_NUM_PWR_STEP) {
			printf("wlmTxCalPowerTssiTableSet: Entries exceeded max allowed\n");
			return FALSE;
		}
		txcal_pwr_tssi.tssi[core][i] = tssiValues[i];
	}

	if (i != txcal_pwr_tssi.num_entries[core]) {
		printf("wlmTxXalPowerTsiTableset: Incorrect Number of Entries."
		       "Expected %d, Entered %d\n",
		       txcal_pwr_tssi.num_entries[core], i);
		return FALSE;
	}

	txcal_pwr_tssi.gen_tbl = 0;
	if (wlu_var_setbuf(irh, "txcal_pwr_tssi_tbl",
		&txcal_pwr_tssi, sizeof(txcal_pwr_tssi)) < 0) {
		printf("wlmTxCalPowerTssiTableSet: failed to set TxCal TssiTable Set\n");
		return FALSE;
	}

	return TRUE;
}

int
wlmTxCalPowerTssiTableGenerate(int core, int channel, int startPower, int num, int *tssi)
{
	wl_txcal_power_tssi_t txcal_pwr_tssi;
	wl_txcal_power_tssi_t *txcal_pwr_tssi_ptr;
	void *ptr = NULL;
	uint8 i;

	if (num >= MAX_NUM_PWR_STEP) {
		printf("wlmTxCalPowerTssiTableGenerate: "
		       "Entries exceeded max allowed\n");
		return FALSE;
	}

	if (wlu_var_getbuf_med(irh, "txcal_pwr_tssi_tbl",
		(uint8*)&channel, sizeof(uint8), &ptr) < 0) {
		printf("wlmTxCalPowerTssiTableGenerate: "
		       "could not generate consolidate table, %s\n", wlmLastError());
		return FALSE;
	}
	txcal_pwr_tssi_ptr = ptr;

	txcal_pwr_tssi = *txcal_pwr_tssi_ptr;
	txcal_pwr_tssi.set_core = core;
	txcal_pwr_tssi.channel = channel;
	txcal_pwr_tssi.pwr_start[core] = startPower;
	txcal_pwr_tssi.num_entries[core] = num;

	txcal_pwr_tssi.gen_tbl = 1;

	if (wlu_var_setbuf(irh, "txcal_pwr_tssi_tbl",
		&txcal_pwr_tssi, sizeof(txcal_pwr_tssi)) < 0) {
		printf("wlmTxCalPowerTssiTableGenerate: "
		       "could not generate consolidate table, %s\n", wlmLastError());
		return FALSE;
	}

	for (i = 0; i < txcal_pwr_tssi_ptr->num_entries[core]; i++)
		tssi[i] = txcal_pwr_tssi_ptr->tssi[core][i] & 0xff;


	return TRUE;
}

int
wlmTxCalEstimatePowerLookupTableRead(int core, int *estPwrLut)
{
	uint16 *estpwrlut;
	void *ptr = NULL;
	int i;
	int val;

	val = htod32(core);

	if (wlu_var_getbuf_med(irh, "phy_read_estpwrlut", &val, sizeof(val), &ptr) < 0) {
		printf("wlmTxCalEstimatePowerLookupTableRead: failed %s\n", wlmLastError());
		return FALSE;
	}

	estpwrlut = ptr;

	for (i = 0; i < 128; i++)
		estPwrLut[i] = (int)(estpwrlut[i] > 0x7f ?
			(int16) (estpwrlut[i] - 0x100) : estpwrlut[i]);

	return TRUE;
}

int
wlmTxCalApplyPowerTssiTable(int mode)
{
	if ((mode < 0) && (mode > 1)) {
		printf("wlmTxCalApplyPowerTssiTable: invalid argument. Must be 0 or 1.\n");
		return FALSE;
	}

	if (wlu_iovar_setint(irh, "txcal_apply_pwr_tssi_tbl", mode)) {
		printf("wlmTxCalApplyPowerTssiTable: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmPhyForceCRSMinSet(int *values, int count)
{
	int err = -1;
	int8 th[4] = { 0 };
	int32 value;
	uint8 i = 0;
	uint32 phytype;
	int N;

	if (curPhyType == PHY_TYPE_NULL)
		phytype = wlmPhyTypeGet();
	else
		phytype = curPhyType;

	if (phytype != WLC_PHY_TYPE_AC) {
		printf("wlmPhyForceCRSMinSet: "
			"phy type %d is not supported\n", phytype);
		 return FALSE;
	} else {
		N = 3; /* IOVAR works only for up to 3 cores */
		if (count > N) {
			printf("wlmPhyForceCRSMinSet: "
			       "this iovara works only for up to 3 core\n");
			return FALSE;
		}

		for (i = 0; i < count; i++) {
			value = (int32)htod32(values[i]);

			if ((i == 0) && (value < -1)) {
				/* Offset values (2nd/3rd arguments) can be negative */
				printf("wlmPhyForceCRSMinSet: "
				       "first value core0_th can not be negative, except for -1\n");
				return FALSE;
			}
			th[i] = (int8)value;
		}

		if ((err = wlu_var_setbuf(irh, "phy_force_crsmin", th, 4*sizeof(int8))) < 0) {
			printf("wlmPhyForceCRSMinSet: "
			       "failed to set phy_force_crsmin values\n");
			return FALSE;
		}
	}

	return TRUE;
}

int
wlmOlpcAnchorIndexGet(int core, int channel, int *pwr_start_index, int *tempsense)
{
	wl_txcal_power_tssi_t *txcal_pwr_tssi_ptr;
	void *ptr = NULL;

	if (wlu_var_getbuf_med(irh, "olpc_anchoridx", (uint8*)&channel, sizeof(uint8), &ptr) < 0) {
		printf("wlmOlpcAnchorIndexGet: failed to retrieve anchor point gain index\n");
		return FALSE;
	}

	txcal_pwr_tssi_ptr = ptr;
	*pwr_start_index = (int) (txcal_pwr_tssi_ptr->pwr_start_idx[core]);
	*tempsense = (int) (txcal_pwr_tssi_ptr->tempsense[core]);
	return TRUE;
}

int
wlmOlpcAnchorIndexSet(int core, int channel, int pwr_start_index, int tempsense)
{
	wl_txcal_power_tssi_t txcal_pwr_tssi;
	wl_txcal_power_tssi_t *txcal_pwr_tssi_ptr;
	void *ptr = NULL;

	if (wlu_var_getbuf_med(irh, "olpc_anchoridx", (uint8*)&channel, sizeof(uint8), &ptr) < 0) {
		printf("wlmOlpcAnchorIndexSet: failed to retrieve anchor point gain index\n");
		return FALSE;
	}

	txcal_pwr_tssi_ptr = ptr;
	txcal_pwr_tssi = *txcal_pwr_tssi_ptr;
	txcal_pwr_tssi.channel = channel;
	txcal_pwr_tssi.tempsense[core] = tempsense;
	txcal_pwr_tssi.set_core = core;
	txcal_pwr_tssi.pwr_start_idx[core] = pwr_start_index;
	if (wlu_var_setbuf(irh, "olpc_anchoridx", &txcal_pwr_tssi, sizeof(txcal_pwr_tssi)) < 0) {
		printf("wlmOlpcAnchorIndexSet: failed to set anchor point gain index\n");
		return FALSE;
	}


	return TRUE;
}

int
wlmOlpcAnchorPower2GGet(int *power)
{
	if (wlu_iovar_getint(irh, "olpc_anchor2g", power)) {
		printf("wlmOlpcAnchorPower2GGet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmOlpcAnchorPower2GSet(int power)
{
	if (wlu_iovar_setint(irh, "olpc_anchor2g", power)) {
		printf("wlmOlpcAnchorPower2GSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmOlpcAnchorPower5GGet(int *power)
{
	if (wlu_iovar_getint(irh, "olpc_anchor5g", power)) {
		printf("wlmOlpcAnchorPower5GGet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmOlpcAnchorPower5GSet(int power)
{
	if (wlu_iovar_setint(irh, "olpc_anchor5g", power)) {
		printf("wlmOlpcAnchorPower5GSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmOlpcDisable(void)
{
	if (wlu_iovar_setint(irh, "disable_olpc", 1)) {
		printf("wlmOlpcDisable: %s\n", wlmLastError());
		return FALSE;
	}
	return TRUE;
}

int
wlmOlpcEanble(void)
{
	if (wlu_iovar_setint(irh, "disable_olpc", 0)) {
		printf("wlmOlpcEnable: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmOlpcThresholdGet(int *val)
{
	if (wlu_iovar_getint(irh, "olpc_thresh", val)) {
		printf("wlmOlpcThresholdGet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmOlpcThresholdSet(int val)
{
	if (wlu_iovar_setint(irh, "olpc_thresh", val)) {
		printf("wlmOlpcThresholdSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmOlpcIndexValidGet(int *val)
{
	if (wlu_iovar_getint(irh, "olpc_idx_valid", val)) {
		printf("wlmOlpcIndexValidGet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmOlpcIndexValidSet(int val)
{
	if (wlu_iovar_setint(irh, "olpc_idx_valid", val)) {
		printf("wlmOlpcIndexValidSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmPhyTxPowerCapGet(int *txpwrcap)
{
	if (wlu_iovar_getint(irh, "phy_txpwrcap", txpwrcap)) {
		printf("wlmPhyTxPowerCapGet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmPhyTxPowerCapTableGet(int core,  int *numAnt, int *cellOnCaps, int *cellOffCaps)
{
	wl_txpwrcap_tbl_t * txpwrcap_tbl_ptr;
	void *ptr = NULL;
	uint8 i;

	if (wlu_var_getbuf_med(irh, "phy_txpwrcap_tbl", NULL, 0, &ptr) < 0) {
		printf("wlmPhyTxPowerCapTableGet: %s\n", wlmLastError());
		return FALSE;
	}

	txpwrcap_tbl_ptr = ptr;
	*numAnt = txpwrcap_tbl_ptr->num_antennas_per_core[core];

	/*
	printf("wl_txpwrcap_tbl_ptr->num_antennas_per_core:\n");
	for (i = 0; i < TXPWRCAP_MAX_NUM_CORES; i++) {
		printf("%d ", txpwrcap_tbl_ptr->num_antennas_per_core[i]);
	}

	printf("\n");
	printf("pwrcap_cell_on:\n");
	for (i = 0; i < TXPWRCAP_MAX_NUM_ANTENNAS; i++) {
		printf("%d ", txpwrcap_tbl_ptr->pwrcap_cell_on[i]);
	}
	printf("\n");

	printf("pwrcap_cell_off\n");
	for (i = 0; i < TXPWRCAP_MAX_NUM_ANTENNAS; i++) {
		printf("%d ", txpwrcap_tbl_ptr->pwrcap_cell_off[i]);
	}
	*/

	for (i = 0; i < *numAnt; i++) {
		cellOnCaps[i] = txpwrcap_tbl_ptr->pwrcap_cell_on[core*2+i];
		cellOffCaps[i] = txpwrcap_tbl_ptr->pwrcap_cell_off[core*2+i];
	}

	return TRUE;
}

int
wlmPhyTxPowerCapTableSet(
int numOfCores,	int *numAntPerCore, int *cellOnCaps, int *cellOffCaps, int numOfCaps)
{
	wl_txpwrcap_tbl_t txpwrcap_tbl;
	uint8 num_antennas = 0;
	uint8 i;

	memset(txpwrcap_tbl.pwrcap_cell_on, 127,
	               TXPWRCAP_MAX_NUM_ANTENNAS *
	       sizeof(txpwrcap_tbl.pwrcap_cell_on[0]));
	memset(txpwrcap_tbl.pwrcap_cell_off, 127,
	               TXPWRCAP_MAX_NUM_ANTENNAS *
	       sizeof(txpwrcap_tbl.pwrcap_cell_off[0]));
	memset(txpwrcap_tbl.num_antennas_per_core, 0,
	       TXPWRCAP_MAX_NUM_CORES);

	for (i = 0; i < numOfCores; i++) {
		if ((numAntPerCore[i] < 1) || (numAntPerCore[i] > 2)) {
			printf("wlmPhyTxPwerTableSet:"
			       "invalid antenna number %d for core %d."
			       "should be [1, 2]\n", numAntPerCore[i], i);
			return FALSE;
		}
		txpwrcap_tbl.num_antennas_per_core[i] = numAntPerCore[i];
		num_antennas += txpwrcap_tbl.num_antennas_per_core[i];
	}

	if (numOfCaps != num_antennas) {
			printf("wlmPhyTxPwerTableSet:"
			       "expect %d caps numbers but we only got %d\n",
			       num_antennas, numOfCaps);
			return FALSE;
	}

	for (i = 0; i < num_antennas; i++) {
		txpwrcap_tbl.pwrcap_cell_on[i] = cellOnCaps[i];
		txpwrcap_tbl.pwrcap_cell_off[i] = cellOffCaps[i];
	}

	/*
	printf("wl_txpwrcap_tbl.num_antennas_per_core:\n");
	for (i = 0; i < TXPWRCAP_MAX_NUM_CORES; i++) {
		printf("%d ", txpwrcap_tbl.num_antennas_per_core[i]);
	}

	printf("\n");
	printf("pwrcap_cell_on:\n");
	for (i = 0; i < TXPWRCAP_MAX_NUM_ANTENNAS; i++) {
		printf("%d ", txpwrcap_tbl.pwrcap_cell_on[i]);
	}
	printf("\n");
	printf("pwrcap_cell_off\n");
	for (i = 0; i < TXPWRCAP_MAX_NUM_ANTENNAS; i++) {
		printf("%d ", txpwrcap_tbl.pwrcap_cell_off[i]);
	}
	*/

	if (wlu_var_setbuf(irh, "phy_txpwrcap_tbl",  &txpwrcap_tbl, sizeof(txpwrcap_tbl)) < 0) {
		printf("wlmPhyTxPwerTableSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

/* phy_cellstatus */
int
wlmPhyCellStatusGet(int *status)
{
	if (wlu_iovar_getint(irh, "phy_cellstatus", status)) {
		printf("wlmPhyCellStatusGet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmPhyCellStatusSet(int status)
{
	if (wlu_iovar_setint(irh, "phy_cellstatus", status)) {
		printf("wlmPhyCellStatusSet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmClmLoad(char *fileName)
{
	if (process_clm_data(irh, fileName, 0) < 0) {
		printf("wlmClmLoad: download clm file %s failed. %s\n",
		       fileName, wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmClmLoadStatusGet(int *status)
{
	if (wlu_iovar_getint(irh, "clmload_status", status)) {
		printf("wlmClmLoadStatusGet: %s\n", wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmClmVersionGet(char *clmverstr, int len)
{
	void *ptr = NULL;

	if (wlu_var_getbuf_sm(irh, "clmver", NULL, 0, &ptr) < 0) {
		printf("wlmClmVersionGet: %s\n", wlmLastError());
		return FALSE;
	}

	/* printf("wlmClmVersionGet: %s\n", ptr); */
	strncpy(clmverstr, ptr, len);
	return TRUE;
}

int
wlmTxCapLoad(char *fileName)
{
	if (process_txcap_data(irh, fileName) < 0) {
		printf("wlmTxCapLoad: download txcap file %s failed. %s\n",
		       fileName, wlmLastError());
		return FALSE;
	}

	return TRUE;
}

int
wlmTxCapVersionGet(char *txcapver, int len)
{
	void *ptr = NULL;

	if (wlu_var_getbuf_sm(irh, "txcapver", NULL, 0, &ptr) < 0) {
		printf("wlmTxCapVersionGet: %s\n", wlmLastError());
		return FALSE;
	}

	/* printf("wlmTxCapVersionGet: %s\n", ptr); */
	strncpy(txcapver, ptr, len);

	return TRUE;
}

int
wlmSWDiversityStatsGet(char *stats, int length)
{
	char *statsbuf;
	void *ptr;
	wlc_swdiv_stats_t *cnt;

	if (length < (int)sizeof(wlc_swdiv_stats_t)) {
		printf("wlmSWDiversityStatsGet: "
		       "requested length %d is less than expected %d\n",
		       length, (int)sizeof(wlc_swdiv_stats_t));
		return FALSE;
	}

	if (wlu_var_getbuf_med (irh, "swdiv_stats", NULL, 0, &ptr) < 0) {
		printf("wlmSWDiversityStatsGet: %s\n", wlmLastError());
		return FALSE;
	}

	statsbuf = (char *)ptr;

	cnt = (wlc_swdiv_stats_t*)malloc(sizeof(wlc_swdiv_stats_t));
	if (cnt == NULL) {
		printf("wlmSWDiversityStatsGet: "
		       "Can not allocate %d bytes for wl swdiv stats struct\n",
		       (int)sizeof(wlc_swdiv_stats_t));
		return FALSE;
	}
	memcpy(cnt, statsbuf, sizeof(wlc_swdiv_stats_t));

	WLMPRVAL(auto_en); WLMPRVAL(active_ant); WLMPRVAL(rxcount); WLMPRNL();
	WLMPRVAL(avg_snr_per_ant0); WLMPRVAL(avg_snr_per_ant1);
	WLMPRVAL(avg_snr_per_ant2); WLMPRNL();
	WLMPRVAL(swap_ge_rxcount0); WLMPRVAL(swap_ge_rxcount1); WLMPRNL();
	WLMPRVAL(swap_ge_snrthresh0); WLMPRVAL(swap_ge_snrthresh1); WLMPRNL();
	WLMPRVAL(swap_txfail0); WLMPRVAL(swap_txfail1); WLMPRNL();
	WLMPRVAL(swap_timer0); WLMPRVAL(swap_timer1); WLMPRNL();
	WLMPRVAL(swap_alivecheck0); WLMPRVAL(swap_alivecheck1); WLMPRNL();

	stats += sprintf(stats, "\n");
	printf("stats: %s\n", stats);

	return TRUE;
}

#if defined(MACOSX)
int
wlmSetBaudRate(int speed)
{
	if (rwl_get_remote_type() != REMOTE_SERIAL) {
		printf("wlmSetBaudRate: remote type has to be serial mode\n");
		return FALSE;
	}

	if (rwl_set_baud_rate(irh, speed) < 0) {
		printf("wlmSetBaudRate: failed to set baud rate to %d\n", speed);
		return FALSE;
	}

	return TRUE;
}
#endif /* MACOSX */
