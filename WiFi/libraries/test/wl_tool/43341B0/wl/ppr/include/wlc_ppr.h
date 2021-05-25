/*
 * PHY module Power-per-rate API. Provides interface functions and definitions for opaque
 * ppr structure for use containing regulatory and board limits and tx power targets.
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
 * $Id: $
 */

#ifndef _wlc_ppr_h_
#define _wlc_ppr_h_

#include <bcmwifi_rates.h>
#include <bcmutils.h>
#ifdef BCMDRIVER
#include <osl.h>
#else
#define osl_t void
#endif

/* Helper macro */
#define PPR_CHSPEC_BW(x) (CHSPEC_IS80(x) ? WL_TX_BW_80 : \
	(CHSPEC_IS40(x) ? WL_TX_BW_40 : WL_TX_BW_20))

#ifndef BCMPHYCORENUM
#define PPR_MAX_TX_CHAINS 3
#else
#if (BCMPHYCORENUM < 1) || (BCMPHYCORENUM > 3)
#error Invalid number of PHY cores
#endif /* BCMPHYCORENUM < 1 || BCMPHYCORENUM > 3 */
#define PPR_MAX_TX_CHAINS (BCMPHYCORENUM)
#endif /* !defined(BCMPHYCORENUM) */

/* Opaque PPR data - it keeps its structure to itself */
typedef struct ppr ppr_t;


/* Power values for DSSS 1, 2, 5.5, 11 */
typedef struct ppr_dsss_rateset { int8 pwr[WL_RATESET_SZ_DSSS]; } ppr_dsss_rateset_t;

/* Power values for OFDM 6, 9, 12... 54 */
typedef struct ppr_ofdm_rateset { int8 pwr[WL_RATESET_SZ_OFDM]; } ppr_ofdm_rateset_t;

/* Power values one set of 8 HT MCS values (0-7, 8-15, etc) */
typedef struct ppr_ht_mcs_rateset { int8 pwr[WL_RATESET_SZ_HT_MCS]; } ppr_ht_mcs_rateset_t;

/* Power values one set of 10 VHT MCS values (0-9) */
typedef struct ppr_vht_mcs_rateset { int8 pwr[WL_RATESET_SZ_VHT_MCS]; } ppr_vht_mcs_rateset_t;


/* API Routines */

/* Initialization routine for opaque PPR struct */
void ppr_init(ppr_t* pprptr, wl_tx_bw_t bw);

/* Reinitialization routine for opaque PPR struct */
void ppr_clear(ppr_t* pprptr);


/* Size routine for user alloc/dealloc */
uint32 ppr_size(wl_tx_bw_t bw);

/* Size routine for user serialization alloc */
uint32 ppr_ser_size(const ppr_t* pprptr);

/* Size routine for user serialization alloc for a given bw */
uint32 ppr_ser_size_by_bw(wl_tx_bw_t bw);

/* Init allocated memory with a ppr serialization head
 * This function must be called if serialization side has different
 * compilation conditions (e.g PPR_MAX_TX_CHAINS, WL_BEAMFORMING etc.)
 */
int ppr_init_ser_mem_by_bw(uint8* pbuf, wl_tx_bw_t bw, uint32 len);

/* Init allocated memory with a ppr serialization head for the given ppr pointer
 * This function must be called if serialization side has different
 * compilation conditions (e.g PPR_MAX_TX_CHAINS, WL_BEAMFORMING etc.)
 */
int ppr_init_ser_mem(uint8* pbuf, ppr_t * ppr, uint32 len);

/* Constructor routine for opaque PPR struct */
ppr_t* ppr_create(osl_t *osh, wl_tx_bw_t bw);

/* Destructor routine for opaque PPR struct */
void ppr_delete(osl_t *osh, ppr_t* pprptr);

/* Type routine for inferring opaque structure size */
wl_tx_bw_t ppr_get_ch_bw(const ppr_t* pprptr);

/* Type routine to get ppr supported maximum bw */
wl_tx_bw_t ppr_get_max_bw(void);

/* Get the DSSS values for the given number of tx_chains and 20, 20in40, etc. */
int ppr_get_dsss(ppr_t* pprptr, wl_tx_bw_t bw, wl_tx_chains_t tx_chains,
	ppr_dsss_rateset_t* dsss);

/* Get the OFDM values for the given number of tx_chains and 20, 20in40, etc. */
int ppr_get_ofdm(ppr_t* pprptr, wl_tx_bw_t bw, wl_tx_mode_t mode, wl_tx_chains_t tx_chains,
	ppr_ofdm_rateset_t* ofdm);

/* Get the HT MCS values for the group specified by Nss, with the given bw and tx chains */
int ppr_get_ht_mcs(ppr_t* pprptr, wl_tx_bw_t bw, wl_tx_nss_t Nss, wl_tx_mode_t mode,
	wl_tx_chains_t tx_chains, ppr_ht_mcs_rateset_t* mcs);

/* Get the VHT MCS values for the group specified by Nss, with the given bw and tx chains */
int ppr_get_vht_mcs(ppr_t* pprptr, wl_tx_bw_t bw, wl_tx_nss_t Nss, wl_tx_mode_t mode,
 wl_tx_chains_t tx_chains, ppr_vht_mcs_rateset_t* mcs);


/* Routines to set target powers per rate for a whole rate set */

/* Set the DSSS values for the given number of tx_chains and 20, 20in40, etc. */
int ppr_set_dsss(ppr_t* pprptr, wl_tx_bw_t bw, wl_tx_chains_t tx_chains,
	const ppr_dsss_rateset_t* dsss);

/* Set the OFDM values for the given number of tx_chains and 20, 20in40, etc. */
int ppr_set_ofdm(ppr_t* pprptr, wl_tx_bw_t bw, wl_tx_mode_t mode, wl_tx_chains_t tx_chains,
	const ppr_ofdm_rateset_t* ofdm);

/* Set the HT MCS values for the group specified by Nss, with the given bw and tx chains */
int ppr_set_ht_mcs(ppr_t* pprptr, wl_tx_bw_t bw, wl_tx_nss_t Nss, wl_tx_mode_t mode,
	wl_tx_chains_t tx_chains, const ppr_ht_mcs_rateset_t* mcs);

/* Set the VHT MCS values for the group specified by Nss, with the given bw and tx chains */
int ppr_set_vht_mcs(ppr_t* pprptr, wl_tx_bw_t bw, wl_tx_nss_t Nss, wl_tx_mode_t mode,
	wl_tx_chains_t tx_chains, const ppr_vht_mcs_rateset_t* mcs);


/* Routines to set a whole rate set to a single target value */

/* Set the DSSS values for the given number of tx_chains and 20, 20in40, etc. */
int ppr_set_same_dsss(ppr_t* pprptr, wl_tx_bw_t bw, wl_tx_chains_t tx_chains, const int8 power);

/* Set the OFDM values for the given number of tx_chains and 20, 20in40, etc. */
int ppr_set_same_ofdm(ppr_t* pprptr, wl_tx_bw_t bw, wl_tx_mode_t mode, wl_tx_chains_t tx_chains,
	const int8 power);

/* Set the HT MCS values for the group specified by Nss, with the given bw and tx chains */
int ppr_set_same_ht_mcs(ppr_t* pprptr, wl_tx_bw_t bw, wl_tx_nss_t Nss, wl_tx_mode_t mode,
	wl_tx_chains_t tx_chains, const int8 power);


/* Set the HT MCS values for the group specified by Nss, with the given bw and tx chains */
int ppr_set_same_vht_mcs(ppr_t* pprptr, wl_tx_bw_t bw, wl_tx_nss_t Nss, wl_tx_mode_t mode,
	wl_tx_chains_t tx_chains, const int8 power);


/* Helper routines to operate on the entire ppr set */

/* Ensure no rate limit is greater than the specified maximum */
uint ppr_apply_max(ppr_t* pprptr, int8 max);

/* Ensure no rate limit is lower than the specified minimum */
uint ppr_apply_min(ppr_t* pprptr, int8 min);


/* Ensure no rate limit in this ppr set is greater than the corresponding limit in ppr_cap */
uint ppr_apply_vector_ceiling(ppr_t* pprptr, const ppr_t* ppr_cap);

/* Ensure no rate limit in this ppr set is lower than the corresponding limit in ppr_min */
uint ppr_apply_vector_floor(ppr_t* pprptr, const ppr_t* ppr_min);


/* Get the maximum power in the ppr set */
int8 ppr_get_max(ppr_t* pprptr);

/*
 * Get the minimum power in the ppr set excluding disallowed
 * rates and powers set to the minimum for the phy
 */
int8 ppr_get_min(ppr_t* pprptr, int8 floor);


/* Get the maximum power for a given bandwidth in the ppr set */
int8 ppr_get_max_for_bw(ppr_t* pprptr, wl_tx_bw_t bw);

/* Get the minimum power for a given bandwidth  in the ppr set */
int8 ppr_get_min_for_bw(ppr_t* pprptr, wl_tx_bw_t bw);


typedef void (*ppr_mapfn_t)(void *context, uint8 *a, uint8 *b);

/* Map the given function with its context value over the two power vectors */
void ppr_map_vec_dsss(ppr_mapfn_t fn, void* context, ppr_t* pprptr1,
	ppr_t* pprptr2, wl_tx_bw_t bw, wl_tx_chains_t tx_chains);

/* Map the given function with its context value over the two power vectors */
void ppr_map_vec_ofdm(ppr_mapfn_t fn, void* context, ppr_t* pprptr1,
	ppr_t* pprptr2, wl_tx_bw_t bw, wl_tx_mode_t mode, wl_tx_chains_t tx_chains);

/* Map the given function with its context value over the two power vectors */
void ppr_map_vec_ht_mcs(ppr_mapfn_t fn, void* context, ppr_t* pprptr1,
	ppr_t* pprptr2, wl_tx_bw_t bw, wl_tx_nss_t Nss, wl_tx_mode_t mode,
	wl_tx_chains_t tx_chains);

/* Map the given function with its context value over the two power vectors */
void ppr_map_vec_vht_mcs(ppr_mapfn_t fn, void* context, ppr_t* pprptr1,
	ppr_t* pprptr2, wl_tx_bw_t bw, wl_tx_nss_t Nss, wl_tx_mode_t mode,
	wl_tx_chains_t tx_chains);

/* Map the given function with its context value over the two power vectors */
void ppr_map_vec_all(ppr_mapfn_t fn, void* context, ppr_t* pprptr1, ppr_t* pprptr2);

/* Get the first index that is larger than a given power level */
int16 ppr_get_idx(ppr_t* pprptr, int8 pwr);

/* Set PPR struct to a certain power level */
void ppr_set_cmn_val(ppr_t* pprptr, int8 val);

/* Make an identical copy of a ppr structure (for ppr_bw==all case) */
void ppr_copy_struct(ppr_t* pprptr_s, ppr_t* pprptr_d);

/* make an identical copy of a ppr structure inside an WL_TX_BW_ALL type */
void ppr_copy_struct_to_all(ppr_t* pprptr_s, ppr_t* pprptr_d);

/* Subtract each power from a common value and re-store */
void ppr_cmn_val_minus(ppr_t* pprptr, int8 val);

/* Compare two ppr variables p1 and p2, save the min. value of each
 * contents to variable p1
 */
void ppr_compare_min(ppr_t* p1, ppr_t* p2);

/* compare two ppr variables p1 and p2, save the max. value of each
 * contents to variable p1
 */
void ppr_compare_max(ppr_t* p1, ppr_t* p2);

/* Serialize the contents of the opaque ppr struct.
 * Writes number of bytes copied, zero on error.
 * Returns error code, BCME_OK if successful.
 */
int ppr_serialize(const ppr_t* pprptr, uint8* buf, uint buflen, uint* bytes_copied);

/* Deserialize the contents of a buffer into an existing opaque ppr struct.
 * The ppr struct *must* be of the same type as the one that was serialized.
 * Returns error code, BCME_OK if successful.
 */
int ppr_deserialize(ppr_t* pprptr, const uint8* buf, uint buflen);

/* Deserialize the contents of a buffer into a new opaque ppr struct.
 * Creates an opaque structure referenced by *pptrptr, NULL on error.
 * Returns error code, BCME_OK if successful.
 */
int ppr_deserialize_create(osl_t *osh, const uint8* buf, uint buflen, ppr_t** pprptr);

/* Subtract a common value from each power and re-store */
void ppr_minus_cmn_val(ppr_t* pprptr, int8 val);

/* Add a common value to the given bw struct components */
void ppr_plus_cmn_val(ppr_t* pprptr, int8 val);

/* Multiply a percentage */
void ppr_multiply_percentage(ppr_t* pprptr, uint8 val);

#endif	/* _wlc_ppr_h_ */
