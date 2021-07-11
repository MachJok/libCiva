/*
 * CDDL HEADER START
 *
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 *
 * CDDL HEADER END
*/
/*
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#ifndef	_ICAO2CC_H_
#define	_ICAO2CC_H_

#include "core.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define	icao2cc		ACFSYM(icao2cc)
API_EXPORT const char *icao2cc(const char *icao);
#define	icao2lang	ACFSYM(icao2lang)
API_EXPORT const char *icao2lang(const char *icao);

#ifdef	__cplusplus
}
#endif

#endif	/* _ICAO2CC_H_ */
