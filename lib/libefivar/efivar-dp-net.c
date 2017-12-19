/*-
 * Copyright (c) 2017 Netflix, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer
 *    in this position and unchanged.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <devinfo.h>
#include <efivar.h>
#include <errno.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_types.h>
#include <net/ethernet.h>
#include <ifaddrs.h>
#include <stdio.h>
#include <string.h>

#undef MAX
#undef MIN

#include "efichar.h"

#include "efi-osdep.h"
#include "efivar-dp.h"

#include "uefi-dplib.h"

#define MAX_DP_SANITY	4096		/* Biggest device path in bytes */
#define MAX_DP_TEXT_LEN	4096		/* Longest string rep of dp */

#define IsMacDP(dp) (DevicePathType(dp) == MESSAGING_DEVICE_PATH && \
	    DevicePathSubType(dp) == MSG_MAC_ADDR_DP)
#define ValidLen(dp) (DevicePathNodeLength(dp) >= sizeof(EFI_DEVICE_PATH_PROTOCOL) && \
	    DevicePathNodeLength(dp) < MAX_DP_SANITY)

int
efivar_device_path_to_ifnet(const_efidp dp, char **ifnet)
{
	const MAC_ADDR_DEVICE_PATH *macp;
	const_efidp walker;
	struct ifaddrs *ifap, *ifa;
	const struct sockaddr_dl *sdl;
	const caddr_t *mac;

	/*
	 * Rather than walk the device path elements to find the device_t to
	 * convert it to an ifnet interface name, instead, we cheat and search
	 * all the ifnets for a MAC address that matches. So, search the device
	 * path for a mac entry and do some sanity checks.
	 */
	if (!ValidLen(dp))
		return (EINVAL);
	walker = dp;
	while (!IsMacDP(walker) && !IsDevicePathEnd(walker)) {
		walker = (const_efidp)NextDevicePathNode(walker);
		if ((uintptr_t)walker - (uintptr_t)dp > MAX_DP_SANITY)
			return (EINVAL);
		if (!ValidLen(walker))
			return (EINVAL);
	}

	if (!IsMacDP(walker))
		return (EINVAL);
	if (DevicePathNodeLength(walker) != sizeof(MAC_ADDR_DEVICE_PATH))
		return (EINVAL);

	macp = (const MAC_ADDR_DEVICE_PATH *)walker;
	if (macp->IfType != IFT_OTHER)	/* UEFI uses IFT_OTHER, but IANA says IFT_ETHER :( */
		return (EINVAL);
	mac = (const caddr_t *)&macp->MacAddress;

	/*
	 * Now that we have a mac address, look for it in the interfaces and
	 * return a match, if we find one.
	 */
	if (getifaddrs(&ifap) != 0)
		return (errno);
	for (ifa = ifap; ifa; ifa = ifa->ifa_next) {
		if (ifa->ifa_addr->sa_family != AF_LINK)
			continue;
		sdl = (const struct sockaddr_dl *) ifa->ifa_addr;
		if (sdl->sdl_type != IFT_ETHER || sdl->sdl_alen != ETHER_ADDR_LEN)
			continue;
		if (memcmp(CLLADDR(sdl), mac, ETHER_ADDR_LEN) != 0)
			continue;
		*ifnet = strdup(ifa->ifa_name);
		freeifaddrs(ifap);
		return (0);
	}
	freeifaddrs(ifap);
	return (ENOENT);
}

struct dev_state 
{
	const char *name;
	struct devinfo_dev *found;
};

static int
find_device(struct devinfo_dev *dev, void *xstate)
{
	struct dev_state *state = xstate;

	if (strcmp(dev->dd_name, state->name) == 0) {
		state->found = dev;
		return (1);
	}
	return (devinfo_foreach_device_child(dev, find_device, xstate));
}

int
efivar_ifnet_to_device_path(const char *ifnet, efidp *dp)
{
	int rv = 0;
	struct devinfo_dev *root;
	struct dev_state state;
	struct devinfo_dev *walker;
	char *buffer, *ins, *copy;

	if (devinfo_init() != 0)
		return (ENOMEM);
	if ((root = devinfo_handle_to_device(DEVINFO_ROOT_DEVICE)) == NULL)
		return (ENOENT);
	state.name = ifnet;
	state.found = NULL;
	if (devinfo_foreach_device_child(root, find_device, &state) == 0 ||
		state.found == NULL) {
		rv = ENOENT;
		goto out;
	}

	buffer = malloc(MAX_DP_TEXT_LEN);
	ins = buffer + MAX_DP_TEXT_LEN - 1;
	*ins-- = '\0';
	walker = state.found;
	while (walker != NULL && walker != root) {
		/*
		 * ACPI node?
		 */
		hid = strstr(walker->dd_pnpinfo, "_HID=")
		printf("%s-", walker->dd_name);
		walker = devinfo_handle_to_device(walker->dd_parent);
	}
	printf("root\n");

out:
	devinfo_free();
	return (rv);
}
