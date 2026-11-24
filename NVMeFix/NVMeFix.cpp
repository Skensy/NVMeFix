//
// @file NVMeFix.cpp
//
// NVMeFix
//
// Copyright © 2019 acidanthera. All rights reserved.
//
// This program and the accompanying materials
// are licensed and made available under the terms and conditions of the BSD License
// which accompanies this distribution.  The full text of the license may be found at
// http://opensource.org/licenses/bsd-license.php
// THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
// WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

#include <IOKit/IOService.h>
#include <IOKit/IODMACommand.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <kern/assert.h>
#include <libkern/c++/OSMetaClass.h>

#include <Headers/kern_api.hpp>
#include <Headers/kern_disasm.hpp>
#include <Headers/kern_iokit.hpp>
#include <Headers/hde64.h>
#include <Headers/kern_util.hpp>
#include <Headers/plugin_start.hpp>

#include "Log.hpp"
#include "NVMeFixPlugin.hpp"

static NVMeFixPlugin plugin;

NVMeFixPlugin& NVMeFixPlugin::globalPlugin() {
	return plugin;
}

/**
 * This may be invoked before or after we get IOBSD mount notification, so in the both functions we
 * attempt to solve symbols and handle the controllers.
 */
void NVMeFixPlugin::processKext(void* that, KernelPatcher& patcher, size_t index, mach_vm_address_t,
								size_t) {
	auto plugin = static_cast<NVMeFixPlugin*>(that);
	assert(plugin);

	if (index != plugin->kextInfo.loadIndex)
		return;

	DBGLOG(Log::Plugin, "processKext %s", plugin->kextInfo.id);

	if (plugin->solveSymbols(patcher)) {
		atomic_store_explicit(&plugin->solvedSymbols, true, memory_order_release);
		plugin->handleControllers();
	}
}

bool NVMeFixPlugin::solveSymbols(KernelPatcher& kp) {
	auto idx = plugin.kextInfo.loadIndex;
	bool res = true;
	res &= (kextFuncs.IONVMeController.IssueIdentifyCommandNew.solve(kp, idx) ||
			kextFuncs.IONVMeController.IssueIdentifyCommand.solve(kp, idx)) &&
	kextFuncs.IONVMeController.ProcessSyncNVMeRequest.solve(kp, idx) &&
	(kextFuncs.IONVMeController.GetRequestNew.solve(kp, idx) ||
	 kextFuncs.IONVMeController.GetRequest.solve(kp, idx)) &&
	kextFuncs.AppleNVMeRequest.BuildCommandGetFeatures.solve(kp, idx) &&
	kextFuncs.AppleNVMeRequest.BuildCommandSetFeaturesCommon.solve(kp, idx) &&
	kextFuncs.IONVMeController.ReturnRequest.solve(kp, idx) &&
	kextFuncs.AppleNVMeRequest.GetStatus.solve(kp, idx) &&
	kextFuncs.AppleNVMeRequest.GetOpcode.solve(kp, idx) &&
	kextFuncs.AppleNVMeRequest.GenerateIOVMSegments.solve(kp, idx) &&
	kextFuncs.IONVMeController.FilterInterruptRequest.solve(kp, idx);

	/* mov eax, [rdi+0xA8] */
	res &= kextMembers.AppleNVMeRequest.result.fromFunc(kextFuncs.AppleNVMeRequest.GetStatus.fptr,
														 0x8b, 0, 7, 4) &&
	/* movzx eax, byte ptr [rdi+0x10A] */
		kextMembers.AppleNVMeRequest.command.fromFunc(kextFuncs.AppleNVMeRequest.GetOpcode.fptr,
												  0xf, 0, 7) &&
	/* mov [r14+0xC0], r15 (14.0+) or mov [rbx+0xC0], r15 (11.3-13.x) or mov [rbx+0xC0], r12 (<=11.2) */
		(kextFuncs.IONVMeController.IssueIdentifyCommandNew.fptr ?
		kextMembers.AppleNVMeRequest.prpDescriptor.fromFunc(kextFuncs.IONVMeController.IssueIdentifyCommandNew.fptr, 0x89, 7, getKernelVersion() >= KernelVersion::Sonoma ? 14 : 3) :
		 kextMembers.AppleNVMeRequest.prpDescriptor.fromFunc(kextFuncs.IONVMeController.IssueIdentifyCommand.fptr, 0x89, 4, 3));

	/* cmp byte ptr [rdi+269h], 0 */
	kextMembers.IONVMeController.ANS2MSIWorkaround.fromFunc(kextFuncs.IONVMeController.FilterInterruptRequest.fptr,
																0x80, 7, 7, 0, 32);

	if (res)
		kextMembers.AppleNVMeRequest.controller.offs = kextMembers.AppleNVMeRequest.result.offs - 12;

	res &= PM.solveSymbols(kp);
	if (!res)
		DBGLOG(Log::Plugin, "Failed to solve symbols");
	return res;
}

/**
 * This handler will be invoked when a media (whole disk or a partition) BSD node becomes registered.
 * We need to do two things now:
 * 1. Discover any undetected NVMe controllers.
 * 2. Try and solve symbols. If the relevant partition for symbol solving is not available, the call
 * will fail and we may succeed at next mount.
 * If we have all the symbols ready, we can proceed working with controllers.
 */
bool NVMeFixPlugin::matchingNotificationHandler(void* that, void* , IOService* service,
												IONotifier* notifier) {
	auto plugin = static_cast<NVMeFixPlugin*>(that);
	assert(plugin);
	assert(service);

	IOLockLock(plugin->lck);

	DBGLOG("nvmef", "matchingNotificationHandler for %s", service->getName());

	auto parent = service->getProvider();

	/* Typical depth is 9 on real setups */
	for (int i = 0; parent && i < controllerSearchDepth; i++) {
//		DBGLOG("nvmef", "Parent %s", parent->getName());

		if (parent->metaCast("IONVMeController")) {
			auto existing = plugin->entryForController(parent);
			if (existing && !parent->isInactive() && IOLockTryLock(existing->lck)) {
				if (existing->retryIdentify) {
					existing->retryIdentify = false;
					existing->processed = false;
				}
				IOLockUnlock(existing->lck);
			}
			if (!existing && !parent->isInactive()) {
				auto entry = new ControllerEntry(parent);
				if (!entry) {
					SYSLOG(Log::Plugin, "Failed to allocate ControllerEntry memory");
					break;
				}
				if (!entry->lck || !plugin->controllers.push_back(entry)) {
					SYSLOG(Log::Plugin, "Failed to insert ControllerEntry memory");
					delete entry;
					break;
				}
				break;
			}
		}

		parent = parent->getProvider();
	}

//	DBGLOG("nvmef", "Discovered %u controllers", plugin->controllers.size());

	IOLockUnlock(plugin->lck);

	if (atomic_load_explicit(&plugin->solvedSymbols, memory_order_acquire))
		plugin->handleControllers();

	return true;
}

void NVMeFixPlugin::handleControllers() {
	for (;;) {
		ControllerEntry* entry = nullptr;
		IOLockLock(lck);
		for (size_t i = 0; i < controllers.size(); ++i) {
			if (!controllers[i]->processed) {
				entry = controllers[i];
				// An unclaimed entry has no PM proxy and is skipped by activityTickle.
				IOLockLock(entry->lck);
				entry->processed = true;
				break;
			}
		}
		IOLockUnlock(lck);
		if (!entry)
			return;

		handleController(*entry);
		IOLockUnlock(entry->lck);
	}
}

void NVMeFixPlugin::forceEnableASPM(IOService *controller) {
	// Use virtual PCI accessors so OpenCore can link without direct IOPCIFamily imports.
	auto pciParent = [](IORegistryEntry *entry, const IORegistryPlane *plane) {
		auto parent = entry->getParentEntry(plane);
		return parent ? static_cast<IOPCIDevice *>(parent->metaCast("IOPCIDevice")) : nullptr;
	};
	auto ssd = pciParent(controller, gIOServicePlane);
	auto bridge = ssd ? pciParent(ssd, gIODTPlane) : nullptr;
	if (!bridge)
		return;

	// Both partners must support L1. Enable the upstream end first.
	IOPCIDevice *partners[] {bridge, ssd};
	UInt8 offsets[2] {};
	for (size_t i = 0; i < arrsize(partners); ++i) {
		auto pci = partners[i];
		if (!pci->findPCICapability(kIOPCICapabilityIDPCIExpress, &offsets[i]))
			return;
		if (offsets[i] > 0xEC)
			return;
		auto space = pci->space;
		space.es.registerNumExtended = 0;
		auto capabilities = pci->configRead32(space, offsets[i] + 0x0C);
		if (capabilities == UINT32_MAX || !(capabilities & (1U << 11)))
			return;
	}

	for (size_t i = 0; i < arrsize(partners); ++i) {
		auto pci = partners[i];
		auto space = pci->space;
		space.es.registerNumExtended = 0;
		UInt8 offset = offsets[i] + 0x10;
		auto control = pci->configRead16(space, offset);
		if (control == UINT16_MAX)
			return;
		if (!(control & ASPM_L1EntryEnabled))
			pci->configWrite16(space, offset, control | ASPM_L1EntryEnabled);
		control = pci->configRead16(space, offset);
		if (control == UINT16_MAX || !(control & ASPM_L1EntryEnabled))
			return;
		pci->setProperty("pci-aspm-custom", control, 32);
	}
}

void NVMeFixPlugin::handleController(ControllerEntry& entry) {
	assert(entry.controller);

	uint32_t vendor {};
	propertyFromParent(entry.controller, "vendor-id", vendor);
	if (vendor == 0x106b || entry.controller->metaCast("AppleNVMeController")) {
		SYSLOG(Log::Plugin, "Ignoring Apple controller");
		return;
	}

	/**
	 * Force enable ASPM when the user cannot provide device properties.
	 */
	if (checkKernelArgument("-nvmefaspm"))
		forceEnableASPM(entry.controller);

	/**
	 * Force enable ANS2MSIWorkaround.
	 *
	 * We would often get a panic with I/O Read command timeout on VMware and Samsung PM981.
	 * Some investigation showed that CQ head entry phase and CQ phase would mismatch.
	 * This implies there is a race such that CQ head gets updated to point to an entry with inverted
	 * phase.
	 * Since FilterIRQ detects phase mismatch, it does not schedule HandleIRQ at the workloop of NVMe
	 * controller, so a request is never handled and we get a timeout. If Filter and Handle IRQ calls can
	 * possibly race, this may happen because HandleIRQ does not manage to update CQ phase in time before
	 * FilterIRQ phase check is scheduled, observing the old phase. This is the case in situation where
	 * two FilterIRQ calls happen with no HandleIRQ in between where the controller was too slow to
	 * notice INTMS being set.
	 * Normally, IRQ is masked just before HandleIRQ is scheduled in FilterIRQ, and unmasked when
	 * HandleIRQ is done. IONVMeController::ANS2MSIWorkaround forces IRQ to be masked at the very
	 * start of FilterIRQ instead so that FilterIRQ does not race with itself. This seems to eliminate
	 * the timeouts.
	 **/
	if (kextMembers.IONVMeController.ANS2MSIWorkaround.has()) {
		kextMembers.IONVMeController.ANS2MSIWorkaround.get(entry.controller) = 1;
	} else {
		DBGLOG(Log::Plugin, "Ignoring ANS2 workaround patch on newer system");
	}

	/* First get quirks based on PCI device */
	entry.quirks = NVMe::quirksForController(entry.controller);
	propertyFromParent(entry.controller, "ps-max-latency-us", entry.ps_max_latency_us);

	IOBufferMemoryDescriptor* identifyDesc {nullptr};

	if (identify(entry, identifyDesc) != kIOReturnSuccess || !identifyDesc) {
		entry.retryIdentify = true;
		SYSLOG(Log::Plugin, "Failed to identify controller");
		return;
	}

	auto ctrl = reinterpret_cast<NVMe::nvme_id_ctrl*>(identifyDesc->getBytesNoCopy());
	if (!ctrl) {
		DBGLOG(Log::Plugin, "Failed to get identify buffer bytes");
		if (identifyDesc)
			identifyDesc->release();
		return;
	}

	entry.identify = identifyDesc;
	if (ctrl->npss >= arrsize(ctrl->psd)) {
		SYSLOG(Log::Plugin, "Invalid NPSS");
		return;
	}

	/* Get additional quirks based on identify data */
	entry.quirks |= NVMe::quirksForController(ctrl->vid, ctrl->mn, ctrl->fr);

	entry.controller->setProperty("quirks", entry.quirks, 8 * sizeof(entry.quirks));

#ifdef DEBUG
	char mn[40];
	lilu_os_memcpy(mn, ctrl->mn, sizeof(mn));
	mn[sizeof(mn) - 1] = '\0';

	DBGLOG(Log::Plugin, "Identified model %s (vid 0x%x)", mn, ctrl->vid);
#endif

	auto state = applyAPST(entry, ctrl);
	if (!PM.init(entry, ctrl, state))
		SYSLOG(Log::PM, "Failed to initialise power management");
}

IOReturn NVMeFixPlugin::identify(ControllerEntry& entry, IOBufferMemoryDescriptor*& desc) {
	IOReturn ret = kIOReturnSuccess;

	uint8_t* data {nullptr};
	bool prepared {false};

	desc = IOBufferMemoryDescriptor::withCapacity(sizeof(NVMe::nvme_id_ctrl), kIODirectionIn);

	if (!desc) {
		SYSLOG(Log::Plugin, "Failed to init descriptor");
		ret = kIOReturnNoResources;
		goto fail;
	}
	data = static_cast<uint8_t*>(desc->getBytesNoCopy());
	if (!data) {
		ret = kIOReturnNoResources;
		goto fail;
	}
	memset(data, '\0', desc->getLength());

	ret = desc->prepare();
	if (ret != kIOReturnSuccess) {
		SYSLOG(Log::Plugin, "Failed to prepare descriptor");
		goto fail;
	}
	prepared = true;

	if (kextFuncs.IONVMeController.IssueIdentifyCommandNew.fptr)
		ret = kextFuncs.IONVMeController.IssueIdentifyCommandNew(entry.controller, desc, 0, false);
	else
		ret = kextFuncs.IONVMeController.IssueIdentifyCommand(entry.controller, desc, nullptr, 0);
	if (ret != kIOReturnSuccess) {
		SYSLOG(Log::Plugin, "issueIdentifyCommand failed");
		goto fail;
	}

fail:
	if (prepared)
		desc->complete();
	if (ret != kIOReturnSuccess && desc) {
		desc->release();
		desc = nullptr;
	}
	return ret;
}

IOReturn NVMeFixPlugin::NVMeFeatures(ControllerEntry& entry, unsigned fid, unsigned* dword11,
										IOBufferMemoryDescriptor* desc, uint32_t* res, bool set) {
	auto ret = kIOReturnSuccess;

	bool prepared {false};

	if (desc) {
		ret = desc->prepare();
		prepared = ret == kIOReturnSuccess;
	}

	if (!desc || prepared) {
		void *req;
		if (kextFuncs.IONVMeController.GetRequestNew.fptr)
			req = kextFuncs.IONVMeController.GetRequestNew(entry.controller, 1, 0); /* Set 0b10 to tickle */
		else
			req = kextFuncs.IONVMeController.GetRequest(entry.controller, 1); /* Set 0b10 to tickle */

		if (!req) {
			DBGLOG(Log::Feature, "IONVMeController::GetRequest failed");
			ret = kIOReturnNoResources;
		} else if (desc)
			ret = reinterpret_cast<IODMACommand*>(req)->setMemoryDescriptor(desc, false);

		if (ret == kIOReturnSuccess) {
			bool dmaPrepared = false;
			if (set)
				kextFuncs.AppleNVMeRequest.BuildCommandSetFeaturesCommon(req, fid);
			else
				kextFuncs.AppleNVMeRequest.BuildCommandGetFeatures(req, fid);

			if (dword11)
				kextMembers.AppleNVMeRequest.command.get(req).features.dword11 = *dword11;
			if (desc) {
				kextMembers.AppleNVMeRequest.prpDescriptor.get(req) = desc;
				ret = reinterpret_cast<IODMACommand*>(req)->prepare(0, desc->getLength());
				dmaPrepared = ret == kIOReturnSuccess;
			}

			if (ret != kIOReturnSuccess)
				DBGLOG(Log::Feature, "Failed to prepare DMA command");
			else {
				if (desc)
					ret = kextFuncs.AppleNVMeRequest.GenerateIOVMSegments(req, 0,
																		  desc->getLength());

				if (ret != kIOReturnSuccess)
					DBGLOG(Log::Feature, "Failed to generate IO VM segments");
				else {
					kextMembers.AppleNVMeRequest.controller.get(req) = entry.controller;

					ret = kextFuncs.IONVMeController.ProcessSyncNVMeRequest(entry.controller,
																			req);
					if (ret != kIOReturnSuccess)
						DBGLOG(Log::Feature, "ProcessSyncNVMeRequest failed");
					else if (res)
						*res = kextMembers.AppleNVMeRequest.result.get(req);
				}
			}
			if (dmaPrepared)
				reinterpret_cast<IODMACommand*>(req)->complete();
		}
		if (req)
			kextFuncs.IONVMeController.ReturnRequest(entry.controller, req);
	} else
		SYSLOG(Log::Feature, "Failed to prepare buffer");

	if (desc && prepared)
		desc->complete();

	return ret;
}

bool NVMeFixPlugin::terminatedNotificationHandler(void* that, void* , IOService* service,
												IONotifier* notifier) {
	auto plugin = static_cast<NVMeFixPlugin*>(that);
	assert(plugin);
	assert(service && service->metaCast("IONVMeController"));

	ControllerEntry* entry = nullptr;
	IOLockLock(plugin->lck);
	for (size_t i = 0; i < plugin->controllers.size(); ++i) {
		if (plugin->controllers[i]->controller == service) {
			entry = plugin->controllers[i];
			plugin->controllers.erase(i);
			break;
		}
	}
	IOLockUnlock(plugin->lck);

	if (entry) {
		// Drain setup or a tickle that acquired the entry before removal.
		IOLockLock(entry->lck);
		entry->removed = true;
		plugin->setActivePM(*entry, false);
		auto pm = entry->pm;
		IOLockUnlock(entry->lck);
		if (pm) {
			service->deRegisterInterestedDriver(pm);
			pm->PMstop();
			pm->release();
		} else {
			delete entry;
		}
	}

	return false;
}

/**
 * NOTE: We are in kmod context, not IOService. This works fine as long as we publish our personality
 * in Info.plist to match something in ioreg, but specify a non-existing IOClass so that IOKit attempts
 * to load us anyway. It is otherwise unsafe to use matching notifications from kmod when we have a
 * living IOService.
 */
void NVMeFixPlugin::init() {
	LiluAPI::Error err;

	if (!(lck = IOLockAlloc())) {
		SYSLOG(Log::Plugin, "Failed to alloc lock");
		goto fail;
	}

	atomic_store_explicit(&solvedSymbols, false, memory_order_relaxed);

	matchingNotifier = IOService::addMatchingNotification(gIOPublishNotification,
							IOService::serviceMatching("IOMediaBSDClient"),
							matchingNotificationHandler,
						    this);
	if (!matchingNotifier) {
		SYSLOG(Log::Plugin, "Failed to register for matching notification");
		goto fail;
	}

	terminationNotifier = IOService::addMatchingNotification(gIOTerminatedNotification,
							IOService::serviceMatching("IONVMeController"),
							terminatedNotificationHandler,
						    this);
	if (!terminationNotifier) {
		SYSLOG(Log::Plugin, "Failed to register for termination notification");
		goto fail;
	}

	DBGLOG(Log::Plugin, "Registered for matching notifications");

	err = lilu.onKextLoad(&kextInfo, 1, NVMeFixPlugin::processKext, this);
	if (err != LiluAPI::Error::NoError) {
		SYSLOG(Log::Plugin, "Failed to register kext load cb");
		goto fail;
	}

	return;
fail:
	if (matchingNotifier)
		matchingNotifier->remove();
	if (terminationNotifier)
		terminationNotifier->remove();
	for (size_t i = 0; i < controllers.size(); ++i)
		delete controllers[i];
	controllers.deinit();
	if (lck)
		IOLockFree(lck);
}

void NVMeFixPlugin::deinit() {
	/* This kext is not unloadable */
	panic("nvmef: deinit called");
}

NVMeFixPlugin::ControllerEntry* NVMeFixPlugin::entryForController(IOService* controller) const {
	for (size_t i = 0; i < controllers.size(); i++)
		if (controllers[i]->controller == controller)
			return controllers[i];
	return nullptr;
}

static const char *bootargOff[] {
	"-nvmefoff"
};

static const char *bootargDebug[] {
	"-nvmefdbg"
};

PluginConfiguration ADDPR(config) {
	xStringify(PRODUCT_NAME),
	parseModuleVersion(xStringify(MODULE_VERSION)),
	LiluAPI::AllowNormal | LiluAPI::AllowInstallerRecovery,
	bootargOff,
	arrsize(bootargOff),
	bootargDebug,
	arrsize(bootargDebug),
	nullptr,
	0,
	KernelVersion::Mojave,
	KernelVersion::Tahoe,
	[]() {
		NVMeFixPlugin::globalPlugin().init();
	}
};
