//
// @file nvme_pm.cpp
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
#include <IOKit/pwr_mgt/IOPM.h>
#include <kern/assert.h>

#include "Log.hpp"
#include "NVMeFixPlugin.hpp"

/**
* For Apple Controllers, AppleNVMeController toggles self-refresh for low-power states, and
* completely ignores PCI PM. For generic controllers, IONVMeController uses PCI PM and ignores
* NVMe Power Management features.
* We implement active power management by attaching our own IOService to PM root and registering
* the operational power states of the controller. We intercept activityTickle method of the relevant
* IONVMeController to tickle our service, and use NVMe power management feature to set the
* corresponding state.
* Our PM should operate transparently w.r.t PCI link PM and APST. APST is still useful, because typical
* idle intervals for APST transitions are in order of hundreds of milliseconds, while IOPM has seconds
* resolution; PCI link power management is still used by IONVMeController.
* As we never transition to idle states, we need not freeze the command queue, so we don't have to
* touch the internal state of IONVMe.
*/
bool NVMeFixPlugin::PM::init(ControllerEntry& entry, const NVMe::nvme_id_ctrl* ctrl, APSTState state) {
	auto proxy = new NVMePMProxy();
	if (!proxy)
		return false;
	if (!proxy->init()) {
		DBGLOG(Log::PM, "Failed to init IOService");
		proxy->release();
		return false;
	}
	proxy->entry = &entry;
	entry.pm = proxy;

	entry.pm->PMinit();
	auto root = IOService::getPMRootDomain();
	if (root)
		reinterpret_cast<IOService*>(root)->joinPMtree(entry.pm);
	assert(root);

	// A failed Set Features at boot still needs a retry when the controller wakes.
	if (ctrl->apsta & 1)
		entry.controller->registerInterestedDriver(entry.pm);

	return updatePolicy(entry, ctrl, state);
}

// The caller holds the entry lock. The count is only a fast negative lookup:
// the registry and entry locks still protect lifetimes on the slow path.
void NVMeFixPlugin::setActivePM(ControllerEntry& entry, bool active) {
	if (entry.activePM == active)
		return;
	entry.activePM = active;
	if (active)
		atomic_fetch_add_explicit(&activePMControllers, 1, memory_order_release);
	else
		atomic_fetch_sub_explicit(&activePMControllers, 1, memory_order_release);
}

bool NVMeFixPlugin::PM::updatePolicy(ControllerEntry& entry, const NVMe::nvme_id_ctrl* ctrl, APSTState state) {
	bool ok = true;
	if (state == APSTState::Disabled) {
		if (entry.activePM)
			return true;
		if (!entry.powerStates && !registerPowerStates(entry, ctrl))
			return false;
		if (entry.pm->makeUsable() == kIOReturnSuccess &&
			entry.pm->changePowerStateTo(1) == kIOReturnSuccess &&
			entry.pm->setIdleTimerPeriod(idlePeriod) == kIOReturnSuccess) {
			plugin.setActivePM(entry, true);
			return true;
		}
		ok = false; // Release any IOKit requests accepted before the failure.
	}

	plugin.setActivePM(entry, false);
	if (entry.powerStates) {
		// Drop the timer and both IOKit power requests left by the manual policy.
		ok &= entry.pm->setIdleTimerPeriod(0) == kIOReturnSuccess;
		ok &= entry.pm->changePowerStateTo(0) == kIOReturnSuccess;
		ok &= entry.pm->changePowerStateToPriv(0) == kIOReturnSuccess;
	}
	if (state == APSTState::Enabled && entry.restorePS0) {
		unsigned ps = 0;
		if (plugin.NVMeFeatures(entry, NVMe::NVME_FEAT_POWER_MGMT, &ps, nullptr, nullptr, true) != kIOReturnSuccess)
			return false;
		entry.restorePS0 = false;
	}
	return ok;
}

bool NVMeFixPlugin::PM::registerPowerStates(ControllerEntry& entry, const NVMe::nvme_id_ctrl* ctrl) {
	unsigned op {};
	
	for (int state = ctrl->npss; state >= 0; state--)
		if (!(ctrl->psd[state].flags & NVMe::NVME_PS_FLAGS_NON_OP_STATE))
			op++;

	entry.nstates = op + 1; // IOPM state zero precedes the operational states.
	entry.powerStates = new IOPMPowerState[entry.nstates];
	if (!entry.powerStates) {
		SYSLOG(Log::PM, "Failed to allocate power state buffer");
		return false;
	}

	if (op <= 1)
		SYSLOG(Log::PM, "Controller declares too few operational power states");
	DBGLOG(Log::PM, "npss 0x%x", ctrl->npss);
	
	entry.powerStates[0] = {kIOPMPowerStateVersion1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	/**
	 * Linux has a different model: they save state upon suspend and switch to
	 * npss state; upon resume, they restore last state. They also reset ps
	 * when they fail to set or get it, but it's unclear how that would ever
	 * occur given that NVMe 1.4 spec 5.21.1.2 only mentions error when trying
	 * to set unsupported state.
	 */
	size_t idx {1};
	for (int state = ctrl->npss; state >= 0; state--) {
		if (ctrl->psd[state].flags & NVMe::NVME_PS_FLAGS_NON_OP_STATE)
			continue;

		entry.operationalPS[idx - 1] = static_cast<uint8_t>(state);
		auto& ps = entry.powerStates[idx];
		ps = {kIOPMPowerStateVersion1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		/**
		 * We shouldn't have any IOPM clients that require power, so don't
		 * set outputPowerCharacter.
		 */
//		ps.outputPowerCharacter = kIOPMPowerOn;
		ps.inputPowerRequirement = kIOPMPowerOn;

		/* In non-operational state device is not usable and we may sleep */
		ps.capabilityFlags |= kIOPMPreventIdleSleep;
		ps.capabilityFlags |= kIOPMDeviceUsable;
		DBGLOG(Log::PM, "Setting ps %u capabilityFlags 0x%x", idx, ps.capabilityFlags);
		idx++;
	}

	assert(idx == entry.nstates);
	DBGLOG(Log::PM, "Publishing %u states", entry.nstates);
	
	auto status = entry.pm->registerPowerDriver(entry.pm, entry.powerStates, entry.nstates);
	if (status != kIOReturnSuccess) {
		SYSLOG(Log::PM, "registerPowerDriver failed with 0x%x", status);
		delete[] entry.powerStates;
		entry.powerStates = nullptr;
		return false;
	}
	return true;
}

OSDefineMetaClassAndStructors(NVMePMProxy, IOService);

void NVMePMProxy::free() {
	assert(!entry || entry->removed);
	delete entry;
	IOService::free();
}

IOReturn NVMePMProxy::setPowerState(unsigned long powerStateOrdinal, IOService *whatDevice) {
	DBGLOG(Log::PM, "setPowerState %lu", powerStateOrdinal);

	if (powerStateOrdinal == 0)
		return kIOPMAckImplied;

	auto& plugin = NVMeFixPlugin::globalPlugin();

	/* It's ok to skip active PM */
	if (IOLockTryLock(entry->lck)) {
		if (!entry->activePM || powerStateOrdinal >= entry->nstates) {
			IOLockUnlock(entry->lck);
			return kIOPMAckImplied;
		}
		unsigned dword11 = entry->operationalPS[powerStateOrdinal - 1];
		auto ctrl = static_cast<const NVMe::nvme_id_ctrl*>(entry->identify->getBytesNoCopy());
		uint32_t res {};
		auto ret = plugin.NVMeFeatures(*entry, NVMe::NVME_FEAT_POWER_MGMT, nullptr, nullptr, &res,
			false);
		res &= 0x1F;

		DBGLOG(Log::PM, "Current ps 0x%x, proposed 0x%x", res, dword11);

		if (ret != kIOReturnSuccess) {
			SYSLOG(Log::PM, "Failed to get power state");
		} else if (res != dword11 && ctrl && res <= ctrl->npss && !(ctrl->psd[res].flags & NVMe::NVME_PS_FLAGS_NON_OP_STATE)) {
			DBGLOG(Log::PM, "Setting power state 0x%x", dword11);

			ret = plugin.NVMeFeatures(*entry, NVMe::NVME_FEAT_POWER_MGMT, &dword11, nullptr, nullptr,
										   true);
			if (ret != kIOReturnSuccess)
				SYSLOG(Log::PM, "Failed to set power state");
			else
				entry->restorePS0 = dword11 != 0;
		}

		IOLockUnlock(entry->lck);
	} else
		DBGLOG(Log::PM, "Failed to obtain entry lock");

	/**
	 * FIXME: We should return entry + exit + switching overhead latency here, but at least in my tests
	 * it is always 0.
	 */
	return kIOPMAckImplied; /* No real way to signal error (not that we expect any) */
}

IOReturn NVMePMProxy::powerStateDidChangeTo(IOPMPowerFlags capabilities, unsigned long stateNumber,
											IOService *whatDevice) {
	DBGLOG(Log::PM, "powerStateDidChangeTo 0x%x", stateNumber);

	/* FIXME: Should we ignore PAUSE->ACTIVE transition? */
	if (!(capabilities & kIOPMDeviceUsable)) {
		DBGLOG(Log::PM, "Ignoring transition to non-usable state 0x%x", stateNumber);
		return kIOPMAckImplied;
	}

	auto& plugin = NVMeFixPlugin::globalPlugin();

	/* We only get once chance after wake, so we insist on getting to critical section */
	IOLockLock(entry->lck);
	NVMe::nvme_id_ctrl* identify {};

	if (entry->removed || entry->controller != whatDevice) {
		DBGLOG(Log::PM, "Power state change for irrelevant device %s",
			   whatDevice->getMetaClass()->getClassName());
		goto done;
	}

	assert(entry->identify);
	identify = static_cast<decltype(identify)>(entry->identify->getBytesNoCopy());
	if (!identify) {
		DBGLOG(Log::PM, "Failed to get identify bytes");
		goto done;
	} else if (!plugin.PM.updatePolicy(*entry, identify, plugin.applyAPST(*entry, identify))) {
		SYSLOG(Log::PM, "Failed to restore power policy");
	}

done:
	IOLockUnlock(entry->lck);

	return IOPMAckImplied;
}

bool NVMeFixPlugin::PM::solveSymbols(KernelPatcher& kp) {
	auto idx = plugin.kextInfo.loadIndex;
	bool ret =
	plugin.kextFuncs.IONVMeController.activityTickle.routeVirtual(kp, idx,
													"__ZTV16IONVMeController", 249, activityTickle);

	return ret;
}

bool NVMeFixPlugin::PM::activityTickle(void* controller, unsigned long type, unsigned long stateNumber) {
	auto& plugin = NVMeFixPlugin::globalPlugin();

	if (atomic_load_explicit(&plugin.activePMControllers, memory_order_acquire) != 0) {
		IOLockLock(plugin.lck);
		auto entry = plugin.entryForController(static_cast<IOService*>(controller));
		if (entry && (!entry->processed || !IOLockTryLock(entry->lck)))
			entry = nullptr;
		IOLockUnlock(plugin.lck);

		if (entry) {
			if (entry->activePM)
				entry->pm->activityTickle(kIOPMSuperclassPolicy1, entry->nstates - 1);
			IOLockUnlock(entry->lck);
		}
	}

	return plugin.kextFuncs.IONVMeController.activityTickle(controller, type, stateNumber);
}
