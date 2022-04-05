/*
 *
 *    Copyright (c) 2021 Project CHIP Authors
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include <lib/support/CodeUtils.h>
#include <lib/support/SafeInt.h>
#include <platform/CHIPDeviceLayer.h>
#include <platform/NetworkCommissioning.h>
#include <platform/OpenThread/GenericNetworkCommissioningThreadDriver.h>
#include <platform/OpenThread/GenericThreadStackManagerImpl_OpenThread.h>
#include <platform/ThreadStackManager.h>

#include <limits>

using namespace chip;
using namespace chip::Thread;

namespace chip {
namespace DeviceLayer {
namespace NetworkCommissioning {
// NOTE: For ThreadDriver, we use two network configs, one is mSavedNetwork, and another is mStagingNetwork, during init, it will
// load the network config from thread persistent info, and loads it into both mSavedNetwork and mStagingNetwork. When updating the
// networks, all changes are made on the staging network. When validated we can commit it and save it to the persistent info

CHIP_ERROR GenericThreadDriver::Init(Internal::BaseDriver::NetworkStatusChangeCallback * statusChangeCallback)
{
    ThreadStackMgrImpl().SetNetworkStatusChangeCallback(statusChangeCallback);

    VerifyOrReturnError(ThreadStackMgrImpl().IsThreadAttached(), CHIP_NO_ERROR);
    VerifyOrReturnError(ThreadStackMgrImpl().GetThreadProvision(mStagingNetwork) == CHIP_NO_ERROR, CHIP_NO_ERROR);

    mSavedNetwork.Init(mStagingNetwork.AsByteSpan());

    return CHIP_NO_ERROR;
}

CHIP_ERROR GenericThreadDriver::Shutdown()
{
    ThreadStackMgrImpl().SetNetworkStatusChangeCallback(nullptr);
    return CHIP_NO_ERROR;
}

CHIP_ERROR GenericThreadDriver::CommitConfiguration()
{
    // Note: on AttachToThreadNetwork OpenThread will persist the networks on its own,
    // we don't have much to do for saving the networks (see Init() above,
    // we just load the saved dataset from ot instance.)
    mSavedNetwork = mStagingNetwork;
    return CHIP_NO_ERROR;
}

CHIP_ERROR GenericThreadDriver::RevertConfiguration()
{
    mStagingNetwork = mSavedNetwork;
    return CHIP_NO_ERROR;
}

Status GenericThreadDriver::AddOrUpdateNetwork(ByteSpan operationalDataset, MutableCharSpan & outDebugText,
                                               uint8_t & outNetworkIndex)
{
    ByteSpan newExtpanid;
    Thread::OperationalDataset newDataset;

    outDebugText.reduce_size(0);
    outNetworkIndex = 0;

    VerifyOrReturnError(newDataset.Init(operationalDataset) == CHIP_NO_ERROR && newDataset.IsCommissioned(), Status::kOutOfRange);
    newDataset.GetExtendedPanIdAsByteSpan(newExtpanid);

    // We only support one active operational dataset. Add/Update based on either:
    // Staging network not commissioned yet (active) or we are updating the dataset with same Extended Pan ID.
    VerifyOrReturnError(!mStagingNetwork.IsCommissioned() || MatchesNetworkId(mStagingNetwork, newExtpanid) == Status::kSuccess,
                        Status::kBoundsExceeded);

    mStagingNetwork = newDataset;
    return Status::kSuccess;
}

Status GenericThreadDriver::RemoveNetwork(ByteSpan networkId, MutableCharSpan & outDebugText, uint8_t & outNetworkIndex)
{
    outDebugText.reduce_size(0);
    outNetworkIndex = 0;

    NetworkCommissioning::Status status = MatchesNetworkId(mStagingNetwork, networkId);

    VerifyOrReturnError(status == Status::kSuccess, status);
    mStagingNetwork.Clear();

    return Status::kSuccess;
}

Status GenericThreadDriver::ReorderNetwork(ByteSpan networkId, uint8_t index, MutableCharSpan & outDebugText)
{
    outDebugText.reduce_size(0);

    NetworkCommissioning::Status status = MatchesNetworkId(mStagingNetwork, networkId);

    VerifyOrReturnError(status == Status::kSuccess, status);
    VerifyOrReturnError(index == 0, Status::kOutOfRange);

    return Status::kSuccess;
}

void GenericThreadDriver::ConnectNetwork(ByteSpan networkId, ConnectCallback * callback)
{
    NetworkCommissioning::Status status = MatchesNetworkId(mStagingNetwork, networkId);

    if (status == Status::kSuccess &&
        DeviceLayer::ThreadStackMgrImpl().AttachToThreadNetwork(mStagingNetwork.AsByteSpan(), callback) != CHIP_NO_ERROR)
    {
        status = Status::kUnknownError;
    }

    if (status != Status::kSuccess)
    {
        callback->OnResult(status, CharSpan(), 0);
    }
}

void GenericThreadDriver::ScanNetworks(ThreadDriver::ScanCallback * callback)
{
    CHIP_ERROR err = DeviceLayer::ThreadStackMgrImpl().StartThreadScan(callback);
    if (err != CHIP_NO_ERROR)
    {
        mScanStatus.SetValue(Status::kUnknownError);
        callback->OnFinished(Status::kUnknownError, CharSpan(), nullptr);
    }
    else
    {
        // OpenThread's "scan" will always success once started, so we can set the value of scan result here.
        mScanStatus.SetValue(Status::kSuccess);
    }
}

Status GenericThreadDriver::MatchesNetworkId(const Thread::OperationalDataset & dataset, const ByteSpan & networkId) const
{
    ByteSpan extpanid;

    if (!dataset.IsCommissioned())
    {
        return Status::kNetworkIDNotFound;
    }

    if (dataset.GetExtendedPanIdAsByteSpan(extpanid) != CHIP_NO_ERROR)
    {
        return Status::kUnknownError;
    }

    return networkId.data_equal(extpanid) ? Status::kSuccess : Status::kNetworkIDNotFound;
}

size_t GenericThreadDriver::ThreadNetworkIterator::Count()
{
    return driver->mStagingNetwork.IsCommissioned() ? 1 : 0;
}

bool GenericThreadDriver::ThreadNetworkIterator::Next(Network & item)
{
    if (exhausted || !driver->mStagingNetwork.IsCommissioned())
    {
        return false;
    }
    uint8_t extpanid[kSizeExtendedPanId];
    VerifyOrReturnError(driver->mStagingNetwork.GetExtendedPanId(extpanid) == CHIP_NO_ERROR, false);
    memcpy(item.networkID, extpanid, kSizeExtendedPanId);
    item.networkIDLen = kSizeExtendedPanId;
    item.connected    = false;
    exhausted         = true;

    Thread::OperationalDataset currentDataset;
    uint8_t enabledExtPanId[Thread::kSizeExtendedPanId];

    // The Thread network is not actually enabled.
    VerifyOrReturnError(ConnectivityMgrImpl().IsThreadAttached(), true);
    VerifyOrReturnError(ThreadStackMgrImpl().GetThreadProvision(currentDataset) == CHIP_NO_ERROR, true);
    // The Thread network is not enabled, but has a different extended pan id.
    VerifyOrReturnError(currentDataset.GetExtendedPanId(enabledExtPanId) == CHIP_NO_ERROR, true);
    VerifyOrReturnError(memcmp(extpanid, enabledExtPanId, kSizeExtendedPanId) == 0, true);
    // The Thread network is enabled and has the same extended pan id as the one in our record.
    item.connected = true;

    return true;
}

} // namespace NetworkCommissioning
} // namespace DeviceLayer
} // namespace chip
