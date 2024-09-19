/*
 *
 *    Copyright (c) 2023-2024 Project CHIP Authors
 *    All rights reserved.
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

#include "AppTask.h"
#include "Device.h"
#include "PWMManager.h"
#include <app-common/zap-generated/callback.h>

#include <app-common/zap-generated/attributes/Accessors.h>
#include <app/reporting/reporting.h>
#include <app/util/endpoint-config-api.h>
#include <lib/support/ZclString.h>

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

#define LIGHT1_ENDPIONT 0x0003
/*#define LIGHT2_ENDPIONT 0x0004*/
#define LIGHT3_ENDPIONT 0x0005
#define LIGHT4_ENDPIONT 0x0006
#define LIGHT2_ENDPIONT 0x0007
#define SENSOR1_ENDPIONT 0x0008

namespace {
bool sTurnedOn;
uint8_t sLevel;
int light1_idx;
/*
int light2_idx;
int light3_idx;
int light4_idx;
int sensor1_idx;
*/
} // namespace

AppTask AppTask::sAppTask;
#include <app/InteractionModelEngine.h>

#define debug_msg(a, ...) printk("[ D ] %d %s(): " a, __LINE__, __func__, ##__VA_ARGS__)

int AddDeviceEndpoint(Device * dev, EmberAfEndpointType * ep, const Span<const EmberAfDeviceType> & deviceTypeList,
                      const Span<DataVersion> & dataVersionStorage, chip::EndpointId parentEndpointId);
CHIP_ERROR RemoveDeviceEndpoint(Device * dev);
void HandleDeviceTempSensorStatusChanged(DeviceTempSensor * dev, DeviceTempSensor::Changed_t itemChangedMask);
Protocols::InteractionModel::Status HandleReadTempMeasurementAttribute(DeviceTempSensor * dev, chip::AttributeId attributeId,
                                                                       uint8_t * buffer, uint16_t maxReadLength);

static const int kNodeLabelSize = 32;
// Current ZCL implementation of Struct uses a max-size array of 254 bytes
static const int kDescriptorAttributeArraySize = 254;

static EndpointId gCurrentEndpointId;
static EndpointId gFirstDynamicEndpointId;

static Device * gDevices[CHIP_DEVICE_CONFIG_DYNAMIC_ENDPOINT_COUNT]; // number of dynamic endpoints count

const int16_t minMeasuredValue     = -27315;
const int16_t maxMeasuredValue     = 32766;
const int16_t initialMeasuredValue = 100;

// 5 Bridged devices
static Device gLight1("Light 1", "Office");
static Device gLight2("Light 2", "Office");
static Device gLight3("Light 3", "Kitchen");
static Device gLight4("Light 4", "Kitchen");
static DeviceTempSensor TempSensor1("TempSensor 1", "Office", minMeasuredValue, maxMeasuredValue, initialMeasuredValue);

// (taken from src/app/zap-templates/zcl/data-model/chip/matter-devices.xml)
#define DEVICE_TYPE_BRIDGED_NODE 0x0013
// (taken from lo-devices.xml)
#define DEVICE_TYPE_LO_ON_OFF_LIGHT 0x0100
#define DEVICE_TYPE_DIMMALBE_LIGHT 0x0101
#define DEVICE_TYPE_ROOT_NODE 0x0016
#define DEVICE_TYPE_BRIDGE 0x000e
#define DEVICE_TYPE_TEMP_SENSOR 0x0302
// Device Version for dynamic endpoints:
#define DEVICE_VERSION_DEFAULT 1

// from src/app/zap-templates/zcl/data-model/chip/level-control-cluster.xml
#define DEVICE_TYPE_LEVEL_CONTROL_LIGHT 0x0008
#define DEVICE_TYPE_COLOR_CONTROL_LIGHT 0x0300

/* REVISION definitions:
 */

#define ZCL_DESCRIPTOR_CLUSTER_REVISION (1u)
#define ZCL_BRIDGED_DEVICE_BASIC_INFORMATION_CLUSTER_REVISION (2u)
#define ZCL_FIXED_LABEL_CLUSTER_REVISION (1u)
#define ZCL_ON_OFF_CLUSTER_REVISION (4u)
#define ZCL_LEVEL_CONTROL_CLUSTER_REVISION (6u)
#define ZCL_COLOR_CONTROL_CLUSTER_REVISION (7u)
#define ZCL_TEMPERATURE_SENSOR_CLUSTER_REVISION (4u)
#define ZCL_BRIDGED_DEVICE_BASIC_INFORMATION_FEATURE_MAP (0u)
#define ZCL_TEMPERATURE_SENSOR_FEATURE_MAP (0u)

/* BRIDGED DEVICE ENDPOINT: contains the following clusters:
   - On/Off
   - Descriptor
   - Bridged Device Basic Information
*/

// Declare On/Off cluster attributes
DECLARE_DYNAMIC_ATTRIBUTE_LIST_BEGIN(onOffAttrs)
DECLARE_DYNAMIC_ATTRIBUTE(Clusters::OnOff::Attributes::OnOff::Id, BOOLEAN, 1, ATTRIBUTE_MASK_WRITABLE), /* on/off */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::OnOff::Attributes::ClusterRevision::Id, INT16U, ZCL_ON_OFF_CLUSTER_REVISION, 0),
    DECLARE_DYNAMIC_ATTRIBUTE_LIST_END();

// Declare level control cluster attributes
DECLARE_DYNAMIC_ATTRIBUTE_LIST_BEGIN(levelControlAttrs)
DECLARE_DYNAMIC_ATTRIBUTE(Clusters::LevelControl::Attributes::CurrentLevel::Id, INT8U, 1,
                          ATTRIBUTE_MASK_WRITABLE), /* CurrentLevel */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::LevelControl::Attributes::RemainingTime::Id, INT16U, 1, ATTRIBUTE_MASK_WRITABLE),
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::LevelControl::Attributes::MinLevel::Id, INT8U, 1, ATTRIBUTE_MASK_WRITABLE),
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::LevelControl::Attributes::MaxLevel::Id, INT8U, 1, ATTRIBUTE_MASK_WRITABLE),
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::LevelControl::Attributes::CurrentFrequency::Id, INT16U, 1, ATTRIBUTE_MASK_WRITABLE),
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::LevelControl::Attributes::MinFrequency::Id, INT16U, 1, ATTRIBUTE_MASK_WRITABLE),
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::LevelControl::Attributes::MaxFrequency::Id, INT16U, 1, ATTRIBUTE_MASK_WRITABLE),
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::LevelControl::Attributes::Options::Id, BITMAP8, 1, ATTRIBUTE_MASK_WRITABLE),
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::LevelControl::Attributes::OnOffTransitionTime::Id, INT16U, 1, ATTRIBUTE_MASK_WRITABLE),
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::LevelControl::Attributes::OnLevel::Id, INT8U, 1, ATTRIBUTE_MASK_WRITABLE),
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::LevelControl::Attributes::OnTransitionTime::Id, INT16U, 1, ATTRIBUTE_MASK_WRITABLE),
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::LevelControl::Attributes::OffTransitionTime::Id, INT16U, 1, ATTRIBUTE_MASK_WRITABLE),
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::LevelControl::Attributes::StartUpCurrentLevel::Id, INT8U, 1, ATTRIBUTE_MASK_WRITABLE),
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::LevelControl::Attributes::DefaultMoveRate::Id, INT8U, 1,
                              ATTRIBUTE_MASK_WRITABLE), /* DefaultMoveRate */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::LevelControl::Attributes::ClusterRevision::Id, INT16U, ZCL_LEVEL_CONTROL_CLUSTER_REVISION,
                              0),
    DECLARE_DYNAMIC_ATTRIBUTE_LIST_END();

// Declare color control cluster attributes
DECLARE_DYNAMIC_ATTRIBUTE_LIST_BEGIN(colorControlAttrs)
DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::CurrentHue::Id, INT8U, 1, ATTRIBUTE_MASK_WRITABLE), /* CurrentHue */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::CurrentSaturation::Id, INT8U, 1,
                              ATTRIBUTE_MASK_WRITABLE), /* CurrentSaturation */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::RemainingTime::Id, INT16U, 1,
                              ATTRIBUTE_MASK_WRITABLE), /* RemainingTime */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::CurrentX::Id, INT16U, 1, ATTRIBUTE_MASK_WRITABLE), /* CurrentX */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::CurrentY::Id, INT16U, 1, ATTRIBUTE_MASK_WRITABLE), /* CurrentY */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::ColorTemperatureMireds::Id, INT16U, 1,
                              ATTRIBUTE_MASK_WRITABLE), /* ColorTemperatureMireds */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::ColorMode::Id, INT8U, 1, ATTRIBUTE_MASK_WRITABLE), /* ColorMode */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::Options::Id, INT8U, 1, ATTRIBUTE_MASK_WRITABLE),   /* Options */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::NumberOfPrimaries::Id, INT8U, 1,
                              ATTRIBUTE_MASK_WRITABLE), /* NumberOfPrimaries */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::EnhancedCurrentHue::Id, INT16U, 1,
                              ATTRIBUTE_MASK_WRITABLE), /* EnhancedCurrentHue */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::EnhancedColorMode::Id, INT8U, 1,
                              ATTRIBUTE_MASK_WRITABLE), /* EnhancedColorMode */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::ColorLoopActive::Id, INT8U, 1,
                              ATTRIBUTE_MASK_WRITABLE), /* ColorLoopActive */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::ColorLoopDirection::Id, INT8U, 1,
                              ATTRIBUTE_MASK_WRITABLE), /* ColorLoopDirection */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::ColorLoopTime::Id, INT16U, 1,
                              ATTRIBUTE_MASK_WRITABLE), /* ColorLoopTime */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::ColorLoopStartEnhancedHue::Id, INT16U, 1,
                              ATTRIBUTE_MASK_WRITABLE), /* ColorLoopStartEnhancedHue */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::ColorLoopStoredEnhancedHue::Id, INT16U, 1,
                              ATTRIBUTE_MASK_WRITABLE), /* ColorLoopStoredEnhancedHue */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::ColorCapabilities::Id, INT8U, 1,
                              ATTRIBUTE_MASK_WRITABLE), /* ColorCapabilities */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::ColorTempPhysicalMinMireds::Id, INT16U, 1,
                              ATTRIBUTE_MASK_WRITABLE), /* ColorTempPhysicalMinMireds */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::ColorTempPhysicalMaxMireds::Id, INT16U, 1,
                              ATTRIBUTE_MASK_WRITABLE), /* ColorTempPhysicalMaxMireds */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::ColorControl::Attributes::ClusterRevision::Id, INT16U, ZCL_COLOR_CONTROL_CLUSTER_REVISION,
                              ATTRIBUTE_MASK_WRITABLE),
    DECLARE_DYNAMIC_ATTRIBUTE_LIST_END();

// Declare Descriptor cluster attributes
DECLARE_DYNAMIC_ATTRIBUTE_LIST_BEGIN(descriptorAttrs)
DECLARE_DYNAMIC_ATTRIBUTE(Clusters::Descriptor::Attributes::DeviceTypeList::Id, ARRAY, kDescriptorAttributeArraySize,
                          0), /* device list */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::Descriptor::Attributes::ServerList::Id, ARRAY, kDescriptorAttributeArraySize,
                              0), /* server list */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::Descriptor::Attributes::ClientList::Id, ARRAY, kDescriptorAttributeArraySize,
                              0), /* client list */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::Descriptor::Attributes::PartsList::Id, ARRAY, kDescriptorAttributeArraySize,
                              0), /* parts list */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::Descriptor::Attributes::ClusterRevision::Id, INT16U, ZCL_DESCRIPTOR_CLUSTER_REVISION,
                              0), /* cluster revision */
    DECLARE_DYNAMIC_ATTRIBUTE_LIST_END();

// Declare Bridged Device Basic Information cluster attributes
DECLARE_DYNAMIC_ATTRIBUTE_LIST_BEGIN(bridgedDeviceBasicAttrs)
DECLARE_DYNAMIC_ATTRIBUTE(chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id, CHAR_STRING,
                          kNodeLabelSize, 0), /* NodeLabel */
    DECLARE_DYNAMIC_ATTRIBUTE(chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::Reachable::Id, BOOLEAN, 1,
                              0), /* Reachable */
    DECLARE_DYNAMIC_ATTRIBUTE(chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::ClusterRevision::Id, INT16U,
                              ZCL_BRIDGED_DEVICE_BASIC_INFORMATION_CLUSTER_REVISION, 0), /* cluster revision */
    DECLARE_DYNAMIC_ATTRIBUTE_LIST_END();

// Declare Cluster List for Bridged Light endpoint
// TODO: It's not clear whether it would be better to get the command lists from
// the ZAP config on our last fixed endpoint instead.
constexpr CommandId onOffIncomingCommands[] = {
    app::Clusters::OnOff::Commands::Off::Id,
    app::Clusters::OnOff::Commands::On::Id,
    app::Clusters::OnOff::Commands::Toggle::Id,
    app::Clusters::OnOff::Commands::OffWithEffect::Id,
    app::Clusters::OnOff::Commands::OnWithRecallGlobalScene::Id,
    app::Clusters::OnOff::Commands::OnWithTimedOff::Id,
    kInvalidCommandId,
};

constexpr CommandId levelControlIncomingCommands[] = {
    app::Clusters::LevelControl::Commands::MoveToLevel::Id,
    app::Clusters::LevelControl::Commands::Move::Id,
    app::Clusters::LevelControl::Commands::Step::Id,
    app::Clusters::LevelControl::Commands::Stop::Id,
    app::Clusters::LevelControl::Commands::MoveToLevelWithOnOff::Id,
    app::Clusters::LevelControl::Commands::MoveWithOnOff::Id,
    app::Clusters::LevelControl::Commands::StepWithOnOff::Id,
    app::Clusters::LevelControl::Commands::StopWithOnOff::Id,
    kInvalidCommandId,
};

constexpr CommandId colorControlIncomingCommands[] = {
    app::Clusters::ColorControl::Commands::MoveToHue::Id,
    app::Clusters::ColorControl::Commands::MoveHue::Id,
    app::Clusters::ColorControl::Commands::StepHue::Id,
    app::Clusters::ColorControl::Commands::MoveToSaturation::Id,
    app::Clusters::ColorControl::Commands::MoveSaturation::Id,
    app::Clusters::ColorControl::Commands::StepSaturation::Id,
    app::Clusters::ColorControl::Commands::MoveToHueAndSaturation::Id,
    app::Clusters::ColorControl::Commands::MoveToColor::Id,
    app::Clusters::ColorControl::Commands::MoveColor::Id,
    app::Clusters::ColorControl::Commands::StepColor::Id,
    app::Clusters::ColorControl::Commands::MoveToColorTemperature::Id,
    app::Clusters::ColorControl::Commands::EnhancedMoveToHue::Id,
    kInvalidCommandId,
};

DECLARE_DYNAMIC_CLUSTER_LIST_BEGIN(bridgedLightClusters)
DECLARE_DYNAMIC_CLUSTER(Clusters::OnOff::Id, onOffAttrs, ZAP_CLUSTER_MASK(SERVER), onOffIncomingCommands, nullptr),
    DECLARE_DYNAMIC_CLUSTER(Clusters::LevelControl::Id, levelControlAttrs, ZAP_CLUSTER_MASK(SERVER), levelControlIncomingCommands,
                            nullptr),
    DECLARE_DYNAMIC_CLUSTER(Clusters::ColorControl::Id, colorControlAttrs, ZAP_CLUSTER_MASK(SERVER), colorControlIncomingCommands,
                            nullptr),
    DECLARE_DYNAMIC_CLUSTER(Clusters::Descriptor::Id, descriptorAttrs, ZAP_CLUSTER_MASK(SERVER), nullptr, nullptr),
    DECLARE_DYNAMIC_CLUSTER(chip::app::Clusters::BridgedDeviceBasicInformation::Id, bridgedDeviceBasicAttrs,
                            ZAP_CLUSTER_MASK(SERVER), nullptr, nullptr),
    DECLARE_DYNAMIC_CLUSTER_LIST_END;

// ----------------------------Temperature sensor-----------------------------------------------
DECLARE_DYNAMIC_ATTRIBUTE_LIST_BEGIN(tempSensorAttrs)
DECLARE_DYNAMIC_ATTRIBUTE(Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Id, INT16S, 2, 0), /* Measured Value */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::TemperatureMeasurement::Attributes::MinMeasuredValue::Id, INT16S, 2,
                              0), /* Min Measured Value */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::TemperatureMeasurement::Attributes::MaxMeasuredValue::Id, INT16S, 2,
                              0), /* Max Measured Value */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::TemperatureMeasurement::Attributes::FeatureMap::Id, BITMAP32, 4, 0), /* FeatureMap */
    DECLARE_DYNAMIC_ATTRIBUTE(Clusters::TemperatureMeasurement::Attributes::ClusterRevision::Id, INT16U,
                              ZCL_TEMPERATURE_SENSOR_CLUSTER_REVISION, 0), /* cluster revision */
    DECLARE_DYNAMIC_ATTRIBUTE_LIST_END();

// TEMPERATURE SENSOR ENDPOINT: contains the following clusters:
//   - Temperature measurement
//   - Descriptor
//   - Bridged Device Basic Information
DECLARE_DYNAMIC_CLUSTER_LIST_BEGIN(bridgedTempSensorClusters)
DECLARE_DYNAMIC_CLUSTER(Clusters::TemperatureMeasurement::Id, tempSensorAttrs, ZAP_CLUSTER_MASK(SERVER), nullptr, nullptr),
    DECLARE_DYNAMIC_CLUSTER(Clusters::Descriptor::Id, descriptorAttrs, ZAP_CLUSTER_MASK(SERVER), nullptr, nullptr),
    DECLARE_DYNAMIC_CLUSTER(Clusters::BridgedDeviceBasicInformation::Id, bridgedDeviceBasicAttrs, ZAP_CLUSTER_MASK(SERVER), nullptr,
                            nullptr),
    DECLARE_DYNAMIC_CLUSTER_LIST_END;

// Declare Bridged Light endpoint
DECLARE_DYNAMIC_ENDPOINT(bridgedTempSensorEndpoint, bridgedTempSensorClusters);
DataVersion gTempSensor1DataVersions[ArraySize(bridgedTempSensorClusters)];

// Declare Bridged Light endpoint
DECLARE_DYNAMIC_ENDPOINT(bridgedLightEndpoint, bridgedLightClusters);

DataVersion gLight1DataVersions[ArraySize(bridgedLightClusters)];
DataVersion gLight2DataVersions[ArraySize(bridgedLightClusters)];
DataVersion gLight3DataVersions[ArraySize(bridgedLightClusters)];
DataVersion gLight4DataVersions[ArraySize(bridgedLightClusters)];
// DataVersion gThermostatDataVersions[ArraySize(thermostatAttrs)];

const EmberAfDeviceType gRootDeviceTypes[]          = { { DEVICE_TYPE_ROOT_NODE, DEVICE_VERSION_DEFAULT } };
const EmberAfDeviceType gAggregateNodeDeviceTypes[] = { { DEVICE_TYPE_BRIDGE, DEVICE_VERSION_DEFAULT } };

const EmberAfDeviceType gBridgedDimmableLightDeviceTypes[] = { { DEVICE_TYPE_DIMMALBE_LIGHT, DEVICE_VERSION_DEFAULT },
                                                               { DEVICE_TYPE_BRIDGED_NODE, DEVICE_VERSION_DEFAULT } };

const EmberAfDeviceType gBridgedTempSensorDeviceTypes[] = { { DEVICE_TYPE_TEMP_SENSOR, DEVICE_VERSION_DEFAULT },
                                                            { DEVICE_TYPE_BRIDGED_NODE, DEVICE_VERSION_DEFAULT } };

int AddDeviceEndpoint(Device * dev, EmberAfEndpointType * ep, const Span<const EmberAfDeviceType> & deviceTypeList,
                      const Span<DataVersion> & dataVersionStorage, chip::EndpointId parentEndpointId)
{
    uint8_t index = 0;
    while (index < CHIP_DEVICE_CONFIG_DYNAMIC_ENDPOINT_COUNT)
    {
        if (NULL == gDevices[index])
        {
            gDevices[index] = dev;
            CHIP_ERROR err;
            while (true)
            {
                dev->SetEndpointId(gCurrentEndpointId);
                err =
                    emberAfSetDynamicEndpoint(index, gCurrentEndpointId, ep, dataVersionStorage, deviceTypeList, parentEndpointId);
                if (err == CHIP_NO_ERROR)
                {
                    ChipLogProgress(DeviceLayer, "Added device %s to dynamic endpoint %d (index=%d)", dev->GetName(),
                                    gCurrentEndpointId, index);
                    return index;
                }
                else if (err != CHIP_ERROR_ENDPOINT_EXISTS)
                {
                    return -1;
                }
                // Handle wrap condition
                if (++gCurrentEndpointId < gFirstDynamicEndpointId)
                {
                    gCurrentEndpointId = gFirstDynamicEndpointId;
                }
            }
        }
        index++;
    }
    ChipLogProgress(DeviceLayer, "Failed to add dynamic endpoint: No endpoints available!");
    return -1;
}

CHIP_ERROR RemoveDeviceEndpoint(Device * dev)
{
    for (uint8_t index = 0; index < CHIP_DEVICE_CONFIG_DYNAMIC_ENDPOINT_COUNT; index++)
    {
        if (gDevices[index] == dev)
        {
            // Silence complaints about unused ep when progress logging
            // disabled.
            [[maybe_unused]] EndpointId ep = emberAfClearDynamicEndpoint(index);
            gDevices[index]                = NULL;
            ChipLogProgress(DeviceLayer, "Removed device %s from dynamic endpoint %d (index=%d)", dev->GetName(), ep, index);
            return CHIP_NO_ERROR;
        }
    }
    return CHIP_ERROR_INTERNAL;
}

Protocols::InteractionModel::Status HandleReadBridgedDeviceBasicAttribute(Device * dev, chip::AttributeId attributeId,
                                                                          uint8_t * buffer, uint16_t maxReadLength)
{
    using namespace chip::app::Clusters::BridgedDeviceBasicInformation::Attributes;
    ChipLogProgress(DeviceLayer, "HandleReadBridgedDeviceBasicAttribute: attrId=%" PRIu32 ", maxReadLength=%u", attributeId,
                    maxReadLength);

    if ((attributeId == Reachable::Id) && (maxReadLength == 1))
    {
        *buffer = dev->IsReachable() ? 1 : 0;
    }
    else if ((attributeId == NodeLabel::Id) && (maxReadLength == 32))
    {
        MutableByteSpan zclNameSpan(buffer, maxReadLength);
        MakeZclCharString(zclNameSpan, dev->GetName());
    }
    else if ((attributeId == FeatureMap::Id) && (maxReadLength == 4))
    {
        uint32_t featureMap = ZCL_BRIDGED_DEVICE_BASIC_INFORMATION_FEATURE_MAP;
        memcpy(buffer, &featureMap, sizeof(featureMap));
    }
    else if ((attributeId == ClusterRevision::Id) && (maxReadLength == 4))
    {
        uint16_t clusterRevision = ZCL_BRIDGED_DEVICE_BASIC_INFORMATION_CLUSTER_REVISION;
        memcpy(buffer, &clusterRevision, sizeof(clusterRevision));
    }
    else
    {
        return Protocols::InteractionModel::Status::Failure;
    }

    return Protocols::InteractionModel::Status::Success;
}

Protocols::InteractionModel::Status HandleReadOnOffAttribute(Device * dev, chip::AttributeId attributeId, uint8_t * buffer,
                                                             uint16_t maxReadLength)
{
    using namespace Clusters::OnOff::Attributes;
    ChipLogProgress(DeviceLayer, "HandleReadOnOffAttribute: attrId=%" PRIu32 ", maxReadLength=%u", attributeId, maxReadLength);

    if ((attributeId == OnOff::Id) && (maxReadLength == 1))
    {
        bool onOff = dev->IsOn() ? 1 : 0;
        debug_msg("OnOff=[%u]\n", onOff);
        memcpy(buffer, &onOff, sizeof(onOff));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if (attributeId == GlobalSceneControl::Id)
    {
        bool globalSceneControl = true;
        debug_msg("GlobalSceneControl=[%u]\n", globalSceneControl);
        memcpy(buffer, &globalSceneControl, sizeof(globalSceneControl));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if (attributeId == OnTime::Id)
    {
        uint16_t onTime = 0x00;
        debug_msg("OnTime=[%u]\n", onTime);
        memcpy(buffer, &onTime, sizeof(onTime));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if (attributeId == OffWaitTime::Id)
    {
        uint16_t offWaitTime = 0x00;
        debug_msg("OffWaitTime=[%u]\n", offWaitTime);
        memcpy(buffer, &offWaitTime, sizeof(offWaitTime));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if (attributeId == StartUpOnOff::Id)
    {
        uint8_t startUpOnOff = 0xff;
        debug_msg("StartUpOnOff=[%u]\n", startUpOnOff);
        memcpy(buffer, &startUpOnOff, sizeof(startUpOnOff));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == ClusterRevision::Id) && (maxReadLength == 4))
    {
        uint16_t clusterRevision = ZCL_ON_OFF_CLUSTER_REVISION;
        memcpy(buffer, &clusterRevision, sizeof(clusterRevision));
    }
    else
    {
        return Protocols::InteractionModel::Status::Failure;
    }

    return Protocols::InteractionModel::Status::Success;
}

Protocols::InteractionModel::Status HandleWriteOnOffAttribute(Device * dev, chip::AttributeId attributeId, uint8_t * buffer)
{
    ChipLogProgress(DeviceLayer, "HandleWriteOnOffAttribute: attrId=%" PRIu32, attributeId);

    ReturnErrorCodeIf((attributeId != Clusters::OnOff::Attributes::OnOff::Id) || (!dev->IsReachable()),
                      Protocols::InteractionModel::Status::Failure);
    dev->SetOnOff(*buffer == 1);
    return Protocols::InteractionModel::Status::Success;
}

Protocols::InteractionModel::Status HandleReadLevelControlAttribute(Device * dev, chip::AttributeId attributeId, uint8_t * buffer,
                                                                    uint16_t maxReadLength)
{
    using namespace Clusters::LevelControl::Attributes;
    ChipLogProgress(DeviceLayer, "HandleReadLevelControlAttribute: attrId=%" PRIu32 ", maxReadLength=%u", attributeId,
                    maxReadLength);

    if ((attributeId == CurrentLevel::Id) /* && (maxReadLength == 1)*/)
    {
        // *buffer = dev->GetLevel();
        uint8_t currentLevelValue = dev->GetLevel();
        // if (!currentLevelValue)
        // {
        //     currentLevelValue = 100;
        //     dev->SetLevel(100);
        //     Clusters::LevelControl::Attributes::CurrentLevel::Set(LIGHT1_ENDPIONT, 100);
        // }
        debug_msg("currentLevelValue=[%u]\n", currentLevelValue);
        memcpy(buffer, &currentLevelValue, sizeof(currentLevelValue));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == RemainingTime::Id) /* && (maxReadLength == 4)*/)
    {
        debug_msg("RemainingTime\n");
        uint16_t remainingTime = 0;
        memcpy(buffer, &remainingTime, sizeof(remainingTime));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == MinLevel::Id) /* && (maxReadLength == 2)*/)
    {
        debug_msg("MinLevel\n");
        uint8_t MinLevel = 0;
        memcpy(buffer, &MinLevel, sizeof(MinLevel));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == MaxLevel::Id) /* && (maxReadLength == 2)*/)
    {
        debug_msg("MaxLevel\n");
        uint8_t maxLevel = 254;
        memcpy(buffer, &maxLevel, sizeof(maxLevel));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == CurrentFrequency::Id) /* && (maxReadLength == 2)*/)
    {
        debug_msg("CurrentFrequency\n");
        uint8_t currentFrequency = 254;
        memcpy(buffer, &currentFrequency, sizeof(currentFrequency));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == MinFrequency::Id) /* && (maxReadLength == 2)*/)
    {
        debug_msg("MinFrequency\n");
        uint8_t minFrequency = 254;
        memcpy(buffer, &minFrequency, sizeof(minFrequency));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == MaxFrequency::Id) /* && (maxReadLength == 2)*/)
    {
        debug_msg("MaxFrequency\n");
        uint8_t maxFrequency = 254;
        memcpy(buffer, &maxFrequency, sizeof(maxFrequency));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == Options::Id) /* && (maxReadLength == 1)*/)
    {
        debug_msg("Options\n");
        uint8_t Options = 0x00;
        memcpy(buffer, &Options, sizeof(Options));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == OnOffTransitionTime::Id) /* && (maxReadLength == 1)*/)
    {
        debug_msg("OnOffTransitionTime\n");
        uint8_t OnOffTransitionTime = 0;
        memcpy(buffer, &OnOffTransitionTime, sizeof(OnOffTransitionTime));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == OnLevel::Id) /* && (maxReadLength == 2)*/)
    {
        debug_msg("OnLevel\n");
        uint8_t OnLevel = 0;
        memcpy(buffer, &OnLevel, sizeof(OnLevel));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == OnTransitionTime::Id) /* && (maxReadLength == 1)*/)
    {
        debug_msg("OnTransitionTime\n");
        uint8_t OnTransitionTime = 0;
        memcpy(buffer, &OnTransitionTime, sizeof(OnTransitionTime));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == OffTransitionTime::Id) /* && (maxReadLength == 1)*/)
    {
        debug_msg("OffTransitionTime\n");
        uint8_t OffTransitionTime = 0;
        memcpy(buffer, &OffTransitionTime, sizeof(OffTransitionTime));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == StartUpCurrentLevel::Id) /* && (maxReadLength == 1)*/)
    {
        debug_msg("StartUpCurrentLevel\n");
        uint8_t StartUpCurrentLevel = 255;
        memcpy(buffer, &StartUpCurrentLevel, sizeof(StartUpCurrentLevel));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == DefaultMoveRate::Id) /* && (maxReadLength == 1)*/)
    {
        uint8_t defaultMoveRate = dev->GetDefaultMoveRate();
        debug_msg("DefaultMoveRate::Id=[%u]\n", defaultMoveRate);
        memcpy(buffer, &defaultMoveRate, sizeof(defaultMoveRate));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == ClusterRevision::Id) && (maxReadLength == 4))
    {
        debug_msg("ClusterRevision\n");
        uint16_t clusterRevision = ZCL_LEVEL_CONTROL_CLUSTER_REVISION;
        memcpy(buffer, &clusterRevision, sizeof(clusterRevision));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else
    {
        debug_msg("EMBER_ZCL_STATUS_FAILURE \n");
        return Protocols::InteractionModel::Status::Failure;
    }
    debug_msg("EMBER_ZCL_STATUS_SUCCESS \n");
    return Protocols::InteractionModel::Status::Success;
}

Protocols::InteractionModel::Status HandleWriteLevelControlAttribute(Device * dev, chip::AttributeId attributeId, uint8_t * buffer)
{
    debug_msg("HandleWriteLevelControlAttribute: attrId=%d\n", attributeId);

    if (dev->IsReachable())
    {
        if (attributeId == Clusters::LevelControl::Attributes::CurrentLevel::Id)
        {
            debug_msg("CurrentLevel\n");

            dev->SetLevel(*buffer);
            if (dev->GetLevel())
            {
                dev->SetOnOff(true);
            }
            else
            {
                dev->SetOnOff(false);
            }

            return Protocols::InteractionModel::Status::Success;
        }
        else if (attributeId == Clusters::LevelControl::Attributes::DefaultMoveRate::Id)
        {
            debug_msg("DefaultMoveRate\n");
            dev->SetDefaultMoveRate(*buffer);
            return Protocols::InteractionModel::Status::Success;
        }
    }

    debug_msg("EMBER_ZCL_STATUS_FAILURE \n");
    return Protocols::InteractionModel::Status::Failure;
    // ReturnErrorCodeIf((attributeId != Clusters::LevelControl::Attributes::CurrentLevel::Id) || (!dev->IsReachable()),
    // EMBER_ZCL_STATUS_FAILURE); dev->SetLevel(*buffer); debug_msg("buffer[%d]\n", *buffer); return EMBER_ZCL_STATUS_SUCCESS;
}

Protocols::InteractionModel::Status HandleReadColorControlAttribute(Device * dev, chip::AttributeId attributeId, uint8_t * buffer,
                                                                    uint16_t maxReadLength)
{
    using namespace Clusters::ColorControl::Attributes;
    ChipLogProgress(DeviceLayer, "HandleReadColorControlAttribute: attrId=%" PRIu32 ", maxReadLength=%u", attributeId,
                    maxReadLength);

    if ((attributeId == CurrentHue::Id) /* && (maxReadLength == 1)*/)
    {
        uint8_t currentHue = dev->GetCurrentHue();
        debug_msg("CurrentHue::Id=[%u]\n", currentHue);
        memcpy(buffer, &currentHue, sizeof(currentHue));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == CurrentSaturation::Id) /* && (maxReadLength == 1)*/)
    {
        uint8_t currentSaturation = dev->GetCurrentSaturation();
        debug_msg("SurrentSaturation::Id=[%u]\n", currentSaturation);
        memcpy(buffer, &currentSaturation, sizeof(currentSaturation));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == RemainingTime::Id) /* && (maxReadLength == 1)*/)
    {
        uint16_t remainingTime = 0;
        debug_msg("RemainingTime::Id=[%u]\n", remainingTime);
        memcpy(buffer, &remainingTime, sizeof(remainingTime));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == CurrentX::Id) /* && (maxReadLength == 1)*/)
    {
        uint16_t currentX = 0x616b;
        debug_msg("CurrentX::Id=[%u]\n", currentX);
        memcpy(buffer, &currentX, sizeof(currentX));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == CurrentY::Id) /* && (maxReadLength == 1)*/)
    {
        uint16_t currentY = 0x607D;
        debug_msg("CurrentY::Id=[%u]\n", currentY);
        memcpy(buffer, &currentY, sizeof(currentY));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == ColorTemperatureMireds::Id) /* && (maxReadLength == 1)*/)
    {
        uint16_t colorTemperatureMireds = 0x00fa;
        debug_msg("ColorTemperatureMireds::Id=[%u]\n", colorTemperatureMireds);
        memcpy(buffer, &colorTemperatureMireds, sizeof(colorTemperatureMireds));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == ColorMode::Id) /* && (maxReadLength == 1)*/)
    {
        uint16_t colorMode = 0x01;
        debug_msg("ColorMode::Id=[%u]\n", colorMode);
        memcpy(buffer, &colorMode, sizeof(colorMode));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == Options::Id) /* && (maxReadLength == 1)*/)
    {
        uint16_t options = 0;
        debug_msg("Options::Id=[%u]\n", options);
        memcpy(buffer, &options, sizeof(options));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == NumberOfPrimaries::Id) /* && (maxReadLength == 1)*/)
    {
        uint8_t numberOfPrimaries = 0;
        debug_msg("NumberOfPrimaries::Id=[%u]\n", numberOfPrimaries);
        memcpy(buffer, &numberOfPrimaries, sizeof(numberOfPrimaries));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == EnhancedCurrentHue::Id) /* && (maxReadLength == 1)*/)
    {
        uint16_t enhancedCurrentHue = 0x00;
        debug_msg("EnhancedCurrentHue::Id=[%u]\n", enhancedCurrentHue);
        memcpy(buffer, &enhancedCurrentHue, sizeof(enhancedCurrentHue));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == EnhancedColorMode::Id) /* && (maxReadLength == 1)*/)
    {
        uint16_t enhancedColorMode = 0x01;
        debug_msg("EnhancedColorMode::Id=[%u]\n", enhancedColorMode);
        memcpy(buffer, &enhancedColorMode, sizeof(enhancedColorMode));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == ColorLoopActive::Id) /* && (maxReadLength == 1)*/)
    {
        uint8_t colorLoopActive = 0x00;
        debug_msg("ColorLoopActive::Id=[%u]\n", colorLoopActive);
        memcpy(buffer, &colorLoopActive, sizeof(colorLoopActive));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == ColorLoopDirection::Id) /* && (maxReadLength == 1)*/)
    {
        uint8_t colorLoopDirection = 0x00;
        debug_msg("ColorLoopDirection::Id=[%u]\n", colorLoopDirection);
        memcpy(buffer, &colorLoopDirection, sizeof(colorLoopDirection));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == ColorLoopTime::Id) /* && (maxReadLength == 1)*/)
    {
        uint16_t colorLoopTime = 0x0019;
        debug_msg("ColorLoopTime::Id=[%u]\n", colorLoopTime);
        memcpy(buffer, &colorLoopTime, sizeof(colorLoopTime));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == ColorLoopStartEnhancedHue::Id) /* && (maxReadLength == 1)*/)
    {
        uint16_t colorLoopStartEnhancedHue = 0x2300;
        debug_msg("ColorLoopStartEnhancedHue::Id=[%u]\n", colorLoopStartEnhancedHue);
        memcpy(buffer, &colorLoopStartEnhancedHue, sizeof(colorLoopStartEnhancedHue));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == ColorLoopStoredEnhancedHue::Id) /* && (maxReadLength == 1)*/)
    {
        uint16_t colorLoopStoredEnhancedHue = 0x00;
        debug_msg("ColorLoopStoredEnhancedHue::Id=[%u]\n", colorLoopStoredEnhancedHue);
        memcpy(buffer, &colorLoopStoredEnhancedHue, sizeof(colorLoopStoredEnhancedHue));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == ColorCapabilities::Id) /* && (maxReadLength == 1)*/)
    {
        uint8_t colorCapabilities = 0x1f;
        debug_msg("ColorCapabilities::Id=[%u]\n", colorCapabilities);
        memcpy(buffer, &colorCapabilities, sizeof(colorCapabilities));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == ColorTempPhysicalMinMireds::Id) /* && (maxReadLength == 1)*/)
    {
        uint16_t colorTempPhysicalMinMireds = 0x009a;
        debug_msg("ColorTempPhysicalMinMireds::Id=[%u]\n", colorTempPhysicalMinMireds);
        memcpy(buffer, &colorTempPhysicalMinMireds, sizeof(colorTempPhysicalMinMireds));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == ColorTempPhysicalMaxMireds::Id) /* && (maxReadLength == 1)*/)
    {
        uint16_t colorTempPhysicalMaxMireds = 0x01c6;
        debug_msg("ColorTempPhysicalMaxMireds::Id=[%u]\n", colorTempPhysicalMaxMireds);
        memcpy(buffer, &colorTempPhysicalMaxMireds, sizeof(colorTempPhysicalMaxMireds));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else if ((attributeId == ClusterRevision::Id) /* && (maxReadLength == 1)*/)
    {
        uint16_t clusterRevision = ZCL_COLOR_CONTROL_CLUSTER_REVISION;
        debug_msg("ClusterRevision::Id=[%u]\n", clusterRevision);
        memcpy(buffer, &clusterRevision, sizeof(clusterRevision));
        debug_msg("*buffer=[%d]\n", *buffer);
    }
    else
    {
        debug_msg("EMBER_ZCL_STATUS_FAILURE \n");
        return Protocols::InteractionModel::Status::Failure;
    }
    debug_msg("EMBER_ZCL_STATUS_SUCCESS \n");

    return Protocols::InteractionModel::Status::Success;
}

Protocols::InteractionModel::Status HandleWriteColorControlAttribute(Device * dev, chip::AttributeId attributeId, uint8_t * buffer)
{
    using namespace Clusters::ColorControl::Attributes;
    debug_msg("HandleWriteColorControlAttribute: attrId=%d", attributeId);

    if (dev->IsReachable())
    {
        if (attributeId == CurrentHue::Id)
        {
            debug_msg("CurrentHue\n");
            dev->SetCurrentHue(*buffer);
            return Protocols::InteractionModel::Status::Success;
        }
        else if (attributeId == CurrentSaturation::Id)
        {
            debug_msg("CurrentSaturation\n");
            dev->SetCurrentSaturation(*buffer);
            return Protocols::InteractionModel::Status::Success;
        }
    }

    debug_msg("EMBER_ZCL_STATUS_FAILURE \n");
    return Protocols::InteractionModel::Status::Failure;
}

Protocols::InteractionModel::Status emberAfExternalAttributeReadCallback(EndpointId endpoint, ClusterId clusterId,
                                                                         const EmberAfAttributeMetadata * attributeMetadata,
                                                                         uint8_t * buffer, uint16_t maxReadLength)
{
    using namespace Clusters;

    debug_msg("emberAfExternalAttributeReadCallback: ClusterId=0x%x endpoint=0x%x\n", clusterId, endpoint);

    uint16_t endpointIndex = emberAfGetDynamicIndexFromEndpoint(endpoint);

    if ((endpointIndex < CHIP_DEVICE_CONFIG_DYNAMIC_ENDPOINT_COUNT) && (gDevices[endpointIndex] != NULL))
    {
        Device * dev = gDevices[endpointIndex];

        if (clusterId == BridgedDeviceBasicInformation::Id)
        {
            return HandleReadBridgedDeviceBasicAttribute(dev, attributeMetadata->attributeId, buffer, maxReadLength);
        }
        else if (clusterId == OnOff::Id)
        {
            return HandleReadOnOffAttribute(dev, attributeMetadata->attributeId, buffer, maxReadLength);
        }
        else if (clusterId == TemperatureMeasurement::Id)
        {
            return HandleReadTempMeasurementAttribute(static_cast<DeviceTempSensor *>(dev), attributeMetadata->attributeId, buffer,
                                                      maxReadLength);
        }
        else if (clusterId == LevelControl::Id)
        {
            return HandleReadLevelControlAttribute(dev, attributeMetadata->attributeId, buffer, maxReadLength);
        }
        else if (clusterId == ColorControl::Id)
        {
            return HandleReadColorControlAttribute(dev, attributeMetadata->attributeId, buffer, maxReadLength);
        }
    }

    return Protocols::InteractionModel::Status::Failure;
}

Protocols::InteractionModel::Status emberAfExternalAttributeWriteCallback(EndpointId endpoint, ClusterId clusterId,
                                                                          const EmberAfAttributeMetadata * attributeMetadata,
                                                                          uint8_t * buffer)
{
    debug_msg("emberAfExternalAttributeWriteCallback: ClusterId=0x%x endpoint=0x%x\n", clusterId, endpoint);

    uint16_t endpointIndex = emberAfGetDynamicIndexFromEndpoint(endpoint);

    if (endpointIndex < CHIP_DEVICE_CONFIG_DYNAMIC_ENDPOINT_COUNT)
    {
        Device * dev = gDevices[endpointIndex];

        if (dev->IsReachable())
        {
            if (clusterId == Clusters::OnOff::Id)
            {
                return HandleWriteOnOffAttribute(dev, attributeMetadata->attributeId, buffer);
            }
            else if (clusterId == Clusters::LevelControl::Id)
            {
                return HandleWriteLevelControlAttribute(dev, attributeMetadata->attributeId, buffer);
            }
            else if (clusterId == Clusters::ColorControl::Id)
            {
                return HandleWriteColorControlAttribute(dev, attributeMetadata->attributeId, buffer);
            }
        }
    }

    return Protocols::InteractionModel::Status::Failure;
}

namespace {
void CallReportingCallback(intptr_t closure)
{
    auto path = reinterpret_cast<app::ConcreteAttributePath *>(closure);
    MatterReportingAttributeChangeCallback(*path);
    Platform::Delete(path);
}

void ScheduleReportingCallback(Device * dev, ClusterId cluster, AttributeId attribute)
{
    auto * path = Platform::New<app::ConcreteAttributePath>(dev->GetEndpointId(), cluster, attribute);
    DeviceLayer::PlatformMgr().ScheduleWork(CallReportingCallback, reinterpret_cast<intptr_t>(path));
}
} // anonymous namespace

void HandleDeviceStatusChanged(Device * dev, Device::Changed_t itemChangedMask)
{
    using namespace chip::app::Clusters;
    if (itemChangedMask & Device::kChanged_Reachable)
    {
        ScheduleReportingCallback(dev, BridgedDeviceBasicInformation::Id, BridgedDeviceBasicInformation::Attributes::Reachable::Id);
    }

    if (itemChangedMask & Device::kChanged_State)
    {
        ScheduleReportingCallback(dev, OnOff::Id, OnOff::Attributes::OnOff::Id);
    }

    if (itemChangedMask & Device::kChanged_Name)
    {
        ScheduleReportingCallback(dev, BridgedDeviceBasicInformation::Id, BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
    }
}

bool emberAfActionsClusterInstantActionCallback(app::CommandHandler * commandObj, const app::ConcreteCommandPath & commandPath,
                                                const Clusters::Actions::Commands::InstantAction::DecodableType & commandData)
{
    // No actions are implemented, just return status NotFound.
    commandObj->AddStatus(commandPath, Protocols::InteractionModel::Status::NotFound);
    return true;
}

CHIP_ERROR AppTask::Init(void)
{
    SetExampleButtonCallbacks(LightingActionEventHandler);
    InitCommonParts();

    Protocols::InteractionModel::Status status;

    // app::DataModel::Nullable<uint8_t> level;
    //  Read brightness value
    // status = Clusters::LevelControl::Attributes::CurrentLevel::Get(LIGHT1_ENDPIONT, level);
    // if (status == Protocols::InteractionModel::Status::Success && !level.IsNull())
    //{
    //     sLevel = level.Value();
    //     ChipLogProgress(DeviceLayer, "==add by clz:level value is %d in endpoint %d ", sLevel, kExampleEndpointId);
    // }

    bool isOn;
    // Read storedValue on/off value
    status = Clusters::OnOff::Attributes::OnOff::Get(LIGHT1_ENDPIONT, &isOn);
    if (status == Protocols::InteractionModel::Status::Success)
    {
        sTurnedOn = isOn;
        PwmManager::getInstance().setPwm(PwmManager::EAppPwm_Red, sTurnedOn);
    }
    ChipLogProgress(DeviceLayer, "==add by clz:isOn value is %d in endpoint 1 ", isOn);

    for (size_t i = 0; i < CHIP_DEVICE_CONFIG_DYNAMIC_ENDPOINT_COUNT; i++)
    {
        gDevices[i] = nullptr;
    }

    gLight1.SetReachable(true);
    /*
    gLight2.SetReachable(true);
    gLight3.SetReachable(true);
    gLight4.SetReachable(true);
    TempSensor1.SetReachable(true);
    */

    // Whenever bridged device changes its state
    gLight1.SetChangeCallback(&HandleDeviceStatusChanged);
    /* gLight2.SetChangeCallback(&HandleDeviceStatusChanged);
    gLight3.SetChangeCallback(&HandleDeviceStatusChanged);
    gLight4.SetChangeCallback(&HandleDeviceStatusChanged);
    TempSensor1.SetChangeCallback(&HandleDeviceTempSensorStatusChanged);
    */

    PlatformMgr().ScheduleWork(InitServer, reinterpret_cast<intptr_t>(nullptr));

    return CHIP_NO_ERROR;
}

void AppTask::InitServer(intptr_t context)
{
    // Set starting endpoint id where dynamic endpoints will be assigned, which
    // will be the next consecutive endpoint id after the last fixed endpoint.
    uint16_t pre_compiled_endpoint_num = 0;
    pre_compiled_endpoint_num          = static_cast<uint16_t>(emberAfFixedEndpointCount() - 1);

    gFirstDynamicEndpointId = static_cast<chip::EndpointId>(
        static_cast<int>(emberAfEndpointFromIndex(static_cast<uint16_t>(emberAfFixedEndpointCount() - 1))) + 1);
    gCurrentEndpointId = gFirstDynamicEndpointId;

    // Disable last fixed endpoint, which is used as a placeholder for all of the
    // supported clusters so that ZAP will generate the requisite code.
    emberAfEndpointEnableDisable(emberAfEndpointFromIndex(static_cast<uint16_t>(emberAfFixedEndpointCount() - 1)), false);

    pre_compiled_endpoint_num = static_cast<uint16_t>(emberAfFixedEndpointCount() - 1);
    ChipLogProgress(DeviceLayer, "==add by clz:2:pre_compiled_endpoint_num is %d", pre_compiled_endpoint_num);

    // A bridge has root node device type on EP0 and aggregate node device type (bridge) at EP1
    emberAfSetDeviceTypeList(0, Span<const EmberAfDeviceType>(gRootDeviceTypes));
    emberAfSetDeviceTypeList(1, Span<const EmberAfDeviceType>(gAggregateNodeDeviceTypes));

    pre_compiled_endpoint_num = static_cast<uint16_t>(emberAfFixedEndpointCount() - 1);
    ChipLogProgress(DeviceLayer, "==add by clz:3:pre_compiled_endpoint_num is %d", pre_compiled_endpoint_num);

    // Add lights 1..3 --> will be mapped to ZCL endpoints 3, 4, 5
    light1_idx = AddDeviceEndpoint(&gLight1, &bridgedLightEndpoint, Span<const EmberAfDeviceType>(gBridgedDimmableLightDeviceTypes),
                                   Span<DataVersion>(gLight1DataVersions), 1);
    debug_msg("light1_idx=0x%x\n", light1_idx);
    /*
    pre_compiled_endpoint_num = static_cast<uint16_t>(emberAfFixedEndpointCount() - 1);
    ChipLogProgress(DeviceLayer, "==add by clz:4:pre_compiled_endpoint_num is %d", pre_compiled_endpoint_num);

    AddDeviceEndpoint(&gLight2, &bridgedLightEndpoint, Span<const EmberAfDeviceType>(gBridgedDimmableLightDeviceTypes),
                      Span<DataVersion>(gLight2DataVersions), 1);
    pre_compiled_endpoint_num = static_cast<uint16_t>(emberAfFixedEndpointCount() - 1);
    ChipLogProgress(DeviceLayer, "==add by clz:5:pre_compiled_endpoint_num is %d", pre_compiled_endpoint_num);

    AddDeviceEndpoint(&gLight3, &bridgedLightEndpoint, Span<const EmberAfDeviceType>(gBridgedDimmableLightDeviceTypes),
                      Span<DataVersion>(gLight3DataVersions), 1);

    pre_compiled_endpoint_num = static_cast<uint16_t>(emberAfFixedEndpointCount() - 1);
    ChipLogProgress(DeviceLayer, "==add by clz:6:pre_compiled_endpoint_num is %d", pre_compiled_endpoint_num);
    // Remove Light 2 -- Lights 1 & 3 will remain mapped to endpoints 3 & 5
    RemoveDeviceEndpoint(&gLight2);

    pre_compiled_endpoint_num = static_cast<uint16_t>(emberAfFixedEndpointCount() - 1);
    ChipLogProgress(DeviceLayer, "==add by clz:7:pre_compiled_endpoint_num is %d", pre_compiled_endpoint_num);
    // Add Light 4 -- > will be mapped to ZCL endpoint 6
    AddDeviceEndpoint(&gLight4, &bridgedLightEndpoint, Span<const EmberAfDeviceType>(gBridgedDimmableLightDeviceTypes),
                      Span<DataVersion>(gLight4DataVersions), 1);

    pre_compiled_endpoint_num = static_cast<uint16_t>(emberAfFixedEndpointCount() - 1);
    ChipLogProgress(DeviceLayer, "==add by clz:8:pre_compiled_endpoint_num is %d", pre_compiled_endpoint_num);
    // Re-add Light 2 -- > will be mapped to ZCL endpoint 7
    AddDeviceEndpoint(&gLight2, &bridgedLightEndpoint, Span<const EmberAfDeviceType>(gBridgedDimmableLightDeviceTypes),
                      Span<DataVersion>(gLight2DataVersions), 1);

    pre_compiled_endpoint_num = static_cast<uint16_t>(emberAfFixedEndpointCount() - 1);
    ChipLogProgress(DeviceLayer, "==add by clz:9:pre_compiled_endpoint_num is %d", pre_compiled_endpoint_num);
    // Add Temperature Sensor devices --> will be mapped to endpoint 8
    AddDeviceEndpoint(&TempSensor1, &bridgedTempSensorEndpoint, Span<const EmberAfDeviceType>(gBridgedTempSensorDeviceTypes),
                      Span<DataVersion>(gTempSensor1DataVersions), 1);
    */
    emberAfLevelControlClusterServerInitCallback(LIGHT1_ENDPIONT);
    emberAfColorControlClusterServerInitCallback(LIGHT1_ENDPIONT);
    debug_msg("light1_idx=0x%x\n", LIGHT1_ENDPIONT);

    Device * dev_init = gDevices[light1_idx];
    if (dev_init->IsReachable())
    {
        Protocols::InteractionModel::Status status;
        EndpointId light_endpiont = LIGHT1_ENDPIONT;
        app::DataModel::Nullable<uint8_t> init_level;
        Clusters::LevelControl::Attributes::MaxLevel::Set(LIGHT1_ENDPIONT, 254);
        Clusters::LevelControl::Attributes::MinLevel::Set(LIGHT1_ENDPIONT, 0);

        status = Clusters::LevelControl::Attributes::CurrentLevel::Set(LIGHT1_ENDPIONT, 100);
        status = Clusters::LevelControl::Attributes::CurrentLevel::Get(LIGHT1_ENDPIONT, init_level);
        if (status == Protocols::InteractionModel::Status::Success && !init_level.IsNull())
        {
            sLevel = init_level.Value();
            ChipLogProgress(DeviceLayer, "==add by clz:init_level value is %d in endpoint %d ", sLevel, light_endpiont);

            dev_init->SetLevel(sLevel);
        }
    }
}

void HandleDeviceTempSensorStatusChanged(DeviceTempSensor * dev, DeviceTempSensor::Changed_t itemChangedMask)
{
    using namespace Clusters;
    if (itemChangedMask &
        (DeviceTempSensor::kChanged_Reachable | DeviceTempSensor::kChanged_Name | DeviceTempSensor::kChanged_Location))
    {
        HandleDeviceStatusChanged(static_cast<Device *>(dev), (Device::Changed_t) itemChangedMask);
    }
    if (itemChangedMask & DeviceTempSensor::kChanged_MeasurementValue)
    {
        ScheduleReportingCallback(dev, TemperatureMeasurement::Id, TemperatureMeasurement::Attributes::MeasuredValue::Id);
    }
}

Protocols::InteractionModel::Status HandleReadTempMeasurementAttribute(DeviceTempSensor * dev, chip::AttributeId attributeId,
                                                                       uint8_t * buffer, uint16_t maxReadLength)
{
    using namespace Clusters::TemperatureMeasurement::Attributes;

    if ((attributeId == MeasuredValue::Id) && (maxReadLength == 2))
    {
        int16_t measuredValue = dev->GetMeasuredValue();
        memcpy(buffer, &measuredValue, sizeof(measuredValue));
    }
    else if ((attributeId == MinMeasuredValue::Id) && (maxReadLength == 2))
    {
        int16_t minValue = dev->mMin;
        memcpy(buffer, &minValue, sizeof(minValue));
    }
    else if ((attributeId == MaxMeasuredValue::Id) && (maxReadLength == 2))
    {
        int16_t maxValue = dev->mMax;
        memcpy(buffer, &maxValue, sizeof(maxValue));
    }
    else if ((attributeId == FeatureMap::Id) && (maxReadLength == 4))
    {
        uint32_t featureMap = ZCL_TEMPERATURE_SENSOR_FEATURE_MAP;
        memcpy(buffer, &featureMap, sizeof(featureMap));
    }
    else if ((attributeId == ClusterRevision::Id) && (maxReadLength == 4))
    {
        uint16_t clusterRevision = ZCL_TEMPERATURE_SENSOR_CLUSTER_REVISION;
        memcpy(buffer, &clusterRevision, sizeof(clusterRevision));
    }
    else
    {
        return Protocols::InteractionModel::Status::Failure;
    }

    return Protocols::InteractionModel::Status::Success;
}

void AppTask::LightingActionEventHandler(AppEvent * aEvent)
{
    Action_t action = INVALID_ACTION;
    int32_t actor   = 0;

    if (aEvent->Type == AppEvent::kEventType_DeviceAction)
    {
        action = static_cast<Action_t>(aEvent->DeviceEvent.Action);
        actor  = aEvent->DeviceEvent.Actor;
    }
    else if (aEvent->Type == AppEvent::kEventType_Button)
    {
        sTurnedOn = !sTurnedOn;

        PwmManager::getInstance().setPwm(PwmManager::EAppPwm_Red, sTurnedOn);
        GetAppTask().UpdateClusterState();
    }
}

void AppTask::UpdateClusterState(void)
{
    bool isTurnedOn = sTurnedOn;

    // write the new on/off value
    Protocols::InteractionModel::Status status = Clusters::OnOff::Attributes::OnOff::Set(kExampleEndpointId, isTurnedOn);

    if (status != Protocols::InteractionModel::Status::Success)
    {
        LOG_ERR("Update OnOff fail: %x", to_underlying(status));
    }
    uint8_t setLevel = sLevel;
    status           = Clusters::LevelControl::Attributes::CurrentLevel::Set(kExampleEndpointId, setLevel);
    if (status != Protocols::InteractionModel::Status::Success)
    {
        LOG_ERR("Update CurrentLevel fail: %x", to_underlying(status));
    }
    Device * dev_init = gDevices[light1_idx];
    if (dev_init->IsReachable())
    {
        EndpointId light_endpiont = LIGHT1_ENDPIONT;

        ChipLogProgress(DeviceLayer, "==add by clz:level value is %d in endpoint %d ", setLevel, light_endpiont);

        dev_init->SetLevel(setLevel);
    }
}
