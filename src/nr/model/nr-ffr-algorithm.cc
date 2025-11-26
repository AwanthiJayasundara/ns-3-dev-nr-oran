/*
 * Copyright (c) 2014 Piotr Gawlowicz
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Piotr Gawlowicz <gawlowicz.p@gmail.com>
 *
 */

#include "nr-ffr-algorithm.h"

#include "ns3/boolean.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrFfrAlgorithm");

/// Type 0 RBG allocation (see table 7.1.6.1-1 of 3GPP TS 36.213)
static const int Type0AllocationRbg[4] = {
    10,  // RBG size 1
    26,  // RBG size 2
    63,  // RBG size 3
    110, // RBG size 4
};

NS_OBJECT_ENSURE_REGISTERED(NrFfrAlgorithm);

NrFfrAlgorithm::NrFfrAlgorithm()
    : m_needReconfiguration(true)
{
}

NrFfrAlgorithm::~NrFfrAlgorithm()
{
}

TypeId
NrFfrAlgorithm::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrFfrAlgorithm")
            .SetParent<Object>()
            .SetGroupName("Nr")
            .AddAttribute("FrCellTypeId",
                          "Downlink FR cell type ID for automatic configuration,"
                          "default value is 0 and it means that user needs to configure FR "
                          "algorithm manually,"
                          "if it is set to 1,2 or 3 FR algorithm will be configured automatically",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrFfrAlgorithm::SetFrCellTypeId,
                                               &NrFfrAlgorithm::GetFrCellTypeId),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("EnabledInUplink",
                          "If FR algorithm will also work in Uplink, default value true",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrFfrAlgorithm::m_enabledInUplink),
                          MakeBooleanChecker());
    return tid;
}

void
NrFfrAlgorithm::DoDispose()
{
    NS_LOG_FUNCTION(this);
}

uint16_t
NrFfrAlgorithm::GetUlBandwidth() const
{
    NS_LOG_FUNCTION(this);
    return m_ulBandwidth;
}

void
NrFfrAlgorithm::SetUlBandwidth(uint16_t bw)
{
    NS_LOG_FUNCTION(this << bw);
    switch (bw)
    {
    case 6:
    case 15:
    case 25:
    case 50:
    case 75:
    case 100:
        m_ulBandwidth = bw;
        break;

    default:
        NS_FATAL_ERROR("invalid bandwidth value " << bw);
        break;
    }
}

uint16_t
NrFfrAlgorithm::GetDlBandwidth() const
{
    NS_LOG_FUNCTION(this);
    return m_dlBandwidth;
}

void
NrFfrAlgorithm::SetDlBandwidth(uint16_t bw)
{
    NS_LOG_FUNCTION(this << bw);
    switch (bw)
    {
    case 6:
    case 15:
    case 25:
    case 50:
    case 75:
    case 100:
        m_dlBandwidth = bw;
        break;

    default:
        NS_FATAL_ERROR("invalid bandwidth value " << bw);
        break;
    }
}

void
NrFfrAlgorithm::SetFrCellTypeId(uint8_t cellTypeId)
{
    NS_LOG_FUNCTION(this << uint16_t(cellTypeId));
    m_frCellTypeId = cellTypeId;
    m_needReconfiguration = true;
}

uint8_t
NrFfrAlgorithm::GetFrCellTypeId() const
{
    NS_LOG_FUNCTION(this);
    return m_frCellTypeId;
}

int
NrFfrAlgorithm::GetRbgSize(int dlbandwidth)
{
    for (int i = 0; i < 4; i++)
    {
        if (dlbandwidth < Type0AllocationRbg[i])
        {
            return i + 1;
        }
    }

    return -1;
}

void
NrFfrAlgorithm::DoSetCellId(uint16_t cellId)
{
    NS_LOG_FUNCTION(this);
    m_cellId = cellId;
}

void
NrFfrAlgorithm::DoSetBandwidth(uint16_t ulBandwidth, uint16_t dlBandwidth)
{
    NS_LOG_FUNCTION(this);
    SetDlBandwidth(dlBandwidth);
    SetUlBandwidth(ulBandwidth);
}

} // end of namespace ns3
