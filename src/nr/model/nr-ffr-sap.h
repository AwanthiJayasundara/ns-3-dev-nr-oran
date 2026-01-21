/*
 * Copyright (c) 2014 Piotr Gawlowicz
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Piotr Gawlowicz <gawlowicz.p@gmail.com>
 *
 */

#ifndef NR_FFR_SAP_H
#define NR_FFR_SAP_H

#include "nr-mac-sched-sap.h"

#include <map>

namespace ns3
{

/**
 * \brief Service Access Point (SAP) offered by the Frequency Reuse algorithm
 *        instance to the MAC Scheduler instance.
 *
 * This is the *NrFfrSapProvider*, i.e., the part of the SAP
 * that contains the Frequency Reuse algorithm methods called by the MAC Scheduler
 * instance.
 */
class NrFfrSapProvider
{
  public:
    virtual ~NrFfrSapProvider();

    /**
     * \brief Get vector of available RBG in DL for this Cell
     * \return vector of size (m_dlBandwidth/RbgSize); false indicates
     *                   that RBG is free to use, true otherwise
     *
     * This function is called by MAC Scheduler in the beginning of DL
     * scheduling process. Frequency Reuse Algorithm based on its policy
     * generates vector of RBG which can be used and which can not be used
     * by Scheduler to schedule transmission.
     */
    virtual std::vector<bool> GetAvailableDlRbg() = 0;

    /**
     * \brief Check if UE can be served on i-th RB in DL
     * \param i RBG ID
     * \param rnti Radio Network Temporary Identity, an integer identifying the UE
     *             where the report originates from
     * \return true if UE can be served on i-th RB, false otherwise
     *
     * This function is called by MAC Scheduler during DL scheduling process
     * to check if UE is allowed to be served with i-th RBG. Frequency Reuse
     * Algorithm based on its policy decides if RBG is allowed to UE.
     * If yes, Scheduler will try to allocate this RBG for UE, if not this UE
     * will not be served with this RBG.
     */
    virtual bool IsDlRbgAvailableForUe(int i, uint16_t rnti) = 0;

    /**
     * \brief Get vector of available RB in UL for this Cell
     * \return vector of size m_ulBandwidth; false indicates
     *                    that RB is free to use, true otherwise
     *
     * This function is called by MAC Scheduler in the beginning of UL
     * scheduling process. Frequency Reuse Algorithm based on its policy
     * generates vector of RB which can be used and which can not be used
     * by Scheduler to schedule transmission.
     */
    virtual std::vector<bool> GetAvailableUlRbg() = 0;

    /**
     * \brief Check if UE can be served on i-th RB in UL
     * \param i RB ID
     * \param rnti Radio Network Temporary Identity, an integer identifying the UE
     *             where the report originates from
     * \return true if UE can be served on i-th RB, false otherwise
     *
     * This function is called by MAC Scheduler during UL scheduling process
     * to check if UE is allowed to be served with i-th RB. Frequency Reuse
     * Algorithm based on its policy decides if RB is allowed to UE.
     * If yes, Scheduler will try to allocate this RB for UE, if not this UE
     * will not be served with this RB.
     */
    virtual bool IsUlRbgAvailableForUe(int i, uint16_t rnti) = 0;

    /**
     * \brief ReportDlCqiInfo
     * \param params the struct NrMacSchedSapProvider::SchedDlCqiInfoReqParameters
     */
    virtual void ReportDlCqiInfo(
        const NrMacSchedSapProvider::SchedDlCqiInfoReqParameters& params) = 0;


    /**
     * \brief ReportUlCqiInfo
     * \param params the struct NrMacSchedSapProvider::SchedUlCqiInfoReqParameters
     */
    virtual void ReportUlCqiInfo(
        const NrMacSchedSapProvider::SchedUlCqiInfoReqParameters& params) = 0;

    /**
     * \brief ReportUlCqiInfo
     * \param ulCqiMap the UL CQI map
     */
    virtual void ReportUlCqiInfo(std::map<uint16_t, std::vector<double>> ulCqiMap) = 0;

    /**
     * \brief GetTpc
     * \param rnti the RNTI
     * \returns the TCP
     */
    virtual uint8_t GetTpc(uint16_t rnti) = 0;

    /**
     * \brief Get the minimum continuous Ul bandwidth
     * \returns the minimum continuous UL bandwidth
     */
    virtual uint16_t GetMinContinuousUlBandwidth() = 0;
}; // end of class NrFfrSapProvider

/**
 * \brief Service Access Point (SAP) offered by the eNodeB RRC instance to the
 *        Frequency Reuse algorithm instance.
 *
 * This is the *NrFfrSapUser*, i.e., the part of the SAP that
 * contains the MAC Scheduler methods called by the Frequency Reuse algorithm instance.
 */
class NrFfrSapUser
{
  public:
    virtual ~NrFfrSapUser();

}; // end of class NrFfrSapUser

/**
 * \brief Template for the implementation of the NrFfrSapProvider
 *        as a member of an owner class of type C to which all methods are
 *        forwarded.
 */
template <class C>
class MemberNrFfrSapProvider : public NrFfrSapProvider
{
  public:
    /**
     * Constructor
     *
     * \param owner the owner class
     */
    MemberNrFfrSapProvider(C* owner);

    // Delete default constructor to avoid misuse
    MemberNrFfrSapProvider() = delete;

    // inherited from NrFfrSapProvider
    std::vector<bool> GetAvailableDlRbg() override;
    bool IsDlRbgAvailableForUe(int i, uint16_t rnti) override;
    std::vector<bool> GetAvailableUlRbg() override;
    bool IsUlRbgAvailableForUe(int i, uint16_t rnti) override;
    void ReportDlCqiInfo(const NrMacSchedSapProvider::SchedDlCqiInfoReqParameters& params) override;
    void ReportUlCqiInfo(const NrMacSchedSapProvider::SchedUlCqiInfoReqParameters& params) override;
    void ReportUlCqiInfo(std::map<uint16_t, std::vector<double>> ulCqiMap) override;
    uint8_t GetTpc(uint16_t rnti) override;
    uint16_t GetMinContinuousUlBandwidth() override;

  private:
    C* m_owner; ///< the owner class

}; // end of class MemberNrFfrSapProvider

template <class C>
MemberNrFfrSapProvider<C>::MemberNrFfrSapProvider(C* owner)
    : m_owner(owner)
{
}

template <class C>
std::vector<bool>
MemberNrFfrSapProvider<C>::GetAvailableDlRbg()
{
    return m_owner->DoGetAvailableDlRbg();
}

template <class C>
bool
MemberNrFfrSapProvider<C>::IsDlRbgAvailableForUe(int i, uint16_t rnti)
{
    return m_owner->DoIsDlRbgAvailableForUe(i, rnti);
}

template <class C>
std::vector<bool>
MemberNrFfrSapProvider<C>::GetAvailableUlRbg()
{
    return m_owner->DoGetAvailableUlRbg();
}

template <class C>
bool
MemberNrFfrSapProvider<C>::IsUlRbgAvailableForUe(int i, uint16_t rnti)
{
    return m_owner->DoIsUlRbgAvailableForUe(i, rnti);
}

template <class C>
void
MemberNrFfrSapProvider<C>::ReportDlCqiInfo(
    const NrMacSchedSapProvider::SchedDlCqiInfoReqParameters& params)
{
    m_owner->DoReportDlCqiInfo(params);
}

template <class C>
void
MemberNrFfrSapProvider<C>::ReportUlCqiInfo(
    const NrMacSchedSapProvider::SchedUlCqiInfoReqParameters& params)
{
    m_owner->DoReportUlCqiInfo(params);
}

template <class C>
void
MemberNrFfrSapProvider<C>::ReportUlCqiInfo(std::map<uint16_t, std::vector<double>> ulCqiMap)
{
    m_owner->DoReportUlCqiInfo(ulCqiMap);
}

template <class C>
uint8_t
MemberNrFfrSapProvider<C>::GetTpc(uint16_t rnti)
{
    return m_owner->DoGetTpc(rnti);
}

template <class C>
uint16_t
MemberNrFfrSapProvider<C>::GetMinContinuousUlBandwidth()
{
    return m_owner->DoGetMinContinuousUlBandwidth();
}

/**
 * \brief Template for the implementation of the NrFfrSapUser
 *        as a member of an owner class of type C to which all methods are
 *        forwarded.
 */
template <class C>
class MemberNrFfrSapUser : public NrFfrSapUser
{
  public:
    /**
     * Constructor
     *
     * \param owner the owner class
     */
    MemberNrFfrSapUser(C* owner);

    // Delete default constructor to avoid misuse
    MemberNrFfrSapUser() = delete;

  private:
    C* m_owner; ///< the owner class

}; // end of class NrFfrSapUser

template <class C>
MemberNrFfrSapUser<C>::MemberNrFfrSapUser(C* owner)
    : m_owner(owner)
{
}

} // end of namespace ns3

#endif /* NR_FFR_SAP_H */
