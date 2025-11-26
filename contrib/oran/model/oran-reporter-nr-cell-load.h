/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * NIST-developed software is provided by NIST as a public service. You may
 * use, copy and distribute copies of the software in any medium, provided that
 * you keep intact this entire notice. You may improve, modify and create
 * derivative works of the software or any portion of the software, and you may
 * copy and distribute such modifications or works. Modified works should carry
 * a notice stating that you changed the software and should note the date and
 * nature of any such change. Please explicitly acknowledge the National
 * Institute of Standards and Technology as the source of the software.
 *
 * NIST-developed software is expressly provided "AS IS." NIST MAKES NO
 * WARRANTY OF ANY KIND, EXPRESS, IMPLIED, IN FACT OR ARISING BY OPERATION OF
 * LAW, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT AND DATA ACCURACY. NIST
 * NEITHER REPRESENTS NOR WARRANTS THAT THE OPERATION OF THE SOFTWARE WILL BE
 * UNINTERRUPTED OR ERROR-FREE, OR THAT ANY DEFECTS WILL BE CORRECTED. NIST
 * DOES NOT WARRANT OR MAKE ANY REPRESENTATIONS REGARDING THE USE OF THE
 * SOFTWARE OR THE RESULTS THEREOF, INCLUDING BUT NOT LIMITED TO THE
 * CORRECTNESS, ACCURACY, RELIABILITY, OR USEFULNESS OF THE SOFTWARE.
 *
 * You are solely responsible for determining the appropriateness of using and
 * distributing the software and you assume all risks associated with its use,
 * including but not limited to the risks and costs of program errors,
 * compliance with applicable laws, damage to or loss of data, programs or
 * equipment, and the unavailability or interruption of operation. This
 * software is not intended to be used in any situation where a failure could
 * cause risk of injury or damage to property. The software developed by NIST
 * employees is not subject to copyright protection within the United States.
 */

#ifndef ORAN_REPORTER_NR_CELL_LOAD_H
#define ORAN_REPORTER_NR_CELL_LOAD_H

//#include "ns3/nr-common.h" 
#include "ns3/nr-phy-mac-common.h"
#include "oran-report.h"
#include "oran-reporter.h"

#include "ns3/ptr.h"
//#include <ns3/nr-common.h>

#include <ns3/nr-ffr-algorithm.h>

#include <vector>

namespace ns3
{

/**
 * \ingroup oran
 *
 * Reporter that attaches to an NR eNB and captures its Cell load.
 */
class OranReporterNrCellLoad : public OranReporter
{
  public:
    /**
     * Get the TypeId of the OranReporterNrCellLoad class.
     *
     * \return The TypeId.
     */
    static TypeId GetTypeId(void);
    /**
     * Constructor of the OranReporterNrCellLoad class.
     */
    OranReporterNrCellLoad(void);
    /**
     * Destructor of the OranReporterNrCellLoad class.
     */
    ~OranReporterNrCellLoad(void) override;
    /**
     * Records the RB usage of each subframe.
     *
     *\param info Struct containing frame number, subframe number,
     * MCS, transport block size and other tx info
     */
    void DlScheduled(NrSchedulingCallbackInfo info);

  protected:
    /**
     * Get the load of the attached NR cell, and generate an
     * OranReportNrCellLoad.
     *
     * \return The generated Report.
     */
    std::vector<Ptr<OranReport>> GenerateReports(void) override;

  private:
	Ptr<NrFfrAlgorithm> m_ffr;
	/**
	 * The sum of RB usage of all subframes in the current measument cycle
	 */
    double m_rbUsageSum;

    uint8_t m_bandwidth;

	Time m_timeLastReport;
}; // class OranReporterNrCellLoad

} // namespace ns3

#endif /* ORAN_REPORTER_NR_CELL_LOAD_H */
