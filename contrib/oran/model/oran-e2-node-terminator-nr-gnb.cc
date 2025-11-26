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

#include "oran-e2-node-terminator-nr-gnb.h"

#include "oran-command-nr-2-nr-handover.h"

#include "ns3/abort.h"
#include "ns3/log.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-rrc.h"
#include "ns3/node.h"
#include "ns3/pointer.h"
#include "ns3/string.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranE2NodeTerminatorNrGnb");

NS_OBJECT_ENSURE_REGISTERED(OranE2NodeTerminatorNrGnb);

TypeId
OranE2NodeTerminatorNrGnb::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranE2NodeTerminatorNrGnb")
                            .SetParent<OranE2NodeTerminator>()
                            .AddConstructor<OranE2NodeTerminatorNrGnb>();

    return tid;
}

OranE2NodeTerminatorNrGnb::OranE2NodeTerminatorNrGnb()
    : OranE2NodeTerminator()
{
    NS_LOG_FUNCTION(this);
}

OranE2NodeTerminatorNrGnb::~OranE2NodeTerminatorNrGnb()
{
    NS_LOG_FUNCTION(this);
}

OranNearRtRic::NodeType
OranE2NodeTerminatorNrGnb::GetNodeType() const
{
    NS_LOG_FUNCTION(this);

    return OranNearRtRic::NodeType::NRGNB;
}

void
OranE2NodeTerminatorNrGnb::ReceiveCommand(Ptr<OranCommand> command)
{
    NS_LOG_FUNCTION(this << command);

    if (m_active)
    {
        //Confirms this command is a OranCommandNr2NrHandover (not some other ORAN command).
        if (command->GetInstanceTypeId() == OranCommandNr2NrHandover::GetTypeId())
        {
            Ptr<Node> node = GetNode();
            //Casts the generic OranCommand to the specific OranCommandNr2NrHandover 
            //So that we can access RNTI and target cell info
            Ptr<OranCommandNr2NrHandover> handoverCommand =
                command->GetObject<OranCommandNr2NrHandover>();
            //Fetches the RRC (Radio Resource Control) module from the gNB’s NetDevice. 
            //This is the module that controls UEs, including handovers.
            Ptr<NrGnbRrc> nrGnbRrc = GetNetDevice()->GetRrc();
            //The serving gNB uses RRC to send a handover request for the specified UE (RNTI) to the target cell (CellId).
            nrGnbRrc->SendHandoverRequest(handoverCommand->GetTargetRnti(),
                                           handoverCommand->GetTargetCellId());
        }
    }
}

Ptr<NrGnbNetDevice>
OranE2NodeTerminatorNrGnb::GetNetDevice() const
{
    NS_LOG_FUNCTION(this);

    Ptr<NrGnbNetDevice> nrGnbNetDev =
        GetNode()->GetDevice(GetNetDeviceIndex())->GetObject<NrGnbNetDevice>();

    NS_ABORT_MSG_IF(nrGnbNetDev == nullptr, "Unable to find appropriate network device");

    return nrGnbNetDev;
}

} // namespace ns3
