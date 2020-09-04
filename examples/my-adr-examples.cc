/*
 * This program creates a simple network which uses an ADR algorithm to set up
 * the Spreading Factors of the devices in the Network.
 */

#include "ns3/point-to-point-module.h"
#include "ns3/forwarder-helper.h"
#include "ns3/network-server-helper.h"
#include "ns3/lora-channel.h"
#include "ns3/mobility-helper.h"
#include "ns3/lora-phy-helper.h"
#include "ns3/lorawan-mac-helper.h"
#include "ns3/lora-helper.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/periodic-sender.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/lora-device-address-generator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/config.h"
#include "ns3/rectangle.h"
#include "ns3/hex-grid-position-allocator.h"

#include <numeric>
#include <math.h>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("AdrExample");

// Trace sources that are called when a node changes its DR or TX power
void OnDataRateChange (uint8_t oldDr, uint8_t newDr)
{
  NS_LOG_DEBUG ("DR" << unsigned(oldDr) << " -> DR" << unsigned(newDr));
}
void OnTxPowerChange (double oldTxPower, double newTxPower)
{
  NS_LOG_DEBUG (oldTxPower << " dBm -> " << newTxPower << " dBm");
}

// Output control
bool print = true;
bool randomLoss_enabled = false;

int main (int argc, char *argv[])
{

  bool verbose = false;
  bool adrEnabled = true;
  bool initializeSF = true;
  int nDevices = 10;
  int nPeriods = 20;
  double mobileNodeProbability = 0;
  double sideLength = 5000;
  int gatewayDistance = 5000;
  double maxRandomLoss = 10;
  double minSpeed = 2;
  double maxSpeed = 16;
  std::string adrType = "ns3::AdrComponent";

  CommandLine cmd;
  cmd.AddValue ("verbose", "Whether to print output or not", verbose);
  cmd.AddValue ("MultipleGwCombiningMethod",
                "ns3::AdrComponent::MultipleGwCombiningMethod");
  cmd.AddValue ("MultiplePacketsCombiningMethod",
                "ns3::AdrComponent::MultiplePacketsCombiningMethod");
  cmd.AddValue ("HistoryRange", "ns3::AdrComponent::HistoryRange");
  cmd.AddValue ("MType", "ns3::EndDeviceLorawanMac::MType");
  cmd.AddValue ("EDDRAdaptation", "ns3::EndDeviceLorawanMac::EnableEDDataRateAdaptation");
  cmd.AddValue ("ChangeTransmissionPower",
                "ns3::AdrComponent::ChangeTransmissionPower");
   cmd.AddValue ("AdrEnabled", "Whether to enable ADR", adrEnabled);
   cmd.AddValue ("nDevices", "Number of devices to simulate", nDevices);
   cmd.AddValue ("PeriodsToSimulate", "Number of periods to simulate", nPeriods);
   cmd.AddValue ("MobileNodeProbability",
                 "Probability of a node being a mobile node",
                 mobileNodeProbability);
   cmd.AddValue ("sideLength",
                 "Length of the side of the rectangle nodes will be placed in",
                 sideLength);
   cmd.AddValue ("maxRandomLoss",
                 "Maximum amount in dB of the random loss component",
                 maxRandomLoss);
   cmd.AddValue ("gatewayDistance",
                 "Distance between gateways",
                 gatewayDistance);
   cmd.AddValue ("initializeSF",
                 "Whether to initialize the SFs",
                 initializeSF);
   cmd.AddValue ("MinSpeed",
                 "Minimum speed for mobile devices",
                 minSpeed);
   cmd.AddValue ("MaxSpeed",
                 "Maximum speed for mobile devices",
                 maxSpeed);
   cmd.AddValue ("MaxTransmissions",
                 "ns3::EndDeviceLorawanMac::MaxTransmissions");
   cmd.Parse (argc, argv);

   // int gatewayRings = 2 + (std::sqrt(2) * sideLength) / (gatewayDistance);
   // int nGateways = 3*gatewayRings*gatewayRings-3*gatewayRings+1;
   int nGateways = 1;

   // Logging
   //////////

   // LogComponentEnable ("AdrExample", LOG_LEVEL_ALL);
   // LogComponentEnable ("LoraPacketTracker", LOG_LEVEL_ALL);
   // LogComponentEnable ("NetworkServer", LOG_LEVEL_ALL);
   // LogComponentEnable ("NetworkController", LOG_LEVEL_ALL);
   // LogComponentEnable ("NetworkScheduler", LOG_LEVEL_ALL);
   // LogComponentEnable ("NetworkStatus", LOG_LEVEL_ALL);
   // LogComponentEnable ("EndDeviceStatus", LOG_LEVEL_ALL);
   // LogComponentEnable ("AdrComponent", LOG_LEVEL_ALL);
   // LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
   // LogComponentEnable ("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
   // LogComponentEnable ("MacCommand", LOG_LEVEL_ALL);
   // LogComponentEnable ("AdrExploraSf", LOG_LEVEL_ALL);
   // LogComponentEnable ("AdrExploraAt", LOG_LEVEL_ALL);
   // LogComponentEnable ("EndDeviceLorawanMac", LOG_LEVEL_ALL);
   LogComponentEnableAll (LOG_PREFIX_FUNC);
   LogComponentEnableAll (LOG_PREFIX_NODE);
   LogComponentEnableAll (LOG_PREFIX_TIME);

   // Set the EDs to require Data Rate control from the NS
   Config::SetDefault ("ns3::EndDeviceLorawanMac::DRControl", BooleanValue (true));

   // Create a simple wireless channel
   ///////////////////////////////////

   Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
   loss->SetPathLossExponent (3.76);
   loss->SetReference (1, 7.7);
   if (randomLoss_enabled){
     Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
     x->SetAttribute ("Min", DoubleValue (0.0));
     x->SetAttribute ("Max", DoubleValue (maxRandomLoss));

     Ptr<RandomPropagationLossModel> randomLoss = CreateObject<RandomPropagationLossModel> ();
     randomLoss->SetAttribute ("Variable", PointerValue (x));

     loss->SetNext (randomLoss);
   }

   Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

   Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

   // Helpers
   //////////

   // End Device mobility
   MobilityHelper mobilityEd, mobilityGw;
   mobilityEd.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
                                    "X", PointerValue (CreateObjectWithAttributes<UniformRandomVariable>
                                                       ("Min", DoubleValue(-sideLength),
                                                        "Max", DoubleValue(sideLength))),
                                    "Y", PointerValue (CreateObjectWithAttributes<UniformRandomVariable>
                                                       ("Min", DoubleValue(-sideLength),
                                                        "Max", DoubleValue(sideLength))));

   // // Gateway mobility
   // Ptr<ListPositionAllocator> positionAllocGw = CreateObject<ListPositionAllocator> ();
   // positionAllocGw->Add (Vector (0.0, 0.0, 15.0));
   // positionAllocGw->Add (Vector (-5000.0, -5000.0, 15.0));
   // positionAllocGw->Add (Vector (-5000.0, 5000.0, 15.0));
   // positionAllocGw->Add (Vector (5000.0, -5000.0, 15.0));
   // positionAllocGw->Add (Vector (5000.0, 5000.0, 15.0));
   // mobilityGw.SetPositionAllocator (positionAllocGw);
   // mobilityGw.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
   Ptr<HexGridPositionAllocator> hexAllocator = CreateObject<HexGridPositionAllocator> (gatewayDistance / 2);
   mobilityGw.SetPositionAllocator (hexAllocator);
   mobilityGw.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

   // Create the LoraPhyHelper
   LoraPhyHelper phyHelper = LoraPhyHelper ();
   phyHelper.SetChannel (channel);

   // Create the LorawanMacHelper
   LorawanMacHelper macHelper = LorawanMacHelper ();

   // Create the LoraHelper
   LoraHelper helper = LoraHelper ();
   helper.EnablePacketTracking ();

   ////////////////
   // Create GWs //
   ////////////////

   NodeContainer gateways;
   gateways.Create (nGateways);
   mobilityGw.Install (gateways);

   // Create the LoraNetDevices of the gateways
   phyHelper.SetDeviceType (LoraPhyHelper::GW);
   macHelper.SetDeviceType (LorawanMacHelper::GW);
   helper.Install (phyHelper, macHelper, gateways);

   // Create EDs
   /////////////

   NodeContainer endDevices;
   endDevices.Create (nDevices);
   // Verificar e alterar pra ter só eds estáticos na rede ###############################
   // Install mobility model on fixed nodes
   mobilityEd.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
   mobilityEd.Install (endDevices);

  // Make it so that nodes are at a certain height > 0
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<MobilityModel> mobilityEd = (*j)->GetObject<MobilityModel> ();
      Vector position = mobilityEd->GetPosition ();
      position.z = 1.2;
      mobilityEd->SetPosition (position);
    }

   
  //  int fixedPositionNodes = double (nDevices) * (1 - mobileNodeProbability);
  //  for (int i = 0; i < fixedPositionNodes; ++i)
  //   {
  //     mobilityEd.Install (endDevices.Get (i));
  //   }
  // // Install mobility model on mobile nodes
  // mobilityEd.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
  //                              "Bounds", RectangleValue (Rectangle (-sideLength, sideLength,
  //                                                                   -sideLength, sideLength)),
  //                              "Distance", DoubleValue (1000),
  //                              "Speed", PointerValue (CreateObjectWithAttributes<UniformRandomVariable>
  //                                                     ("Min", DoubleValue(minSpeed),
  //                                                      "Max", DoubleValue(maxSpeed))));
  // for (int i = fixedPositionNodes; i < (int) endDevices.GetN (); ++i)
  //   {
  //     mobilityEd.Install (endDevices.Get (i));
  //   }

  // Create a LoraDeviceAddressGenerator
  uint8_t nwkId = 54;
  uint32_t nwkAddr = 1864;
  Ptr<LoraDeviceAddressGenerator> addrGen = CreateObject<LoraDeviceAddressGenerator> (nwkId,nwkAddr);

  // Create the LoraNetDevices of the end devices
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  macHelper.SetAddressGenerator (addrGen);
  macHelper.SetRegion (LorawanMacHelper::EU);
  helper.Install (phyHelper, macHelper, endDevices);

  // Install applications in EDs
  int appPeriodSeconds = 1200;      // One packet every 20 minutes
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (appPeriodSeconds));
  ApplicationContainer appContainer = appHelper.Install (endDevices);

  // Do not set spreading factors up: we will wait for the NS to do this
  if (initializeSF)
    {
      // macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);
      
      double ToAvec[] {49.41, 90.62, 164.86, 329.73, 659.46, 1187.84};
      int length = *(&ToAvec + 1) - ToAvec;
      double sum_array = std::accumulate(ToAvec, ToAvec+length, 0.0);
      double w[length];
      unsigned int limits[length];

      for (int i=0;i<length;i++){
         std::cout << ToAvec[i] << " ";
      }
      std::cout<<std::endl;
      for (int i=0;i<length;i++){
        w[i] = 1/ToAvec[i]/sum_array;
        std::cout << w[i] << " ";
      }
      std::cout<<std::endl;
      double sum_w = std::accumulate(w, w+length, 0.0);
      for (int i=0;i<length;i++){
        limits[i] = rint(w[i]/sum_w*nDevices);
        std::cout << limits[i] << " ";
      }
      std::cout<<std::endl;
      unsigned int sum_limits = std::accumulate(limits, limits+length, 0.0);
      std::cout<<sum_limits<<std::endl;

      // RSSIMat create
      double RSSIMat[nGateways][nDevices];
      int gw = 0;
      for (NodeContainer::Iterator it_gw = gateways.Begin (); it_gw != gateways.End (); ++it_gw,++gw){
        Ptr<Node> object = *it_gw;
        Ptr<MobilityModel> gw_mobility = (*it_gw)->GetObject<MobilityModel> ();
        // Vector position = gw_mobility->GetPosition ();
        int ed = 0;
        for (NodeContainer::Iterator it_ed = endDevices.Begin (); it_ed != endDevices.End (); ++it_ed,++ed){
          Ptr<Node> object = *it_ed;
          Ptr<MobilityModel> ed_mobility = (*it_ed)->GetObject<MobilityModel> ();
          // Ptr<NetDevice> netDevice = object->GetDevice (0);
          // Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
          // NS_ASSERT (loraNetDevice != 0);
          // Ptr<ClassAEndDeviceLorawanMac> mac =
          //     loraNetDevice->GetMac ()->GetObject<ClassAEndDeviceLorawanMac> ();
          // NS_ASSERT (mac != 0);
          // int DR = mac->GetDataRate ();
          RSSIMat[gw][ed] = channel->GetRxPower (14, ed_mobility, gw_mobility);
        }
      }
      // int gw = 0;
      std::cout << "RSSIMat" << std::endl;
      for (int i = 0; i < nGateways; i++){
        for (int j = 0; j < nDevices; j++){
          std::cout << RSSIMat[i][j] << " ";
        }
        std::cout << std::endl;
      }
      // // Considerando que todos os GWs tem a mesma sensibilidade,
      // // vou usar a sensibilidade do primeiro gateway como referência.
      // Ptr<Node> firstGW = gateways.Get (0);
      // Ptr<NetDevice> gatewayNetDevice = firstGW->GetDevice (0);
      // Ptr<LoraNetDevice> gatewayLoraNetDevice = gatewayNetDevice->GetObject<LoraNetDevice> ();
      // Ptr<GatewayLoraPhy> gatewayPhy = gatewayLoraNetDevice->GetPhy ()->GetObject<GatewayLoraPhy> ();
      // const double *gwSensitivity = gatewayPhy->sensitivity;
      const double gwSensitivity[6] = {-130.0, -132.5, -135.0, -137.5, -140.0, -142.5};
      unsigned int sf = 7;

      std::cout << "RSSIMat" << std::endl;
      for (int i = 0; i < nGateways; i++){
        for (int j = 0; j < nDevices; j++){
          std::cout << RSSIMat[i][j] << " ";// ##########################
        }
        std::cout << std::endl;
      }
      // std::cout << "GW sensitivity: ";
      // for(unsigned int i=0; i<sizeof(gwSensitivity)/sizeof(gwSensitivity[0]);i++){
      //   std::cout << gwSensitivity[i] << " ";
      // }
      // std::cout << std::endl;
    }

  ////////////
  // Create NS
  ////////////

  NodeContainer networkServers;
  networkServers.Create (1);

  // Install the NetworkServer application on the network server
  NetworkServerHelper networkServerHelper;
  networkServerHelper.SetGateways (gateways);
  networkServerHelper.SetEndDevices (endDevices);
  networkServerHelper.EnableAdr (adrEnabled);
  networkServerHelper.SetAdr (adrType);
  networkServerHelper.Install (networkServers);

  // Install the Forwarder application on the gateways
  ForwarderHelper forwarderHelper;
  forwarderHelper.Install (gateways);

  // Connect our traces
  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/TxPower",
                                 MakeCallback (&OnTxPowerChange));
  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/DataRate",
                                 MakeCallback (&OnDataRateChange));

  // Activate printing of ED MAC parameters
  Time stateSamplePeriod = Seconds (1200);
  helper.EnablePeriodicDeviceStatusPrinting (endDevices, gateways, "nodeData.txt", stateSamplePeriod);
  helper.EnablePeriodicPhyPerformancePrinting (gateways, "phyPerformance.txt", stateSamplePeriod);
  helper.EnablePeriodicGlobalPerformancePrinting ("globalPerformance.txt", stateSamplePeriod);

  LoraPacketTracker& tracker = helper.GetPacketTracker ();
  
  //////////////////////////////////////////////////////////////
  // Print nodes
  if (print)
    {
      std::ofstream myfile;
      myfile.open ("nodes.txt");
      // std::vector<Ptr<Building>>::const_iterator it;
      int j = 1;
      for (NodeContainer::Iterator it = endDevices.Begin (); it != endDevices.End (); ++it, ++j)
        {
          Ptr<Node> object = *it;
          Ptr<NetDevice> netDevice = object->GetDevice (0);
          Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
          NS_ASSERT (loraNetDevice != 0);
          Ptr<ClassAEndDeviceLorawanMac> mac =
              loraNetDevice->GetMac ()->GetObject<ClassAEndDeviceLorawanMac> ();
          NS_ASSERT (mac != 0);
          int DR = mac->GetDataRate ();
          int SF = 12-DR;
          Ptr<MobilityModel> mobility = (*it)->GetObject<MobilityModel> ();
          
          Vector position = mobility->GetPosition ();
          Ptr<Node> bestGateway = gateways.Get (0);
          Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel> ();
          double RxPower = channel->GetRxPower (14, mobility, bestGatewayPosition);
          myfile << position.x << " " << position.y << " " << SF << " " << RxPower << std::endl;
          // myfile << "node " << j << " coords " << position.x << "," << position.y << "," << 
          // position.z << ";" << " DataRate " << DR << std::endl;
        }
      myfile.close ();
    }
  // Print GWs
  if (print)
    {
      std::ofstream myfile;
      myfile.open ("gws.txt");
      // std::vector<Ptr<Building>>::const_iterator it;
      int j = 1;
      for (NodeContainer::Iterator it = gateways.Begin (); it != gateways.End (); ++it, ++j)
        {
          Ptr<Node> object = *it;
          Ptr<MobilityModel> mobility = (*it)->GetObject<MobilityModel> ();
          Vector position = mobility->GetPosition ();
          // myfile << "gateway " << j << " coords " << position.x << "," << position.y << "," << 
          // position.z << std::endl;
          myfile << position.x << " " << position.y << std::endl;
        }
      myfile.close ();
    }
  //////////////////////////////////////////////////////////

  // Start simulation
  // Time simulationTime = Seconds (1200 * nPeriods);
  Time simulationTime = Hours (1);
  Simulator::Stop (simulationTime);
  Simulator::Run ();
  Simulator::Destroy ();

  // std::cout << tracker.CountMacPacketsGlobally(Seconds (1200 * (nPeriods - 2)),
  //                                              Seconds (1200 * (nPeriods - 1))) << std::endl;
  std::cout << tracker.CountMacPacketsGlobally(Seconds (0),
                                               simulationTime) << std::endl;

  return 0;
}
