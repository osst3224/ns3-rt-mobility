#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/sionna-helper.h"
#include "ns3/sionna-connection-handler.h"

#include "ns3/point-to-point-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/ipv4-flow-classifier.h"

#include "ns3/udp-echo-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/random-variable-stream.h"
#include "ns3/internet-apps-module.h"  // for OnOffHelper, PacketSinkHelper

#include "ue-trajectory.h"

#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/propagation-loss-model.h"
#include <fstream>
#include <cmath>


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LteRandomMobilityAndTrafficWithSionna");


std::map<FlowId, uint64_t> previousRxBytes;
std::map<FlowId, uint32_t> previousTxPackets;
std::map<FlowId, uint32_t> previousRxPackets;
std::map<FlowId, double> previousJitterSum;
NodeContainer ueNodes;
NodeContainer enbNodes;
Ptr<Ipv4FlowClassifier> classifier;
NetDeviceContainer ueDevs;
int pktsize;


static const std::array<double,24> hourlyMean = {
    0.5,        // 00
    0.4,        // 01
    0.3,        // 02
    0.3,        // 03
    0.33333333, // 04
    0.36666667, // 05
    0.4,        // 06
    0.5,        // 07
    0.6,        // 08
    0.8,        // 09
    1.0,        // 10
    1.2,        // 11
    1.4,        // 12
    1.4,        // 13
    1.2,        // 14
    1.1,        // 15
    1.0,        // 16
    0.93333333, // 17
    0.9,        // 18
    0.8,        // 19
    0.7,        // 20
    0.6,        // 21
    0.5,        // 22
    0.5         // 23

};


static const std::array<double,24> hourlySigma = {
  /* 00 */ 0.05, 0.05, 0.04, 0.04, 0.05, 0.07,
  /* 06 */ 0.10, 0.12, 0.15, 0.15, 0.15, 0.12,
  /* 12 */ 0.10, 0.10, 0.10, 0.10, 0.10, 0.08,
  /* 18 */ 0.08, 0.07, 0.06, 0.05, 0.05, 0.05
};


void PositionChanged(Ptr<const MobilityModel> mobility) {
    Vector pos = mobility->GetPosition();
    NS_LOG_UNCOND("Node " << mobility->GetObject<Node>()->GetId() 
                 << " moved. New position is: x=" << pos.x 
                 << ", y=" << pos.y);
}


//uint16_t GetServingCellId(Ptr<Node> ueNode) {
Ptr<LteUeNetDevice> GetServingCellId(Ptr<Node> ueNode) {
    for (uint32_t i = 0; i < ueNode->GetNDevices(); ++i) {
        Ptr<LteUeNetDevice> ueDev = ueNode->GetDevice(i)->GetObject<LteUeNetDevice>();
        if (ueDev) {
            return ueDev;
            //return ueDev->GetRrc()->GetCellId();
        }
    }
    return 0;  // 0 = not found
}


Ptr<Node> GetEnbNodeByCellId(uint16_t cellId, size_t& numConnectedUes)
{
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
    {
        Ptr<Node> nodeN = enbNodes.Get(i);
        NS_ABORT_MSG_IF(!nodeN, "Error: Ptr<Node> nodeN in FindUeByIp not linked to a Node. This is needed for Sionna to track the object location!");
        Ptr<LteEnbNetDevice> enbDev = enbNodes.Get(i)->GetDevice(0)->GetObject<LteEnbNetDevice>();
        if (enbDev && enbDev->GetCellId() == cellId)
        {  
            Ptr<LteEnbRrc> rrc = enbDev->GetRrc();
            numConnectedUes = rrc->GetUeMap().size();
            return enbNodes.Get(i);
        }
    }
    std::cout << "Nullptr CellId " << cellId << std::endl;
    return nullptr;
}


Ptr<Node> FindUeByIp(Ipv4Address addr) {
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i) {
        Ptr<Node> nodeN = ueNodes.Get(i);
        NS_ABORT_MSG_IF(!nodeN, "Error: Ptr<MobilityModel> in FindUeByIp not linked to a Node. This is needed for Sionna to track the object location!");
        Ptr<Ipv4> ipv4 = nodeN->GetObject<Ipv4>();
        for (uint32_t j = 0; j < ipv4->GetNInterfaces(); ++j) {
            for (uint32_t k = 0; k < ipv4->GetNAddresses(j); ++k) {
                //std::cout << "IP address: " << ipv4->GetAddress(j, k).GetLocal() << std::endl;
                if (ipv4->GetAddress(j, k).GetLocal() == addr) {
                    return ueNodes.Get(i);
                    //std::cout << "UEid " << i << std::endl;
                }
            }
        }
    }
    return nullptr;
}


bool uesSharePosition(std::vector<std::vector<Point3D>>& sampled_coords) {
    size_t numUEs = sampled_coords.size();
    if (numUEs == 0) return false;

    // Find the minimum time points across all UEs
    size_t minTime = sampled_coords[0].size();
    for (const auto& ue_positions : sampled_coords) {
        if (ue_positions.size() < minTime) {
            minTime = ue_positions.size();
        }
    }

    for (size_t t = 0; t < minTime; ++t) {
        for (size_t i = 0; i < numUEs; ++i) {
            const Point3D& pt1 = sampled_coords[i][t];
            for (size_t j = i + 1; j < numUEs; ++j) {
                const Point3D& pt2 = sampled_coords[j][t];
                if (pt1.x == pt2.x && pt1.y == pt2.y) {
                    // Found two UEs at the same position at time t
                    return true;
                }
            }
        }
    }

    return false; // No UEs share the same position at the same time
}


void LogFlowMonitorStats(Ptr<FlowMonitor> monitor, std::ofstream* outFile, double interval, double simTime, std::vector<int>& uePktSize) {
    monitor->CheckForLostPackets();  // Refresh flow stats

    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    double currentTime = Simulator::Now().GetMilliSeconds();
    currentTime = currentTime/1000;
    double intShift = 0;

    if (currentTime == interval) {
        intShift = 0.5;
    }

    for (const auto& flow : stats) {
        FlowId flowId = flow.first;
        const FlowMonitor::FlowStats& f = flow.second;
        
        // Match flowId to a UE
        auto flowTuple = classifier->FindFlow(flowId);
        Ipv4Address src = flowTuple.destinationAddress;
        
        Ptr<Node> ueNodeTemp = FindUeByIp(src);  
        if (!ueNodeTemp) {
            //Simulator::Schedule(Seconds(interval), &LogFlowMonitorStats, monitor, outFile, interval, simTime);
            //NS_ABORT_MSG_IF(!servingEnb, "Error: Ptr<Node> servingEnb not linked to a Cell. This is needed for Sionna to track the object location!");
            std::cout << "No ue found for for IP: " << src << std::endl;
            //return;
        } else {
            Ptr<MobilityModel> ueMob = ueNodeTemp->GetObject<MobilityModel>();
            Vector uePos = ueMob->GetPosition();

            Ptr<LteUeNetDevice> ueDev = GetServingCellId(ueNodeTemp);
            uint16_t cellId = ueDev->GetRrc()->GetCellId();


            if (cellId == 0) {
                //Simulator::Schedule(Seconds(interval), &LogFlowMonitorStats, monitor, outFile, interval, simTime);
                //NS_ABORT_MSG_IF(!servingEnb, "Error: Ptr<Node> servingEnb not linked to a Cell. This is needed for Sionna to track the object location!");
                std::cout << "No cellId for UE." << std::endl;
                //return;
            } else {
                size_t numUe;
                Ptr<Node> servingEnb = GetEnbNodeByCellId(cellId, numUe);
                if (!servingEnb) {
                    //Simulator::Schedule(Seconds(interval), &LogFlowMonitorStats, monitor, outFile, interval, simTime);
                    //NS_ABORT_MSG_IF(!servingEnb, "Error: Ptr<Node> servingEnb not linked to a Cell. This is needed for Sionna to track the object location!");
                    std::cout << "No eNB for cellId: " << cellId << std::endl;
                    //return;
                } else {
                    Ptr<MobilityModel> enbMob = servingEnb->GetObject<MobilityModel>();
                    Vector ueVelocity = ueMob->GetVelocity();
                    double ueDirection = atan2(ueVelocity.y, ueVelocity.x) * 180.0 / M_PI;
                    double ueSpeedKmh = 3.6*std::sqrt(ueVelocity.x * ueVelocity.x + ueVelocity.y * ueVelocity.y);

                    Ptr<LteUePhy> uePhy = ueDev->GetPhy();
                    double sinr = uePhy->GetSinrDb();
                    double rsrp = uePhy->GetRsrpDbm();

                    auto [sionnaPower, losStatusSionna] = SionnaPowerLos(servingEnb->GetId(), ueNodeTemp->GetId()); //PropagationLossModel::

                    // Compute throughput in kbps (kilobits per second)
                    uint64_t currentRxBytes = f.rxBytes;
                    uint64_t prevRxBytes = previousRxBytes[flowId];
                    double throughputKbps = ((currentRxBytes - prevRxBytes) * 8.0) / ((interval-intShift) * 1000.0);
                    previousRxBytes[flowId] = currentRxBytes;

                    // --- Packet Loss Rate (per-interval) ---
                    uint32_t deltaTx = f.txPackets - previousTxPackets[flowId];
                    uint32_t deltaRx = f.rxPackets - previousRxPackets[flowId];
                    double lossRate = (deltaTx > 0) ? ((double)(deltaTx - deltaRx) / deltaTx) : 0.0;

                    if (lossRate > 1) 
                    {
                        lossRate = 0; // Set lossRate to 0 if it is greater than 1
                    }   
                
                    previousTxPackets[flowId] = f.txPackets;
                    previousRxPackets[flowId] = f.rxPackets;
                
                    // --- Average delay ---
                    double avgDelay = (f.rxPackets > 0) ? (f.delaySum.GetSeconds() / f.rxPackets) : 0.0;
                
                    // --- Jitter per interval ---
                    double jitterSum = f.jitterSum.GetSeconds();
                    double deltaJitter = jitterSum - previousJitterSum[flowId];
                    previousJitterSum[flowId] = jitterSum;

                    *outFile << currentTime << ","
                             << flowId << ","
                             << cellId << ","
                             << numUe << ","
                             << uePos.x << ","
                             << uePos.y << ","
                             << ueSpeedKmh << ","
                             << ueDirection << ","
                             << uePktSize[flowId-1] << ","
                             << deltaTx << ","         // Tx this interval
                             << deltaRx << ","         // Rx this interval
                             << avgDelay << ","
                             << throughputKbps << ","
                             << lossRate << ","                     
                             << deltaJitter << ","       
                             << sinr << ","                         
                             << rsrp << ","                      
                             << sionnaPower << ","
                             << losStatusSionna << std::endl;
                }
            }
        }

        

    }

    // Schedule next sample, unless we've reached end of simulation
    if (currentTime + interval <= simTime) {
        Simulator::Schedule(Seconds(interval), &LogFlowMonitorStats, monitor, outFile, interval, simTime, uePktSize);
    }
}

int main(int argc, char* argv[])
{
    double txPower = 30;  // dBm. 
    uint16_t earfcn = 100;  // EARFCN (2100 MHz)
    double simTime = 240.1; // Your Simulator::Stop time
    double interval = 1.0; // Sampling time measurements in seconds
    uint32_t numCars = 30;
    int ueSamplingTime = 100; // ms
    double ueVelocity = 60.0; // kmh
    int RBs = 25;
    std::string datarate_p2pRH = "60"; //70
    std::string scheduler = "Cqa";
    // RrFfMacScheduler // Round-robin, simple fair scheduler
    // PfFfMacScheduler // 	Proportional Fair Frequency-Fair: Throughput-fairness tradeoff
    // TdBetFfMacScheduler // Throughput-Delay Best Effort Traffic: Throughput + delay balancing
    // CqaFfMacScheduler // Channel Quality Aware scheduler: Uses CQI for throughput/fairness
    // FdBetFfMacScheduler // 	Frequency-Domain Best Effort Throughput: Best-effort, throughput focused

    std::vector<Vector> enbPositions = {
        Vector(640,  490, 50),
        Vector(-690,  490, 50),
        Vector(640,  -530, 50),
        Vector(-690, -530, 50),
        Vector(75, -3, 29),
        Vector(-335, 295, 30),
        Vector(-190, -395, 28),
        Vector(-75, -110, 29),
        Vector(445, 98, 29), 
        Vector(0, 495, 50),
        Vector(230, -190, 30),
        Vector(-488, 40, 30)
    };
    
    bool sionna = true; // Use Sionna
    std::string server_ip = argv[1]; // external host
    bool local_machine = false;
    bool verb = false;
    int hour = std::stoi(argv[2]);//5;
    double m = hourlyMean[hour];
    double s = hourlySigma[hour];
    pktsize = static_cast<int>(m*1500.0);

    SionnaHelper& sionnaHelper = SionnaHelper::GetInstance();
    sionnaHelper.SetSionna(sionna); // Enable Sionna
    sionnaHelper.SetServerIp(server_ip); // Set server IP (in this case, localhost)
    sionnaHelper.SetLocalMachine(local_machine); // Set True if Sionna is running locally, as in this example
    sionnaHelper.SetVerbose(verb); // Enable verbose logging

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    
    // Setting all eNBs to the same frequency
    lteHelper->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(earfcn));
    lteHelper->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(earfcn + 18000));
    lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(RBs)); // #RBs. 25RBs = 4.5MHz Bandwidth, 50 RBs (~9 MHz) // Think default is 10 MHz
    lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(RBs)); // #RBs. 25RBs = 4.5MHz Bandwidth, 50 RBs (~9 MHz) // Think default is 10 MHz

    Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator>();

    for (const auto& pos : enbPositions)
    {
        enbPositionAlloc->Add(pos);
    }

    enbNodes.Create(enbPositions.size());
    ueNodes.Create(numCars);

    // Introduce mobility for eNB (static) and UEs (random walk, for example)
    MobilityHelper mobilityEnb;
    mobilityEnb.SetPositionAllocator(enbPositionAlloc);
    mobilityEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityEnb.Install(enbNodes);

    auto [uw_start, uw_end] = getUWPoints();
    auto [ue_start, ue_end] = getUEPoints();
    auto [mw_start, mw_end] = getMWPoints();
    auto [me_start, me_end] = getMEPoints();
    auto [lw_start, lw_end] = getLWPoints();
    auto [le_start, le_end] = getLEPoints();
    
    // Low load on lower, mid-high load on middle, and high load on upper side
    Graph graph_uw = build_connection_graph_xyz(uw_start, uw_end); // 25%
    Graph graph_ue = build_connection_graph_xyz(ue_start, ue_end); // 25%
    Graph graph_mw = build_connection_graph_xyz(mw_start, mw_end); // 15%
    Graph graph_me = build_connection_graph_xyz(me_start, me_end); // 15%
    Graph graph_lw = build_connection_graph_xyz(lw_start, lw_end); // 10%
    Graph graph_le = build_connection_graph_xyz(le_start, le_end); // 10%

    uint32_t lwNumCars = numCars*0.1;
    uint32_t leNumCars = numCars*0.1;
    uint32_t mwNumCars = numCars*0.15;
    uint32_t meNumCars = numCars*0.15;
    uint32_t uwNumCars = numCars*0.2;
    uint32_t ueNumCars = numCars - (lwNumCars+leNumCars+mwNumCars+meNumCars+uwNumCars);

    std::vector<std::vector<Point3D>> sampled_coords;
    std::vector<double> sampled_times;

    // Pass all graphs and inform how to divide number of cars
    int seed = 42;
    std::mt19937 rng(seed);
    
    simulateUEPaths(graph_uw, uwNumCars, sampled_coords, sampled_times, rng, ueSamplingTime, ueVelocity*0.5, simTime); 
    simulateUEPaths(graph_ue, ueNumCars, sampled_coords, sampled_times, rng, ueSamplingTime, ueVelocity*0.5, simTime); 
    simulateUEPaths(graph_mw, mwNumCars, sampled_coords, sampled_times, rng, ueSamplingTime, ueVelocity*0.9, simTime); 
    simulateUEPaths(graph_me, meNumCars, sampled_coords, sampled_times, rng, ueSamplingTime, ueVelocity*0.9, simTime); 
    simulateUEPaths(graph_lw, lwNumCars, sampled_coords, sampled_times, rng, ueSamplingTime, ueVelocity*1.2, simTime); 
    simulateUEPaths(graph_le, leNumCars, sampled_coords, sampled_times, rng, ueSamplingTime, ueVelocity*1.2, simTime); 

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    mobility.Install(ueNodes);

    for (size_t ue_id = 0; ue_id < sampled_coords.size(); ++ue_id) {
        Ptr<WaypointMobilityModel> mobility = ueNodes.Get(ue_id)->GetObject<WaypointMobilityModel>();

        for (size_t t = 0; t < sampled_times.size(); ++t) {
            const Point3D& pt = sampled_coords[ue_id][t];
            double time = sampled_times[t];
        
            Waypoint waypoint(Seconds(time), Vector(pt.x, pt.y, pt.z));
            mobility->AddWaypoint(waypoint);
        }
    }

    // Assert that ...
    NS_ABORT_MSG_IF(uesSharePosition(sampled_coords), "Error: Two UEs loacated at the same place at the same time!");

    // Create and install LTE devices
    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbNodes);
    ueDevs = lteHelper->InstallUeDevice(ueNodes);

    // 1. Create “remoteHostNode” and install the Internet stack on it
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHostNode = remoteHostContainer.Get(0);

    InternetStackHelper rhInternet;
    rhInternet.Install(remoteHostContainer);

    // 2. Retrieve the PGW node (part of the EPC helper) 
    // Commenting out this Ptr<PointToPointEpcHelper> epcHelper = DynamicCast<PointToPointEpcHelper>(lteHelper->GetEpcHelper());
    Ptr<Node> pgwNode = epcHelper->GetPgwNode();


    // Install Internet stack on routers
    InternetStackHelper internet;
    internet.Install(ueNodes);
    internet.Install(enbNodes); 

    // --- 3) Finally, your original PGW <-> remoteHost 1 Gb/s link — also fresh: ---
    PointToPointHelper p2pRH;
    p2pRH.SetDeviceAttribute("DataRate", DataRateValue(DataRate(datarate_p2pRH + "Mb/s")));
    p2pRH.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
    NetDeviceContainer internetDevices = p2pRH.Install(pgwNode, remoteHostNode);

    Ipv4AddressHelper ipv4rh;
    ipv4rh.SetBase("7.0.1.0", Ipv4Mask("255.255.255.0"));
    Ipv4InterfaceContainer internetIpIfaces = ipv4rh.Assign(internetDevices);

    Ipv4Address pgwAddr        = internetIpIfaces.GetAddress(0);

    // 5. Add a default route on remoteHostNode so it sends anything “off‑net” back to PGW
    Ipv4StaticRoutingHelper   staticRoutingHelper;
    Ptr<Ipv4StaticRouting>    remoteHostStaticRouting = staticRoutingHelper.GetStaticRouting(remoteHostNode->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("0.0.0.0"), Ipv4Mask("0.0.0.0"), pgwAddr, 1);

    Ipv4InterfaceContainer ueIpv4Interfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    // Set TX power for eNB and UEs
    for (uint32_t i = 0; i < enbDevs.GetN(); ++i) {
        enbDevs.Get(i)->GetObject<LteEnbNetDevice>()->GetPhy()->SetTxPower(txPower);
    }
    //for (uint32_t i = 0; i < ueDevs.GetN(); ++i) {
    //    ueDevs.Get(i)->GetObject<LteUeNetDevice>()->GetPhy()->SetTxPower(txPower);
    //}

    // Setup X2 interface between eNBs
    lteHelper->AddX2Interface(enbNodes);
    // Set handover algorithm
    lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
    // Tweak sensitivity
    lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(3.0)); //dB
    lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(256)));
    lteHelper->SetSchedulerType("ns3::"+scheduler+"FfMacScheduler");

    // Attach UEs to eNB 
    for (uint32_t i = 0; i < ueNodes.GetN(); i++) {
        lteHelper->Attach(ueDevs.Get(i)); // Removed this to let ns3 mtach UEs and eNBs with the best signal strength  // , enbDevs.Get(0));
        //lteHelper->Attach(ueDevs.Get(i), enbDevs.Get(i % enbDevs.GetN()));
    }

    // Somewhere before the UE loops:
    UdpEchoServerHelper remoteEchoServer(9);
    ApplicationContainer rhServerApp = remoteEchoServer.Install(remoteHostNode);
    rhServerApp.Start(Seconds(0.25));
    rhServerApp.Stop(Seconds(simTime));
    
    double sample;
    double m1 = 0.8*m;
    double m2 = 0.9*m;
    double m3 = 1.0*m;
    double m4 = 1.1*m;
    double m5 = 1.2*m;
    double m6 = 1.3*m;
    double s1 = 1.3*s;
    double s2 = 1.2*s;
    double s3 = 1.1*s;
    double s4 = 1.0*s;
    double s5 = 0.9*s;
    double s6 = 0.8*s;

    std::vector<int> uePktSize(numCars);

    // Setting the for loop myself
    uint16_t port = 9;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i) {

        if (i < uwNumCars) {
            m = m1;
            s = s1;
        } else if (i < uwNumCars + ueNumCars) {
            m = m2;
            s = s2;
        } else if (i < uwNumCars + ueNumCars + mwNumCars) {
            m = m3;
            s = s3;
        } else if (i < uwNumCars + ueNumCars + mwNumCars + meNumCars) {
            m = m4;
            s = s4;
        } else if (i < uwNumCars + ueNumCars + mwNumCars + meNumCars + lwNumCars) {
            m = m5;
            s = s5;
        } else {
            m = m6;
            s = s6;
        }

        std::normal_distribution<double> dist(m, s);
        sample = dist(rng);

        uePktSize[i] = pktsize*sample;

        // Get UE i’s IP (e.g., if you assigned IPs via an Internet stack helper):
        Ipv4Address ueIP = ueIpv4Interfaces.GetAddress(i);
        UdpEchoClientHelper echoClient(ueIP, port);

        // Sends One packet of size 1200 (bytes?) every 10ms.
        // 2400 pktsize => 1.92 MBits/s per user
        echoClient.SetAttribute("MaxPackets", UintegerValue(360000)); // 360000 //1h
        echoClient.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
        echoClient.SetAttribute("PacketSize", UintegerValue(uePktSize[i]));

        ApplicationContainer clientApps = echoClient.Install(remoteHostNode); 
        clientApps.Start(Seconds(0.5)); 
        clientApps.Stop(Seconds(simTime));
    }

    
    // Add background noise?
    //Ptr<Node> interferer = CreateObject<Node>();
    //NetDeviceContainer dev = lteHelper->InstallEnbDevice(interferer); // Configure to transmit, but not connected to UEs
    //Config::SetDefault("ns3::LteSpectrumPhy::NoiseFigure", DoubleValue(7.0));  // dB
    //OnOffHelper bgTraffic("ns3::UdpSocketFactory", InetSocketAddress(bgTarget, 8000));
    //bgTraffic.SetAttribute("DataRate", DataRateValue(DataRate("5Mbps")));
    //bgTraffic.SetAttribute("PacketSize", UintegerValue(512));

    FlowMonitorHelper flowmon;
    NodeContainer monitoredNodes;
    monitoredNodes.Add(ueNodes);
    monitoredNodes.Add(remoteHostNode);
    Ptr<FlowMonitor> monitor = flowmon.Install(monitoredNodes);
    classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());

    std::string hour_str;
    if (hour < 10) {
        hour_str = "0" + std::to_string(hour);
    } else {
        hour_str = std::to_string(hour);
    }

    std::string flowmon_filename = "flowmonitor-hour"+hour_str+"-varyingPktSize"+ std::to_string(pktsize)+"-"+datarate_p2pRH+"MBs-"+ std::to_string(RBs) +"RBs-" + std::to_string(enbPositions.size())+ "enb-"+ std::to_string((int)txPower)+"dbm.csv";

    // Open CSV file
    static std::ofstream flowCsv(flowmon_filename);
    flowCsv << "Time,FlowId,CellId,CellLoad,UE_X,UE_Y,Speed,Direction,PacketSize,DeltaTxPackets,DeltaRxPackets,AvgDelay,ThroughputKbps,IntervalLossRate,IntervalJitter,SINR,Rsrp,Gain,LOS" << std::endl;
   
    Simulator::Schedule(Seconds(interval), &LogFlowMonitorStats, monitor, &flowCsv, interval, simTime, uePktSize);
    Simulator::Stop(Seconds(simTime)); // Set length of the simulation
    auto startClock = std::chrono::high_resolution_clock::now();

    Simulator::Run();
    auto endClock = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::minutes>(endClock - startClock).count();
    std::cout << "Time taken: " << duration << " minutes" << std::endl;

    Simulator::Destroy();

    // Optionally, close Sionna on server
    sionnaHelper.ShutdownSionna();

    return 0;
}