/*
 * Minimal proof-of-chain:
 *
 *   ns-3 UDP source UE -> Sionna-managed LTE hop -> eNB/EPC PGW
 *                       -> CSMA link -> ghost node -> Linux TAP
 *
 * The TapBridge is deliberately attached to a separate CSMA device on the
 * ghost node, not directly to an LTE wireless device.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/sionna-helper.h"
#include "ns3/tap-bridge-module.h"

#include <sstream>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SionnaNs3ToLinuxTap");

namespace
{

std::string
PositionToString(const Vector& position)
{
    std::ostringstream oss;
    oss << "(" << position.x << ", " << position.y << ", " << position.z << ")";
    return oss.str();
}

void
LogClientTx(Ptr<const Packet> packet)
{
    NS_LOG_UNCOND("[udp-app-tx] t=" << Simulator::Now().As(Time::S)
                                    << " size=" << packet->GetSize());
}

void
LogServerRx(Ptr<const Packet> packet)
{
    NS_LOG_UNCOND("[ns3-ghost-rx] t=" << Simulator::Now().As(Time::S)
                                      << " size=" << packet->GetSize()
                                      << " reached the ghost node UDP server");
}

} // namespace

int
main(int argc, char* argv[])
{
    bool enableTap = true;
    bool enableSionna = true;
    bool enablePcap = false;
    bool enableFlowMonitor = false;
    bool localMachine = true;
    bool sionnaVerbose = false;
    bool shutdownSionna = false;
    double distance = 15.0;
    double simTime = 60.0;
    uint32_t packetSize = 512;
    uint32_t maxPackets = 0;
    uint16_t port = 9000;
    std::string interval = "100ms";
    std::string dataRate = "40960bps";
    std::string trafficApp = "UdpClient";
    std::string sionnaServerIp = "127.0.0.1";
    std::string tapMode = "ConfigureLocal";
    std::string tapName = "sionna-tap0";
    std::string flowMonitorFile = "sionna-ns3-to-linux-tap.flowmon.xml";
    double txPower = 23.0;
    uint16_t earfcn = 100;

    CommandLine cmd(__FILE__);
    cmd.AddValue("enableTap", "Enable TapBridge egress to Linux", enableTap);
    cmd.AddValue("sionna", "Use Sionna RT for the wireless propagation models", enableSionna);
    cmd.AddValue("localMachine", "Set true when Sionna runs on this machine", localMachine);
    cmd.AddValue("sionnaServerIp", "Sionna server IP address for remote mode", sionnaServerIp);
    cmd.AddValue("sionna-local-machine", "Alias for localMachine", localMachine);
    cmd.AddValue("sionna-server-ip", "Alias for sionnaServerIp", sionnaServerIp);
    cmd.AddValue("sionnaVerbose", "Enable verbose Sionna logging", sionnaVerbose);
    cmd.AddValue("shutdownSionna", "Ask the Sionna server to shut down when the simulation ends", shutdownSionna);
    cmd.AddValue("tapMode", "TapBridge mode: ConfigureLocal or UseLocal", tapMode);
    cmd.AddValue("tapName", "Linux TAP interface name", tapName);
    cmd.AddValue("packetSize", "UDP payload size in bytes", packetSize);
    cmd.AddValue("interval", "UDP client packet interval, e.g. 100ms", interval);
    cmd.AddValue("dataRate", "OnOffApplication data rate, e.g. 40960bps", dataRate);
    cmd.AddValue("trafficApp", "Traffic generator: UdpClient or OnOff", trafficApp);
    cmd.AddValue("port", "Destination UDP port", port);
    cmd.AddValue("simTime", "Simulation duration in seconds", simTime);
    cmd.AddValue("distance", "Distance in meters between the LTE UE and eNB", distance);
    cmd.AddValue("maxPackets", "UDP client max packets; 0 derives from simTime/interval", maxPackets);
    cmd.AddValue("enablePcap", "Enable LTE and CSMA tracing", enablePcap);
    cmd.AddValue("enableFlowMonitor", "Write FlowMonitor XML output", enableFlowMonitor);
    cmd.AddValue("flowMonitorFile", "FlowMonitor XML output path", flowMonitorFile);
    cmd.AddValue("txPower", "LTE UE/eNB TX power in dBm", txPower);
    cmd.AddValue("earfcn", "LTE DL EARFCN; default 100 is near 2.1 GHz", earfcn);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(tapMode != "ConfigureLocal" && tapMode != "UseLocal",
                    "tapMode must be ConfigureLocal or UseLocal");
    NS_ABORT_MSG_IF(trafficApp != "UdpClient" && trafficApp != "OnOff",
                    "trafficApp must be UdpClient or OnOff");
    NS_ABORT_MSG_IF(simTime <= 1.0, "simTime must be greater than one second");

    // Real-time scheduling and checksums are needed because this simulation can
    // exchange real Ethernet frames with the Linux network stack through TAP.
    GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

    if (enableSionna)
    {
        SionnaHelper& sionnaHelper = SionnaHelper::GetInstance();
        sionnaHelper.SetSionna(true);
        sionnaHelper.SetServerIp(sionnaServerIp);
        sionnaHelper.SetLocalMachine(localMachine);
        sionnaHelper.SetVerbose(sionnaVerbose);
    }

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    ueNodes.Create(1);
    enbNodes.Create(1);
    Ptr<Node> sourceNode = ueNodes.Get(0);
    Ptr<Node> enbNode = enbNodes.Get(0);

    NodeContainer ghostNode;
    ghostNode.Create(1);

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator>();
    positions->Add(Vector(0.0, 0.0, 1.5));
    positions->Add(Vector(distance, 0.0, 1.5));
    mobility.SetPositionAllocator(positions);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(ueNodes);
    mobility.Install(enbNodes);

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(earfcn));
    lteHelper->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(earfcn + 18000));

    Ptr<Node> pgwNode = epcHelper->GetPgwNode();

    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);
    enbLteDevs.Get(0)->GetObject<LteEnbNetDevice>()->GetPhy()->SetTxPower(txPower);
    ueLteDevs.Get(0)->GetObject<LteUeNetDevice>()->GetPhy()->SetTxPower(txPower);

    InternetStackHelper internet;
    internet.Install(ueNodes);
    internet.Install(ghostNode);

    Ipv4InterfaceContainer ueIfaces = epcHelper->AssignUeIpv4Address(ueLteDevs);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(sourceNode->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

    lteHelper->Attach(ueLteDevs.Get(0), enbLteDevs.Get(0));

    NodeContainer wiredNodes(pgwNode, ghostNode.Get(0));
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", DataRateValue(DataRate("100Mbps")));
    csma.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
    NetDeviceContainer wiredDevices = csma.Install(wiredNodes);

    Ipv4AddressHelper wiredIpv4;
    wiredIpv4.SetBase("10.10.2.0", "255.255.255.0");
    Ipv4InterfaceContainer wiredIfaces = wiredIpv4.Assign(wiredDevices);

    const Ipv4Address sourceIp = ueIfaces.GetAddress(0);
    const Ipv4Address gatewayWiredIp = wiredIfaces.GetAddress(0);
    const Ipv4Address destinationIp = wiredIfaces.GetAddress(1);

    if (enableTap)
    {
        TapBridgeHelper tapBridge;
        tapBridge.SetAttribute("Mode", StringValue(tapMode));
        tapBridge.SetAttribute("DeviceName", StringValue(tapName));
        if (tapMode == "ConfigureLocal")
        {
            tapBridge.SetAttribute("IpAddress", Ipv4AddressValue(destinationIp));
            tapBridge.SetAttribute("Netmask", Ipv4MaskValue("255.255.255.0"));
            tapBridge.SetAttribute("Gateway", Ipv4AddressValue(gatewayWiredIp));
        }
        tapBridge.Install(ghostNode.Get(0), wiredDevices.Get(1));
    }

    const Time packetInterval = Time(interval);
    if (maxPackets == 0)
    {
        const double activeSeconds = simTime - 1.0;
        maxPackets = static_cast<uint32_t>(activeSeconds / packetInterval.GetSeconds()) + 1;
    }

    if (!enableTap)
    {
        UdpServerHelper server(port);
        ApplicationContainer serverApps = server.Install(ghostNode.Get(0));
        serverApps.Start(Seconds(0.5));
        serverApps.Stop(Seconds(simTime));

        Ptr<UdpServer> udpServer = DynamicCast<UdpServer>(serverApps.Get(0));
        if (udpServer)
        {
            udpServer->TraceConnectWithoutContext("Rx", MakeCallback(&LogServerRx));
        }
    }

    ApplicationContainer clientApps;
    if (trafficApp == "UdpClient")
    {
        UdpClientHelper client(destinationIp, port);
        client.SetAttribute("MaxPackets", UintegerValue(maxPackets));
        client.SetAttribute("Interval", TimeValue(packetInterval));
        client.SetAttribute("PacketSize", UintegerValue(packetSize));

        clientApps = client.Install(sourceNode);
        Ptr<UdpClient> udpClient = DynamicCast<UdpClient>(clientApps.Get(0));
        if (udpClient)
        {
            udpClient->TraceConnectWithoutContext("Tx", MakeCallback(&LogClientTx));
        }
    }
    else
    {
        OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(destinationIp, port));
        onoff.SetAttribute("PacketSize", UintegerValue(packetSize));
        onoff.SetAttribute("DataRate", DataRateValue(DataRate(dataRate)));
        onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

        clientApps = onoff.Install(sourceNode);
        Ptr<OnOffApplication> onOff = DynamicCast<OnOffApplication>(clientApps.Get(0));
        if (onOff)
        {
            onOff->TraceConnectWithoutContext("Tx", MakeCallback(&LogClientTx));
        }
    }
    clientApps.Start(Seconds(1.0));
    clientApps.Stop(Seconds(simTime));

    Ptr<FlowMonitor> monitor;
    FlowMonitorHelper flowHelper;
    if (enableFlowMonitor)
    {
        monitor = flowHelper.InstallAll();
    }

    if (enablePcap)
    {
        lteHelper->EnablePhyTraces();
        lteHelper->EnableMacTraces();
        lteHelper->EnableRlcTraces();
        csma.EnablePcap("sionna-ns3-to-linux-tap-csma", wiredDevices, false);
    }

    NS_LOG_UNCOND("Sionna LTE ns-3 to Linux TAP proof-of-chain");
    NS_LOG_UNCOND("  source nodeId=" << sourceNode->GetId() << " Sionna object=car_1"
                                      << " position="
                                      << PositionToString(sourceNode->GetObject<MobilityModel>()->GetPosition()));
    NS_LOG_UNCOND("  eNB nodeId=" << enbNode->GetId() << " Sionna object=car_2"
                                  << " position="
                                  << PositionToString(enbNode->GetObject<MobilityModel>()->GetPosition()));
    NS_LOG_UNCOND("  UE IP=" << sourceIp << " EPC default gateway="
                             << epcHelper->GetUeDefaultGatewayAddress());
    NS_LOG_UNCOND("  PGW wired IP=" << gatewayWiredIp << " destination IP=" << destinationIp);
    NS_LOG_UNCOND("  UDP destination=" << destinationIp << ":" << port
                                       << " app=" << trafficApp
                                       << " packetSize=" << packetSize
                                       << " interval=" << interval
                                       << " dataRate=" << dataRate);
    NS_LOG_UNCOND("  TapBridge enabled=" << (enableTap ? "true" : "false")
                                         << " mode=" << tapMode
                                         << " tapName=" << tapName);
    NS_LOG_UNCOND("  expected tcpdump: sudo tcpdump -i " << tapName << " -n -vv udp port " << port);
    NS_LOG_UNCOND("  expected listener: nc -ul " << port);

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    if (enableFlowMonitor && monitor)
    {
        monitor->SerializeToXmlFile(flowMonitorFile, true, true);
        NS_LOG_UNCOND("  wrote FlowMonitor XML: " << flowMonitorFile);
    }

    Simulator::Destroy();

    if (enableSionna && shutdownSionna)
    {
        SionnaHelper::GetInstance().ShutdownSionna();
    }

    return 0;
}
