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

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <limits>
#include <signal.h>
#include <sstream>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SionnaNs3ToLinuxTap");

namespace
{

struct CoordinatePacketStats
{
    uint32_t index;
    Vector uePosition;
    double windowStart;
    double windowEnd;
    uint64_t appTxPackets = 0;
    uint64_t appTxBytes = 0;
    uint64_t tapSideRxPackets = 0;
    uint64_t tapSideRxBytes = 0;
    double firstAppTx = -1.0;
    double lastAppTx = -1.0;
    double firstTapSideRx = -1.0;
    double lastTapSideRx = -1.0;
    double previousTapSideRx = -1.0;
    double tapSideRxInterarrivalSum = 0.0;
    uint64_t tapSideRxInterarrivalSamples = 0;
};

std::string
PositionToString(const Vector& position)
{
    std::ostringstream oss;
    oss << "(" << position.x << ", " << position.y << ", " << position.z << ")";
    return oss.str();
}

uint32_t
GetCoordinateIndex(const std::vector<CoordinatePacketStats>* stats)
{
    const double now = Simulator::Now().GetSeconds();
    for (uint32_t i = 0; i < stats->size(); ++i)
    {
        const bool inWindow = now >= stats->at(i).windowStart &&
                              (now < stats->at(i).windowEnd || i == stats->size() - 1);
        if (inWindow)
        {
            return i;
        }
    }
    return stats->empty() ? 0 : static_cast<uint32_t>(stats->size() - 1);
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

void
RecordAppTx(std::vector<CoordinatePacketStats>* stats, Ptr<const Packet> packet)
{
    const uint32_t index = GetCoordinateIndex(stats);
    CoordinatePacketStats& row = stats->at(index);
    const double now = Simulator::Now().GetSeconds();
    row.appTxPackets++;
    row.appTxBytes += packet->GetSize();
    if (row.firstAppTx < 0.0)
    {
        row.firstAppTx = now;
    }
    row.lastAppTx = now;

    LogClientTx(packet);
}

void
RecordTapSideRx(std::vector<CoordinatePacketStats>* stats, uint32_t minimumDataPacketSize, Ptr<const Packet> packet)
{
    if (packet->GetSize() < minimumDataPacketSize)
    {
        return;
    }

    const uint32_t index = GetCoordinateIndex(stats);
    CoordinatePacketStats& row = stats->at(index);
    const double now = Simulator::Now().GetSeconds();
    row.tapSideRxPackets++;
    row.tapSideRxBytes += packet->GetSize();
    if (row.firstTapSideRx < 0.0)
    {
        row.firstTapSideRx = now;
    }
    if (row.previousTapSideRx >= 0.0)
    {
        row.tapSideRxInterarrivalSum += now - row.previousTapSideRx;
        row.tapSideRxInterarrivalSamples++;
    }
    row.previousTapSideRx = now;
    row.lastTapSideRx = now;
}

Vector
ParseVector(const std::string& text)
{
    std::stringstream stream(text);
    std::string part;
    std::vector<double> values;
    while (std::getline(stream, part, ','))
    {
        values.push_back(std::stod(part));
    }
    NS_ABORT_MSG_IF(values.size() != 3, "Expected position as x,y,z but got: " << text);
    return Vector(values[0], values[1], values[2]);
}

std::vector<Vector>
ParsePositionList(const std::string& text)
{
    std::stringstream stream(text);
    std::string item;
    std::vector<Vector> positions;
    while (std::getline(stream, item, ';'))
    {
        if (!item.empty())
        {
            positions.push_back(ParseVector(item));
        }
    }
    NS_ABORT_MSG_IF(positions.empty(), "uePositions must contain at least one x,y,z position");
    return positions;
}

std::vector<CoordinatePacketStats>
CreateCoordinateStats(const std::vector<Vector>& uePositionList,
                      bool enableUePositionSequence,
                      double uePositionInterval,
                      double simTime)
{
    const uint32_t count = enableUePositionSequence ? uePositionList.size() : 1;
    std::vector<CoordinatePacketStats> stats;
    stats.reserve(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        CoordinatePacketStats row;
        row.index = i;
        row.uePosition = uePositionList[i];
        row.windowStart = enableUePositionSequence ? i * uePositionInterval : 0.0;
        row.windowEnd = enableUePositionSequence ? std::min((i + 1) * uePositionInterval, simTime) : simTime;
        if (i == count - 1)
        {
            row.windowEnd = simTime;
        }
        stats.push_back(row);
    }
    return stats;
}

void
WritePacketSummaryCsv(const std::string& filename,
                      const std::vector<CoordinatePacketStats>& stats,
                      const Vector& enbPosition)
{
    std::ofstream csv(filename);
    NS_ABORT_MSG_IF(!csv.is_open(), "Could not open packet summary CSV: " << filename);

    csv << "index,window_start_s,window_end_s,enb_x,enb_y,enb_z,ue_x,ue_y,ue_z,"
        << "tx_rx_distance_3d_m,app_tx_packets,app_tx_bytes,tap_side_rx_packets,tap_side_rx_bytes,"
        << "delivery_ratio,first_app_tx_s,last_app_tx_s,first_tap_side_rx_s,last_tap_side_rx_s,"
        << "mean_tap_side_rx_interarrival_s\n";

    csv << std::fixed << std::setprecision(9);
    for (const auto& row : stats)
    {
        const double dx = row.uePosition.x - enbPosition.x;
        const double dy = row.uePosition.y - enbPosition.y;
        const double dz = row.uePosition.z - enbPosition.z;
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double deliveryRatio =
            row.appTxPackets == 0 ? 0.0 : static_cast<double>(row.tapSideRxPackets) / row.appTxPackets;
        const double meanInterarrival =
            row.tapSideRxInterarrivalSamples == 0
                ? std::numeric_limits<double>::quiet_NaN()
                : row.tapSideRxInterarrivalSum / row.tapSideRxInterarrivalSamples;

        csv << row.index << ","
            << row.windowStart << ","
            << row.windowEnd << ","
            << enbPosition.x << ","
            << enbPosition.y << ","
            << enbPosition.z << ","
            << row.uePosition.x << ","
            << row.uePosition.y << ","
            << row.uePosition.z << ","
            << distance << ","
            << row.appTxPackets << ","
            << row.appTxBytes << ","
            << row.tapSideRxPackets << ","
            << row.tapSideRxBytes << ","
            << deliveryRatio << ","
            << row.firstAppTx << ","
            << row.lastAppTx << ","
            << row.firstTapSideRx << ","
            << row.lastTapSideRx << ","
            << meanInterarrival << "\n";
    }
}

void
SetNodePosition(Ptr<Node> node, const Vector& position, const std::string& label)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    NS_ABORT_MSG_IF(!mobility, "Node has no MobilityModel");
    mobility->SetPosition(position);
    NS_LOG_UNCOND("[mobility] t=" << Simulator::Now().As(Time::S)
                                  << " " << label << " position="
                                  << PositionToString(position));
}

void
StartSystemTapPcap(pid_t* tcpdumpPid,
                   const std::string& tapName,
                   const std::string& tapPcapFile,
                   uint16_t port,
                   bool useSudo)
{
    (void)port;
    if (*tcpdumpPid > 0)
    {
        return;
    }

    pid_t pid = fork();
    if (pid == 0)
    {
        setpgid(0, 0);
        if (useSudo)
        {
            execlp("sudo",
                   "sudo",
                   "-n",
                   "tcpdump",
                   "-i",
                   tapName.c_str(),
                   "-n",
                   "-w",
                   tapPcapFile.c_str(),
                   static_cast<char*>(nullptr));
        }
        else
        {
            execlp("tcpdump",
                   "tcpdump",
                   "-i",
                   tapName.c_str(),
                   "-n",
                   "-w",
                   tapPcapFile.c_str(),
                   static_cast<char*>(nullptr));
        }
        _exit(127);
    }

    if (pid > 0)
    {
        *tcpdumpPid = pid;
        NS_LOG_UNCOND("  started Linux TAP tcpdump pid=" << pid << " file=" << tapPcapFile);
    }
    else
    {
        NS_LOG_UNCOND("  failed to start Linux TAP tcpdump; run it manually if needed");
    }
}

void
PauseForManualCapture(double seconds, const std::string& tapName, uint16_t port)
{
    if (seconds <= 0.0)
    {
        return;
    }

    NS_LOG_UNCOND("  manual capture pause: TAP should be available as " << tapName);
    NS_LOG_UNCOND("  run now: sudo tcpdump -i " << tapName << " -n -vv udp port " << port
                                                << " -w " << tapName << "-udp" << port << ".pcap");
    NS_LOG_UNCOND("  waiting " << seconds << " wall-clock seconds before UDP traffic continues");
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

void
StopSystemTapPcap(pid_t* tcpdumpPid)
{
    if (*tcpdumpPid <= 0)
    {
        return;
    }

    int status = 0;
    kill(-(*tcpdumpPid), SIGINT);
    for (uint32_t i = 0; i < 20; ++i)
    {
        const pid_t done = waitpid(*tcpdumpPid, &status, WNOHANG);
        if (done == *tcpdumpPid)
        {
            NS_LOG_UNCOND("  stopped Linux TAP tcpdump pid=" << *tcpdumpPid);
            *tcpdumpPid = -1;
            return;
        }
        usleep(100000);
    }

    NS_LOG_UNCOND("  tcpdump did not exit after SIGINT; terminating pid=" << *tcpdumpPid);
    kill(-(*tcpdumpPid), SIGTERM);
    for (uint32_t i = 0; i < 10; ++i)
    {
        const pid_t done = waitpid(*tcpdumpPid, &status, WNOHANG);
        if (done == *tcpdumpPid)
        {
            NS_LOG_UNCOND("  stopped Linux TAP tcpdump pid=" << *tcpdumpPid);
            *tcpdumpPid = -1;
            return;
        }
        usleep(100000);
    }

    NS_LOG_UNCOND("  tcpdump did not exit after SIGTERM; killing pid=" << *tcpdumpPid);
    kill(-(*tcpdumpPid), SIGKILL);
    waitpid(*tcpdumpPid, &status, 0);
    NS_LOG_UNCOND("  killed Linux TAP tcpdump pid=" << *tcpdumpPid);
    *tcpdumpPid = -1;
}

} // namespace

int
main(int argc, char* argv[])
{
    bool enableTap = true;
    bool enableSionna = true;
    bool enablePcap = true;
    bool enableFlowMonitor = false;
    bool enablePacketSummary = true;
    bool enableSystemTapPcap = true;
    bool systemTapPcapUseSudo = true;
    bool enableStaticNeighborCache = true;
    bool localMachine = true;
    bool sionnaVerbose = false;
    bool shutdownSionna = true;
    bool enableUePositionSequence = true;
    double simTime = 100.0;
    double uePositionInterval = 20.0;
    double appStartTime = 3.0;
    double systemTapPcapStartTime = 1.0;
    double manualCapturePauseSeconds = 0.0;
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
    std::string pcapPrefix = "sionna-ns3-to-linux-tap-csma";
    std::string packetSummaryFile = "sionna-ns3-to-linux-tap-packet-summary.csv";
    std::string systemTapPcapFile = "sionna-tap0-udp9000.pcap";
    std::string enbPosition = "49,-17,12";
    std::string uePositions = "32,-9,1.5;-72,-31,1.5;-120,90,1.5;-208,-101,1.5;205,177,1.5";
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
    cmd.AddValue("appStartTime", "Application start time in seconds", appStartTime);
    cmd.AddValue("enbPosition", "LTE eNB position as x,y,z", enbPosition);
    cmd.AddValue("uePositions", "Semicolon-separated LTE UE positions as x,y,z;x,y,z", uePositions);
    cmd.AddValue("enableUePositionSequence", "Move the UE through uePositions during the run", enableUePositionSequence);
    cmd.AddValue("uePositionInterval", "Seconds between UE position changes", uePositionInterval);
    cmd.AddValue("maxPackets", "UDP client max packets; 0 derives from simTime/interval", maxPackets);
    cmd.AddValue("enablePcap", "Enable LTE and CSMA tracing", enablePcap);
    cmd.AddValue("pcapPrefix", "PCAP filename prefix for the CSMA/TAP-facing link", pcapPrefix);
    cmd.AddValue("enablePacketSummary", "Write per-coordinate packet summary CSV", enablePacketSummary);
    cmd.AddValue("packetSummaryFile", "Per-coordinate packet summary CSV output path", packetSummaryFile);
    cmd.AddValue("enableSystemTapPcap", "Start tcpdump on the Linux TAP interface during the run", enableSystemTapPcap);
    cmd.AddValue("systemTapPcapFile", "Linux TAP tcpdump PCAP output path", systemTapPcapFile);
    cmd.AddValue("systemTapPcapUseSudo", "Run tcpdump through sudo", systemTapPcapUseSudo);
    cmd.AddValue("systemTapPcapStartTime", "Simulation time at which to start Linux TAP tcpdump", systemTapPcapStartTime);
    cmd.AddValue("manualCapturePauseSeconds",
                 "Wall-clock pause after TAP setup so tcpdump can be started manually",
                 manualCapturePauseSeconds);
    cmd.AddValue("enableStaticNeighborCache",
                 "Pre-populate ARP entries on the PGW-to-TAP CSMA link",
                 enableStaticNeighborCache);
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
    NS_ABORT_MSG_IF(appStartTime < 0.0 || appStartTime >= simTime,
                    "appStartTime must be non-negative and less than simTime");
    NS_ABORT_MSG_IF(systemTapPcapStartTime < 0.0 || systemTapPcapStartTime >= simTime,
                    "systemTapPcapStartTime must be non-negative and less than simTime");
    NS_ABORT_MSG_IF(uePositionInterval <= 0.0, "uePositionInterval must be positive");

    const Vector enbVector = ParseVector(enbPosition);
    const std::vector<Vector> uePositionList = ParsePositionList(uePositions);
    std::vector<CoordinatePacketStats> packetStats =
        CreateCoordinateStats(uePositionList, enableUePositionSequence, uePositionInterval, simTime);

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
    positions->Add(uePositionList.front());
    positions->Add(enbVector);
    mobility.SetPositionAllocator(positions);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(ueNodes);
    mobility.Install(enbNodes);

    Simulator::Schedule(Seconds(0.0), &SetNodePosition, sourceNode, uePositionList.front(), "UE car_1");
    Simulator::Schedule(Seconds(0.0), &SetNodePosition, enbNode, enbVector, "eNB car_2");
    if (enableUePositionSequence)
    {
        for (uint32_t i = 1; i < uePositionList.size(); ++i)
        {
            Simulator::Schedule(Seconds(i * uePositionInterval),
                                &SetNodePosition,
                                sourceNode,
                                uePositionList[i],
                                "UE car_1");
        }
    }

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
    if (enableStaticNeighborCache)
    {
        NeighborCacheHelper neighborCache;
        neighborCache.PopulateNeighborCache(wiredIfaces);
    }

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
        const double activeSeconds = simTime - appStartTime;
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
            udpClient->TraceConnectWithoutContext("Tx", MakeBoundCallback(&RecordAppTx, &packetStats));
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
            onOff->TraceConnectWithoutContext("Tx", MakeBoundCallback(&RecordAppTx, &packetStats));
        }
    }
    clientApps.Start(Seconds(appStartTime));
    clientApps.Stop(Seconds(simTime));

    wiredDevices.Get(1)->TraceConnectWithoutContext(
        "MacRx",
        MakeBoundCallback(&RecordTapSideRx, &packetStats, packetSize));

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
        csma.EnablePcap(pcapPrefix, wiredDevices, false);
    }

    pid_t systemTapPcapPid = -1;
    if (enableTap && enableSystemTapPcap)
    {
        Simulator::Schedule(Seconds(systemTapPcapStartTime),
                            &StartSystemTapPcap,
                            &systemTapPcapPid,
                            tapName,
                            systemTapPcapFile,
                            port,
                            systemTapPcapUseSudo);
    }
    if (enableTap && manualCapturePauseSeconds > 0.0)
    {
        const double pauseTime =
            std::max(0.1, std::min(appStartTime - 0.1, systemTapPcapStartTime + 0.5));
        Simulator::Schedule(Seconds(pauseTime),
                            &PauseForManualCapture,
                            manualCapturePauseSeconds,
                            tapName,
                            port);
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
                                       << " dataRate=" << dataRate
                                       << " appStartTime=" << appStartTime << "s");
    NS_LOG_UNCOND("  TapBridge enabled=" << (enableTap ? "true" : "false")
                                         << " mode=" << tapMode
                                         << " tapName=" << tapName);
    NS_LOG_UNCOND("  static PGW/TAP ARP cache=" << (enableStaticNeighborCache ? "true" : "false"));
    NS_LOG_UNCOND("  UE position sequence enabled=" << (enableUePositionSequence ? "true" : "false")
                                                    << " interval=" << uePositionInterval
                                                    << "s count=" << uePositionList.size());
    NS_LOG_UNCOND("  PCAP enabled=" << (enablePcap ? "true" : "false")
                                    << " prefix=" << pcapPrefix);
    NS_LOG_UNCOND("  packet summary enabled=" << (enablePacketSummary ? "true" : "false")
                                              << " file=" << packetSummaryFile);
    NS_LOG_UNCOND("  Linux TAP tcpdump enabled="
                  << (enableTap && enableSystemTapPcap ? "true" : "false")
                  << " file=" << systemTapPcapFile
                  << " startTime=" << systemTapPcapStartTime << "s");
    NS_LOG_UNCOND("  manual capture pause=" << manualCapturePauseSeconds << "s");
    NS_LOG_UNCOND("  expected tcpdump: sudo tcpdump -i " << tapName << " -n -vv udp port " << port);
    NS_LOG_UNCOND("  expected listener: nc -ul " << port);

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    if (enableFlowMonitor && monitor)
    {
        monitor->SerializeToXmlFile(flowMonitorFile, true, true);
        NS_LOG_UNCOND("  wrote FlowMonitor XML: " << flowMonitorFile);
    }
    if (enablePcap)
    {
        NS_LOG_UNCOND("  wrote CSMA/TAP-side PCAP files with prefix: " << pcapPrefix);
    }
    if (enablePacketSummary)
    {
        WritePacketSummaryCsv(packetSummaryFile, packetStats, enbVector);
        NS_LOG_UNCOND("  wrote packet summary CSV: " << packetSummaryFile);
    }
    if (enableTap && enableSystemTapPcap)
    {
        StopSystemTapPcap(&systemTapPcapPid);
        NS_LOG_UNCOND("  wrote Linux TAP PCAP: " << systemTapPcapFile);
    }

    if (enableSionna && shutdownSionna)
    {
        NS_LOG_UNCOND("  asking Sionna server to shut down");
        SionnaHelper::GetInstance().ShutdownSionna();
    }

    Simulator::Destroy();

    return 0;
}
