/*
 * Minimal proof-of-chain:
 *
 *   ns-3 TCP/UDP source UE -> Sionna-managed LTE hop -> eNB/EPC PGW
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
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <limits>
#include <signal.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstring>
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

std::string
TrimCommandLineQuotes(std::string text)
{
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front())))
    {
        text.erase(text.begin());
    }
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back())))
    {
        text.pop_back();
    }
    if (text.size() >= 2 &&
        ((text.front() == '\'' && text.back() == '\'') ||
         (text.front() == '"' && text.back() == '"')))
    {
        return text.substr(1, text.size() - 2);
    }
    if (!text.empty() && (text.front() == '\'' || text.front() == '"'))
    {
        text.erase(text.begin());
    }
    if (!text.empty() && (text.back() == '\'' || text.back() == '"'))
    {
        text.pop_back();
    }
    return text;
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
    NS_LOG_UNCOND("[app-tx] t=" << Simulator::Now().As(Time::S)
                                    << " size=" << packet->GetSize());
}

void
LogSinkRx(Ptr<const Packet> packet, const Address& from)
{
    (void)from;
    NS_LOG_UNCOND("[ns3-ghost-rx] t=" << Simulator::Now().As(Time::S)
                                      << " size=" << packet->GetSize()
                                      << " reached the ghost node packet sink");
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

void
TcpCwndTracer(std::ofstream* stream, uint32_t oldValue, uint32_t newValue)
{
    if (stream && stream->is_open())
    {
        *stream << std::fixed << std::setprecision(9)
                << Simulator::Now().GetSeconds() << "," << oldValue << "," << newValue << "\n";
    }
}

void
TcpRttTracer(std::ofstream* stream, Time oldValue, Time newValue)
{
    if (stream && stream->is_open())
    {
        *stream << std::fixed << std::setprecision(9)
                << Simulator::Now().GetSeconds() << ","
                << oldValue.GetMilliSeconds() << ","
                << newValue.GetMilliSeconds() << "\n";
    }
}

void
TcpCongStateTracer(std::ofstream* stream,
                   TcpSocketState::TcpCongState_t oldValue,
                   TcpSocketState::TcpCongState_t newValue)
{
    if (stream && stream->is_open())
    {
        *stream << std::fixed << std::setprecision(9)
                << Simulator::Now().GetSeconds() << ","
                << static_cast<uint32_t>(oldValue) << ","
                << static_cast<uint32_t>(newValue) << "\n";
    }
}

void
ConnectTcpTraces(uint32_t nodeId,
                 std::ofstream* cwndStream,
                 std::ofstream* rttStream,
                 std::ofstream* retransmissionStream)
{
    const std::string socketPath =
        "/NodeList/" + std::to_string(nodeId) + "/$ns3::TcpL4Protocol/SocketList/0/";
    Config::ConnectWithoutContext(socketPath + "CongestionWindow",
                                  MakeBoundCallback(&TcpCwndTracer, cwndStream));
    Config::ConnectWithoutContext(socketPath + "RTT",
                                  MakeBoundCallback(&TcpRttTracer, rttStream));
    Config::ConnectWithoutContext(socketPath + "CongState",
                                  MakeBoundCallback(&TcpCongStateTracer, retransmissionStream));
}

Vector
ParseVector(const std::string& text)
{
    std::stringstream stream(TrimCommandLineQuotes(text));
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
    std::stringstream stream(TrimCommandLineQuotes(text));
    std::string item;
    std::vector<Vector> positions;
    while (std::getline(stream, item, ';'))
    {
        if (!item.empty())
        {
            positions.push_back(ParseVector(TrimCommandLineQuotes(item)));
        }
    }
    NS_ABORT_MSG_IF(positions.empty(), "uePositions must contain at least one x,y,z position");
    return positions;
}

std::vector<Vector>
ParsePositionFile(const std::string& filename)
{
    std::ifstream file(filename);
    NS_ABORT_MSG_IF(!file.is_open(), "Could not open uePositionsFile: " << filename);
    std::string line;
    std::vector<Vector> positions;
    while (std::getline(file, line))
    {
        line = TrimCommandLineQuotes(line);
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        positions.push_back(ParseVector(line));
    }
    NS_ABORT_MSG_IF(positions.empty(), "uePositionsFile must contain at least one x,y,z position");
    return positions;
}

std::vector<CoordinatePacketStats>
CreateCoordinateStats(const std::vector<Vector>& uePositionList,
                      bool enableUePositionSequence,
                      double uePositionInterval,
                      double uePositionStartTime,
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
        if (enableUePositionSequence)
        {
            row.windowStart = (i == 0) ? 0.0 : uePositionStartTime + i * uePositionInterval;
            row.windowEnd = std::min(uePositionStartTime + (i + 1) * uePositionInterval, simTime);
        }
        else
        {
            row.windowStart = 0.0;
            row.windowEnd = simTime;
        }
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
RequestSionnaRadioMapExport(const std::string& serverIp,
                            uint16_t serverPort,
                            double timeoutSeconds,
                            uint32_t coordinateIndex)
{
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        NS_LOG_UNCOND("[sionna-export] coordinate=" << coordinateIndex
                                                    << " skipped: could not create UDP socket");
        return;
    }

    timeval timeout;
    timeout.tv_sec = static_cast<long>(timeoutSeconds);
    timeout.tv_usec = static_cast<long>((timeoutSeconds - timeout.tv_sec) * 1000000.0);
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(serverPort);
    if (inet_pton(AF_INET, serverIp.c_str(), &address.sin_addr) != 1)
    {
        NS_LOG_UNCOND("[sionna-export] coordinate=" << coordinateIndex
                                                    << " skipped: invalid server IP " << serverIp);
        close(fd);
        return;
    }

    const char* message = "EXPORT_RADIO_MAP";
    const ssize_t sent = sendto(fd,
                                message,
                                std::strlen(message),
                                0,
                                reinterpret_cast<sockaddr*>(&address),
                                sizeof(address));
    if (sent < 0)
    {
        NS_LOG_UNCOND("[sionna-export] coordinate=" << coordinateIndex
                                                    << " skipped: sendto failed");
        close(fd);
        return;
    }

    char buffer[256] = {0};
    const ssize_t received = recvfrom(fd, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
    if (received > 0)
    {
        NS_LOG_UNCOND("[sionna-export] coordinate=" << coordinateIndex
                                                    << " response=" << buffer);
    }
    else
    {
        NS_LOG_UNCOND("[sionna-export] coordinate=" << coordinateIndex
                                                    << " timed out waiting for radio-map export");
    }
    close(fd);
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
PauseForManualCapture(double seconds, const std::string& tapName, uint16_t port, std::string transport)
{
    if (seconds <= 0.0)
    {
        return;
    }

    NS_LOG_UNCOND("  manual capture pause: TAP should be available as " << tapName);
    NS_LOG_UNCOND("  run now: sudo tcpdump -i " << tapName << " -n -vv " << transport
                                                << " port " << port
                                                << " -w " << tapName << "-" << transport
                                                << port << ".pcap");
    NS_LOG_UNCOND("  waiting " << seconds << " wall-clock seconds before application traffic continues");
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
    bool enableSionnaExportSequence = false;
    bool enableTcpTraces = true;
    double simTime = 100.0;
    double uePositionInterval = 20.0;
    double uePositionStartTime = 0.0;
    double appStart = 3.0;
    double appStop = 0.0;
    double sionnaExportDelay = 0.05;
    double sionnaExportTimeout = 600.0;
    double systemTapPcapStartTime = 1.0;
    double manualCapturePauseSeconds = 0.0;
    uint32_t packetSize = 512;
    uint64_t maxBytes = 0;
    uint32_t maxPackets = 0;
    uint16_t port = 9000;
    uint16_t sionnaExportPort = 8103;
    std::string interval = "100ms";
    std::string dataRate = "40960bps";
    std::string transport = "tcp";
    std::string tcpVariant = "TcpNewReno";
    std::string trafficApp = "BulkSend";
    std::string sionnaServerIp = "127.0.0.1";
    std::string tapMode = "ConfigureLocal";
    std::string tapName = "tap0";
    std::string tapMac = "02:00:00:00:02:02";
    std::string flowMonitorFile = "sionna-ns3-to-linux-tap.flowmon.xml";
    std::string pcapPrefix = "sionna-ns3-to-linux-tap-csma";
    std::string packetSummaryFile = "sionna-ns3-to-linux-tap-packet-summary.csv";
    std::string systemTapPcapFile = "sionna-tap0-udp9000.pcap";
    std::string tcpCwndFile = "tcp_cwnd.csv";
    std::string tcpRttFile = "tcp_rtt.csv";
    std::string tcpRetransmissionsFile = "tcp_retransmissions.csv";
    std::string topologyMetadataFile = "ns3_to_tap_topology.json";
    std::string enbPosition = "49,-17,12";
    std::string uePositions = "32,-9,1.5;-72,-31,1.5;-120,90,1.5;-208,-101,1.5;205,177,1.5";
    std::string uePositionsFile = "";
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
    cmd.AddValue("tapMac", "MAC address used for both Linux TAP and ns-3 output device in UseLocal mode", tapMac);
    cmd.AddValue("transport", "Transport protocol: tcp or udp", transport);
    cmd.AddValue("tcpVariant", "TCP variant TypeId short name, e.g. TcpNewReno or TcpCubic", tcpVariant);
    cmd.AddValue("packetSize", "Application packet/send size in bytes", packetSize);
    cmd.AddValue("maxBytes", "TCP BulkSend MaxBytes; 0 means unlimited", maxBytes);
    cmd.AddValue("interval", "UDP client packet interval, e.g. 100ms", interval);
    cmd.AddValue("dataRate", "OnOffApplication data rate, e.g. 40960bps", dataRate);
    cmd.AddValue("trafficApp", "Traffic generator: BulkSend, OnOff, or UdpClient", trafficApp);
    cmd.AddValue("port", "Destination TCP/UDP port", port);
    cmd.AddValue("simTime", "Simulation duration in seconds", simTime);
    cmd.AddValue("appStart", "Application start time in seconds", appStart);
    cmd.AddValue("appStartTime", "Backward-compatible alias for appStart", appStart);
    cmd.AddValue("appStop", "Application stop time in seconds; 0 means simTime", appStop);
    cmd.AddValue("enbPosition", "LTE eNB position as x,y,z", enbPosition);
    cmd.AddValue("uePositions", "Semicolon-separated LTE UE positions as x,y,z;x,y,z", uePositions);
    cmd.AddValue("uePositionsFile", "Text file containing one LTE UE position x,y,z per line", uePositionsFile);
    cmd.AddValue("enableUePositionSequence", "Move the UE through uePositions during the run", enableUePositionSequence);
    cmd.AddValue("uePositionInterval", "Seconds between UE position changes", uePositionInterval);
    cmd.AddValue("uePositionStartTime", "Simulation time for the first scheduled UE movement after the initial position", uePositionStartTime);
    cmd.AddValue("enableSionnaExportSequence", "Request Sionna radio-map export after each scheduled UE position", enableSionnaExportSequence);
    cmd.AddValue("sionnaExportPort", "UDP port of the Sionna server used for on-request radio-map export", sionnaExportPort);
    cmd.AddValue("sionnaExportDelay", "Delay after each UE position update before requesting radio-map export", sionnaExportDelay);
    cmd.AddValue("sionnaExportTimeout", "Wall-clock seconds to wait for each Sionna radio-map export response", sionnaExportTimeout);
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
    cmd.AddValue("enableTcpTraces", "Write TCP cwnd/RTT/congestion-state trace CSV files", enableTcpTraces);
    cmd.AddValue("tcpCwndFile", "TCP congestion window CSV output path", tcpCwndFile);
    cmd.AddValue("tcpRttFile", "TCP RTT CSV output path", tcpRttFile);
    cmd.AddValue("tcpRetransmissionsFile", "TCP congestion-state/retransmission event CSV output path", tcpRetransmissionsFile);
    cmd.AddValue("topologyMetadataFile", "JSON metadata describing the ns-3 -> Sionna -> TAP topology", topologyMetadataFile);
    cmd.AddValue("txPower", "LTE UE/eNB TX power in dBm", txPower);
    cmd.AddValue("earfcn", "LTE DL EARFCN; default 100 is near 2.1 GHz", earfcn);
    cmd.Parse(argc, argv);

    std::transform(transport.begin(), transport.end(), transport.begin(), [](unsigned char c) {
        return std::tolower(c);
    });
    if (appStop == 0.0)
    {
        appStop = simTime;
    }
    if (trafficApp == "UdpClient")
    {
        transport = "udp";
    }
    if (transport == "udp" && trafficApp == "BulkSend")
    {
        trafficApp = "UdpClient";
    }
    if (transport == "tcp" && trafficApp == "UdpClient")
    {
        trafficApp = "BulkSend";
    }

    NS_ABORT_MSG_IF(tapMode != "ConfigureLocal" && tapMode != "UseLocal",
                    "tapMode must be ConfigureLocal or UseLocal");
    NS_ABORT_MSG_IF(transport != "tcp" && transport != "udp",
                    "transport must be tcp or udp");
    NS_ABORT_MSG_IF(trafficApp != "BulkSend" && trafficApp != "UdpClient" && trafficApp != "OnOff",
                    "trafficApp must be BulkSend, UdpClient, or OnOff");
    NS_ABORT_MSG_IF(transport == "udp" && trafficApp == "BulkSend",
                    "BulkSend requires TCP; use --transport=tcp or --trafficApp=OnOff");
    NS_ABORT_MSG_IF(transport == "tcp" && trafficApp == "UdpClient",
                    "UdpClient requires UDP; use --transport=udp or --trafficApp=BulkSend/OnOff");
    NS_ABORT_MSG_IF(simTime <= 1.0, "simTime must be greater than one second");
    NS_ABORT_MSG_IF(appStart < 0.0 || appStart >= simTime,
                    "appStart must be non-negative and less than simTime");
    NS_ABORT_MSG_IF(appStop <= appStart || appStop > simTime,
                    "appStop must be greater than appStart and no greater than simTime");
    NS_ABORT_MSG_IF(systemTapPcapStartTime < 0.0 || systemTapPcapStartTime >= simTime,
                    "systemTapPcapStartTime must be non-negative and less than simTime");
    NS_ABORT_MSG_IF(uePositionInterval <= 0.0, "uePositionInterval must be positive");
    NS_ABORT_MSG_IF(uePositionStartTime < 0.0 || uePositionStartTime >= simTime,
                    "uePositionStartTime must be non-negative and less than simTime");
    NS_ABORT_MSG_IF(sionnaExportDelay < 0.0, "sionnaExportDelay must be non-negative");

    const Vector enbVector = ParseVector(enbPosition);
    const std::vector<Vector> uePositionList =
        uePositionsFile.empty() ? ParsePositionList(uePositions) : ParsePositionFile(uePositionsFile);
    std::vector<CoordinatePacketStats> packetStats =
        CreateCoordinateStats(uePositionList,
                              enableUePositionSequence,
                              uePositionInterval,
                              uePositionStartTime,
                              simTime);

    // Real-time scheduling and checksums are needed because this simulation can
    // exchange real Ethernet frames with the Linux network stack through TAP.
    GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));
    if (transport == "tcp")
    {
        Config::SetDefault("ns3::TcpL4Protocol::SocketType",
                           TypeIdValue(TypeId::LookupByName("ns3::" + tcpVariant)));
        Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(packetSize));
    }

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
    if (enableSionna && enableSionnaExportSequence &&
        uePositionStartTime + sionnaExportDelay < simTime)
    {
        Simulator::Schedule(Seconds(uePositionStartTime + sionnaExportDelay),
                            &RequestSionnaRadioMapExport,
                            sionnaServerIp,
                            sionnaExportPort,
                            sionnaExportTimeout,
                            0);
    }
    if (enableUePositionSequence)
    {
        for (uint32_t i = 1; i < uePositionList.size(); ++i)
        {
            const double moveTime = uePositionStartTime + i * uePositionInterval;
            if (moveTime >= simTime)
            {
                continue;
            }
            Simulator::Schedule(Seconds(moveTime),
                                &SetNodePosition,
                                sourceNode,
                                uePositionList[i],
                                "UE car_1");
            if (enableSionna && enableSionnaExportSequence && moveTime + sionnaExportDelay < simTime)
            {
                Simulator::Schedule(Seconds(moveTime + sionnaExportDelay),
                                    &RequestSionnaRadioMapExport,
                                    sionnaServerIp,
                                    sionnaExportPort,
                                    sionnaExportTimeout,
                                    i);
            }
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
    if (enableTap && tapMode == "UseLocal")
    {
        wiredDevices.Get(1)->SetAddress(Mac48Address(tapMac.c_str()));
    }

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
        const double activeSeconds = appStop - appStart;
        maxPackets = static_cast<uint32_t>(activeSeconds / packetInterval.GetSeconds()) + 1;
    }

    if (!enableTap)
    {
        const std::string socketFactory =
            transport == "tcp" ? "ns3::TcpSocketFactory" : "ns3::UdpSocketFactory";
        PacketSinkHelper sink(socketFactory, InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer serverApps = sink.Install(ghostNode.Get(0));
        serverApps.Start(Seconds(0.5));
        serverApps.Stop(Seconds(simTime));

        Ptr<PacketSink> packetSink = DynamicCast<PacketSink>(serverApps.Get(0));
        if (packetSink)
        {
            packetSink->TraceConnectWithoutContext("Rx", MakeCallback(&LogSinkRx));
        }
    }

    ApplicationContainer clientApps;
    if (transport == "udp" && trafficApp == "UdpClient")
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
    else if (transport == "tcp" && trafficApp == "BulkSend")
    {
        BulkSendHelper bulkSend("ns3::TcpSocketFactory", InetSocketAddress(destinationIp, port));
        bulkSend.SetAttribute("SendSize", UintegerValue(packetSize));
        bulkSend.SetAttribute("MaxBytes", UintegerValue(maxBytes));

        clientApps = bulkSend.Install(sourceNode);
        Ptr<BulkSendApplication> bulkSendApp = DynamicCast<BulkSendApplication>(clientApps.Get(0));
        if (bulkSendApp)
        {
            bulkSendApp->TraceConnectWithoutContext("Tx",
                                                    MakeBoundCallback(&RecordAppTx, &packetStats));
        }
    }
    else
    {
        const std::string socketFactory =
            transport == "tcp" ? "ns3::TcpSocketFactory" : "ns3::UdpSocketFactory";
        OnOffHelper onoff(socketFactory, InetSocketAddress(destinationIp, port));
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
    clientApps.Start(Seconds(appStart));
    clientApps.Stop(Seconds(appStop));

    std::ofstream tcpCwndStream;
    std::ofstream tcpRttStream;
    std::ofstream tcpRetransmissionsStream;
    if (transport == "tcp" && enableTcpTraces)
    {
        tcpCwndStream.open(tcpCwndFile);
        tcpRttStream.open(tcpRttFile);
        tcpRetransmissionsStream.open(tcpRetransmissionsFile);
        if (tcpCwndStream.is_open())
        {
            tcpCwndStream << "time_seconds,old_cwnd_bytes,new_cwnd_bytes\n";
        }
        if (tcpRttStream.is_open())
        {
            tcpRttStream << "time_seconds,old_rtt_ms,new_rtt_ms\n";
        }
        if (tcpRetransmissionsStream.is_open())
        {
            tcpRetransmissionsStream << "time_seconds,old_congestion_state,new_congestion_state\n";
        }
        Simulator::Schedule(Seconds(appStart + 0.001),
                            &ConnectTcpTraces,
                            sourceNode->GetId(),
                            &tcpCwndStream,
                            &tcpRttStream,
                            &tcpRetransmissionsStream);
    }

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
            std::max(0.1, std::min(appStart - 0.1, systemTapPcapStartTime + 0.5));
        Simulator::Schedule(Seconds(pauseTime),
                            &PauseForManualCapture,
                            manualCapturePauseSeconds,
                            tapName,
                            port,
                            transport);
    }

    NS_LOG_UNCOND("Sionna LTE ns-3 to Linux TAP proof-of-chain");
    NS_LOG_UNCOND("  traffic direction=ns3_to_tap");
    NS_LOG_UNCOND("  topology: ns-3 UE app -> Sionna-managed LTE channel -> eNB/EPC PGW -> CSMA output link -> TapBridge -> Linux " << tapName);
    NS_LOG_UNCOND("  source nodeId=" << sourceNode->GetId() << " Sionna object=car_1"
                                      << " position="
                                      << PositionToString(sourceNode->GetObject<MobilityModel>()->GetPosition()));
    NS_LOG_UNCOND("  eNB nodeId=" << enbNode->GetId() << " Sionna object=car_2"
                                  << " position="
                                  << PositionToString(enbNode->GetObject<MobilityModel>()->GetPosition()));
    NS_LOG_UNCOND("  UE IP=" << sourceIp << " EPC default gateway="
                             << epcHelper->GetUeDefaultGatewayAddress());
    NS_LOG_UNCOND("  PGW wired IP=" << gatewayWiredIp << " destination IP=" << destinationIp);
    NS_LOG_UNCOND("  " << transport << " destination=" << destinationIp << ":" << port
                       << " app=" << trafficApp
                       << " tcpVariant=" << tcpVariant
                       << " packetSize=" << packetSize
                       << " maxBytes=" << maxBytes
                       << " interval=" << interval
                       << " dataRate=" << dataRate
                       << " appStart=" << appStart << "s"
                       << " appStop=" << appStop << "s");
    NS_LOG_UNCOND("  TapBridge enabled=" << (enableTap ? "true" : "false")
                                         << " mode=" << tapMode
                                         << " tapName=" << tapName
                                         << " tapMac=" << tapMac);
    NS_LOG_UNCOND("  static PGW/TAP ARP cache=" << (enableStaticNeighborCache ? "true" : "false"));
    NS_LOG_UNCOND("  UE position sequence enabled=" << (enableUePositionSequence ? "true" : "false")
                                                    << " interval=" << uePositionInterval
                                                    << "s startTime=" << uePositionStartTime
                                                    << "s count=" << uePositionList.size());
    NS_LOG_UNCOND("  Sionna export sequence enabled=" << (enableSionna && enableSionnaExportSequence ? "true" : "false")
                                                      << " exportDelay=" << sionnaExportDelay
                                                      << "s exportPort=" << sionnaExportPort);
    NS_LOG_UNCOND("  PCAP enabled=" << (enablePcap ? "true" : "false")
                                    << " prefix=" << pcapPrefix);
    NS_LOG_UNCOND("  packet summary enabled=" << (enablePacketSummary ? "true" : "false")
                                              << " file=" << packetSummaryFile);
    NS_LOG_UNCOND("  Linux TAP tcpdump enabled="
                  << (enableTap && enableSystemTapPcap ? "true" : "false")
                  << " file=" << systemTapPcapFile
                  << " startTime=" << systemTapPcapStartTime << "s");
    NS_LOG_UNCOND("  manual capture pause=" << manualCapturePauseSeconds << "s");
    NS_LOG_UNCOND("  expected tcpdump: sudo tcpdump -i " << tapName << " -n -vv "
                                              << transport << " port " << port);
    NS_LOG_UNCOND("  expected listener: "
                  << (transport == "tcp" ? "nc -l " : "nc -ul ") << port);

    if (!topologyMetadataFile.empty())
    {
        std::ofstream topology(topologyMetadataFile);
        if (topology.is_open())
        {
            topology << "{\n";
            topology << "  \"traffic_direction\": \"ns3_to_tap\",\n";
            topology << "  \"transport\": \"" << transport << "\",\n";
            topology << "  \"tap_enabled\": " << (enableTap ? "true" : "false") << ",\n";
            topology << "  \"tap_name\": \"" << tapName << "\",\n";
            topology << "  \"tap_mode\": \"" << tapMode << "\",\n";
            topology << "  \"tap_mac\": \"" << tapMac << "\",\n";
            topology << "  \"source_node\": \"UE car_1\",\n";
            topology << "  \"source_ip\": \"" << sourceIp << "\",\n";
            topology << "  \"sionna_channel\": \"LTE UE car_1 to eNB car_2 via SionnaHelper\",\n";
            topology << "  \"sionna_tx_object\": \"car_1\",\n";
            topology << "  \"sionna_rx_object\": \"car_2\",\n";
            topology << "  \"output_node\": \"ghost/TapBridge side\",\n";
            topology << "  \"pgw_wired_ip\": \"" << gatewayWiredIp << "\",\n";
            topology << "  \"tap_destination_ip\": \"" << destinationIp << "\",\n";
            topology << "  \"ue_position_sequence_enabled\": " << (enableUePositionSequence ? "true" : "false") << ",\n";
            topology << "  \"ue_position_count\": " << uePositionList.size() << ",\n";
            topology << "  \"ue_position_interval_s\": " << uePositionInterval << ",\n";
            topology << "  \"ue_position_start_time_s\": " << uePositionStartTime << ",\n";
            topology << "  \"sionna_export_sequence_enabled\": " << (enableSionna && enableSionnaExportSequence ? "true" : "false") << ",\n";
            topology << "  \"port\": " << port << ",\n";
            topology << "  \"tap_tcpdump_command\": \"sudo tcpdump -i " << tapName << " -n -vv " << transport << " port " << port << "\",\n";
            topology << "  \"host_listener_command\": \"" << (transport == "tcp" ? "nc -l " : "nc -ul ") << port << "\",\n";
            topology << "  \"notes\": \"Packets are generated inside ns-3 and sent outward toward the Linux TAP. Traffic does not enter from TAP first.\"\n";
            topology << "}\n";
        }
    }

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
    if (transport == "tcp" && enableTcpTraces)
    {
        NS_LOG_UNCOND("  wrote TCP cwnd trace: " << tcpCwndFile);
        NS_LOG_UNCOND("  wrote TCP RTT trace: " << tcpRttFile);
        NS_LOG_UNCOND("  wrote TCP congestion-state trace: " << tcpRetransmissionsFile);
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
