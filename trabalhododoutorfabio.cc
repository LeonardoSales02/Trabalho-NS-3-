#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/vector.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("IoTWifiSimulation");

int
main(int argc, char *argv[])
{
  // ----------------- PARÂMETROS PRINCIPAIS -----------------
  uint32_t nSensors = 27;      // número de sensores
  double simTime = 47.0;       // tempo de simulação [s]
  double packetInterval = 1.0; // intervalo entre pacotes [s]
  uint32_t packetSize = 64;    // tamanho do pacote [bytes]
  double txPower = 20.0;       // potência de transmissão [dBm]

  // Permite alterar parâmetros pela linha de comando
  CommandLine cmd;
  cmd.AddValue("packetInterval", "Intervalo entre pacotes (s)", packetInterval);
  cmd.AddValue("txPower", "Potencia de transmissao (dBm)", txPower);
  cmd.AddValue("nSensors", "Numero de nos sensores", nSensors);
  cmd.Parse(argc, argv);

  // ----------------- CRIAÇÃO DOS NÓS -----------------
  NodeContainer sensorNodes;
  sensorNodes.Create(nSensors);

  NodeContainer sinkNode;
  sinkNode.Create(1);

  NodeContainer allNodes(sensorNodes, sinkNode);

  // ----------------- CANAL + FÍSICA (Wi-Fi) -----------------
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
  // Modelo de propagação: Friis (alcance "amigável")
  channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  channel.AddPropagationLoss("ns3::FriisPropagationLossModel");

  // PHY sem ::Default(), compatível com tua versão
  YansWifiPhyHelper phy;
  phy.SetChannel(channel.Create());

  // Ajuste de TxPower (pode variar nos experimentos)
  phy.Set("TxPowerStart", DoubleValue(txPower));
  phy.Set("TxPowerEnd", DoubleValue(txPower));

  WifiHelper wifi;
  // Não definimos SetStandard nem RemoteStationManager explicitamente:
  // usa os padrões da tua versão do ns-3, que são compatíveis.

  WifiMacHelper mac;
  Ssid ssid = Ssid("IoT-Network");

  // Sensores como STAs
  mac.SetType("ns3::StaWifiMac",
              "Ssid", SsidValue(ssid),
              "ActiveProbing", BooleanValue(false));
  NetDeviceContainer sensorDevs = wifi.Install(phy, mac, sensorNodes);

  // Sink como AP
  mac.SetType("ns3::ApWifiMac",
              "Ssid", SsidValue(ssid));
  NetDeviceContainer sinkDev = wifi.Install(phy, mac, sinkNode);

  // ----------------- MOBILIDADE (POSIÇÃO ALEATÓRIA ESTÁTICA) -----------------
  MobilityHelper mobility;

  // Área 30m x 30m
  mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                "X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=30.0]"),
                                "Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=30.0]"));

  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(allNodes);

  // Força o sink a ficar no centro da área (15,15)
  Ptr<MobilityModel> sinkMobility = sinkNode.Get(0)->GetObject<MobilityModel>();
  sinkMobility->SetPosition(Vector(15.0, 15.0, 0.0));

  // ----------------- PILHA IP -----------------
  InternetStackHelper stack;
  stack.Install(allNodes);

  Ipv4AddressHelper address;
  address.SetBase("10.1.1.0", "255.255.255.0");

  NetDeviceContainer allDevs;
  allDevs.Add(sensorDevs);
  allDevs.Add(sinkDev);

  Ipv4InterfaceContainer interfaces = address.Assign(allDevs);

  // Endereço do sink é o último (índice nSensors)
  Ipv4Address sinkAddress = interfaces.GetAddress(nSensors);
  uint16_t sinkPort = 4000;

  // ----------------- APLICAÇÕES UDP -----------------

  // Servidor no sink
  UdpServerHelper server(sinkPort);
  ApplicationContainer serverApp = server.Install(sinkNode.Get(0));
  serverApp.Start(Seconds(0.0));
  serverApp.Stop(Seconds(simTime));

  // Clientes nos sensores
  UdpClientHelper client(sinkAddress, sinkPort);
  client.SetAttribute("MaxPackets", UintegerValue(0)); // envia até o Stop
  client.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
  client.SetAttribute("PacketSize", UintegerValue(packetSize));

  ApplicationContainer clientApps;
  for (uint32_t i = 0; i < nSensors; ++i)
    {
      clientApps.Add(client.Install(sensorNodes.Get(i)));
    }

  clientApps.Start(Seconds(1.0));
  clientApps.Stop(Seconds(simTime));

  // ----------------- FLOW MONITOR (MÉTRICAS) -----------------
  FlowMonitorHelper flowmonHelper;
  Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

  Simulator::Stop(Seconds(simTime));
  Simulator::Run();

  monitor->CheckForLostPackets();
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

  uint64_t totalTxPackets = 0;
  uint64_t totalRxPackets = 0;
  uint64_t totalRxBytes = 0;
  double   sumDelay = 0.0;        // soma dos atrasos (s)
  uint64_t rxPacketsForDelay = 0; // número de pacotes recebidos (para média)

  for (auto const &flow : stats)
    {
      const FlowMonitor::FlowStats &fs = flow.second;
      totalTxPackets += fs.txPackets;
      totalRxPackets += fs.rxPackets;
      totalRxBytes   += fs.rxBytes;
      sumDelay       += fs.delaySum.GetSeconds();
      rxPacketsForDelay += fs.rxPackets;
    }

  double pdr = 0.0;
  double avgDelay = 0.0;
  double throughputKbps = 0.0;

  if (totalTxPackets > 0)
    {
      pdr = static_cast<double>(totalRxPackets) / static_cast<double>(totalTxPackets);
    }

  if (rxPacketsForDelay > 0)
    {
      avgDelay = sumDelay / static_cast<double>(rxPacketsForDelay);
    }

  if (simTime > 0.0)
    {
      double throughputbps = (static_cast<double>(totalRxBytes) * 8.0) / simTime;
      throughputKbps = throughputbps / 1000.0;
    }

  std::cout << "========== RESULTADOS ==========\n";
  std::cout << "Sensores:            " << nSensors << "\n";
  std::cout << "Tempo de simulacao:  " << simTime << " s\n";
  std::cout << "TxPower:             " << txPower << " dBm\n";
  std::cout << "Intervalo pacotes:   " << packetInterval << " s\n";
  std::cout << "Pacotes transmitidos:" << totalTxPackets << "\n";
  std::cout << "Pacotes recebidos:   " << totalRxPackets << "\n";
  std::cout << "PDR:                 " << pdr * 100.0 << " %\n";
  std::cout << "Atraso medio:        " << avgDelay << " s\n";
  std::cout << "Throughput medio:    " << throughputKbps << " kbps\n";
  std::cout << "================================\n";

  Simulator::Destroy();
  return 0;
}
