#include "ros_linker.h"

rnl::Interface::Interface ( std::string _phyMode = "DsssRate1Mbps", 
  double _rss = -80,  // -dBm [deprecated]
  uint32_t _packetSize = 1000, // bytes
  uint32_t _numPackets = 10,
  double _interval = 1.0, // seconds
  bool _verbose = false,
  double _stopTime = 1000.0,
  int _num_nodes = 3
):
  phy_mode{_phyMode}, 
  rss{_rss}, 
  packet_size{_packetSize}, 
  num_packets{_numPackets},
  interval{_interval}, 
  verbose{_verbose},
  stop_time{_stopTime},
  num_nodes{_num_nodes}
{
}

void rnl::Interface::initialize(bool rt , bool chsum ) 
{
  inter_packet_interval = ns3::Seconds(interval);
  broadcast_interval = ns3::Seconds(interval);
  
  if (rt){
      ns3::GlobalValue::Bind ("SimulatorImplementationType", ns3::StringValue ("ns3::RealtimeSimulatorImpl"));
      ns3::GlobalValue::Bind ("ChecksumEnabled", ns3::BooleanValue (chsum));
  }

  // Fix non-unicast data rate to be the same as that of unicast
  ns3::Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", ns3::StringValue (phy_mode));

  c.Create(num_nodes);
  tid = ns3::TypeId::LookupByName ("ns3::UdpSocketFactory");
  std::cerr << "Initialization Completed" << std::endl;
}

void rnl::Interface::setWifi()
{
  // The below set of helpers will help us to put together the wifi NICs we want

  if (verbose)
  {
    wifi.EnableLogComponents ();  // Turn on all Wifi logging
  }
    
  wifi.SetStandard (ns3::WIFI_STANDARD_80211b);

  ns3::YansWifiPhyHelper wifiPhy;

  // This is one parameter that matters when using FixedRssLossModel
  // set it to zero; otherwise, gain will be added
  wifiPhy.Set ("RxGain", ns3::DoubleValue (0) );

  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType (ns3::WifiPhyHelper::DLT_IEEE802_11_RADIO);

  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");

  // The below FixedRssLossModel will cause the rss to be fixed regardless
  // of the distance between the two stations, and the transmit power
  wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",ns3::DoubleValue (rss));
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add a mac and disable rate control
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                  "DataMode",ns3::StringValue (phy_mode),
                                  "ControlMode",ns3::StringValue (phy_mode));

  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  
  devices = wifi.Install (wifiPhy, wifiMac, this->c);

  wifiPhy.EnablePcap ("mavad", devices);

  std::cerr << "Wifi Properties Set" << std::endl;
}


void rnl::Interface::setMobility()
{
  // Note that with FixedRssLossModel, the positions below are not
  // used for received signal strength.
  ns3::Ptr<ns3::ListPositionAllocator> positionAlloc = ns3::CreateObject<ns3::ListPositionAllocator> ();
  for (int i = 0; i< num_nodes; ++i){
    positionAlloc->Add (ns3::Vector (5*i, 0.0, 0.0));
  }
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c);

  std::cerr << "Mobility Set" << std::endl;
}

void rnl::Interface::setInternet()
{
  internet.Install (c);
  std::cerr << "Assigning IP" << std::endl;
  
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  i = ipv4.Assign (devices);

  std::cerr << "IPs Assigned" << std::endl;
}

void rnl::Interface::setReceivers()
{
  recv_sink1 = ns3::Socket::CreateSocket (c.Get (0), tid);
  ns3::InetSocketAddress local1 = ns3::InetSocketAddress (ns3::Ipv4Address::GetAny (), 9);
  recv_sink1->Bind (local1);
  recv_sink1->SetRecvCallback (ns3::MakeCallback (&rnl::Interface::receivePacket1 , this));

  recv_sink2 = ns3::Socket::CreateSocket (c.Get (1), tid);
  ns3::InetSocketAddress local2 = ns3::InetSocketAddress (ns3::Ipv4Address::GetAny (), 9);
  recv_sink2->Bind (local2);
  recv_sink2->SetRecvCallback (ns3::MakeCallback (&rnl::Interface::receivePacket2, this));
  
  recv_sink3 = ns3::Socket::CreateSocket (c.Get (2), tid);
  ns3::InetSocketAddress local3 = ns3::InetSocketAddress (ns3::Ipv4Address::GetAny (), 9);
  recv_sink3->Bind (local3);
  recv_sink3->SetRecvCallback (ns3::MakeCallback (&rnl::Interface::receivePacket3, this));
  
  std::cerr << "Receivers Set" << std::endl;
}

void rnl::Interface::setSenders()
{
  source1 = ns3::Socket::CreateSocket (c.Get (0), tid);
  ns3::InetSocketAddress remote1 = ns3::InetSocketAddress (ns3::Ipv4Address (i.GetAddress(1,0)), 9);
  source1->SetAllowBroadcast (true);
  source1->Connect (remote1);
  

  source2 = ns3::Socket::CreateSocket (c.Get (1), tid);
  ns3::InetSocketAddress remote2 = ns3::InetSocketAddress (ns3::Ipv4Address (i.GetAddress(2,0)), 9);
  source2->SetAllowBroadcast (true);
  source2->Connect (remote2);

  std::cerr << "Senders Set" << std::endl;
}

void rnl::Interface::receivePacket1(ns3::Ptr<ns3::Socket> soc)
{
  /*The Master node. Only sends*/
  while (ns3::Ptr<ns3::Packet> msg = soc->Recv ())
  {
	  std::cerr << "Node 1 received Packet at " << ns3::Simulator::Now ().GetSeconds() << std::endl;
  }
} 

void rnl::Interface::receivePacket2(ns3::Ptr<ns3::Socket> soc)
{
  std::string receivedData;
  while (ns3::Ptr<ns3::Packet> msg = soc->Recv ())
  {
    std::cerr << "Node 2 received Packet at " << ns3::Simulator::Now ().GetSeconds() << std::endl;
    uint8_t *buffer = new uint8_t[msg->GetSize ()];
    msg->CopyData (buffer, msg->GetSize ());
    receivedData = std::string ((char *) buffer);
  }
    
  ns3::Simulator::Schedule (inter_packet_interval, &rnl::Interface::sendPacket2,this,
	source2, packet_size, inter_packet_interval);
} 

void rnl::Interface::receivePacket3(ns3::Ptr<ns3::Socket> soc)
{
  std::string receivedData;
  while (ns3::Ptr<ns3::Packet> msg = soc->Recv ())
  {
	  std::cerr << "Node 3 received Packet at " << ns3::Simulator::Now ().GetSeconds() << std::endl;
    uint8_t *buffer = new uint8_t[msg->GetSize ()];
    msg->CopyData (buffer, msg->GetSize ());
    receivedData = std::string ((char *) buffer);
  }
} 


void rnl::Interface::sendPacket1 (ns3::Ptr<ns3::Socket> soc, uint32_t pktSize, ns3::Time pktInterval)
{
  /*The Master node. Only sends*/
  msg1.str("");
  msg1.clear();
  msg1 << "Hello from Node1";
	ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> ((uint8_t*) msg1.str().c_str(), msg1.str().length());
	
  std::cerr << "Sending packet from node 1 at " << ns3::Simulator::Now().GetSeconds() << std::endl;
  soc->Send (packet);

	ns3::Simulator::Schedule (3*pktInterval, &rnl::Interface::sendPacket1, this, soc, pktSize, pktInterval);
}


void rnl::Interface::sendPacket2 (ns3::Ptr<ns3::Socket> soc, uint32_t pktSize, ns3::Time pktInterval)
{
  msg2.str("");
  msg2.clear();
  msg2<<"Hello from Node2";
  
	ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> ((uint8_t*) msg2.str().c_str(), msg2.str().length());
	std::cerr << "Sending packet from node 2 at " << ns3::Simulator::Now().GetSeconds() << std::endl;
	soc->Send (packet);   
}

void rnl::Interface::startSimul()
{
  ns3::Simulator::Schedule (ns3::Seconds (2.0), &rnl::Interface::sendPacket1, this, source1, packet_size, inter_packet_interval);
  ns3::Simulator::Stop(ns3::Seconds(stop_time));
  ns3::AnimationInterface anim ("animation.xml");
  ns3::Simulator::Run();
  ns3::Simulator::Destroy();
}
