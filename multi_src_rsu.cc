#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/netanim-module.h"
#include "ns3/ipv4-header.h"
#include "ns3/packet.h"
#include "ns3/tag.h"
#include "ns3/on-off-helper.h"
#include "ns3/snr-tag.h"
#include "ns3/event-id.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <unistd.h>
#include <map>
#include<stdio.h>
#include <iomanip>

NS_LOG_COMPONENT_DEFINE ("Truth1");
using namespace ns3;
//using namespace std;
uint16_t rsu_nmbr;
bool found=false;
uint16_t maxDistance = 11; // m //最大传播距离
uint32_t packetSize  = 1000; // bytes
uint16_t beaconPort  = 80;
uint32_t numNodes    = 16;
uint32_t numNodes2    = 200;
uint32_t numPackets  = 1;
double interval = 1.0;
double rsu_time=0;
int xt=0;
std::map<int, int> src_status;
std::map<int, int> fin_status;
 Time interPacketInterval = Seconds (interval);
bool verbose=false;
uint32_t dest;
std::set< int> nodecnt;
std::set<int> nodecnt2;
std::vector< int> left;
std::vector< int> right;
NodeContainer rsu,vehicles;
double stnd=100;
bool frontpath(std::vector< int> &left,std::set< int> &nodecnt,int ttl,double sim_time, ns3::Ipv4InterfaceContainer i_source,uint32_t dest);
static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval );
void ReceivePacket (Ptr<Socket> socket)
{
   //std::cout<<"Received one packet!"<<std::endl;
  while (socket->Recv ())
    {
      std::cout<<"Received one packet!"<<std::endl;
    }
   
 
}
static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}
bool frontpath(std::vector< int> &left,std::set< int> &nodecnt,int ttl,double sim_time, ns3::Ipv4InterfaceContainer i_source2,uint32_t dest)
{

   if(ttl==0 || left.size()==1)
   { 
    if(left.size()==1)
     std::cout<<"nodes unavailable"<<std::endl;
     else
     std::cout<<"ttl over"<<std::endl;  
      return false;
    }

 
  std::set<int>:: iterator it;
 if(left[0]==500)
 {
   ttl--;
   sim_time+=Simulator::Now ().GetSeconds ()+.02;
   //std::cout<<"present here"<<std::endl;
   left.erase(left.begin()+0);
   // std::cout<<"contents of left"<<std::endl;
   // for(uint32_t j=0;j<left.size();j++)
   //  std::cout<<left[j]+16<<std::endl;
   if(frontpath(left,nodecnt,ttl,sim_time,i_source2,dest))
   {
    return true;
   }
 }
 else
 {
    Ptr<MobilityModel> mob2=vehicles.Get(left[0])->GetObject<MobilityModel>();
  Vector pos=mob2->GetPosition();
   TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    // Ptr<Socket> recvSink = Socket::CreateSocket (vehicles.Get (dest), tid);
    // InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
    // recvSink->Bind (local);
    // recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
    
      ns3::Ptr<Socket> source = Socket::CreateSocket (vehicles.Get (left[0]), tid);
      ns3::InetSocketAddress remote = InetSocketAddress (i_source2.GetAddress (dest, 0), 80);//agar phncha iss address pe toh revert krta hai
       source->Connect (remote);
    Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                   Seconds (sim_time), &GenerateTraffic,
                                   source, packetSize, numPackets, interPacketInterval);
    //std::cout<<"Paari start: "<<(16+left[0])<<std::endl;
  // uint32_t initsize=left.size();
   for(uint32_t j=0;j<16;j++)
   {
     Ptr<MobilityModel> mob3=rsu.Get(j)->GetObject<MobilityModel>();
       Vector rsu_node=mob3->GetPosition(); 
        double dis2=((pos.x-rsu_node.x)*(pos.x-rsu_node.x))+((pos.y-rsu_node.y)*(pos.y-rsu_node.y));
       if(dis2<=stnd)
       {
        rsu_nmbr=j;
        //std::cout<<"Rsu no."<<j<<"sim_time"<<sim_time<<std::endl;
        rsu_time=Simulator::Now ().GetSeconds ()+.2;
       // std::cout<<"rsu_time "<<rsu_time<<std::endl;
        return true;
       }
   }
   for(it=nodecnt.begin();it!=nodecnt.end();it++)
   {
     uint32_t check_node=*it;
      Ptr<MobilityModel> mob=vehicles.Get(check_node)->GetObject<MobilityModel>();
       Vector dest_pos=mob->GetPosition(); 
       double dis1=((pos.x-dest_pos.x)*(pos.x-dest_pos.x))+((pos.y-dest_pos.y)*(pos.y-dest_pos.y));
      if(dis1<=stnd)
      {
        if(check_node==dest)
          found=true;
       //std::cout<<"vehicle caught"<<*it+16<<std::endl;
        left.push_back(*it);
      }
   }
       
      left.erase(left.begin()+0);
      
     
      for(uint32_t j=0;j<left.size();j++)
      {
         it=nodecnt.find(left[j]);
         if(it!=nodecnt.end())
         {

        // std::cout<<*it<<std::endl;
          nodecnt.erase(it);
         }
      }
      left.push_back(500);
      if(frontpath(left,nodecnt,ttl,sim_time,i_source2,dest))
      {
        return true;
      }
  }
  return false;
} 

int main (int argc, char *argv[])
{
  
  std::string phyMode ("DsssRate1Mbps");
  std::string traceFile;
  
  CommandLine cmd;
  cmd.Parse (argc, argv);
   // disable fragmentation for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  // turn off RTS/CTS for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (phyMode));

  
   rsu.Create (numNodes);
   vehicles.Create(numNodes2);

  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents ();  // Turn on all Wifi logging
    }
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
 
  wifiPhy.Set("TxPowerStart", DoubleValue(43));
  wifiPhy.Set("TxPowerEnd", DoubleValue(43));
  wifiPhy.Set("TxPowerLevels", UintegerValue(1));
  wifiPhy.Set("TxGain", DoubleValue(1));
  wifiPhy.Set("RxGain", DoubleValue (1));
  wifiPhy.Set ("RxNoiseFigure", DoubleValue (10));

  Config::SetDefault( "ns3::RangePropagationLossModel::MaxRange", DoubleValue( maxDistance ) );
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay( "ns3::ConstantSpeedPropagationDelayModel" );
  wifiChannel.AddPropagationLoss("ns3::NakagamiPropagationLossModel");
  wifiChannel.AddPropagationLoss(  "ns3::RangePropagationLossModel" );
  wifiPhy.SetChannel( wifiChannel.Create() );
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  // Add a non-QoS upper mac, and disable rate control
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));
 
  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer rsu_devices = wifi.Install (wifiPhy, wifiMac, rsu);
  NetDeviceContainer vehicle_devices=wifi.Install(wifiPhy, wifiMac, vehicles);
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
  "MinX", DoubleValue (0.0),
  "MinY", DoubleValue (0.0),
  "DeltaX", DoubleValue (5.0),
  "DeltaY", DoubleValue (5.0),
  "GridWidth", UintegerValue (3),
  "LayoutType", StringValue ("RowFirst")); 
   
 /*  mobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (0, 100, 0, 100)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=20]"),
                                "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=0.2]")); 
  */
  MobilityHelper mobility2;
 mobility2.SetPositionAllocator ("ns3::GridPositionAllocator",
 "MinX", DoubleValue (0.0),
 "MinY", DoubleValue (0.0),
 "DeltaX", DoubleValue (5.0),
 "DeltaY", DoubleValue (5.0),
 "GridWidth", UintegerValue (3),
 "LayoutType", StringValue ("RowFirst")); 
   
  mobility2.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (0, 100, 0, 100)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=20]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=0.2]")); 

  
   Ptr<ListPositionAllocator> positionAlloc2 = CreateObject<ListPositionAllocator> ();
  for(uint32_t j=0;j<=19;j=j+1)
 {
  for(uint32_t k=0;k<=19;k=k+2)
   {
     positionAlloc2->Add (Vector (j*5+1, k*5+1, 0.0));
   }
 }
  mobility2.SetPositionAllocator (positionAlloc2);
   Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    for(uint32_t j=0;j<=6;j=j+2)
 {
  for(uint32_t k=0;k<=6;k=k+2)
   {
     positionAlloc->Add (Vector (j*16+2, k*16+2, 0.0));
   }
 }
  mobility.SetPositionAllocator (positionAlloc);
 mobility.Install (rsu);
 InternetStackHelper internet;
 internet.Install (rsu);
 
 mobility2.Install (vehicles);
 InternetStackHelper internet2;
 internet2.Install (vehicles);
 Ipv4AddressHelper ipv4_source2;
 ipv4_source2.SetBase ("20.1.1.0", "255.255.255.0");
 Ipv4InterfaceContainer i_source2 = ipv4_source2.Assign (vehicle_devices);
 Ipv4AddressHelper ipv4_source;
 NS_LOG_INFO ("Assign IP Addresses.");
 ipv4_source.SetBase ("10.1.1.0", "255.255.255.0");
 Ipv4InterfaceContainer i_source = ipv4_source.Assign (rsu_devices);
// std::vector<int> res;
 uint16_t ttl=4;
 double sim_time=Simulator::Now ().GetSeconds ()+.3;
// uint32_t pd=0;
std::set<int>:: iterator it;//int l=0;                          
TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
TypeId tid2 = TypeId::LookupByName ("ns3::UdpSocketFactory");
//for(uint32_t k=0;k<200;k++)
        

 uint32_t k;
for(k=0;k<=199;k++)
{
found=false;
//bool stat=false;
int dRan;//,sRan
std::srand( time(0)); // This will ensure a really randomized number by help of time.

//sRan=std::rand()%200;
dRan=std::rand()%200;
dest=dRan;
// k=sRan;
std::cout<<"Source :"<<k<<"Dest:"<<dest<<std::endl;
left.clear();
nodecnt.clear();
for(uint32_t j=0;j<200;j++)
{
if(j!=k)
nodecnt.insert(j);
// nodecnt2.insert(j);
}
left.push_back(k);
left.push_back(500);
if(frontpath(left,nodecnt,ttl,sim_time,i_source2,dest))
{
std::cout<<"RSU Number"<<rsu_nmbr<<std::endl;
src_status.insert(std::make_pair(k,1));
//continue;

// Config::SetDefault( "ns3::RangePropagationLossModel::MaxRange", DoubleValue( 200.0 ) );
// TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

Ptr<Socket> rsu_source = Socket::CreateSocket (rsu.Get (rsu_nmbr), tid2);
InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
//res.push_back(rsu_nmbr);
rsu_source->SetAllowBroadcast (true);
rsu_source->Connect (remote);
rsu_time=Simulator::Now ().GetSeconds ()+sim_time+.3;
Simulator::ScheduleWithContext (rsu_source->GetNode ()->GetId (),
                 Seconds (rsu_time), &GenerateTraffic,
                 rsu_source, packetSize, numPackets, interPacketInterval);
}
}

AnimationInterface anim("Atruth1.xml");

 Simulator::Stop(MilliSeconds(2000));
 Simulator::Run ();
 Simulator::Destroy();
}

