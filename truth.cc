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
uint16_t rsu_nmbr=0;
//bool found2=false;
bool found=false;
uint16_t maxDistance = 11; // m //最大传播距离
uint32_t packetSize  = 1; // bytes
uint16_t beaconPort  = 80;
uint32_t numNodes    = 16;
uint32_t numNodes2    = 900;
uint32_t numPackets  = 1;
double interval = 1.0;
double rsu_time=0;
//uint32_t brk=50000;
uint32_t pacDel=0;
int xt=1;
std::map<int, int> src_status;
std::map<int, int> fin_status;
 Time interPacketInterval = Seconds (interval);
bool verbose=false;
//uint32_t dest;
std::set< int> nodecnt;
std::set<int> nodecnt2;
 uint16_t ttl=4;
std::vector< int> right;
NodeContainer rsu,vehicles;
double stnd=13;
bool frontpath(std::vector< int> &left,std::set< int> &nodecnt,int ttl,double sim_time, ns3::Ipv4InterfaceContainer i_source,uint32_t dest);
static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval );
bool Call_rsu(std::set<int> &nodecnt2, std::vector<uint32_t> find_dest[],uint32_t dest);
void ReceivePacket (Ptr<Socket> socket)
{
   //std::cout<<"Received one packet!"<<std::endl;
  if(xt!=0)
  {
  //while (socket->Recv ())
    {
      xt=0;
      std::cout<<"Received one packet!"<<std::endl;
    }
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
bool frontpath(std::vector< int> &left,std::set< int> &nodecnt,int ttll,double sim_time, ns3::Ipv4InterfaceContainer i_source2,uint32_t dest)
{

   if(ttll==0 || left.size()==1)
   { 
    if(left.size()==1)
     std::cout<<"nodes unavailable"<<std::endl;
     else
     std::cout<<"ttl over"<<std::endl;  
      return false;
    }

 
  std::set<int>:: iterator it;
 if(left[0]==50000)
 {
   ttll--;
   sim_time+=Simulator::Now ().GetSeconds ()+.02;
   //std::cout<<"present here"<<std::endl;
   left.erase(left.begin()+0);
   // std::cout<<"contents of left"<<std::endl;
   // for(uint32_t j=0;j<left.size();j++)
   //  std::cout<<left[j]+16<<std::endl;
   if(frontpath(left,nodecnt,ttll,sim_time,i_source2,dest))
   {
    return true;
   }
 }
 else
 {
    Ptr<MobilityModel> mob2=vehicles.Get(left[0])->GetObject<MobilityModel>();
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    Ptr<Socket> recvSink = Socket::CreateSocket (vehicles.Get (dest), tid);
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
    recvSink->Bind (local);
    recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
    
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
     double dis2=mob3->GetDistanceFrom(mob2);
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
      double dis1=mob2->GetDistanceFrom(mob);
      if(dis1<=stnd)
      {
        if(check_node==dest)
         { 
          found=true;
          return true;
          }
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
      left.push_back(50000);
      if(frontpath(left,nodecnt,ttl,sim_time,i_source2,dest))
      {
        return true;
      }
  }
  if(found==true)
    return true;
  return false;
} 
bool Call_rsu(std::set<int> &nodecnt2, std::vector<uint32_t> find_dest[],uint32_t dest)
{
  bool found2=false;
  std::set<int>:: iterator it;
  //std::cout<<"LEVEL 1 NODES"<<std::endl;
  for(uint32_t j=0;j<numNodes2;j++)
  {
     it=nodecnt2.find(j);
     Ptr<MobilityModel> mob=vehicles.Get(j)->GetObject<MobilityModel>();
     if(it==nodecnt2.end())
      {
        continue;
      }
     for(uint16_t k=0;k<16;k++)
     {
      Ptr<MobilityModel> mob2=rsu.Get(k)->GetObject<MobilityModel>();
      double dis1=mob2->GetDistanceFrom(mob);
      if(dis1<=stnd)
       {
          //std::cout<<i<<std::endl;
          if(j==dest)
          found2=true;
          find_dest[0].push_back(j);
          if(nodecnt2.find(j)!=nodecnt2.end())
          nodecnt2.erase(it);
          break;
        }
      }
     }
  for(uint16_t h=0;h<ttl-1;h++)
   {
   //  std::cout<<"LEVEL "<< h+1<< " NODES AFTER RSU"<<std::endl;
    for(uint32_t n=0;n<numNodes2;n++)
    {
      Ptr<MobilityModel> mobi=vehicles.Get(n)->GetObject<MobilityModel>();
      it=nodecnt2.find(n);
      if(it==nodecnt2.end())
      {
       continue;
      }
      for(uint32_t f=0;f<find_dest[h].size() ;f++)
      {
        
        if( n!=find_dest[h][f])
        {
             Ptr<MobilityModel> mobii=vehicles.Get(find_dest[h][f])->GetObject<MobilityModel>();
             double dis1=mobi->GetDistanceFrom(mobii);
           if(dis1<=stnd)
             {
              //std::cout<<n<<std::endl;
                find_dest[h+1].push_back(n);
                if(nodecnt2.find(n)!=nodecnt2.end())
                nodecnt2.erase(it);
                if(n==dest)
                  {
                    //stat=true;
                    found2=true;
                    std::cout<<"FOUND FINALLY"<<std::endl;
                    break;
                  }
              }
         }
       }
     }
  }
  return found2;
}
int main (int argc, char *argv[])
{
  
  std::string phyMode ("DsssRate1Mbps");
  std::string traceFile;
  Simulator::Run ();
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
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=5]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=0.2]")); 
  std::vector< int> left;
  int pd=0;
  bool found2=false;
  //std::vector<int> desti{2,191,86,18,168,81,98,5,195};
   Ptr<ListPositionAllocator> positionAlloc2 = CreateObject<ListPositionAllocator> ();
  for(uint32_t j=0;j<=19;j=j+1)
 {
  for(uint32_t k=0;k<=44;k=k+1)
   {
     positionAlloc2->Add (Vector (j*5+1, k*2+1, 0.0));
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
 ipv4_source2.SetBase ("20.1.0.0", "255.255.0.0");
 Ipv4InterfaceContainer i_source2 = ipv4_source2.Assign (vehicle_devices);
 Ipv4AddressHelper ipv4_source;
 NS_LOG_INFO ("Assign IP Addresses.");
 ipv4_source.SetBase ("10.1.1.0", "255.255.255.0");
 Ipv4InterfaceContainer i_source = ipv4_source.Assign (rsu_devices);
// std::vector<int> res;

 double sim_time=Simulator::Now ().GetSeconds ()+.3;
// uint32_t pd=0;
std::set<int>:: iterator it;//int l=0;                          
TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
TypeId tid2 = TypeId::LookupByName ("ns3::UdpSocketFactory");
//for(uint32_t k=0;k<200;k++)
uint32_t dest;        
uint32_t pac_drop=0;
uint32_t pac_del=0;
 uint32_t k;
 

 uint32_t count=0;
 int arr[]={0,22,43,450,472,494,856,878,898};
// sleep(10);
// double tym=Simulator::Now ().GetSeconds();
 //  while(tym<0.05)
 // {
 //   tym=Simulator::Now ().GetSeconds();
 // }
for(k=0;k<numNodes2;k++)
{
    found=false;
    found2=false;
    //bool stat=false;
    for(uint32_t st=0;st<sizeof(arr)/sizeof(arr[0]) ;st++)
    {
      
    //   int dRan;//,sRan
    //   std::srand( time(0)); // This will ensure a really randomized number by help of time.

    // // // //sRan=std::rand()%200;
    //   dRan=std::rand()%numNodes2;
      dest=arr[st];
      // dest=dRan;
      if(dest==k)
        break;
      count++;
    // k=sRan;
    std::cout<<"Source :"<<k<<"Dest:"<<dest<<std::endl;
    left.clear();
    nodecnt.clear();
    nodecnt2.clear();
    for(uint32_t j=0;j<numNodes2;j++)
    {
      if(j!=k)
      nodecnt.insert(j);
      nodecnt2.insert(j);
    // nodecnt2.insert(j);
    }
    left.push_back(k);
    left.push_back(50000);
    if(frontpath(left,nodecnt,ttl,sim_time,i_source2,dest))
    {
        std::cout<<"RSU Number"<<rsu_nmbr<<std::endl;
        //src_status.insert(std::make_pair(k,1));
        std::vector< uint32_t> find_dest[ttl];
        if(found==false)
            {
             /* Ptr<Socket> rsu_source = Socket::CreateSocket (rsu.Get (rsu_nmbr), tid2);
              InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
              //res.push_back(rsu_nmbr);
              rsu_source->SetAllowBroadcast (true);
              rsu_source->Connect (remote);
              rsu_time=Simulator::Now ().GetSeconds ()+sim_time+.3;
              Simulator::ScheduleWithContext (rsu_source->GetNode ()->GetId (),
                               Seconds (rsu_time), &GenerateTraffic,
                               rsu_source, packetSize, numPackets, interPacketInterval);
              */
              found2=Call_rsu(nodecnt2,find_dest,dest);
              if(found2==true)
              {
                 pacDel++;
                  src_status.insert(std::make_pair(k,1));
              }
              else
              {
                 src_status.insert(std::make_pair(k,0));
              }
            }
            else
            {
              pacDel++;
               src_status.insert(std::make_pair(k,1));
              std::cout<<"Dest found without RSU"<<std::endl;
            }

    }
     else
     {
      std::cout<<"NOT Found any RSU"<<std::endl;
      src_status.insert(std::make_pair(k,0));
      pd++;
            //std::cout<<"Packet drop"<<std::endl;
     }
  }
  pac_drop+=(numNodes2-pacDel);
  pd=0;
  pac_del+=pacDel;
  pacDel=0;
}

AnimationInterface anim("Atruth1.xml");
for(auto it=src_status.begin();it!=src_status.end();it++)
{
  std::cout<<it->first<<"  "<<it->second<<std::endl;
}
std::cout<<"packet drop number: "<<(count-pac_del)<<"Packets Delivered:"<<pac_del<<" "<<count<<std::endl;
 Simulator::Stop(MilliSeconds(2000));
 
 Simulator::Destroy();
}
