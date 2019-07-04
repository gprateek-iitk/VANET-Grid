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
#include<chrono>

NS_LOG_COMPONENT_DEFINE ("Truth1");
using namespace ns3;
//using namespace std;
uint16_t rsu_nmbr=0;
//bool found2=false;
bool found=false;
bool rsu_found=false;
bool dest_found=false;
int pac_Dis=0;
int pac_Dat=0;
float pkt=0.4;
uint16_t maxDistance = 11; // m //最大传播距离
uint32_t packetSize  = 1024; // bytes
uint16_t beaconPort  = 80;
uint32_t numNodes    = 5;
uint32_t numNodes2    = 15;
uint32_t numPackets  = 1;
double interval = 1.0;
double rsu_time=0;
uint32_t level=10;
std::vector<double> e2e;
//uint32_t brk=50000;
uint32_t pacDel=0;
int xt=1;
std::map<int, int> src_status;
std::map<int, int> fin_status;
std::vector<float> pdr_set;
std::vector<float> traffic_set;
 Time interPacketInterval = Seconds (interval);
bool verbose=false;
//uint32_t dest;
std::set< int> nodecnt;
std::set<int> nodecnt2;
 uint32_t ttl=5;
std::vector< int> right;
NodeContainer rsu,vehicles;
double stnd=190;
double avg_fr_traffic=0,avg_bk_traffic=0;
void pseudo(int a);
bool frontpath(std::set< int> &nodecnt,uint32_t src,uint32_t ttl,uint32_t dest);
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
bool frontpath(std::set< int> &nodecnt,uint32_t src,uint32_t ttl,uint32_t dest)
{
   // std::set<int>:: iterator it;
    //int count=0;
    Ptr<MobilityModel> mob2=vehicles.Get(src)->GetObject<MobilityModel>();
    // TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    // Ptr<Socket> recvSink = Socket::CreateSocket (vehicles.Get (dest), tid);
    // InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
    // recvSink->Bind (local);
    // recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
    
    //   ns3::Ptr<Socket> source = Socket::CreateSocket (vehicles.Get (left[0]), tid);
    //   ns3::InetSocketAddress remote = InetSocketAddress (i_source2.GetAddress (dest, 0), 80);//agar phncha iss address pe toh revert krta hai
    //    source->Connect (remote);
    // Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
    //                                Seconds (sim_time), &GenerateTraffic,
    //                                source, packetSize, numPackets, interPacketInterval);
    
    //std::cout<<"Paari start: "<<(16+left[0])<<std::endl;
  // uint32_t initsize=left.size();
   std::vector<uint32_t> find_rsu[ttl];
   for(uint32_t j=0;j<numNodes;j++)
   {
     Ptr<MobilityModel> mob3=rsu.Get(j)->GetObject<MobilityModel>();
     double dis2=mob3->GetDistanceFrom(mob2);
    // std::cout<<dis2<<std::endl;
       if(dis2<=stnd)
       {
        //std::cout<<"Yha aaya"<<std::endl;
        rsu_nmbr=j;
        //std::cout<<"Rsu no."<<j<<"sim_time"<<sim_time<<std::endl;
        rsu_time=Simulator::Now ().GetSeconds ()+.2;
       // std::cout<<"rsu_time "<<rsu_time<<std::endl;
        rsu_found =true;
       }
   }
   for(uint32_t y=0;y<numNodes2;y++)
   {
     Ptr<MobilityModel> mob=vehicles.Get(y)->GetObject<MobilityModel>();
     double dis2=mob->GetDistanceFrom(mob2);
     if(dis2<=stnd)
     {
       if(y==dest)
        dest_found=true;
       find_rsu[0].push_back(y);
        // std::cout<<"present"<<std::endl;
       if(nodecnt.find(y)!=nodecnt.end())
       nodecnt.erase(nodecnt.find(y));
       pac_Dis++;
     }

   } 
   for(uint32_t h=1;h<ttl;h++)
   {

       for(uint32_t k=0;k<numNodes2;k++)
       {
            Ptr<MobilityModel> mob2=vehicles.Get(k)->GetObject<MobilityModel>();
            for(uint32_t l=0;l<find_rsu[h-1].size();l++)
            {
              Ptr<MobilityModel> mob=vehicles.Get(find_rsu[h-1][l])->GetObject<MobilityModel>();
              if(rsu_found==false)
              {
                for(uint32_t g=0;g<numNodes;g++)
                {
                 Ptr<MobilityModel> mob3=rsu.Get(g)->GetObject<MobilityModel>();
                 double dis2=mob3->GetDistanceFrom(mob);
                 if(dis2<=stnd)
                  {
                    rsu_found=true;
                    rsu_nmbr=g;
                  }
                }
              }
              if(nodecnt.find(k)!=nodecnt.end())
              {                
                 double dis2=mob->GetDistanceFrom(mob2);
                 if(dis2<=stnd)
                 {
                  if(k==dest)
                    dest_found=true;
                  find_rsu[h].push_back(k);
                  if(nodecnt.find(k)!=nodecnt.end())
                  nodecnt.erase(nodecnt.find(k));
                   pac_Dis++; 
                 }
              }
           }
        }
    } 
    return rsu_found;
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
     for(uint16_t k=0;k<numNodes;k++)
     {
      Ptr<MobilityModel> mob2=rsu.Get(k)->GetObject<MobilityModel>();
      double dis1=mob2->GetDistanceFrom(mob);
      if(dis1<=stnd)
       {
        pac_Dat++;
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
    if(found2==true)
        break;
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
              pac_Dat++;
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
       if(found2==true)
        break;
     }
  }
  return found2;
}
void pseudo(int a)
{
     int pd=0;
    bool found2=false;
    uint32_t dest;        
uint32_t pac_drop=0;
uint32_t pac_del=0;
// float x=0,y=0;
auto startt = std::chrono::high_resolution_clock::now();

 // uint32_t k=17;
 // std::cout<<"presnet"<<std::endl;
 // Ptr<MobilityModel> mob=vehicles.Get(2)->GetObject<MobilityModel>();
 // Vector pos=mob.GetPosition();
 // std::cout<<pos.x<<" "<<pos.y<<std::endl;
 uint32_t count=0;
 // int arr[]={50,74,98,50,54,54,90,92,99};

 int arr[numNodes2];
 for(uint32_t i=0;i<numNodes2;i++)
 {
  arr[i]=i;
 }

 std::vector<int> front_traffic;
 std::vector<int> back_traffic;
  for(uint32_t k=0;k<numNodes2;k++)
  {
    for(uint32_t st=0;st<sizeof(arr)/sizeof(arr[0]) ;st++)
    {
      
     rsu_found=false;
     dest_found=false;

    //   int dRan;//,sRan
    //   std::srand( time(0)); // This will ensure a really randomized number by help of time.

    // // // //sRan=std::rand()%200;
    //   dRan=std::rand()%numNodes2;
      dest=arr[st];
      // dest=dRan;
      if(dest==k)
        continue;
      count++;
    // k=sRan;
    std::cout<<"Source :"<<k<<"Dest:"<<dest<<std::endl;
    // left.clear();
    nodecnt.clear();
    nodecnt2.clear();
    for(uint32_t j=0;j<numNodes2;j++)
    {
      if(j!=k)
      nodecnt.insert(j);
      nodecnt2.insert(j);
    // nodecnt2.insert(j);
    }
   
    if(frontpath(nodecnt,k,ttl,dest))
    {

        std::cout<<"RSU Number"<<rsu_nmbr<<std::endl;
        //src_status.insert(std::make_pair(k,1));
        std::vector< uint32_t> find_dest[ttl];
        if(dest_found==false)
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
              // std::cout<<"hello"<<std::endl;
              found2=Call_rsu(nodecnt2,find_dest,dest);
              //std::cout<< found2 <<std::endl;
              if(found2==true)
              {
                 pacDel++;
                  src_status.insert(std::make_pair(arr[st],1));
              }
              else
              {
                 src_status.insert(std::make_pair(arr[st],0));
              }
            }
            else
            {
              pacDel++;
               src_status.insert(std::make_pair(arr[st],1));
              std::cout<<"Dest found without RSU"<<std::endl;
            }
            // for(uint32_t s=0;s<= std::min(level,ttl-1);s++)
            // {
            //   for(uint32_t z=0;z<find_dest[s].size();z++)
            //     pac_Dat++;
            // }

    }
     else
     {
      std::cout<<"NOT Found any RSU"<<std::endl;
      src_status.insert(std::make_pair(arr[st],0));
      pd++;
      if(dest_found)
      pacDel++;
           //std::cout<<"Packet drop"<<std::endl;
     }
     front_traffic.push_back(pac_Dis);
     back_traffic.push_back(pac_Dat);
     pac_Dis=0;
     pac_Dat=0;
  }
  pac_drop+=(numNodes2-pacDel);
  pd=0;
  pac_del+=pacDel;
  pacDel=0;

 }
 auto finish = std::chrono::high_resolution_clock::now();
std::chrono::duration<double> elapsed = finish - startt;
 e2e.push_back(125*(elapsed.count())/count);
 float pdr=100;
 pdr=(pdr*pac_del)/count; 
  for(uint32_t j=0;j<front_traffic.size();j++)
    {
      avg_fr_traffic+=front_traffic[j];
    }
  for(uint32_t j=0;j<back_traffic.size();j++)
    {
      avg_bk_traffic+=back_traffic[j];
    }
avg_fr_traffic/=front_traffic.size();
avg_bk_traffic/=back_traffic.size();
float total=avg_fr_traffic+avg_bk_traffic;
total*=(numNodes2/2.3)*pkt;
traffic_set.push_back(total);
std::cout<<"Front Packet traffic"<<(avg_fr_traffic)<<"Back Packet Traffic"<<(avg_bk_traffic)<<std::endl;

std::cout<<"packet drop number: "<<(count-pac_del)<<"Packets Delivered:"<<pac_del<<" "<<count<<std::endl;
std::cout<<"PDR:"<<pdr<<std::endl;
pdr_set.push_back(pdr);
}
int main (int argc, char *argv[])
{
  //std::cout<<"Start Tym:"<<Simulator::Now ().GetSeconds ()<<std::endl;
  double tt=Simulator::Now ().GetSeconds ();
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

  auto startt = std::chrono::high_resolution_clock::now();
   
 
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
  // NetDeviceContainer rsu_devices = wifi.Install (wifiPhy, wifiMac, rsu);
  // NetDeviceContainer vehicle_devices=wifi.Install(wifiPhy, wifiMac, vehicles);
 //  MobilityHelper mobility;
 //  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
 //  "MinX", DoubleValue (0.0),
 //  "MinY", DoubleValue (0.0),
 //  "DeltaX", DoubleValue (5.0),
 //  "DeltaY", DoubleValue (5.0),
 //  "GridWidth", UintegerValue (3),
 //  "LayoutType", StringValue ("RowFirst")); 
   
 // /*  mobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
 //                             "Bounds", RectangleValue (Rectangle (0, 100, 0, 100)),
 //                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=20]"),
 //                                "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=0.2]")); 
 //  */
 //  MobilityHelper mobility2;
 // mobility2.SetPositionAllocator ("ns3::GridPositionAllocator",
 // "MinX", DoubleValue (0.0),
 // "MinY", DoubleValue (0.0),
 // "DeltaX", DoubleValue (5.0),
 // "DeltaY", DoubleValue (5.0),
 // "GridWidth", UintegerValue (3),
 // "LayoutType", StringValue ("RowFirst")); 
   
 //  mobility2.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
 //                             "Bounds", RectangleValue (Rectangle (0, 100, 0, 100)),
 //                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=5]"),
 //                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=0.2]")); 
  // std::vector< int> left;
  
  // if(numNodes2>30)
  //   ttl=3;
  //std::vector<int> desti{2,191,86,18,168,81,98,5,195};
 //   Ptr<ListPositionAllocator> positionAlloc2 = CreateObject<ListPositionAllocator> ();
 //  for(uint32_t j=0;j<=9;j=j+1)
 // {
 //  for(uint32_t k=0;k<=19;k=k+1)
 //   {
 //     positionAlloc2->Add (Vector (j*11, k*5, 0.0));
 //   }
 // }
  // mobility2.SetPositionAllocator (positionAlloc2);
  Ns2MobilityHelper ns2 = Ns2MobilityHelper ("/home/prateek/Downloads/guindy/into/delhi_mobility.tcl");
 ns2.Install();
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
  "MinX", DoubleValue (0.0),
  "MinY", DoubleValue (0.0),
  "DeltaX", DoubleValue (5.0),
  "DeltaY", DoubleValue (10.0),
  "GridWidth", UintegerValue (3),
  "LayoutType", StringValue ("RowFirst"));

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  rsu.Create (numNodes);
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (175,225,0));
  positionAlloc->Add (Vector (775,225, 0.0));
  positionAlloc->Add (Vector (500,354, 0.0));
  positionAlloc->Add (Vector (175, 650,0.0));
  positionAlloc->Add (Vector (775,650,0.0));
  // positionAlloc->Add (Vector (275,325,0.0));
  // positionAlloc->Add (Vector (675,325,0.0));
  // positionAlloc->Add (Vector (275,550,0.0));
  // positionAlloc->Add (Vector (675,550,0.0));
   mobility.SetPositionAllocator (positionAlloc);
   mobility.Install (rsu);
   // Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
 //    for(uint32_t j=0;j<=6;j=j+2)
 // {
 //  for(uint32_t k=0;k<=6;k=k+2)
 //   {
     // positionAlloc->Add (Vector (j*16+1, k*16+1, 0.0));
 //   }
 // }
 // positionAlloc->Add (Vector (50,50, 0.0));
 //  mobility.SetPositionAllocator (positionAlloc);
 // mobility.Install (rsu);
 // InternetStackHelper internet;
 // internet.Install (rsu);
 
 // mobility2.Install (vehicles);
 InternetStackHelper internet2;
 internet2.Install (vehicles);
 Ipv4AddressHelper ipv4_source2;
 ipv4_source2.SetBase ("20.1.0.0", "255.255.0.0");
 // Ipv4InterfaceContainer i_source2 = ipv4_source2.Assign (vehicle_devices);
 Ipv4AddressHelper ipv4_source;
 NS_LOG_INFO ("Assign IP Addresses.");
 ipv4_source.SetBase ("10.1.1.0", "255.255.255.0");
 // Ipv4InterfaceContainer i_source = ipv4_source.Assign (rsu_devices);
// std::vector<int> res;

 //double sim_time=Simulator::Now ().GetSeconds ()+.3;
// uint32_t pd=0;
std::set<int>:: iterator it;//int l=0;                          
TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
TypeId tid2 = TypeId::LookupByName ("ns3::UdpSocketFactory");
//for(uint32_t k=0;k<200;k++)

AnimationInterface anim("Atruth1.xml");
// for(auto itr=src_status.begin();itr!=src_status.end();itr++)
// {
//   std::cout<<itr->first<<"  "<<itr->second<<std::endl;
// }
 
for(uint32_t tym=5;tym<100;tym+=10)
{
ns3::Simulator::Schedule(Seconds(tym), &pseudo,5 );
}
std::cout<<Simulator::Now().GetSeconds ()<<std::endl;

// Simulator::Destroy ();
auto finish = std::chrono::high_resolution_clock::now();
std::chrono::duration<double> elapsed = finish - startt;
std::cout << "Elapsed time: " << elapsed.count() << std::endl;
tt=Simulator::Now ().GetSeconds ()-tt;
 Simulator::Stop(MilliSeconds(2000000));
 Simulator::Run ();
 Simulator::Destroy();

 float avg=0;
 for(uint32_t j=0;j<pdr_set.size();j++)
 {
  std::cout<<pdr_set[j]<<std::endl;
  avg+=pdr_set[j];
 }
 avg=avg/(pdr_set.size());
 std::cout<<"Avg PDR:"<<avg<<std::endl;
 avg=0;
  for(uint32_t j=0;j<e2e.size();j++)
 {
  // std::cout<<pdr_set[j]<<std::endl;
  avg+=e2e[j];
 }
 avg=avg/(e2e.size());
 std::cout<<"Avg E2E:"<<avg<<std::endl;
 avg=0;
 for(uint32_t j=0;j<traffic_set.size();j++)
 {
  // std::cout<<pdr_set[j]<<std::endl;
  avg+=traffic_set[j];
 }
 avg=avg/(traffic_set.size());
 std::cout<<"Avg Packet traffic:"<<avg<<std::endl;
 return 0;
}


