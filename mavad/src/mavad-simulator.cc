#include "ns3/core-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("MavadSimulator");

int 
main (int argc, char *argv[])
{
  NS_LOG_UNCOND ("Mavad Simulator");

  Simulator::Run ();
  Simulator::Destroy ();
}
