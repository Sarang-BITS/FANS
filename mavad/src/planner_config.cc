#include "planner_config.h"

rnl::USMsg::USMsg (
        int                      id,
        int                      dst,
        const std::string&       n,
        int                      co, 
        int                      st,
        int                      p,
        int                      neighbour_cnt,
        ns3::Vector3D            _ploc
        ): source_id {id}, dst_id {dst}, nbs {n}, control {co},
            state {st}, p_id {p}, p_loc{_ploc}, neigh_cnt{neighbour_cnt}
{
}

rnl::URMsg::URMsg (
        int                      id,
        int                      dst,
        const std::string&       n,
        int                      co, 
        int                      st,
        int                      p,
        int                      neighbour_cnt,
        ns3::Vector3D            _ploc
        ): source_id {id}, dst_id {dst}, nbs {n}, control {co},
            state {st}, p_id {p}, p_loc{_ploc}, neigh_cnt{neighbour_cnt}
{
}

rnl::USMsg::USMsg ()
{
    source_id  = -998;
    dst_id = -998;
    nbs    = "";
    control= -998;
    state  = -998;
    p_id   = -998;
    neigh_cnt = -998;
    p_loc  = ns3::Vector3D (-998,-998,-998);
	bc_nbs = "";
}

rnl::URMsg::URMsg ()
{
    source_id  = -998;
    dst_id = -998;
    nbs    = "";
    control= -998;
    state  = -998;
    p_id   = -998;
    neigh_cnt = -998;
    p_loc  = ns3::Vector3D (-998,-998,-998);
	bc_nbs = "";
}

void rnl::USMsg::serialize (std::string* loc)
{
    std::stringstream _msg;
    _msg << "u" << rnl::DELIM;
    _msg << source_id  << rnl::DELIM
        << dst_id  << rnl::DELIM
        << nbs     << rnl::DELIM
        << control << rnl::DELIM
        << state   << rnl::DELIM
        << p_id    << rnl::DELIM
        << neigh_cnt << rnl::DELIM
        << std::fixed << p_loc //.x << ":" << std::setprecision(5) << p_loc.y << ":" << std::setprecision(5) << p_loc.z
        << '\0';

    *loc = _msg.str ();
}

void rnl::USMsg::serializeBC (std::string* loc, int id, ns3::Vector3D pos)
{
    std::stringstream _msg;
    _msg << "b" << rnl::DELIM;
    _msg << id  << rnl::DELIM
    <<   std::fixed << pos << '\0';
    *loc = _msg.str ();
}

void rnl::URMsg::parseBroadcast (std::string& msg)
{
	std::string _tok;
  std::stringstream _bc_nbs;
  _tok          = msg.substr(0, msg.find(rnl::DELIM));
  _bc_nbs << _tok << rnl::DELIM;
  msg.erase(0, msg.find(rnl::DELIM) + rnl::DELIM.size()); 
    
  _tok          = msg.substr(0, msg.find(rnl::DELIM));
  _bc_nbs << _tok;
	bc_nbs = _bc_nbs.str();
}

void rnl::URMsg::parseUnicast (std::string& msg)
{
	std::string _tok;
  _tok          = msg.substr(0, msg.find(rnl::DELIM));
  source_id     = std::stoi(_tok);
  msg.erase(0, msg.find(rnl::DELIM) + rnl::DELIM.size()); 
  
  _tok          = msg.substr(0, msg.find(rnl::DELIM));
  dst_id        = std::stoi(_tok);
  msg.erase(0, msg.find(rnl::DELIM) + rnl::DELIM.size()); 
  
  _tok          = msg.substr(0, msg.find(rnl::DELIM));
  nbs           = _tok;
  msg.erase(0, msg.find(rnl::DELIM) + rnl::DELIM.size()); 
  
  _tok          = msg.substr(0, msg.find(rnl::DELIM));
  control       = std::stoi(_tok);
  msg.erase(0, msg.find(rnl::DELIM) + rnl::DELIM.size()); 
  
  _tok          = msg.substr(0, msg.find(rnl::DELIM));
  state         = std::stoi(_tok);
  msg.erase(0, msg.find(rnl::DELIM) + rnl::DELIM.size()); 
  
  _tok          = msg.substr(0, msg.find(rnl::DELIM));
  p_id          = std::stoi(_tok);
  msg.erase(0, msg.find(rnl::DELIM) + rnl::DELIM.size()); 

  _tok          = msg.substr(0, msg.find(rnl::DELIM));
  neigh_cnt     = std::stoi(_tok);
  msg.erase(0, msg.find(rnl::DELIM)+  rnl::DELIM.size());
  
  _tok          = msg.substr(0, msg.find(rnl::DELIM));
//   std :: cerr << "planner_config_94_parse__tok: " << _tok << std::endl; 
  p_loc.x       = std::stod(_tok.substr(0, _tok.find(rnl::DELIM_POS)));
  _tok.erase(0, _tok.find(rnl::DELIM_POS) + rnl::DELIM_POS.size());
  p_loc.y       = std::stod(_tok.substr(0, _tok.find(rnl::DELIM_POS)));
  _tok.erase(0, _tok.find(rnl::DELIM_POS) + rnl::DELIM_POS.size());
  p_loc.z       = std::stod(_tok.substr(0, _tok.find(rnl::DELIM_POS)));
  _tok.erase(0, _tok.find(rnl::DELIM_POS) + rnl::DELIM_POS.size());
  msg.erase(0, msg.find(rnl::DELIM) + rnl::DELIM.size()); 
}

void rnl::URMsg::parse (std::string& msg){
  std::string _tok;
  _tok          = msg.substr(0, msg.find(rnl::DELIM));
  msg.erase(0, msg.find(rnl::DELIM) + rnl::DELIM.size()); 
  if (_tok == "b")
  {
    parseBroadcast (msg);
  }

  else {
    parseUnicast (msg);
  }

}

rnl::Nbt::Nbt ()
{
	one_hop = {};
	two_hop = {};
}

void rnl::Nbt::serialize (std::string* dst)
{
    std::stringstream f;    
    for (auto n: one_hop)
      f << n.first << rnl::DELIM_NBTID_POS << n.second.x << rnl::DELIM_NBTPOS << n.second.y << rnl::DELIM_NBTPOS << n.second.z << rnl::DELIM_NBTHOP;
    
    f << rnl::DELIM_NBTMHOP;
    
    for (auto n: two_hop)
        f << n.first << rnl::DELIM_NBTID_POS << n.second.x << rnl::DELIM_NBTPOS << n.second.y << rnl::DELIM_NBTPOS << n.second.z << rnl::DELIM_NBTHOP;
    
    *dst = f.str();
}


void rnl::Nbt::parseSingleNb(std::string msg)
{
	if (msg.size())
	{
		std::string _tok;
		/*Get BC ID*/
		_tok = msg.substr (0, msg.find(rnl::DELIM));
		int _id = std::stoi (_tok);
		msg.erase(0, msg.find(rnl::DELIM) + rnl::DELIM.size());
		
		/*Get Pos*/
		_tok          = msg.substr(0, msg.find(rnl::DELIM));
		ns3::Vector3D temp; 
		temp.x       = std::stod(_tok.substr(0, _tok.find(rnl::DELIM_POS)));
		_tok.erase(0, _tok.find(rnl::DELIM_POS) + rnl::DELIM_POS.size());
		temp.y       = std::stod(_tok.substr(0, _tok.find(rnl::DELIM_POS)));
		_tok.erase(0, _tok.find(rnl::DELIM_POS) + rnl::DELIM_POS.size());
		temp.z       = std::stod(_tok.substr(0, _tok.find(rnl::DELIM_POS)));
		_tok.erase(0, _tok.find(rnl::DELIM_POS) + rnl::DELIM_POS.size());
		msg.erase(0, msg.find(rnl::DELIM) + rnl::DELIM.size());
		
		auto it = std::find_if(one_hop.begin(), one_hop.end(),
		[&_id](const std::pair<int, ns3::Vector3D>& _p){return _p.first  == _id; } ); 

		if (one_hop.size() && it->first == _id)
		{
			it->second = temp;
		}
		else
		{
			one_hop.push_back(std::pair<int, ns3::Vector3D>(_id, temp));
		}
	}
}
