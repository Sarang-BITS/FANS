#include "planner_ns3_utils.h"


void
rnl::setPosition (ns3::Ptr<ns3::Node> node, ns3::Vector3D position)
{
  ns3::Ptr<ns3::MobilityModel> mobility = node->GetObject<ns3::MobilityModel> ();
  mobility->SetPosition (position);
}


ns3::Vector3D
rnl::getPosition (ns3::Ptr<ns3::Node> node)
{
  ns3::Ptr<ns3::MobilityModel> mobility = node->GetObject<ns3::MobilityModel> ();
  return mobility->GetPosition ();
}


bool
rnl::getTrajectory
    (
        std::vector<ns3::Vector3D>* wpts,
        ns3::Vector3D start_pos, 
        ns3::Vector3D end_pos,
        double step 
    )
{
    try
    {
        
        wpts -> clear();
        ns3::Vector3D unit_vec = end_pos - start_pos;

        /*Unit Vector in the direction*/
        double vec_len = unit_vec.GetLength();
        
        if (vec_len > 10000.0)
            throw std::range_error("getTrajectory Failed. Goal too Far");

        if (std::isnan(start_pos.x) || std::isnan(start_pos.y) || std::isnan(start_pos.z)
            || std::isnan(end_pos.x) || std::isnan(end_pos.y) || std::isnan(end_pos.z))
        {
            throw std::range_error ("getTrajectory Failed. Position is nan");
        }

        if (!vec_len){
            wpts -> push_back(start_pos);
            return true;
        }
        
        unit_vec.x = unit_vec.x/vec_len;
        unit_vec.y = unit_vec.y/vec_len;
        unit_vec.z = unit_vec.z/vec_len;

        if ( (int)vec_len/step > 1000)
        {
            throw std::range_error ("getTrajectory Failed. Size of Vector too big:" + std::to_string(vec_len/step));
        }
        
        for (int i = 1; i < (int)vec_len/step; ++i){
            wpts -> push_back(ns3::Vector3D(
                            start_pos.x + step * i * unit_vec.x,
                            start_pos.y + step * i * unit_vec.y,
                            start_pos.z + step * i * unit_vec.z)
                        );
            if (wpts->size() > 1000)
            {
                throw std::range_error ("Out of Range");
            }
        }
        
        wpts -> push_back(end_pos);

        return true;
    }

    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}

bool
rnl::getTrajectoryContinue
    (
        std::vector<ns3::Vector3D>* wpts,
        ns3::Vector3D start_pos, 
        ns3::Vector3D end_pos,
        double step 
    )
{
    try
    {
        ns3::Vector3D unit_vec = end_pos - start_pos;

        /*Unit Vector in the direction*/
        double vec_len = unit_vec.GetLength();
        
        if (vec_len > 10000.0)
            throw std::range_error("Out of range");

        if (std::isnan(start_pos.x) || std::isnan(start_pos.y) || std::isnan(start_pos.z)
            || std::isnan(end_pos.x) || std::isnan(end_pos.y) || std::isnan(end_pos.z))
        {
            throw std::range_error ("Out of Range");
        }

        if (!vec_len){
            wpts -> push_back(start_pos);
            return true;
        }
        
        unit_vec.x = unit_vec.x/vec_len;
        unit_vec.y = unit_vec.y/vec_len;
        unit_vec.z = unit_vec.z/vec_len;
        
        for (int i = 1; i < (int)vec_len/step; ++i){
            wpts -> push_back(ns3::Vector3D(
                            start_pos.x + step * i * unit_vec.x,
                            start_pos.y + step * i * unit_vec.y,
                            start_pos.z + step * i * unit_vec.z)
                        );
            if (wpts->size() > 1000)
            {
                throw std::range_error ("Out of Range");
            }
        }
        
        wpts -> push_back(end_pos);

        return true;
    }

    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}

float
rnl::circlingOffset
    (
        ns3::Vector3D anch,
        ns3::Vector3D child,
        float         _cr
    )
{
    float _f = ns3::CalculateDistance (anch, child) - _cr;
    return _f;
}


bool 
rnl::getToCircleRange (std::vector<ns3::Vector3D>* wpts,
                        ns3::Vector3D              _anch_p,
                        ns3::Vector3D              _my_p,
                        float                      cr,
                        float                      step
                    )
{
    ns3::Vector3D unit_vec = (_anch_p - _my_p);

    double vec_len = unit_vec.GetLength();

    unit_vec.x = unit_vec.x / vec_len;
    unit_vec.y = unit_vec.y / vec_len;
    unit_vec.z = unit_vec.z / vec_len;
    
    float d = rnl::circlingOffset (_anch_p, _my_p, cr);
    
    ns3::Vector3D _goal_pos (_my_p.x + d * unit_vec.x, _my_p.y + d * unit_vec.y, _my_p.z + d * unit_vec.z);
    bool res = rnl::getTrajectory (wpts, _my_p, _goal_pos, step);
    return res;
}

bool
rnl::getCirclewpts
            (
                std::vector<ns3::Vector3D>* wpts,
                ns3::Vector3D anch_node,
                ns3::Vector3D child_node,
                const float         circle_radius,
                const float         dtheta,
                const int           dir,
                const float         step
            )
{
    ns3::Vector3D _anch_p = (anch_node);
    ns3::Vector3D _my_p   = (child_node);
    
    try
    {
        rnl::getToCircleRange(wpts, _anch_p, _my_p, circle_radius, step);
        
        float theta = atan2 (_my_p.y - _anch_p.y, 
                            _my_p.x - _anch_p.x);
        float theta_it = theta;
        // while (abs(theta_it - theta) < 2 * M_PI)
        while (abs(theta_it - theta) < 1.5*M_PI/2)
        {
            theta_it += dir * dtheta;
            wpts -> push_back (ns3::Vector3D(
                                _anch_p.x + circle_radius * cos(theta_it),
                                _anch_p.y + circle_radius * sin(theta_it),
                                _anch_p.z
                                ));
        }
        return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    
}

void rnl::posHold
(
    std::vector<ns3::Vector3D>* wpts,
    ns3::Vector3D       pos
)

{
    wpts->push_back (pos);
    wpts->clear();
    wpts->push_back (pos);
}
