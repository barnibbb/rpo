#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include "dose_calculator.h"

using namespace std;

namespace rpo
{
    struct LampModelElement { int x, y, z; bool color; };

    class ROSVisualizer : public DoseCalculator
    {
    public:
        ROSVisualizer(const AugmentedModel& augmented_model);

        void readLampModel();
        void placeLamp(double x, double y);
        void deleteLamps();  

        void showResult(RadiationPlan& radiation_plan);

        void setModelColor(const ExposureMap& dose_map, bool viridis);

        void publish();

    private:
        ros::NodeHandle m_node_handle;
        ros::Publisher m_model_publisher;

        vector<LampModelElement> m_lamp_model;
        
        KeySet m_placed_lamps; 
    };
}
