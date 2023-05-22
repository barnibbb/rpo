#include <unordered_map>
#include <memory>
#include <vector>
#include <optional>

#include <boost/algorithm/string.hpp>

#include "augmented_octree.h"

using namespace std;

namespace rpo
{
    using RadiationPlan = pair<vector<double>, double>;
    using PlanElement = pair<point3d, double>;
    using ExposureMap = unordered_map<OcTreeKey, double, OcTreeKey::KeyHash>;

    class DoseCalculator
    {
    public:
        DoseCalculator(const AugmentedOcTree& augmented_model);

        void computeDoseForPlan(RadiationPlan& radiation_plan);
        ExposureMap computeDoseForPlanElement(const PlanElement& plan_element);
        ExposureMap computeIrradianceForPosition(const point3d& lamp_position);
        double computeIrradianceForElement(const point3d& lamp_position, const OcTreeKey& key) const;

        bool compute2DVisibility() const;
        bool compute3DVisibility(const point3d& lamp_position, const point3d& element, const double distance) const;
    
        ExposureMap loadIrradianceMap(const OcTreeKey& plan_element_key) const;
        void saveIrradianceMap(const OcTreeKey& plan_element_key, const ExposureMap& irradiance_map);
        void preComputeIrradianceMaps();
        void deleteIrradianceMaps() const;

        void computeRayOrigins();

    protected:
        Parameters m_parameters;

        shared_ptr<AugmentedOcTree> m_augmented_model = nullptr;

        vector<double> m_ray_origins;

        KeySet m_radiation_positions;

        // unordered_map<OcTreeKey, ExposureMap, OcTreeKey::KeyHash> m_irradiance_maps;
    };
}
