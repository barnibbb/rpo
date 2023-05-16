#include <vector>
#include <memory>

#include "augmented_model.h"

using namespace std;

namespace rpo
{
    class PlanGenerator
    {
    public:
        PlanGenerator(const AugmentedModel& augmented_model);

        void createInitialPopulation();
        void addNewPosition();

    private:
        shared_ptr<AugmentedModel> m_augmented_model = nullptr;
    };
}