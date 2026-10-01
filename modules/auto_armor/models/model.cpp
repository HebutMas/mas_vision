#include "modules/auto_armor/models/model.hpp"

#include "modules/auto_armor/models/shenzhen_model.hpp"

namespace rm::armor
{

// 新模型在这里登记即可,Detector 无需改动。
const ModelSpec *find_model_spec(const std::string &filename)
{
    if (is_shenzhen_model(filename))
    {
        return &shenzhen_model_spec();
    }
    return nullptr;
}

} // namespace rm::armor
