#include "modules/auto_buff/models/model.hpp"

#include "modules/auto_buff/models/shenzhenbuff.hpp"

namespace rm::buff
{

// 新模型在这里登记即可,Detector 无需改动。
const ModelSpec *find_model_spec(const std::string &filename)
{
    if (is_shenzhenbuff(filename))
    {
        return &shenzhenbuff_spec();
    }
    return nullptr;
}

} // namespace rm::buff
