#pragma once

#include <filesystem>

namespace fs =  std::filesystem;

namespace ReUseX
{
    int slam(fs::path const & folder);
} // namespace ReUseX
