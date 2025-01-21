#pragma once
#include <types/Dataset.hh>
#include <string>
#include <cmath>
#include <optional>

#include <fmt/core.h>
#include <fmt/printf.h>
#include <fmt/format.h>
#include <fmt/color.h>

namespace ReUseX{

    /// @brief Parse a dataset
    /// @param dataset The dataset to parse
    /// @param output_path The path to save the parsed data
    /// @param start The start frame
    /// @param stop The stop frame
    /// @param step The step size
    void parse_Dataset( 
        Dataset const & dataset, 
        std::string const & output_path,
        int start = 0,
        std::optional<int> stop = std::optional<int>{},
        int step = 5
        );
    
}