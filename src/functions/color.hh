#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/enum.h>

#include <opencv2/core/matx.hpp>
#include <opencv4/opencv2/opencv.hpp>




#define PHI 1.618033988749894
#define PI 3.14159265358979323846

namespace ReUseX {

    using Kernel =  CGAL::Exact_predicates_inexact_constructions_kernel;    
    using FT = Kernel::FT;
    // using Color = cv::Vec3b;


    /**
     * @brief Generates a modular sample point on a circle based on the index.
     * 
     * @param i The index of the sample point.
     * @param radius The radius of the circle.
     * @return float The position on the circle's parameterization.
     */
    static float sample_circle(int i){
        return std::fmod(i * PHI * PI, 2 * PI);
    }

    /**
     * @brief Converts HSL to RGB.
     * 
     * @param h Hue value (0.0 - 1.0).
     * @param s Saturation value (0.0 - 1.0).
     * @param l Lightness value (0.0 - 1.0).
     * @return Color RGB color vector.
     */
    static cv::Scalar hsl_to_rgb(float h, float s, float l) {
        auto hue_to_rgb = [](float p, float q, float t) -> float {
            if (t < 0.0f) t += 1.0f;
            if (t > 1.0f) t -= 1.0f;
            if (t < 1.0f / 6.0f) return p + (q - p) * 6.0f * t;
            if (t < 1.0f / 2.0f) return q;
            if (t < 2.0f / 3.0f) return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;
            return p;
        };

        float q = (l < 0.5f) ? (l * (1.0f + s)) : (l + s - l * s);
        float p = 2.0f * l - q;

        float r = hue_to_rgb(p, q, h + 1.0f / 3.0f);
        float g = hue_to_rgb(p, q, h);
        float b = hue_to_rgb(p, q, h - 1.0f / 3.0f);

        return cv::Scalar(r * 255, g * 255,b * 255);
    }

    /**
     * @brief Maps an angle in radians to a color.
     * 
     * @param radians The angle in radians.
     * @return Color The resulting color in HSL.
     */
    static cv::Scalar get_color_from_angle(float radians) {
        float pi_2 = 2.0f * PI;
        float val = std::fmod(radians, pi_2) / pi_2;

        // Convert HSL to RGB using a simple color utility.
        // Placeholder function. Replace `hsl_to_rgb` with actual implementation.
        return hsl_to_rgb(val, 1.0f, 0.5f); // Assuming `hsl_to_rgb` exists.
    }

}
