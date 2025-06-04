#pragma once
#include <Eigen/Core>

#include <opencv4/opencv2/core/types.hpp>




namespace ReUseX
{

    constexpr double PI = 3.14159265358979323846264338327950288419716939937510582097494459230781640628;
    constexpr double PHI = 1.61803398874989484820458683436563811772030917980576286213544862270526046281;  // Define golden ratio

    class Color: public cv::Scalar
    {

        using cv::Scalar::Scalar;

        private:

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

        public:

            /**
             * @brief Maps an angle in radians to a color.
             * 
             * @param radians The angle in radians.
             * @return Color The resulting color in HSL.
             */
            static Color from_angle(float radians) {
                float pi_2 = 2.0 * PI;
                float val = std::fmod(radians, pi_2) / pi_2;

                // Convert HSL to RGB using a simple color utility.
                // Placeholder function. Replace `hsl_to_rgb` with actual implementation.
                return hsl_to_rgb(val, 1.0f, 0.5f); // Assuming `hsl_to_rgb` exists.
            }

            /**
             * @brief Maps an index to a color, by sampling a circle in HSL space with a golden ratio.
             * 
             * @param index The index of the color.
             * @return Color The resulting color in RGB.
             */
            static Color from_index(int index) {

                // Generates a modular sample point on a circle based on the index.
                // and returns the position on the circle's parameterization.
                auto radians = std::fmod(index * PHI * PI, 2 * PI);
                return from_angle(radians);

            }
    };
} // namespace ReUseX