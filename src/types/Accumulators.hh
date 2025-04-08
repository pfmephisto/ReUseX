#pragma once
#include <vector>
#include <functional>
#include <pcl/common/impl/accumulators.hpp>


namespace ReUseX {


    /* Meta-function that checks if an accumulator is compatible with given
     * point type(s). */
    template <typename Point1T, typename Point2T = Point1T>
    struct IsAccumulatorCompatible {

      template <typename AccumulatorT>
      struct apply : boost::mpl::and_<
                       boost::mpl::apply<typename AccumulatorT::IsCompatible, Point1T>,
                       boost::mpl::apply<typename AccumulatorT::IsCompatible, Point2T>
                     > {};
    };

    /* Meta-function that creates a Fusion vector of accumulator types that are
     * compatible with a given point type. */
    template <typename PointT>
    struct Accumulators
    {
      using type =
        typename boost::fusion::result_of::as_vector<
          typename boost::mpl::filter_view<
            boost::mpl::vector<
              pcl::detail::AccumulatorXYZ
            , pcl::detail::AccumulatorNormal
            , pcl::detail::AccumulatorCurvature
            , pcl::detail::AccumulatorRGBA
            , pcl::detail::AccumulatorIntensity
            , pcl::detail::AccumulatorLabel
            >
          , IsAccumulatorCompatible<PointT>
          >
        >::type;
    };

    // /* Fusion function object to invoke point addition on every accumulator in
    //  * a fusion sequence. */
    // template <typename PointT>
    // struct AddPoint
    // {

    //   const PointT& p;

    //   AddPoint (const PointT& point) : p (point) { }

    //   template <typename AccumulatorT> void
    //   operator () (AccumulatorT& accumulator) const
    //   {
    //     accumulator.add (p);
    //   }

    // };

    // /* Fusion function object to invoke get point on every accumulator in a
    //  * fusion sequence. */
    // template <typename PointT>
    // struct GetPoint
    // {

    //   PointT& p;
    //   std::size_t n;

    //   GetPoint (PointT& point, std::size_t num) : p (point), n (num) { }

    //   template <typename AccumulatorT> void
    //   operator () (AccumulatorT& accumulator) const
    //   {
    //     accumulator.get (p, n);
    //   }

    // };
} // namespace ReUseX