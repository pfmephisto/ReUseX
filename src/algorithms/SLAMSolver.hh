#pragma once


#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include <g2o/types/slam3d/types_slam3d.h>
#include <Eigen/Core>
#include <iostream>
#include <algorithm>

namespace ReUseX
{
    class SLAMSolver
    {
        private:

        // Create the optimizer
        std::unique_ptr<g2o::SparseOptimizer> optimizer;

        public:
        SLAMSolver(){
            

            // Construct the graph and set up the solver and optimiser
            std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver =
                std::make_unique<
                    g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();

            // Set up the solver
            std::unique_ptr<g2o::BlockSolverX> blockSolver =
                std::make_unique<g2o::BlockSolverX>(std::move(linearSolver));

                    
            // Set up the optimisation algorithm
            g2o::OptimizationAlgorithm* optimisationAlgorithm =
                new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

            // Create the graph and configure it
            optimizer = std::make_unique<g2o::SparseOptimizer>();
            optimizer->setVerbose(true);
            optimizer->setAlgorithm(optimisationAlgorithm);

        };

        ~SLAMSolver(){
            optimizer.clear();
        };

        template <typename PointT, typename PoseT>
        void add_vertices(PointT* point, PoseT* pose, size_t n){


            for (size_t idx = 0; idx < n; idx++){


                auto vertex = static_cast<g2o::VertexSE3*>(point+idx);
                auto pose = static_cast<Eigen::Isometry3d*>(pose+idx);

                // // Add vertices (SE3 poses)
                // auto* vertex = new g2o::VertexSE3();

                vertex->setId(idx);
                if (idx == 0) {
                    vertex->setFixed(true);  // Fix the first pose as the origin
                }

                // Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
                // pose.translate(Eigen::Vector3d(i, 0, 0));  // Example: Moving along the X axis


                vertex->setEstimate(pose);
                optimizer.addVertex(vertex);

            }


            // size_t idx = 0;
            // std::for_each(begin, end,[&](auto point){

            //     idx++;
            // });
        };

        template <typename MeasurementT, typename Container>
        void add_edges(MeasurementT* data, Container* set,  size_t n){

            // Add edges (odometry constraints)
            for (int idx = 1; idx < n; ++idx) {

                auto* edge = new g2o::EdgeSE3();
                edge->setId(idx);
                
                edge->setVertex(0, optimizer.vertex((set+idx)*[1]));  // From pose i-1
                edge->setVertex(1, optimizer.vertex((set+idx)*[0]));      // To pose i

                auto measurement = static_cast<Eigen::Isometry3d*>(data+idx);
                // Eigen::Isometry3d measurement = Eigen::Isometry3d::Identity();
                // measurement.translate(Eigen::Vector3d(1, 0, 0));  // Example: 1m along X axis

                edge->setMeasurement(measurement);
                edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity());  // Covariance
                optimizer.addEdge(edge);
            }

        };

        void optimize(){
            optimizer.initializeOptimization();
            optimizer.optimize(10);
        };


        size_t size(){
            return optimizer.vertices().size();
        };

        template <typename T>
        T [size_t idx]()
        {
            auto* vertex = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(i));
            if (vertex == nullptr) {
                throw std::runtime_error("Vertex is not of type g2o::VertexSE3");
            }

            return static_cast<T*>(vertex->estimate().matrix());
        };
    };
} // namespace ReUseX
