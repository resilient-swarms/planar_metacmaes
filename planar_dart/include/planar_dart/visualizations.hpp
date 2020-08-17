#ifndef PLANAR_DART_VISUALIZATIONS_HPP
#define PLANAR_DART_VISUALIZATIONS_HPP

#include <Eigen/Core>

namespace planar_dart {
    namespace visualizations {

        struct TargetArrow {
        public:
            TargetArrow(){}

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
            }
        protected:
        };
        /*struct TargetBin {
        public:
            TargetArrow() : init(false) {}

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                if(!init){
                    _bin = std::shared_ptr<dart::dynamics::CylinderShape>(new dart::dynamics::CylinderShape(
                        0.1, 0.1
                    ));
                }
            }
        protected:
            std::shared_ptr<dart::dynamics::CylinderShape> _bin;
            bool init;
        }   */ 
    }
}

#endif
