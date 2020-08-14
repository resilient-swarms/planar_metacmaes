#ifndef PLANAR_DART_SAFETY_MEASURES_HPP
#define PLANAR_DART_SAFETY_MEASURES_HPP

namespace planar_dart {

    namespace safety_measures {

        struct LinkColliding {
        public:
            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                const dart::collision::CollisionResult& col_res = simu.world()->getLastCollisionResult();
                auto body = rob->skeleton()->getRootBodyNode();

                if (col_res.inCollision(body)){
                    //std::cout<<"Stopping for body collision" <<std::endl;
                    simu.stop_sim();
                }
            }
        };
    }
}

#endif
