#ifndef PLANAR_DART_SIMU_HPP
#define PLANAR_DART_SIMU_HPP

#include <boost/parameter.hpp>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/include/accumulate.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/find.hpp>

#include <dart/dart.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/math/Constants.hpp>

#include <Eigen/Core>
#include "../planar_dart/planar.hpp"
#include "../planar_dart/control.hpp>
#include "../planar_dart/safety_measures.hpp"
#include "../planar_dart/descriptors.hpp"
#include <rhex_dart/visualizations.hpp>
#include <cmath>
#include <cstdlib>

#ifdef GRAPHIC
#include <dart/gui/osg/osg.hpp>
#endif

namespace planar_dart {

    BOOST_PARAMETER_TEMPLATE_KEYWORD(planar_control)
    BOOST_PARAMETER_TEMPLATE_KEYWORD(safety)
    BOOST_PARAMETER_TEMPLATE_KEYWORD(desc)
    BOOST_PARAMETER_TEMPLATE_KEYWORD(viz)

    typedef boost::parameter::parameters<boost::parameter::optional<tag::planar_control>,
        boost::parameter::optional<tag::safety>,
        boost::parameter::optional<tag::desc>,
        boost::parameter::optional<tag::viz>> class_signature;

    template <typename Simu, typename robot>
    struct Refresh {
        Refresh(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            : _simu(simu), _robot(rob), _init_trans(init_trans) {}

        Simu& _simu;
        std::shared_ptr<robot> _robot;
        Eigen::Vector6d _init_trans;

        template <typename T>
        void operator()(T& x) const { x(_simu, _robot, _init_trans); }
    };

    template <class A1 = boost::parameter::void_, class A2 = boost::parameter::void_, class A3 = boost::parameter::void_, class A4 = boost::parameter::void_>
    class planarDARTSimu {
    public:
        using robot_t = std::shared_ptr<planar>;
        // defaults
        struct defaults {
            using planar_control_t = control;
            // using safety_measures_t = boost::fusion::vector<safety_measures::MaxHeight, safety_measures::BodyColliding, safety_measures::TurnOver>;
            using safety_measures_t = boost::fusion::vector<safety_measures::LinkColliding>;
            using descriptors_t = boost::fusion::vector<descriptors::PositionalCoord, descriptors::PolarCoord, descriptors::AngleSum, descriptors::Random>;
            using viz_t = boost::fusion::vector<visualizations::TargetArrow>;
        };

        // extract the types
        using args = typename class_signature::bind<A1, A2, A3, A4>::type;
        using planar_control_t = typename boost::parameter::binding<args, tag::planar_control, typename defaults::planar_control_t>::type;
        using SafetyMeasures = typename boost::parameter::binding<args, tag::safety, typename defaults::safety_measures_t>::type;
        using Descriptors = typename boost::parameter::binding<args, tag::desc, typename defaults::descriptors_t>::type;
        using Visualizations = typename boost::parameter::binding<args, tag::viz, typename defaults::viz_t>::type;
        using safety_measures_t = typename boost::mpl::if_<boost::fusion::traits::is_sequence<SafetyMeasures>, SafetyMeasures, boost::fusion::vector<SafetyMeasures>>::type;
        using descriptors_t = typename boost::mpl::if_<boost::fusion::traits::is_sequence<Descriptors>, Descriptors, boost::fusion::vector<Descriptors>>::type;
        using viz_t = typename boost::mpl::if_<boost::fusion::traits::is_sequence<Visualizations>, Visualizations, boost::fusion::vector<Visualizations>>::type;

        planarDARTSimu(const std::vector<double>& ctrl, robot_t robot, std::vector<planar_dart::planarDamage> damages = {}) : _euclidean_distance(10000),
                                                                          _world(std::make_shared<dart::simulation::World>()),
                                                                          _controller(ctrl, robot, damages),
                                                                          _old_index(0),
                                                                          _desc_period(2),
                                                                          _break(false)
        {
            _world->getConstraintSolver()->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
            _robot = robot;

            // set position of rhex
            _robot->skeleton()->setPosition(0, 0);
           

            _world->addSkeleton(_robot->skeleton());
            _world->setTimeStep(0.005);

            _controller.set_parameters(ctrl);

            _controller.update(_world->getTime());

            //_world->setTime(0.0);

#ifdef GRAPHIC
            _fixed_camera = false;
            _osg_world_node = new dart::gui::osg::WorldNode(_world);
            _osg_viewer.addWorldNode(_osg_world_node);
            _osg_viewer.setUpViewInWindow(0, 0, 640, 480);

            // full-screen
            // _osg_viewer.setUpViewOnSingleScreen();
#endif
        }

        ~planarDARTSimu() {}

        void run(double duration = 5.0, bool continuous = false, bool chain = false)
        {
            run()
        }

        void run()
        {
            _break = false;
            robot_t rob = this->robot();
            //double old_t = _world->getTime();
            Eigen::Vector3d init_pos = rob->pos();

            static Eigen::Vector6d init_trans = rob->pose();
#ifdef GRAPHIC
            if (!_osg_viewer.done())
#else
            if (true)
#endif
            {
                 _controller.update();

                _world->step(false);

                

                // update safety measures
                boost::fusion::for_each(_safety_measures, Refresh<planarDARTSimu, planar>(*this, rob, init_trans));
                // update visualizations
                boost::fusion::for_each(_visualizations, Refresh<planarDARTSimu, planar>(*this, rob, init_trans));

                if (_break) {
                    _euclidean_distance = 10002.0
                    //_covered_distance = -10002.0;
                    //_arrival_angle = -10002.0;
                    //_energy = -10002.0;
                    return;
                }

#ifdef GRAPHIC
                if (!_fixed_camera) {
                    auto COM = rob->skeleton()->getCOM();
                    _osg_viewer.getCameraManipulator()->setHomePosition(
                        osg::Vec3d(-1, -3, 2), osg::Vec3d(COM(0), COM(1), COM(2)), osg::Vec3d(0, 0, 1));
                    _osg_viewer.home();
                }
                // process next frame
                _osg_viewer.frame();
#endif

                boost::fusion::for_each(_descriptors, Refresh<planarDARTSimu, planar>(*this, rob, init_trans));
            }
            Eigen::Vector3d _posi = rob->pos();
            Eigen::Vector3d _bin(0.3875, 0.3875, 0.0);

            // updates values of covered distance average body height and arrival angle
            _euclidean_distance = sqrt(pow((_posi[i] - _bin[0]),2) + pow((_posi[1] - _bin[1]),2))
            //_covered_distance = fin_pos[0]-init_pos[0];
            //_body_avg_height = (chain)?_body_avg_height/(_world->getTime()-old_t):_world->getTime();
            //_arrival_angle = std::round(atan((fin_pos[1]-init_pos[1])/(_covered_distance)) * 100) / 100.0;
        }

        robot_t robot()
        {
            return _robot;
        }

        dart::simulation::WorldPtr world()
        {
            return _world;
        }

#ifdef GRAPHIC
        void fixed_camera(const Eigen::Vector3d& camera_pos, const Eigen::Vector3d& look_at = Eigen::Vector3d(0, 0, 0), const Eigen::Vector3d& up = Eigen::Vector3d(0, 0, 1))
        {
            _fixed_camera = true;
            _camera_pos = camera_pos;
            _look_at = look_at;
            _camera_up = up;

            // set camera position
            _osg_viewer.getCameraManipulator()->setHomePosition(
                osg::Vec3d(_camera_pos(0), _camera_pos(1), _camera_pos(2)), osg::Vec3d(_look_at(0), _look_at(1), _look_at(2)), osg::Vec3d(_camera_up(0), _camera_up(1), _camera_up(2)));
            _osg_viewer.home();
        }

        void follow_planar()
        {
            _fixed_camera = false;
        }
#endif

        template <typename Desc, typename T>
        void get_descriptor(T& result)
        {
            auto d = boost::fusion::find<Desc>(_descriptors);
            (*d).get(result);
        }

        double euclidean_distance() const
        {
            return _euclidean_distance;
        }

        double step() const
        {
            assert(_world != nullptr);
            return _world->getTimeStep();
        }

        void set_step(double step)
        {
            assert(_world != nullptr);
            _world->setTimeStep(step);
        }

        size_t desc_dump() const
        {
            return _desc_period;
        }

        void set_desc_dump(size_t desc_dump)
        {
            _desc_period = desc_dump;
        }

        void stop_sim(bool disable = true)
        {
            _break = disable;
        }

        planar_control_t& controller()
        {
            return _controller;
        }

        void clear_objects()
        {
            for (auto obj : _objects) {
                _world->removeSkeleton(obj);
            }
            _objects.clear();
        }

    protected:
        // Helper function to squash dart warnings from console output
        std::string _get_unique(std::string name) {
            while (_world->getSkeleton(name) != nullptr) {
                if (name[name.size() - 2] == '_') {
                    int i = name.back() - '0';
                    i++;
                    name.pop_back();
                    name = name + std::to_string(i);
                }
                else {
                    name = name + "_1";
                }
            }

            return name;
        }

        robot_t _robot;
        Eigen::Vector3d _final_pos;
        Eigen::Vector3d _final_rot;
        double _euclidean_distance;
        dart::simulation::WorldPtr _world;
        planar_control_t _controller;
        size_t _old_index;
        size_t _desc_period;
        bool _break;
        safety_measures_t _safety_measures;
        descriptors_t _descriptors;
        viz_t _visualizations;
        std::vector<dart::dynamics::SkeletonPtr> _objects;

#ifdef GRAPHIC
        bool _fixed_camera;
        Eigen::Vector3d _look_at;
        Eigen::Vector3d _camera_pos;
        Eigen::Vector3d _camera_up;
        osg::ref_ptr<dart::gui::osg::WorldNode> _osg_world_node;
        dart::gui::osg::Viewer _osg_viewer;
#endif
    };
}

#endif
