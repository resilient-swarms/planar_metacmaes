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
#include <planar_dart/planar.hpp>
#include <planar_dart/control.hpp>
#include <planar_dart/safety_measures.hpp>
#include <planar_dart/descriptors.hpp>
#include <planar_dart/visualizations.hpp>
#include <cmath>
#include <cstdlib>

#ifdef GRAPHIC
#include <dart/gui/osg/osg.hpp>
#endif

#define JOINT_SIZE 8

#include <random>

namespace planar_dart
{

    BOOST_PARAMETER_TEMPLATE_KEYWORD(planar_control)
    BOOST_PARAMETER_TEMPLATE_KEYWORD(safety)
    BOOST_PARAMETER_TEMPLATE_KEYWORD(desc)
    BOOST_PARAMETER_TEMPLATE_KEYWORD(viz)

    typedef boost::parameter::parameters<boost::parameter::optional<tag::planar_control>,
                                         boost::parameter::optional<tag::safety>,
                                         boost::parameter::optional<tag::desc>,
                                         boost::parameter::optional<tag::viz>>
        class_signature;

    template <typename Simu, typename robot>
    struct Refresh
    {
        Refresh(Simu &simu, std::shared_ptr<robot> rob, const Eigen::Vector6d &init_trans)
            : _simu(simu), _robot(rob), _init_trans(init_trans) {}

        Simu &_simu;
        std::shared_ptr<robot> _robot;
        Eigen::Vector6d _init_trans;

        template <typename T>
        void operator()(T &x) const { x(_simu, _robot, _init_trans); }
    };

    template <class A1 = boost::parameter::void_, class A2 = boost::parameter::void_, class A3 = boost::parameter::void_, class A4 = boost::parameter::void_>
    class planarDARTSimu
    {
    public:
        const int ISOMETRIC = 0, TOP = 1;
        const double MAXDIST = 0.5425;
        const double delta = MAXDIST / 15.0;
        using robot_t = std::shared_ptr<planar>;
        // defaults
        struct defaults
        {
            using planar_control_t = control;
            // using safety_measures_t = boost::fusion::vector<safety_measures::MaxHeight, safety_measures::BodyColliding, safety_measures::TurnOver>;
            using safety_measures_t = boost::fusion::vector<safety_measures::LinkColliding>;
            using descriptors_t = boost::fusion::vector<planar_dart::descriptors::PositionalCoord,
                                                        planar_dart::descriptors::PolarCoord,
                                                        planar_dart::descriptors::JointPairAngle,
                                                        planar_dart::descriptors::AngleSum>;
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

        planarDARTSimu(const std::vector<double> &ctrl, robot_t robot, std::vector<planar_dart::planarDamage> damages = {}) : _euclidean_distance(10000),
                                                                                                                              _world(std::make_shared<dart::simulation::World>()),
                                                                                                                              _controller(ctrl, robot, damages),
                                                                                                                              _old_index(0),
                                                                                                                              _desc_period(2),
                                                                                                                              _break(false),
                                                                                                                              _variance_angles(0.0)
        {
            _world->getConstraintSolver()->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
            _robot = robot;
            // set position of rhex
            _robot->skeleton()->setPosition(0, 0);

            _world->addSkeleton(_robot->skeleton());
            _world->setTimeStep(1);
            //_world->setTimeStep(0.05);

            add_wall();
            _controller.set_robot(robot);
            _controller.set_parameters(ctrl);

            _controller.update();

            //_world->setTime(0.0);

#ifdef GRAPHIC
            _fixed_camera = true;
            _osg_world_node = new dart::gui::osg::WorldNode(_world);
            _osg_viewer.addWorldNode(_osg_world_node);
            _osg_viewer.setUpViewInWindow(0, 0, 640, 480);
            // full-screen
            // _osg_viewer.setUpViewOnSingleScreen();
#endif


        }
        int find_bin(const std::vector<Eigen::Vector2d> &bin_locations) const
        {
            if (this->dead())
            {
                return -1;
            }
            double mindist = INFINITY;
            int match = -1;
            Eigen::Vector2d location = this->final_position();
            // first transform numbers to be positive (simplifies binning)
            //double x = location[0] + MAXDIST;
            //double y = location[1] + MAXDIST;
            //double x_bin = std::round(x / delta);
            //double y_bin = std::round(y / delta);

            //std::cout << "location " << location.transpose() << std::endl; //" : (" << x_bin << "," << y_bin << ")" << std::endl;
            for (int i = 0; i < bin_locations.size(); ++i)
            {
                // first transform numbers to be positive (simplifies binning)
                Eigen::Vector2d bin = bin_locations[i];
                //double xx = bin[0] + MAXDIST;
                //double yy = bin[1] + MAXDIST;
                //double xx_bin = std::round(xx / delta);
                //double yy_bin = std::round(yy / delta);
                double dist = (bin - location).norm();
                if (dist < mindist)
                {
                    mindist = dist;
                    match = i;
                    //return i;
                }
            }
            //std::cout << "matching location " << bin_locations[match].transpose() << std::endl;
            return match;
            //return -1;
        }
        ~planarDARTSimu() {}

        void run(double duration, bool continuous = false, bool chain = false)
        {
            run();
        }

        void run()
        {
            _break = false;
            robot_t rob = this->robot();
            static Eigen::Vector6d init_trans = rob->pose();
            double old_t = _world->getTime();
#ifdef GRAPHIC
            if (!_osg_viewer.done())
            //while ((_world->getTime() - old_t) < 5.0 && !_osg_viewer.done())
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

#ifdef GRAPHIC
                fixed_camera();
                _osg_viewer.frame();
                _osg_viewer.captureScreen("c.png");
#endif

                if (_break)
                {
                    _euclidean_distance = -1;
                    _variance_angles = -10002.0;
                    return;
                }

                boost::fusion::for_each(_descriptors, Refresh<planarDARTSimu, planar>(*this, rob, init_trans));
            }
            _final_pos = rob->gripper("final_pos")->getWorldPosition().head(2);
#ifdef TEST

#endif

            std::vector<double> normalised_angles = _controller.parameters();
            double _mean_angles = 0.0;
            for (size_t i = 0; i < JOINT_SIZE; ++i)
            {
                _mean_angles += normalised_angles[i];
            }
            _mean_angles /= JOINT_SIZE;
            _variance_angles = 0.0;
            for (size_t i = 0; i < JOINT_SIZE; ++i)
            {
                _variance_angles += pow((normalised_angles[i] - _mean_angles), 2);
            }
            _variance_angles /= JOINT_SIZE;
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
        void fixed_camera()
        {
            fixed_camera(view_setting);
        }
        void fixed_camera(int view)
        {
            view_setting = view;
            switch (0)
            {
            case 0:
                _camera_pos = Eigen::Vector3d(-1, -1, 1);
                _look_at = Eigen::Vector3d(0, 0, 0);
                _camera_up = Eigen::Vector3d(0, 0, 1);
                break;
            case 1:
                _camera_pos = Eigen::Vector3d(-1, -1, 1);
                _look_at = Eigen::Vector3d(0, -0.25, 0);
                _camera_up = Eigen::Vector3d(0, 1, 0);
                break;
            }

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
        void get_descriptor(T &result)
        {
            auto d = boost::fusion::find<Desc>(_descriptors);
            (*d).get(result);
        }
        Eigen::VectorXd final_position() const
        {
            return _final_pos; // either empty or 2d, use for pairwisedist
        }
        double euclidean_distance() const
        {
            return _euclidean_distance;
        }
        double performance_val() const
        {
            return -_variance_angles; //lower variance is more efficient
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

        bool dead() const
        {
            return _break;
        }

        planar_control_t &controller()
        {
            return _controller;
        }

        void clear_objects()
        {
            for (auto obj : _objects)
            {
                _world->removeSkeleton(obj);
            }
            _objects.clear();
        }

    protected:
        // Helper function to squash dart warnings from console output
        std::string _get_unique(std::string name)
        {
            while (_world->getSkeleton(name) != nullptr)
            {
                if (name[name.size() - 2] == '_')
                {
                    int i = name.back() - '0';
                    i++;
                    name.pop_back();
                    name = name + std::to_string(i);
                }
                else
                {
                    name = name + "_1";
                }
            }

            return name;
        }

        void add_wall()
        {
            if (_world->getSkeleton("wall") != nullptr)
                return;
            dart::dynamics::SkeletonPtr slope = dart::dynamics::Skeleton::create("wall");

            dart::dynamics::BodyNodePtr sbody = slope->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
            //sbody->setFrictionCoeff(friction);

            // Give the body a shape
            double slope_width = 0.003;
            double slope_height = 0.1;

            auto box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(slope_width * 500, slope_width, slope_height));

            auto box_node = sbody->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);

            box_node->getVisualAspect()->setColor(dart::Color::Gray());

            // Put the body into position
            Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
            tf.translation() = Eigen::Vector3d(0, 0.08, 0);

            sbody->getParentJoint()->setTransformFromParentBodyNode(tf);

            _world->addSkeleton(slope);
        }

        robot_t _robot;
        Eigen::VectorXd _final_pos;
        Eigen::Vector3d _final_rot;
        double _euclidean_distance;
        double _variance_angles;
        dart::simulation::WorldPtr _world;
        planar_control_t _controller;
        size_t _old_index;
        size_t _desc_period;
        bool _break;
        safety_measures_t _safety_measures;
        descriptors_t _descriptors;
        viz_t _visualizations;
        std::vector<dart::dynamics::SkeletonPtr> _objects;
        std::random_device rd;

#ifdef GRAPHIC
        bool _fixed_camera;
        Eigen::Vector3d _look_at;
        Eigen::Vector3d _camera_pos;
        Eigen::Vector3d _camera_up;
        osg::ref_ptr<dart::gui::osg::WorldNode> _osg_world_node;
        dart::gui::osg::Viewer _osg_viewer;
        int view_setting;
#endif
    };
} // namespace planar_dart

#endif
