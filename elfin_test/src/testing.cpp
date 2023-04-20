#include "elfin_test/testing.h"

using namespace elfin5_ikfast_simple_api;

enum { NS_PER_SECOND = 1000000000 };

void sub_timespec(struct timespec t1, struct timespec t2, struct timespec *td)
{
    td->tv_nsec = t2.tv_nsec - t1.tv_nsec;
    td->tv_sec  = t2.tv_sec - t1.tv_sec;
    if (td->tv_sec > 0 && td->tv_nsec < 0)
    {
        td->tv_nsec += NS_PER_SECOND;
        td->tv_sec--;
    }
    else if (td->tv_sec < 0 && td->tv_nsec > 0)
    {
        td->tv_nsec -= NS_PER_SECOND;
        td->tv_sec++;
    }
}

void IKFast_test(bool verbose=false){
    Elfin5KinematicSolver solver;
    
    Eigen::Vector3d trans(0.0, 0.0, 1.0);
    Eigen::Matrix3d orient;
    orient<< 1, 0, 0,
             0, 1, 0,
             0, 0, 1;
    std::vector<double> vfree;
    std::vector<double> seed_state;
    seed_state.resize(6);
    std::vector<double> solution;
    solution.resize(6);

    bool found = solver.getPositionIK(trans, orient, vfree, seed_state, solution);

    if(verbose){
        std::cout<<"Found: "<<found<<std::endl<<std::endl;

        std::cout<<"IK Solution"<<std::endl;
        for (auto i = solution.begin(); i != solution.end(); ++i)
            std::cout << *i << " ";
        std::cout<<std::endl<<std::endl;

        std::cout<<"FK Solution"<<std::endl;
        Eigen::Vector3d trans_new;
        Eigen::Matrix3d orient_new;
        solver.getPositionFK(solution, trans_new, orient_new);
        std::cout<<trans_new<<std::endl;
        std::cout<<orient_new<<std::endl;
    }
}

void timespec_test(){
    struct timespec start, stop;
    clock_gettime(CLOCK_REALTIME, &start);
    sleep(1);
    clock_gettime(CLOCK_REALTIME, &stop);
    std::cout<<"Start  sec: "<<start.tv_sec<<std::endl;
    std::cout<<"Start nsec: "<<start.tv_nsec<<std::endl<<std::endl;

    std::cout<<"Stop  sec: "<<stop.tv_sec<<std::endl;
    std::cout<<"Stop nsec: "<<stop.tv_nsec<<std::endl<<std::endl;
    std::cout<<" Delta (sec): "<<((double) (stop.tv_sec - start.tv_sec))<<std::endl;
    std::cout<<"Delta (nsec): "<<((double) (stop.tv_nsec - start.tv_nsec))/((double) 100000)<<std::endl;
    std::cout<<"       Delta: "<<(stop.tv_sec + double(stop.tv_nsec)/1e+9) -  (start.tv_sec + double(start.tv_nsec)/1e+9)<<std::endl;
}

void IKFast_benchmark_test(int iter){
    struct timespec start, stop;
    float min=100, max=0, average=0, cum=0, delta;
    int found_cum = 0;

    // Testing Loop
    for(size_t i=0; i<iter; i++){
        // PROCESS
        Elfin5KinematicSolver solver;
    
        Eigen::Vector3d trans(rand(), rand(), rand());
        trans = trans/RAND_MAX/2;
        
        Eigen::AngleAxisd rollAngle(rand()/(float)RAND_MAX, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle(rand()/(float)RAND_MAX, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(rand()/(float)RAND_MAX, Eigen::Vector3d::UnitX());

        Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

        Eigen::Matrix3d orient = q.matrix();

        std::vector<double> vfree;
        std::vector<double> seed_state;
        seed_state.resize(6);
        std::vector<double> solution;
        solution.resize(6);

        // Start timer
        clock_gettime(CLOCK_REALTIME, &start);

        bool found = solver.getPositionIK(trans, orient, vfree, seed_state, solution);

        // Stop timer
        clock_gettime(CLOCK_REALTIME, &stop);

        if (found)
            found_cum++;
        else{
            // std::cout<<"---------------------------------------------------------------------------"<<std::endl;
            // std::cout<<"Solution not found for iter "<<i+1<<std::endl;
            // std::cout<<"Position:\n"<<trans<<std::endl;
            // std::cout<<"Orientation:\n"<<orient<<std::endl;
        }


        Eigen::Matrix<double, 1, 6> solution_eigen(solution.data());
        delta = (stop.tv_sec + double(stop.tv_nsec)/1e+9) -  (start.tv_sec + double(start.tv_nsec)/1e+9);
        //std::cout<<"---------------------------------------------------------------------------"<<std::endl;
        //std::cout<<"Iteration "<<i+1<<std::endl;
        //std::cout<<"Found: "<<found<<std::endl;
        //std::cout<<"Solution: "<<solution_eigen<<std::endl;
        //std::cout<<"Delta: "<<delta<<std::endl;
        //std::cout<< trans << std::endl;

        // Calculate min, max, average
        if(delta < min)
            min = delta;
        if (delta > max)
            max = delta;
        cum += delta;
    }
    average = cum / iter;
    std::cout<<"==========================================================================="<<std::endl;    
    std::cout<<"Minimum:"<<min*1e6<<std::endl;
    std::cout<<"Maximum:"<<max*1e6<<std::endl;
    std::cout<<"Average:"<<average*1e6<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Solutions found: "<<found_cum<<std::endl;
    std::cout<<"Out of: "<<iter<<std::endl;
}

float rand_num(){
    return 2*(rand()/(float)RAND_MAX)-1;
}

void IKFast_FK_benchmark_test(int iter){
    struct timespec start, stop;
    float min=100, max=0, average=0, cum=0, delta;
    int found_cum = 0;

    // Testing Loop
    for(size_t i=0; i<iter; i++){
        // Initializing stuff
        Elfin5KinematicSolver solver;
        std::vector<double> joint_angles;
        Eigen::Vector3d trans;
        Eigen::Matrix3d orient;
        joint_angles = {rand_num()*3.14, rand_num()*2.35, rand_num()*2.61, rand_num()*3.14, rand_num()*2.56, rand_num()*3.14};

        // Start timer
        clock_gettime(CLOCK_REALTIME, &start);

        // PROCESS
        bool found = solver.getPositionFK(joint_angles, trans, orient);

        // Stop timer
        clock_gettime(CLOCK_REALTIME, &stop);

        if (found)
            found_cum++;
        else{
            std::cout<<"---------------------------------------------------------------------------"<<std::endl;
            std::cout<<"Solution not found for iter "<<i+1<<std::endl;
            std::cout<<"Position:\n"<<trans<<std::endl;
            std::cout<<"Orientation:\n"<<orient<<std::endl;
        }

        delta = (stop.tv_sec + double(stop.tv_nsec)/1e+9) -  (start.tv_sec + double(start.tv_nsec)/1e+9);
        // std::cout<<"---------------------------------------------------------------------------"<<std::endl;
        // std::cout<<"Iteration "<<i+1<<std::endl;
        // std::cout<<"Found: "<<found<<std::endl;
        // std::cout<<"Delta: "<<delta<<std::endl;
        // std::cout<< trans << std::endl;
        // std::cout<< orient << std::endl;

        // Calculate min, max, average
        if(delta < min)
            min = delta;
        if (delta > max)
            max = delta;
        cum += delta;
    }
    average = cum / iter;
    std::cout<<"==========================================================================="<<std::endl;    
    std::cout<<"Minimum:"<<min*1e6<<std::endl;
    std::cout<<"Maximum:"<<max*1e6<<std::endl;
    std::cout<<"Average:"<<average*1e6<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Solutions found: "<<found_cum<<std::endl;
    std::cout<<"Out of: "<<iter<<std::endl;
}

void TracIK_benchmark_test(int iter){
    struct timespec start, stop;
    float min=100, max=0, average=0, cum=0, delta;
    int found_cum = 0;

    TRAC_IK::TRAC_IK* tracik_solver;
    KDL::JntArray nominal;
    KDL::Vector trans;
    KDL::Rotation orient;
    KDL::Frame end_effector_pose;
    KDL::JntArray result;

    std::string chain_start = "elfin_base";
    std::string chain_end = "elfin_end_link";
    std::string urdf_param = "/robot_description";
    double timeout = 0.005;
    double eps = 1e-5;
    tracik_solver = new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, timeout, eps);
    nominal = KDL::JntArray(6);

    for (size_t i=0; i<iter; i++){
        trans = KDL::Vector(rand(), rand(), rand());
        trans = trans/RAND_MAX/2;

        float ox = rand()/(float)RAND_MAX, oy = rand()/(float)RAND_MAX, oz = rand()/(float)RAND_MAX, ow = rand()/(float)RAND_MAX;
        float norm = sqrt(ox*ox + oy*oy + oz*oz + ow*ow);
        ox /= norm;
        oy /= norm;
        oz /= norm;
        ow /= norm;
        orient = KDL::Rotation::Quaternion(ox, oy, oz, ow);

        // Obtain IK solution
        end_effector_pose = KDL::Frame(orient, trans);

        // Start timer
        clock_gettime(CLOCK_REALTIME, &start);

        int result_count = tracik_solver->CartToJnt(nominal, end_effector_pose, result);

        // Stop timer
        clock_gettime(CLOCK_REALTIME, &stop);
        delta = (stop.tv_sec + double(stop.tv_nsec)/1e+9) -  (start.tv_sec + double(start.tv_nsec)/1e+9);

        if (result_count > 0)
            found_cum++;
        else{
            // std::cout<<"---------------------------------------------------------------------------"<<std::endl;
            // std::cout<<"Iteration "<<i<<std::endl;
            // std::cout<<"Translation: "<<std::endl;
            // std::cout<<trans(0)<<std::endl;
            // std::cout<<trans(1)<<std::endl;
            // std::cout<<trans(2)<<std::endl;
            // std::cout<<std::endl;
            // std::cout<<"Orientation: "<<std::endl;
            // for (size_t i=0; i<3; i++) {
            //     for (size_t j=0; j<3; j++) {
            //         std::cout<< orient(i,j) <<"\t\t";
            //     }
            //     std::cout<<std::endl;
            // }
            // std::cout<<std::endl;
        }

        // Calculate min, max, average
        if(delta < min)
            min = delta;
        if (delta > max)
            max = delta;
        cum += delta;
    }
    average = cum / iter;
    std::cout<<"==========================================================================="<<std::endl;    
    std::cout<<"Minimum:"<<min*1e6<<std::endl;
    std::cout<<"Maximum:"<<max*1e6<<std::endl;
    std::cout<<"Average:"<<average*1e6<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Solutions found: "<<found_cum<<std::endl;
    std::cout<<"Out of: "<<iter<<std::endl;
}

void TracIK_FK_benchmark_test(int iter){
    struct timespec start, stop;
    float min=100, max=0, average=0, cum=0, delta;
    int found_cum = 0;

    std::string chain_start = "elfin_base_link";
    std::string chain_end = "elfin_end_link";
    std::string urdf_param = "/robot_description";
    double timeout = 0.005;
    double eps = 1e-5;
    TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

    KDL::Chain chain;
    tracik_solver.getKDLChain(chain);
    KDL::ChainFkSolverPos_recursive fk_solver(chain);

    KDL::JntArray joints(6);
    KDL::Frame pose;

    // Testing Loop
    for(size_t i=0; i<iter; i++){
        // Initializing stuff
        std::vector<double> joint_angles = {rand_num()*3.14, rand_num()*2.35, rand_num()*2.61, rand_num()*3.14, rand_num()*2.56, rand_num()*3.14};

        for (size_t i=0; i<6; i++)
            joints(i) = joint_angles[i];

        // Start timer
        clock_gettime(CLOCK_REALTIME, &start);

        // PROCESS
        fk_solver.JntToCart(joints, pose);
        //bool found = solver.getPositionFK(joint_angles, trans, orient);

        // Stop timer
        clock_gettime(CLOCK_REALTIME, &stop);

        bool found = true;
        if (found)
            found_cum++;
        else{
            // std::cout<<"---------------------------------------------------------------------------"<<std::endl;
            // std::cout<<"Solution not found for iter "<<i+1<<std::endl;
            // std::cout<<"Position:\n"<<trans<<std::endl;
            // std::cout<<"Orientation:\n"<<orient<<std::endl;
        }

        delta = (stop.tv_sec + double(stop.tv_nsec)/1e+9) -  (start.tv_sec + double(start.tv_nsec)/1e+9);
        // std::cout<<"---------------------------------------------------------------------------"<<std::endl;
        // std::cout<<"Iteration "<<i+1<<std::endl;
        // std::cout<<"Found: "<<found<<std::endl;
        // std::cout<<"Delta: "<<delta<<std::endl;
        // std::cout<< trans << std::endl;
        // std::cout<< orient << std::endl;

        // Calculate min, max, average
        if(delta < min)
            min = delta;
        if (delta > max)
            max = delta;
        cum += delta;
    }
    average = cum / iter;
    std::cout<<"==========================================================================="<<std::endl;    
    std::cout<<"Minimum:"<<min*1e6<<std::endl;
    std::cout<<"Maximum:"<<max*1e6<<std::endl;
    std::cout<<"Average:"<<average*1e6<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Solutions found: "<<found_cum<<std::endl;
    std::cout<<"Out of: "<<iter<<std::endl;
}

void forward_kinematics(Elfin5KinematicSolver* solver, const std::vector<double> joint_angles, Eigen::VectorXd& cartesian){
    Eigen::Vector3d trans;
    Eigen::Matrix3d orient;
    solver->getPositionFK(joint_angles, trans, orient);
    Eigen::Quaterniond quat(orient);
    cartesian.resize(7);
    cartesian<<trans(0), trans(1), trans(2), quat.w(), quat.x(), quat.y(), quat.z();
}

void analytical_jacobian(Elfin5KinematicSolver* solver, std::vector<double> joint_angles, double delta, Eigen::MatrixXd& jacobian){
    Eigen::VectorXd cartesian;
    Eigen::VectorXd initial_cartesian(7);
    forward_kinematics(solver, joint_angles, initial_cartesian);
    for(size_t i=0; i<6; i++){
        // Add a delta to the joint
        joint_angles[i] += delta;

        // Calculate cartesian
        forward_kinematics(solver, joint_angles, cartesian);

        // Assign it in jacobian
        for(size_t j=0; j<7; j++){
            jacobian(j, i) = cartesian(j)-initial_cartesian(j);
        }

        // Go back
        joint_angles[i] -= delta;
    }
    jacobian /= delta;
}

void geometric_to_analytical_matrix(const Eigen::Quaterniond q, Eigen::MatrixXd& E){
    E.resize(7, 6);
    E<<1, 0, 0,        0,        0,        0,
       0, 1, 0,        0,        0,        0,
       0, 0, 1,        0,        0,        0,
       0, 0, 0, -q.x()/2, -q.y()/2, -q.z()/2,
       0, 0, 0,  q.w()/2,  q.z()/2, -q.y()/2,
       0, 0, 0, -q.z()/2,  q.w()/2,  q.x()/2,
       0, 0, 0,  q.y()/2, -q.x()/2,  q.w()/2;
}

void JacobianTesting(){
    struct timespec start, stop;
    double delta;

    // Initialize solvers
    urdf::Model urdf;
    urdf.initParam("robot_description");
    
    KDL::Tree kdl_tree;
    kdl_parser::treeFromUrdfModel(urdf, kdl_tree);
    
    KDL::Chain kdl_chain;
    kdl_tree.getChain("elfin_base_link", "elfin_end_link", kdl_chain);

    Elfin5KinematicSolver* elfin_solver = new Elfin5KinematicSolver();
    KDL::ChainJntToJacSolver* jnt_to_jac_solver = new KDL::ChainJntToJacSolver(kdl_chain);

    // Initialize Variables for Elfin Solver
    std::vector<double> joint_angles;
    Eigen::Vector3d trans;
    Eigen::Matrix3d orient;
    Eigen::VectorXd cartesian;
    Eigen::MatrixXd jacobian(7, 6);
    Eigen::MatrixXd jacobian_converted(7, 6);

    // Initialize Variables for Geometric Jacobian
    KDL::JntArray q(6);
    KDL::Jacobian J(6);
    Eigen::MatrixXd E;

    float ja_min=100, ja_max=0, ja_average=0, ja_cum=0;
    float jg_min=100, jg_max=0, jg_average=0, jg_cum=0;

    unsigned int iterations = 100;
    for (size_t iter=0; iter<iterations; iter++){
        // Populate angles
        joint_angles = {rand_num()*3.14, rand_num()*2.35, rand_num()*2.61, rand_num()*3.14, rand_num()*2.56, rand_num()*3.14};
        //joint_angles = {0, 0, 0, 0, 0, 0};
        for (size_t i=0; i<6; i++)
            q(i) = joint_angles[i];

        // Calculate Analytical Jacobian
        //forward_kinematics(elfin_solver, joint_angles, cartesian);
        clock_gettime(CLOCK_REALTIME, &start);
        analytical_jacobian(elfin_solver, joint_angles, 0.000001, jacobian);
        clock_gettime(CLOCK_REALTIME, &stop);
        delta = (stop.tv_sec + double(stop.tv_nsec)/1e+9) -  (start.tv_sec + double(start.tv_nsec)/1e+9);
        //std::cout<<"Time for analytical: "<<delta<<std::endl;
        // Calculate min, max, average
        if(delta < ja_min)
            ja_min = delta;
        if (delta > ja_max)
            ja_max = delta;
        ja_cum += delta;

        // Calculate Geometric Jacobian
        clock_gettime(CLOCK_REALTIME, &start);
        jnt_to_jac_solver->JntToJac(q, J);

        // Calculate Forward Kinematics
        elfin_solver->getPositionFK(joint_angles, trans, orient);
        Eigen::Quaterniond quat(orient);
        
        // Convert using E matrix
        geometric_to_analytical_matrix(quat, E);
        jacobian_converted = E*J.data;
        clock_gettime(CLOCK_REALTIME, &stop);
        delta = (stop.tv_sec + double(stop.tv_nsec)/1e+9) -  (start.tv_sec + double(start.tv_nsec)/1e+9);
        //std::cout<<"Time for converted: "<<delta<<std::endl;
        if(delta < jg_min)
            jg_min = delta;
        if (delta > jg_max)
            jg_max = delta;
        jg_cum += delta;
    }
    jg_average = jg_cum / (float)iterations;
    std::cout<<"===================================Geometric========================================"<<std::endl;    
    std::cout<<"Minimum:"<<jg_min*1e6<<std::endl;
    std::cout<<"Maximum:"<<jg_max*1e6<<std::endl;
    std::cout<<"Average:"<<jg_average*1e6<<std::endl;
    std::cout<<std::endl;

    ja_average = ja_cum / (float)iterations;
    std::cout<<"===================================Analytical========================================"<<std::endl;    
    std::cout<<"Minimum:"<<ja_min*1e6<<std::endl;
    std::cout<<"Maximum:"<<ja_max*1e6<<std::endl;
    std::cout<<"Average:"<<ja_average*1e6<<std::endl;
    std::cout<<std::endl;

    // Results
    // std::cout<<"Geometric"<<std::endl;
    // std::cout<<J.data<<std::endl;
    // std::cout<<std::endl<<std::endl;
    
    // std::cout<<"Analytical"<<std::endl;
    // std::cout<< jacobian<<std::endl;
    // std::cout<<std::endl<<std::endl;

    // std::cout<<"Converted"<<std::endl;
    // std::cout<< jacobian_converted;
    // std::cout<<std::endl<<std::endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "testing");
    //Eigen::Quaterniond q(0.5, 0.5, 0, 0);
    //Eigen::Vector3d v(1, 2, 3);

    //std::cout<< q.normalized().toRotationMatrix() << std::endl;
    //std::cout<< v << std::endl;

    //IKFast_benchmark_test(1000000);
    //TracIK_benchmark_test(1000000);
    //IKFast_test(true);
    
    //TracIK_benchmark_test(10);
    //IKFast_FK_benchmark_test(1);
    //TracIK_FK_benchmark_test(1);
    
    // vector to Eigen
    JacobianTesting();

    return 0;
}